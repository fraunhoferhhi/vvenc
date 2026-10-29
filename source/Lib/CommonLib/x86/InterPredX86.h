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
/** \file     InterPredX86.h
    \brief    SIMD for InterPrediction
*/

//! \ingroup CommonLib
//! \{


//#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "Rom.h"
#include "InterPrediction.h"
#include "InterpolationFilter4x4X86.h"

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT_BDOF

//! \ingroup CommonLib
//! \{

namespace vvenc {



static inline int rightShiftMSB(int numer, int denom)
{
  int shiftIdx = bit_scan_reverse(denom);
  return (numer >> shiftIdx);
}
  
template<X86_VEXT vext>
static ALWAYS_INLINE void addBIOAvg4_SSE(const int16_t* src0, const int16_t* src1, int16_t* dst, ptrdiff_t dstStride, const int16_t* gradX0, const int16_t* gradX1, const int16_t* gradY0, const int16_t* gradY1, ptrdiff_t widthG, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
  const ptrdiff_t src0Stride = widthG + 2;
  const ptrdiff_t src1Stride = widthG + 2;
  const ptrdiff_t gradStride = widthG;

  __m128i mm_tmpx    = _mm_set1_epi32( ( tmpx & 0xffff ) | ( tmpy << 16 ) );
  __m128i mm_offset  = _mm_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min() );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max() );
  __m128i mm_a;
  __m128i mm_b;
  __m128i mm_sum;

  for( int y = 0; y < 4; y++, dst += dstStride, src0 += src0Stride, src1 += src1Stride, gradX0 += gradStride, gradX1 += gradStride, gradY0 += gradStride, gradY1 += gradStride )
  {
    mm_a   = _mm_unpacklo_epi16 ( _vv_loadl_epi64( (const __m128i *) gradX0 ), _vv_loadl_epi64( (const __m128i *) gradY0 ) );
    mm_b   = _mm_unpacklo_epi16 ( _vv_loadl_epi64( (const __m128i *) gradX1 ), _vv_loadl_epi64( (const __m128i *) gradY1 ) );
    mm_a   = _mm_sub_epi16      ( mm_a, mm_b );
    mm_sum = _mm_madd_epi16     ( mm_a, mm_tmpx );
    mm_a   = _mm_cvtepi16_epi32 ( _vv_loadl_epi64( (const __m128i *) ( src0 ) ) );
    mm_b   = _mm_cvtepi16_epi32 ( _vv_loadl_epi64( (const __m128i *) ( src1 ) ) );
    mm_sum = _mm_add_epi32      ( _mm_add_epi32( mm_sum, mm_a ), _mm_add_epi32( mm_b, mm_offset ) );
    mm_sum = _mm_packs_epi32    ( _mm_srai_epi32( mm_sum, shift ), mm_a );
    mm_sum = _mm_min_epi16      ( vibdimax, _mm_max_epi16( vibdimin, mm_sum ) );
    _vv_storel_epi64            ( (__m128i *) dst, mm_sum );
  }
}

#if USE_AVX2
static ALWAYS_INLINE void addBIOAvg4_2x_AVX2(const int16_t* src0, const int16_t* src1, int16_t* dst, ptrdiff_t dstStride, const int16_t* gradX0, const int16_t* gradX1, const int16_t* gradY0, const int16_t* gradY1, ptrdiff_t widthG, int tmpx0, int tmpx1, int tmpy0, int tmpy1, int shift, int offset, const ClpRng& clpRng)
{
  const ptrdiff_t src0Stride = widthG + 2;
  const ptrdiff_t src1Stride = widthG + 2;
  const ptrdiff_t gradStride = widthG;

  __m256i mm_tmpx    = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_set1_epi32( ( tmpx0 & 0xffff ) | ( tmpy0 * ( 1 << 16 )) ) ), _mm_set1_epi32( ( tmpx1 & 0xffff ) | ( tmpy1 * ( 1 << 16 )) ), 1 );
  __m256i mm_offset  = _mm256_set1_epi32( offset );
  __m256i vibdimin   = _mm256_set1_epi32( clpRng.min() );
  __m256i vibdimax   = _mm256_set1_epi32( clpRng.max() );
  __m256i mm_a;
  __m256i mm_b;
  __m256i mm_sum;
  __m128i xsrc0, xsrc1;

  for( int y = 0; y < 4; y++, dst += dstStride, src0 += src0Stride, src1 += src1Stride, gradX0 += gradStride, gradX1 += gradStride, gradY0 += gradStride, gradY1 += gradStride )
  {
    xsrc0  = _mm_loadu_si128       ( ( const __m128i * ) gradX0 );
    xsrc1  = _mm_loadu_si128       ( ( const __m128i * ) gradY0 );
    mm_a   = _mm256_castsi128_si256( _mm_unpacklo_epi16( xsrc0, xsrc1 ) );
    mm_a   = _mm256_inserti128_si256( mm_a, _mm_unpackhi_epi16( xsrc0, xsrc1 ), 1 );
    xsrc0  = _mm_loadu_si128       ( ( const __m128i * ) gradX1 );
    xsrc1  = _mm_loadu_si128       ( ( const __m128i * ) gradY1 );
    mm_b   = _mm256_castsi128_si256( _mm_unpacklo_epi16( xsrc0, xsrc1 ) );
    mm_b   = _mm256_inserti128_si256( mm_b, _mm_unpackhi_epi16( xsrc0, xsrc1 ), 1 );
    mm_a   = _mm256_sub_epi16      ( mm_a, mm_b );
    mm_sum = _mm256_madd_epi16     ( mm_a, mm_tmpx );
    mm_a   = _mm256_cvtepi16_epi32 ( _mm_loadu_si128( (const __m128i *) ( src0 ) ) );
    mm_b   = _mm256_cvtepi16_epi32 ( _mm_loadu_si128( (const __m128i *) ( src1 ) ) );
    mm_sum = _mm256_add_epi32      ( _mm256_add_epi32( mm_sum, mm_a ), _mm256_add_epi32( mm_b, mm_offset ) );
    mm_sum = _mm256_srai_epi32     ( mm_sum, shift );
    mm_sum = _mm256_min_epi32      ( vibdimax, _mm256_max_epi32( vibdimin, mm_sum ) );
    _mm_storeu_si128               ( (__m128i *) dst, _mm256_cvtepi32_epi16x( mm_sum ) );
  }
}
#endif

template< X86_VEXT vext >
static inline void calcBIOSums_SSE(const Pel* srcY0Tmp, const Pel* srcY1Tmp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, const int widthG, const int bitDepth, int limit, int &tmpx, int &tmpy)
{
  static constexpr int shift4 = 4;
  static constexpr int shift5 = 1;
  const int srcStride = widthG + 2;

  __m128i sumAbsGXTmp    = _mm_setzero_si128();
  __m128i sumDIXTmp      = _mm_setzero_si128();
  __m128i sumAbsGYTmp    = _mm_setzero_si128();
  __m128i sumDIYTmp      = _mm_setzero_si128();
  __m128i sumSignGyGxTmp = _mm_setzero_si128();

  for (int y = 0; y < 6; y++)
  {
    __m128i shiftSrcY0Tmp = _mm_srai_epi16(_mm_loadu_si128((__m128i*)(srcY0Tmp)), shift4);
    __m128i shiftSrcY1Tmp = _mm_srai_epi16(_mm_loadu_si128((__m128i*)(srcY1Tmp)), shift4);
    __m128i loadGradX0    = _mm_loadu_si128((__m128i*)(gradX0));
    __m128i loadGradX1    = _mm_loadu_si128((__m128i*)(gradX1));
    __m128i loadGradY0    = _mm_loadu_si128((__m128i*)(gradY0));
    __m128i loadGradY1    = _mm_loadu_si128((__m128i*)(gradY1));
    __m128i subTemp1      = _mm_sub_epi16(shiftSrcY1Tmp, shiftSrcY0Tmp);
    __m128i packTempX     = _mm_srai_epi16(_mm_add_epi16(loadGradX0, loadGradX1), shift5);
    __m128i packTempY     = _mm_srai_epi16(_mm_add_epi16(loadGradY0, loadGradY1), shift5);
    __m128i gX            = _mm_abs_epi16(packTempX);
    __m128i gY            = _mm_abs_epi16(packTempY);
    __m128i dIX           = _mm_sign_epi16(subTemp1,  packTempX);
    __m128i dIY           = _mm_sign_epi16(subTemp1,  packTempY);
    __m128i signGY_GX     = _mm_sign_epi16(packTempX, packTempY);

    sumAbsGXTmp     = _mm_add_epi16(sumAbsGXTmp, gX);
    sumDIXTmp       = _mm_add_epi16(sumDIXTmp, dIX);
    sumAbsGYTmp     = _mm_add_epi16(sumAbsGYTmp, gY);
    sumDIYTmp       = _mm_add_epi16(sumDIYTmp, dIY);
    sumSignGyGxTmp  = _mm_add_epi16(sumSignGyGxTmp, signGY_GX);

    srcY0Tmp += srcStride;
    srcY1Tmp += srcStride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
  }

  sumAbsGXTmp    = _mm_madd_epi16(sumAbsGXTmp,    _mm_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0));
  sumDIXTmp      = _mm_madd_epi16(sumDIXTmp,      _mm_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0));
  sumAbsGYTmp    = _mm_madd_epi16(sumAbsGYTmp,    _mm_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0));
  sumDIYTmp      = _mm_madd_epi16(sumDIYTmp,      _mm_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0));
  sumSignGyGxTmp = _mm_madd_epi16(sumSignGyGxTmp, _mm_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0));

  __m128i a12 = _mm_unpacklo_epi32(sumAbsGXTmp, sumAbsGYTmp);
  __m128i a3  = _mm_unpackhi_epi32(sumAbsGXTmp, sumAbsGYTmp);
  __m128i b12 = _mm_unpacklo_epi32(sumDIXTmp, sumDIYTmp);
  __m128i b3  = _mm_unpackhi_epi32(sumDIXTmp, sumDIYTmp);
  __m128i c1  = _mm_unpacklo_epi64(a12, b12);
  __m128i c2  = _mm_unpackhi_epi64(a12, b12);
  __m128i c3  = _mm_unpacklo_epi64(a3, b3);

  c1 = _mm_add_epi32(c1, c2);
  c1 = _mm_add_epi32(c1, c3);

  int sumAbsGX = _mm_cvtsi128_si32(c1);
  int sumAbsGY = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0x55));
  int sumDIX   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xaa));
  int sumDIY   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xff));

  sumSignGyGxTmp = _mm_add_epi32(sumSignGyGxTmp, _mm_shuffle_epi32(sumSignGyGxTmp, 0x4e));   // 01001110
  sumSignGyGxTmp = _mm_add_epi32(sumSignGyGxTmp, _mm_shuffle_epi32(sumSignGyGxTmp, 0xb1));   // 10110001
  int sumSignGY_GX  = _mm_cvtsi128_si32(sumSignGyGxTmp);

  tmpx = sumAbsGX == 0 ? 0 : rightShiftMSB( sumDIX << 2, sumAbsGX );
  tmpx = Clip3( -limit, limit, tmpx );

  int mainsGxGy = sumSignGY_GX >> 12;
  int secsGxGy  = sumSignGY_GX & ( ( 1 << 12 ) - 1 );
  int tmpData   = tmpx * mainsGxGy;
  tmpData       = ( ( tmpData << 12 ) + tmpx * secsGxGy ) >> 1;
  tmpy = sumAbsGY == 0 ? 0 : rightShiftMSB( ( ( sumDIY << 2 ) - tmpData ), sumAbsGY );
  tmpy = Clip3( -limit, limit, tmpy );
}

#if USE_AVX2
static inline void calcBIOSums2x_AVX2(const Pel* srcY0Tmp, const Pel* srcY1Tmp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, const int widthG, const int bitDepth, int limit, int &tmpx0, int &tmpx1, int &tmpy0, int &tmpy1 )
{
  static constexpr int shift4 = 4;
  static constexpr int shift5 = 1;
  const int srcStride = widthG + 2;
  
  __m256i sumAbsGXTmp     = _mm256_setzero_si256();
  __m256i sumDIXTmp       = _mm256_setzero_si256();
  __m256i sumAbsGYTmp     = _mm256_setzero_si256();
  __m256i sumDIYTmp       = _mm256_setzero_si256();
  __m256i sumSignGyGxTmp  = _mm256_setzero_si256();

#define _mm256_load2_si128_offset4(addr) _mm256_inserti128_si256( _mm256_castsi128_si256(_mm_loadu_si128((const __m128i*) &addr[0])), _mm_loadu_si128((const __m128i*) &addr[4]), 1 )

  for (int y = 0; y < 6; y++)
  {
    __m256i shiftSrcY0Tmp = _mm256_srai_epi16(_mm256_load2_si128_offset4(srcY0Tmp), shift4);
    __m256i shiftSrcY1Tmp = _mm256_srai_epi16(_mm256_load2_si128_offset4(srcY1Tmp), shift4);
    __m256i loadGradX0    = _mm256_load2_si128_offset4(gradX0);
    __m256i loadGradX1    = _mm256_load2_si128_offset4(gradX1);
    __m256i loadGradY0    = _mm256_load2_si128_offset4(gradY0);
    __m256i loadGradY1    = _mm256_load2_si128_offset4(gradY1);
    __m256i subTemp1      = _mm256_sub_epi16(shiftSrcY1Tmp, shiftSrcY0Tmp);
    __m256i packTempX     = _mm256_srai_epi16(_mm256_add_epi16(loadGradX0, loadGradX1), shift5);
    __m256i packTempY     = _mm256_srai_epi16(_mm256_add_epi16(loadGradY0, loadGradY1), shift5);
    __m256i gX            = _mm256_abs_epi16(packTempX);
    __m256i gY            = _mm256_abs_epi16(packTempY);
    __m256i dIX           = _mm256_sign_epi16(subTemp1,  packTempX );
    __m256i dIY           = _mm256_sign_epi16(subTemp1,  packTempY );
    __m256i signGY_GX     = _mm256_sign_epi16(packTempX, packTempY );

    sumAbsGXTmp     = _mm256_add_epi16(sumAbsGXTmp, gX);
    sumDIXTmp       = _mm256_add_epi16(sumDIXTmp, dIX);
    sumAbsGYTmp     = _mm256_add_epi16(sumAbsGYTmp, gY);
    sumDIYTmp       = _mm256_add_epi16(sumDIYTmp, dIY);
    sumSignGyGxTmp  = _mm256_add_epi16(sumSignGyGxTmp, signGY_GX);

    srcY0Tmp += srcStride;
    srcY1Tmp += srcStride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
  }

#undef _mm256_load2_si128_offset4

  sumAbsGXTmp    = _mm256_madd_epi16(sumAbsGXTmp,    _mm256_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0));
  sumDIXTmp      = _mm256_madd_epi16(sumDIXTmp,      _mm256_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0));
  sumAbsGYTmp    = _mm256_madd_epi16(sumAbsGYTmp,    _mm256_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0));
  sumDIYTmp      = _mm256_madd_epi16(sumDIYTmp,      _mm256_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0));
  sumSignGyGxTmp = _mm256_madd_epi16(sumSignGyGxTmp, _mm256_setr_epi16(1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0));

  __m256i a12 = _mm256_unpacklo_epi32(sumAbsGXTmp, sumAbsGYTmp);
  __m256i a3  = _mm256_unpackhi_epi32(sumAbsGXTmp, sumAbsGYTmp);
  __m256i b12 = _mm256_unpacklo_epi32(sumDIXTmp, sumDIYTmp);
  __m256i b3  = _mm256_unpackhi_epi32(sumDIXTmp, sumDIYTmp);
  __m256i c1  = _mm256_unpacklo_epi64(a12, b12);
  __m256i c2  = _mm256_unpackhi_epi64(a12, b12);
  __m256i c3  = _mm256_unpacklo_epi64(a3, b3);

  c1 = _mm256_add_epi32(c1, c2);
  c1 = _mm256_add_epi32(c1, c3);

  int tmpData[8];

  _mm256_storeu_si256( ( __m256i* ) &tmpData[0], c1 );

  #define sumAbsGX0 tmpData[0]
  #define sumAbsGX1 tmpData[4]

  #define sumAbsGY0 tmpData[1]
  #define sumAbsGY1 tmpData[5]

  #define sumDIX0   tmpData[2]
  #define sumDIX1   tmpData[6]

  #define sumDIY0   tmpData[3]
  #define sumDIY1   tmpData[7]

  sumSignGyGxTmp = _mm256_add_epi32(sumSignGyGxTmp, _mm256_shuffle_epi32(sumSignGyGxTmp, 0x4e));   // 01001110
  sumSignGyGxTmp = _mm256_add_epi32(sumSignGyGxTmp, _mm256_shuffle_epi32(sumSignGyGxTmp, 0xb1));   // 10110001

  int sumSignGY_GX0 = _mm256_extract_epi32( sumSignGyGxTmp, 0 );
  int sumSignGY_GX1 = _mm256_extract_epi32( sumSignGyGxTmp, 4 );

#if 0
  tmpx0 = sumAbsGX0 == 0 ? 0 : Clip3( -limit, limit, rightShiftMSB( sumDIX0 << 2, sumAbsGX0 ) );
  tmpx1 = sumAbsGX1 == 0 ? 0 : Clip3( -limit, limit, rightShiftMSB( sumDIX1 << 2, sumAbsGX1 ) );
  __m128i vtmpx         = _mm_setr_epi32 ( tmpx0, tmpx1, 0, 0 );
  __m128i vsumSignGY_GX = _mm_setr_epi32 ( sumSignGY_GX0, sumSignGY_GX1, 0, 0 );
  __m128i vmainsGxGy    = _mm_srai_epi32 ( vsumSignGY_GX, 12 );
  __m128i vsecsGxGy     = _mm_and_si128  ( vsumSignGY_GX, _mm_set1_epi32( ( 1 << 12 ) - 1 ) );
  __m128i vtmpData      = _mm_mullo_epi32( vtmpx, vmainsGxGy );
  vtmpData              = _mm_slli_epi32 ( vtmpData, 12 );
  vtmpData              = _mm_add_epi32  ( vtmpData, _mm_mullo_epi32( vtmpx, vsecsGxGy ) );
  vtmpData              = _mm_srai_epi32 ( vtmpData, 1 );
  __m128i vtmpyIn       = _mm_slli_epi32 ( _mm_setr_epi32( sumDIY0, sumDIY1, 0, 0 ), 2 );
  vtmpyIn               = _mm_sub_epi32  ( vtmpyIn, vtmpData );

  tmpy0 = sumAbsGY0 == 0 ? 0 : Clip3( -limit, limit, rightShiftMSB( _mm_extract_epi32( vtmpyIn, 0 ), sumAbsGY0 ) );
  tmpy1 = sumAbsGY1 == 0 ? 0 : Clip3( -limit, limit, rightShiftMSB( _mm_extract_epi32( vtmpyIn, 1 ), sumAbsGY1 ) );
#else
  tmpx0 = sumAbsGX0 == 0 ? 0 : rightShiftMSB( sumDIX0 *4, sumAbsGX0 );
  tmpx0 = Clip3( -limit, limit, tmpx0 );

  int mainsGxGy0 = sumSignGY_GX0 >> 12;
  int secsGxGy0  = sumSignGY_GX0 & ( ( 1 << 12 ) - 1 );
  int tmpData0   = tmpx0 * mainsGxGy0;
  tmpData0       = ( ( tmpData0 * ( 1 << 12 )) + tmpx0 * secsGxGy0 ) >> 1;
  tmpy0 = sumAbsGY0 == 0 ? 0 : rightShiftMSB( ( ( sumDIY0 *4) - tmpData0 ), sumAbsGY0 );
  tmpy0 = Clip3( -limit, limit, tmpy0 );


  tmpx1 = sumAbsGX1 == 0 ? 0 : rightShiftMSB( sumDIX1 *4, sumAbsGX1 );
  tmpx1 = Clip3( -limit, limit, tmpx1 );

  int mainsGxGy1 = sumSignGY_GX1 >> 12;
  int secsGxGy1  = sumSignGY_GX1 & ( ( 1 << 12 ) - 1 );
  int tmpData1   = tmpx1 * mainsGxGy1;
  tmpData1 = ( ( tmpData1 * ( 1 << 12 )) + tmpx1 * secsGxGy1 ) >> 1;
  tmpy1 = sumAbsGY1 == 0 ? 0 : rightShiftMSB( ( ( sumDIY1*4 ) - tmpData1 ), sumAbsGY1 );
  tmpy1 = Clip3( -limit, limit, tmpy1 );
#endif

#undef sumAbsGX0
#undef sumAbsGX1
#undef sumAbsGY0
#undef sumAbsGY1
#undef sumDIX0  
#undef sumDIX1  
#undef sumDIY0  
#undef sumDIY1  
}
#endif

template<X86_VEXT vext, int bitDepth>
static void BiOptFlowCoreSIMDBitDepth( const Pel* srcY0,
                        const Pel* srcY1,
                        const Pel* gradX0,
                        const Pel* gradX1,
                        const Pel* gradY0,
                        const Pel* gradY1,
                        const int  width,
                        const int  height,
                              Pel* dstY,
                        const ptrdiff_t dstStride,
                        const int  shiftNum,
                        const int  offset,
                        const int  limit,
                        const ClpRng& clpRng,
                        const int /* bitDepth: the template argument */ )
{
  CHECKD( shiftNum != IF_INTERNAL_PREC + 1 - bitDepth, "BDOF shift" );
  const int widthG        = width  + 2 * BDOF_EXTEND_SIZE;
  const int stridePredMC  = widthG + 2;
        int offsetPos     = widthG * BDOF_EXTEND_SIZE + BDOF_EXTEND_SIZE;
  const int xUnit         = ( width  >> 2 );
  const int yUnit         = ( height >> 2 );

  const Pel* srcY0Temp;
  const Pel* srcY1Temp;
        Pel *dstY0;
  
  int OffPos;
  int OffPad = 0;

  for( int yu = 0; yu < yUnit; yu++, srcY0 += ( stridePredMC << 2 ), srcY1 += ( stridePredMC << 2 ), dstY += ( dstStride << 2 ), offsetPos += ( widthG << 2 ) )
  {
    srcY0Temp = srcY0;
    srcY1Temp = srcY1;
    dstY0     = dstY;
    
    OffPos = offsetPos;
    OffPad = ( ( yu * widthG ) << 2 );

#if USE_AVX2
    for( int xu = 0; xu < xUnit; xu += 2, srcY0Temp += 8, srcY1Temp += 8, dstY0 += 8, OffPos += 8, OffPad += 8 )
    {
      int tmpx0, tmpy0, tmpx1, tmpy1;

      //calcBIOSums_SSE<vext>( srcY0Temp + 0, srcY1Temp + 0, gradX0 + OffPad + 0, gradX1 + OffPad + 0, gradY0 + OffPad + 0, gradY1 + OffPad + 0, stridePredMC, bitDepth, limit, tmpx, tmpy );
      //calcBIOSums_SSE<vext>( srcY0Temp + 0, srcY1Temp + 0, gradX0 + OffPad + 0, gradX1 + OffPad + 0, gradY0 + OffPad + 0, gradY1 + OffPad + 0, stridePredMC, bitDepth, limit, tmpx, tmpy );
      calcBIOSums2x_AVX2( srcY0Temp, srcY1Temp, gradX0 + OffPad, gradX1 + OffPad, gradY0 + OffPad, gradY1 + OffPad, widthG, bitDepth, limit, tmpx0, tmpx1, tmpy0, tmpy1 );

      //addBIOAvg4_SSE<vext>( srcY0Temp + stridePredMC + 1 + 0, srcY1Temp + stridePredMC + 1 + 0, dstY0 + 0, dstStride, gradX0 + OffPos + 0, gradX1 + OffPos + 0, gradY0 + OffPos + 0, gradY1 + OffPos + 0, widthG, tmpx0, tmpy0, shiftNum, offset, clpRng );
      //addBIOAvg4_SSE<vext>( srcY0Temp + stridePredMC + 1 + 4, srcY1Temp + stridePredMC + 1 + 4, dstY0 + 4, dstStride, gradX0 + OffPos + 4, gradX1 + OffPos + 4, gradY0 + OffPos + 4, gradY1 + OffPos + 4, widthG, tmpx1, tmpy1, shiftNum, offset, clpRng );
      addBIOAvg4_2x_AVX2( srcY0Temp + stridePredMC + 1, srcY1Temp + stridePredMC + 1, dstY0, dstStride, gradX0 + OffPos, gradX1 + OffPos, gradY0 + OffPos, gradY1 + OffPos, widthG, tmpx0, tmpx1, tmpy0, tmpy1, IF_INTERNAL_PREC + 1 - bitDepth, offset, clpRng );
    }  // xu
#else
    for( int xu = 0; xu < xUnit; xu++, srcY0Temp += 4, srcY1Temp += 4, dstY0 += 4, OffPos += 4, OffPad += 4 )
    {
      int tmpx, tmpy;

      calcBIOSums_SSE<vext>( srcY0Temp, srcY1Temp, gradX0 + OffPad, gradX1 + OffPad, gradY0 + OffPad, gradY1 + OffPad, widthG, bitDepth, limit, tmpx, tmpy );

      addBIOAvg4_SSE<vext> ( srcY0Temp + stridePredMC + 1, srcY1Temp + stridePredMC + 1, dstY0, dstStride, gradX0 + OffPos, gradX1 + OffPos, gradY0 + OffPos, gradY1 + OffPos, widthG, tmpx, tmpy, IF_INTERNAL_PREC + 1 - bitDepth, offset, clpRng );
    }  // xu
#endif
  }  // yu
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext>
void BiOptFlowCoreSIMD( const Pel* srcY0,
                        const Pel* srcY1,
                        const Pel* gradX0,
                        const Pel* gradX1,
                        const Pel* gradY0,
                        const Pel* gradY1,
                        const int  width,
                        const int  height,
                              Pel* dstY,
                        const ptrdiff_t dstStride,
                        const int  shiftNum,
                        const int  offset,
                        const int  limit,
                        const ClpRng& clpRng,
                        const int bitDepth )
{
  CHECKD( bitDepth != clpRng.bd, "BDOF bit depth" );
  dispatchBitDepth( clpRng.bd, [&]( auto bd ) { BiOptFlowCoreSIMDBitDepth<vext, decltype( bd )::value>( srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, width, height, dstY, dstStride, shiftNum, offset, limit, clpRng, bitDepth ); } );
}

template< X86_VEXT vext, bool PAD = true>
void gradFilter_SSE(const Pel* src, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth)
{
  const Pel* srcTmp = src + srcStride + 1;
  Pel* gradXTmp = gradX + gradStride + 1;
  Pel* gradYTmp = gradY + gradStride + 1;

  int widthInside = width - 2 * BDOF_EXTEND_SIZE;
  int heightInside = height - 2 * BDOF_EXTEND_SIZE;
  int shift1 = 6;
  __m128i mmShift1 = _mm_cvtsi32_si128(shift1);
  assert((widthInside & 3) == 0);

  if ((widthInside & 7) == 0)
  {
    for (int y = 0; y < heightInside; y++)
    {
      int x = 0;
      for (; x < widthInside; x += 8)
      {
        __m128i mmPixTop = _mm_sra_epi16(_mm_loadu_si128((__m128i*) (srcTmp + x - srcStride)), mmShift1);
        __m128i mmPixBottom = _mm_sra_epi16(_mm_loadu_si128((__m128i*) (srcTmp + x + srcStride)), mmShift1);
        __m128i mmPixLeft = _mm_sra_epi16(_mm_loadu_si128((__m128i*) (srcTmp + x - 1)), mmShift1);
        __m128i mmPixRight = _mm_sra_epi16(_mm_loadu_si128((__m128i*) (srcTmp + x + 1)), mmShift1);

        __m128i mmGradVer = _mm_sub_epi16(mmPixBottom, mmPixTop);
        __m128i mmGradHor = _mm_sub_epi16(mmPixRight, mmPixLeft);

        _mm_storeu_si128((__m128i*) (gradYTmp + x), mmGradVer);
        _mm_storeu_si128((__m128i*) (gradXTmp + x), mmGradHor);
      }

      gradXTmp += gradStride;
      gradYTmp += gradStride;
      srcTmp += srcStride;
    }
  }
  else
  {
    __m128i mmPixTop = _mm_sra_epi16(_mm_unpacklo_epi64(_vv_loadl_epi64((__m128i*) (srcTmp - srcStride)), _vv_loadl_epi64((__m128i*) (srcTmp))), mmShift1);
    for (int y = 0; y < heightInside; y += 2)
    {
      __m128i mmPixBottom = _mm_sra_epi16(_mm_unpacklo_epi64(_vv_loadl_epi64((__m128i*) (srcTmp + srcStride)), _vv_loadl_epi64((__m128i*) (srcTmp + (srcStride << 1)))), mmShift1);
      __m128i mmPixLeft = _mm_sra_epi16(_mm_unpacklo_epi64(_vv_loadl_epi64((__m128i*) (srcTmp - 1)), _vv_loadl_epi64((__m128i*) (srcTmp - 1 + srcStride))), mmShift1);
      __m128i mmPixRight = _mm_sra_epi16(_mm_unpacklo_epi64(_vv_loadl_epi64((__m128i*) (srcTmp + 1)), _vv_loadl_epi64((__m128i*) (srcTmp + 1 + srcStride))), mmShift1);

      __m128i mmGradVer = _mm_sub_epi16(mmPixBottom, mmPixTop);
      __m128i mmGradHor = _mm_sub_epi16(mmPixRight, mmPixLeft);

      _vv_storel_epi64((__m128i*) gradYTmp, mmGradVer);
      _vv_storel_epi64((__m128i*) (gradYTmp + gradStride), _mm_unpackhi_epi64(mmGradVer, mmGradHor));
      _vv_storel_epi64((__m128i*) gradXTmp, mmGradHor);
      _vv_storel_epi64((__m128i*) (gradXTmp + gradStride), _mm_unpackhi_epi64(mmGradHor, mmGradVer));

      mmPixTop = mmPixBottom;
      gradXTmp += gradStride << 1;
      gradYTmp += gradStride << 1;
      srcTmp += srcStride << 1;
    }
  }

  if (PAD)
  {
    gradXTmp = gradX + gradStride + 1;
    gradYTmp = gradY + gradStride + 1;
    for (int y = 0; y < heightInside; y++)
    {
      gradXTmp[-1] = gradXTmp[0];
      gradXTmp[widthInside] = gradXTmp[widthInside - 1];
      gradXTmp += gradStride;

      gradYTmp[-1] = gradYTmp[0];
      gradYTmp[widthInside] = gradYTmp[widthInside - 1];
      gradYTmp += gradStride;
    }

    gradXTmp = gradX + gradStride;
    gradYTmp = gradY + gradStride;
    ::memcpy(gradXTmp - gradStride, gradXTmp, sizeof(Pel) * (width));
    ::memcpy(gradXTmp + heightInside * gradStride, gradXTmp + (heightInside - 1) * gradStride, sizeof(Pel) * (width));
    ::memcpy(gradYTmp - gradStride, gradYTmp, sizeof(Pel) * (width));
    ::memcpy(gradYTmp + heightInside * gradStride, gradYTmp + (heightInside - 1) * gradStride, sizeof(Pel) * (width));
  }
}

template< X86_VEXT vext >
void applyPROF_SSE(Pel* dstPel, int dstStride, const Pel* srcPel, int srcStride, int width, int height, const Pel* gradX, const Pel* gradY, int gradStride, const int* dMvX, const int* dMvY, int dMvStride, const bool& bi, int shiftNum, Pel offset, const ClpRng& clpRng)
{
  CHECKD( width != 4 || height != 4, "block width error!");

  const int dILimit = 1 << std::max<int>(clpRng.bd + 1, 13);

#if USE_AVX2
  __m256i mm_dmvx, mm_dmvy, mm_gradx, mm_grady, mm_dI, mm_dI0, mm_src;
  __m256i mm_offset = _mm256_set1_epi16( offset );
  __m256i vibdimin  = _mm256_set1_epi16( clpRng.min() );
  __m256i vibdimax  = _mm256_set1_epi16( clpRng.max() );
  __m256i mm_dimin  = _mm256_set1_epi32( -dILimit );
  __m256i mm_dimax  = _mm256_set1_epi32( dILimit - 1 );

  const int *vX0 = dMvX, *vY0 = dMvY;
  const Pel *gX0 = gradX, *gY0 = gradY;

  // first two rows
  mm_dmvx = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) vX0 ) ), _mm_loadu_si128( ( const __m128i * )( vX0 + dMvStride ) ), 1 );
  mm_dmvy = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) vY0 ) ), _mm_loadu_si128( ( const __m128i * )( vY0 + dMvStride ) ), 1 );

  mm_dmvx = _mm256_packs_epi32( mm_dmvx, _mm256_setzero_si256() );
  mm_dmvy = _mm256_packs_epi32( mm_dmvy, _mm256_setzero_si256() );

  mm_gradx = _mm256_inserti128_si256( _mm256_castsi128_si256( _vv_loadl_epi64( ( __m128i* )gX0 ) ), _vv_loadl_epi64( ( __m128i* )( gX0 + gradStride ) ), 1 );
  mm_grady = _mm256_inserti128_si256( _mm256_castsi128_si256( _vv_loadl_epi64( ( __m128i* )gY0 ) ), _vv_loadl_epi64( ( __m128i* )( gY0 + gradStride ) ), 1 );
  
  mm_dI0   = _mm256_madd_epi16( _mm256_unpacklo_epi16( mm_dmvx, mm_dmvy ), _mm256_unpacklo_epi16( mm_gradx, mm_grady ) );
  mm_dI0   = _mm256_min_epi32( mm_dimax, _mm256_max_epi32( mm_dimin, mm_dI0 ) );

  // next two rows
  vX0 += ( dMvStride << 1 ); vY0 += ( dMvStride << 1 ); gX0 += ( gradStride << 1 ); gY0 += ( gradStride << 1 );
  
  mm_dmvx = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) vX0 ) ), _mm_loadu_si128( ( const __m128i * )( vX0 + dMvStride ) ), 1 );
  mm_dmvy = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) vY0 ) ), _mm_loadu_si128( ( const __m128i * )( vY0 + dMvStride ) ), 1 );

  mm_dmvx = _mm256_packs_epi32( mm_dmvx, _mm256_setzero_si256() );
  mm_dmvy = _mm256_packs_epi32( mm_dmvy, _mm256_setzero_si256() );

  mm_gradx = _mm256_inserti128_si256( _mm256_castsi128_si256( _vv_loadl_epi64( ( __m128i* )gX0 ) ), _vv_loadl_epi64( ( __m128i* )( gX0 + gradStride ) ), 1 );
  mm_grady = _mm256_inserti128_si256( _mm256_castsi128_si256( _vv_loadl_epi64( ( __m128i* )gY0 ) ), _vv_loadl_epi64( ( __m128i* )( gY0 + gradStride ) ), 1 );
  
  mm_dI    = _mm256_madd_epi16( _mm256_unpacklo_epi16( mm_dmvx, mm_dmvy ), _mm256_unpacklo_epi16( mm_gradx, mm_grady ) );
  mm_dI    = _mm256_min_epi32( mm_dimax, _mm256_max_epi32( mm_dimin, mm_dI ) );

  // combine four rows
  mm_dI = _mm256_packs_epi32( mm_dI0, mm_dI );
  const Pel* src0 = srcPel + srcStride;
  mm_src = _mm256_inserti128_si256(
    _mm256_castsi128_si256(_mm_unpacklo_epi64(_vv_loadl_epi64((const __m128i *)srcPel), _vv_loadl_epi64((const __m128i *)(srcPel + (srcStride << 1))))),
    _mm_unpacklo_epi64(_vv_loadl_epi64((const __m128i *)src0), _vv_loadl_epi64((const __m128i *)(src0 + (srcStride << 1)))),
    1
  );
  mm_dI = _mm256_add_epi16(mm_dI, mm_src);
  if (!bi)
  {
    mm_dI = _mm256_srai_epi16(_mm256_adds_epi16(mm_dI, mm_offset), shiftNum);
    mm_dI = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, mm_dI));
  }

  // store final results
  __m128i dITmp = _mm256_extractf128_si256(mm_dI, 1);
  Pel* dst0 = dstPel;
  _vv_storel_epi64((__m128i *)dst0, _mm256_castsi256_si128(mm_dI));
  dst0 += dstStride; _vv_storel_epi64((__m128i *)dst0, dITmp);
  dst0 += dstStride; _vv_storel_epi64((__m128i *)dst0, _mm_unpackhi_epi64(_mm256_castsi256_si128(mm_dI), _mm256_castsi256_si128(mm_dI)));
  dst0 += dstStride; _vv_storel_epi64((__m128i *)dst0, _mm_unpackhi_epi64(dITmp, dITmp));
#else
  __m128i mm_dmvx, mm_dmvy, mm_gradx, mm_grady, mm_dI, mm_dI0;
  __m128i mm_offset = _mm_set1_epi16( offset );
  __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
  __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );
  __m128i mm_dimin  = _mm_set1_epi32( -dILimit );
  __m128i mm_dimax  = _mm_set1_epi32( dILimit - 1 );

  for( int h = 0; h < height; h += 2 )
  {
    const int* vX = dMvX;
    const int* vY = dMvY;
    const Pel* gX = gradX;
    const Pel* gY = gradY;
    const Pel* src = srcPel;
    Pel*       dst = dstPel;

    // first row
    mm_dmvx  = _mm_packs_epi32( _mm_loadu_si128( ( const __m128i * ) vX ), _mm_setzero_si128() );
    mm_dmvy  = _mm_packs_epi32( _mm_loadu_si128( ( const __m128i * ) vY ), _mm_setzero_si128() );
    mm_gradx = _vv_loadl_epi64( ( __m128i* ) gX );
    mm_grady = _vv_loadl_epi64( ( __m128i* ) gY );
    mm_dI0   = _mm_madd_epi16 ( _mm_unpacklo_epi16( mm_dmvx, mm_dmvy ), _mm_unpacklo_epi16( mm_gradx, mm_grady ) );
    mm_dI0   = _mm_min_epi32  ( mm_dimax, _mm_max_epi32( mm_dimin, mm_dI0 ) );

    // second row
    mm_dmvx  = _mm_packs_epi32( _mm_loadu_si128( ( const __m128i * ) ( vX + dMvStride ) ), _mm_setzero_si128() );
    mm_dmvy  = _mm_packs_epi32( _mm_loadu_si128( ( const __m128i * ) ( vY + dMvStride ) ), _mm_setzero_si128() );
    mm_gradx = _vv_loadl_epi64( ( __m128i* ) ( gX + gradStride ) );
    mm_grady = _vv_loadl_epi64( ( __m128i* ) ( gY + gradStride ) );
    mm_dI    = _mm_madd_epi16 ( _mm_unpacklo_epi16( mm_dmvx, mm_dmvy ), _mm_unpacklo_epi16( mm_gradx, mm_grady ) );
    mm_dI    = _mm_min_epi32  ( mm_dimax, _mm_max_epi32( mm_dimin, mm_dI ) );

    // combine both rows
    mm_dI = _mm_packs_epi32( mm_dI0, mm_dI );
    mm_dI = _mm_add_epi16  ( _mm_unpacklo_epi64( _vv_loadl_epi64( ( const __m128i * )src ), _vv_loadl_epi64( ( const __m128i * )( src + srcStride ) ) ), mm_dI );
    if (!bi)
    {
      mm_dI = _mm_srai_epi16(_mm_adds_epi16(mm_dI, mm_offset), shiftNum);
      mm_dI = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, mm_dI));
    }

    _vv_storel_epi64( ( __m128i * )  dst,                                   mm_dI );
    _vv_storel_epi64( ( __m128i * )( dst + dstStride ), _mm_unpackhi_epi64( mm_dI, mm_dI ) );

    dMvX   += (dMvStride  << 1);
    dMvY   += (dMvStride  << 1);
    gradX  += (gradStride << 1);
    gradY  += (gradStride << 1);
    srcPel += (srcStride  << 1);
    dstPel += (dstStride  << 1);
  }
#endif
}

// One row of the 6x6 PROF window, columns -1 .. 6 of the sub-block row at ref (only -1 .. 4 are used), converted to
// the 14-bit intermediate format of the prediction with the same 16-bit arithmetic as the scalar border copy.
// The two extra samples on the right stay inside the picture margin: xPredAffineBlk clips each sub-block MV so that
// it moves the CU origin at most 7 samples past the right picture edge, the sub-block starts at most CTUSize - 4 into
// the CU and xOffset adds 1, so the load ends at most CTUSize + 10 samples past the edge, inside the CTUSize + 16
// margin of the reference pictures (EncStage.h).
// borderShift is a template argument: an immediate shift, which SIMDE's Neon translation also needs.
template<int borderShift>
static ALWAYS_INLINE __m128i profBorderRow( const Pel* ref )
{
  const __m128i row = _mm_loadu_si128( ( const __m128i* ) ( ref - 1 ) );
  return _mm_sub_epi16( _mm_slli_epi16( row, borderShift ), _mm_set1_epi16( IF_INTERNAL_OFFS ) );
}

// PROF of one 4x4 affine sub-block. pred holds the filtered sub-block, ref its nearest integer position in the
// reference picture, which supplies the one-sample border. Each window row lives in one register (columns -1 .. 6 in
// lanes 0 .. 7), so both gradients are register shifts: gradX = right - left two lanes apart, gradY = the row below
// minus the row above. The arithmetic matches gradFilter_SSE< vext, false > followed by applyPROF_SSE, including the
// saturating packs and adds. Always inlined into affineLumaCU_SSE, which runs it for every sub-block.
template<X86_VEXT vext, int borderShift>
ALWAYS_INLINE void affinePROF4x4_SSE( Pel* dst, int dstStride, const Pel* pred, int predStride, const Pel* ref, int refStride, const int16_t* dMvXY, const bool bi, int shiftNum, Pel offset, const ClpRng& clpRng )
{
  const int     dILimit    = 1 << std::max<int>( clpRng.bd + 1, 13 );
  const __m128i dIMin      = _mm_set1_epi32( -dILimit );
  const __m128i dIMax      = _mm_set1_epi32( dILimit - 1 );

  // Window rows -1 .. 4, each shifted right by 6 as the gradient filter does: the top and bottom rows come from the
  // reference only (their corner lanes are never used), the inner rows take the prediction in lanes 1 .. 4 (blend
  // mask 0x1E).
  __m128i predRow[4];
  __m128i window[6];
  window[0] = _mm_srai_epi16( profBorderRow<borderShift>( ref - refStride ), 6 );
  for( int r = 0; r < 4; r++ )
  {
    predRow[r]        = _vv_loadl_epi64( ( const __m128i* ) ( pred + r * predStride ) );
    const __m128i row = _mm_blend_epi16( profBorderRow<borderShift>( ref + r * refStride ), _mm_slli_si128( predRow[r], 2 ), 0x1E );
    window[r + 1]     = _mm_srai_epi16( row, 6 );
  }
  window[5] = _mm_srai_epi16( profBorderRow<borderShift>( ref + 4 * refStride ), 6 );

  // dI = dMvX * gradX + dMvY * gradY per sample, clipped to the PROF limit (32 bit, one row per register).
  __m128i dI[4];
  for( int r = 0; r < 4; r++ )
  {
    const __m128i gradX = _mm_sub_epi16( _mm_srli_si128( window[r + 1], 4 ), window[r + 1] );
    const __m128i gradY = _mm_srli_si128( _mm_sub_epi16( window[r + 2], window[r] ), 2 );
    const __m128i dMv   = _mm_loadu_si128( ( const __m128i* ) ( dMvXY + 8 * r ) );
    dI[r] = _mm_madd_epi16( dMv, _mm_unpacklo_epi16( gradX, gradY ) );
    dI[r] = _mm_min_epi32( dIMax, _mm_max_epi32( dIMin, dI[r] ) );
  }

  const __m128i vOffset = _mm_set1_epi16( offset );
  const __m128i vMin    = _mm_set1_epi16( clpRng.min() );
  const __m128i vMax    = _mm_set1_epi16( clpRng.max() );
  for( int r = 0; r < 4; r += 2 )
  {
    __m128i res = _mm_add_epi16( _mm_packs_epi32( dI[r], dI[r + 1] ), _mm_unpacklo_epi64( predRow[r], predRow[r + 1] ) );
    if( !bi )
    {
      res = _mm_srai_epi16( _mm_adds_epi16( res, vOffset ), shiftNum );
      res = _mm_min_epi16( vMax, _mm_max_epi16( vMin, res ) );
    }
    _vv_storel_epi64( ( __m128i* ) ( dst + r * dstStride ), res );
    _vv_storel_epi64( ( __m128i* ) ( dst + ( r + 1 ) * dstStride ), _mm_unpackhi_epi64( res, res ) );
  }
}

// The MV of one luma sub-block, as xPredAffineBlk derives it: the affine model at the sub-block centre, rounded to
// 1/16 sample and to the MV storage range. It is stored for the chroma pass, which reads the luma MVs.
static inline Mv affineLumaSubblockMv( const InterPredInterpolation::AffineLumaCU& cu, int w, int h )
{
  const int half = AFFINE_MIN_BLOCK_SIZE >> 1;
  const int x    = cu.spreadOverLimit ? cu.width  >> 1 : half + w;
  const int y    = cu.spreadOverLimit ? cu.height >> 1 : half + h;
  int mvHor      = cu.mvScaleHor + cu.dMvHorX * x + cu.dMvVerX * y;
  int mvVer      = cu.mvScaleVer + cu.dMvHorY * x + cu.dMvVerY * y;
  roundAffineMv( mvHor, mvVer, cu.roundShift );
  Mv mv( mvHor, mvVer );
  mv.clipToStorageBitDepth();
  cu.storedMv[( h / AFFINE_MIN_BLOCK_SIZE ) * cu.storedMvStride + w / AFFINE_MIN_BLOCK_SIZE] = mv;
  return mv;
}

// The 6-tap 4x4 luma filter of m_filter4x4[0][isLast] without the call; src points at the sub-block.
template<X86_VEXT vext, bool isLast>
static ALWAYS_INLINE void affineLumaFilter4x4( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride,
                                               const TFilterCoeff* coeffH, const TFilterCoeff* coeffV, const Filter4x4N6Rounding& rnd )
{
  src -= 2 + 2 * srcStride;
#if USE_AVX2
  if( vext >= AVX2 )
  {
    simdFilter4x4_N6_AVX2<isLast>( clpRng, src, srcStride, dst, dstStride, coeffH, coeffV, rnd.offset1st, rnd.shift1st, rnd.offset2nd, rnd.shift2nd );
    return;
  }
#endif
  simdFilter4x4_N6_SSE<isLast, false>( clpRng, src, srcStride, dst, dstStride, coeffH, coeffV, rnd.offset1st, rnd.shift1st, rnd.offset2nd, rnd.shift2nd );
}

// Where one luma sub-block of the CU loop reads the reference: its MV, clipped to the reference window, split into the
// integer position (ref) and the 1/16 fractions; nearest is the nearest integer position, which PROF takes its border
// from.
struct AffineLumaSubblockRef
{
  const Pel* ref;
  const Pel* nearest;
  int        xFrac, yFrac;
};

static ALWAYS_INLINE AffineLumaSubblockRef affineLumaSubblockRef( const InterPredInterpolation::AffineLumaCU& cu, int w, int h )
{
  const Mv  mv    = affineLumaSubblockMv( cu, w, h );
  const int mvHor = std::min<int>( cu.horMax, std::max<int>( cu.horMin, mv.hor ) );
  const int mvVer = std::min<int>( cu.verMax, std::max<int>( cu.verMin, mv.ver ) );
  AffineLumaSubblockRef r;
  r.xFrac   = mvHor & 15;
  r.yFrac   = mvVer & 15;
  r.ref     = cu.ref + ( h + ( mvVer >> 4 ) ) * cu.refStride + w + ( mvHor >> 4 );
  r.nearest = r.ref + ( r.yFrac >> 3 ) * cu.refStride + ( r.xFrac >> 3 );
  return r;
}

// One luma sub-block of the CU loop: its 4x4 filter into dst. Returns the sub-block's nearest integer position.
template<X86_VEXT vext, bool isLast>
static ALWAYS_INLINE const Pel* affineLumaPredict4x4( const InterPredInterpolation::AffineLumaCU& cu, int w, int h,
                                                      const Filter4x4N6Rounding& rnd, Pel* dst, int dstStride )
{
  const AffineLumaSubblockRef r = affineLumaSubblockRef( cu, w, h );
  affineLumaFilter4x4<vext, isLast>( cu.clpRng, r.ref, cu.refStride, dst, dstStride,
                                     InterpolationFilter::m_lumaFilter4x4[r.xFrac], InterpolationFilter::m_lumaFilter4x4[r.yFrac], rnd );
  return r.nearest;
}

#if USE_AVX2
// Coefficient pair (k, k + 1) of the left sub-block's filter in the low 128-bit lane, of the right one's in the high lane.
static ALWAYS_INLINE __m256i coeffPairLanes( const TFilterCoeff* left, const TFilterCoeff* right, int k )
{
  return _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_set1_epi32( xCoeffPair( left[k], left[k + 1] ) ) ),
                                  _mm_set1_epi32( xCoeffPair( right[k], right[k + 1] ) ), 1 );
}

// The 14-bit (not last) 4x4 6-tap filter of two side-by-side sub-blocks in one pass, the left one in the low 128-bit
// lane and the right one in the high lane, each with its own fractions. Per lane these are the sums of
// simdFilter4x4_N6_AVX2<false>: the same sample pairs and coefficient pairs, the same multiply-add order and the same
// saturating packs, so the results are identical. The output is in the layout affinePROF4x4Pair_AVX2 works in:
// pred[0] holds rows 0 and 1 of each sub-block (row 0 in the low 64 bits of its lane), pred[1] rows 2 and 3. Unlike
// a filter per sub-block, the hand-off to PROF needs no reorder across the lanes and no trip through memory.
// srcLeft and srcRight point at the sub-blocks; the reads are those of the single 4x4 filter.
static ALWAYS_INLINE void affineLumaFilter4x4Pair_AVX2( const Pel* srcLeft, const Pel* srcRight, int srcStride,
                                                        const TFilterCoeff* coeffHLeft, const TFilterCoeff* coeffHRight,
                                                        const TFilterCoeff* coeffVLeft, const TFilterCoeff* coeffVRight,
                                                        const Filter4x4N6Rounding& rnd, __m256i* pred )
{
  srcLeft  -= 2 + 2 * srcStride;
  srcRight -= 2 + 2 * srcStride;
  const __m256i h12 = coeffPairLanes( coeffHLeft, coeffHRight, 1 );
  const __m256i h34 = coeffPairLanes( coeffHLeft, coeffHRight, 3 );
  const __m256i h56 = coeffPairLanes( coeffHLeft, coeffHRight, 5 );
  const __m256i off = _mm256_set1_epi32( rnd.offset1st );

  // horizontal filter of the 9 rows the 4 output rows need: rows[y] holds row y of both sub-blocks as 32-bit values
  __m256i rows[9];
  for( int y = 0; y < 9; y++ )
  {
    const Pel*    left  = srcLeft  + y * srcStride;
    const Pel*    right = srcRight + y * srcStride;
    const __m256i from0 = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* ) left ) ),
                                                   _mm_loadu_si128( ( const __m128i* ) right ), 1 );               // samples 0..7
    const __m256i from1 = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* ) ( left + 1 ) ) ),
                                                   _mm_loadu_si128( ( const __m128i* ) ( right + 1 ) ), 1 );       // samples 1..8
    const __m256i samplePairs0 = _mm256_unpacklo_epi16( from0, from1 );                  // (0,1) (1,2) (2,3) (3,4)
    const __m256i samplePairs4 = _mm256_unpackhi_epi16( from0, from1 );                  // (4,5) (5,6) (6,7) (7,8)
    const __m256i samplePairs2 = _mm256_alignr_epi8( samplePairs4, samplePairs0, 8 );    // (2,3) (3,4) (4,5) (5,6)
    __m256i sum = _mm256_madd_epi16( samplePairs0, h12 );
    sum = _mm256_add_epi32( sum, _mm256_madd_epi16( samplePairs2, h34 ) );
    sum = _mm256_add_epi32( sum, _mm256_madd_epi16( samplePairs4, h56 ) );
    rows[y] = _mm256_srai_epi32( _mm256_add_epi32( sum, off ), rnd.shift1st );
  }

  // vertical filter: rowPairs[y] holds rows y and y + 1 as interleaved 16-bit values, so output row y is three
  // multiply-adds, of the pairs (y, y + 1), (y + 2, y + 3) and (y + 4, y + 5)
  const __m256i interleave = _mm256_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15,
                                               0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 );
  __m256i rowPairs[8];
  for( int y = 0; y < 8; y++ )
  {
    rowPairs[y] = _mm256_shuffle_epi8( _mm256_packs_epi32( rows[y], rows[y + 1] ), interleave );
  }
  const __m256i v12  = coeffPairLanes( coeffVLeft, coeffVRight, 1 );
  const __m256i v34  = coeffPairLanes( coeffVLeft, coeffVRight, 3 );
  const __m256i v56  = coeffPairLanes( coeffVLeft, coeffVRight, 5 );
  const __m256i off2 = _mm256_set1_epi32( rnd.offset2nd );
  __m256i out[4];
  for( int y = 0; y < 4; y++ )
  {
    __m256i sum = _mm256_add_epi32( off2, _mm256_madd_epi16( rowPairs[y], v12 ) );
    sum = _mm256_add_epi32( sum, _mm256_madd_epi16( rowPairs[y + 2], v34 ) );
    sum = _mm256_add_epi32( sum, _mm256_madd_epi16( rowPairs[y + 4], v56 ) );
    out[y] = _mm256_srai_epi32( sum, rnd.shift2nd );
  }
  pred[0] = _mm256_packs_epi32( out[0], out[1] );
  pred[1] = _mm256_packs_epi32( out[2], out[3] );
}

// profBorderRow for two side-by-side sub-blocks, one per 128-bit lane.
template<int borderShift>
static ALWAYS_INLINE __m256i profBorderRowPair( const Pel* refLeft, const Pel* refRight )
{
  const __m256i row = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* ) ( refLeft - 1 ) ) ),
                                               _mm_loadu_si128( ( const __m128i* ) ( refRight - 1 ) ), 1 );
  return _mm256_sub_epi16( _mm256_slli_epi16( row, borderShift ), _mm256_set1_epi16( IF_INTERNAL_OFFS ) );
}

// affinePROF4x4_SSE for two side-by-side sub-blocks of a CU at once: the left one in the low 128-bit lane, the right
// one in the high lane. Every step of the 4x4 PROF is lane-local (byte shifts, blend, unpack, madd, packs, adds,
// min / max), so each lane computes exactly what affinePROF4x4_SSE computes for its sub-block, for about the work of
// one. pred is the output of affineLumaFilter4x4Pair_AVX2; dMvRows the CU's dMv pairs, row r in both lanes.
template<bool bi, int borderShift>
static ALWAYS_INLINE void affinePROF4x4Pair_AVX2( Pel* dst, int dstStride, const __m256i* pred, const Pel* refLeft, const Pel* refRight,
                                                  int refStride, const __m256i* dMvRows, int shiftNum, Pel offset,
                                                  const ClpRng& clpRng )
{
  const int     dILimit   = 1 << std::max<int>( clpRng.bd + 1, 13 );
  const __m256i dIMin     = _mm256_set1_epi32( -dILimit );
  const __m256i dIMax     = _mm256_set1_epi32( dILimit - 1 );

  // Window rows -1 .. 4: prediction row r of each sub-block (from pred, moved to lanes 1 .. 4) between its border
  // samples. Rows 1 and 3 sit in the high 64 bits of pred's lanes, so they shift right by 6 bytes instead of left by 2.
  __m256i window[6];
  window[0] = _mm256_srai_epi16( profBorderRowPair<borderShift>( refLeft - refStride, refRight - refStride ), 6 );
  for( int r = 0; r < 4; r++ )
  {
    const __m256i predRow = r & 1 ? _mm256_bsrli_epi128( pred[r >> 1], 6 ) : _mm256_bslli_epi128( pred[r >> 1], 2 );
    const __m256i row     = _mm256_blend_epi16( profBorderRowPair<borderShift>( refLeft + r * refStride, refRight + r * refStride ),
                                                predRow, 0x1E );
    window[r + 1]     = _mm256_srai_epi16( row, 6 );
  }
  window[5] = _mm256_srai_epi16( profBorderRowPair<borderShift>( refLeft + 4 * refStride, refRight + 4 * refStride ), 6 );

  __m256i dI[4];
  for( int r = 0; r < 4; r++ )
  {
    const __m256i gradX = _mm256_sub_epi16( _mm256_bsrli_epi128( window[r + 1], 4 ), window[r + 1] );
    const __m256i gradY = _mm256_bsrli_epi128( _mm256_sub_epi16( window[r + 2], window[r] ), 2 );
    dI[r] = _mm256_madd_epi16( dMvRows[r], _mm256_unpacklo_epi16( gradX, gradY ) );
    dI[r] = _mm256_min_epi32( dIMax, _mm256_max_epi32( dIMin, dI[r] ) );
  }

  const __m256i vOffset = _mm256_set1_epi16( offset );
  const __m256i vMin    = _mm256_set1_epi16( clpRng.min() );
  const __m256i vMax    = _mm256_set1_epi16( clpRng.max() );
  for( int r = 0; r < 4; r += 2 )
  {
    __m256i res = _mm256_add_epi16( _mm256_packs_epi32( dI[r], dI[r + 1] ), pred[r >> 1] );
    if( !bi )
    {
      res = _mm256_srai_epi16( _mm256_adds_epi16( res, vOffset ), shiftNum );
      res = _mm256_min_epi16( vMax, _mm256_max_epi16( vMin, res ) );
    }
    // the lanes hold (left r, left r + 1) and (right r, right r + 1): reorder to whole rows of 8 samples
    res = _mm256_permute4x64_epi64( res, 0xD8 );
    _mm_storeu_si128( ( __m128i* ) ( dst + r * dstStride ), _mm256_castsi256_si128( res ) );
    _mm_storeu_si128( ( __m128i* ) ( dst + ( r + 1 ) * dstStride ), _mm256_extracti128_si256( res, 1 ) );
  }
}
#endif

// xPredAffineBlk's luma sub-block loop in one call, so the filter and PROF of each sub-block need no call of their own.
// Every sub-block goes through the 2-D 4x4 filter, whatever its fractions: a zero fraction takes the identity taps
// m_lumaFilter4x4[0] = { 0, 0, 0, 64, 0, 0, 0, 0 }, which give exactly the horizontal-only, vertical-only and copy
// results of m_if's filterHor and filterVer (the identity stage stays within 14 bits, so no pack saturates), with no
// branch on the fractions. So every sub-block reads the 2-D footprint, rows and columns -2 .. +6 of the sub-block
// (columns to +8 with SSE41), as a 2-D sub-block already does anywhere inside the MV clip window: at most
// CTUSize + 9 (SSE41: + 11) samples past the right or bottom picture edge and CTUSize + 9 before the left or top
// one, inside the CTUSize + 16 margin of the reference pictures (EncStage.h).
// Instantiated per internal bit depth: the filter's shifts, the PROF border shift and shiftNum are constants, which the
// always-inlined sub-block functions take as immediates.
template<X86_VEXT vext, bool prof, bool bi, int bitDepth>
static void affineLumaCUBitDepth( const InterPredInterpolation::AffineLumaCU& cu )
{
  constexpr bool isLast   = !prof && !bi;   // as in xPredAffineBlk: PROF works on the 14-bit prediction
  constexpr Filter4x4N6Rounding rnd = filter4x4N6Rounding<isLast>( bitDepth );
  constexpr int  shiftNum = std::max<int>( 2, IF_INTERNAL_PREC - bitDepth );
  CHECKD( cu.shiftNum != shiftNum, "affine shiftNum" );

#if USE_AVX2
  if( vext >= AVX2 && prof )
  {
    // PROF two sub-blocks at a time, one per 128-bit lane; every row of an affine CU holds an even number of them
    __m256i dMvRows[4];
    for( int r = 0; r < 4; r++ )
    {
      dMvRows[r] = _mm256_broadcastsi128_si256( _mm_loadu_si128( ( const __m128i* ) ( cu.dMvXY + 8 * r ) ) );
    }
    for( int h = 0; h < cu.height; h += AFFINE_MIN_BLOCK_SIZE )
    {
      for( int w = 0; w < cu.width; w += 2 * AFFINE_MIN_BLOCK_SIZE )
      {
        const AffineLumaSubblockRef left  = affineLumaSubblockRef( cu, w, h );
        const AffineLumaSubblockRef right = affineLumaSubblockRef( cu, w + AFFINE_MIN_BLOCK_SIZE, h );
        __m256i pred[2];
        affineLumaFilter4x4Pair_AVX2( left.ref, right.ref, cu.refStride,
                                      InterpolationFilter::m_lumaFilter4x4[left.xFrac], InterpolationFilter::m_lumaFilter4x4[right.xFrac],
                                      InterpolationFilter::m_lumaFilter4x4[left.yFrac], InterpolationFilter::m_lumaFilter4x4[right.yFrac], rnd, pred );
        affinePROF4x4Pair_AVX2<bi, shiftNum>( cu.dst + h * cu.dstStride + w, cu.dstStride, pred, left.nearest, right.nearest,
                                              cu.refStride, dMvRows, shiftNum, cu.offset, cu.clpRng );
      }
    }
    _mm256_zeroupper();
    return;
  }
#endif

  for( int h = 0; h < cu.height; h += AFFINE_MIN_BLOCK_SIZE )
  {
    for( int w = 0; w < cu.width; w += AFFINE_MIN_BLOCK_SIZE )
    {
      Pel* dst = cu.dst + h * cu.dstStride + w;
      // with PROF the filtered sub-block goes through a small buffer (its stores forward to PROF's loads)
      alignas( 16 ) Pel pred[AFFINE_MIN_BLOCK_SIZE * AFFINE_MIN_BLOCK_SIZE];
      const Pel* refNearest = affineLumaPredict4x4<vext, isLast>( cu, w, h, rnd, prof ? pred : dst, prof ? AFFINE_MIN_BLOCK_SIZE : cu.dstStride );
      if( prof )
      {
        affinePROF4x4_SSE<vext, shiftNum>( dst, cu.dstStride, pred, AFFINE_MIN_BLOCK_SIZE, refNearest, cu.refStride, cu.dMvXY, bi,
                                           shiftNum, cu.offset, cu.clpRng );
      }
    }
  }
#if USE_AVX2
  if( vext >= AVX2 )
  {
    _mm256_zeroupper();
  }
#endif
}

template<X86_VEXT vext, bool prof, bool bi>
void affineLumaCU_SSE( const InterPredInterpolation::AffineLumaCU& cu )
{
  dispatchBitDepth( cu.clpRng.bd, [&]( auto bd ) { affineLumaCUBitDepth<vext, prof, bi, decltype( bd )::value>( cu ); } );
}

template<X86_VEXT vext>
void padDmvr_SSE( const Pel* src, const int srcStride, Pel* dst, const int dstStride, int width, int height, int padSize )
{
  _mm_prefetch( ( const char* )  src,            _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &src[srcStride], _MM_HINT_T0 );

  if( width == 7 && padSize == 1 )
  {
    const __m128i sl = _mm_setr_epi8( 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 12, 13 );
    __m128i l = _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i* ) src ), sl );

    _mm_storeu_si16 ( ( __m128i* ) ( dst - 1 * dstStride - 1 ), l );
    _mm_storeu_si128( ( __m128i* ) ( dst - 1 * dstStride     ), l );

    _mm_storeu_si16 ( ( __m128i* ) ( dst - 0 * dstStride - 1 ), l );
    _mm_storeu_si128( ( __m128i* ) ( dst - 0 * dstStride     ), l );

    for( height--, dst += dstStride, src += srcStride; height > 0; height--, src += srcStride, dst += dstStride )
    {
      _mm_prefetch( ( const char* ) &src[srcStride], _MM_HINT_T0 );

      l = _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i* ) src ), sl );

      _mm_storeu_si16 ( ( __m128i* ) ( dst - 1 ), l );
      _mm_storeu_si128( ( __m128i* ) ( dst     ), l );
    }

    _mm_storeu_si16 ( ( __m128i* ) ( dst - 1 ), l );
    _mm_storeu_si128( ( __m128i* ) ( dst     ), l );
  }
  else if( width == 11 && padSize == 1 )
  {
    const __m128i sl = _mm_setr_epi8( 0, 1, 2, 3, 4, 5, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15 );
    __m128i l0 =                   _mm_loadu_si128( ( const __m128i* ) &src[0] );
    __m128i l1 = _mm_shuffle_epi8( _vv_loadl_epi64( ( const __m128i* ) &src[8] ), sl );

    _mm_storeu_si16 ( ( __m128i* ) ( dst - 1 * dstStride - 1 ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 1 * dstStride     ), l0 );
    _vv_storel_epi64( ( __m128i* ) ( dst - 1 * dstStride + 8 ), l1 );

    _mm_storeu_si16 ( ( __m128i* ) ( dst - 0 * dstStride - 1 ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 0 * dstStride     ), l0 );
    _vv_storel_epi64( ( __m128i* ) ( dst - 0 * dstStride + 8 ), l1 );

    for( height--, dst += dstStride, src += srcStride; height > 0; height--, src += srcStride, dst += dstStride )
    {
      _mm_prefetch( ( const char* ) &src[srcStride], _MM_HINT_T0 );

      l0 =                   _mm_loadu_si128( ( const __m128i* ) &src[0] );
      l1 = _mm_shuffle_epi8( _vv_loadl_epi64( ( const __m128i* ) &src[8] ), sl );

      _mm_storeu_si16 ( ( __m128i* ) ( dst - 1 ), l0 );
      _mm_storeu_si128( ( __m128i* ) ( dst     ), l0 );
      _vv_storel_epi64( ( __m128i* ) ( dst + 8 ), l1 );
    }
    
    _mm_storeu_si16 ( ( __m128i* ) ( dst - 1 ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst     ), l0 );
    _vv_storel_epi64( ( __m128i* ) ( dst + 8 ), l1 );
  }
  else if( width == 15 && padSize == 2 )
  {
    const __m128i sl = _mm_setr_epi8(  0,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 12, 13 );
    const __m128i sb = _mm_setr_epi8(  0,  1, 0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 );
    const __m128i se = _mm_setr_epi8( 12, 13, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 );
    __m128i l0 =                   _mm_loadu_si128( ( const __m128i* ) &src[0] );
    __m128i l1 = _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i* ) &src[8] ), sl );
    __m128i b  = _mm_shuffle_epi8( l0, sb );
    __m128i e  = _mm_shuffle_epi8( l1, se );

    _mm_storeu_si32 ( ( __m128i* ) ( dst - 2 * dstStride -  2 ), b  );
    _mm_storeu_si128( ( __m128i* ) ( dst - 2 * dstStride      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 2 * dstStride +  8 ), l1 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst - 2 * dstStride + 16 ), e  );

    _mm_storeu_si32 ( ( __m128i* ) ( dst - 1 * dstStride -  2 ), b  );
    _mm_storeu_si128( ( __m128i* ) ( dst - 1 * dstStride      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 1 * dstStride +  8 ), l1 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst - 1 * dstStride + 16 ), e  );

    _mm_storeu_si32 ( ( __m128i* ) ( dst - 0 * dstStride -  2 ), b  );
    _mm_storeu_si128( ( __m128i* ) ( dst - 0 * dstStride      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 0 * dstStride +  8 ), l1 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst - 0 * dstStride + 16 ), e  );

    for( height--, dst += dstStride, src += srcStride; height > 0; height--, src += srcStride, dst += dstStride )
    {
      _mm_prefetch( ( const char* ) &src[srcStride], _MM_HINT_T0 );

      l0 =                   _mm_loadu_si128( ( const __m128i* ) &src[0] );
      l1 = _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i* ) &src[8] ), sl );
      b = _mm_shuffle_epi8( l0, sb );
      e = _mm_shuffle_epi8( l1, se );
      
      _mm_storeu_si32 ( ( __m128i* ) ( dst -  2 ), b  );
      _mm_storeu_si128( ( __m128i* ) ( dst      ), l0 );
      _mm_storeu_si128( ( __m128i* ) ( dst +  8 ), l1 );
      _mm_storeu_si16 ( ( __m128i* ) ( dst + 16 ), e  );
    }
    
    _mm_storeu_si32 ( ( __m128i* ) ( dst -  2 ), b  );
    _mm_storeu_si128( ( __m128i* ) ( dst      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst +  8 ), l1 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst + 16 ), e  );

    dst += dstStride;

    _mm_storeu_si32 ( ( __m128i* ) ( dst -  2 ), b  );
    _mm_storeu_si128( ( __m128i* ) ( dst      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst +  8 ), l1 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst + 16 ), e  );
  }
  else if( width == 23 && padSize == 2 )
  {
    const __m128i sl = _mm_setr_epi8(  0,  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 12, 13 );
    const __m128i sb = _mm_setr_epi8(  0,  1, 0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 );
    const __m128i se = _mm_setr_epi8( 12, 13, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 );
    __m128i l0 =                   _mm_loadu_si128( ( const __m128i* ) &src[ 0] );
    __m128i l1 =                   _mm_loadu_si128( ( const __m128i* ) &src[ 8] );
    __m128i l2 = _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i* ) &src[16] ), sl );
    __m128i b  = _mm_shuffle_epi8( l0, sb );
    __m128i e  = _mm_shuffle_epi8( l2, se );

    _mm_storeu_si32 ( ( __m128i* ) ( dst - 2 * dstStride -  2 ), b );
    _mm_storeu_si128( ( __m128i* ) ( dst - 2 * dstStride      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 2 * dstStride +  8 ), l1 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 2 * dstStride + 16 ), l2 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst - 2 * dstStride + 24 ), e );

    _mm_storeu_si32 ( ( __m128i* ) ( dst - 1 * dstStride -  2 ), b );
    _mm_storeu_si128( ( __m128i* ) ( dst - 1 * dstStride      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 1 * dstStride +  8 ), l1 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 1 * dstStride + 16 ), l2 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst - 1 * dstStride + 24 ), e );

    _mm_storeu_si32 ( ( __m128i* ) ( dst - 0 * dstStride -  2 ), b );
    _mm_storeu_si128( ( __m128i* ) ( dst - 0 * dstStride      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 0 * dstStride +  8 ), l1 );
    _mm_storeu_si128( ( __m128i* ) ( dst - 0 * dstStride + 16 ), l2 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst - 0 * dstStride + 24 ), e );

    for( height--, dst += dstStride, src += srcStride; height > 0; height--, src += srcStride, dst += dstStride )
    {
      _mm_prefetch( ( const char* ) &src[srcStride], _MM_HINT_T0 );

      l0 =                   _mm_loadu_si128( ( const __m128i* ) &src[ 0] );
      l1 =                   _mm_loadu_si128( ( const __m128i* ) &src[ 8] );
      l2 = _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i* ) &src[16] ), sl );
      b  = _mm_shuffle_epi8( l0, sb );
      e  = _mm_shuffle_epi8( l2, se );

      _mm_storeu_si32 ( ( __m128i* ) ( dst -  2 ), b );
      _mm_storeu_si128( ( __m128i* ) ( dst      ), l0 );
      _mm_storeu_si128( ( __m128i* ) ( dst +  8 ), l1 );
      _mm_storeu_si128( ( __m128i* ) ( dst + 16 ), l2 );
      _mm_storeu_si16 ( ( __m128i* ) ( dst + 24 ), e );
    }
    
    _mm_storeu_si32 ( ( __m128i* ) ( dst -  2 ), b );
    _mm_storeu_si128( ( __m128i* ) ( dst      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst +  8 ), l1 );
    _mm_storeu_si128( ( __m128i* ) ( dst + 16 ), l2 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst + 24 ), e );

    dst += dstStride;

    _mm_storeu_si32 ( ( __m128i* ) ( dst -  2 ), b );
    _mm_storeu_si128( ( __m128i* ) ( dst      ), l0 );
    _mm_storeu_si128( ( __m128i* ) ( dst +  8 ), l1 );
    _mm_storeu_si128( ( __m128i* ) ( dst + 16 ), l2 );
    _mm_storeu_si16 ( ( __m128i* ) ( dst + 24 ), e );
  }
  else
  {
    // TODO: fix for 444!

    g_pelBufOP.copyBuffer( ( const char* ) src, srcStride * sizeof( Pel ), ( char* ) dst, dstStride * sizeof( Pel ), width * sizeof( Pel ), height );

    /*left and right padding*/
    Pel* ptrTemp1 = dst;
    Pel* ptrTemp2 = dst + (width - 1);
    ptrdiff_t offset = 0;
    for( int i = 0; i < height; i++ )
    {
      offset = dstStride * i;
      for( int j = 1; j <= padSize; j++ )
      {
        *(ptrTemp1 - j + offset) = *(ptrTemp1 + offset);
        *(ptrTemp2 + j + offset) = *(ptrTemp2 + offset);
      }
    }
    /*Top and Bottom padding*/
    int numBytes = (width + padSize + padSize) * sizeof( Pel );
    ptrTemp1 = (dst - padSize);
    ptrTemp2 = (dst + (dstStride * (height - 1)) - padSize);
    for( int i = 1; i <= padSize; i++ )
    {
      memcpy( ptrTemp1 - (i * dstStride), (ptrTemp1), numBytes );
      memcpy( ptrTemp2 + (i * dstStride), (ptrTemp2), numBytes );
    }
  }
}

#if ENABLE_SIMD_OPT_BDOF
template<X86_VEXT vext>
void InterPredInterpolation::_initInterPredictionX86()
{
  xFpBiDirOptFlow     = BiOptFlowCoreSIMD<vext>;
  xFpBDOFGradFilter   = gradFilter_SSE<vext>;
  xFpProfGradFilter   = gradFilter_SSE<vext, false>;
  xFpApplyPROF        = applyPROF_SSE<vext>;
  xFpPadDmvr          = padDmvr_SSE<vext>;
#if defined( REAL_TARGET_X86 )   // builds that compile these kernels through SIMDE (Arm) keep their own path
  xFpAffineLumaCU[0][0] = affineLumaCU_SSE<vext, false, false>;
  xFpAffineLumaCU[0][1] = affineLumaCU_SSE<vext, false, true>;
  xFpAffineLumaCU[1][0] = affineLumaCU_SSE<vext, true, false>;
  xFpAffineLumaCU[1][1] = affineLumaCU_SSE<vext, true, true>;
#endif
}
template void InterPredInterpolation::_initInterPredictionX86<SIMDX86>();

#endif
} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86
//! \}

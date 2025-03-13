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
/** \file     InterPredX86.h
    \brief    SIMD for InterPrediction
*/

//! \ingroup CommonLib
//! \{


//#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "Rom.h"
#include "InterPrediction.h"

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
static inline void addBIOAvg4_SSE(const int16_t* src0, const int16_t* src1, int16_t* dst, ptrdiff_t dstStride, const int16_t* gradX0, const int16_t* gradX1, const int16_t* gradY0, const int16_t* gradY1, ptrdiff_t widthG, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
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
static inline void addBIOAvg4_2x_AVX2(const int16_t* src0, const int16_t* src1, int16_t* dst, ptrdiff_t dstStride, const int16_t* gradX0, const int16_t* gradX1, const int16_t* gradY0, const int16_t* gradY1, ptrdiff_t widthG, int tmpx0, int tmpx1, int tmpy0, int tmpy1, int shift, int offset, const ClpRng& clpRng)
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
      addBIOAvg4_2x_AVX2( srcY0Temp + stridePredMC + 1, srcY1Temp + stridePredMC + 1, dstY0, dstStride, gradX0 + OffPos, gradX1 + OffPos, gradY0 + OffPos, gradY1 + OffPos, widthG, tmpx0, tmpx1, tmpy0, tmpy1, shiftNum, offset, clpRng );
    }  // xu
#else
    for( int xu = 0; xu < xUnit; xu++, srcY0Temp += 4, srcY1Temp += 4, dstY0 += 4, OffPos += 4, OffPad += 4 )
    {
      int tmpx, tmpy;

      calcBIOSums_SSE<vext>( srcY0Temp, srcY1Temp, gradX0 + OffPad, gradX1 + OffPad, gradY0 + OffPad, gradY1 + OffPad, widthG, bitDepth, limit, tmpx, tmpy );

      addBIOAvg4_SSE<vext> ( srcY0Temp + stridePredMC + 1, srcY1Temp + stridePredMC + 1, dstY0, dstStride, gradX0 + OffPos, gradX1 + OffPos, gradY0 + OffPos, gradY1 + OffPos, widthG, tmpx, tmpy, shiftNum, offset, clpRng );
    }  // xu
#endif
  }  // yu
#if USE_AVX2

  _mm256_zeroupper();
#endif
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
}
template void InterPredInterpolation::_initInterPredictionX86<SIMDX86>();

#endif
} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86
//! \}

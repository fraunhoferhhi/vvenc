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
/** \file     InterPredX86.h
    \brief    SIMD for InterPrediction
*/

//! \ingroup CommonLib
//! \{


//#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "Rom.h"
#include "InterPrediction.h"

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {

#define _mm_storeu_si32(p, a) (void)(*(int*)(p) = _mm_cvtsi128_si32((a)))

template< X86_VEXT vext >
void addBDOFAvg4_SSE(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dst, int dstStride, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, unsigned shift, int offset, const ClpRng& clpRng)
{
#if USE_AVX2
  if( vext >= AVX2 )
  {
    __m256i mm_tmpx    = _mm256_set1_epi32( ( tmpx & 0xffff ) | ( tmpy << 16 ) );
    __m256i mm_offset  = _mm256_set1_epi32( offset );
    __m256i vibdimin   = _mm256_set1_epi16( clpRng.min );
    __m256i vibdimax   = _mm256_set1_epi16( clpRng.max );
    __m256i mm_a, mm_b, mm_c;
    __m256i mm_sum;

    for( int y = 0; y < 4; y += 2, dst += 2 * dstStride, src0 += 2 * src0Stride, src1 += 2 * src0Stride, gradX0 += 2 * gradStride, gradX1 += 2 * gradStride, gradY0 += 2 * gradStride, gradY1 += 2 * gradStride )
    {
      mm_a   = _mm256_castsi128_si256 (       _mm_loadl_epi64( (const __m128i *)   gradX0 ) );
      mm_a   = _mm256_inserti128_si256( mm_a, _mm_loadl_epi64( (const __m128i *) ( gradX0 + gradStride ) ), 1 );
      mm_b   = _mm256_castsi128_si256 (       _mm_loadl_epi64( (const __m128i *)   gradY0 ) );
      mm_b   = _mm256_inserti128_si256( mm_b, _mm_loadl_epi64( (const __m128i *) ( gradY0 + gradStride ) ), 1 );

      mm_a   = _mm256_unpacklo_epi16  ( mm_a, mm_b );

      mm_c   = _mm256_castsi128_si256 (       _mm_loadl_epi64( (const __m128i *)   gradX1 ) );
      mm_c   = _mm256_inserti128_si256( mm_c, _mm_loadl_epi64( (const __m128i *) ( gradX1 + gradStride ) ), 1 );
      mm_b   = _mm256_castsi128_si256 (       _mm_loadl_epi64( (const __m128i *)   gradY1 ) );
      mm_b   = _mm256_inserti128_si256( mm_b, _mm_loadl_epi64( (const __m128i *) ( gradY1 + gradStride ) ), 1 );

      mm_b   = _mm256_unpacklo_epi16  ( mm_c, mm_b );
                                      
      mm_a   = _mm256_sub_epi16       ( mm_a, mm_b );
      mm_sum = _mm256_madd_epi16      ( mm_a, mm_tmpx );
      mm_a   = _mm256_cvtepi16_epi32  (                           _mm_loadl_epi64( (const __m128i *) ( src0 ) ) );
      mm_a   = _mm256_inserti128_si256( mm_a, _mm_cvtepi16_epi32( _mm_loadl_epi64( (const __m128i *) ( src0 + src0Stride ) ) ), 1 );
      mm_b   = _mm256_cvtepi16_epi32  (                           _mm_loadl_epi64( (const __m128i *) ( src1 ) ) );
      mm_b   = _mm256_inserti128_si256( mm_b, _mm_cvtepi16_epi32( _mm_loadl_epi64( (const __m128i *) ( src1 + src1Stride ) ) ), 1 );
      mm_sum = _mm256_add_epi32       ( _mm256_add_epi32( mm_sum, mm_a ), _mm256_add_epi32( mm_b, mm_offset ) );
      mm_sum = _mm256_packs_epi32     ( _mm256_srai_epi32( mm_sum, shift ), mm_a );
      mm_sum = _mm256_min_epi16       ( vibdimax, _mm256_max_epi16( vibdimin, mm_sum ) );
                                      
      _mm_storel_epi64                ( (__m128i *)   dst,               _mm256_castsi256_si128  ( mm_sum    ) );
      _mm_storel_epi64                ( (__m128i *) ( dst + dstStride ), _mm256_extracti128_si256( mm_sum, 1 ) );
    }

  }
  else
  {
#endif
  __m128i mm_tmpx    = _mm_set1_epi32( ( tmpx & 0xffff ) | ( tmpy << 16 ) );
  __m128i mm_offset  = _mm_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max );
  __m128i mm_a;
  __m128i mm_b;
  __m128i mm_sum;

  for( int y = 0; y < 4; y++, dst += dstStride, src0 += src0Stride, src1 += src1Stride, gradX0 += gradStride, gradX1 += gradStride, gradY0 += gradStride, gradY1 += gradStride )
  {
    mm_a   = _mm_unpacklo_epi16 ( _mm_loadl_epi64( (const __m128i *) gradX0 ), _mm_loadl_epi64( (const __m128i *) gradY0 ) );
    mm_b   = _mm_unpacklo_epi16 ( _mm_loadl_epi64( (const __m128i *) gradX1 ), _mm_loadl_epi64( (const __m128i *) gradY1 ) );
    mm_a   = _mm_sub_epi16      ( mm_a, mm_b );
    mm_sum = _mm_madd_epi16     ( mm_a, mm_tmpx );
    mm_a   = _mm_cvtepi16_epi32 ( _mm_loadl_epi64( (const __m128i *) ( src0 ) ) );
    mm_b   = _mm_cvtepi16_epi32 ( _mm_loadl_epi64( (const __m128i *) ( src1 ) ) );
    //mm_sum = _mm_srai_epi32     ( _mm_add_epi32( mm_sum, mm_boffset ), 1 );
    mm_sum = _mm_add_epi32      ( _mm_add_epi32( mm_sum, mm_a ), _mm_add_epi32( mm_b, mm_offset ) );
    mm_sum = _mm_packs_epi32    ( _mm_srai_epi32( mm_sum, shift ), mm_a );
    mm_sum = _mm_min_epi16      ( vibdimax, _mm_max_epi16( vibdimin, mm_sum ) );
    _mm_storel_epi64            ( (__m128i *) dst, mm_sum );
  }
#if USE_AVX2
  }
#endif
}

template< X86_VEXT vext >
void calcBDOFSums_SSE(const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int xu, int yu, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGY_GX)

{
  int shift4 = 4;
  int shift5 = 1;

  __m128i sumAbsGXTmp = _mm_setzero_si128();
  __m128i sumDIXTmp = _mm_setzero_si128();
  __m128i sumAbsGYTmp = _mm_setzero_si128();
  __m128i sumDIYTmp = _mm_setzero_si128();
  __m128i sumSignGyGxTmp = _mm_setzero_si128();
  Pel tmpStore[8];
  for (int y = 0; y < 6; y++)
  {
    __m128i shiftSrcY0Tmp = _mm_srai_epi16(_mm_loadu_si128((__m128i*)(srcY0Tmp)), shift4);
    __m128i shiftSrcY1Tmp = _mm_srai_epi16(_mm_loadu_si128((__m128i*)(srcY1Tmp)), shift4);
    __m128i loadGradX0 = _mm_loadu_si128((__m128i*)(gradX0));
    __m128i loadGradX1 = _mm_loadu_si128((__m128i*)(gradX1));
    __m128i loadGradY0 = _mm_loadu_si128((__m128i*)(gradY0));
    __m128i loadGradY1 = _mm_loadu_si128((__m128i*)(gradY1));
    __m128i subTemp1 = _mm_sub_epi16(shiftSrcY1Tmp, shiftSrcY0Tmp);
    __m128i packTempX = _mm_srai_epi16(_mm_add_epi16(loadGradX0, loadGradX1), shift5);
    __m128i packTempY = _mm_srai_epi16(_mm_add_epi16(loadGradY0, loadGradY1), shift5);
    __m128i gX = _mm_abs_epi16(packTempX);
    __m128i gY = _mm_abs_epi16(packTempY);
    __m128i dIX       = _mm_sign_epi16(subTemp1,  packTempX );
    __m128i dIY       = _mm_sign_epi16(subTemp1,  packTempY );
    __m128i signGY_GX = _mm_sign_epi16(packTempX, packTempY );

    sumAbsGXTmp = _mm_add_epi16(sumAbsGXTmp, gX);
    sumDIXTmp = _mm_add_epi16(sumDIXTmp, dIX);
    sumAbsGYTmp = _mm_add_epi16(sumAbsGYTmp, gY);
    sumDIYTmp = _mm_add_epi16(sumDIYTmp, dIY);
    sumSignGyGxTmp = _mm_add_epi16(sumSignGyGxTmp, signGY_GX);
    srcY0Tmp += src0Stride;
    srcY1Tmp += src1Stride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
  }
  _mm_storeu_si128((__m128i *)tmpStore, sumAbsGXTmp);
  *sumAbsGX = tmpStore[0] + tmpStore[1] + tmpStore[2] + tmpStore[3] + tmpStore[4] + tmpStore[5];
  _mm_storeu_si128((__m128i *)tmpStore, sumAbsGYTmp);
  *sumAbsGY = tmpStore[0] + tmpStore[1] + tmpStore[2] + tmpStore[3] + tmpStore[4] + tmpStore[5];
  _mm_storeu_si128((__m128i *)tmpStore, sumDIXTmp);
  *sumDIX = tmpStore[0] + tmpStore[1] + tmpStore[2] + tmpStore[3] + tmpStore[4] + tmpStore[5];
  _mm_storeu_si128((__m128i *)tmpStore, sumDIYTmp);
  *sumDIY = tmpStore[0] + tmpStore[1] + tmpStore[2] + tmpStore[3] + tmpStore[4] + tmpStore[5];
  _mm_storeu_si128((__m128i *)tmpStore, sumSignGyGxTmp);
  *sumSignGY_GX = tmpStore[0] + tmpStore[1] + tmpStore[2] + tmpStore[3] + tmpStore[4] + tmpStore[5];
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
  __m128i mmShift1 = _mm_cvtsi32_si128( shift1 );
  assert((widthInside & 3) == 0);

  if ( ( widthInside & 7 ) == 0 )
  {
    for (int y = 0; y < heightInside; y++)
    {
      int x = 0;
      for ( ; x < widthInside; x += 8 )
      {
        __m128i mmPixTop    = _mm_sra_epi16( _mm_loadu_si128( ( __m128i* ) ( srcTmp + x - srcStride ) ), mmShift1 );
        __m128i mmPixBottom = _mm_sra_epi16( _mm_loadu_si128( ( __m128i* ) ( srcTmp + x + srcStride ) ), mmShift1 );
        __m128i mmPixLeft   = _mm_sra_epi16( _mm_loadu_si128( ( __m128i* ) ( srcTmp + x - 1 ) ), mmShift1 );
        __m128i mmPixRight  = _mm_sra_epi16( _mm_loadu_si128( ( __m128i* ) ( srcTmp + x + 1 ) ), mmShift1 );

        __m128i mmGradVer = _mm_sub_epi16( mmPixBottom, mmPixTop );
        __m128i mmGradHor = _mm_sub_epi16( mmPixRight, mmPixLeft );

        _mm_storeu_si128( ( __m128i * ) ( gradYTmp + x ), mmGradVer );
        _mm_storeu_si128( ( __m128i * ) ( gradXTmp + x ), mmGradHor );
      }

      gradXTmp += gradStride;
      gradYTmp += gradStride;
      srcTmp += srcStride;
    }
  }
  else
  {
    __m128i mmPixTop = _mm_sra_epi16( _mm_unpacklo_epi64( _mm_loadl_epi64( (__m128i*) ( srcTmp - srcStride ) ), _mm_loadl_epi64( (__m128i*) ( srcTmp ) ) ), mmShift1 );
    for ( int y = 0; y < heightInside; y += 2 )
    {
      __m128i mmPixBottom = _mm_sra_epi16( _mm_unpacklo_epi64( _mm_loadl_epi64( (__m128i*) ( srcTmp + srcStride ) ), _mm_loadl_epi64( (__m128i*) ( srcTmp + ( srcStride << 1 ) ) ) ), mmShift1 );
      __m128i mmPixLeft   = _mm_sra_epi16( _mm_unpacklo_epi64( _mm_loadl_epi64( (__m128i*) ( srcTmp - 1 ) ), _mm_loadl_epi64( (__m128i*) ( srcTmp - 1 + srcStride ) ) ), mmShift1 );
      __m128i mmPixRight  = _mm_sra_epi16( _mm_unpacklo_epi64( _mm_loadl_epi64( (__m128i*) ( srcTmp + 1 ) ), _mm_loadl_epi64( (__m128i*) ( srcTmp + 1 + srcStride ) ) ), mmShift1 );

      __m128i mmGradVer = _mm_sub_epi16( mmPixBottom, mmPixTop );
      __m128i mmGradHor = _mm_sub_epi16( mmPixRight, mmPixLeft );

      _mm_storel_epi64( (__m128i *) gradYTmp, mmGradVer );
      _mm_storel_epi64( (__m128i *) ( gradYTmp + gradStride ), _mm_unpackhi_epi64( mmGradVer, mmGradHor ) );
      _mm_storel_epi64( (__m128i *) gradXTmp, mmGradHor );
      _mm_storel_epi64( (__m128i *) ( gradXTmp + gradStride ), _mm_unpackhi_epi64( mmGradHor, mmGradVer ) );

      mmPixTop = mmPixBottom;
      gradXTmp += gradStride << 1;
      gradYTmp += gradStride << 1;
      srcTmp   += srcStride << 1;
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
    ::memcpy(gradXTmp - gradStride, gradXTmp, sizeof(Pel)*(width));
    ::memcpy(gradXTmp + heightInside*gradStride, gradXTmp + (heightInside - 1)*gradStride, sizeof(Pel)*(width));
    ::memcpy(gradYTmp - gradStride, gradYTmp, sizeof(Pel)*(width));
    ::memcpy(gradYTmp + heightInside*gradStride, gradYTmp + (heightInside - 1)*gradStride, sizeof(Pel)*(width));
  }
}

template<X86_VEXT vext>
void InterPredInterpolation::_initInterPredictionX86()
{
  xFpAddBDOFAvg4      = addBDOFAvg4_SSE<vext>;
  xFpBDOFGradFilter   = gradFilter_SSE<vext>;
  xFpCalcBDOFSums     = calcBDOFSums_SSE<vext>;
}
template void InterPredInterpolation::_initInterPredictionX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86
//! \}

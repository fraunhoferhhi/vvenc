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
/** \file     IntraPredX86.h
    \brief    SIMD for IntraPrediction
*/

#pragma once

#include "CommonDefX86.h"
#include "Rom.h"
#include "IntraPrediction.h"
#include "InterpolationFilter.h"

#include "Unit.h"

#if ENABLE_SIMD_OPT_INTRAPRED
#ifdef TARGET_SIMD_X86
//! \ingroup CommonLib
//! \{

namespace vvenc {

//#define USE_AVX2
template< X86_VEXT vext >
void IntraPredAngleChroma_SIMD(int16_t* pDst,const ptrdiff_t dstStride,int16_t* pBorder,int width,int height,int deltaPos,int intraPredAngle)
{
  int deltaInt;
  int deltaFract;
  int refMainIndex;
  __m128i voffset = _mm_set1_epi16(16);
  if( width >= 8 )
  {
    if( vext >= AVX2 )
    {
#ifdef USE_AVX2
      if (( width & 15 ) == 0 )
      {
       int deltaInt;
        int deltaFract;
        int refMainIndex;

        __m256i voffset = _mm256_set1_epi16(16);
        for (int k=0; k<height; k++) {

          deltaInt   = deltaPos >> 5;
          deltaFract = deltaPos & (32 - 1);

          __m256i vfract = _mm256_set1_epi16(deltaFract);
          __m256i v32minfract = _mm256_set1_epi16(32-deltaFract);
          // Do linear filtering
          for (int l=0; l<width; l+=16) {
            refMainIndex   = l+ deltaInt+1;
            __m256i vpred0 = _mm256_lddqu_si256((__m256i*)&pBorder[refMainIndex]);
            __m256i vpred1 = _mm256_lddqu_si256((__m256i*)&pBorder[refMainIndex+1]);
            vpred0 = _mm256_mullo_epi16(v32minfract, vpred0);
            vpred1 = _mm256_mullo_epi16(vfract, vpred1);
            __m256i vpred = _mm256_srli_epi16(_mm256_add_epi16(_mm256_add_epi16(vpred0, vpred1), voffset), 5);
            _mm256_storeu_si256((__m256i*)&pDst[l], vpred);
          }
          pDst+=dstStride;
          deltaPos += intraPredAngle;
        }
      }
      else // width==8
      {
        for (int k=0; k<height; k++)
        {
          deltaInt   = deltaPos >> 5;
          deltaFract = deltaPos & (32 - 1);

          __m128i vfract = _mm_set1_epi16(deltaFract);
          __m128i v32minfract = _mm_set1_epi16(32-deltaFract);
          // Do linear filtering
          for (int l=0; l<width; l+=8) {
            refMainIndex        = l+ deltaInt+1;
            __m128i vpred0 = _mm_lddqu_si128((__m128i*)&pBorder[refMainIndex]);
            __m128i vpred1 = _mm_lddqu_si128((__m128i*)&pBorder[refMainIndex+1]);
            vpred0 = _mm_mullo_epi16(v32minfract, vpred0);
            vpred1 = _mm_mullo_epi16(vfract, vpred1);
            __m128i vpred = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(vpred0, vpred1), voffset), 5);
            _mm_storeu_si128((__m128i*)&pDst[l], vpred);
          }
          deltaPos += intraPredAngle;

          pDst+=dstStride;
        }

      }
#endif
    }  //AVX2
    else
    {
      for (int k=0; k<height; k++) {
        deltaInt   = deltaPos >> 5;
        deltaFract = deltaPos & (32 - 1);

        __m128i vfract = _mm_set1_epi16(deltaFract);
        __m128i v32minfract = _mm_set1_epi16(32-deltaFract);
        // Do linear filtering
        for (int l=0; l<width; l+=8) {
          refMainIndex        = l+ deltaInt+1;
          __m128i vpred0 = _mm_lddqu_si128((__m128i*)&pBorder[refMainIndex]);
          __m128i vpred1 = _mm_lddqu_si128((__m128i*)&pBorder[refMainIndex+1]);
          vpred0 = _mm_mullo_epi16(v32minfract, vpred0);
          vpred1 = _mm_mullo_epi16(vfract, vpred1);
          __m128i vpred = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(vpred0, vpred1), voffset), 5);
          _mm_storeu_si128((__m128i*)&pDst[l], vpred);
        }
        deltaPos += intraPredAngle;
        pDst+=dstStride;
      }
    }
  }
  else if( width == 4 )
  {
    for (int k=0; k<height; k++) {
      deltaInt   = deltaPos >> 5;
      deltaFract = deltaPos & (32 - 1);

      __m128i vfract = _mm_set1_epi16(deltaFract);
      __m128i v32minfract = _mm_set1_epi16(32-deltaFract);
      // Do linear filtering
      refMainIndex        = deltaInt+1;
      __m128i vpred0 = _mm_lddqu_si128((__m128i*)&pBorder[refMainIndex]);
      __m128i vpred1 = _mm_lddqu_si128((__m128i*)&pBorder[refMainIndex+1]);
      vpred0 = _mm_mullo_epi16(v32minfract, vpred0);
      vpred1 = _mm_mullo_epi16(vfract, vpred1);
      __m128i vpred = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(vpred0, vpred1), voffset), 5);
      _mm_storel_epi64( ( __m128i * )(pDst ), vpred);
      deltaPos += intraPredAngle;
      pDst+=dstStride;
    }
  }
  else
  {
    for (int y = 0; y<height; y++)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & (32 - 1);

      // Do linear filtering
      const Pel* pRM = pBorder + deltaInt + 1;
      int lastRefMainPel = *pRM++;

      for( int x = 0; x < 2; pRM++, x++ )
      {
        int thisRefMainPel = *pRM;
        pDst[x + 0] = ( Pel ) ( ( ( 32 - deltaFract )*lastRefMainPel + deltaFract*thisRefMainPel + 16 ) >> 5 );
        lastRefMainPel = thisRefMainPel;
      }
      deltaPos += intraPredAngle;
      pDst += dstStride;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext >
void IntraPredAngleLumaCore_SIMD(int16_t* pDstBuf,const ptrdiff_t dstStride,int16_t* refMain,int width,int height,int deltaPos,int intraPredAngle,const TFilterCoeff *ff_unused,const bool useCubicFilter,const ClpRng& clpRng)
{
  
  int16_t* pDst;
  if( width >= 8 )
  {

    if( vext >= AVX2 )
    {
#ifdef USE_AVX2
      __m256i shflmask1= _mm256_set_epi8(0xd, 0xc, 0xb, 0xa,0x9, 0x8, 0x7, 0x6,   0xb, 0xa, 0x9, 0x8,0x7, 0x6, 0x5, 0x4,
          0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2,     0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
      __m256i offset = _mm256_set1_epi32( 32 );

      if (( width & 15 ) == 0 )
      {
        __m256i vbdmin,vbdmax;

        if (useCubicFilter)
        {
          vbdmin = _mm256_set1_epi16( clpRng.min );
          vbdmax = _mm256_set1_epi16( clpRng.max );
        }

        for (int y = 0; y<height; y++ )
        {
          int deltaInt   = deltaPos >> 5;
          int deltaFract = deltaPos & (32 - 1);
          
          const TFilterCoeff      intraSmoothingFilter[4] = {TFilterCoeff(16 - (deltaFract >> 1)), TFilterCoeff(32 - (deltaFract >> 1)), TFilterCoeff(16 + (deltaFract >> 1)), TFilterCoeff(deltaFract >> 1)};
          const TFilterCoeff *f = useCubicFilter ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

          int refMainIndex   = deltaInt + 1;
          pDst=&pDstBuf[y*dstStride];
//          __m128i tmp = _mm_loadl_epi64( ( __m128i const * )&ff[deltaFract<<2] );   //load 4 16 bit filter coeffs
          __m128i tmp = _mm_loadl_epi64( ( __m128i const * )f );   //load 4 16 bit filter coeffs
          tmp = _mm_shuffle_epi32(tmp,0x44);
          __m256i coeff = _mm256_broadcastsi128_si256(tmp);
          for( int x = 0; x < width; x+=16)
          {
            __m256i src0 = _mm256_lddqu_si256( ( const __m256i * )&refMain[refMainIndex - 1]  );//load 16 16 bit reference Pels   -1 0 1 2  3 4 5 6  7 8 9 10  11 12 13 14
            __m256i src2 = _mm256_castsi128_si256 (_mm_lddqu_si128( ( __m128i const * )&refMain[refMainIndex +4 - 1] ));
            __m256i src1 = _mm256_permute2f128_si256  (src0,src0,0x00);
            src2 = _mm256_permute2f128_si256  (src2,src2,0x00);
            src1 = _mm256_shuffle_epi8(src1,shflmask1);									// -1 0 1 2  0 1 2 3 1 2 3 4  2 3 4 5
            src2 = _mm256_shuffle_epi8(src2,shflmask1);									// 3 4 5 6  4 5 6 7  5 6 7 8 6 7 8 9

            src1 = _mm256_madd_epi16 (src1, coeff);
            src2 = _mm256_madd_epi16 (src2, coeff);

            __m256i  sum  = _mm256_hadd_epi32( src1, src2 );
            sum = _mm256_permute4x64_epi64(sum,0xD8);

            sum = _mm256_add_epi32( sum, offset );
            sum = _mm256_srai_epi32( sum, 6 );

            refMainIndex+=8;

            src1 = _mm256_permute2f128_si256  (src0,src0,0x1);
            src2 =  _mm256_inserti128_si256(src2, _mm_lddqu_si128( ( __m128i const * )&refMain[refMainIndex +4 - 1] ), 0x0);
            src1 = _mm256_permute2f128_si256  (src1,src1,0x00);
            src2 = _mm256_permute2f128_si256  (src2,src2,0x00);

            src1 = _mm256_shuffle_epi8(src1,shflmask1);									// -1 0 1 2  0 1 2 3 1 2 3 4  2 3 4 5
            src2 = _mm256_shuffle_epi8(src2,shflmask1);									// 3 4 5 6  4 5 6 7  5 6 7 8 6 7 8 9
            src1 = _mm256_madd_epi16 (src1, coeff);
            src2 = _mm256_madd_epi16 (src2, coeff);

            __m256i  sum1  = _mm256_hadd_epi32( src1, src2 );
            sum1 = _mm256_permute4x64_epi64(sum1,0xD8);

            sum1 = _mm256_add_epi32( sum1, offset );
            sum1 = _mm256_srai_epi32( sum1, 6 );
            src0 = _mm256_packs_epi32( sum, sum1 );

            src0 = _mm256_permute4x64_epi64(src0,0xD8);

            refMainIndex+=8;

            if (useCubicFilter)
              src0 = _mm256_min_epi16( vbdmax, _mm256_max_epi16( vbdmin, src0 ) );

            _mm256_storeu_si256( ( __m256i * )(pDst + x), src0);
          }
          deltaPos += intraPredAngle;
        }
      }
      else // width =8
      {
        //				printf("AVX2 Block %d \n",width);
        __m128i vbdmin,vbdmax;

        if (useCubicFilter)
        {
          vbdmin = _mm_set1_epi16( clpRng.min );
          vbdmax = _mm_set1_epi16( clpRng.max );
        }

        for (int y = 0; y<height; y++ )
        {
          int deltaInt   = deltaPos >> 5;
          int deltaFract = deltaPos & (32 - 1);

          const TFilterCoeff      intraSmoothingFilter[4] = {TFilterCoeff(16 - (deltaFract >> 1)), TFilterCoeff(32 - (deltaFract >> 1)), TFilterCoeff(16 + (deltaFract >> 1)), TFilterCoeff(deltaFract >> 1)};
          const TFilterCoeff *f = useCubicFilter ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

          int refMainIndex   = deltaInt + 1;
          pDst=&pDstBuf[y*dstStride];
//          __m128i tmp = _mm_loadl_epi64( ( __m128i const * )&ff[deltaFract<<2] );   //load 4 16 bit filter coeffs
          __m128i tmp = _mm_loadl_epi64( ( __m128i const * )f );   //load 4 16 bit filter coeffs
          tmp = _mm_shuffle_epi32(tmp,0x44);
          __m256i coeff = _mm256_broadcastsi128_si256(tmp);
          __m256i src0 = _mm256_lddqu_si256( ( const __m256i * )&refMain[refMainIndex - 1]  );//load 16 16 bit reference Pels   -1 0 1 2  3 4 5 6  7 8 9 10  11 12 13 14
          //					__m256i src2 =  _mm256_inserti128_si256(src2, _mm_lddqu_si128( ( __m128i const * )&refMain[refMainIndex +4 - 1] ), 0x0);
          __m256i src2 = _mm256_castsi128_si256 (_mm_lddqu_si128( ( __m128i const * )&refMain[refMainIndex +4 - 1] ));
          __m256i src1 = _mm256_permute2f128_si256  (src0,src0,0x00);
          src2 = _mm256_permute2f128_si256  (src2,src2,0x00);
          src1 = _mm256_shuffle_epi8(src1,shflmask1);									// -1 0 1 2  0 1 2 3 1 2 3 4  2 3 4 5
          src2 = _mm256_shuffle_epi8(src2,shflmask1);									// 3 4 5 6  4 5 6 7  5 6 7 8 6 7 8 9

          src1 = _mm256_madd_epi16 (src1, coeff);
          src2 = _mm256_madd_epi16 (src2, coeff);

          __m256i  sum  = _mm256_hadd_epi32( src1, src2 );
          sum = _mm256_permute4x64_epi64(sum,0xD8);

          sum = _mm256_add_epi32( sum, offset );
          sum = _mm256_srai_epi32( sum, 6 );
          src0 = _mm256_permute4x64_epi64( _mm256_packs_epi32( sum, sum ), 0x88  );
          __m128i dest128 = _mm256_castsi256_si128( src0);

          if (useCubicFilter)
            dest128 = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, dest128 ) );

          _mm_storeu_si128( ( __m128i * )(pDst), dest128);
          deltaPos += intraPredAngle;
        }
      }
#endif
    }
    else
    {
      __m128i shflmask1= _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2,   0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
      __m128i shflmask2= _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6,   0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
      __m128i vbdmin,vbdmax;

      __m128i offset = _mm_set1_epi32( 32 );
      if (useCubicFilter)
      {
        vbdmin = _mm_set1_epi16( clpRng.min );
        vbdmax = _mm_set1_epi16( clpRng.max );
      }
      for (int y = 0; y<height; y++ )
      {
        int deltaInt   = deltaPos >> 5;
        int deltaFract = deltaPos & (32 - 1);

        const TFilterCoeff      intraSmoothingFilter[4] = {TFilterCoeff(16 - (deltaFract >> 1)), TFilterCoeff(32 - (deltaFract >> 1)), TFilterCoeff(16 + (deltaFract >> 1)), TFilterCoeff(deltaFract >> 1)};
        const TFilterCoeff *f = useCubicFilter ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;

        int refMainIndex   = deltaInt + 1;
        pDst=&pDstBuf[y*dstStride];
        __m128i coeff = _mm_loadl_epi64( ( __m128i const * )f );   //load 4 16 bit filter coeffs
//        __m128i coeff = _mm_loadl_epi64( ( __m128i const * )&ff[deltaFract<<2] );   //load 4 16 bit filter coeffs
        coeff = _mm_shuffle_epi32(coeff,0x44);
        for( int x = 0; x < width; x+=8)
        {
          __m128i src0 = _mm_lddqu_si128( ( __m128i const * )&refMain[refMainIndex - 1] );   //load 8 16 bit reference Pels   -1 0 1 2 3 4 5 6
          __m128i src1 = _mm_shuffle_epi8(src0,shflmask1);									// -1 0 1 2  0 1 2 3
          __m128i src2 = _mm_shuffle_epi8(src0,shflmask2);									// 1 2 3 4  2 3 4 5
          src0 = _mm_madd_epi16( coeff,src1 );
          src1 = _mm_madd_epi16( coeff,src2 );
          __m128i sum  = _mm_hadd_epi32( src0, src1 );
          sum = _mm_add_epi32( sum, offset );
          sum = _mm_srai_epi32( sum, 6 );

          refMainIndex+=4;
          src0 = _mm_lddqu_si128( ( __m128i const * )&refMain[refMainIndex - 1] );   //load 8 16 bit reference Pels   -1 0 1 2 3 4 5 6
          src1 = _mm_shuffle_epi8(src0,shflmask1);						                    // -1 0 1 2  0 1 2 3
          src2 = _mm_shuffle_epi8(src0,shflmask2);

          // 1 2 3 4  2 3 4 5
          src0 = _mm_madd_epi16( coeff,src1 );
          src1 = _mm_madd_epi16( coeff,src2 );

          __m128i sum1  = _mm_hadd_epi32( src0, src1 );
          sum1 = _mm_add_epi32( sum1, offset );
          sum1 = _mm_srai_epi32( sum1, 6 );
          src0 = _mm_packs_epi32( sum, sum1 );

          refMainIndex+=4;
          if (useCubicFilter)
            src0 = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, src0 ) );

          _mm_storeu_si128( ( __m128i * )(pDst + x), src0);

        }
        deltaPos += intraPredAngle;
      }
    }
  }
  else if( width == 4 )
  {
    __m128i shflmask1= _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2,   0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
    __m128i shflmask2= _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6,   0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
    __m128i vbdmin,vbdmax;

    __m128i offset = _mm_set1_epi32( 32 );

    if (useCubicFilter)
    {
      vbdmin = _mm_set1_epi16( clpRng.min );
      vbdmax = _mm_set1_epi16( clpRng.max );
    }

    for (int y = 0; y<height; y++ )
    {
      int deltaInt   = deltaPos >> 5;
      int deltaFract = deltaPos & (32 - 1);
 
      const TFilterCoeff      intraSmoothingFilter[4] = {TFilterCoeff(16 - (deltaFract >> 1)), TFilterCoeff(32 - (deltaFract >> 1)), TFilterCoeff(16 + (deltaFract >> 1)), TFilterCoeff(deltaFract >> 1)};
      const TFilterCoeff *f = useCubicFilter ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;
   
      int refMainIndex   = deltaInt + 1;
      pDst=&pDstBuf[y*dstStride];
      __m128i coeff = _mm_loadl_epi64( ( __m128i const * )f);   //load 4 16 bit filter coeffs
//      __m128i coeff = _mm_loadl_epi64( ( __m128i const * )&ff[deltaFract<<2] );   //load 4 16 bit filter coeffs
      coeff = _mm_shuffle_epi32(coeff,0x44);
      {
        __m128i src0 = _mm_lddqu_si128( ( __m128i const * )&refMain[refMainIndex - 1] );   //load 8 16 bit reference Pels   -1 0 1 2 3 4 5 6
        __m128i src1 = _mm_shuffle_epi8(src0,shflmask1);									// -1 0 1 2  0 1 2 3
        __m128i src2 = _mm_shuffle_epi8(src0,shflmask2);									// 1 2 3 4  2 3 4 5
        src0 = _mm_madd_epi16( coeff,src1 );
        src1 = _mm_madd_epi16( coeff,src2 );
        __m128i sum  = _mm_hadd_epi32( src0, src1 );
        sum = _mm_add_epi32( sum, offset );
        sum = _mm_srai_epi32( sum, 6 );

        src0 = _mm_packs_epi32( sum, sum );

        refMainIndex+=4;

        if (useCubicFilter)
          src0 = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, src0 ) );

        _mm_storel_epi64( ( __m128i * )(pDst ), src0);

      }
      deltaPos += intraPredAngle;
    }
  }
  else
  {
    THROW( "Unsupported size in IntraPredAngleCore_SIMD" );
  }
#if USE_AVX2
  _mm256_zeroupper();
#endif
}
#define _mm_storeu_si32(p, a) (void)(*(int*)(p) = _mm_cvtsi128_si32((a)))
#define _mm_loadu_si64(p) _mm_loadl_epi64((__m128i const*)(p))
#define _mm_loadu_si32(p) _mm_cvtsi32_si128(*(unsigned int const*)(p))


template< X86_VEXT vext, int W >
void  IntraPredSampleFilter_SIMD(PelBuf& dstBuf, const CPelBuf& Src)
{
  const int iWidth  = dstBuf.width;
  const int iHeight = dstBuf.height;
  Pel* pDst         = dstBuf.buf;
  const ptrdiff_t  dstStride=dstBuf.stride;

  const Pel* ptrSrc = Src.buf;
  const ptrdiff_t  srcStride=Src.stride;

  const int scale = ((floorLog2(iWidth * iHeight) - 2) >> 2);
  CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 2");

#if USE_AVX2
  if( W > 8 )
  {
    __m256i tmplo,tmphi;
    __m256i w32 = _mm256_set_epi32(32,32,32,32,32,32,32,32);
    __m256i wl16,wl16start;

    wl16start = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,0,0,0,2,8,32);
    
    if (scale==1)
    {
      wl16start = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,1,2,4,8,16,32);
    }
    else if (scale==2)
    {
      wl16start = _mm256_set_epi16(0,0,0,0,1,1,2,2,4,4,8,8,16,16,32,32);
    }
    
    for (int y = 0; y < iHeight; y++)
    {
      int wT = 32 >> std::min(31, ((y << 1) >> scale));

      __m256i wt16 = _mm256_set_epi16(wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT);
      __m256i x16left = _mm256_broadcastw_epi16(_mm_loadu_si128 ((__m128i const *) (ptrSrc+((y+1)+srcStride))));
      if (wT)
      {
        for (int x = 0; x < iWidth; x+=16)
        {
          if (x==0)
          {
            wl16=wl16start;

            __m256i x16top = _mm256_loadu_si256((__m256i *) (ptrSrc+x+1)); // load top
            __m256i x16dst = _mm256_loadu_si256((const __m256i *) (pDst+y*dstStride+x)); // load dst

            tmphi = _mm256_sub_epi16(x16left,x16dst);
            tmplo = _mm256_mullo_epi16(tmphi,wl16);  //wL * left-val
            tmphi = _mm256_mulhi_epi16(tmphi,wl16);  //wL * left-val
            __m256i leftlo = _mm256_unpacklo_epi16(tmplo,tmphi);
            __m256i lefthi = _mm256_unpackhi_epi16(tmplo,tmphi);

            x16top = _mm256_sub_epi16(x16top,x16dst);
            tmplo = _mm256_mullo_epi16(x16top,wt16);    // wT*top-val
            tmphi = _mm256_mulhi_epi16(x16top,wt16);    // wT*top-val
            __m256i toplo = _mm256_unpacklo_epi16(tmplo,tmphi);
            __m256i tophi = _mm256_unpackhi_epi16(tmplo,tmphi);

            __m256i dstlo = _mm256_add_epi32(leftlo,toplo);
            __m256i dsthi = _mm256_add_epi32(lefthi,tophi);
            dstlo = _mm256_add_epi32(dstlo,w32);
            dsthi = _mm256_add_epi32(dsthi,w32);

            dstlo =  _mm256_srai_epi32(dstlo,6);
            dsthi =  _mm256_srai_epi32(dsthi,6);

            dstlo =  _mm256_packs_epi32(dstlo,dsthi);
            dstlo =  _mm256_permute4x64_epi64 ( dstlo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

            dstlo = _mm256_adds_epi16(dstlo,x16dst);
            _mm256_storeu_si256( ( __m256i * )(pDst+y*dstStride+x), dstlo );
          }
          else
          {
            __m256i x16top = _mm256_loadu_si256((__m256i *) (ptrSrc+x+1)); // load top
            __m256i x16dst = _mm256_loadu_si256((const __m256i *) (pDst+y*dstStride+x)); // load dst

            x16top = _mm256_sub_epi16(x16top,x16dst);
            tmplo = _mm256_mullo_epi16(x16top,wt16);    // wT*top-val
            tmphi = _mm256_mulhi_epi16(x16top,wt16);    // wT*top-val
            __m256i toplo = _mm256_unpacklo_epi16(tmplo,tmphi);
            __m256i tophi = _mm256_unpackhi_epi16(tmplo,tmphi);

            __m256i dstlo = _mm256_add_epi32(toplo,w32);
            __m256i dsthi = _mm256_add_epi32(tophi,w32);

            dstlo =  _mm256_srai_epi32(dstlo,6);
            dsthi =  _mm256_srai_epi32(dsthi,6);

            dstlo =  _mm256_packs_epi32(dstlo,dsthi);
            dstlo =  _mm256_permute4x64_epi64 ( dstlo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

            dstlo = _mm256_adds_epi16(dstlo,x16dst);
            _mm256_storeu_si256( ( __m256i * )(pDst+y*dstStride+x), dstlo );
          }
        }  // for x
      }
      else
      { // wT =0
        wl16=wl16start;

        __m256i x16dst = _mm256_loadu_si256((const __m256i *) (pDst+y*dstStride)); // load dst

        tmphi = _mm256_sub_epi16(x16left,x16dst);
        tmplo = _mm256_mullo_epi16(tmphi,wl16);  //wL * left-val
        tmphi = _mm256_mulhi_epi16(tmphi,wl16);  //wL * left-val
        __m256i leftlo = _mm256_unpacklo_epi16(tmplo,tmphi);
        __m256i lefthi = _mm256_unpackhi_epi16(tmplo,tmphi);

        __m256i dstlo = _mm256_add_epi32(leftlo,w32);
        __m256i dsthi = _mm256_add_epi32(lefthi,w32);

        dstlo =  _mm256_srai_epi32(dstlo,6);
        dsthi =  _mm256_srai_epi32(dsthi,6);

        dstlo =  _mm256_packs_epi32(dstlo,dsthi);
        dstlo =  _mm256_permute4x64_epi64 ( dstlo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

        dstlo = _mm256_adds_epi16(dstlo,x16dst);
        _mm256_storeu_si256( ( __m256i * )(pDst+y*dstStride), dstlo );
      }
    }
  }
  else
#endif
  {
    __m128i tmplo8,tmphi8;
    __m128i w32_8 = _mm_set_epi32(32,32,32,32);
    __m128i wl8start,wl8start2;
    CHECK(scale < 0 || scale > 2, "PDPC: scale < 0 || scale > 2");

    wl8start = _mm_set_epi16(0,0,0,0,0,2,8,32);
    wl8start2 = _mm_set_epi16(0,0,0,0,0,0,0,0);
    
    if (scale==1)
    {
      wl8start = _mm_set_epi16(0,0,1,2,4,8,16,32);
      wl8start2 = _mm_set_epi16(0,0,0,0,0,0,0,0);
    }
    else if (scale==2)
    {
      wl8start = _mm_set_epi16(4,4,8,8,16,16,32,32);
      wl8start2 = _mm_set_epi16(0,0,0,0,1,1,2,2);
    }

    __m128i wl8 = wl8start;
    for (int y = 0; y < iHeight; y++)
    {
      int wT = 32 >> std::min(31, ((y << 1) >> scale));

      __m128i wt8 = _mm_set_epi16(wT,wT,wT,wT,wT,wT,wT,wT);
      __m128i x8left;


      if ( W == 4 )
      {
        x8left = _mm_loadu_si64 ((__m128i const *) (ptrSrc+((y+1)+srcStride)));
      }
      else if ( W == 2 )
      {
        x8left = _mm_loadu_si32 ((__m128i const *) (ptrSrc+((y+1)+srcStride)));
      }
      else
      {
        x8left = _mm_loadu_si128 ((__m128i const *) (ptrSrc+((y+1)+srcStride)));
      }
      x8left =_mm_shufflelo_epi16(x8left,0);
      x8left =_mm_shuffle_epi32(x8left,0);


      if (wT)
      {
        for (int x = 0; x < iWidth; x+=8)
        {
          if (x>8)
          {
            __m128i x8top;
            __m128i x8dst;

            if ( W == 4 )
            {
              x8top =  _mm_loadu_si64((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si64((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            else if ( W == 2 )
            {
              x8top =  _mm_loadu_si32((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si32((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            else
            {
              x8top =  _mm_loadu_si128((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si128((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }

            tmphi8 = _mm_sub_epi16(x8top,x8dst);
            tmplo8 = _mm_mullo_epi16(tmphi8,wt8);    // wT*top-val
            tmphi8 = _mm_mulhi_epi16(tmphi8,wt8);    // wT*top-val
            __m128i toplo8 = _mm_unpacklo_epi16(tmplo8,tmphi8);
            __m128i tophi8 = _mm_unpackhi_epi16(tmplo8,tmphi8);

            __m128i dstlo8 = _mm_add_epi32(toplo8,w32_8);
            __m128i dsthi8 = _mm_add_epi32(tophi8,w32_8);

            dstlo8 =  _mm_srai_epi32(dstlo8,6);
            dsthi8 =  _mm_srai_epi32(dsthi8,6);

            dstlo8 =  _mm_packs_epi32(dstlo8,dsthi8);
            dstlo8 =  _mm_adds_epi16(dstlo8,x8dst);

            _mm_storeu_si128(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );

          }
          else // x<=8
          {
            
            if (x==0)
              wl8=wl8start;
            else if (x==8)
              wl8=wl8start2;

            __m128i x8top;
            __m128i x8dst;

            if ( W == 4 )
            {
              x8top =  _mm_loadu_si64((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si64((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            else if ( W == 2 )
            {
              x8top =  _mm_loadu_si32((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si32((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            else
            {
              x8top =  _mm_loadu_si128((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si128((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            tmphi8 = _mm_sub_epi16(x8left,x8dst);
            tmplo8 = _mm_mullo_epi16(tmphi8,wl8);  //wL * left-val
            tmphi8 = _mm_mulhi_epi16(tmphi8,wl8);  //wL * left-val
            __m128i leftlo8 = _mm_unpacklo_epi16(tmplo8,tmphi8);
            __m128i lefthi8 = _mm_unpackhi_epi16(tmplo8,tmphi8);

            tmphi8 = _mm_sub_epi16(x8top,x8dst);
            tmplo8 = _mm_mullo_epi16(tmphi8,wt8);    // wT*top-val
            tmphi8 = _mm_mulhi_epi16(tmphi8,wt8);    // wT*top-val
            __m128i toplo8 = _mm_unpacklo_epi16(tmplo8,tmphi8);
            __m128i tophi8 = _mm_unpackhi_epi16(tmplo8,tmphi8);

            __m128i dstlo8 = _mm_add_epi32(leftlo8,toplo8);
            __m128i dsthi8 = _mm_add_epi32(lefthi8,tophi8);
            dstlo8 = _mm_add_epi32(dstlo8,w32_8);
            dsthi8 = _mm_add_epi32(dsthi8,w32_8);

            dstlo8 =  _mm_srai_epi32(dstlo8,6);
            dsthi8 =  _mm_srai_epi32(dsthi8,6);

            dstlo8 =  _mm_packs_epi32(dstlo8,dsthi8);
            dstlo8 =  _mm_adds_epi16(dstlo8,x8dst);

            if (W>=8)
              _mm_storeu_si128(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );
            else if (W==4)
              _mm_storel_epi64(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );
            else if (W==2)
              _mm_storeu_si32(( __m128i * )(pDst+y*dstStride+x),(dstlo8) );
          }
        }
      }
      else //wT =0
      {
        for (int x = 0; x < std::min(iWidth,16); x+=8)
        {
          if (x==0)
            wl8=wl8start;
          else
            wl8=wl8start2;

          __m128i x8dst ;

          if ( W == 4 )
          {
            x8dst =  _mm_loadu_si64((const __m128i *) (pDst+y*dstStride+x)); // load dst
          }
          else if ( W == 2 )
          {
            x8dst =  _mm_loadu_si32((const __m128i *) (pDst+y*dstStride+x)); // load dst
          }
          else
          {
            x8dst =  _mm_loadu_si128((const __m128i *) (pDst+y*dstStride+x)); // load dst
          }
          tmphi8 = _mm_sub_epi16(x8left,x8dst);
          tmplo8 = _mm_mullo_epi16(tmphi8,wl8);  //wL * left-val
          tmphi8 = _mm_mulhi_epi16(tmphi8,wl8);  //wL * left-val
          __m128i leftlo8 = _mm_unpacklo_epi16(tmplo8,tmphi8);
          __m128i lefthi8 = _mm_unpackhi_epi16(tmplo8,tmphi8);

          __m128i dstlo8 = _mm_add_epi32(leftlo8,w32_8);
          __m128i dsthi8 = _mm_add_epi32(lefthi8,w32_8);

          dstlo8 =  _mm_srai_epi32(dstlo8,6);
          dsthi8 =  _mm_srai_epi32(dsthi8,6);

          dstlo8 =  _mm_packs_epi32(dstlo8,dsthi8);
          dstlo8 =  _mm_adds_epi16(dstlo8,x8dst);

          if (W>=8)
            _mm_storeu_si128(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );
          else if (W==4)
            _mm_storel_epi64(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );
          else if (W==2)
            _mm_storeu_si32(( __m128i * )(pDst+y*dstStride+x),(dstlo8) );
        }
      }
    }
  }


}

template< X86_VEXT vext >
void  IntraPredSampleFilter_SIMD(PelBuf& dstBuf, const CPelBuf& srcBuf)
{
  const int iWidth  = dstBuf.width;

  if (iWidth>8)
    IntraPredSampleFilter_SIMD<vext,16>(dstBuf, srcBuf);
  else  if (iWidth==8)
    IntraPredSampleFilter_SIMD<vext,8>(dstBuf, srcBuf);
  else  if (iWidth==4)
    IntraPredSampleFilter_SIMD<vext,4>(dstBuf, srcBuf);
  else
    IntraPredSampleFilter_SIMD<vext,2>(dstBuf, srcBuf);

#if USE_AVX2
  _mm256_zeroupper();
#endif
}


/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra coding).
 */
template< X86_VEXT vext>
void xPredIntraPlanar_SIMD( PelBuf& pDst, const CPelBuf& pSrc)
{

  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;
#if ISP_VVC
  const uint32_t log2W  = floorLog2(width);
  const uint32_t log2H  = floorLog2(height);
#else
  const uint32_t log2W  = floorLog2( width  < 2 ? 2 : width );
  const uint32_t log2H  = floorLog2( height < 2 ? 2 : height );
#endif
  const uint32_t offset = 1 << (log2W + log2H);
  const ptrdiff_t stride     = pDst.stride;
  Pel*       pred       = pDst.buf;

  const Pel* ptrSrc =pSrc.buf;

  int leftColumn,rightColumn;
  Pel tmp;
  int topRight = pSrc.at( width + 1, 0 );

  tmp=pSrc.at( height+1, 1 );
  __m128i bottomLeft16 = _mm_set_epi16(tmp,tmp,tmp,tmp,tmp,tmp,tmp,tmp);
  __m128i zero = _mm_xor_si128(bottomLeft16,bottomLeft16);
  __m128i eight = _mm_set_epi16(8,8,8,8,8,8,8,8);
  __m128i offset32 = _mm_set_epi32(offset,offset,offset,offset);

  const uint32_t finalShift = 1 + log2W + log2H;

  for( int y = 0; y < height; y++)
  {
    leftColumn=pSrc.at( y + 1, 1 );
    rightColumn = topRight - leftColumn;
    leftColumn  = leftColumn << log2W;
    __m128i leftColumn32 = _mm_set_epi32(leftColumn,leftColumn,leftColumn,leftColumn);
    __m128i rightcolumn16 = _mm_set_epi16(rightColumn,rightColumn,rightColumn,rightColumn,rightColumn,rightColumn,rightColumn,rightColumn);
    __m128i y16 = _mm_set_epi16(y+1,y+1,y+1,y+1,y+1,y+1,y+1,y+1);
    __m128i x16 = _mm_set_epi16(8,7,6,5,4,3,2,1);

    for( int x = 0; x < width; x+=8 )
    {
      //topRow[x] = pSrc.at( x + 1, 0 );
      __m128i topRow16 = _mm_loadu_si128 ((__m128i const *) (ptrSrc+(x+1)));
      //bottomRow[x] = bottomLeft - topRow[x];
      __m128i bottomRow16L = _mm_sub_epi16(bottomLeft16,topRow16);
      // (y+1)*bottomRow[x]
      __m128i  tmpH = _mm_mulhi_epi16(bottomRow16L,y16);
      __m128i tmpL = _mm_mullo_epi16(bottomRow16L,y16);
      bottomRow16L = _mm_unpacklo_epi16(tmpL,tmpH);
      __m128i bottomRow16H = _mm_unpackhi_epi16(tmpL,tmpH);

      // (topRow[x] topRow16H<< log2H)
      __m128i topRow32L = _mm_unpacklo_epi16(topRow16,zero);
      __m128i topRow32H = _mm_unpackhi_epi16(topRow16,zero);
      topRow32L = _mm_slli_epi32(topRow32L,log2H);
      topRow32H = _mm_slli_epi32(topRow32H,log2H);
      // vertPred    = (topRow[x] << log2H) + (y+1)*bottomRow[x];
      topRow32L = _mm_add_epi32(topRow32L,bottomRow16L);
      topRow32H = _mm_add_epi32(topRow32H,bottomRow16H);
      // horPred = leftColumn + (x+1)*rightColumn;
      tmpL = _mm_mullo_epi16(rightcolumn16,x16);
      tmpH = _mm_mulhi_epi16(rightcolumn16,x16);
      __m128i horpred32L = _mm_unpacklo_epi16(tmpL,tmpH);
      __m128i horpred32H = _mm_unpackhi_epi16(tmpL,tmpH);
      horpred32L = _mm_add_epi32(leftColumn32,horpred32L);
      horpred32H = _mm_add_epi32(leftColumn32,horpred32H);
      // pred[x]      = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
      horpred32L = _mm_slli_epi32(horpred32L,log2H);
      horpred32H = _mm_slli_epi32(horpred32H,log2H);
      topRow32L = _mm_slli_epi32(topRow32L,log2W);
      topRow32H = _mm_slli_epi32(topRow32H,log2W);
      horpred32L = _mm_add_epi32(horpred32L,topRow32L);
      horpred32H = _mm_add_epi32(horpred32H,topRow32H);
      horpred32L = _mm_add_epi32(horpred32L,offset32);
      horpred32H = _mm_add_epi32(horpred32H,offset32);
      horpred32L = _mm_srli_epi32(horpred32L,finalShift);
      horpred32H = _mm_srli_epi32(horpred32H,finalShift);

      tmpL = _mm_packs_epi32(horpred32L,horpred32H);
      if (width>=8)
        _mm_storeu_si128(( __m128i * )(pred+y*stride+x), (tmpL) );
      else if (width==4)
        _mm_storel_epi64(( __m128i * )(pred+y*stride+x), (tmpL) );
      else if (width==2)
        _mm_storeu_si32(( __m128i * )(pred+y*stride+x),(tmpL) );
      else
        pred[y*stride+x]=(Pel)_mm_extract_epi16 (tmpL,0);

      x16 = _mm_add_epi16(x16,eight);
    }
  }
}

template< X86_VEXT vext>
void GetLumaRecPixel420SIMD (const int width,const int height, const Pel* pRecSrc0,const ptrdiff_t iRecStride,Pel* pDst0,const ptrdiff_t iDstStride)
{
#ifdef USE_AVX2
  if( ( width & 15 ) == 0 )    // width>=16
  {
    __m256i vzero = _mm256_set1_epi8(0);
    __m256i vfour = _mm256_set1_epi32(4);
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x += 16 )
      {
        int x2=x<<1;
        __m256i vsrc_l = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2-1]);      // -1 0 1 2 3 4 5 6
        __m256i vsrc = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2]);          // 0 1 2 3 4 5 6 7

        __m256i vsrc01 = _mm256_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
        __m256i vsrc0 = _mm256_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
        __m256i vsrc10 = _mm256_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
        vsrc10 = _mm256_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm256_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm256_add_epi32(vsrc0,vsrc10);
        __m256i vdst0 = _mm256_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile fehlt noch

        vsrc_l = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2 +15]);      // 7 8 9 10 11 12 13 14
        vsrc = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2 + 16 ]);          // 8 9 10 11 12 13 14 15

        x2+= (int)iRecStride;

        vsrc01 = _mm256_blend_epi16(vzero,vsrc_l,0x55);
        vsrc0 = _mm256_blend_epi16(vzero,vsrc,0x55);
        vsrc10 = _mm256_blend_epi16(vzero,vsrc,0xAA);
        vsrc10 = _mm256_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm256_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm256_add_epi32(vsrc0,vsrc10);
        __m256i vdst1 = _mm256_add_epi32(vsrc0,vsrc01);   // dst 4 5 6 7 32 Bit, untere Zeile fehlt noch

        // jetzt die nächste Zeile dazu
        vsrc_l = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2-1]);      // -1 0 1 2 3 4 5 6
        vsrc = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2]);          // 0 1 2 3 4 5 6 7

        vsrc01 = _mm256_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
        vsrc0 = _mm256_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
        vsrc10 = _mm256_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
        vsrc10 = _mm256_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm256_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm256_add_epi32(vsrc0,vsrc10);
        __m256i vdst01 = _mm256_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile

        vsrc_l = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2 + 15]);      // 7 8 9 10 11 12 13 14
        vsrc = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2 + 16 ]);          // 8 9 10 11 12 13 14 15

        vsrc01 = _mm256_blend_epi16(vzero,vsrc_l,0x55);
        vsrc0 = _mm256_blend_epi16(vzero,vsrc,0x55);
        vsrc10 = _mm256_blend_epi16(vzero,vsrc,0xAA);
        vsrc10 = _mm256_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm256_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm256_add_epi32(vsrc0,vsrc10);
        __m256i vdst11 = _mm256_add_epi32(vsrc0,vsrc01);   // dst 4 5 6 7 32 Bit, untere Zeile

        vdst0 = _mm256_add_epi32(vdst0,vdst01);
        vdst1 = _mm256_add_epi32(vdst1,vdst11);
        vdst0 =  _mm256_add_epi32(vdst0,vfour);
        vdst1 =  _mm256_add_epi32(vdst1,vfour);
        vdst0 = _mm256_srli_epi32(vdst0,3);
        vdst1 = _mm256_srli_epi32(vdst1,3);
        vdst0 = _mm256_packus_epi32 (vdst0,vdst1);   // 16 bit
        vdst0 = _mm256_permute4x64_epi64(vdst0,0xd8);

        _mm256_storeu_si256((__m256i*)&pDst0[x], vdst0);
        //        _mm_storeu_si128((__m128i*)&pDstTmp[x], vdst0);
      }
      pDst0 += iDstStride;
      pRecSrc0 += (iRecStride<<1);
    }
  }
  else
#endif
    if( ( width & 7 ) == 0 )    // width>=8
    {
      __m128i vzero = _mm_set1_epi8(0);
      __m128i vfour = _mm_set1_epi32(4);


      for( int y = 0; y < height; y++ )
      {

        for( int x = 0; x < width; x += 8 )
        {
          int x2=x<<1;
          __m128i vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[x2-1]);      // -1 0 1 2 3 4 5 6
          __m128i vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[x2]);          // 0 1 2 3 4 5 6 7

          __m128i vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
          __m128i vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
          __m128i vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
          vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
          vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

          vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
          __m128i vdst0 = _mm_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile fehlt noch

          vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[x2 +7]);      // 7 8 9 10 11 12 13 14
          vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[x2 + 8 ]);          // 8 9 10 11 12 13 14 15

          x2+=(int)iRecStride;

          vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);
          vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);
          vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);
          vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
          vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

          vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
          __m128i vdst1 = _mm_add_epi32(vsrc0,vsrc01);   // dst 4 5 6 7 32 Bit, untere Zeile fehlt noch

          // jetzt die nächste Zeile dazu
          vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[x2-1]);      // -1 0 1 2 3 4 5 6
          vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[x2]);          // 0 1 2 3 4 5 6 7

          vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
          vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
          vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
          vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
          vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

          vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
          __m128i vdst01 = _mm_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile

          vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[x2 + 7]);      // 7 8 9 10 11 12 13 14
          vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[x2 + 8 ]);          // 8 9 10 11 12 13 14 15

          vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);
          vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);
          vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);
          vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
          vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

          vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
          __m128i vdst11 = _mm_add_epi32(vsrc0,vsrc01);   // dst 4 5 6 7 32 Bit, untere Zeile

          vdst0 = _mm_add_epi32(vdst0,vdst01);
          vdst1 = _mm_add_epi32(vdst1,vdst11);
          vdst0 =  _mm_add_epi32(vdst0,vfour);
          vdst1 =  _mm_add_epi32(vdst1,vfour);
          vdst0 = _mm_srli_epi32(vdst0,3);
          vdst1 = _mm_srli_epi32(vdst1,3);
          vdst0 = _mm_packus_epi32 (vdst0,vdst1);   // 16 bit__m256i wl16start;

          _mm_storeu_si128((__m128i*)&pDst0[x], vdst0);
          //        _mm_storeu_si128((__m128i*)&pDstTmp[x], vdst0);
        }
        pDst0 += iDstStride;
        pRecSrc0 += (iRecStride<<1);
      }
    }
    else     // width<=4
    {
      __m128i vzero = _mm_set1_epi8(0);
      __m128i vfour = _mm_set1_epi32(4);

      for( int y = 0; y < height; y++ )
      {
        __m128i vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[-1]);      // -1 0 1 2 3 4 5 6
        __m128i vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[0]);          // 0 1 2 3 4 5 6 7

        __m128i vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
        __m128i vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
        __m128i vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
        vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
        __m128i vdst0 = _mm_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile fehlt noch

        // jetzt die nächste Zeile dazu
        vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[iRecStride-1]);      // -1 0 1 2 3 4 5 6
        vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[iRecStride]);          // 0 1 2 3 4 5 6_mm_storeu_si32 7

        vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
        vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
        vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
        vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
        __m128i vdst01 = _mm_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile


        vdst0 = _mm_add_epi32(vdst0,vdst01);
        vdst0 =  _mm_add_epi32(vdst0,vfour);
        vdst0 = _mm_srli_epi32(vdst0,3);
        vdst0 = _mm_packus_epi32 (vdst0,vdst0);   // 16 bit

        if (width==4)
          _mm_storel_epi64(( __m128i * )&pDst0[0], (vdst0) );
        else if (width==2)
          _mm_storeu_si32(( __m128i * )&pDst0[0], (vdst0) );
        else
        {
          int tmp = _mm_cvtsi128_si32(vdst0);
          pDst0[0] = (Pel) tmp;
        }

        pDst0 += iDstStride;
        pRecSrc0 += (iRecStride<<1);
      }
    }
}


template<X86_VEXT vext, int W >
void IntraAnglePDPC_SIMD(Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height,int scale,int invAngle)
{  

  if (W>=16)
  {
#ifdef USE_AVX2
    ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE,short ref[16]);
    VALGRIND_MEMCLEAR( ref, sizeof( ref ) );

    //  Pel dummy[16];
    //  Pel* pdum=&dummy[0];
    //  int scaledum=scale;

    __m256i wl16; 
    if (scale==0)
    {
      wl16 = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,0,0,0,2,8,32);
      scale=3;
    }
    else if (scale==1)
    {
      wl16 = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,1,2,4,8,16,32);
      scale=6;
    }
    else
    {
      wl16 = _mm256_set_epi16(0,0,0,0,1,1,2,2,4,4,8,8,16,16,32,32);
      scale=12;
    }


    __m256i v32 = _mm256_set1_epi32(32);
    for (int y = 0; y<height; y++, pDsty += dstStride)
    {
      int       invAngleSum = 256;
      for (int x = 0; x < scale; x++)
      {
        invAngleSum += invAngle;
        ref[x]=refSide[y + (invAngleSum >> 9) + 1];
      }
      __m256i xleft= _mm256_load_si256((__m256i*)&ref[0]);
      __m256i xdst= _mm256_loadu_si256((__m256i*)pDsty);
      __m256i xdstlo=_mm256_sub_epi16 (xleft,xdst);
      __m256i tmplo = _mm256_mullo_epi16(xdstlo,wl16);
      __m256i tmphi = _mm256_mulhi_epi16(xdstlo,wl16);
      xdstlo = _mm256_unpacklo_epi16(tmplo,tmphi);  //low
      tmphi = _mm256_unpackhi_epi16(tmplo,tmphi);  // high

      tmplo = _mm256_add_epi32(xdstlo,v32);
      tmphi = _mm256_add_epi32(tmphi,v32);
      tmplo = _mm256_srai_epi32(tmplo,6);
      tmphi = _mm256_srai_epi32(tmphi,6);

      tmplo =  _mm256_packs_epi32(tmplo,tmphi);
      tmplo =  _mm256_permute4x64_epi64 ( tmplo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );
      xdst = _mm256_add_epi16(tmplo,xdst);
      _mm256_storeu_si256( ( __m256i * )(pDsty), xdst );

    }
#else
    for (int y = 0; y<height; y++, pDsty += dstStride)
    {
      int       invAngleSum = 256;
      for (int x = 0; x < std::min(3 << scale, width); x++)
      {
        invAngleSum += invAngle;
        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[y + (invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }
#endif
  }
  else
  {
    ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE,short ref[8]);
    VALGRIND_MEMCLEAR( ref, sizeof( ref ) );

    __m128i wl16; 
    if (scale==0)
    {
      wl16 = _mm_set_epi16(0,0,0,0,0,2,8,32);
      scale=3;
    }
    else if (scale==1)
    {
      wl16 = _mm_set_epi16(0,0,1,2,4,8,16,32);
      scale=6;
    }
    else
    {
      wl16 = _mm_set_epi16(4,4,8,8,16,16,32,32);
      scale=8;
    }

    int xlim=std::min(scale, width);

    __m128i v32 = _mm_set1_epi32(32);
    for (int y = 0; y<height; y++, pDsty += dstStride)
    {
      int       invAngleSum = 256;
      for (int x = 0; x < xlim; x++)
      {
        invAngleSum += invAngle;
        ref[x]=refSide[y + (invAngleSum >> 9) + 1];
      }

      __m128i xleft;
      __m128i xdst;
      if (W==8)
      {
        xleft= _mm_load_si128((__m128i*)&ref[0]);
        xdst= _mm_loadu_si128((__m128i*)pDsty);
      }
      else
      {
        xleft= _mm_load_si128((__m128i*)&ref[0]);
        xdst= _mm_loadu_si64((__m128i*)pDsty);
      }
      __m128i xdstlo=_mm_sub_epi16 (xleft,xdst);
      __m128i tmplo = _mm_mullo_epi16(xdstlo,wl16);
      __m128i tmphi = _mm_mulhi_epi16(xdstlo,wl16);
      xdstlo = _mm_unpacklo_epi16(tmplo,tmphi);  //low
      tmphi = _mm_unpackhi_epi16(tmplo,tmphi);  // high

      tmplo = _mm_add_epi32(xdstlo,v32);
      tmphi = _mm_add_epi32(tmphi,v32);
      tmplo = _mm_srai_epi32(tmplo,6);
      tmphi = _mm_srai_epi32(tmphi,6);

      tmplo =  _mm_packs_epi32(tmplo,tmphi);
      xdst = _mm_add_epi16(tmplo,xdst);
      if (W==8)
        _mm_storeu_si128( ( __m128i * )(pDsty), xdst );
      else if (W==4)
        _mm_storel_epi64( ( __m128i * )(pDsty), xdst );
      else
      {
        EXIT("wrong blocksize");
      }
    }
  }
}

template<X86_VEXT vext >
void IntraAnglePDPC_SIMD(Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height,int scale,int invAngle)
{  
  if (width>=16)
    IntraAnglePDPC_SIMD<vext,16>(pDsty,dstStride,refSide,width,height,scale,invAngle);
  else if (width==8)
    IntraAnglePDPC_SIMD<vext,8>(pDsty,dstStride,refSide,width,height,scale,invAngle);
  else if (width==4)
    IntraAnglePDPC_SIMD<vext,4>(pDsty,dstStride,refSide,width,height,scale,invAngle);
  else
    for (int y = 0; y<height; y++, pDsty += dstStride)
    {
      int       invAngleSum = 256;
      for (int x = 0; x < 2; x++)
      {
        invAngleSum += invAngle;
        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[y + (invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }
#if USE_AVX2
  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void IntraHorVerPDPC_SIMD(Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height,int scale,const Pel* refMain, const ClpRng& clpRng)
{
   const Pel topLeft = refMain[0];

   if (width>=16)
   {
#ifdef USE_AVX2
     __m256i v32 = _mm256_set1_epi32(32);
     __m256i vbdmin   = _mm256_set1_epi16( clpRng.min );
     __m256i vbdmax   = _mm256_set1_epi16( clpRng.max );

     __m256i wl16;
     if (scale==0)
     {
       wl16 = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,0,0,0,2,8,32);
     }
     else if (scale==1)
     {
       wl16 = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,1,2,4,8,16,32);
     }
     else
     {
       wl16 = _mm256_set_epi16(0,0,0,0,1,1,2,2,4,4,8,8,16,16,32,32);
     }
     __m256i xtopLeft = _mm256_set_epi16(topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft);

     for (int y = 0; y<height; y++, pDsty += dstStride)
     {
       // first column
       const Pel left    = refSide[1 + y];
       __m256i xleft= _mm256_set_epi16(left,left,left,left,left,left,left,left,left,left,left,left,left,left,left,left);

       __m256i xdst= _mm256_loadu_si256((__m256i*)&refMain[1]);
       xleft = _mm256_sub_epi16(xleft,xtopLeft);

       __m256i tmplo = _mm256_mullo_epi16(xleft,wl16);
       __m256i tmphi = _mm256_mulhi_epi16(xleft,wl16);
       xleft = _mm256_unpacklo_epi16(tmplo,tmphi);  //low
       tmphi = _mm256_unpackhi_epi16(tmplo,tmphi);  // high

       tmplo = _mm256_add_epi32(xleft,v32);
       tmphi = _mm256_add_epi32(tmphi,v32);
       tmplo = _mm256_srai_epi32(tmplo,6);
       tmphi = _mm256_srai_epi32(tmphi,6);

       tmplo =  _mm256_packs_epi32(tmplo,tmphi);
       tmplo =  _mm256_permute4x64_epi64 ( tmplo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

       xdst = _mm256_adds_epi16(tmplo,xdst);
       xdst = _mm256_min_epi16( vbdmax, _mm256_max_epi16( vbdmin, xdst ) );
       _mm256_storeu_si256( ( __m256i * )(pDsty), xdst );

       // rest memcpy
       for (int x = 16; x < width; x+=16)
       {
         __m256i xdst= _mm256_loadu_si256((__m256i*)&refMain[x+1]);
         _mm256_storeu_si256( ( __m256i * )(&pDsty[x]), xdst );
       }
     }
#else
     for( int y = 0; y < height; y++ )
     {
       memcpy(pDsty,&refMain[1],width*sizeof(Pel));
       const Pel left    = refSide[1 + y];
       for (int x = 0; x < std::min(3 << scale, width); x++)
       {
         const int wL  = 32 >> (2 * x >> scale);
         const Pel val = pDsty[x];
         pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
       }
       pDsty += dstStride;
     }
#endif
   }
   else      //width <= 8
   {
     __m128i vbdmin   = _mm_set1_epi16( clpRng.min );
     __m128i vbdmax   = _mm_set1_epi16( clpRng.max );
     __m128i wl16;

     if (scale==0)
     {
       wl16 = _mm_set_epi16(0,0,0,0,0,2,8,32);
     }
     else if (scale==1)
     {
       wl16 = _mm_set_epi16(0,0,1,2,4,8,16,32);
     }
     else
     {
       wl16 = _mm_set_epi16(4,4,8,8,16,16,32,32);
     }

     __m128i v32 = _mm_set1_epi32(32);
     __m128i xtopLeft = _mm_set_epi16(topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft);

     for (int y = 0; y<height; y++, pDsty += dstStride)
     {
       // first column
        const Pel left    = refSide[1 + y];
        __m128i xleft= _mm_set_epi16(left,left,left,left,left,left,left,left);

        __m128i xdst= _mm_loadu_si128((__m128i*)&refMain[1]);
        xleft = _mm_sub_epi16(xleft,xtopLeft);

        __m128i tmplo = _mm_mullo_epi16(xleft,wl16);
        __m128i tmphi = _mm_mulhi_epi16(xleft,wl16);
        xleft = _mm_unpacklo_epi16(tmplo,tmphi);  //low
        tmphi = _mm_unpackhi_epi16(tmplo,tmphi);  // high

        tmplo = _mm_add_epi32(xleft,v32);
        tmphi = _mm_add_epi32(tmphi,v32);
        tmplo = _mm_srai_epi32(tmplo,6);
        tmphi = _mm_srai_epi32(tmphi,6);

        tmplo =  _mm_packs_epi32(tmplo,tmphi);

        xdst = _mm_adds_epi16(tmplo,xdst);
        xdst = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, xdst ) );

        if (width==8)
           _mm_storeu_si128( ( __m128i * )(pDsty), xdst );
         else if (width==4)
           _mm_storel_epi64( ( __m128i * )(pDsty), xdst );
         else
         {
           _mm_storeu_si32( ( __m128i * )(pDsty), xdst );
         }
     }
   }
#if USE_AVX2
  _mm256_zeroupper();
#endif
}


template<X86_VEXT vext>
void IntraPrediction::_initIntraPredictionX86()
{
  IntraPredAngleLuma    = IntraPredAngleLumaCore_SIMD<vext>;
  IntraPredAngleChroma  = IntraPredAngleChroma_SIMD<vext>;
  IntraAnglePDPC        = IntraAnglePDPC_SIMD<vext>;
  IntraHorVerPDPC       = IntraHorVerPDPC_SIMD<vext>;
  IntraPredSampleFilter = IntraPredSampleFilter_SIMD<vext>;
  xPredIntraPlanar      = xPredIntraPlanar_SIMD<vext>;
}
template void IntraPrediction::_initIntraPredictionX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86
#endif
//! \}

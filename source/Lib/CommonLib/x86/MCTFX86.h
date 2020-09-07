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
/**
 * \file
 * \brief Implementation of AffineGradientSearch class
 */
//#define USE_AVX2
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "CommonDefX86.h"

#include "MCTF.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_MCTF

#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

namespace vvenc {

#if _MSC_VER <= 1900 && !defined( _mm256_extract_epi32 )
inline uint32_t _mm256_extract_epi32(__m256i vec, const int i )
{   
  __m128i indx = _mm_cvtsi32_si128(i);
  __m256i val  = _mm256_permutevar8x32_epi32(vec, _mm256_castsi128_si256(indx));
  return         _mm_cvtsi128_si32(_mm256_castsi256_si128(val));
}
#endif

template<X86_VEXT vext>
int motionErrorLumaInt_SIMD( const Pel* origOrigin, const ptrdiff_t origStride, const Pel* buffOrigin, const ptrdiff_t buffStride, const int bs, const int x, const int y, const int dx, const int dy, const int besterror )
{
  int error = 0;

  CHECK( bs & 7, "SIMD blockSize needs to be a multiple of 8" );

#if USE_AVX2
  if( ( bs & 15 ) == 0 && vext >= AVX2 )
  {
    for( int y1 = 0; y1 < bs; y1 += 2 )
    {
      const Pel* origRowStart   = origOrigin + ( y + y1 + 0      )*origStride +   x;
      const Pel* bufferRowStart = buffOrigin + ( y + y1 + 0 + dy )*buffStride + ( x + dx );

      __m256i vsum = _mm256_setzero_si256();

      for( int x1 = 0; x1 < bs; x1 += 16 )
      {
        __m256i vorg1 = _mm256_loadu_si256( ( const __m256i* ) &origRowStart[x1] );
        __m256i vorg2 = _mm256_loadu_si256( ( const __m256i* ) &origRowStart[x1+origStride] );
        __m256i vbuf1 = _mm256_loadu_si256( ( const __m256i* ) &bufferRowStart[x1] );
        __m256i vbuf2 = _mm256_loadu_si256( ( const __m256i* ) &bufferRowStart[x1+buffStride] );

        __m256i vsum1 = _mm256_sub_epi16( vorg1, vbuf1 );
        __m256i vsum2 = _mm256_sub_epi16( vorg2, vbuf2 );

        __m256i vtmp1 = _mm256_madd_epi16( vsum1, vsum1 );
        __m256i vtmp2 = _mm256_madd_epi16( vsum2, vsum2 );

        vtmp1         = _mm256_hadd_epi32( vtmp1, vtmp2 );
        vsum          = _mm256_add_epi32 ( vsum,  vtmp1 );

        //int diff = origRowStart[x1] - bufferRowStart[x1];
        //error += diff * diff;
        //diff = origRowStart[x1 + 1] - bufferRowStart[x1 + 1];
        //error += diff * diff;
      }
      
      vsum = _mm256_hadd_epi32     ( vsum, vsum );
      __m128i
      xtmp = _mm256_extractf128_si256( vsum, 1 );

      error += _mm256_extract_epi32( vsum, 0 );
      error += _mm256_extract_epi32( vsum, 1 );
      error += _mm_extract_epi32   ( xtmp, 0 );
      error += _mm_extract_epi32   ( xtmp, 1 );

      if( error > besterror )
      {
        return error;
      }
    }

    return error;
  }
#endif
  for( int y1 = 0; y1 < bs; y1 += 2 )
  {
    const Pel* origRowStart   = origOrigin + ( y + y1 + 0      )*origStride +   x;
    const Pel* bufferRowStart = buffOrigin + ( y + y1 + 0 + dy )*buffStride + ( x + dx );

    __m128i xsum = _mm_setzero_si128();

    for( int x1 = 0; x1 < bs; x1 += 8 )
    {
      __m128i xorg1 = _mm_loadu_si128( ( const __m128i* ) &origRowStart[x1] );
      __m128i xorg2 = _mm_loadu_si128( ( const __m128i* ) &origRowStart[x1+origStride] );
      __m128i xbuf1 = _mm_loadu_si128( ( const __m128i* ) &bufferRowStart[x1] );
      __m128i xbuf2 = _mm_loadu_si128( ( const __m128i* ) &bufferRowStart[x1+buffStride] );

      __m128i xsum1 = _mm_sub_epi16( xorg1, xbuf1 );
      __m128i xsum2 = _mm_sub_epi16( xorg2, xbuf2 );

      __m128i xtmp1 = _mm_madd_epi16( xsum1, xsum1 );
      __m128i xtmp2 = _mm_madd_epi16( xsum2, xsum2 );

      xtmp1         = _mm_hadd_epi32( xtmp1, xtmp2 );
      xsum          = _mm_add_epi32 ( xsum,  xtmp1 );

      //int diff = origRowStart[x1] - bufferRowStart[x1];
      //error += diff * diff;
      //diff = origRowStart[x1 + 1] - bufferRowStart[x1 + 1];
      //error += diff * diff;
    }
    
    xsum   = _mm_hadd_epi32   ( xsum, xsum );
    error += _mm_extract_epi32( xsum, 0 );
    error += _mm_extract_epi32( xsum, 1 );

    if( error > besterror )
    {
      return error;
    }
  }

  return error;
}

template<X86_VEXT vext>
int motionErrorLumaFrac_SIMD( const Pel* origOrigin, const ptrdiff_t origStride, const Pel* buffOrigin, const ptrdiff_t buffStride, const int bs, const int x, const int y, const int dx, const int dy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror )
{
  int error = 0;
  int tempArray[64 + 8][64];
  int base;
  
  CHECK( bs & 7, "SIMD blockSize needs to be a multiple of 8" );

  __m128i xfilt1 = _mm_loadu_si128( ( const __m128i * ) xFilter );

  for( int y1 = 1; y1 < bs + 7; y1++ )
  {
    const int yOffset    = y + y1 + ( dy >> 4 ) - 3;
    const Pel* sourceRow = buffOrigin + ( yOffset ) *buffStride + 0;

    for( int x1 = 0; x1 < bs; x1 += 4 )
    {
      base                = x + x1 + ( dx >> 4 ) - 3;
      const Pel* rowStart = sourceRow + base;

      __m128i vsrc0 = _mm_loadu_si128( ( const __m128i * ) &rowStart[0] );
      __m128i vsrc1 = _mm_loadu_si128( ( const __m128i * ) &rowStart[1] );
      __m128i vsrc2 = _mm_loadu_si128( ( const __m128i * ) &rowStart[2] );
      __m128i vsrc3 = _mm_loadu_si128( ( const __m128i * ) &rowStart[3] );

      vsrc0 = _mm_madd_epi16( vsrc0, xfilt1 );
      vsrc1 = _mm_madd_epi16( vsrc1, xfilt1 );
      vsrc2 = _mm_madd_epi16( vsrc2, xfilt1 );
      vsrc3 = _mm_madd_epi16( vsrc3, xfilt1 );

      vsrc0 = _mm_hadd_epi32( vsrc0, vsrc1 );
      vsrc2 = _mm_hadd_epi32( vsrc2, vsrc3 );

      vsrc0 = _mm_hadd_epi32( vsrc0, vsrc2 );

      _mm_storeu_si128( ( __m128i * ) &tempArray[y1][x1], vsrc0 );
      
      //sum  = 0;
      //sum += xFilter[1] * rowStart[1];
      //sum += xFilter[2] * rowStart[2];
      //sum += xFilter[3] * rowStart[3];
      //sum += xFilter[4] * rowStart[4];
      //sum += xFilter[5] * rowStart[5];
      //sum += xFilter[6] * rowStart[6];
      //
      //tempArray[y1][x1] = sum;
    }
  }

  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

#if USE_AVX2
  if( ( bs & 7 ) == 0 && vext >= AVX2 )
  {
    __m256i vfilt1 = _mm256_set1_epi32( yFilter[1] );
    __m256i vfilt2 = _mm256_set1_epi32( yFilter[2] );
    __m256i vfilt3 = _mm256_set1_epi32( yFilter[3] );
    __m256i vfilt4 = _mm256_set1_epi32( yFilter[4] );
    __m256i vfilt5 = _mm256_set1_epi32( yFilter[5] );
    __m256i vfilt6 = _mm256_set1_epi32( yFilter[6] );

    __m256i vmax   = _mm256_set1_epi32( maxSampleValue );
    __m256i vmin   = _mm256_setzero_si256();

    for( int y1 = 0; y1 < bs; y1++ )
    {
      const Pel* origRow = origOrigin + ( y + y1 )*origStride + 0;
      for( int x1 = 0; x1 < bs; x1 += 8 )
      {
        __m256i vsum = _mm256_set1_epi32( 1 << 11 );

        vsum = _mm256_add_epi32( vsum, _mm256_mullo_epi32( vfilt1, _mm256_loadu_si256( ( const __m256i * ) &tempArray[y1 + 1][x1] ) ) );
        vsum = _mm256_add_epi32( vsum, _mm256_mullo_epi32( vfilt2, _mm256_loadu_si256( ( const __m256i * ) &tempArray[y1 + 2][x1] ) ) );
        vsum = _mm256_add_epi32( vsum, _mm256_mullo_epi32( vfilt3, _mm256_loadu_si256( ( const __m256i * ) &tempArray[y1 + 3][x1] ) ) );
        vsum = _mm256_add_epi32( vsum, _mm256_mullo_epi32( vfilt4, _mm256_loadu_si256( ( const __m256i * ) &tempArray[y1 + 4][x1] ) ) );
        vsum = _mm256_add_epi32( vsum, _mm256_mullo_epi32( vfilt5, _mm256_loadu_si256( ( const __m256i * ) &tempArray[y1 + 5][x1] ) ) );
        vsum = _mm256_add_epi32( vsum, _mm256_mullo_epi32( vfilt6, _mm256_loadu_si256( ( const __m256i * ) &tempArray[y1 + 6][x1] ) ) );

        vsum = _mm256_srai_epi32( vsum, 12 );

        vsum = _mm256_min_epi32( vmax, _mm256_max_epi32( vmin, vsum ) );

        __m256i
        vorg = _mm256_cvtepi16_epi32( _mm_loadu_si128( ( const __m128i * ) &origRow[x + x1] ) );

        vsum = _mm256_sub_epi32( vsum, vorg );
        vsum = _mm256_mullo_epi32( vsum, vsum );

        vsum = _mm256_hadd_epi32( vsum, vsum );

        __m128i
        xsum = _mm256_extractf128_si256( vsum, 1 );

        error += _mm256_extract_epi32( vsum, 0 );
        error += _mm256_extract_epi32( vsum, 1 );
        
        error += _mm_extract_epi32   ( xsum, 0 );
        error += _mm_extract_epi32   ( xsum, 1 );

        //sum = 0;
        //sum += yFilter[1] * tempArray[y1 + 1][x1];
        //sum += yFilter[2] * tempArray[y1 + 2][x1];
        //sum += yFilter[3] * tempArray[y1 + 3][x1];
        //sum += yFilter[4] * tempArray[y1 + 4][x1];
        //sum += yFilter[5] * tempArray[y1 + 5][x1];
        //sum += yFilter[6] * tempArray[y1 + 6][x1];
        //
        //sum = ( sum + ( 1 << 11 ) ) >> 12;
        //sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );
        //
        //error += ( sum - origRow[x + x1] ) * ( sum - origRow[x + x1] );
      }
      if( error > besterror )
      {
        return error;
      }
    }

    return error;
  }
#endif

          xfilt1 = _mm_set1_epi32( yFilter[1] );
  __m128i xfilt2 = _mm_set1_epi32( yFilter[2] );
  __m128i xfilt3 = _mm_set1_epi32( yFilter[3] );
  __m128i xfilt4 = _mm_set1_epi32( yFilter[4] );
  __m128i xfilt5 = _mm_set1_epi32( yFilter[5] );
  __m128i xfilt6 = _mm_set1_epi32( yFilter[6] );

  __m128i xmax   = _mm_set1_epi32( maxSampleValue );
  __m128i xmin   = _mm_setzero_si128();

  for( int y1 = 0; y1 < bs; y1++ )
  {
    const Pel* origRow = origOrigin + ( y + y1 )*origStride + 0;
    for( int x1 = 0; x1 < bs; x1 += 4 )
    {
      __m128i xsum = _mm_set1_epi32( 1 << 11 );

      xsum = _mm_add_epi32( xsum, _mm_mullo_epi32( xfilt1, _mm_loadu_si128( ( const __m128i * ) &tempArray[y1 + 1][x1] ) ) );
      xsum = _mm_add_epi32( xsum, _mm_mullo_epi32( xfilt2, _mm_loadu_si128( ( const __m128i * ) &tempArray[y1 + 2][x1] ) ) );
      xsum = _mm_add_epi32( xsum, _mm_mullo_epi32( xfilt3, _mm_loadu_si128( ( const __m128i * ) &tempArray[y1 + 3][x1] ) ) );
      xsum = _mm_add_epi32( xsum, _mm_mullo_epi32( xfilt4, _mm_loadu_si128( ( const __m128i * ) &tempArray[y1 + 4][x1] ) ) );
      xsum = _mm_add_epi32( xsum, _mm_mullo_epi32( xfilt5, _mm_loadu_si128( ( const __m128i * ) &tempArray[y1 + 5][x1] ) ) );
      xsum = _mm_add_epi32( xsum, _mm_mullo_epi32( xfilt6, _mm_loadu_si128( ( const __m128i * ) &tempArray[y1 + 6][x1] ) ) );

      xsum = _mm_srai_epi32( xsum, 12 );

      xsum = _mm_min_epi32( xmax, _mm_max_epi32( xmin, xsum ) );

      __m128i
      xorg = _mm_loadl_epi64( ( const __m128i * ) &origRow[x + x1] );
      xorg = _mm_cvtepi16_epi32( xorg );

      xsum = _mm_sub_epi32( xsum, xorg );
      xsum = _mm_mullo_epi32( xsum, xsum );

      xsum = _mm_hadd_epi32( xsum, xsum );

      error += _mm_extract_epi32( xsum, 0 );
      error += _mm_extract_epi32( xsum, 1 );

      //sum = 0;
      //sum += yFilter[1] * tempArray[y1 + 1][x1];
      //sum += yFilter[2] * tempArray[y1 + 2][x1];
      //sum += yFilter[3] * tempArray[y1 + 3][x1];
      //sum += yFilter[4] * tempArray[y1 + 4][x1];
      //sum += yFilter[5] * tempArray[y1 + 5][x1];
      //sum += yFilter[6] * tempArray[y1 + 6][x1];
      //
      //sum = ( sum + ( 1 << 11 ) ) >> 12;
      //sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );
      //
      //error += ( sum - origRow[x + x1] ) * ( sum - origRow[x + x1] );
    }
    if( error > besterror )
    {
      return error;
    }
  }

  return error;
}

template<X86_VEXT vext>
void MCTF::_initMCTF_X86()
{
  m_motionErrorLumaInt8  = motionErrorLumaInt_SIMD <vext>;
  m_motionErrorLumaFrac8 = motionErrorLumaFrac_SIMD<vext>;
}

template
void MCTF::_initMCTF_X86<SIMDX86>();

}
#endif
//! \}

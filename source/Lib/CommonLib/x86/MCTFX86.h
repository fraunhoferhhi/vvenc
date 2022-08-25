/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
int motionErrorLumaInt_SIMD( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int besterror )
{
  int error = 0;

  CHECK( w & 7, "SIMD blockSize needs to be a multiple of 8" );

#if USE_AVX2
  if( ( w & 15 ) == 0 && vext >= AVX2 )
  {
    for( int y1 = 0; y1 < h; y1 += 2 )
    {
      const Pel* origRowStart   = org + y1 * origStride;
      const Pel* bufferRowStart = buf + y1 * buffStride;

      __m256i vsum = _mm256_setzero_si256();

      for( int x1 = 0; x1 < w; x1 += 16 )
      {
        __m256i vorg1 = _mm256_loadu_si256( ( const __m256i* ) &origRowStart[x1] );
        __m256i vorg2 = _mm256_loadu_si256( ( const __m256i* ) &origRowStart[x1+origStride] );
        __m256i vbuf1 = _mm256_loadu_si256( ( const __m256i* ) &bufferRowStart[x1] );
        __m256i vbuf2 = _mm256_loadu_si256( ( const __m256i* ) &bufferRowStart[x1+buffStride] );

        __m256i vsum1 = _mm256_sub_epi16( vorg1, vbuf1 );
        __m256i vsum2 = _mm256_sub_epi16( vorg2, vbuf2 );

        __m256i vtmp1 = _mm256_madd_epi16( vsum1, vsum1 );
        __m256i vtmp2 = _mm256_madd_epi16( vsum2, vsum2 );

        vsum          = _mm256_add_epi32 ( vsum,  vtmp1 );
        vsum          = _mm256_add_epi32 ( vsum,  vtmp2 );

        //int diff = origRowStart[x1] - bufferRowStart[x1];
        //error += diff * diff;
        //diff = origRowStart[x1 + 1] - bufferRowStart[x1 + 1];
        //error += diff * diff;
      }
      
      __m128i
      xtmp = _mm256_extractf128_si256( vsum, 1 );
      xtmp = _mm_add_epi32( xtmp, _mm256_castsi256_si128( vsum ) );
      xtmp = _mm_hadd_epi32( xtmp, xtmp );

      error += _mm_extract_epi32( xtmp, 1 );
      error += _mm_extract_epi32( xtmp, 0 );

      if( error > besterror )
      {
        return error;
      }
    }

    return error;
  }
#endif
  for( int y1 = 0; y1 < h; y1 += 2 )
  {
    const Pel* origRowStart   = org + y1 * origStride;
    const Pel* bufferRowStart = buf + y1 * buffStride;

    __m128i xsum = _mm_setzero_si128();

    for( int x1 = 0; x1 < w; x1 += 8 )
    {
      __m128i xorg1 = _mm_loadu_si128( ( const __m128i* ) &origRowStart[x1] );
      __m128i xorg2 = _mm_loadu_si128( ( const __m128i* ) &origRowStart[x1+origStride] );
      __m128i xbuf1 = _mm_loadu_si128( ( const __m128i* ) &bufferRowStart[x1] );
      __m128i xbuf2 = _mm_loadu_si128( ( const __m128i* ) &bufferRowStart[x1+buffStride] );

      __m128i xsum1 = _mm_sub_epi16( xorg1, xbuf1 );
      __m128i xsum2 = _mm_sub_epi16( xorg2, xbuf2 );

      __m128i xtmp1 = _mm_madd_epi16( xsum1, xsum1 );
      __m128i xtmp2 = _mm_madd_epi16( xsum2, xsum2 );

      xsum          = _mm_add_epi32 ( xsum,  xtmp1 );
      xsum          = _mm_add_epi32 ( xsum,  xtmp2 );

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
int motionErrorLumaFrac_SIMD( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror )
{
  int error = 0;
  const int base = -3;
  
  CHECK( w & 7, "SIMD blockSize needs to be a multiple of 8" );

  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

#if USE_AVX2
  if( vext >= AVX2 && ( w & 15 ) == 0 )
  {
    const __m256i yfilt12 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[1] ), _mm256_set1_epi16( yFilter[2] ) );
    const __m256i yfilt34 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[3] ), _mm256_set1_epi16( yFilter[4] ) );
    const __m256i yfilt56 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[5] ), _mm256_set1_epi16( yFilter[6] ) );

    const __m256i xfilt12 = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[1] ), _mm256_set1_epi16( xFilter[2] ) );
    const __m256i xfilt34 = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[3] ), _mm256_set1_epi16( xFilter[4] ) );
    const __m256i xfilt56 = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[5] ), _mm256_set1_epi16( xFilter[6] ) );

    const __m256i xmax = _mm256_set1_epi16( maxSampleValue );
    const __m256i xmin = _mm256_setzero_si256();

    const int yOffset = 1 - 3;
    const Pel* sourceCol = buf + base + yOffset * buffStride;
    const Pel* origCol = org;

    for( int x1 = 0; x1 < w; x1 += 16, sourceCol += 16, origCol += 16 )
    {
      const Pel* origRow = origCol;
      const Pel* rowStart = sourceCol;

      __m256i xsrc[6];

      for( int y1 = 1; y1 < h + 6; y1++, rowStart += buffStride )
      {
        __m256i xsrc1 = _mm256_loadu_si256( ( const __m256i* ) & rowStart[1] );
        __m256i xsrc2 = _mm256_loadu_si256( ( const __m256i* ) & rowStart[2] );
        __m256i xsrc3 = _mm256_loadu_si256( ( const __m256i* ) & rowStart[3] );
        __m256i xsrc4 = _mm256_loadu_si256( ( const __m256i* ) & rowStart[4] );
        __m256i xsrc5 = _mm256_loadu_si256( ( const __m256i* ) & rowStart[5] );
        __m256i xsrc6 = _mm256_loadu_si256( ( const __m256i* ) & rowStart[6] );

        __m256i
          xsum0 = _mm256_set1_epi32( 1 << 5 );
        __m256i
          xsum1 = _mm256_set1_epi32( 1 << 5 );

        xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( _mm256_unpacklo_epi16( xsrc1, xsrc2 ), xfilt12 ) );
        xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( _mm256_unpackhi_epi16( xsrc1, xsrc2 ), xfilt12 ) );

        xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( _mm256_unpacklo_epi16( xsrc3, xsrc4 ), xfilt34 ) );
        xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( _mm256_unpackhi_epi16( xsrc3, xsrc4 ), xfilt34 ) );

        xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( _mm256_unpacklo_epi16( xsrc5, xsrc6 ), xfilt56 ) );
        xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( _mm256_unpackhi_epi16( xsrc5, xsrc6 ), xfilt56 ) );

        xsum0 = _mm256_srai_epi32( xsum0, 6 );
        xsum1 = _mm256_srai_epi32( xsum1, 6 );
        __m256i
        xsum = _mm256_packs_epi32( xsum0, xsum1 );
        xsum = _mm256_min_epi16( xmax, _mm256_max_epi16( xmin, xsum ) );

        if( y1 >= 6 )
        {
          xsrc[0] = xsrc[1];
          xsrc[1] = xsrc[2];
          xsrc[2] = xsrc[3];
          xsrc[3] = xsrc[4];
          xsrc[4] = xsrc[5];
          xsrc[5] = xsum;

          xsum0 = _mm256_set1_epi32( 1 << 5 );
          xsum1 = _mm256_set1_epi32( 1 << 5 );

          xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( yfilt12, _mm256_unpacklo_epi16( xsrc[0], xsrc[1] ) ) );
          xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( yfilt12, _mm256_unpackhi_epi16( xsrc[0], xsrc[1] ) ) );

          xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( yfilt34, _mm256_unpacklo_epi16( xsrc[2], xsrc[3] ) ) );
          xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( yfilt34, _mm256_unpackhi_epi16( xsrc[2], xsrc[3] ) ) );

          xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( yfilt56, _mm256_unpacklo_epi16( xsrc[4], xsrc[5] ) ) );
          xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( yfilt56, _mm256_unpackhi_epi16( xsrc[4], xsrc[5] ) ) );

          xsum0 = _mm256_srai_epi32( xsum0, 6 );
          xsum1 = _mm256_srai_epi32( xsum1, 6 );

          xsum = _mm256_packs_epi32( xsum0, xsum1 );
          xsum = _mm256_min_epi16( xmax, _mm256_max_epi16( xmin, xsum ) );

          __m256i
          xorg  = _mm256_loadu_si256( ( const __m256i* ) origRow );
          origRow += origStride;

          xsum = _mm256_sub_epi16( xsum, xorg );
          xsum = _mm256_madd_epi16( xsum, xsum );
          xsum = _mm256_hadd_epi32( xsum, xsum );
        
          __m128i
          ysum = _mm_add_epi32( _mm256_castsi256_si128( xsum ), _mm256_extracti128_si256( xsum, 1 ) );

          error += _mm_extract_epi32( ysum, 0 );
          error += _mm_extract_epi32( ysum, 1 );

          if( error > besterror )
          {
            return error;
          }
        }
        else
        {
          xsrc[y1] = xsum;
        }
      }
    }

    return error;
  }
#endif

  const __m128i yfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[1] ), _mm_set1_epi16( yFilter[2] ) );
  const __m128i yfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[3] ), _mm_set1_epi16( yFilter[4] ) );
  const __m128i yfilt56 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[5] ), _mm_set1_epi16( yFilter[6] ) );

  const __m128i xfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[1] ), _mm_set1_epi16( xFilter[2] ) );
  const __m128i xfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[3] ), _mm_set1_epi16( xFilter[4] ) );
  const __m128i xfilt56 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[5] ), _mm_set1_epi16( xFilter[6] ) );
  
  const __m128i xmax   = _mm_set1_epi16( maxSampleValue );
  const __m128i xmin   = _mm_setzero_si128();
  
  const int yOffset    = 1 - 3;
  const Pel* sourceCol = buf + base + yOffset * buffStride;
  const Pel* origCol   = org;

  for( int x1 = 0; x1 < w; x1 += 8, sourceCol += 8, origCol += 8 )
  {
    const Pel* origRow  = origCol;
    const Pel* rowStart = sourceCol;

    __m128i xsrc[6];

    for( int y1 = 1; y1 < h + 6; y1++, rowStart += buffStride )
    {
      __m128i xsrc1 = _mm_loadu_si128( ( const __m128i * ) &rowStart[1] );
      __m128i xsrc2 = _mm_loadu_si128( ( const __m128i * ) &rowStart[2] );
      __m128i xsrc3 = _mm_loadu_si128( ( const __m128i * ) &rowStart[3] );
      __m128i xsrc4 = _mm_loadu_si128( ( const __m128i * ) &rowStart[4] );
      __m128i xsrc5 = _mm_loadu_si128( ( const __m128i * ) &rowStart[5] );
      __m128i xsrc6 = _mm_loadu_si128( ( const __m128i * ) &rowStart[6] );

      __m128i
      xsum0 = _mm_set1_epi32( 1 << 5 );
      __m128i
      xsum1 = _mm_set1_epi32( 1 << 5 );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc1, xsrc2 ), xfilt12 ) );
      xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( _mm_unpackhi_epi16( xsrc1, xsrc2 ), xfilt12 ) );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc3, xsrc4 ), xfilt34 ) );
      xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( _mm_unpackhi_epi16( xsrc3, xsrc4 ), xfilt34 ) );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc5, xsrc6 ), xfilt56 ) );
      xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( _mm_unpackhi_epi16( xsrc5, xsrc6 ), xfilt56 ) );

      xsum0 = _mm_srai_epi32( xsum0, 6 );
      xsum1 = _mm_srai_epi32( xsum1, 6 );
      __m128i
      xsum  = _mm_packs_epi32( xsum0, xsum1 );
      xsum  = _mm_min_epi16( xmax, _mm_max_epi16( xmin, xsum ) );

      if( y1 >= 6 )
      {
        xsrc[0] = xsrc[1];
        xsrc[1] = xsrc[2];
        xsrc[2] = xsrc[3];
        xsrc[3] = xsrc[4];
        xsrc[4] = xsrc[5];
        xsrc[5] = xsum;
        
        xsum0 = _mm_set1_epi32( 1 << 5 );
        xsum1 = _mm_set1_epi32( 1 << 5 );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt12, _mm_unpacklo_epi16( xsrc[0], xsrc[1] ) ) );
        xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( yfilt12, _mm_unpackhi_epi16( xsrc[0], xsrc[1] ) ) );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt34, _mm_unpacklo_epi16( xsrc[2], xsrc[3] ) ) );
        xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( yfilt34, _mm_unpackhi_epi16( xsrc[2], xsrc[3] ) ) );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt56, _mm_unpacklo_epi16( xsrc[4], xsrc[5] ) ) );
        xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( yfilt56, _mm_unpackhi_epi16( xsrc[4], xsrc[5] ) ) );
        
        xsum0 = _mm_srai_epi32( xsum0, 6 );
        xsum1 = _mm_srai_epi32( xsum1, 6 );

        xsum  = _mm_packs_epi32( xsum0, xsum1 );
        xsum  = _mm_min_epi16  ( xmax, _mm_max_epi16( xmin, xsum ) );

        __m128i
        xorg = _mm_loadu_si128( ( const __m128i * ) origRow );
        origRow += origStride;

        xsum = _mm_sub_epi16 ( xsum, xorg );
        xsum = _mm_madd_epi16( xsum, xsum );
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
        //sum = ( sum + ( 1 << 5 ) ) >> 6;
        //sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );
        //
        //error += ( sum - origRow[x + x1] ) * ( sum - origRow[x + x1] );

        if( error > besterror )
        {
          return error;
        }
      }
      else
      {
        xsrc[y1] = xsum;
      
        //sum  = 0;
        //sum += xFilter[1] * rowStart[1];
        //sum += xFilter[2] * rowStart[2];
        //sum += xFilter[3] * rowStart[3];
        //sum += xFilter[4] * rowStart[4];
        //sum += xFilter[5] * rowStart[5];
        //sum += xFilter[6] * rowStart[6];
        //
        //sum = ( sum + ( 1 << 5 ) ) >> 6;
        //sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );
        //
        //tempArray[y1][x1] = sum;
      }
    }
  }

  return error;
}


template<X86_VEXT vext>
int motionErrorLumaFrac_loRes_SIMD( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror )
{
  int error = 0;
  const int base = -1;
  
  CHECK( w & 7, "SIMD blockSize needs to be a multiple of 8" );

#if USE_AVX2
  if( vext >= AVX2 && ( w & 15 ) == 0 )
  {
    const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

    const __m256i yfilt12 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[0] ), _mm256_set1_epi16( yFilter[1] ) );
    const __m256i yfilt34 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[2] ), _mm256_set1_epi16( yFilter[3] ) );

    const __m256i xfilt12 = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[0] ), _mm256_set1_epi16( xFilter[1] ) );
    const __m256i xfilt34 = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[2] ), _mm256_set1_epi16( xFilter[3] ) );
  
    const __m256i xmax   = _mm256_set1_epi16( maxSampleValue );
    const __m256i xmin = _mm256_setzero_si256();
  
    const int yOffset    = -1;
    const Pel* sourceCol = buf + base + yOffset * buffStride;
    const Pel* origCol   = org;

    for( int x1 = 0; x1 < w; x1 += 16, sourceCol += 16, origCol += 16 )
    {
      const Pel* origRow  = origCol;
      const Pel* rowStart = sourceCol;

      __m256i xsrc[4];

      for( int y1 = 0; y1 < h + 3; y1++, rowStart += buffStride )
      {
        __m256i xsrc1 = _mm256_loadu_si256( ( const __m256i * ) &rowStart[0] );
        __m256i xsrc2 = _mm256_loadu_si256( ( const __m256i * ) &rowStart[1] );
        __m256i xsrc3 = _mm256_loadu_si256( ( const __m256i * ) &rowStart[2] );
        __m256i xsrc4 = _mm256_loadu_si256( ( const __m256i * ) &rowStart[3] );

        __m256i
        xsum0 = _mm256_set1_epi32( 1 << 5 );
        __m256i
        xsum1 = _mm256_set1_epi32( 1 << 5 );

        xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( _mm256_unpacklo_epi16( xsrc1, xsrc2 ), xfilt12 ) );
        xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( _mm256_unpackhi_epi16( xsrc1, xsrc2 ), xfilt12 ) );

        xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( _mm256_unpacklo_epi16( xsrc3, xsrc4 ), xfilt34 ) );
        xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( _mm256_unpackhi_epi16( xsrc3, xsrc4 ), xfilt34 ) );

        xsum0 = _mm256_srai_epi32( xsum0, 6 );
        xsum1 = _mm256_srai_epi32( xsum1, 6 );
        __m256i
        xsum  = _mm256_packs_epi32( xsum0, xsum1 );
        xsum  = _mm256_min_epi16( xmax, _mm256_max_epi16( xmin, xsum ) );

        if( y1 >= 3 )
        {
          xsrc[0] = xsrc[1];
          xsrc[1] = xsrc[2];
          xsrc[2] = xsrc[3];
          xsrc[3] = xsum;
        
          xsum0 = _mm256_set1_epi32( 1 << 5 );
          xsum1 = _mm256_set1_epi32( 1 << 5 );

          xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( yfilt12, _mm256_unpacklo_epi16( xsrc[0], xsrc[1] ) ) );
          xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( yfilt12, _mm256_unpackhi_epi16( xsrc[0], xsrc[1] ) ) );

          xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( yfilt34, _mm256_unpacklo_epi16( xsrc[2], xsrc[3] ) ) );
          xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( yfilt34, _mm256_unpackhi_epi16( xsrc[2], xsrc[3] ) ) );
        
          xsum0 = _mm256_srai_epi32( xsum0, 6 );
          xsum1 = _mm256_srai_epi32( xsum1, 6 );

          xsum  = _mm256_packs_epi32( xsum0, xsum1 );
          xsum  = _mm256_min_epi16  ( xmax, _mm256_max_epi16( xmin, xsum ) );

          __m256i
          xorg  = _mm256_loadu_si256( ( const __m256i * ) origRow );
          origRow += origStride;

          xsum = _mm256_sub_epi16( xsum, xorg );
          xsum = _mm256_madd_epi16( xsum, xsum );
          xsum = _mm256_hadd_epi32( xsum, xsum );

          __m128i
          ysum = _mm_add_epi32( _mm256_castsi256_si128( xsum ), _mm256_extracti128_si256( xsum, 1 ) );

          error += _mm_extract_epi32( ysum, 0 );
          error += _mm_extract_epi32( ysum, 1 );

          if( error > besterror )
          {
            return error;
          }
        }
        else
        {
          xsrc[y1 + 1] = xsum;
        }
      }
    }

    return error;
  }
#endif 

  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

  const __m128i yfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[0] ), _mm_set1_epi16( yFilter[1] ) );
  const __m128i yfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[2] ), _mm_set1_epi16( yFilter[3] ) );

  const __m128i xfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[0] ), _mm_set1_epi16( xFilter[1] ) );
  const __m128i xfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[2] ), _mm_set1_epi16( xFilter[3] ) );
  
  const __m128i xmax   = _mm_set1_epi16( maxSampleValue );
  const __m128i xmin   = _mm_setzero_si128();
  
  const int yOffset    = -1;
  const Pel* sourceCol = buf + base + yOffset * buffStride;
  const Pel* origCol   = org;

  for( int x1 = 0; x1 < w; x1 += 8, sourceCol += 8, origCol += 8 )
  {
    const Pel* origRow  = origCol;
    const Pel* rowStart = sourceCol;

    __m128i xsrc[4];

    for( int y1 = 0; y1 < h + 3; y1++, rowStart += buffStride )
    {
      __m128i xsrc1 = _mm_loadu_si128( ( const __m128i * ) &rowStart[0] );
      __m128i xsrc2 = _mm_loadu_si128( ( const __m128i * ) &rowStart[1] );
      __m128i xsrc3 = _mm_loadu_si128( ( const __m128i * ) &rowStart[2] );
      __m128i xsrc4 = _mm_loadu_si128( ( const __m128i * ) &rowStart[3] );

      __m128i
      xsum0 = _mm_set1_epi32( 1 << 5 );
      __m128i
      xsum1 = _mm_set1_epi32( 1 << 5 );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc1, xsrc2 ), xfilt12 ) );
      xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( _mm_unpackhi_epi16( xsrc1, xsrc2 ), xfilt12 ) );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc3, xsrc4 ), xfilt34 ) );
      xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( _mm_unpackhi_epi16( xsrc3, xsrc4 ), xfilt34 ) );

      xsum0 = _mm_srai_epi32( xsum0, 6 );
      xsum1 = _mm_srai_epi32( xsum1, 6 );
      __m128i
      xsum  = _mm_packs_epi32( xsum0, xsum1 );
      xsum  = _mm_min_epi16( xmax, _mm_max_epi16( xmin, xsum ) );

      if( y1 >= 3 )
      {
        xsrc[0] = xsrc[1];
        xsrc[1] = xsrc[2];
        xsrc[2] = xsrc[3];
        xsrc[3] = xsum;
        
        xsum0 = _mm_set1_epi32( 1 << 5 );
        xsum1 = _mm_set1_epi32( 1 << 5 );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt12, _mm_unpacklo_epi16( xsrc[0], xsrc[1] ) ) );
        xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( yfilt12, _mm_unpackhi_epi16( xsrc[0], xsrc[1] ) ) );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt34, _mm_unpacklo_epi16( xsrc[2], xsrc[3] ) ) );
        xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( yfilt34, _mm_unpackhi_epi16( xsrc[2], xsrc[3] ) ) );
        
        xsum0 = _mm_srai_epi32( xsum0, 6 );
        xsum1 = _mm_srai_epi32( xsum1, 6 );

        xsum  = _mm_packs_epi32( xsum0, xsum1 );
        xsum  = _mm_min_epi16  ( xmax, _mm_max_epi16( xmin, xsum ) );

        __m128i
        xorg = _mm_loadu_si128( ( const __m128i * ) origRow );
        origRow += origStride;

        xsum = _mm_sub_epi16 ( xsum, xorg );
        xsum = _mm_madd_epi16( xsum, xsum );
        xsum = _mm_hadd_epi32( xsum, xsum );

        error += _mm_extract_epi32( xsum, 0 );
        error += _mm_extract_epi32( xsum, 1 );

        //sum = 0;
        //sum += yFilter[1] * tempArray[y1 + 1][x1];
        //sum += yFilter[2] * tempArray[y1 + 2][x1];
        //sum += yFilter[3] * tempArray[y1 + 3][x1];
        //sum += yFilter[4] * tempArray[y1 + 4][x1];
        //
        //sum = ( sum + ( 1 << 5 ) ) >> 6;
        //sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );
        //
        //error += ( sum - origRow[x + x1] ) * ( sum - origRow[x + x1] );

        if( error > besterror )
        {
          return error;
        }
      }
      else
      {
        xsrc[y1 + 1] = xsum;
      
        //sum  = 0;
        //sum += xFilter[1] * rowStart[1];
        //sum += xFilter[2] * rowStart[2];
        //sum += xFilter[3] * rowStart[3];
        //sum += xFilter[4] * rowStart[4];
        //
        //sum = ( sum + ( 1 << 5 ) ) >> 6;
        //sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );
        //
        //tempArray[y1][x1] = sum;
      }
    }
  }

  return error;
}

template<X86_VEXT vext>
void applyFrac6tap_SIMD_8x( const Pel* org, const ptrdiff_t origStride, Pel* buf, const ptrdiff_t buffStride, const int bsx, const int bsy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  const int base = -3;

  CHECK( bsx & 7, "SIMD blockSizeX needs to be a multiple of 8" );

  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

  const __m128i yfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[1] ), _mm_set1_epi16( yFilter[2] ) );
  const __m128i yfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[3] ), _mm_set1_epi16( yFilter[4] ) );
  const __m128i yfilt56 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[5] ), _mm_set1_epi16( yFilter[6] ) );

  const __m128i xfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[1] ), _mm_set1_epi16( xFilter[2] ) );
  const __m128i xfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[3] ), _mm_set1_epi16( xFilter[4] ) );
  const __m128i xfilt56 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[5] ), _mm_set1_epi16( xFilter[6] ) );

  const __m128i xmax = _mm_set1_epi16( maxSampleValue );
  const __m128i xmin = _mm_setzero_si128();

  const int yOffset = 1 - 3;
  const Pel* srcCol = org + base + yOffset * origStride;
        Pel* dstCol = buf;

  for( int x1 = 0; x1 < bsx; x1 += 8, srcCol += 8, dstCol += 8 )
  {
    const Pel* srcRow = srcCol;
          Pel* dstRow = dstCol;

    __m128i xsrc[6];

    for( int y1 = 1; y1 < bsy + 6; y1++, srcRow += origStride )
    {
      __m128i xsrc1 = _mm_loadu_si128( ( const __m128i* ) &srcRow[1] );
      __m128i xsrc2 = _mm_loadu_si128( ( const __m128i* ) &srcRow[2] );
      __m128i xsrc3 = _mm_loadu_si128( ( const __m128i* ) &srcRow[3] );
      __m128i xsrc4 = _mm_loadu_si128( ( const __m128i* ) &srcRow[4] );
      __m128i xsrc5 = _mm_loadu_si128( ( const __m128i* ) &srcRow[5] );
      __m128i xsrc6 = _mm_loadu_si128( ( const __m128i* ) &srcRow[6] );

      __m128i
      xsum0 = _mm_set1_epi32( 1 << 5 );
      __m128i
      xsum1 = _mm_set1_epi32( 1 << 5 );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc1, xsrc2 ), xfilt12 ) );
      xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( _mm_unpackhi_epi16( xsrc1, xsrc2 ), xfilt12 ) );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc3, xsrc4 ), xfilt34 ) );
      xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( _mm_unpackhi_epi16( xsrc3, xsrc4 ), xfilt34 ) );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc5, xsrc6 ), xfilt56 ) );
      xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( _mm_unpackhi_epi16( xsrc5, xsrc6 ), xfilt56 ) );

      xsum0 = _mm_srai_epi32( xsum0, 6 );
      xsum1 = _mm_srai_epi32( xsum1, 6 );
      __m128i
      xsum = _mm_packs_epi32( xsum0, xsum1 );

      if( y1 >= 6 )
      {
        xsrc[0] = xsrc[1];
        xsrc[1] = xsrc[2];
        xsrc[2] = xsrc[3];
        xsrc[3] = xsrc[4];
        xsrc[4] = xsrc[5];
        xsrc[5] = xsum;

        xsum0 = _mm_set1_epi32( 1 << 5 );
        xsum1 = _mm_set1_epi32( 1 << 5 );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt12, _mm_unpacklo_epi16( xsrc[0], xsrc[1] ) ) );
        xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( yfilt12, _mm_unpackhi_epi16( xsrc[0], xsrc[1] ) ) );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt34, _mm_unpacklo_epi16( xsrc[2], xsrc[3] ) ) );
        xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( yfilt34, _mm_unpackhi_epi16( xsrc[2], xsrc[3] ) ) );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt56, _mm_unpacklo_epi16( xsrc[4], xsrc[5] ) ) );
        xsum1 = _mm_add_epi32( xsum1, _mm_madd_epi16( yfilt56, _mm_unpackhi_epi16( xsrc[4], xsrc[5] ) ) );

        xsum0 = _mm_srai_epi32( xsum0, 6 );
        xsum1 = _mm_srai_epi32( xsum1, 6 );

        xsum = _mm_packs_epi32( xsum0, xsum1 );
        xsum = _mm_min_epi16( xmax, _mm_max_epi16( xmin, xsum ) );

        _mm_storeu_si128( ( __m128i* ) dstRow, xsum );
        dstRow += buffStride;
      }
      else
      {
        xsrc[y1] = xsum;
      }
    }
  }
}


template<X86_VEXT vext>
void applyFrac6tap_SIMD_4x( const Pel* org, const ptrdiff_t origStride, Pel* buf, const ptrdiff_t buffStride, const int bsx, const int bsy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  const int base = -3;

  CHECK( bsx & 3, "SIMD blockSizeX needs to be a multiple of 4" );

  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

  const __m128i yfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[1] ), _mm_set1_epi16( yFilter[2] ) );
  const __m128i yfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[3] ), _mm_set1_epi16( yFilter[4] ) );
  const __m128i yfilt56 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[5] ), _mm_set1_epi16( yFilter[6] ) );

  const __m128i xfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[1] ), _mm_set1_epi16( xFilter[2] ) );
  const __m128i xfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[3] ), _mm_set1_epi16( xFilter[4] ) );
  const __m128i xfilt56 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[5] ), _mm_set1_epi16( xFilter[6] ) );

  const __m128i xmax = _mm_set1_epi16( maxSampleValue );
  const __m128i xmin = _mm_setzero_si128();

  const int yOffset = 1 - 3;
  const Pel* srcCol = org + base + yOffset * origStride;
        Pel* dstCol = buf;

  for( int x1 = 0; x1 < bsx; x1 += 4, srcCol += 4, dstCol += 4 )
  {
    const Pel* srcRow = srcCol;
          Pel* dstRow = dstCol;

    __m128i xsrc[6];

    for( int y1 = 1; y1 < bsy + 6; y1++, srcRow += origStride )
    {
      __m128i xsrc1 = _mm_loadl_epi64( ( const __m128i* ) &srcRow[1] );
      __m128i xsrc2 = _mm_loadl_epi64( ( const __m128i* ) &srcRow[2] );
      __m128i xsrc3 = _mm_loadl_epi64( ( const __m128i* ) &srcRow[3] );
      __m128i xsrc4 = _mm_loadl_epi64( ( const __m128i* ) &srcRow[4] );
      __m128i xsrc5 = _mm_loadl_epi64( ( const __m128i* ) &srcRow[5] );
      __m128i xsrc6 = _mm_loadl_epi64( ( const __m128i* ) &srcRow[6] );

      __m128i
      xsum0 = _mm_set1_epi32( 1 << 5 );

      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc1, xsrc2 ), xfilt12 ) );
      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc3, xsrc4 ), xfilt34 ) );
      xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( _mm_unpacklo_epi16( xsrc5, xsrc6 ), xfilt56 ) );

      xsum0 = _mm_srai_epi32( xsum0, 6 );
      __m128i
      xsum = _mm_packs_epi32( xsum0, _mm_setzero_si128() );

      if( y1 >= 6 )
      {
        xsrc[0] = xsrc[1];
        xsrc[1] = xsrc[2];
        xsrc[2] = xsrc[3];
        xsrc[3] = xsrc[4];
        xsrc[4] = xsrc[5];
        xsrc[5] = xsum;

        xsum0 = _mm_set1_epi32( 1 << 5 );

        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt12, _mm_unpacklo_epi16( xsrc[0], xsrc[1] ) ) );
        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt34, _mm_unpacklo_epi16( xsrc[2], xsrc[3] ) ) );
        xsum0 = _mm_add_epi32( xsum0, _mm_madd_epi16( yfilt56, _mm_unpacklo_epi16( xsrc[4], xsrc[5] ) ) );

        xsum0 = _mm_srai_epi32( xsum0, 6 );
        xsum = _mm_packs_epi32( xsum0, _mm_setzero_si128() );
        xsum = _mm_min_epi16( xmax, _mm_max_epi16( xmin, xsum ) );

        _mm_storel_epi64( ( __m128i* ) dstRow, xsum );
        dstRow += buffStride;
      }
      else
      {
        xsrc[y1] = xsum;
      }
    }
  }
}

template<X86_VEXT vext>
void applyBlockSIMD( const CPelBuf& src, PelBuf& dst, const CompArea& blk, const ClpRng& clpRng, const Pel **correctedPics, int numRefs, const int *verror, const double *refStrenghts, double weightScaling, double sigmaSq )
{
  const int         w = blk.width;
  const int         h = blk.height;
  const int        bx = blk.x;
  const int        by = blk.y;

  const ptrdiff_t srcStride = src.stride;
  const ptrdiff_t dstStride = dst.stride;

  const Pel *srcPel = src.bufAt( bx, by );
        Pel *dstPel = dst.bufAt( bx, by );

  int vnoise[2 * VVENC_MCTF_RANGE] = { 0, };
  float vsw[2 * VVENC_MCTF_RANGE] = { 0.0f, };
  float vww[2 * VVENC_MCTF_RANGE] = { 0.0f, };

  int minError = 9999999;

  for( int i = 0; i < numRefs; i++ )
  {
    int64_t variance = 0, diffsum = 0;
    const ptrdiff_t refStride = w;
    const Pel *     refPel    = correctedPics[i];
    for( int y1 = 0; y1 < h; y1++ )
    {
      for( int x1 = 0; x1 < w; x1++ )
      {
        const Pel pix = *( srcPel + srcStride * y1 + x1 );
        const Pel ref = *( refPel + refStride * y1 + x1 );

        const int diff = pix - ref;
        variance += diff * diff;
        if( x1 != w - 1 )
        {
          const Pel pixR = *( srcPel + srcStride * y1 + x1 + 1 );
          const Pel refR = *( refPel + refStride * y1 + x1 + 1 );
          const int diffR = pixR - refR;
          diffsum += ( diffR - diff ) * ( diffR - diff );
        }
        if( y1 != h - 1 )
        {
          const Pel pixD = *( srcPel + srcStride * y1 + x1 + srcStride );
          const Pel refD = *( refPel + refStride * y1 + x1 + refStride );
          const int diffD = pixD - refD;
          diffsum += ( diffD - diff ) * ( diffD - diff );
        }
      }
    }
    const int cntV = w * h;
    const int cntD = 2 * cntV - w - h;
    vnoise[i] = ( int ) ( ( ( 15.0 * cntD / cntV * variance + 5.0 ) / ( diffsum + 5.0 ) ) + 0.5 );
    minError = std::min( minError, verror[i] );
  }

  for( int i = 0; i < numRefs; i++ )
  {
    const int error = verror[i];
    const int noise = vnoise[i];
    float ww = 1, sw = 1;
    ww *= ( noise < 25 ) ? 1.0 : 0.6;
    sw *= ( noise < 25 ) ? 1.0 : 0.8;
    ww *= ( error < 50 ) ? 1.2 : ( ( error > 100 ) ? 0.6 : 1.0 );
    sw *= ( error < 50 ) ? 1.0 : 0.8;
    ww *= ( ( minError + 1.0 ) / ( error + 1.0 ) );

    vww[i] = ww * weightScaling * refStrenghts[i];
    vsw[i] = sw * 2 * sigmaSq;
  }

  //inline static float fastExp( float x )
  //{
  //  // using the e^x ~= ( 1 + x/n )^n for n -> inf
  //  float x = 1.0 + x / 1024;
  //  x *= x; x *= x; x *= x; x *= x;
  //  x *= x; x *= x; x *= x; x *= x;
  //  x *= x; x *= x;
  //  return x;
  //}

  for( int y = 0; y < h; y++ )
  {
    for( int x = 0; x < w; x += 4 )
    {
      __m128i vorgi = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i* ) ( srcPel + srcStride * y + x ) ) );
      __m128  vorg  = _mm_cvtepi32_ps( vorgi );
      //const Pel orgVal  = *( srcPel + srcStride * y + x );
      __m128  vtws  = _mm_set1_ps( 1.0f );
      //float temporalWeightSum = 1.0;
      //float newVal = ( float ) orgVal;
      __m128  vnewv = vorg;

      for( int i = 0; i < numRefs; i++ )
      {
        const Pel* pCorrectedPelPtr = correctedPics[i] + y * w + x;
        __m128i vrefi = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i* ) pCorrectedPelPtr ) );
        //const int    refVal = *pCorrectedPelPtr;
        __m128i vdifi = _mm_sub_epi16( vrefi, vorgi );
        //const int    diff   = refVal - orgVal;
        //const float  diffSq = diff * diff;
        __m128i vdsqi = _mm_madd_epi16( vdifi, vdifi );
        __m128  vdsq  = _mm_cvtepi32_ps( vdsqi );

        // apply fast exp with 10 iterations!
        __m128  vwght = _mm_div_ps( vdsq, _mm_set1_ps( -vsw[i] * 1024.0f ) );
        vwght = _mm_add_ps( vwght, _mm_set1_ps( 1.0f ) );

        vwght = _mm_mul_ps( vwght, vwght ); //  1
        vwght = _mm_mul_ps( vwght, vwght ); //  2
        vwght = _mm_mul_ps( vwght, vwght ); //  3
        vwght = _mm_mul_ps( vwght, vwght ); //  4
        vwght = _mm_mul_ps( vwght, vwght ); //  5
                                       
        vwght = _mm_mul_ps( vwght, vwght ); //  6
        vwght = _mm_mul_ps( vwght, vwght ); //  7
        vwght = _mm_mul_ps( vwght, vwght ); //  8
        vwght = _mm_mul_ps( vwght, vwght ); //  9
        vwght = _mm_mul_ps( vwght, vwght ); // 10

        vwght = _mm_mul_ps( vwght, _mm_set1_ps( vww[i] ) );
        //float weight = vww[i] * fastExp( -diffSq, vsw[i] );
        
        vnewv = _mm_add_ps( vnewv, _mm_mul_ps( vwght, _mm_cvtepi32_ps( vrefi ) ) );
        //newVal += weight * refVal;

        vtws  = _mm_add_ps( vtws, vwght );
        //temporalWeightSum += weight;
      }

      vnewv = _mm_div_ps( vnewv, vtws );
      //newVal /= temporalWeightSum;
      vnewv = _mm_add_ps( vnewv, _mm_set1_ps( 0.5f ) );
      vnewv = _mm_round_ps( vnewv, ( _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC ) );
      //Pel sampleVal = ( Pel ) ( newVal + 0.5 );
      __m128i vnewi = _mm_cvtps_epi32( vnewv );

      vnewi = _mm_max_epi32( vnewi, _mm_setzero_si128() );
      vnewi = _mm_min_epi32( vnewi, _mm_set1_epi32( clpRng.max() ) );
      //sampleVal = ( sampleVal < 0 ? 0 : ( sampleVal > maxSampleValue ? maxSampleValue : sampleVal ) );
      
      vnewi = _mm_packs_epi32( vnewi, vnewi );
      //*( dstPel + srcStride * y + x ) = sampleVal;
      _mm_storel_epi64( ( __m128i * ) ( dstPel + dstStride * y + x ), vnewi );
    }
  }
}


template<X86_VEXT vext>
void MCTF::_initMCTF_X86()
{
  m_motionErrorLumaInt8     = motionErrorLumaInt_SIMD<vext>;
  m_motionErrorLumaFrac8[0] = motionErrorLumaFrac_SIMD<vext>;
  m_motionErrorLumaFrac8[1] = motionErrorLumaFrac_loRes_SIMD<vext>;

  m_applyFrac[0][0] = applyFrac6tap_SIMD_8x<vext>;
  m_applyFrac[1][0] = applyFrac6tap_SIMD_4x<vext>;

  m_applyBlock      = applyBlockSIMD<vext>;
}

template
void MCTF::_initMCTF_X86<SIMDX86>();

}
#endif
//! \}

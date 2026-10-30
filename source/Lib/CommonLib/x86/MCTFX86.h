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

#define cond_mm_prefetch(a,b) _mm_prefetch(a,b)
//#define cond_mm_prefetch(a,b)

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_MCTF

namespace vvenc {

#if USE_AVX2
static inline int sumOfLanes_AVX2( const __m256i v )
{
  __m128i x = _mm_add_epi32( _mm256_castsi256_si128( v ), _mm256_extracti128_si256( v, 1 ) );
  x = _mm_hadd_epi32( x, x );
  x = _mm_hadd_epi32( x, x );
  return _mm_cvtsi128_si32( x );
}

// The largest block whose squared error cannot overflow 32 bits: 2048 * 1023^2 < 2^31 (samples of at most 10 bits).
static constexpr int MAX_EXACT_INT_AREA = 2048;

// motionErrorLumaInt_SIMD for blocks of a width that is a multiple of 16 and at most MAX_EXACT_INT_AREA samples. It
// returns the exact error when that is not above besterror and a partial sum above besterror otherwise, as every other
// kernel does (only a smaller error replaces the best, so the values above besterror are never used). Blocks of up to
// 16 rows are summed in full: for them the checks cost more than the early exits save. Taller blocks check the exact
// running sum after every 4 rows, which never computes more rows than the general path's check after every row pair
// (that check reads the sum of all but the last two row pairs).
static int motionErrorLumaInt_AVX2( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int besterror )
{
  const bool check = h > 16;
  __m256i vsum = _mm256_setzero_si256();
  for( int y = 0; y < h; y += 2 )
  {
    const Pel* orgRow = org + y * origStride;
    const Pel* bufRow = buf + y * buffStride;
    for( int x = 0; x < w; x += 16 )
    {
      const __m256i diff1 = _mm256_sub_epi16( _mm256_loadu_si256( ( const __m256i* ) ( orgRow + x ) ),
                                              _mm256_loadu_si256( ( const __m256i* ) ( bufRow + x ) ) );
      const __m256i diff2 = _mm256_sub_epi16( _mm256_loadu_si256( ( const __m256i* ) ( orgRow + origStride + x ) ),
                                              _mm256_loadu_si256( ( const __m256i* ) ( bufRow + buffStride + x ) ) );
      vsum = _mm256_add_epi32( vsum, _mm256_add_epi32( _mm256_madd_epi16( diff1, diff1 ), _mm256_madd_epi16( diff2, diff2 ) ) );
    }
    // after rows 4, 8, ...; the last check is the final sum
    if( check && ( y & 2 ) && y + 2 < h )
    {
      const int error = sumOfLanes_AVX2( vsum );
      if( error > besterror )
      {
        return error;
      }
    }
  }
  return sumOfLanes_AVX2( vsum );
}
#endif

template<X86_VEXT vext>
int motionErrorLumaInt_SIMD( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int besterror )
{
#if USE_AVX2
  if( vext >= AVX2 && ( w & 15 ) == 0 && w * h <= MAX_EXACT_INT_AREA )
  {
    return motionErrorLumaInt_AVX2( org, origStride, buf, buffStride, w, h, besterror );
  }
#endif
  int error = 0;
  __m128i xerror = _mm_setzero_si128();

  cond_mm_prefetch( ( const char* ) ( org ),              _MM_HINT_T0 );
  cond_mm_prefetch( ( const char* ) ( org + origStride ), _MM_HINT_T0 );
  cond_mm_prefetch( ( const char* ) ( buf ),              _MM_HINT_T0 );
  cond_mm_prefetch( ( const char* ) ( buf + buffStride ), _MM_HINT_T0 );

  CHECK( w & 7, "SIMD blockSize needs to be a multiple of 8" );

#if USE_AVX2
  if( ( w & 15 ) == 0 && vext >= AVX2 )
  {
    for( int y1 = 0; y1 < h; y1 += 2 )
    {
      const Pel* origRowStart   = org + y1 * origStride;
      const Pel* bufferRowStart = buf + y1 * buffStride;

      cond_mm_prefetch( ( const char* ) ( origRowStart   + 2 * origStride ), _MM_HINT_T0 );
      cond_mm_prefetch( ( const char* ) ( origRowStart   + 3 * origStride ), _MM_HINT_T0 );
      cond_mm_prefetch( ( const char* ) ( bufferRowStart + 2 * buffStride ), _MM_HINT_T0 );
      cond_mm_prefetch( ( const char* ) ( bufferRowStart + 3 * buffStride ), _MM_HINT_T0 );

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
      xerror = _mm_hadd_epi32( xerror, xtmp );
      error = _mm_cvtsi128_si32( xerror );

      if( error > besterror )
      {
        return error;
      }
    }

    xerror = _mm_hadd_epi32( xerror, xerror );
    xerror = _mm_hadd_epi32( xerror, xerror );
    error = _mm_cvtsi128_si32( xerror );
    return error;
  }
#endif
  for( int y1 = 0; y1 < h; y1 += 2 )
  {
    const Pel* origRowStart   = org + y1 * origStride;
    const Pel* bufferRowStart = buf + y1 * buffStride;

    __m128i xsum   = _mm_setzero_si128();

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
    
    xerror = _mm_hadd_epi32   ( xerror, xsum );
    error  = _mm_cvtsi128_si32( xerror );

    if( error > besterror )
    {
      return error;
    }
  }

  xerror = _mm_hadd_epi32( xerror, xerror );
  xerror = _mm_hadd_epi32( xerror, xerror );
  error  = _mm_cvtsi128_si32( xerror );
  return error;
}

template<X86_VEXT vext>
int motionErrorLumaFrac_SIMD( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror )
{
  int error = 0;
  const int base = -3;
  __m128i xerror = _mm_setzero_si128();
  
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
        
          __m128i
          ysum = _mm_add_epi32( _mm256_castsi256_si128( xsum ), _mm256_extracti128_si256( xsum, 1 ) );
          xerror = _mm_hadd_epi32( xerror, ysum );
          error = _mm_cvtsi128_si32( xerror );

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

    xerror = _mm_hadd_epi32( xerror, xerror );
    xerror = _mm_hadd_epi32( xerror, xerror );
    error  = _mm_cvtsi128_si32( xerror );

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
        xerror = _mm_hadd_epi32( xerror, xsum );
        error  = _mm_cvtsi128_si32( xerror );

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

  xerror = _mm_hadd_epi32( xerror, xerror );
  xerror = _mm_hadd_epi32( xerror, xerror );
  error  = _mm_cvtsi128_si32( xerror );

  return error;
}


#if USE_AVX2
// Filter 0 of the 4-tap table is { 0, 64, 0, 0 }: its pass only clips the samples to the sample range
// ( ( 64 * s + 32 ) >> 6 = s ), so a vector whose horizontal or vertical part is a full sample needs only the other pass.
static inline bool isIdentityFilter4( const int16_t* filter )
{
  return filter[0] == 0 && filter[1] == 64 && filter[2] == 0 && filter[3] == 0;
}

static inline __m256i clipToSampleRange_AVX2( const __m256i v, const __m256i maxSample )
{
  return _mm256_min_epi16( maxSample, _mm256_max_epi16( _mm256_setzero_si256(), v ) );
}

// Rounds the 32-bit filter sums of 16 samples ( >> 6 ) and clips them to the sample range.
static inline __m256i roundAndClip4Tap_AVX2( const __m256i sumLo, const __m256i sumHi, const __m256i maxSample )
{
  return clipToSampleRange_AVX2( _mm256_packs_epi32( _mm256_srai_epi32( sumLo, 6 ), _mm256_srai_epi32( sumHi, 6 ) ), maxSample );
}

// The horizontal 4-tap filter of 16 samples: output sample x from source samples x - 1 to x + 2, rounded and clipped.
static inline __m256i horizontal4Tap_AVX2( const Pel* src, const __m256i xfilt12, const __m256i xfilt34, const __m256i rounding, const __m256i maxSample )
{
  const __m256i s0 = _mm256_loadu_si256( ( const __m256i* ) ( src - 1 ) );
  const __m256i s1 = _mm256_loadu_si256( ( const __m256i* ) src );
  const __m256i s2 = _mm256_loadu_si256( ( const __m256i* ) ( src + 1 ) );
  const __m256i s3 = _mm256_loadu_si256( ( const __m256i* ) ( src + 2 ) );
  __m256i sumLo = _mm256_add_epi32( rounding, _mm256_madd_epi16( _mm256_unpacklo_epi16( s0, s1 ), xfilt12 ) );
  __m256i sumHi = _mm256_add_epi32( rounding, _mm256_madd_epi16( _mm256_unpackhi_epi16( s0, s1 ), xfilt12 ) );
  sumLo = _mm256_add_epi32( sumLo, _mm256_madd_epi16( _mm256_unpacklo_epi16( s2, s3 ), xfilt34 ) );
  sumHi = _mm256_add_epi32( sumHi, _mm256_madd_epi16( _mm256_unpackhi_epi16( s2, s3 ), xfilt34 ) );
  return roundAndClip4Tap_AVX2( sumLo, sumHi, maxSample );
}

// The squared error against org of the vertical 4-tap filter over the rows of src: output row y from rows y to y + 3.
// clipInput clips the input samples to the sample range first, as a skipped horizontal pass would.
// The filter runs in 16-bit lanes, exactly: every tap of the table is even, so with the halved taps and samples of at
// most 10 bits the sum lies in [-5115, 37851]. Adding the rounding 16 and a bias of 160 << 5 puts it in [21, 42987],
// inside the unsigned 16-bit range, so ( ( sum + 16 + ( 160 << 5 ) ) >> 5 ) - 160 == ( 2 * sum + 32 ) >> 6, the
// rounding of the full taps, for every input; the shift is logical (unsigned 16-bit).
template<bool clipInput>
static int verticalPassError_AVX2( const Pel* org, const ptrdiff_t origStride, const Pel* src, const ptrdiff_t srcStride, const int w, const int h, const int16_t* yFilter, const int bitDepth )
{
  static constexpr int BIAS = 160;
  CHECKD( bitDepth > 10 || ( ( yFilter[0] | yFilter[1] | yFilter[2] | yFilter[3] ) & 1 ), "The 16-bit vertical pass needs even taps and at most 10 bits" );
  const __m256i tap0      = _mm256_set1_epi16( yFilter[0] / 2 );
  const __m256i tap1      = _mm256_set1_epi16( yFilter[1] / 2 );
  const __m256i tap2      = _mm256_set1_epi16( yFilter[2] / 2 );
  const __m256i tap3      = _mm256_set1_epi16( yFilter[3] / 2 );
  const __m256i biased    = _mm256_set1_epi16( 16 + ( BIAS << 5 ) );
  const __m256i bias      = _mm256_set1_epi16( BIAS );
  const __m256i maxSample = _mm256_set1_epi16( ( 1 << bitDepth ) - 1 );
  __m256i verror = _mm256_setzero_si256();

  for( int x = 0; x < w; x += 16 )
  {
    auto loadRow = [&]( int y )
    {
      const __m256i row = _mm256_loadu_si256( ( const __m256i* ) ( src + y * srcStride + x ) );
      return clipInput ? clipToSampleRange_AVX2( row, maxSample ) : row;
    };
    __m256i row0 = loadRow( 0 );
    __m256i row1 = loadRow( 1 );
    __m256i row2 = loadRow( 2 );
    for( int y = 0; y < h; y++ )
    {
      const __m256i row3 = loadRow( y + 3 );
      const __m256i sum  = _mm256_add_epi16( _mm256_add_epi16( _mm256_mullo_epi16( row0, tap0 ), _mm256_mullo_epi16( row1, tap1 ) ),
                                             _mm256_add_epi16( _mm256_mullo_epi16( row2, tap2 ), _mm256_mullo_epi16( row3, tap3 ) ) );
      const __m256i pred = _mm256_sub_epi16( _mm256_srli_epi16( _mm256_add_epi16( sum, biased ), 5 ), bias );

      const __m256i org16 = _mm256_loadu_si256( ( const __m256i* ) ( org + y * origStride + x ) );
      const __m256i diff  = _mm256_sub_epi16( clipToSampleRange_AVX2( pred, maxSample ), org16 );
      verror = _mm256_add_epi32( verror, _mm256_madd_epi16( diff, diff ) );

      row0 = row1;
      row1 = row2;
      row2 = row3;
    }
  }
  return sumOfLanes_AVX2( verror );
}

// The squared error between the w x h blocks a and org.
static int squaredError_AVX2( const Pel* org, const ptrdiff_t origStride, const Pel* a, const ptrdiff_t aStride, const int w, const int h )
{
  __m256i verror = _mm256_setzero_si256();
  for( int y = 0; y < h; y++ )
  {
    for( int x = 0; x < w; x += 16 )
    {
      const __m256i diff = _mm256_sub_epi16( _mm256_loadu_si256( ( const __m256i* ) ( a + y * aStride + x ) ),
                                             _mm256_loadu_si256( ( const __m256i* ) ( org + y * origStride + x ) ) );
      verror = _mm256_add_epi32( verror, _mm256_madd_epi16( diff, diff ) );
    }
  }
  return sumOfLanes_AVX2( verror );
}

// motionErrorLumaFrac_loRes_SIMD for a full-sample horizontal position: the vertical pass alone, on the clipped samples.
static int motionErrorLumaFracV_loRes_AVX2( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* yFilter, const int bitDepth )
{
  // output row y is filtered from source rows y - 1 to y + 2
  return verticalPassError_AVX2<true>( org, origStride, buf - buffStride, buffStride, w, h, yFilter, bitDepth );
}

// motionErrorLumaFrac_loRes_SIMD for a full-sample vertical position: the horizontal pass alone.
static int motionErrorLumaFracH_loRes_AVX2( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter, const int bitDepth )
{
  const __m256i xfilt12   = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[0] ), _mm256_set1_epi16( xFilter[1] ) );
  const __m256i xfilt34   = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[2] ), _mm256_set1_epi16( xFilter[3] ) );
  const __m256i maxSample = _mm256_set1_epi16( ( 1 << bitDepth ) - 1 );
  const __m256i rounding  = _mm256_set1_epi32( 1 << 5 );
  __m256i verror = _mm256_setzero_si256();

  for( int x = 0; x < w; x += 16 )
  {
    for( int y = 0; y < h; y++ )
    {
      const __m256i pred  = horizontal4Tap_AVX2( buf + y * buffStride + x, xfilt12, xfilt34, rounding, maxSample );
      const __m256i org16 = _mm256_loadu_si256( ( const __m256i* ) ( org + y * origStride + x ) );
      const __m256i diff  = _mm256_sub_epi16( pred, org16 );
      verror = _mm256_add_epi32( verror, _mm256_madd_epi16( diff, diff ) );
    }
  }
  return sumOfLanes_AVX2( verror );
}

// MCTF::m_motionErrorLumaFracColumn: the exact errors of num vectors with the same horizontal position. The horizontal
// pass of the union of their rows is done once into a buffer, then the vertical pass per vector, rowOffset[i] rows lower.
static void motionErrorLumaFracColumn_loRes_AVX2( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h,
                                                  const int16_t* xFilter, const int num, const int16_t* const* yFilter, const int* rowOffset, const int bitDepth, int* error )
{
  static constexpr int MAX_SIZE = 32;   // the block size of the subpel search, MCTFUnitSize, is at most 32 (vvencCfg)
  CHECKD( w > MAX_SIZE || h > MAX_SIZE || ( w & 15 ), "Unsupported block size" );

  const __m256i xfilt12   = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[0] ), _mm256_set1_epi16( xFilter[1] ) );
  const __m256i xfilt34   = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[2] ), _mm256_set1_epi16( xFilter[3] ) );
  const __m256i maxSample = _mm256_set1_epi16( ( 1 << bitDepth ) - 1 );
  const __m256i rounding  = _mm256_set1_epi32( 1 << 5 );

  // row r of filtered is the horizontal pass of source row r - 1; a vector needs h + 3 rows from its offset on
  int maxOffset = 0;
  for( int i = 0; i < num; i++ )
  {
    maxOffset = std::max( maxOffset, rowOffset[i] );
  }
  CHECKD( maxOffset > 1, "The vectors of a column span more than two rows" );
  // h + 3 + maxOffset <= MAX_SIZE + 4 rows; unaligned stores, the alignment only avoids split loads
  alignas( 32 ) Pel filtered[( MAX_SIZE + 4 ) * MAX_SIZE];
  for( int r = 0; r < h + 3 + maxOffset; r++ )
  {
    for( int x = 0; x < w; x += 16 )
    {
      _mm256_storeu_si256( ( __m256i* ) &filtered[r * w + x], horizontal4Tap_AVX2( buf + ( r - 1 ) * buffStride + x, xfilt12, xfilt34, rounding, maxSample ) );
    }
  }

  for( int i = 0; i < num; i++ )
  {
    const Pel* rows = filtered + rowOffset[i] * w;
    // a full-sample vertical position takes the filtered rows as they are: output row y is row y + 1
    error[i] = isIdentityFilter4( yFilter[i] ) ? squaredError_AVX2( org, origStride, rows + w, w, w, h )
                                               : verticalPassError_AVX2<false>( org, origStride, rows, w, w, h, yFilter[i], bitDepth );
  }
}
#endif

template<X86_VEXT vext>
int motionErrorLumaFrac_loRes_SIMD( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror )
{
  int error = 0;
  const int base = -1;
  __m128i xerror = _mm_setzero_si128();
  
  CHECK( w & 7, "SIMD blockSize needs to be a multiple of 8" );

  cond_mm_prefetch( ( const char* )   org,                            _MM_HINT_T0 );
  cond_mm_prefetch( ( const char* ) ( buf + base + -1 * buffStride ), _MM_HINT_T0 );

#if USE_AVX2
  if( vext >= AVX2 && ( w & 15 ) == 0 )
  {
    if( isIdentityFilter4( xFilter ) )
    {
      return motionErrorLumaFracV_loRes_AVX2( org, origStride, buf, buffStride, w, h, yFilter, bitDepth );
    }
    if( isIdentityFilter4( yFilter ) )
    {
      return motionErrorLumaFracH_loRes_AVX2( org, origStride, buf, buffStride, w, h, xFilter, bitDepth );
    }

    GCC_WARNING_DISABLE_maybe_uninitialized
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
    __m256i verror = _mm256_setzero_si256();

    for( int x1 = 0; x1 < w; x1 += 16, sourceCol += 16, origCol += 16 )
    {
      const Pel* origRow  = origCol;
      const Pel* rowStart = sourceCol;

#ifdef NDEBUG
      __m256i vsrc0, vsrc1, vsrc2, vsrc3;
#else
      __m256i
        vsrc0 = _mm256_setzero_si256(), vsrc1 = _mm256_setzero_si256(),
        vsrc2 = _mm256_setzero_si256(), vsrc3 = _mm256_setzero_si256();
#endif

      for( int y1 = 0; y1 < h + 3; y1++, rowStart += buffStride )
      {
        cond_mm_prefetch( ( const char* ) ( origRow  + origStride ), _MM_HINT_T0 );
        cond_mm_prefetch( ( const char* ) ( rowStart + buffStride ), _MM_HINT_T0 );

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
          vsrc0 = vsrc1;
          vsrc1 = vsrc2;
          vsrc2 = vsrc3;
          vsrc3 = xsum;
        
          xsum0 = _mm256_set1_epi32( 1 << 5 );
          xsum1 = _mm256_set1_epi32( 1 << 5 );

          xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( yfilt12, _mm256_unpacklo_epi16( vsrc0, vsrc1 ) ) );
          xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( yfilt12, _mm256_unpackhi_epi16( vsrc0, vsrc1 ) ) );

          xsum0 = _mm256_add_epi32( xsum0, _mm256_madd_epi16( yfilt34, _mm256_unpacklo_epi16( vsrc2, vsrc3 ) ) );
          xsum1 = _mm256_add_epi32( xsum1, _mm256_madd_epi16( yfilt34, _mm256_unpackhi_epi16( vsrc2, vsrc3 ) ) );
        
          xsum0 = _mm256_srai_epi32( xsum0, 6 );
          xsum1 = _mm256_srai_epi32( xsum1, 6 );

          xsum  = _mm256_packs_epi32( xsum0, xsum1 );
          xsum  = _mm256_min_epi16  ( xmax, _mm256_max_epi16( xmin, xsum ) );

          __m256i
          xorg  = _mm256_loadu_si256( ( const __m256i * ) origRow );
          origRow += origStride;

          xsum = _mm256_sub_epi16( xsum, xorg );
          xsum = _mm256_madd_epi16( xsum, xsum );

          verror = _mm256_add_epi32( verror, xsum );
        }
        else
        {
          vsrc1 = vsrc2;
          vsrc2 = vsrc3;
          vsrc3 = xsum;
        }
      }
    }

    GCC_WARNING_RESET

    xerror = _mm_add_epi32( _mm256_castsi256_si128( verror ), _mm256_extracti128_si256( verror , 1 ) );
    xerror = _mm_hadd_epi32( xerror, xerror );
    xerror = _mm_hadd_epi32( xerror, xerror );
    error  = _mm_cvtsi128_si32( xerror );

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
      cond_mm_prefetch( ( const char* ) ( origRow + origStride  ), _MM_HINT_T0 );
      cond_mm_prefetch( ( const char* ) ( rowStart + buffStride ), _MM_HINT_T0 );

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
        xerror = _mm_hadd_epi32( xerror, xsum );
        error = _mm_cvtsi128_si32( xerror );

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

  xerror = _mm_hadd_epi32( xerror, xerror );
  xerror = _mm_hadd_epi32( xerror, xerror );
  error  = _mm_cvtsi128_si32( xerror );

  return error;
}

#if USE_AVX2
// applyFrac6tap_SIMD_8x for a block whose width is a multiple of 16: one 256-bit pass per 16 columns instead of two
// 128-bit strips. unpacklo / unpackhi and packs work per 128-bit lane, so the low lane computes columns 0..7 and the
// high lane columns 8..15 exactly as the 128-bit strips do (same multiply-adds, rounding and saturation), and the packs
// leave them in column order. The reads are those of the two strips.
static void applyFrac6tap_AVX2_16x( const Pel* org, const ptrdiff_t origStride, Pel* buf, const ptrdiff_t buffStride, const int bsx, const int bsy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  const __m256i yfilt12 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[1] ), _mm256_set1_epi16( yFilter[2] ) );
  const __m256i yfilt34 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[3] ), _mm256_set1_epi16( yFilter[4] ) );
  const __m256i yfilt56 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[5] ), _mm256_set1_epi16( yFilter[6] ) );

  const __m256i xfilt12 = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[1] ), _mm256_set1_epi16( xFilter[2] ) );
  const __m256i xfilt34 = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[3] ), _mm256_set1_epi16( xFilter[4] ) );
  const __m256i xfilt56 = _mm256_unpacklo_epi16( _mm256_set1_epi16( xFilter[5] ), _mm256_set1_epi16( xFilter[6] ) );

  const __m256i vmax  = _mm256_set1_epi16( ( 1 << bitDepth ) - 1 );
  const __m256i vmin  = _mm256_setzero_si256();
  const __m256i round = _mm256_set1_epi32( 1 << 5 );

  const Pel* srcCol = org - 3 + ( 1 - 3 ) * origStride;
        Pel* dstCol = buf;

  for( int x1 = 0; x1 < bsx; x1 += 16, srcCol += 16, dstCol += 16 )
  {
    const Pel* srcRow = srcCol;
          Pel* dstRow = dstCol;

    __m256i rows[6];   // the last 6 horizontally filtered rows

    for( int y1 = 1; y1 < bsy + 6; y1++, srcRow += origStride )
    {
      const __m256i s1 = _mm256_loadu_si256( ( const __m256i* ) &srcRow[1] );
      const __m256i s2 = _mm256_loadu_si256( ( const __m256i* ) &srcRow[2] );
      const __m256i s3 = _mm256_loadu_si256( ( const __m256i* ) &srcRow[3] );
      const __m256i s4 = _mm256_loadu_si256( ( const __m256i* ) &srcRow[4] );
      const __m256i s5 = _mm256_loadu_si256( ( const __m256i* ) &srcRow[5] );
      const __m256i s6 = _mm256_loadu_si256( ( const __m256i* ) &srcRow[6] );

      __m256i sum0 = round;
      __m256i sum1 = round;
      sum0 = _mm256_add_epi32( sum0, _mm256_madd_epi16( _mm256_unpacklo_epi16( s1, s2 ), xfilt12 ) );
      sum1 = _mm256_add_epi32( sum1, _mm256_madd_epi16( _mm256_unpackhi_epi16( s1, s2 ), xfilt12 ) );
      sum0 = _mm256_add_epi32( sum0, _mm256_madd_epi16( _mm256_unpacklo_epi16( s3, s4 ), xfilt34 ) );
      sum1 = _mm256_add_epi32( sum1, _mm256_madd_epi16( _mm256_unpackhi_epi16( s3, s4 ), xfilt34 ) );
      sum0 = _mm256_add_epi32( sum0, _mm256_madd_epi16( _mm256_unpacklo_epi16( s5, s6 ), xfilt56 ) );
      sum1 = _mm256_add_epi32( sum1, _mm256_madd_epi16( _mm256_unpackhi_epi16( s5, s6 ), xfilt56 ) );
      const __m256i hor = _mm256_packs_epi32( _mm256_srai_epi32( sum0, 6 ), _mm256_srai_epi32( sum1, 6 ) );

      if( y1 < 6 )
      {
        rows[y1] = hor;
        continue;
      }
      rows[0] = rows[1];
      rows[1] = rows[2];
      rows[2] = rows[3];
      rows[3] = rows[4];
      rows[4] = rows[5];
      rows[5] = hor;

      sum0 = round;
      sum1 = round;
      sum0 = _mm256_add_epi32( sum0, _mm256_madd_epi16( yfilt12, _mm256_unpacklo_epi16( rows[0], rows[1] ) ) );
      sum1 = _mm256_add_epi32( sum1, _mm256_madd_epi16( yfilt12, _mm256_unpackhi_epi16( rows[0], rows[1] ) ) );
      sum0 = _mm256_add_epi32( sum0, _mm256_madd_epi16( yfilt34, _mm256_unpacklo_epi16( rows[2], rows[3] ) ) );
      sum1 = _mm256_add_epi32( sum1, _mm256_madd_epi16( yfilt34, _mm256_unpackhi_epi16( rows[2], rows[3] ) ) );
      sum0 = _mm256_add_epi32( sum0, _mm256_madd_epi16( yfilt56, _mm256_unpacklo_epi16( rows[4], rows[5] ) ) );
      sum1 = _mm256_add_epi32( sum1, _mm256_madd_epi16( yfilt56, _mm256_unpackhi_epi16( rows[4], rows[5] ) ) );
      __m256i ver = _mm256_packs_epi32( _mm256_srai_epi32( sum0, 6 ), _mm256_srai_epi32( sum1, 6 ) );
      ver = _mm256_min_epi16( vmax, _mm256_max_epi16( vmin, ver ) );

      _mm256_storeu_si256( ( __m256i* ) dstRow, ver );
      dstRow += buffStride;
    }
  }
  _mm256_zeroupper();
}
#endif

template<X86_VEXT vext>
void applyFrac6tap_SIMD_8x( const Pel* org, const ptrdiff_t origStride, Pel* buf, const ptrdiff_t buffStride, const int bsx, const int bsy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  const int base = -3;

  CHECK( bsx & 7, "SIMD blockSizeX needs to be a multiple of 8" );

#if USE_AVX2
  if( vext >= AVX2 && ( bsx & 15 ) == 0 )
  {
    applyFrac6tap_AVX2_16x( org, origStride, buf, buffStride, bsx, bsy, xFilter, yFilter, bitDepth );
    return;
  }
#endif

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
      __m128i xsrc1 = _vv_loadl_epi64( ( const __m128i* ) &srcRow[1] );
      __m128i xsrc2 = _vv_loadl_epi64( ( const __m128i* ) &srcRow[2] );
      __m128i xsrc3 = _vv_loadl_epi64( ( const __m128i* ) &srcRow[3] );
      __m128i xsrc4 = _vv_loadl_epi64( ( const __m128i* ) &srcRow[4] );
      __m128i xsrc5 = _vv_loadl_epi64( ( const __m128i* ) &srcRow[5] );
      __m128i xsrc6 = _vv_loadl_epi64( ( const __m128i* ) &srcRow[6] );

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

        _vv_storel_epi64( ( __m128i* ) dstRow, xsum );
        dstRow += buffStride;
      }
      else
      {
        xsrc[y1] = xsum;
      }
    }
  }
}

// The 6-tap MCTF filter for chroma: blocks that are a multiple of 8 wide (MCTFUnitSize 16 or 32 at 4:2:0, and 4:2:2 or
// 4:4:4; edge remainders can still be 4 wide) through the 8-wide kernel,
// whose per-column arithmetic is that of the 4-wide one (same multiply-adds, rounding and saturation); 4-wide blocks
// (MCTFUnitSize 8 at 4:2:0) keep the 4-wide kernel.
template<X86_VEXT vext>
void applyFrac6tap_SIMD_chroma( const Pel* org, const ptrdiff_t origStride, Pel* buf, const ptrdiff_t buffStride, const int bsx, const int bsy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  if( ( bsx & 7 ) == 0 )
  {
    applyFrac6tap_SIMD_8x<vext>( org, origStride, buf, buffStride, bsx, bsy, xFilter, yFilter, bitDepth );
  }
  else
  {
    applyFrac6tap_SIMD_4x<vext>( org, origStride, buf, buffStride, bsx, bsy, xFilter, yFilter, bitDepth );
  }
}

static const int32_t xSzm[6] = {0, 1, 20, 336, 5440, 87296};

// works for bit depths up to incl. 12 and power-of-2 block dimensions in both directions
template<X86_VEXT vext>
void applyPlanarCorrectionSIMD( const Pel* refPel, const ptrdiff_t refStride, Pel* dstPel, const ptrdiff_t dstStride, const int32_t w, const int32_t h, const ClpRng& clpRng, const uint16_t motionError )
{
  const int32_t blockSize = w * h;
  const int32_t log2Width = floorLog2 (w);
  const int32_t maxPelVal = clpRng.max();
  const int32_t mWeight   = std::min (512u, (uint32_t) motionError * (uint32_t) motionError);
  const int32_t xSum      = (blockSize * (w - 1)) >> 1;
  int32_t x1yzm = 0,  x2yzm = 0,  ySum = 0;
  int32_t b0, b1, b2;
  int64_t numer, denom;
  int32_t tmp;

#if USE_AVX2
  if (w > 8)
  {
    __m256i vz;
    __m256i vzz = _mm256_set_epi32(0,0,0,0,0,0,0,0);
    __m256i vDst;
    __m256i vRef;
    __m256i vtmp;
    __m256i v16 = _mm256_set_epi16(16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16);
    __m256i v1 = _mm256_set_epi16(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    __m256i vy = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    __m256i vx2yzm = _mm256_set_epi32(0,0,0,0,0,0,0,0);
    __m256i vx1yzm = _mm256_set_epi32(0,0,0,0,0,0,0,0);

    for (int32_t y = 0; y < h; y++) // sum up dot-products between indices and sample diffs
    {
      __m256i vx = _mm256_set_epi16(15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0);

      for (int32_t x = 0; x < w; x += 16)
      {
        vDst = _mm256_loadu_si256( (const __m256i* ) (dstPel+y*dstStride + x ));
        vRef = _mm256_loadu_si256( (const __m256i* ) (refPel+y*refStride + x ));
        vz = _mm256_sub_epi16 (vDst, vRef);
        vtmp = _mm256_madd_epi16(vz,vx);
        vx1yzm = _mm256_add_epi32(vx1yzm,vtmp);
        vtmp = _mm256_madd_epi16(vz,v1);
        vzz = _mm256_add_epi32(vzz,vtmp);
        vtmp = _mm256_madd_epi16(vz,vy);
        vx2yzm = _mm256_add_epi32(vx2yzm,vtmp);
        vx = _mm256_add_epi16(vx,v16);
      }
      vy = _mm256_add_epi16(vy,v1);
    }
    vx1yzm = _mm256_hadd_epi32(vx1yzm,vx1yzm);
    vx1yzm = _mm256_hadd_epi32(vx1yzm,vx1yzm);
    tmp = _mm256_extract_epi32 (vx1yzm, 0);
    x1yzm += tmp;
    tmp = _mm256_extract_epi32 (vx1yzm, 4);
    x1yzm += tmp;

    vx2yzm = _mm256_hadd_epi32(vx2yzm,vx2yzm);
    vx2yzm = _mm256_hadd_epi32(vx2yzm,vx2yzm);
    tmp = _mm256_extract_epi32 (vx2yzm, 0);
    x2yzm += tmp;
    tmp = _mm256_extract_epi32 (vx2yzm, 4);
    x2yzm += tmp;

    vzz = _mm256_hadd_epi32(vzz,vzz);
    vzz = _mm256_hadd_epi32(vzz,vzz);
    tmp = _mm256_extract_epi32 (vzz, 0);
    ySum += tmp;
    tmp = _mm256_extract_epi32 (vzz, 4);
    ySum += tmp;
  }
  else
#endif
  {
    __m128i vz;
    __m128i vzz = _mm_set_epi32(0,0,0,0);
    __m128i vDst;
    __m128i vRef;
    __m128i vtmp;
    __m128i v8 = _mm_set_epi16(8,8,8,8,8,8,8,8);
    __m128i vx2yzm = _mm_set_epi32(0,0,0,0);
    __m128i vx1yzm = _mm_set_epi32(0,0,0,0);
    __m128i v1 = _mm_set_epi16(1,1,1,1,1,1,1,1);

    if (w == 4)
    {
      __m128i vy = _mm_set_epi16(1,1,1,1,0,0,0,0);
      __m128i v2 = _mm_set_epi16(2,2,2,2,2,2,2,2);

      for (int32_t y = 0; y < h; y += 2) // sum up dot-products between indices and sample diffs
      {
        __m128i vx = _mm_set_epi16(3,2,1,0,3,2,1,0);
        vDst = _mm_loadu_si64( ( __m128i const * ) (dstPel+y*dstStride));
        vRef = _mm_loadu_si64( ( __m128i const * ) (refPel+y*refStride));
        __m128i vDsth = _mm_loadu_si64( ( __m128i const * ) (dstPel+(y+1)*dstStride));
        __m128i vRefh = _mm_loadu_si64( ( __m128i const * ) (refPel+(y+1)*refStride));
        vDsth = _mm_bslli_si128 (vDsth,8);
        vRefh = _mm_bslli_si128 (vRefh,8);
        vDst = _mm_or_si128(vDst,vDsth);
        vRef = _mm_or_si128(vRef,vRefh);

        vz = _mm_sub_epi16 (vDst, vRef);
        vtmp = _mm_madd_epi16(vz,vx);
        vx1yzm = _mm_add_epi32(vx1yzm,vtmp);
        vtmp = _mm_madd_epi16(vz,v1);
        vzz = _mm_add_epi32(vzz,vtmp);
        vtmp = _mm_madd_epi16(vz,vy);
        vx2yzm = _mm_add_epi32(vx2yzm,vtmp);
        vy = _mm_add_epi16(vy,v2);
      }
      vx1yzm = _mm_hadd_epi32(vx1yzm,vx1yzm);
      vx1yzm = _mm_hadd_epi32(vx1yzm,vx1yzm);
      tmp = _mm_extract_epi32 (vx1yzm, 0);
      x1yzm += tmp;

      vx2yzm = _mm_hadd_epi32(vx2yzm,vx2yzm);
      vx2yzm = _mm_hadd_epi32(vx2yzm,vx2yzm);
      tmp = _mm_extract_epi32 (vx2yzm, 0);
      x2yzm += tmp;

      vzz = _mm_hadd_epi32(vzz,vzz);
      vzz = _mm_hadd_epi32(vzz,vzz);
      tmp = _mm_extract_epi32 (vzz, 0);
      ySum += tmp;
    }
    else
    {
      __m128i vy = _mm_set_epi16(0,0,0,0,0,0,0,0);
      __m128i v1 = _mm_set_epi16(1,1,1,1,1,1,1,1);

      for (int32_t y = 0; y < h; y++) // sum up dot-products between indices and sample diffs
      {
        __m128i vx = _mm_set_epi16(7,6,5,4,3,2,1,0);

        for (int32_t x = 0; x < w; x += 8)
        {
          vDst = _mm_loadu_si128( ( __m128i const * ) (dstPel+y*dstStride + x ));
          vRef = _mm_loadu_si128( ( __m128i const * ) (refPel+y*refStride + x ));
          vz = _mm_sub_epi16 (vDst, vRef);
          vtmp = _mm_madd_epi16(vz,vx);
          vx1yzm = _mm_add_epi32(vx1yzm,vtmp);
          vtmp = _mm_madd_epi16(vz,v1);
          vzz = _mm_add_epi32(vzz,vtmp);
          vtmp = _mm_madd_epi16(vz,vy);
          vx2yzm = _mm_add_epi32(vx2yzm,vtmp);
          vx = _mm_add_epi16(vx,v8);
        }
        vy = _mm_add_epi16(vy,v1);
      }
      vx1yzm = _mm_hadd_epi32(vx1yzm,vx1yzm);
      vx1yzm = _mm_hadd_epi32(vx1yzm,vx1yzm);
      tmp = _mm_extract_epi32 (vx1yzm, 0);
      x1yzm += tmp;

      vx2yzm = _mm_hadd_epi32(vx2yzm,vx2yzm);
      vx2yzm = _mm_hadd_epi32(vx2yzm,vx2yzm);
      tmp = _mm_extract_epi32 (vx2yzm, 0);
      x2yzm += tmp;

      vzz = _mm_hadd_epi32(vzz,vzz);
      vzz = _mm_hadd_epi32(vzz,vzz);
      tmp = _mm_extract_epi32 (vzz, 0);
      ySum += tmp;
    }
  }

  denom = blockSize * xSzm[log2Width]; // plane-fit parameters, in fixed-point arithmetic
  numer = (int64_t) mWeight * ((int64_t) x1yzm * blockSize - xSum * ySum);
  b1 = int32_t ((numer < 0 ? numer - (denom >> 1) : numer + (denom >> 1)) / denom);
  b1 = (b1 < INT16_MIN ? INT16_MIN : (b1 > INT16_MAX ? INT16_MAX : b1));
  numer = (int64_t) mWeight * ((int64_t) x2yzm * blockSize - xSum * ySum);
  b2 = int32_t ((numer < 0 ? numer - (denom >> 1) : numer + (denom >> 1)) / denom);
  b2 = (b2 > INT16_MAX ? INT16_MAX : (b2 < INT16_MIN ? INT16_MIN : b2));
  b0 = (mWeight * ySum - (b1 + b2) * xSum + (blockSize >> 1)) >> (log2Width << 1);

  if (b0 == 0 && b1 == 0 && b2 == 0) return;

#if USE_AVX2
  if (w > 8)
  {
    __m256i vb0 = _mm256_set1_epi32 (b0);
    __m256i vb1 = _mm256_set1_epi16 ((int16_t) b1);
    __m256i vb2 = _mm256_set1_epi16 ((int16_t) b2);
    __m256i vy = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    __m256i v1 = _mm256_set_epi16(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    __m256i v0 = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    __m256i v16 = _mm256_set_epi16(16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16);
    __m256i v256 = _mm256_set_epi32 (256,256,256,256,256,256,256,256);

    __m256i vpelmin   = _mm256_set1_epi16(0);
    __m256i vpelmax   = _mm256_set1_epi16(maxPelVal);

    for (int32_t y = 0; y < h; y++) // perform deblocking by adding fitted correction plane
    {
      __m256i vx = _mm256_set_epi16(15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0);

      for (int32_t x = 0; x < w; x += 16)
      {
        __m256i vDst = _mm256_loadu_si256( ( __m256i const * ) (dstPel+y*dstStride + x ));

        __m256i vDstl = _mm256_unpacklo_epi16 (vDst,v0);
        __m256i vDsth = _mm256_unpackhi_epi16 (vDst,v0);

        //const int32_t p = (b0 + b1 * x + b2 * y + 256) >> 9; // fixed-point plane corrector
        __m256i vtmph =_mm256_mulhi_epi16(vb1,vx);
        __m256i vtmpl =_mm256_mullo_epi16(vb1,vx);
        __m256i vb1l  = _mm256_unpacklo_epi16(vtmpl,vtmph);
        __m256i vb1h  = _mm256_unpackhi_epi16(vtmpl,vtmph);

        vtmph =_mm256_mulhi_epi16(vb2,vy);
        vtmpl =_mm256_mullo_epi16(vb2,vy);
        __m256i vb2l  = _mm256_unpacklo_epi16(vtmpl,vtmph);
        __m256i vb2h  = _mm256_unpackhi_epi16(vtmpl,vtmph);

        vb1l = _mm256_add_epi32(vb1l,vb2l);
        vb1h = _mm256_add_epi32(vb1h,vb2h);

        vb1l = _mm256_add_epi32(vb1l,vb0);
        vb1h = _mm256_add_epi32(vb1h,vb0);

        vb1l = _mm256_add_epi32(vb1l,v256);
        vb1h = _mm256_add_epi32(vb1h,v256);

        vb1l = _mm256_srai_epi32 (vb1l,9);
        vb1h = _mm256_srai_epi32 (vb1h,9);

        vDstl = _mm256_sub_epi32 (vDstl,vb1l);
        vDsth = _mm256_sub_epi32 (vDsth,vb1h);

        vDst = _mm256_packs_epi32 (vDstl,vDsth);
        vDst = _mm256_min_epi16( vpelmax, _mm256_max_epi16( vpelmin, vDst ) );
        _mm256_storeu_si256 ((__m256i* ) (dstPel+y*dstStride + x ) ,vDst);

        vx = _mm256_add_epi16(vx,v16);
      }
      vy = _mm256_add_epi16(vy,v1);
    }
  }
  else
#endif
  {
    __m128i vb0 = _mm_set1_epi32 (b0);
    __m128i vb1 = _mm_set1_epi16 ((int16_t) b1);
    __m128i vb2 = _mm_set1_epi16 ((int16_t) b2);
    __m128i v0 = _mm_set_epi16(0,0,0,0,0,0,0,0);
    __m128i v256 = _mm_set_epi32 (256,256,256,256);

    __m128i vpelmin = _mm_set1_epi16(0);
    __m128i vpelmax = _mm_set1_epi16(maxPelVal);

    if (w == 4)
    {
      __m128i vy = _mm_set_epi16(1,1,1,1,0,0,0,0);
      __m128i v2 = _mm_set_epi16(2,2,2,2,2,2,2,2);

      for (int32_t y = 0; y < h; y += 2) // perform deblocking by adding fitted correction plane
      {
        __m128i vx = _mm_set_epi16(3,2,1,0,3,2,1,0);

        {
          __m128i vDst = _mm_loadu_si64( ( __m128i const * ) (dstPel+y*dstStride));
          __m128i vDsth = _mm_loadu_si64( ( __m128i const * ) (dstPel+(y+1)*dstStride));
          vDsth = _mm_bslli_si128 (vDsth,8);
          vDst = _mm_or_si128(vDst,vDsth);
          __m128i vDstl = _mm_unpacklo_epi16 (vDst,v0);
          vDsth = _mm_unpackhi_epi16 (vDst,v0);

          //const int32_t p = (b0 + b1 * x + b2 * y + 256) >> 9; // fixed-point plane corrector
          __m128i vtmph =_mm_mulhi_epi16(vb1,vx);
          __m128i vtmpl =_mm_mullo_epi16(vb1,vx);
          __m128i vb1l  = _mm_unpacklo_epi16(vtmpl,vtmph);
          __m128i vb1h  = _mm_unpackhi_epi16(vtmpl,vtmph);
          vtmph =_mm_mulhi_epi16(vb2,vy);
          vtmpl =_mm_mullo_epi16(vb2,vy);
          __m128i vb2l  = _mm_unpacklo_epi16(vtmpl,vtmph);
          __m128i vb2h  = _mm_unpackhi_epi16(vtmpl,vtmph);

          vb1l = _mm_add_epi32(vb1l,vb2l);
          vb1h = _mm_add_epi32(vb1h,vb2h);

          vb1l = _mm_add_epi32(vb1l,vb0);
          vb1h = _mm_add_epi32(vb1h,vb0);

          vb1l = _mm_add_epi32(vb1l,v256);
          vb1h = _mm_add_epi32(vb1h,v256);

          vb1l = _mm_srai_epi32 (vb1l,9);
          vb1h = _mm_srai_epi32 (vb1h,9);
          vDstl = _mm_sub_epi32 (vDstl,vb1l);
          vDsth = _mm_sub_epi32 (vDsth,vb1h);

          vDst = _mm_packs_epi32 (vDstl,vDsth);
          vDst = _mm_min_epi16( vpelmax, _mm_max_epi16( vpelmin, vDst ) );

          _mm_storeu_si64 ((__m128i* ) (dstPel+y*dstStride),vDst);
          _mm_storeu_si64 ((__m128i* ) (dstPel+(y+1)*dstStride),_mm_srli_si128(vDst,8));
        }
        vy = _mm_add_epi16(vy,v2);
      }
    }
    else
    {
      __m128i vy = _mm_set_epi16(0,0,0,0,0,0,0,0);
      __m128i v1 = _mm_set_epi16(1,1,1,1,1,1,1,1);
      __m128i v8 = _mm_set_epi16(8,8,8,8,8,8,8,8);

      for (int32_t y = 0; y < h; y++) // perform deblocking by adding fitted correction plane
      {
        __m128i vx = _mm_set_epi16(7,6,5,4,3,2,1,0);

        for (int32_t x = 0; x < w; x += 8)
        {
          __m128i vDst = _mm_loadu_si128( ( __m128i const * ) (dstPel+y*dstStride + x ));
          __m128i vDstl = _mm_unpacklo_epi16 (vDst,v0);
          __m128i vDsth = _mm_unpackhi_epi16 (vDst,v0);

          //const int32_t p = (b0 + b1 * x + b2 * y + 256) >> 9; // fixed-point plane corrector
          __m128i vtmph =_mm_mulhi_epi16(vb1,vx);
          __m128i vtmpl =_mm_mullo_epi16(vb1,vx);
          __m128i vb1l  = _mm_unpacklo_epi16(vtmpl,vtmph);
          __m128i vb1h  = _mm_unpackhi_epi16(vtmpl,vtmph);
          vtmph =_mm_mulhi_epi16(vb2,vy);
          vtmpl =_mm_mullo_epi16(vb2,vy);
          __m128i vb2l  = _mm_unpacklo_epi16(vtmpl,vtmph);
          __m128i vb2h  = _mm_unpackhi_epi16(vtmpl,vtmph);

          vb1l = _mm_add_epi32(vb1l,vb2l);
          vb1h = _mm_add_epi32(vb1h,vb2h);

          vb1l = _mm_add_epi32(vb1l,vb0);
          vb1h = _mm_add_epi32(vb1h,vb0);

          vb1l = _mm_add_epi32(vb1l,v256);
          vb1h = _mm_add_epi32(vb1h,v256);

          vb1l = _mm_srai_epi32 (vb1l,9);
          vb1h = _mm_srai_epi32 (vb1h,9);
          vDstl = _mm_sub_epi32 (vDstl,vb1l);
          vDsth = _mm_sub_epi32 (vDsth,vb1h);

          vDst = _mm_packs_epi32 (vDstl,vDsth);

          vDst = _mm_min_epi16( vpelmax, _mm_max_epi16( vpelmin, vDst ) );
          _mm_storeu_si128 ((__m128i* )  (dstPel+y*dstStride + x ) ,vDst);

          vx = _mm_add_epi16(vx,v8);
        }
        vy = _mm_add_epi16(vy,v1);
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
  float vsw [2 * VVENC_MCTF_RANGE] = { 0.0f, };
  float vww [2 * VVENC_MCTF_RANGE] = { 0.0f, };

  int minError = INT32_MAX;

  for( int i = 0; i < numRefs; i++ )
  {
    const ptrdiff_t refStride = w;
    const Pel *     refPel    = correctedPics[i];
    __m128i xvar = _mm_setzero_si128(), xdiffsum = _mm_setzero_si128();

    // xvar is a sum of squares of differences of 10bit unsigned values
    //                             \----------------------------------/
    //                                  still 10 bit
    //                  \---------------------------------------------/
    //                              up to 20 bit
    // leaving 12 bit (2^6*2^6) for the sum, which is ok for blocks up to 64x64, with w and h being usually 8 or 16 (2^3 or 2^4)
    // diffsum has double the number of entries, so one less bit

    if( ( w & 7 ) == 4 )
    {
      const __m128i xshufr = _mm_setr_epi8( 0, 1, 2, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );

      for( int y1 = 0; y1 < h; y1++ )
      {
        // probably only one iteration anyway. the case of w==4 does not occur very often (probably only for chroma, when MCTFUnitSize=8, i.e for res < 720p).
        for( int x1 = 0; x1 < w; x1 += 4 )
        {
          const Pel *pix0 = srcPel + srcStride * y1 + x1;
          const Pel *ref0 = refPel + refStride * y1 + x1;
          const Pel *pixr = pix0 + 1;
          const Pel* refr = ref0 + 1;
          const Pel* pixd = pix0 + srcStride;
          const Pel* refd = ref0 + refStride;

          __m128i xpix0 = _vv_loadl_epi64( ( const __m128i* ) pix0 );
          __m128i xref0 = _vv_loadl_epi64( ( const __m128i* ) ref0 );
          __m128i xpixr = _vv_loadl_epi64( ( const __m128i* ) pixr );
          __m128i xrefr = _vv_loadl_epi64( ( const __m128i* ) refr );

          __m128i xdiff = _mm_sub_epi16( xpix0, xref0 );
          xvar = _mm_add_epi32( xvar, _mm_madd_epi16( xdiff, xdiff ) );

          if( y1 + 1 != h )
          {
            __m128i xpixd  = _vv_loadl_epi64( ( const __m128i* ) pixd );
            __m128i xrefd  = _vv_loadl_epi64( ( const __m128i* ) refd );
            __m128i xdiffd = _mm_sub_epi16( xpixd, xrefd );
            xdiffd = _mm_sub_epi16( xdiffd, xdiff );
            xdiffsum = _mm_add_epi32( xdiffsum, _mm_madd_epi16( xdiffd, xdiffd ) );
          }

          if( x1 + 4 == w )
          {
            xpix0 = _mm_shuffle_epi8( xpix0, xshufr );
            xpixr = _mm_shuffle_epi8( xpixr, xshufr );
            xref0 = _mm_shuffle_epi8( xref0, xshufr );
            xrefr = _mm_shuffle_epi8( xrefr, xshufr );

            xdiff = _mm_sub_epi16( xpix0, xref0 );
          }

          __m128i xdiffr = _mm_sub_epi16( xpixr, xrefr );
          xdiffr = _mm_sub_epi16( xdiffr, xdiff );
          xdiffsum = _mm_add_epi32( xdiffsum, _mm_madd_epi16( xdiffr, xdiffr ) );
        }
      }
    }
    else
    {
      const __m128i xshufr = _mm_setr_epi8( 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, -1, -1 );

      for( int y1 = 0; y1 < h; y1++ )
      {
        for( int x1 = 0; x1 < w; x1 += 8 )
        {
          const Pel *pix0 = srcPel + srcStride * y1 + x1;
          const Pel *ref0 = refPel + refStride * y1 + x1;
          const Pel *pixr = pix0 + 1;
          const Pel* refr = ref0 + 1;
          const Pel* pixd = pix0 + srcStride;
          const Pel* refd = ref0 + refStride;

          __m128i xpix0 = _mm_loadu_si128( ( const __m128i* ) pix0 );
          __m128i xref0 = _mm_loadu_si128( ( const __m128i* ) ref0 );
          __m128i xpixr = _mm_loadu_si128( ( const __m128i* ) pixr );
          __m128i xrefr = _mm_loadu_si128( ( const __m128i* ) refr );

          __m128i xdiff = _mm_sub_epi16( xpix0, xref0 );
          xvar = _mm_add_epi32( xvar, _mm_madd_epi16( xdiff, xdiff ) );

          if( y1 + 1 != h )
          {
            __m128i xpixd  = _mm_loadu_si128( ( const __m128i* ) pixd );
            __m128i xrefd  = _mm_loadu_si128( ( const __m128i* ) refd );
            __m128i xdiffd = _mm_sub_epi16( xpixd, xrefd );
            xdiffd = _mm_sub_epi16( xdiffd, xdiff );
            xdiffsum = _mm_add_epi32( xdiffsum, _mm_madd_epi16( xdiffd, xdiffd ) );
          }

          if( x1 + 8 == w )
          {
            xpix0 = _mm_shuffle_epi8( xpix0, xshufr );
            xpixr = _mm_shuffle_epi8( xpixr, xshufr );
            xref0 = _mm_shuffle_epi8( xref0, xshufr );
            xrefr = _mm_shuffle_epi8( xrefr, xshufr );

            xdiff = _mm_sub_epi16( xpix0, xref0 );
          }

          __m128i xdiffr = _mm_sub_epi16( xpixr, xrefr );
          xdiffr = _mm_sub_epi16( xdiffr, xdiff );
          xdiffsum = _mm_add_epi32( xdiffsum, _mm_madd_epi16( xdiffr, xdiffr ) );
        }
      }
    }

    xvar = _mm_hadd_epi32( xvar, xdiffsum );
    xvar = _mm_hadd_epi32( xvar, xvar );
    int64_t variance = _mm_cvtsi128_si32( xvar );
    int64_t diffsum  = _mm_extract_epi32( xvar, 1 );
    variance *= (int64_t) 1 << (2*(10-clpRng.bd));
    diffsum  *= (int64_t) 1 << (2*(10-clpRng.bd));

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

#if USE_AVX2
  // AVX2: 8 pixels per step, with the per-pixel operations of the 4-pixel loop below in the same order, so every pixel
  // gets the same value (the default build has no FMA contraction; an FMA-enabled -march build fuses the same
  // multiply-add in both loops).
  const int w8 = vext >= AVX2 ? ( w & ~7 ) : 0;
  __m256 vdivisor[2 * VVENC_MCTF_RANGE];
  __m256 vrefweight[2 * VVENC_MCTF_RANGE];
  for( int i = 0; w8 && i < numRefs; i++ )
  {
    vdivisor[i]   = _mm256_set1_ps( -vsw[i] * 1024.0f );
    vrefweight[i] = _mm256_set1_ps( vww[i] );
  }
  const __m256i vmaxSample = _mm256_set1_epi32( clpRng.max() );
#else
  const int w8 = 0;
#endif

  for( int y = 0; y < h; y++ )
  {
#if USE_AVX2
    for( int x = 0; x < w8; x += 8 )
    {
      const __m256i vorgi = _mm256_cvtepi16_epi32( _mm_loadu_si128( ( const __m128i* ) ( srcPel + srcStride * y + x ) ) );
      __m256 vtws  = _mm256_set1_ps( 1.0f );
      __m256 vnewv = _mm256_cvtepi32_ps( vorgi );

      for( int i = 0; i < numRefs; i++ )
      {
        const __m256i vrefi = _mm256_cvtepi16_epi32( _mm_loadu_si128( ( const __m128i* ) ( correctedPics[i] + y * w + x ) ) );
        // the 16-bit difference squared, as below (the high halves of the 32-bit lanes are 0 in both)
        const __m256i vdifi = _mm256_sub_epi16( vrefi, vorgi );
        __m256 vwght = _mm256_div_ps( _mm256_cvtepi32_ps( _mm256_madd_epi16( vdifi, vdifi ) ), vdivisor[i] );
        vwght = _mm256_add_ps( vwght, _mm256_set1_ps( 1.0f ) );
        for( int k = 0; k < 10; k++ )
        {
          vwght = _mm256_mul_ps( vwght, vwght );   // the fast exp below: ( 1 + x / 1024 )^1024
        }
        vwght = _mm256_mul_ps( vwght, vrefweight[i] );
        vnewv = _mm256_add_ps( vnewv, _mm256_mul_ps( vwght, _mm256_cvtepi32_ps( vrefi ) ) );
        vtws  = _mm256_add_ps( vtws, vwght );
      }

      vnewv = _mm256_div_ps( vnewv, vtws );
      vnewv = _mm256_add_ps( vnewv, _mm256_set1_ps( 0.5f ) );
      vnewv = _mm256_round_ps( vnewv, ( SIMDE_MM_FROUND_TO_ZERO | SIMDE_MM_FROUND_NO_EXC ) );
      __m256i vnewi = _mm256_cvtps_epi32( vnewv );
      vnewi = _mm256_min_epi32( _mm256_max_epi32( vnewi, _mm256_setzero_si256() ), vmaxSample );
      // packs works per 128-bit half: pack the two halves in order
      _mm_storeu_si128( ( __m128i* ) ( dstPel + dstStride * y + x ), _mm_packs_epi32( _mm256_castsi256_si128( vnewi ), _mm256_extracti128_si256( vnewi, 1 ) ) );
    }
#endif
    for( int x = w8; x < w; x += 4 )
    {
      __m128i vorgi = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( __m128i* ) ( srcPel + srcStride * y + x ) ) );
      __m128  vorg  = _mm_cvtepi32_ps( vorgi );
      //const Pel orgVal  = *( srcPel + srcStride * y + x );
      __m128  vtws  = _mm_set1_ps( 1.0f );
      //float temporalWeightSum = 1.0;
      //float newVal = ( float ) orgVal;
      __m128  vnewv = vorg;

      for( int i = 0; i < numRefs; i++ )
      {
        const Pel* pCorrectedPelPtr = correctedPics[i] + y * w + x;
        __m128i vrefi = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( __m128i* ) pCorrectedPelPtr ) );
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
      vnewv = _mm_round_ps( vnewv, ( SIMDE_MM_FROUND_TO_ZERO | SIMDE_MM_FROUND_NO_EXC ) );
      //Pel sampleVal = ( Pel ) ( newVal + 0.5 );
      __m128i vnewi = _mm_cvtps_epi32( vnewv );

      vnewi = _mm_max_epi32( vnewi, _mm_setzero_si128() );
      vnewi = _mm_min_epi32( vnewi, _mm_set1_epi32( clpRng.max() ) );
      //sampleVal = ( sampleVal < 0 ? 0 : ( sampleVal > maxSampleValue ? maxSampleValue : sampleVal ) );
      
      vnewi = _mm_packs_epi32( vnewi, vnewi );
      //*( dstPel + srcStride * y + x ) = sampleVal;
      _vv_storel_epi64( ( __m128i * ) ( dstPel + dstStride * y + x ), vnewi );
    }
  }
}

template<X86_VEXT vext>
double calcVarSse( const Pel* org, const ptrdiff_t origStride, const int w, const int h )
{
  // calculate average
  __m128i xavg32 = _mm_setzero_si128();
  __m128i xavg16 = _mm_setzero_si128();
  const __m128i xone = _mm_set1_epi16( 1 );
  for( int y1 = 0; y1 < h; y1++ )
  {
    xavg16 = _mm_setzero_si128();
    for( int x1 = 0; x1 < w; x1 += 8 )
    {
      xavg16 = _mm_add_epi16( xavg16, _mm_loadu_si128( ( const __m128i* ) ( org + x1 + y1 * origStride ) ) );
    }
    xavg32 = _mm_add_epi32( xavg32, _mm_madd_epi16( xone, xavg16 ) );
  }

  xavg32 = _mm_hadd_epi32( xavg32, xavg32 );
  xavg32 = _mm_hadd_epi32( xavg32, xavg32 );
  xavg32 = _mm_shuffle_epi32( xavg32, 0 );
  int shift = Log2( w ) + Log2( h ) - 4;
  xavg32 = _mm_srai_epi32( xavg32, shift );
  xavg16 = _mm_packs_epi32( xavg32, xavg32 );

  // calculate variance
  __m128i xvar = _mm_setzero_si128();
  for( int y1 = 0; y1 < h; y1++ )
  {
    for( int x1 = 0; x1 < w; x1 += 8 )
    {
      __m128i xpix = _mm_loadu_si128( ( const __m128i* ) ( org + x1 + y1 * origStride ) );
      xpix = _mm_slli_epi16( xpix, 4 );
      xpix = _mm_sub_epi16( xpix, xavg16 );
      xpix = _mm_madd_epi16( xpix, xpix );
      xvar = _mm_add_epi64( xvar, _mm_cvtepi32_epi64( xpix ) );
      xvar = _mm_add_epi64( xvar, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( xpix, xpix ) ) );
    }
  }

  xvar = _mm_add_epi64( xvar, _mm_unpackhi_epi64( xvar, xvar ) );

  return _mm_cvtsi128_si64( xvar ) / 256.0;
}

// One strip of W (8 or 4) columns of applyFrac4tap_SIMD.
template<int W>
static void applyFrac4tapColumns( const Pel* srcCol, const ptrdiff_t origStride, Pel* dstCol, const ptrdiff_t buffStride, const int bsy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  static_assert( W == 8 || W == 4, "4 or 8 columns per vector" );

  const __m128i xfilt01 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[0] ), _mm_set1_epi16( xFilter[1] ) );
  const __m128i xfilt23 = _mm_unpacklo_epi16( _mm_set1_epi16( xFilter[2] ), _mm_set1_epi16( xFilter[3] ) );
  const __m128i yfilt01 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[0] ), _mm_set1_epi16( yFilter[1] ) );
  const __m128i yfilt23 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[2] ), _mm_set1_epi16( yFilter[3] ) );
  const __m128i vrnd    = _mm_set1_epi32( 1 << 5 );
  const __m128i vmax    = _mm_set1_epi16( ( 1 << bitDepth ) - 1 );
  const __m128i vmin    = _mm_setzero_si128();

  auto load = []( const Pel* p ) { return W == 8 ? _mm_loadu_si128( ( const __m128i* ) p ) : _vv_loadl_epi64( ( const __m128i* ) p ); };
  // ( sum( a[i] * f[i] ) + 32 ) >> 6 over the 4 taps, for W samples
  auto filter = [&]( __m128i a0, __m128i a1, __m128i a2, __m128i a3, __m128i f01, __m128i f23 )
  {
    __m128i lo = _mm_add_epi32( vrnd, _mm_madd_epi16( _mm_unpacklo_epi16( a0, a1 ), f01 ) );
    lo         = _mm_srai_epi32( _mm_add_epi32( lo, _mm_madd_epi16( _mm_unpacklo_epi16( a2, a3 ), f23 ) ), 6 );
    if( W == 4 )
    {
      return _mm_packs_epi32( lo, lo );
    }
    __m128i hi = _mm_add_epi32( vrnd, _mm_madd_epi16( _mm_unpackhi_epi16( a0, a1 ), f01 ) );
    hi         = _mm_srai_epi32( _mm_add_epi32( hi, _mm_madd_epi16( _mm_unpackhi_epi16( a2, a3 ), f23 ) ), 6 );
    return _mm_packs_epi32( lo, hi );
  };

  auto filterRow = [&]( const Pel* p ) { return filter( load( p ), load( p + 1 ), load( p + 2 ), load( p + 3 ), xfilt01, xfilt23 ); };

  // the last 4 horizontally filtered rows
  __m128i rows[4] = { filterRow( srcCol ), filterRow( srcCol + origStride ), filterRow( srcCol + 2 * origStride ), vmin };
  srcCol += 3 * origStride;

  for( int y = 0; y < bsy; y++, srcCol += origStride, dstCol += buffStride )
  {
    rows[3] = filterRow( srcCol );
    const __m128i ver = _mm_min_epi16( vmax, _mm_max_epi16( vmin, filter( rows[0], rows[1], rows[2], rows[3], yfilt01, yfilt23 ) ) );
    if( W == 8 )
    {
      _mm_storeu_si128( ( __m128i* ) dstCol, ver );
    }
    else
    {
      _vv_storel_epi64( ( __m128i* ) dstCol, ver );
    }
    rows[0] = rows[1];
    rows[1] = rows[2];
    rows[2] = rows[3];
  }
}

// 4-tap filter for the low-resolution MCTF (MCTF::m_lowResFltApply), same arithmetic as applyFrac8Core_4Tap: horizontal
// pass to 16 bit with rounding, vertical pass with rounding and clipping
template<X86_VEXT vext>
void applyFrac4tap_SIMD( const Pel* org, const ptrdiff_t origStride, Pel* buf, const ptrdiff_t buffStride, const int bsx, const int bsy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  CHECK( bsx & 3, "SIMD blockSizeX needs to be a multiple of 4" );

  // the taps cover the samples -1 ... 2 around each position, horizontally and vertically
  const Pel* src = org - 1 - origStride;
  int x = 0;
  for( ; x + 8 <= bsx; x += 8 )
  {
    applyFrac4tapColumns<8>( src + x, origStride, buf + x, buffStride, bsy, xFilter, yFilter, bitDepth );
  }
  if( x < bsx )
  {
    applyFrac4tapColumns<4>( src + x, origStride, buf + x, buffStride, bsy, xFilter, yFilter, bitDepth );
  }
}

template<X86_VEXT vext>
void MCTF::_initMCTF_X86()
{
  m_motionErrorLumaInt8     = motionErrorLumaInt_SIMD<vext>;
  m_motionErrorLumaFrac8[0] = motionErrorLumaFrac_SIMD<vext>;
  m_motionErrorLumaFrac8[1] = motionErrorLumaFrac_loRes_SIMD<vext>;
#if USE_AVX2
  if( vext >= AVX2 )
  {
    m_motionErrorLumaFracColumn = motionErrorLumaFracColumn_loRes_AVX2;
  }
#endif

  m_applyFrac[0][0] = applyFrac6tap_SIMD_8x<vext>;
  m_applyFrac[1][0] = applyFrac6tap_SIMD_chroma<vext>;
  m_applyFrac[0][1] = applyFrac4tap_SIMD<vext>;
  m_applyFrac[1][1] = applyFrac4tap_SIMD<vext>;

  m_applyPlanarCorrection = applyPlanarCorrectionSIMD<vext>;
  m_applyBlock            = applyBlockSIMD<vext>;
  m_calcVar               = calcVarSse<vext>;
}

template
void MCTF::_initMCTF_X86<SIMDX86>();

}
#endif
//! \}

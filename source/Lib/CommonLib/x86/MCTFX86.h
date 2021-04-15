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
  const int base = x + ( dx >> 4 ) - 3;
  
  CHECK( bs & 7, "SIMD blockSize needs to be a multiple of 8" );

  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

#if USE_AVX2
  __m256i vfilt12 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[1] ), _mm256_set1_epi16( yFilter[2] ) );
  __m256i vfilt34 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[3] ), _mm256_set1_epi16( yFilter[4] ) );
  __m256i vfilt56 = _mm256_unpacklo_epi16( _mm256_set1_epi16( yFilter[5] ), _mm256_set1_epi16( yFilter[6] ) );

  __m256i vfilt1 = _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) xFilter ) );
          vfilt1 = _mm256_inserti128_si256( vfilt1, _mm256_castsi256_si128( vfilt1 ), 1 );
  __m256i vmax   = _mm256_set1_epi32( maxSampleValue );
  __m256i vmin   = _mm256_setzero_si256();
  
  const int yOffset    = y + 1 + ( dy >> 4 ) - 3;
  const Pel* sourceCol = buffOrigin + base + yOffset * buffStride;
  const Pel* origCol   = origOrigin + y * origStride + x;

  for( int x1 = 0; x1 < bs; x1 += 8, sourceCol += 8, origCol += 8 )
  {
    const Pel* origRow  = origCol;
    const Pel* rowStart = sourceCol;

    __m128i xsrc[6];

    for( int y1 = 1; y1 < bs + 6; y1++, rowStart += buffStride )
    {
      __m256i vsrc0 = _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) &rowStart[0] ) );
      __m256i vsrc1 = _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) &rowStart[1] ) );
      __m256i vsrc2 = _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) &rowStart[2] ) );
      __m256i vsrc3 = _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * ) &rowStart[3] ) );

      vsrc0 = _mm256_inserti128_si256( vsrc0, _mm_loadu_si128( ( const __m128i * ) &rowStart[4] ), 1 );
      vsrc1 = _mm256_inserti128_si256( vsrc1, _mm_loadu_si128( ( const __m128i * ) &rowStart[5] ), 1 );
      vsrc2 = _mm256_inserti128_si256( vsrc2, _mm_loadu_si128( ( const __m128i * ) &rowStart[6] ), 1 );
      vsrc3 = _mm256_inserti128_si256( vsrc3, _mm_loadu_si128( ( const __m128i * ) &rowStart[7] ), 1 );

      vsrc0 = _mm256_madd_epi16( vsrc0, vfilt1 );
      vsrc1 = _mm256_madd_epi16( vsrc1, vfilt1 );
      vsrc2 = _mm256_madd_epi16( vsrc2, vfilt1 );
      vsrc3 = _mm256_madd_epi16( vsrc3, vfilt1 );

      vsrc0 = _mm256_hadd_epi32( vsrc0, vsrc1 );
      vsrc2 = _mm256_hadd_epi32( vsrc2, vsrc3 );

      vsrc0 = _mm256_hadd_epi32( vsrc0, vsrc2 );

      __m256i
      vsum = _mm256_add_epi32  ( vsrc0, _mm256_set1_epi32( 1 << 5 ) );
      vsum = _mm256_srai_epi32 ( vsum,  6 );
      vsum = _mm256_min_epi32  ( vmax,  _mm256_max_epi32( vmin, vsum ) );

      __m128i
      xsum = _mm256_cvtepi32_epi16x( vsum );

      if( y1 >= 6 )
      {
        xsrc[0] = xsrc[1];
        xsrc[1] = xsrc[2];
        xsrc[2] = xsrc[3];
        xsrc[3] = xsrc[4];
        xsrc[4] = xsrc[5];
        xsrc[5] = xsum;

        __m256i vsrc12 = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_unpacklo_epi16( xsrc[0], xsrc[1] ) ),
                                                                          _mm_unpackhi_epi16( xsrc[0], xsrc[1] ), 1 );
        __m256i vsrc34 = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_unpacklo_epi16( xsrc[2], xsrc[3] ) ),
                                                                          _mm_unpackhi_epi16( xsrc[2], xsrc[3] ), 1 );
        __m256i vsrc56 = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_unpacklo_epi16( xsrc[4], xsrc[5] ) ),
                                                                          _mm_unpackhi_epi16( xsrc[4], xsrc[5] ), 1 );
      
        __m256i
        vsum = _mm256_set1_epi32( 1 << 5 );
        vsum = _mm256_add_epi32( vsum, _mm256_madd_epi16( vfilt12, vsrc12 ) );
        vsum = _mm256_add_epi32( vsum, _mm256_madd_epi16( vfilt34, vsrc34 ) );
        vsum = _mm256_add_epi32( vsum, _mm256_madd_epi16( vfilt56, vsrc56 ) );
        
        vsum = _mm256_srai_epi32 ( vsum, 6 );
        vsum = _mm256_min_epi32  ( vmax, _mm256_max_epi32( vmin, vsum ) );

        __m128i
        xorg = _mm_loadu_si128( ( const __m128i * ) origRow );
        origRow += origStride;
      
        xsum = _mm256_cvtepi32_epi16x( vsum );

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
#else
  __m128i xfilt12 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[1] ), _mm_set1_epi16( yFilter[2] ) );
  __m128i xfilt34 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[3] ), _mm_set1_epi16( yFilter[4] ) );
  __m128i xfilt56 = _mm_unpacklo_epi16( _mm_set1_epi16( yFilter[5] ), _mm_set1_epi16( yFilter[6] ) );
  
  __m128i xfilt1 = _mm_loadu_si128( ( const __m128i * ) xFilter );
  __m128i xmax   = _mm_set1_epi32( maxSampleValue );
  __m128i xmin   = _mm_setzero_si128();
  
  const int yOffset    = y + 1 + ( dy >> 4 ) - 3;
  const Pel* sourceCol = buffOrigin + base + yOffset * buffStride;
  const Pel* origCol   = origOrigin + y * origStride + x;

  for( int x1 = 0; x1 < bs; x1 += 4, sourceCol += 4, origCol += 4 )
  {
    const Pel* origRow  = origCol;
    const Pel* rowStart = sourceCol;

    __m128i xsrc[6];

    for( int y1 = 1; y1 < bs + 6; y1++, rowStart += buffStride )
    {
      __m128i xsrc0 = _mm_loadu_si128( ( const __m128i * ) &rowStart[0] );
      __m128i xsrc1 = _mm_loadu_si128( ( const __m128i * ) &rowStart[1] );
      __m128i xsrc2 = _mm_loadu_si128( ( const __m128i * ) &rowStart[2] );
      __m128i xsrc3 = _mm_loadu_si128( ( const __m128i * ) &rowStart[3] );

      xsrc0 = _mm_madd_epi16( xsrc0, xfilt1 );
      xsrc1 = _mm_madd_epi16( xsrc1, xfilt1 );
      xsrc2 = _mm_madd_epi16( xsrc2, xfilt1 );
      xsrc3 = _mm_madd_epi16( xsrc3, xfilt1 );

      xsrc0 = _mm_hadd_epi32( xsrc0, xsrc1 );
      xsrc2 = _mm_hadd_epi32( xsrc2, xsrc3 );

      xsrc0 = _mm_hadd_epi32( xsrc0, xsrc2 );

      __m128i
      xsum = _mm_add_epi32  ( xsrc0, _mm_set1_epi32( 1 << 5 ) );
      xsum = _mm_srai_epi32 ( xsum,  6 );
      xsum = _mm_min_epi32  ( xmax,  _mm_max_epi32( xmin, xsum ) );
      xsum = _mm_packs_epi32( xsum,  _mm_setzero_si128() );

      if( y1 >= 6 )
      {
        xsrc[0] = xsrc[1];
        xsrc[1] = xsrc[2];
        xsrc[2] = xsrc[3];
        xsrc[3] = xsrc[4];
        xsrc[4] = xsrc[5];
        xsrc[5] = xsum;
        
        xsum = _mm_set1_epi32( 1 << 5 );

        xsum = _mm_add_epi32( xsum, _mm_madd_epi16( xfilt12, _mm_unpacklo_epi16( xsrc[0], xsrc[1] ) ) );
        xsum = _mm_add_epi32( xsum, _mm_madd_epi16( xfilt34, _mm_unpacklo_epi16( xsrc[2], xsrc[3] ) ) );
        xsum = _mm_add_epi32( xsum, _mm_madd_epi16( xfilt56, _mm_unpacklo_epi16( xsrc[4], xsrc[5] ) ) );
        
        xsum = _mm_srai_epi32 ( xsum, 6 );
        xsum = _mm_min_epi32  ( xmax, _mm_max_epi32( xmin, xsum ) );
        xsum = _mm_packs_epi32( xsum, _mm_setzero_si128() );

        __m128i
        xorg = _mm_loadl_epi64( ( const __m128i * ) origRow );
        origRow += origStride;

        xsum = _mm_sub_epi16 ( xsum, xorg );
        xsum = _mm_madd_epi16( xsum, xsum );

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
#endif

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

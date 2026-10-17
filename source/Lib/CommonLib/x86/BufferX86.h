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
/** \file     YuvX86.cpp
    \brief    SIMD averaging.
*/

#pragma once

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP 1

#include "CommonDefX86.h"
#include "Unit.h"
#include "InterpolationFilter.h"

#ifdef TARGET_SIMD_X86
#if ENABLE_SIMD_OPT_BUFFER

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if USE_AVX2
template<bool isAligned> static inline __m256i load_aligned_avx2       ( const void* addr );
template<>                      inline __m256i load_aligned_avx2<true> ( const void* addr ) { return _mm256_load_si256 ( (const __m256i *) addr );}
template<>                      inline __m256i load_aligned_avx2<false>( const void* addr ) { return _mm256_loadu_si256( (const __m256i *) addr );}
#endif

template<bool isAligned> static inline __m128i load_aligned       ( const void* addr );
template<>                      inline __m128i load_aligned<true> ( const void* addr ) { return _mm_load_si128 ( (const __m128i *) addr );}
template<>                      inline __m128i load_aligned<false>( const void* addr ) { return _mm_loadu_si128( (const __m128i *) addr );}

template< X86_VEXT vext >
void weightCiip_SSE( Pel* res, const Pel* src, const int numSamples, int numIntra )
{
#if USE_AVX2
  int n = 16;
  if( numIntra == 1 )
  {
    __m256i vres;
    __m256i vpred = _mm256_load_si256((const __m256i*)&res[0]);
    __m256i vsrc  = _mm256_load_si256((const __m256i*)&src[0]);
    for( ; n < numSamples; n+=16)
    {
      vres = _mm256_avg_epu16( vpred, vsrc );
      vpred = _mm256_load_si256((const __m256i*)&res[n]);
      vsrc  = _mm256_load_si256((const __m256i*)&src[n]);
      _mm256_storeu_si256( ( __m256i * )&res[n-16], vres );
    }
    vres = _mm256_avg_epu16( vpred, vsrc );
    _mm256_storeu_si256( ( __m256i * )&res[n-16], vres );
  }
  else
  {
    const Pel* scale   = ( numIntra == 0 ) ? res : src;
    const Pel* unscale = ( numIntra == 0 ) ? src : res;

    __m256i vres;
    __m256i voffset = _mm256_set1_epi16(2);
    __m256i vscl = _mm256_load_si256((const __m256i*)&scale[0]);
    __m256i vuns = _mm256_load_si256((const __m256i*)&unscale[0]);
    for( ; n < numSamples; n+=16)
    {
      vres = _mm256_srai_epi16( _mm256_adds_epi16( _mm256_adds_epi16(_mm256_adds_epi16( vscl, vscl),_mm256_adds_epi16( vscl, vuns)), voffset), 2 );
      vscl = _mm256_load_si256((const __m256i*)&scale[n]);
      vuns = _mm256_load_si256((const __m256i*)&unscale[n]);
      _mm256_storeu_si256( ( __m256i * )&res[n-16], vres );
    }
    vres = _mm256_srai_epi16( _mm256_adds_epi16( _mm256_adds_epi16(_mm256_adds_epi16( vscl, vscl),_mm256_adds_epi16( vscl, vuns)), voffset), 2 );
    _mm256_storeu_si256( ( __m256i * )&res[n-16], vres );
  }
#else
  int n = 8;
  if( numIntra == 1 )
  {
    __m128i vres;
    __m128i vpred = _mm_load_si128((const __m128i*)&res[0]);
    __m128i vsrc  = _mm_load_si128((const __m128i*)&src[0]);
    for( ; n < numSamples; n+=8)
    {
      vres = _mm_avg_epu16( vpred, vsrc );
      vpred = _mm_load_si128((const __m128i*)&res[n]);
      vsrc  = _mm_load_si128((const __m128i*)&src[n]);
      _mm_storeu_si128( ( __m128i * )&res[n-8], vres );
    }
    vres = _mm_avg_epu16( vpred, vsrc );
    _mm_storeu_si128( ( __m128i * )&res[n-8], vres );
  }
  else
  {
    const Pel* scale   = ( numIntra == 0 ) ? res : src;
    const Pel* unscale = ( numIntra == 0 ) ? src : res;

    __m128i vres;
    __m128i voffset = _mm_set1_epi16(2);
    __m128i vscl = _mm_load_si128((const __m128i*)&scale[0]);
    __m128i vuns = _mm_load_si128((const __m128i*)&unscale[0]);
    for( ; n < numSamples; n+=8)
    {
      vres = _mm_srai_epi16( _mm_adds_epi16( _mm_adds_epi16(_mm_adds_epi16( vscl, vscl),_mm_adds_epi16( vscl, vuns)), voffset), 2 );
      vscl = _mm_load_si128((const __m128i*)&scale[n]);
      vuns = _mm_load_si128((const __m128i*)&unscale[n]);
      _mm_storeu_si128( ( __m128i * )&res[n-8], vres );
    }
    vres = _mm_srai_epi16( _mm_adds_epi16( _mm_adds_epi16(_mm_adds_epi16( vscl, vscl),_mm_adds_epi16( vscl, vuns)), voffset), 2 );
    _mm_storeu_si128( ( __m128i * )&res[n-8], vres );
  }
#endif
}

template< X86_VEXT vext, unsigned inputSize, unsigned outputSize >
void mipMatrixMul_SSE( Pel* res, const Pel* input, const uint8_t* weight, const int maxVal, const int inputOffset, bool transpose )
{
  int sum = 0;
  for( int i = 0; i < inputSize; i++ ) { sum += input[i]; }
  const int offset = (1 << (MIP_SHIFT_MATRIX - 1)) - MIP_OFFSET_MATRIX * sum + (inputOffset << MIP_SHIFT_MATRIX);
  CHECK( inputSize != 4 * (inputSize >> 2), "Error, input size not divisible by four" );

#if USE_AVX2
#if !ENABLE_VALGRIND_CODE
  static
#endif
  const __m256i perm = _mm256_setr_epi32(0,4,1,5,2,6,3,7);
  __m256i vibdimin  = _mm256_set1_epi16( 0 );
  __m256i vibdimax  = _mm256_set1_epi16( maxVal );
  if( inputSize == 4 && outputSize == 4)
  {
    __m256i voffset   = _mm256_set1_epi32( offset );
    __m256i vin = _mm256_set1_epi64x( *((const int64_t*)input) );
    __m256i vw = _mm256_load_si256((const __m256i*)(weight));

    __m256i w0 = _mm256_cvtepi8_epi16( _mm256_castsi256_si128( vw ) );   //w0 - w16
    __m256i w1 = _mm256_cvtepi8_epi16( _mm256_extracti128_si256( vw, 1 ) ); //w16 -w 32
     
    __m256i r0 = _mm256_madd_epi16( vin, w0 );
    __m256i r1 = _mm256_madd_epi16( vin, w1 );
    __m256i r2 = _mm256_hadd_epi32( r0 , r1);
           
            r2 = _mm256_add_epi32( r2, voffset );
    __m256i r3 = _mm256_srai_epi32( r2, MIP_SHIFT_MATRIX );

            vw = _mm256_load_si256((const __m256i*)(weight+32));
            w0 = _mm256_cvtepi8_epi16( _mm256_castsi256_si128( vw ) );   //w0 - w16
            w1 = _mm256_cvtepi8_epi16( _mm256_extracti128_si256( vw, 1 ) ); //w16 -w 32
     
            r0 = _mm256_madd_epi16( vin, w0 );
            r1 = _mm256_madd_epi16( vin, w1 );
            r2 = _mm256_hadd_epi32( r0 , r1);

            r2 = _mm256_add_epi32( r2, voffset );
            r2 = _mm256_srai_epi32( r2, MIP_SHIFT_MATRIX );
            r2 = _mm256_packs_epi32( r3, r2 );
            r2 = _mm256_permutevar8x32_epi32 ( r2, perm );

            r2 = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, r2 ) );

      if( transpose )
      {
        __m256i vshuf0 = _mm256_set_epi8( 0xf, 0xe, 0xb, 0xa, 0x7, 0x6, 0x3, 0x2, 0xd, 0xc, 0x9, 0x8, 0x5, 0x4, 0x1, 0x0,
                                      0xf, 0xe, 0xb, 0xa, 0x7, 0x6, 0x3, 0x2, 0xd, 0xc, 0x9, 0x8, 0x5, 0x4, 0x1, 0x0);
         r2 = _mm256_permutevar8x32_epi32( r2, _mm256_set_epi32(7,5,3,1,6,4,2,0) );
         r2 = _mm256_shuffle_epi8 ( r2, vshuf0);
      }

      _mm256_store_si256( ( __m256i * )&res[0], r2 );
  }
  else if( inputSize == 8 )
  {
    __m256i voffset   = _mm256_set1_epi32( offset );
    __m128i inv =_mm_load_si128( ( __m128i* )input );
    __m256i vin = _mm256_permute2f128_si256(_mm256_castsi128_si256(inv), _mm256_castsi128_si256(inv), 2); 
    __m256i r2;
    for( int i = 0; i < outputSize*outputSize; i+=16)
    {
      __m256i vw = _mm256_load_si256((const __m256i*)(weight));

      __m256i w0 = _mm256_cvtepi8_epi16( _mm256_castsi256_si128( vw ) );   //w0 - w16
      __m256i w1 = _mm256_cvtepi8_epi16( _mm256_extracti128_si256( vw, 1 ) ); //w16 -w 32
     
      __m256i r0 = _mm256_madd_epi16( vin, w0 );
      __m256i r1 = _mm256_madd_epi16( vin, w1 );
              r2 = _mm256_hadd_epi32( r0 , r1);
           
              vw = _mm256_load_si256((const __m256i*)(weight+32));
              w0 = _mm256_cvtepi8_epi16( _mm256_castsi256_si128( vw ) );   //w0 - w16
              w1 = _mm256_cvtepi8_epi16( _mm256_extracti128_si256( vw, 1 ) ); //w16 -w 32
     
              r0 = _mm256_madd_epi16( vin, w0 );
              r1 = _mm256_madd_epi16( vin, w1 );
      __m256i r4 = _mm256_hadd_epi32( r0 , r1);

              r2 = _mm256_hadd_epi32( r2 , r4);

              r2 = _mm256_add_epi32( r2, voffset );
      __m256i r3 = _mm256_srai_epi32( r2, MIP_SHIFT_MATRIX );

              vw = _mm256_load_si256((const __m256i*)(weight+64));

              w0 = _mm256_cvtepi8_epi16( _mm256_castsi256_si128( vw ) );   //w0 - w16
              w1 = _mm256_cvtepi8_epi16( _mm256_extracti128_si256( vw, 1 ) ); //w16 -w 32
     
              r0 = _mm256_madd_epi16( vin, w0 );
              r1 = _mm256_madd_epi16( vin, w1 );
              r2 = _mm256_hadd_epi32( r0 , r1);
           
              vw = _mm256_load_si256((const __m256i*)(weight+96));
              w0 = _mm256_cvtepi8_epi16( _mm256_castsi256_si128( vw ) );   //w0 - w16
              w1 = _mm256_cvtepi8_epi16( _mm256_extracti128_si256( vw, 1 ) ); //w16 -w 32
     
              r0 = _mm256_madd_epi16( vin, w0 );
              r1 = _mm256_madd_epi16( vin, w1 );

              r2 = _mm256_hadd_epi32( r2 , _mm256_hadd_epi32( r0 , r1));

              r2 = _mm256_add_epi32( r2, voffset );
              r2 = _mm256_srai_epi32( r2, MIP_SHIFT_MATRIX );

              r2 = _mm256_permutevar8x32_epi32 ( r2, perm );
              r3 = _mm256_permutevar8x32_epi32 ( r3, perm );

              r3 = _mm256_packs_epi32( r3, r2 );
              r2 = _mm256_permute4x64_epi64( r3, 0xd8 );

              r2 = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, r2 ) );

        _mm256_store_si256( ( __m256i * )&res[0], r2 );
        res+=16;
        weight+=128;
    }

    if( transpose )
    {
      if( outputSize == 4 )
      {
        __m256i vshuf0 = _mm256_set_epi8( 0xf, 0xe, 0xb, 0xa, 0x7, 0x6, 0x3, 0x2, 0xd, 0xc, 0x9, 0x8, 0x5, 0x4, 0x1, 0x0,
                                      0xf, 0xe, 0xb, 0xa, 0x7, 0x6, 0x3, 0x2, 0xd, 0xc, 0x9, 0x8, 0x5, 0x4, 0x1, 0x0);
         r2 = _mm256_permutevar8x32_epi32( r2, _mm256_set_epi32(7,5,3,1,6,4,2,0) );
         r2 = _mm256_shuffle_epi8 ( r2, vshuf0);
        _mm256_store_si256( ( __m256i * )(res-16), r2 );
      }
      else
      {
        res -= 64;

        __m256i va, vb, vc, vd, wa, wb, wc, wd;

        va = _mm256_load_si256( ( const __m256i* ) res ); 
        vb = _mm256_load_si256( ( const __m256i* ) (res+16) ); 
        vc = _mm256_load_si256( ( const __m256i* ) (res+32) ); 


        va =_mm256_permute4x64_epi64(va, 0xd8); 
        vb =_mm256_permute4x64_epi64(vb, 0xd8);
        vc =_mm256_permute4x64_epi64(vc, 0xd8);
        vd =_mm256_permute4x64_epi64(r2, 0xd8);

        wa = _mm256_unpacklo_epi16( va, vb );
        wb = _mm256_unpackhi_epi16( va, vb );
        wc = _mm256_unpacklo_epi16( vc, vd );
        wd = _mm256_unpackhi_epi16( vc, vd );

        va = _mm256_unpacklo_epi16( wa, wb );
        vb = _mm256_unpackhi_epi16( wa, wb );
        vc = _mm256_unpacklo_epi16( wc, wd );
        vd = _mm256_unpackhi_epi16( wc, wd );

        va =_mm256_permute4x64_epi64(va, 0xd8); 
        vb =_mm256_permute4x64_epi64(vb, 0xd8);
        vc =_mm256_permute4x64_epi64(vc, 0xd8);
        vd =_mm256_permute4x64_epi64(vd, 0xd8);

        wa = _mm256_unpacklo_epi64( va, vc );
        wb = _mm256_unpacklo_epi64( vb, vd );
        wc = _mm256_unpackhi_epi64( va, vc );
        wd = _mm256_unpackhi_epi64( vb, vd );

        _mm256_store_si256( ( __m256i* ) res, wa ); 
        _mm256_store_si256( ( __m256i* ) (res+16), wb );
        _mm256_store_si256( ( __m256i* ) (res+32), wc );
        _mm256_store_si256( ( __m256i* ) (res+48), wd );
      }
    }
  }
#else
  __m128i zero  = _mm_set1_epi16( 0 );
  __m128i vibdimax  = _mm_set1_epi16( maxVal );
  if( inputSize == 4 && outputSize == 4)
  {
    __m128i vin = _mm_set1_epi64x( *((const int64_t*)input) );
    __m128i voffset = _mm_set1_epi32( offset );
    __m128i r2 = vin;
    __m128i r;
    for( int i = 0; i < 2; i++)
    {
             r = r2; // save the result from the first interation
    __m128i vw = _mm_load_si128((const __m128i*)weight);

    __m128i w0 = _mm_unpacklo_epi8( vw, zero );
    __m128i w1 = _mm_unpackhi_epi8( vw, zero );
     
    __m128i r0 = _mm_madd_epi16( vin, w0 );
    __m128i r1 = _mm_madd_epi16( vin, w1 );
            r2 = _mm_hadd_epi32( r0 , r1);
           
            r2 = _mm_add_epi32( r2, voffset );
    __m128i r3 = _mm_srai_epi32( r2, MIP_SHIFT_MATRIX );

            vw = _mm_load_si128((const __m128i*)(weight+16));
            w0 = _mm_unpacklo_epi8( vw, zero );
            w1 = _mm_unpackhi_epi8( vw, zero );
     
            r0 = _mm_madd_epi16( vin, w0 );
            r1 = _mm_madd_epi16( vin, w1 );
            r2 = _mm_hadd_epi32( r0 , r1);

            r2 = _mm_add_epi32( r2, voffset );
            r2 = _mm_srai_epi32( r2, MIP_SHIFT_MATRIX );
            r2 = _mm_packs_epi32( r3, r2 );

            r2 = _mm_min_epi16( vibdimax, _mm_max_epi16( zero, r2 ) );

      _mm_store_si128( ( __m128i * )&res[0], r2 );
      res +=8;
      weight += 32;
    }

    if( transpose)
    {
      __m128i vc, vd, va, vb;
      vc = _mm_unpacklo_epi16( r, r2 );
      vd = _mm_unpackhi_epi16( r, r2 );
 
      va = _mm_unpacklo_epi16( vc, vd );
      vb = _mm_unpackhi_epi16( vc, vd );
 
      _mm_store_si128( ( __m128i* ) (res-16), va ); 
      _mm_store_si128( ( __m128i* ) (res-8), vb ); 
    }

  }
  else
  {
    __m128i vin = _mm_load_si128( (const __m128i*)input);
    __m128i voffset = _mm_set1_epi32( offset );

    for( int i = 0; i < outputSize*outputSize; i+=4)
    {
    __m128i vw = _mm_load_si128((const __m128i*)(weight));

    __m128i w0 = _mm_unpacklo_epi8( vw, zero );
    __m128i w1 = _mm_unpackhi_epi8( vw, zero );
     
    __m128i r0 = _mm_madd_epi16( vin, w0 );
    __m128i r1 = _mm_madd_epi16( vin, w1 );
    __m128i r2 = _mm_hadd_epi32( r0 , r1);
           
            vw = _mm_load_si128((const __m128i*)(weight+16));
            w0 = _mm_unpacklo_epi8( vw, zero );
            w1 = _mm_unpackhi_epi8( vw, zero );
     
            r0 = _mm_madd_epi16( vin, w0 );
            r1 = _mm_madd_epi16( vin, w1 );

            r2 = _mm_hadd_epi32( r2 , _mm_hadd_epi32( r0 , r1));

            r2 = _mm_add_epi32( r2, voffset );
            r2 = _mm_srai_epi32( r2, MIP_SHIFT_MATRIX );

            r2 = _mm_packs_epi32( r2, r2 );

            r2 = _mm_min_epi16( vibdimax, _mm_max_epi16( zero, r2 ) );

      _vv_storel_epi64( ( __m128i * )&res[0], r2 );
      res +=4;
      weight += 32;
    }

    if( transpose )
    {
      if( outputSize == 4)
      {
        res -= 16;
        __m128i vc, vd, va, vb;
        va = _mm_load_si128( ( const __m128i* ) (res) );
        vb = _mm_load_si128( ( const __m128i* ) (res+8) );

        vc = _mm_unpacklo_epi16( va, vb );
        vd = _mm_unpackhi_epi16( va, vb );
 
        va = _mm_unpacklo_epi16( vc, vd );
        vb = _mm_unpackhi_epi16( vc, vd );
 
        _mm_store_si128( ( __m128i* ) (res), va ); 
        _mm_store_si128( ( __m128i* ) (res+8), vb ); 
      }
      else
      {
        res -= 64;
        __m128i va, vb, vc, vd, ve, vf, vg, vh;

        va = _mm_load_si128( ( const __m128i* ) (res) );
        vb = _mm_load_si128( ( const __m128i* ) (res+8) );
        vc = _mm_load_si128( ( const __m128i* ) (res+16) );
        vd = _mm_load_si128( ( const __m128i* ) (res+24) );
        ve = _mm_load_si128( ( const __m128i* ) (res+32) );
        vf = _mm_load_si128( ( const __m128i* ) (res+40) );
        vg = _mm_load_si128( ( const __m128i* ) (res+48) );
        vh = _mm_load_si128( ( const __m128i* ) (res+56) );

        __m128i va01b01 = _mm_unpacklo_epi16( va, vb );
        __m128i va23b23 = _mm_unpackhi_epi16( va, vb );
        __m128i vc01d01 = _mm_unpacklo_epi16( vc, vd );
        __m128i vc23d23 = _mm_unpackhi_epi16( vc, vd );
        __m128i ve01f01 = _mm_unpacklo_epi16( ve, vf );
        __m128i ve23f23 = _mm_unpackhi_epi16( ve, vf );
        __m128i vg01h01 = _mm_unpacklo_epi16( vg, vh );
        __m128i vg23h23 = _mm_unpackhi_epi16( vg, vh );

        va = _mm_unpacklo_epi32( va01b01, vc01d01 );
        vb = _mm_unpackhi_epi32( va01b01, vc01d01 );
        vc = _mm_unpacklo_epi32( va23b23, vc23d23 );
        vd = _mm_unpackhi_epi32( va23b23, vc23d23 );
        ve = _mm_unpacklo_epi32( ve01f01, vg01h01 );
        vf = _mm_unpackhi_epi32( ve01f01, vg01h01 );
        vg = _mm_unpacklo_epi32( ve23f23, vg23h23 );
        vh = _mm_unpackhi_epi32( ve23f23, vg23h23 );

        va01b01 = _mm_unpacklo_epi64( va, ve );
        va23b23 = _mm_unpackhi_epi64( va, ve );
        vc01d01 = _mm_unpacklo_epi64( vb, vf );
        vc23d23 = _mm_unpackhi_epi64( vb, vf );
        ve01f01 = _mm_unpacklo_epi64( vc, vg );
        ve23f23 = _mm_unpackhi_epi64( vc, vg );
        vg01h01 = _mm_unpacklo_epi64( vd, vh );
        vg23h23 = _mm_unpackhi_epi64( vd, vh );

        _mm_store_si128( ( __m128i* ) (res),    va01b01 );
        _mm_store_si128( ( __m128i* ) (res+8) , va23b23 );
        _mm_store_si128( ( __m128i* ) (res+16), vc01d01 );
        _mm_store_si128( ( __m128i* ) (res+24), vc23d23 );
        _mm_store_si128( ( __m128i* ) (res+32), ve01f01 );
        _mm_store_si128( ( __m128i* ) (res+40), ve23f23 );
        _mm_store_si128( ( __m128i* ) (res+48), vg01h01 );
        _mm_store_si128( ( __m128i* ) (res+56), vg23h23 );
      }
    }
  }
#endif
}


template< X86_VEXT vext>
void addAvg_SSE( const Pel* src0, const Pel* src1, Pel* dst, int numSamples, unsigned shift, int offset, const ClpRng& clpRng )
{
#if USE_AVX2
  if( numSamples >= 16 )
  {
    const __m256i voffset   = _mm256_set1_epi32( offset );
    const __m256i vibdimin  = _mm256_set1_epi16( clpRng.min() );
    const __m256i vibdimax  = _mm256_set1_epi16( clpRng.max() );
    const __m256i vone      = _mm256_set1_epi16( 1 );

    for( int col = 0; col < numSamples; col += 16 )
    {
      __m256i vsrc0 = _mm256_load_si256( ( const __m256i* )&src0[col] );
      __m256i vsrc1 = _mm256_load_si256( ( const __m256i* )&src1[col] );

      __m256i vsum, vdst;
      vsum = _mm256_unpacklo_epi16    ( vsrc0, vsrc1 );
      vsum = _mm256_madd_epi16        ( vsum, vone );
      vsum = _mm256_add_epi32         ( vsum, voffset );
      vdst = _mm256_srai_epi32        ( vsum, shift );
      
      vsum = _mm256_unpackhi_epi16    ( vsrc0, vsrc1 );
      vsum = _mm256_madd_epi16        ( vsum, vone );
      vsum = _mm256_add_epi32         ( vsum, voffset );
      vsum = _mm256_srai_epi32        ( vsum, shift );

      vdst = _mm256_packs_epi32       ( vdst, vsum );

      vdst = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vdst ) );
      _mm256_store_si256( ( __m256i * )&dst[col], vdst );
    }
  }
  else
#endif
  if( numSamples >= 8 )
  {
    const __m128i vone     = _mm_set1_epi16( 1 );
    const __m128i voffset  = _mm_set1_epi32( offset );
    const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
    const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );

    for( int col = 0; col < numSamples; col += 8 )
    {
      __m128i vsrc0 = _mm_load_si128 ( (const __m128i *)&src0[col] );
      __m128i vsrc1 = _mm_load_si128 ( (const __m128i *)&src1[col] );

      __m128i vsum, vdst;
      vsum = _mm_unpacklo_epi16    ( vsrc0, vsrc1 );
      vsum = _mm_madd_epi16        ( vsum, vone );
      vsum = _mm_add_epi32         ( vsum, voffset );
      vdst = _mm_srai_epi32        ( vsum, shift );
      
      vsum = _mm_unpackhi_epi16    ( vsrc0, vsrc1 );
      vsum = _mm_madd_epi16        ( vsum, vone );
      vsum = _mm_add_epi32         ( vsum, voffset );
      vsum = _mm_srai_epi32        ( vsum, shift );

      vdst = _mm_packs_epi32       ( vdst, vsum );

      vdst = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vdst ) );
      _mm_store_si128( ( __m128i * )&dst[col], vdst );
    }
  }
  else if( numSamples == 4 )
  {
    const __m128i vone      = _mm_set1_epi16( 1 );
    const __m128i vzero     = _mm_setzero_si128();
    const __m128i voffset   = _mm_set1_epi32( offset );
    const __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
    const __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );

    __m128i vsum = _vv_loadl_epi64  ( ( const __m128i * )&src0[0] );
    __m128i vdst = _vv_loadl_epi64  ( ( const __m128i * )&src1[0] );
    vsum = _mm_unpacklo_epi16    ( vsum, vdst );
    vsum = _mm_madd_epi16        ( vsum, vone );
    vsum = _mm_add_epi32         ( vsum, voffset );
    vsum = _mm_srai_epi32        ( vsum, shift );
    vdst = _mm_packs_epi32       ( vsum, vzero );

    vdst = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vdst ) );
    _vv_storel_epi64( ( __m128i * )&dst[0], vdst );
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext>
void roundGeo_SSE( const Pel* src, Pel* dst, const int numSamples, unsigned shift, int offset, const ClpRng &clpRng)
{
#if USE_AVX2
  if( numSamples >= 16 )
  {
    __m256i voffset   = _mm256_set1_epi16( offset );
    __m256i vibdimin  = _mm256_set1_epi16( clpRng.min() );
    __m256i vibdimax  = _mm256_set1_epi16( clpRng.max() );

    for( int col = 0; col < numSamples; col += 16 )
    {
      __m256i val = _mm256_load_si256( ( const __m256i* )&src[col] );
      val = _mm256_adds_epi16        ( val, voffset );
      val = _mm256_srai_epi16        ( val, shift );
      val = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, val ) );
      _mm256_store_si256( ( __m256i * )&dst[col], val );
    }
  }
  else
#endif
  {
    __m128i voffset   = _mm_set1_epi16( offset );
    __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );

    if( numSamples >= 8 )
    {
      for( int col = 0; col < numSamples; col += 8 )
      {
        __m128i val = _mm_load_si128 ( (const __m128i *)&src[col] );
        val  = _mm_adds_epi16        ( val, voffset );
        val  = _mm_srai_epi16        ( val, shift );
        val  = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, val ) );
        _mm_store_si128( ( __m128i * )&dst[col], val );
      }
    }
    else //if( numSamples == 4 )
    {
      __m128i val = _vv_loadl_epi64  ( ( const __m128i * )&src[0] );
      val = _mm_adds_epi16           ( val, voffset );
      val = _mm_srai_epi16           ( val, shift );
      val = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, val ) );
      _vv_storel_epi64( ( __m128i * )&dst[0], val );
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext >
void recoCore_SSE( const Pel* src0, const Pel* src1, Pel* dst, int numSamples, const ClpRng& clpRng )
{
#if USE_AVX2
  if( vext >= AVX2 && numSamples >= 16 )
  {
    __m256i vbdmin = _mm256_set1_epi16( clpRng.min() );
    __m256i vbdmax = _mm256_set1_epi16( clpRng.max() );

    for( int n = 0; n < numSamples; n += 16 )
    {
      __m256i vdest = _mm256_load_si256 ( ( const __m256i * )&src0[n] );
      __m256i vsrc1 = _mm256_load_si256( ( const __m256i * )&src1[n] );

      vdest = _mm256_adds_epi16( vdest, vsrc1 );
      vdest = _mm256_min_epi16 ( vbdmax, _mm256_max_epi16( vbdmin, vdest ) );

      _mm256_store_si256( ( __m256i * )&dst[n], vdest );
    }
  }
  else
#endif
  if( numSamples >= 8 )
  {
    __m128i vbdmin = _mm_set1_epi16( clpRng.min() );
    __m128i vbdmax = _mm_set1_epi16( clpRng.max() );

    for( int n = 0; n < numSamples; n += 8 )
    {
      __m128i vdest = _mm_load_si128 ( ( const __m128i * )&src0[n] );
      __m128i vsrc1 = _mm_load_si128( ( const __m128i * )&src1[n] );

      vdest = _mm_adds_epi16( vdest, vsrc1 );
      vdest = _mm_min_epi16 ( vbdmax, _mm_max_epi16( vbdmin, vdest ) );

      _mm_store_si128( ( __m128i * )&dst[n], vdest );
    }
  }
  else
  {
    __m128i vbdmin = _mm_set1_epi16( clpRng.min() );
    __m128i vbdmax = _mm_set1_epi16( clpRng.max() );

    __m128i vsrc = _vv_loadl_epi64( ( const __m128i * )&src0[0] );
    __m128i vdst = _vv_loadl_epi64( ( const __m128i * )&src1[0] );

    vdst = _mm_adds_epi16( vdst, vsrc );
    vdst = _mm_min_epi16 ( vbdmax, _mm_max_epi16( vbdmin, vdst ) );

    _vv_storel_epi64( ( __m128i * )&dst[0], vdst );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void copyClip_SSE( const Pel* src, Pel* dst, int numSamples, const ClpRng& clpRng )
{
  if( vext >= AVX2 && numSamples >= 16 )
  {
#if USE_AVX2
    __m256i vbdmin   = _mm256_set1_epi16( clpRng.min() );
    __m256i vbdmax   = _mm256_set1_epi16( clpRng.max() );

    for( int col = 0; col < numSamples; col += 16 )
    {
      __m256i val = _mm256_loadu_si256  ( ( const __m256i * ) &src[col] );
      val = _mm256_min_epi16            ( vbdmax, _mm256_max_epi16( vbdmin, val ) );
      _mm256_storeu_si256               ( ( __m256i * )&dst[col], val );
    }
#endif
  }
  else if(numSamples >= 8 )
  {
    __m128i vbdmin = _mm_set1_epi16( clpRng.min() );
    __m128i vbdmax = _mm_set1_epi16( clpRng.max() );

    for( int col = 0; col < numSamples; col += 8 )
    {
      __m128i val = _mm_loadu_si128 ( ( const __m128i * ) &src[col] );
      val = _mm_min_epi16           ( vbdmax, _mm_max_epi16( vbdmin, val ) );
      _mm_storeu_si128              ( ( __m128i * )&dst[col], val );
    }
  }
  else
  {
    __m128i vbdmin  = _mm_set1_epi16( clpRng.min() );
    __m128i vbdmax  = _mm_set1_epi16( clpRng.max() );

    __m128i val;
    val = _vv_loadl_epi64   ( ( const __m128i * )&src[0] );
    val = _mm_min_epi16     ( vbdmax, _mm_max_epi16( vbdmin, val ) );
    _vv_storel_epi64        ( ( __m128i * )&dst[0], val );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}


template< X86_VEXT vext, int W, bool srcAligned >
void addAvg_SSE_algn( const int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int16_t *dst, ptrdiff_t dstStride, int width, int height, unsigned shift, int offset, const ClpRng& clpRng )
{
#if USE_AVX2
  if( W == 16 )
  {
    const __m256i voffset   = _mm256_set1_epi32( offset );
    const __m256i vibdimin  = _mm256_set1_epi16( clpRng.min() );
    const __m256i vibdimax  = _mm256_set1_epi16( clpRng.max() );
    const __m256i vone      = _mm256_set1_epi16( 1 );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 16 )
      {
        __m256i vsrc0 = load_aligned_avx2<srcAligned>( ( const void* )&src0[col] );
        __m256i vsrc1 = load_aligned_avx2<srcAligned>( ( const void* )&src1[col] );

        __m256i vsum, vdst;
        vsum = _mm256_unpacklo_epi16    ( vsrc0, vsrc1 );
        vsum = _mm256_madd_epi16        ( vsum, vone );
        vsum = _mm256_add_epi32         ( vsum, voffset );
        vdst = _mm256_srai_epi32        ( vsum, shift );
        
        vsum = _mm256_unpackhi_epi16    ( vsrc0, vsrc1 );
        vsum = _mm256_madd_epi16        ( vsum, vone );
        vsum = _mm256_add_epi32         ( vsum, voffset );
        vsum = _mm256_srai_epi32        ( vsum, shift );

        vdst = _mm256_packs_epi32       ( vdst, vsum );

        vdst = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vdst ) );
        _mm256_storeu_si256( ( __m256i * )&dst[col], vdst );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
#endif
#if USE_AVX2
  if( W >= 8 )
  {
    __m256i voffset  = _mm256_set1_epi32( offset );
    __m128i vibdimin = _mm_set1_epi16   ( clpRng.min() );
    __m128i vibdimax = _mm_set1_epi16   ( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i vsrc0 = _mm256_cvtepi16_epi32( load_aligned<srcAligned>( ( const void* )&src0[col] ) );
        __m256i vsrc1 = _mm256_cvtepi16_epi32( load_aligned<srcAligned>( ( const void* )&src1[col] ) );

        __m256i
        vsum = _mm256_add_epi32        ( vsrc0, vsrc1 );
        vsum = _mm256_add_epi32        ( vsum, voffset );
        vsum = _mm256_srai_epi32       ( vsum, shift );

        vsum = _mm256_packs_epi32      ( vsum, vsum );
        vsum = _mm256_permute4x64_epi64( vsum, 0 + ( 2 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

        __m128i
        xsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, _mm256_castsi256_si128( vsum ) ) );
        _mm_storeu_si128( ( __m128i * )&dst[col], xsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
#else
  if( W >= 8 )
  {
    const __m128i voffset  = _mm_set1_epi32( offset );
    const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
    const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
    const __m128i vone     = _mm_set1_epi16( 1 );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m128i vsrc0 = load_aligned<srcAligned>( ( const void* )&src0[col] );
        __m128i vsrc1 = load_aligned<srcAligned>( ( const void* )&src1[col] );

        __m128i vsum, vdst;
        vsum = _mm_unpacklo_epi16    ( vsrc0, vsrc1 );
        vsum = _mm_madd_epi16        ( vsum, vone );
        vsum = _mm_add_epi32         ( vsum, voffset );
        vdst = _mm_srai_epi32        ( vsum, shift );
        
        vsum = _mm_unpackhi_epi16    ( vsrc0, vsrc1 );
        vsum = _mm_madd_epi16        ( vsum, vone );
        vsum = _mm_add_epi32         ( vsum, voffset );
        vsum = _mm_srai_epi32        ( vsum, shift );

        vdst = _mm_packs_epi32       ( vdst, vsum );

        vdst = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vdst ) );
        _mm_storeu_si128( ( __m128i * )&dst[col], vdst );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
#endif
  else if( W == 4 )
  {
    __m128i vzero     = _mm_setzero_si128();
    __m128i voffset   = _mm_set1_epi32( offset );
    __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsum = _vv_loadl_epi64  ( ( const __m128i * )&src0[col] );
        __m128i vdst = _vv_loadl_epi64  ( ( const __m128i * )&src1[col] );
        vsum = _mm_cvtepi16_epi32       ( vsum );
        vdst = _mm_cvtepi16_epi32       ( vdst );
        vsum = _mm_add_epi32            ( vsum, vdst );
        vsum = _mm_add_epi32            ( vsum, voffset );
        vsum = _mm_srai_epi32           ( vsum, shift );
        vsum = _mm_packs_epi32          ( vsum, vzero );

        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
        _vv_storel_epi64( ( __m128i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext, int W >
void addAvg_SSE( const int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int16_t *dst, int dstStride, int width, int height, unsigned shift, int offset, const ClpRng& clpRng/*, bool srcAligned*/ )
{
/*
  if( srcAligned )
  {
    addAvg_SSE_algn<vext, W, true>( src0, src0Stride, src1, src1Stride, dst, dstStride, width, height, shift, offset, clpRng );
  }
  else
*/
  {
    addAvg_SSE_algn<vext, W, false>( src0, src0Stride, src1, src1Stride, dst, dstStride, width, height, shift, offset, clpRng );
  }
}


template< X86_VEXT vext, int W >
void addWghtAvg_SSE( const int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int16_t *dst, int dstStride, int width, int height, unsigned shift, int offset, int w0, int w1, const ClpRng& clpRng )
{
  if( W == 8 )
  {
#if USE_AVX2
    if( ( width & 15 ) == 0 && vext >= AVX2 )
    {
      __m256i voffset  = _mm256_set1_epi32( offset );
      __m256i vibdimin = _mm256_set1_epi16( clpRng.min() );
      __m256i vibdimax = _mm256_set1_epi16( clpRng.max() );
      __m256i vw       = _mm256_unpacklo_epi16( _mm256_set1_epi16( w0 ), _mm256_set1_epi16( w1 ) );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 16 )
        {
          __m256i vsrc0 = _mm256_loadu_si256( ( const __m256i * )&src0[col] );
          __m256i vsrc1 = _mm256_loadu_si256( ( const __m256i * )&src1[col] );

          __m256i vtmp, vsum;
          vsum = _mm256_madd_epi16       ( vw, _mm256_unpacklo_epi16( vsrc0, vsrc1 ) );
          vsum = _mm256_add_epi32        ( vsum, voffset );
          vtmp = _mm256_srai_epi32       ( vsum, shift );
        
          vsum = _mm256_madd_epi16       ( vw, _mm256_unpackhi_epi16( vsrc0, vsrc1 ) );
          vsum = _mm256_add_epi32        ( vsum, voffset );
          vsum = _mm256_srai_epi32       ( vsum, shift );
          vsum = _mm256_packs_epi32      ( vtmp, vsum );

          vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
          _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  +=  dstStride;
      }
    }
    else
#endif
    {
      __m128i voffset  = _mm_set1_epi32( offset );
      __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
      __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
      __m128i vw       = _mm_unpacklo_epi16( _mm_set1_epi16( w0 ), _mm_set1_epi16( w1 ) );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 8 )
        {
          __m128i vsrc0 = _mm_loadu_si128( ( const __m128i * )&src0[col] );
          __m128i vsrc1 = _mm_loadu_si128( ( const __m128i * )&src1[col] );

          __m128i vtmp, vsum;
          vsum = _mm_madd_epi16       ( vw, _mm_unpacklo_epi16( vsrc0, vsrc1 ) );
          vsum = _mm_add_epi32        ( vsum, voffset );
          vtmp = _mm_srai_epi32       ( vsum, shift );
        
          vsum = _mm_madd_epi16       ( vw, _mm_unpackhi_epi16( vsrc0, vsrc1 ) );
          vsum = _mm_add_epi32        ( vsum, voffset );
          vsum = _mm_srai_epi32       ( vsum, shift );
          vsum = _mm_packs_epi32      ( vtmp, vsum );

          vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
          _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  +=  dstStride;
      }
    }
  }
  else if( W == 4 )
  {
    __m128i vzero     = _mm_setzero_si128();
    __m128i voffset   = _mm_set1_epi32( offset );
    __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );
    __m128i vw        = _mm_unpacklo_epi16( _mm_set1_epi16( w0 ), _mm_set1_epi16( w1 ) );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsum = _vv_loadl_epi64  ( ( const __m128i * )&src0[col] );
        __m128i vdst = _vv_loadl_epi64  ( ( const __m128i * )&src1[col] );
        vsum = _mm_madd_epi16           ( vw, _mm_unpacklo_epi16( vsum, vdst ) );
        vsum = _mm_add_epi32            ( vsum, voffset );
        vsum = _mm_srai_epi32           ( vsum, shift );
        vsum = _mm_packs_epi32          ( vsum, vzero );

        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
        _vv_storel_epi64( ( __m128i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext >
void roundIntVector_SIMD(int* v, int size, unsigned int nShift, const int dmvLimit)
{
  CHECKD(size % 16 != 0, "Size must be multiple of 16!");
#ifdef USE_AVX512
  if (vext >= AVX512 && size >= 16)
  {
    __m512i dMvMin = _mm256_set1_epi32(-dmvLimit);
    __m512i dMvMax = _mm256_set1_epi32(dmvLimit);
    __m512i nOffset = _mm512_set1_epi32((1 << (nShift - 1)));
    __m512i vones = _mm512_set1_epi32(1);
    __m512i vzero = _mm512_setzero_si512();
    for (int i = 0; i < size; i += 16, v += 16)
    {
      __m512i src = _mm512_loadu_si512(v);
      __mmask16 mask = _mm512_cmpge_epi32_mask(src, vzero);
      src = __mm512_add_epi32(src, nOffset);
      __mm512i dst = _mm512_srai_epi32(_mm512_mask_sub_epi32(src, mask, src, vones), nShift);
      dst = _mm512_min_epi32(dMvMax, _mm512_max_epi32(dMvMin, dst));
      _mm512_storeu_si512(v, dst);
    }
  }
  else
#endif
#ifdef USE_AVX2
  if (vext >= AVX2 && size >= 8)
  {
    __m256i dMvMin = _mm256_set1_epi32(-dmvLimit);
    __m256i dMvMax = _mm256_set1_epi32(dmvLimit);
    __m256i nOffset = _mm256_set1_epi32(1 << (nShift - 1));
    __m256i vzero = _mm256_setzero_si256();
    for (int i = 0; i < size; i += 8, v += 8)
    {
      __m256i src = _mm256_lddqu_si256((__m256i*)v);
      __m256i of  = _mm256_cmpgt_epi32(src, vzero);
      __m256i dst = _mm256_srai_epi32(_mm256_add_epi32(_mm256_add_epi32(src, nOffset), of), nShift);
      dst = _mm256_min_epi32(dMvMax, _mm256_max_epi32(dMvMin, dst));
      _mm256_storeu_si256((__m256i*)v, dst);
    }
  }
  else
#endif
  {
    __m128i dMvMin = _mm_set1_epi32(-dmvLimit);
    __m128i dMvMax = _mm_set1_epi32(dmvLimit);
    __m128i nOffset = _mm_set1_epi32((1 << (nShift - 1)));
    __m128i vzero = _mm_setzero_si128();
    for (int i = 0; i < size; i += 4, v += 4)
    {
      __m128i src = _mm_loadu_si128((__m128i*)v);
      __m128i of  = _mm_cmpgt_epi32(src, vzero);
      __m128i dst = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(src, nOffset), of), nShift);
      dst = _mm_min_epi32(dMvMax, _mm_max_epi32(dMvMin, dst));
      _mm_storeu_si128((__m128i*)v, dst);
    }
  }
}


template< X86_VEXT vext, int W >
void reco_SSE( const int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int16_t *dst, int dstStride, int width, int height, const ClpRng& clpRng )
{
  if( W == 8 )
  {
#if USE_AVX2
    if( vext >= AVX2 && ( width & 15 ) == 0 )
    {
      __m256i vbdmin = _mm256_set1_epi16( clpRng.min() );
      __m256i vbdmax = _mm256_set1_epi16( clpRng.max() );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 16 )
        {
          __m256i vdest = _mm256_loadu_si256 ( ( const __m256i * )&src0[col] );
          __m256i vsrc1 = _mm256_loadu_si256( ( const __m256i * )&src1[col] );

          vdest = _mm256_adds_epi16( vdest, vsrc1 );
          vdest = _mm256_min_epi16 ( vbdmax, _mm256_max_epi16( vbdmin, vdest ) );

          _mm256_storeu_si256( ( __m256i * )&dst[col], vdest );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  += dstStride;
      }
    }
    else
#endif
    {
      __m128i vbdmin = _mm_set1_epi16( clpRng.min() );
      __m128i vbdmax = _mm_set1_epi16( clpRng.max() );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 8 )
        {
          __m128i vdest = _mm_loadu_si128 ( ( const __m128i * )&src0[col] );
          __m128i vsrc1 = _mm_loadu_si128( ( const __m128i * )&src1[col] );

          vdest = _mm_adds_epi16( vdest, vsrc1 );
          vdest = _mm_min_epi16 ( vbdmax, _mm_max_epi16( vbdmin, vdest ) );

          _mm_storeu_si128( ( __m128i * )&dst[col], vdest );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  += dstStride;
      }
    }
  }
  else if( W == 4 )
  {
    __m128i vbdmin = _mm_set1_epi16( clpRng.min() );
    __m128i vbdmax = _mm_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsrc = _vv_loadl_epi64( ( const __m128i * )&src0[col] );
        __m128i vdst = _vv_loadl_epi64( ( const __m128i * )&src1[col] );

        vdst = _mm_adds_epi16( vdst, vsrc );
        vdst = _mm_min_epi16 ( vbdmax, _mm_max_epi16( vbdmin, vdst ) );

        _vv_storel_epi64( ( __m128i * )&dst[col], vdst );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void copyBufferSimd( const char* src, int srcStride, char* dst, int dstStride, int width, int height)
{
  _mm_prefetch( src            , _MM_HINT_T0 );
  _mm_prefetch( src + srcStride, _MM_HINT_T0 );

  if( width == srcStride && width == dstStride )
  {
    memcpy( dst, src, width * height );
  }
  // Rows that are whole vectors are copied with vector loads and stores: for the small blocks the encoder copies
  // most, a memcpy call per row costs several times more than the copy itself.
#if USE_AVX2
  else if( vext >= AVX2 && ( width & 31 ) == 0 )
  {
    while( height-- )
    {
      for( int x = 0; x < width; x += 32 )
      {
        _mm256_storeu_si256( ( __m256i* ) &dst[x], _mm256_loadu_si256( ( const __m256i* ) &src[x] ) );
      }
      src += srcStride;
      dst += dstStride;
    }
    _mm256_zeroupper();
  }
#endif
  else if( ( width & 15 ) == 0 )
  {
    while( height-- )
    {
      for( int x = 0; x < width; x += 16 )
      {
        _mm_storeu_si128( ( __m128i* ) &dst[x], _mm_loadu_si128( ( const __m128i* ) &src[x] ) );
      }
      src += srcStride;
      dst += dstStride;
    }
  }
  else if( width == 8 )
  {
    while( height-- )
    {
      _vv_storel_epi64( ( __m128i* ) dst, _vv_loadl_epi64( ( const __m128i* ) src ) );
      src += srcStride;
      dst += dstStride;
    }
  }
  else
  {
    while( height-- )
    {
      const char* nextSrcLine = src + srcStride;
            char* nextDstLine = dst + dstStride;

      _mm_prefetch( nextSrcLine, _MM_HINT_T0 );

      memcpy( dst, src, width );

      src = nextSrcLine;
      dst = nextDstLine;
    }
  }
}

#if ENABLE_SIMD_OPT_BCW
template< X86_VEXT vext, int W >
void removeHighFreq_SSE(int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int width, int height)
{
 if (W == 8)
 {
   // TODO: AVX2 impl
   {
     for (int row = 0; row < height; row++)
     {
       for (int col = 0; col < width; col += 8)
       {
         __m128i vsrc0 = _mm_load_si128((const __m128i *)&src0[col]);
         __m128i vsrc1 = _mm_load_si128((const __m128i *)&src1[col]);

         vsrc0 = _mm_sub_epi16(_mm_slli_epi16(vsrc0, 1), vsrc1);
         _mm_store_si128((__m128i *)&src0[col], vsrc0);
       }

       src0 += src0Stride;
       src1 += src1Stride;
     }
   }
 }
 else if (W == 4)
 {
   for (int row = 0; row < height; row += 2)
   {
     __m128i vsrc0 = _vv_loadl_epi64((const __m128i *)src0);
     __m128i vsrc1 = _vv_loadl_epi64((const __m128i *)src1);
     __m128i vsrc0_2 = _vv_loadl_epi64((const __m128i *)(src0 + src0Stride));
     __m128i vsrc1_2 = _vv_loadl_epi64((const __m128i *)(src1 + src1Stride));

     vsrc0 = _mm_unpacklo_epi64(vsrc0, vsrc0_2);
     vsrc1 = _mm_unpacklo_epi64(vsrc1, vsrc1_2);

     vsrc0 = _mm_sub_epi16(_mm_slli_epi16(vsrc0, 1), vsrc1);
     _vv_storel_epi64((__m128i *)src0, vsrc0);
     _vv_storel_epi64((__m128i *)(src0 + src0Stride), _mm_unpackhi_epi64(vsrc0, vsrc0));

     src0 += (src0Stride << 1);
     src1 += (src1Stride << 1);
   }
 }
 else
 {
   THROW("Unsupported size");
 }
}
#endif

template<X86_VEXT vext, int W>
void sub_SSE( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride, int width, int height )
{
  if( W == 8 )
  {
    while( height-- )
    {
      for( int x = 0; x < width; x += 8 )
      {
        __m128i vsrc0 = _mm_load_si128( ( const __m128i* ) &src0[x] );
        __m128i vsrc1 = _mm_load_si128( ( const __m128i* ) &src1[x] );
        __m128i vdest = _mm_sub_epi16 ( vsrc0, vsrc1 );

        _mm_storeu_si128( ( __m128i* ) &dest[x], vdest );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dest += destStride;
    }
  }
  else if( W == 4 )
  {
    while( height-- )
    {
      for( int x = 0; x < width; x += 8 )
      {
        __m128i vsrc0 = _vv_loadl_epi64( ( const __m128i* ) &src0[x] );
        __m128i vsrc1 = _vv_loadl_epi64( ( const __m128i* ) &src1[x] );
        __m128i vdest = _mm_sub_epi16  ( vsrc0, vsrc1 );

        _vv_storel_epi64( ( __m128i* ) &dest[x], vdest );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dest += destStride;
    }
  }
  else
  {
    THROW("Unsupported size");
  }
}

template<bool doShift, bool shiftR, typename T> static inline void do_shift( T &vreg, int num );
#if USE_AVX2
template<> inline void do_shift<true,  true , __m256i>( __m256i &vreg, int num ) { vreg = _mm256_sra_epi32( vreg, _mm_cvtsi32_si128( num ) ); }
template<> inline void do_shift<true,  false, __m256i>( __m256i &vreg, int num ) { vreg = _mm256_sll_epi32( vreg, _mm_cvtsi32_si128( num ) ); }
template<> inline void do_shift<false, true , __m256i>( __m256i &vreg, int num ) { }
template<> inline void do_shift<false, false, __m256i>( __m256i &vreg, int num ) { }
#endif
template<> inline void do_shift<true,  true , __m128i>( __m128i &vreg, int num ) { vreg = _mm_sra_epi32( vreg, _mm_cvtsi32_si128( num ) ); }
template<> inline void do_shift<true,  false, __m128i>( __m128i &vreg, int num ) { vreg = _mm_sll_epi32( vreg, _mm_cvtsi32_si128( num ) ); }
template<> inline void do_shift<false, true , __m128i>( __m128i &vreg, int num ) { }
template<> inline void do_shift<false, false, __m128i>( __m128i &vreg, int num ) { }

template<bool mult, typename T> static inline void do_mult( T& vreg, T& vmult );
template<> inline void do_mult<false, __m128i>( __m128i&, __m128i& ) { }
#if USE_AVX2
template<> inline void do_mult<false, __m256i>( __m256i&, __m256i& ) { }
#endif
template<> inline void do_mult<true,   __m128i>( __m128i& vreg, __m128i& vmult ) { vreg = _mm_mullo_epi32   ( vreg, vmult ); }
#if USE_AVX2
template<> inline void do_mult<true,   __m256i>( __m256i& vreg, __m256i& vmult ) { vreg = _mm256_mullo_epi32( vreg, vmult ); }
#endif

template<bool add, typename T> static inline void do_add( T& vreg, T& vadd );
template<> inline void do_add<false, __m128i>( __m128i&, __m128i& ) { }
#if USE_AVX2
template<> inline void do_add<false, __m256i>( __m256i&, __m256i& ) { }
#endif
template<> inline void do_add<true,  __m128i>( __m128i& vreg, __m128i& vadd ) { vreg = _mm_add_epi32( vreg, vadd ); }
#if USE_AVX2
template<> inline void do_add<true,  __m256i>( __m256i& vreg, __m256i& vadd ) { vreg = _mm256_add_epi32( vreg, vadd ); }
#endif

template<bool clip, typename T> static inline void do_clip( T& vreg, T& vbdmin, T& vbdmax );
template<> inline void do_clip<false, __m128i>( __m128i&, __m128i&, __m128i& ) { }
#if USE_AVX2
template<> inline void do_clip<false, __m256i>( __m256i&, __m256i&, __m256i& ) { }
#endif
template<> inline void do_clip<true,  __m128i>( __m128i& vreg, __m128i& vbdmin, __m128i& vbdmax ) { vreg = _mm_min_epi16   ( vbdmax, _mm_max_epi16   ( vbdmin, vreg ) ); }
#if USE_AVX2
template<> inline void do_clip<true,  __m256i>( __m256i& vreg, __m256i& vbdmin, __m256i& vbdmax ) { vreg = _mm256_min_epi16( vbdmax, _mm256_max_epi16( vbdmin, vreg ) ); }
#endif


template<X86_VEXT vext, int W, bool doAdd, bool mult, bool doShift, bool shiftR, bool clip>
void linTf_SSE( const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng )
{
  if( vext >= AVX2 && ( width & 7 ) == 0 && W == 8 )
  {
#if USE_AVX2
    __m256i vzero    = _mm256_setzero_si256();
    __m256i vbdmin   = _mm256_set1_epi16( clpRng.min() );
    __m256i vbdmax   = _mm256_set1_epi16( clpRng.max() );
    __m256i voffset  = _mm256_set1_epi32( offset );
    __m256i vscale   = _mm256_set1_epi32( scale );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i val;
        val = _mm256_cvtepi16_epi32       (  _mm_loadu_si128( ( const __m128i * )&src[col] ) );
        do_mult<mult, __m256i>            ( val, vscale );
        do_shift<doShift, shiftR, __m256i>( val, shift );
        do_add<doAdd, __m256i>            ( val, voffset );
        val = _mm256_packs_epi32          ( val, vzero );
        do_clip<clip, __m256i>            ( val, vbdmin, vbdmax );
        val = _mm256_permute4x64_epi64    ( val, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 1 << 6 ) );

        _mm_storeu_si128                  ( ( __m128i * )&dst[col], _mm256_castsi256_si128( val ) );
      }

      src += srcStride;
      dst += dstStride;
    }
#endif
  }
  else
  {
    __m128i vzero   = _mm_setzero_si128();
    __m128i vbdmin  = _mm_set1_epi16   ( clpRng.min() );
    __m128i vbdmax  = _mm_set1_epi16   ( clpRng.max() );
    __m128i voffset = _mm_set1_epi32   ( offset );
    __m128i vscale  = _mm_set1_epi32   ( scale );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i val;
        val = _vv_loadl_epi64             ( ( const __m128i * )&src[col] );
        val = _mm_cvtepi16_epi32          ( val );
        do_mult<mult, __m128i>            ( val, vscale );
        do_shift<doShift, shiftR, __m128i>( val, shift );
        do_add<doAdd, __m128i>            ( val, voffset );
        val = _mm_packs_epi32             ( val, vzero );
        do_clip<clip, __m128i>            ( val, vbdmin, vbdmax );

        _vv_storel_epi64                  ( ( __m128i * )&dst[col], val );
      }

      src += srcStride;
      dst += dstStride;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext, int W>
void linTf_SSE_entry( const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height, int scale, unsigned shift, int offset, const ClpRng& clpRng, bool clip )
{
  int fn = ( offset == 0 ? 16 : 0 ) + ( scale == 1 ? 8 : 0 ) + ( shift == 0 ? 4 : 0 ) /*+ ( shift < 0 ? 2 : 0 )*/ + ( !clip ? 1 : 0 );

  switch( fn )
  {
  case  0: linTf_SSE<vext, W, true,  true,  true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  1: linTf_SSE<vext, W, true,  true,  true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
//  case  2: linTf_SSE<vext, W, true,  true,  true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
//  case  3: linTf_SSE<vext, W, true,  true,  true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  4: linTf_SSE<vext, W, true,  true,  false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  5: linTf_SSE<vext, W, true,  true,  false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
//  case  6: linTf_SSE<vext, W, true,  true,  false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
//  case  7: linTf_SSE<vext, W, true,  true,  false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  8: linTf_SSE<vext, W, true,  false, true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  9: linTf_SSE<vext, W, true,  false, true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
//  case 10: linTf_SSE<vext, W, true,  false, true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
//  case 11: linTf_SSE<vext, W, true,  false, true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 12: linTf_SSE<vext, W, true,  false, false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 13: linTf_SSE<vext, W, true,  false, false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
//  case 14: linTf_SSE<vext, W, true,  false, false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
//  case 15: linTf_SSE<vext, W, true,  false, false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 16: linTf_SSE<vext, W, false, true,  true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 17: linTf_SSE<vext, W, false, true,  true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
//  case 18: linTf_SSE<vext, W, false, true,  true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
//  case 19: linTf_SSE<vext, W, false, true,  true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 20: linTf_SSE<vext, W, false, true,  false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 21: linTf_SSE<vext, W, false, true,  false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
//  case 22: linTf_SSE<vext, W, false, true,  false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
//  case 23: linTf_SSE<vext, W, false, true,  false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 24: linTf_SSE<vext, W, false, false, true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 25: linTf_SSE<vext, W, false, false, true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
//  case 26: linTf_SSE<vext, W, false, false, true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
//  case 27: linTf_SSE<vext, W, false, false, true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 28: linTf_SSE<vext, W, false, false, false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 29: linTf_SSE<vext, W, false, false, false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
//  case 30: linTf_SSE<vext, W, false, false, false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
//  case 31: linTf_SSE<vext, W, false, false, false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  default:
    THROW( "Unknown parametrization of the linear transformation" );
    break;
  }
}

template<X86_VEXT vext, int W>
void copyClip_SSE( const int16_t* src, int srcStride, int16_t* dst, int dstStride, int width, int height, const ClpRng& clpRng )
{
  if( vext >= AVX2 && ( width & 15 ) == 0 && W == 8 )
  {
#if USE_AVX2
    __m256i vbdmin   = _mm256_set1_epi16( clpRng.min() );
    __m256i vbdmax   = _mm256_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 16 )
      {
        __m256i val = _mm256_loadu_si256  ( ( const __m256i * ) &src[col] );
        val = _mm256_min_epi16            ( vbdmax, _mm256_max_epi16( vbdmin, val ) );
        _mm256_storeu_si256               ( ( __m256i * )&dst[col], val );
      }

      src += srcStride;
      dst += dstStride;
    }
#endif
  }
  else if( W == 8 )
  {
    __m128i vbdmin = _mm_set1_epi16( clpRng.min() );
    __m128i vbdmax = _mm_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m128i val = _mm_loadu_si128 ( ( const __m128i * ) &src[col] );
        val = _mm_min_epi16           ( vbdmax, _mm_max_epi16( vbdmin, val ) );
        _mm_storeu_si128              ( ( __m128i * )&dst[col], val );
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else
  {
    __m128i vbdmin  = _mm_set1_epi16( clpRng.min() );
    __m128i vbdmax  = _mm_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i val;
        val = _vv_loadl_epi64   ( ( const __m128i * )&src[col] );
        val = _mm_min_epi16     ( vbdmax, _mm_max_epi16( vbdmin, val ) );
        _vv_storel_epi64        ( ( __m128i * )&dst[col], val );
      }

      src += srcStride;
      dst += dstStride;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext, int W>
void transposeNxN_SSE( const Pel* src, int srcStride, Pel* dst, int dstStride )
{
  if( W == 4 )
  {
    __m128i va, vb, vc, vd;

    va = _vv_loadl_epi64( ( const __m128i* ) src ); src += srcStride;
    vb = _vv_loadl_epi64( ( const __m128i* ) src ); src += srcStride;
    vc = _vv_loadl_epi64( ( const __m128i* ) src ); src += srcStride;
    vd = _vv_loadl_epi64( ( const __m128i* ) src );

    __m128i va01b01 = _mm_unpacklo_epi16( va,      vb );
    __m128i va23b23 = _mm_unpackhi_epi64( va01b01, vb );
    __m128i vc01d01 = _mm_unpacklo_epi16( vc,      vd );
    __m128i vc23d23 = _mm_unpackhi_epi64( vc01d01, vd );

    va = _mm_unpacklo_epi32( va01b01, vc01d01 );
    vb = _mm_unpackhi_epi64( va,      va );
    vc = _mm_unpacklo_epi32( va23b23, vc23d23 );
    vd = _mm_unpackhi_epi64( vc,      vc );

    _vv_storel_epi64( ( __m128i* ) dst, va ); dst += dstStride;
    _vv_storel_epi64( ( __m128i* ) dst, vb ); dst += dstStride;
    _vv_storel_epi64( ( __m128i* ) dst, vc ); dst += dstStride;
    _vv_storel_epi64( ( __m128i* ) dst, vd );
  }
  else if( W == 8 )
  {
  
  __m128i va, vb, vc, vd, ve, vf, vg, vh;

    va = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vb = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vc = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vd = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    ve = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vf = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vg = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vh = _mm_loadu_si128( ( const __m128i* ) src );

    __m128i va01b01 = _mm_unpacklo_epi16( va, vb );
    __m128i va23b23 = _mm_unpackhi_epi16( va, vb );
    __m128i vc01d01 = _mm_unpacklo_epi16( vc, vd );
    __m128i vc23d23 = _mm_unpackhi_epi16( vc, vd );
    __m128i ve01f01 = _mm_unpacklo_epi16( ve, vf );
    __m128i ve23f23 = _mm_unpackhi_epi16( ve, vf );
    __m128i vg01h01 = _mm_unpacklo_epi16( vg, vh );
    __m128i vg23h23 = _mm_unpackhi_epi16( vg, vh );

    va = _mm_unpacklo_epi32( va01b01, vc01d01 );
    vb = _mm_unpackhi_epi32( va01b01, vc01d01 );
    vc = _mm_unpacklo_epi32( va23b23, vc23d23 );
    vd = _mm_unpackhi_epi32( va23b23, vc23d23 );
    ve = _mm_unpacklo_epi32( ve01f01, vg01h01 );
    vf = _mm_unpackhi_epi32( ve01f01, vg01h01 );
    vg = _mm_unpacklo_epi32( ve23f23, vg23h23 );
    vh = _mm_unpackhi_epi32( ve23f23, vg23h23 );

    va01b01 = _mm_unpacklo_epi64( va, ve );
    va23b23 = _mm_unpackhi_epi64( va, ve );
    vc01d01 = _mm_unpacklo_epi64( vb, vf );
    vc23d23 = _mm_unpackhi_epi64( vb, vf );
    ve01f01 = _mm_unpacklo_epi64( vc, vg );
    ve23f23 = _mm_unpackhi_epi64( vc, vg );
    vg01h01 = _mm_unpacklo_epi64( vd, vh );
    vg23h23 = _mm_unpackhi_epi64( vd, vh );

    _mm_storeu_si128( ( __m128i* ) dst, va01b01 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, va23b23 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, vc01d01 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, vc23d23 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, ve01f01 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, ve23f23 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, vg01h01 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, vg23h23 );

  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

#if INTPTR_MAX == INT64_MAX
template<X86_VEXT vext>
void fillPtrMap_SIMD( void** ptr, ptrdiff_t ptrStride, int width, int height, void* val )
{
  static_assert( sizeof( ptr ) == 8, "Only supported for 64bit systems!" );
  if( ( width & 3 ) == 0 )
  {
#if USE_AVX2
    __m256i vval = _mm256_set1_epi64x( ( int64_t ) val );

    while( height-- )
    {
      for( int x = 0; x < width; x += 4 ) _mm256_storeu_si256( ( __m256i* ) &ptr[x], vval );

      ptr += ptrStride;
    }
#else
    __m128i vval = _mm_set1_epi64x( ( int64_t ) val );

    while( height-- )
    {
      for( int x = 0; x < width; x += 4 )
      {
        _mm_storeu_si128( ( __m128i* ) &ptr[x + 0], vval );
        _mm_storeu_si128( ( __m128i* ) &ptr[x + 2], vval );
      }

      ptr += ptrStride;
    }
#endif
  }
  else if( ( width & 1 ) == 0 )
  {
    __m128i vval = _mm_set1_epi64x( ( int64_t ) val );

    while( height-- )
    {
      for( int x = 0; x < width; x += 2 ) _mm_storeu_si128( ( __m128i* ) &ptr[x], vval );

      ptr += ptrStride;
    }
  }
  else
  {
    while( height-- )
    {
      *ptr = val; ptr += ptrStride;
    }
  }
}
#elif INTPTR_MAX == INT32_MAX
template<X86_VEXT vext>
void fillPtrMap_SIMD( void** ptr, ptrdiff_t ptrStride, int width, int height, void* val )
{
  static_assert( sizeof( ptr ) == 4, "Only supported for 32bit systems!" );
  if( ( width & 7 ) == 0 )
  {
#if USE_AVX2
    __m256i vval = _mm256_set1_epi32( ( int32_t ) val );

    while( height-- )
    {
      for( int x = 0; x < width; x += 8 )
      {
        _mm256_storeu_si256( ( __m256i* ) &ptr[x], vval );
      }

      ptr += ptrStride;
    }
#else
    __m128i vval = _mm_set1_epi32( ( int32_t ) val );

    while( height-- )
    {
      for( int x = 0; x < width; x += 8 )
      {
        _mm_storeu_si128( ( __m128i* ) &ptr[x + 0], vval );
        _mm_storeu_si128( ( __m128i* ) &ptr[x + 4], vval );
      }

      ptr += ptrStride;
    }
#endif
  }
  else if( ( width & 3 ) == 0 )
  {
    __m128i vval = _mm_set1_epi32( ( int32_t ) val );

    while( height-- )
    {
      for( int x = 0; x < width; x += 4 )
      {
        _mm_storeu_si128( ( __m128i* ) &ptr[x], vval );
      }

      ptr += ptrStride;
    }
  }
  else if( ( width & 1 ) == 0 )
  {
    while( height-- )
    {
      ptr[0] = val;
      ptr[1] = val;

      ptr += ptrStride;
    }
  }
  else
  {
    while( height-- )
    {
      for( int x = 0; x < width; ++x )
      {
        ptr[x] = val;
      }
      ptr += ptrStride;
    }
  }
}
#endif  // INTPTR_MAX == INT32_MAX

template<X86_VEXT vext>
uint64_t AvgHighPass_SIMD( const int width, const int height, const Pel* pSrc, const int iSrcStride)
{
  uint64_t saAct=0;
  pSrc -= iSrcStride;

#ifdef USE_AVX2
  int x;
  int sum;

  if (width > 16)
  {
    __m256i scale1 = _mm256_set_epi16 (0,-1,-2,-1,0,-1,-2,-1,0,-1,-2,-1,0,-1,-2,-1);
    __m256i scale0 = _mm256_set_epi16 (0,-2,12,-2,0,-2,12,-2,0,-2,12,-2,0,-2,12,-2);
    __m256i scale11 = _mm256_set_epi16(0,0,0,0,0,-1,-2,-1,0,-1,-2,-1,0,-1,-2,-1);
    __m256i scale00 = _mm256_set_epi16 (0,0,0,0,0,-2,12,-2,0,-2,12,-2,0,-2,12,-2);
    __m256i tmp1, tmp2, tmp3;
    __m256i line0, lineP1, lineM1;

    for (int y = 1; y < height-1; y += 1)
    {
      for (x = 1; x < width-1-14; x += 14)
      {
        sum=0;
        lineM1 = _mm256_lddqu_si256 ((__m256i*) &pSrc[ (y -1)  *iSrcStride + x-1]);
        line0  = _mm256_lddqu_si256 ((__m256i*) &pSrc [(y)*iSrcStride + x-1]);
        lineP1 = _mm256_lddqu_si256 ((__m256i*) &pSrc[(y+1)*iSrcStride + x-1]);

        tmp1 = _mm256_madd_epi16 (line0, scale0);
        tmp2 = _mm256_madd_epi16 (lineP1, scale1);
        tmp3 = _mm256_madd_epi16 (lineM1, scale1);
        tmp1 = _mm256_add_epi32(tmp1,tmp2);
        tmp1 = _mm256_add_epi32(tmp1,tmp3);
        tmp1 = _mm256_hadd_epi32(tmp1,tmp1);
        tmp1 = _mm256_abs_epi32(tmp1);
        tmp1 = _mm256_hadd_epi32(tmp1,tmp1);
        sum+=_mm256_extract_epi32 (tmp1, 0);
        sum+=_mm256_extract_epi32 (tmp1, 4);

        line0  = _mm256_bsrli_epi128 (line0 , 2);
        lineP1 = _mm256_bsrli_epi128 (lineP1, 2);
        lineM1 = _mm256_bsrli_epi128 (lineM1, 2);
        tmp1 = _mm256_madd_epi16 (line0, scale0);
        tmp2 = _mm256_madd_epi16 (lineP1, scale1);
        tmp3 = _mm256_madd_epi16 (lineM1, scale1);

        tmp1 = _mm256_add_epi32(tmp1,tmp2);
        tmp1 = _mm256_add_epi32(tmp1,tmp3);
        tmp1 = _mm256_hadd_epi32(tmp1,tmp1);
        tmp1 = _mm256_abs_epi32(tmp1);
        tmp1 = _mm256_hadd_epi32(tmp1,tmp1);

        sum+=_mm256_extract_epi32 (tmp1, 0);
        sum+=_mm256_extract_epi32 (tmp1, 4);

        lineM1 = _mm256_lddqu_si256 ((__m256i*) &pSrc[ (y -1)  *iSrcStride + x-1+2]);
        line0  = _mm256_lddqu_si256 ((__m256i*) &pSrc [(y)*iSrcStride + x-1+2]);
        lineP1 = _mm256_lddqu_si256 ((__m256i*) &pSrc[(y+1)*iSrcStride + x-1+2]);
        tmp1 = _mm256_madd_epi16 (line0, scale00);
        tmp2 = _mm256_madd_epi16 (lineP1, scale11);
        tmp3 = _mm256_madd_epi16 (lineM1, scale11);
        tmp1 = _mm256_add_epi32(tmp1,tmp2);
        tmp1 = _mm256_add_epi32(tmp1,tmp3);
        tmp1 = _mm256_hadd_epi32(tmp1,tmp1);
        tmp1 = _mm256_abs_epi32(tmp1);
        tmp1 = _mm256_hadd_epi32(tmp1,tmp1);
        sum+=_mm256_extract_epi32 (tmp1, 0);
        sum+=_mm256_extract_epi32 (tmp1, 4);

        line0  = _mm256_bsrli_epi128 (line0 , 2);
        lineP1 = _mm256_bsrli_epi128 (lineP1, 2);
        lineM1 = _mm256_bsrli_epi128 (lineM1, 2);

        tmp1 = _mm256_madd_epi16 (line0, scale00);
        tmp2 = _mm256_madd_epi16 (lineP1, scale11);
        tmp3 = _mm256_madd_epi16 (lineM1, scale11);

        tmp1 = _mm256_add_epi32(tmp1,tmp2);
        tmp1 = _mm256_add_epi32(tmp1,tmp3);
        tmp1 = _mm256_hadd_epi32(tmp1,tmp1);
        tmp1 = _mm256_abs_epi32(tmp1);
        tmp1 = _mm256_hadd_epi32(tmp1,tmp1);

        sum+=_mm256_extract_epi32 (tmp1, 0);
        sum+=_mm256_extract_epi32 (tmp1, 4);
        saAct += (uint64_t) sum;
      }
      // last collum
      for (; x < width - 1; x++) //
      {
        const int s = 12 * (int) pSrc[x  + y*iSrcStride ] - 2 * ((int) pSrc[x-1+y*iSrcStride] + (int) pSrc[x+1+y*iSrcStride] + (int) pSrc[x  -iSrcStride+y*iSrcStride] + (int) pSrc[x  +iSrcStride+y*iSrcStride])
                                                       - ((int) pSrc[x-1-iSrcStride+y*iSrcStride] + (int) pSrc[x+1-iSrcStride+y*iSrcStride] + (int) pSrc[x-1+iSrcStride+y*iSrcStride] + (int) pSrc[x+1+iSrcStride+y*iSrcStride]);
        saAct += abs (s);
      }
    }
  }
  else
#endif
  {
    int x;
    int sum;

    __m128i scale1 = _mm_set_epi16 (0,-1,-2,-1,0,-1,-2,-1);
    __m128i scale0 = _mm_set_epi16 (0,-2,12,-2,0,-2,12,-2);
    __m128i scale11 = _mm_set_epi16(0,0,0,0,0,-1,-2,-1);
    __m128i scale00 = _mm_set_epi16 (0,0,0,0,0,-2,12,-2);
    __m128i tmp1, tmp2, tmp3;
    __m128i line0, lineP1, lineM1;

    for (int y = 1; y < height-1; y += 1)
    {
      for (x = 1; x < width-1-6; x += 6)
      {
        sum=0;
        lineM1 = _mm_loadu_si128 ((__m128i*) &pSrc[ (y -1)  *iSrcStride + x-1]);
        line0  = _mm_loadu_si128 ((__m128i*) &pSrc [(y)*iSrcStride + x-1]);
        lineP1 = _mm_loadu_si128 ((__m128i*) &pSrc[(y+1)*iSrcStride + x-1]);

        tmp1 = _mm_madd_epi16 (line0, scale0);
        tmp2 = _mm_madd_epi16 (lineP1, scale1);
        tmp3 = _mm_madd_epi16 (lineM1, scale1);
        tmp1 = _mm_add_epi32(tmp1,tmp2);
        tmp1 = _mm_add_epi32(tmp1,tmp3);
        tmp1 = _mm_hadd_epi32(tmp1,tmp1);
        tmp1 = _mm_abs_epi32(tmp1);
        tmp1 = _mm_hadd_epi32(tmp1,tmp1);
        sum+=_mm_extract_epi32 (tmp1, 0);

        line0  = _mm_bsrli_si128 (line0 , 2);
        lineP1 = _mm_bsrli_si128 (lineP1, 2);
        lineM1 = _mm_bsrli_si128 (lineM1, 2);
        tmp1 = _mm_madd_epi16 (line0, scale0);
        tmp2 = _mm_madd_epi16 (lineP1, scale1);
        tmp3 = _mm_madd_epi16 (lineM1, scale1);

        tmp1 = _mm_add_epi32(tmp1,tmp2);
        tmp1 = _mm_add_epi32(tmp1,tmp3);
        tmp1 = _mm_hadd_epi32(tmp1,tmp1);
        tmp1 = _mm_abs_epi32(tmp1);
        tmp1 = _mm_hadd_epi32(tmp1,tmp1);

        sum+=_mm_extract_epi32 (tmp1, 0);

        lineM1 = _mm_loadu_si128 ((__m128i*) &pSrc[ (y -1)  *iSrcStride + x-1+2]);
        line0  = _mm_loadu_si128 ((__m128i*) &pSrc [(y)*iSrcStride + x-1+2]);
        lineP1 = _mm_loadu_si128 ((__m128i*) &pSrc[(y+1)*iSrcStride + x-1+2]);
        tmp1 = _mm_madd_epi16 (line0, scale00);
        tmp2 = _mm_madd_epi16 (lineP1, scale11);
        tmp3 = _mm_madd_epi16 (lineM1, scale11);
        tmp1 = _mm_add_epi32(tmp1,tmp2);
        tmp1 = _mm_add_epi32(tmp1,tmp3);
        tmp1 = _mm_hadd_epi32(tmp1,tmp1);
        tmp1 = _mm_abs_epi32(tmp1);
        tmp1 = _mm_hadd_epi32(tmp1,tmp1);
        sum+=_mm_extract_epi32 (tmp1, 0);

        line0  = _mm_bsrli_si128 (line0 , 2);
        lineP1 = _mm_bsrli_si128 (lineP1, 2);
        lineM1 = _mm_bsrli_si128 (lineM1, 2);

        tmp1 = _mm_madd_epi16 (line0, scale00);
        tmp2 = _mm_madd_epi16 (lineP1, scale11);
        tmp3 = _mm_madd_epi16 (lineM1, scale11);

        tmp1 = _mm_add_epi32(tmp1,tmp2);
        tmp1 = _mm_add_epi32(tmp1,tmp3);
        tmp1 = _mm_hadd_epi32(tmp1,tmp1);
        tmp1 = _mm_abs_epi32(tmp1);
        tmp1 = _mm_hadd_epi32(tmp1,tmp1);

        sum+=_mm_extract_epi32 (tmp1, 0);
        saAct += (uint64_t) sum;
      }

      // last collum
      for (; x < width - 1; x++) //
      {
        const int s = 12 * (int) pSrc[x  + y*iSrcStride ] - 2 * ((int) pSrc[x-1+y*iSrcStride] + (int) pSrc[x+1+y*iSrcStride] + (int) pSrc[x  -iSrcStride+y*iSrcStride] + (int) pSrc[x  +iSrcStride+y*iSrcStride])
                                                       - ((int) pSrc[x-1-iSrcStride+y*iSrcStride] + (int) pSrc[x+1-iSrcStride+y*iSrcStride] + (int) pSrc[x-1+iSrcStride+y*iSrcStride] + (int) pSrc[x+1+iSrcStride+y*iSrcStride]);
        saAct += abs (s);
      }
    }
  }
  return saAct;
}

template<X86_VEXT vext>
uint64_t HDHighPass_SIMD  (const int width, const int height,const Pel*  pSrc,const Pel* pSM1,const int iSrcStride,const int iSM1Stride)
{
  uint64_t taAct = 0;
  uint16_t act = 0;
  const __m128i scale1 = _mm_set_epi16 (1,1,1,1,1,1,1,1);
  pSrc -= iSrcStride;
  pSM1 -= iSM1Stride;
  int x;
  if (width>8)
  {
    for (int y = 1; y < height - 1; y++)
    {
      for (x = 1; x < width - 1-8 ; x+=8)  // cnt cols
      {
        __m128i M0 = _mm_loadu_si128 ((__m128i*) &pSrc  [ y   *iSrcStride + x]); /* load 8 16-bit values */
        __m128i M1 = _mm_loadu_si128 ((__m128i*) &pSM1  [y *iSM1Stride + x]);
        M1 = _mm_sub_epi16 (M0, M1);
        M1 = _mm_abs_epi16 (M1);
        M1 = _mm_hadd_epi16 (M1, M1);

        //  (1 + 3 * abs (t)) >> 1
        M0 = _mm_add_epi16(M1,M1);
        M1 = _mm_add_epi16(M0,M1);
        M1 = _mm_add_epi16(M1,scale1);
        M1 = _mm_srai_epi16 (M1,1);

        M1 = _mm_hadds_epi16 (M1, M1);
        M1 = _mm_hadds_epi16 (M1, M1);
        _mm_storeu_si16 (&act, M1);
        taAct += (uint64_t)act;
      }
      // last collum
      __m128i M0 = _mm_loadu_si128 ((__m128i*) &pSrc  [ y   *iSrcStride + x]); /* load 8 16-bit values */
      __m128i M1 = _mm_loadu_si128 ((__m128i*) &pSM1  [y *iSM1Stride + x]);

      M1 = _mm_sub_epi16 (M0, M1);
      M1 = _mm_abs_epi16 (M1);
      int n=8-width+1+x;
      if (n > 0)
      {
        //remove n Pixel
        switch (n)
        {
        case 1:
        {
          M1 = _mm_slli_si128 (M1,2);
          M1 = _mm_srli_si128 (M1,2);
          break;
        }
        case 2:
        {
          M1 = _mm_slli_si128 (M1,4);
          M1 = _mm_srli_si128 (M1,4);
          break;
        }
        case 3:
        {
          M1 = _mm_slli_si128 (M1,6);
          M1 = _mm_srli_si128 (M1,6);
          break;
        }
        case 4:
        {
          M1 = _mm_slli_si128 (M1,8);
          M1 = _mm_srli_si128 (M1,8);
          break;
        }
        case 5:
        {
          M1 = _mm_slli_si128 (M1,10);
          M1 = _mm_srli_si128 (M1,10);
          break;
        }
        case 6:
        {
          M1 = _mm_slli_si128 (M1,12);
          M1 = _mm_srli_si128 (M1,12);
          break;
        }
        case 7:
        {
          M1 = _mm_slli_si128 (M1,14);
          M1 = _mm_srli_si128 (M1,14);
          break;
        }
        }
      }
      M1 = _mm_hadd_epi16 (M1, M1);
      //  (1 + 3 * abs (t)) >> 1
      M0 = _mm_add_epi16(M1,M1);
      M1 = _mm_add_epi16(M0,M1);
      M1 = _mm_add_epi16(M1,scale1);
      M1 = _mm_srai_epi16 (M1,1);

      M1 = _mm_hadds_epi16 (M1, M1);
      M1 = _mm_hadds_epi16 (M1, M1);
      _mm_storeu_si16 (&act, M1);
      taAct += (uint64_t)act;
    }
  }
  else
  {
    for (int y = 1; y < height - 1; y++)
    {
      for (int x = 1; x < width - 1; x++)  // cnt cols
      {
        const int t = (int) pSrc[x] - (int) pSM1[x];
        taAct += (1 + 3 * abs (t)) >> 1;
      }
      pSrc += iSrcStride;
      pSM1 += iSM1Stride;
    }
  }
  return taAct;
}

template<X86_VEXT vext>
uint64_t  HDHighPass2_SIMD  (const int width, const int height,const Pel*  pSrc,const Pel* pSM1,const Pel* pSM2,const int iSrcStride,const int iSM1Stride,const int iSM2Stride)
{
  uint64_t taAct = 0;
  uint16_t act = 0;
  pSrc -= iSrcStride;
  pSM1 -= iSM1Stride;
  pSM2 -= iSM2Stride;
  int x;
  if (width>8)
  {
    for (int y = 1; y < height - 1; y++)
    {
      for (x = 1; x < width - 1-8 ; x+=8)  // cnt cols
      {
        __m128i M0 = _mm_loadu_si128 ((__m128i*) &pSrc  [ y   *iSrcStride + x]); /* load 8 16-bit values */
        __m128i M1 = _mm_loadu_si128 ((__m128i*) &pSM1  [y *iSM1Stride + x]);
        __m128i M2 = _mm_loadu_si128 ((__m128i*) &pSM2  [y *iSM2Stride + x]);
        M1 = _mm_slli_epi16 (M1, 1);
        M1 = _mm_sub_epi16 (M0, M1);
        M1 = _mm_add_epi16 (M1,M2);
        M1 = _mm_abs_epi16 (M1);
        M1 = _mm_hadd_epi16 (M1, M1);

        M1 = _mm_hadds_epi16 (M1, M1);
        M1 = _mm_hadds_epi16 (M1, M1);
        _mm_storeu_si16 (&act, M1);
        taAct += (uint64_t)act;
      }
      // last collum
      __m128i M0 = _mm_loadu_si128 ((__m128i*) &pSrc  [ y   *iSrcStride + x]); /* load 8 16-bit values */
      __m128i M1 = _mm_loadu_si128 ((__m128i*) &pSM1  [y *iSM1Stride + x]);
      __m128i M2 = _mm_loadu_si128 ((__m128i*) &pSM2  [y *iSM2Stride + x]);
      M1 = _mm_slli_epi16 (M1, 1);
      M1 = _mm_sub_epi16 (M0, M1);
      M1 = _mm_add_epi16 (M1,M2);
      M1 = _mm_abs_epi16 (M1);
      int n=8-width+1+x;
      if (n > 0)
      {
        switch (n)
        {
        case 1:
        {
          M1 = _mm_slli_si128 (M1,2);
          M1 = _mm_srli_si128 (M1,2);
          break;
        }
        case 2:
        {
          M1 = _mm_slli_si128 (M1,4);
          M1 = _mm_srli_si128 (M1,4);
          break;
        }
        case 3:
        {
          M1 = _mm_slli_si128 (M1,6);
          M1 = _mm_srli_si128 (M1,6);
          break;
        }
        case 4:
        {
          M1 = _mm_slli_si128 (M1,8);
          M1 = _mm_srli_si128 (M1,8);
          break;
        }
        case 5:
        {
          M1 = _mm_slli_si128 (M1,10);
          M1 = _mm_srli_si128 (M1,10);
          break;
        }
        case 6:
        {
          M1 = _mm_slli_si128 (M1,12);
          M1 = _mm_srli_si128 (M1,12);
          break;
        }
        case 7:
        {
          M1 = _mm_slli_si128 (M1,14);
          M1 = _mm_srli_si128 (M1,14);
          break;
        }
        }
      }
      M1 = _mm_hadd_epi16 (M1, M1);
      M1 = _mm_hadds_epi16 (M1, M1);
      M1 = _mm_hadds_epi16 (M1, M1);
      _mm_storeu_si16 (&act, M1);
      taAct += (uint64_t)act;
    }
  }
  else
  {
    for (int y = 1; y < height - 1; y++)
    {
      for (int x = 1; x < width - 1; x++)  // cnt cols
      {
        const int t = (int) pSrc[x] - 2 * (int) pSM1[x] + (int) pSM2[x];
        taAct += abs (t);
      }
      pSrc += iSrcStride;
      pSM1 += iSM1Stride;
      pSM2 += iSM2Stride;
    }
  }
  return taAct;
}

template<X86_VEXT vext>
uint64_t AvgHighPassWithDownsampling_SIMD ( const int width, const int height, const Pel* pSrc, const int iSrcStride)
{
  uint64_t saAct = 0;
  pSrc -= iSrcStride;
  pSrc -= iSrcStride;

#ifdef USE_AVX2
  if (width > 12)
  {
    const __m128i scale1 = _mm_set_epi16 (0, 0,-1,-2,-3,-3,-2,-1);
    const __m128i scale2 = _mm_set_epi16 (0, 0,-1,-3,12,12,-3,-1);
    const __m128i scale3 = _mm_set_epi16 (0, 0, 0,-1,-1,-1,-1, 0);
    __m128i tmp1, tmp2,tmp3,tmp4,tmp5;
    __m128i l0, lP1, lM1, lP2, lM2, lP3;

    int sum;

    for (int y = 2; y < height-2; y += 2)
    {
      for (int x = 2; x < width-2; x += 12)
      {
        __m256i lineM2 = _mm256_lddqu_si256 ((__m256i*) &pSrc[(y-2)*iSrcStride + x-2]);
        __m256i lineM1 = _mm256_lddqu_si256 ((__m256i*) &pSrc[(y-1)*iSrcStride + x-2]);
        __m256i line0  = _mm256_lddqu_si256 ((__m256i*) &pSrc[ y   *iSrcStride + x-2]);
        __m256i lineP1 = _mm256_lddqu_si256 ((__m256i*) &pSrc[(y+1)*iSrcStride + x-2]);
        __m256i lineP2 = _mm256_lddqu_si256 ((__m256i*) &pSrc[(y+2)*iSrcStride + x-2]);
        __m256i lineP3 = _mm256_lddqu_si256 ((__m256i*) &pSrc[(y+3)*iSrcStride + x-2]);

        for (int xx = 0; xx < 3; xx++)
        {
          l0  = _mm256_castsi256_si128 (line0 );
          lP1 = _mm256_castsi256_si128 (lineP1);
          lM1 = _mm256_castsi256_si128 (lineM1);
          lP2 = _mm256_castsi256_si128 (lineP2);
          lM2 = _mm256_castsi256_si128 (lineM2);
          lP3 = _mm256_castsi256_si128 (lineP3);

          if ((xx << 2) + x < width-2)
          {
            sum = 0;
            tmp1 = _mm_madd_epi16 (l0, scale2);
            tmp2 = _mm_madd_epi16 (lP1, scale2);
            tmp3 = _mm_add_epi32 (tmp1, tmp2);
            tmp1 = _mm_madd_epi16 (lM1, scale1);
            tmp2 = _mm_madd_epi16 (lP2, scale1);
            tmp4 = _mm_add_epi32(tmp1,tmp2);
            tmp4 = _mm_add_epi32(tmp4,tmp3);
            tmp1 = _mm_madd_epi16 (lM2, scale3);
            tmp2 = _mm_madd_epi16 (lP3, scale3);
            tmp5 = _mm_add_epi32(tmp1,tmp2);
            tmp4 = _mm_add_epi32(tmp4,tmp5);
            tmp1 = _mm_hadd_epi32 (tmp4, tmp4);
            tmp1 = _mm_hadd_epi32 (tmp1, tmp1);
            tmp1 = _mm_abs_epi32(tmp1);
            sum += _mm_extract_epi32 (tmp1, 0);
            saAct += (uint64_t) sum;
           }
          if ((xx << 2) + x + 2 < width-2)
          {
            sum = 0;
            l0  = _mm_bsrli_si128 (l0 , 4);
            lP1 = _mm_bsrli_si128 (lP1, 4);
            tmp1 = _mm_madd_epi16 (l0, scale2);
            tmp2 = _mm_madd_epi16 (lP1, scale2);
            tmp3 = _mm_add_epi32 (tmp1, tmp2);

            lM1 = _mm_bsrli_si128 (lM1, 4);
            lP2 = _mm_bsrli_si128 (lP2, 4);
            tmp1 = _mm_madd_epi16 (lM1, scale1);
            tmp2 = _mm_madd_epi16 (lP2, scale1);
            tmp4 = _mm_add_epi32(tmp1,tmp2);
            tmp4 = _mm_add_epi32(tmp4,tmp3);

            lM2 = _mm_bsrli_si128 (lM2, 4);
            lP3 = _mm_bsrli_si128 (lP3, 4);
            tmp1 = _mm_madd_epi16 (lM2, scale3);
            tmp2 = _mm_madd_epi16 (lP3, scale3);
            tmp5 = _mm_add_epi32(tmp1,tmp2);
            tmp4 = _mm_add_epi32(tmp4,tmp5);
            tmp1 = _mm_hadd_epi32 (tmp4, tmp4);
            tmp1 = _mm_hadd_epi32 (tmp1, tmp1);
            tmp1 = _mm_abs_epi32(tmp1);
            sum += _mm_extract_epi32 (tmp1, 0);

             saAct += (uint64_t) sum;
             /* 4 byte to the right */
            lineM2 = _mm256_permute4x64_epi64 (lineM2, 0x39);
            lineM1 = _mm256_permute4x64_epi64 (lineM1, 0x39);
            line0  = _mm256_permute4x64_epi64 (line0 , 0x39);
            lineP1 = _mm256_permute4x64_epi64 (lineP1, 0x39);
            lineP2 = _mm256_permute4x64_epi64 (lineP2, 0x39);
            lineP3 = _mm256_permute4x64_epi64 (lineP3, 0x39);
            }
        }
      }
    }
  }
  else
#endif
  {
    if (width > 6)
    {
      const __m128i scale1 = _mm_set_epi16 (0, 0,-1,-2,-3,-3,-2,-1);
      const __m128i scale2 = _mm_set_epi16 (0, 0,-1,-3,12,12,-3,-1);
      const __m128i scale3 = _mm_set_epi16 (0, 0, 0,-1,-1,-1,-1, 0);
      __m128i tmp1, tmp2,tmp3,tmp4,tmp5;
      __m128i l0, lP1, lM1, lP2, lM2, lP3;

      int sum;

      for (int y = 2; y < height-2; y += 2)
      {
        for (int x = 2; x < width-2; x += 4)
        {
          {
            lM2 = _mm_loadu_si128 ((__m128i*) &pSrc[(y-2)*iSrcStride + x-2]);
            lM1 = _mm_loadu_si128 ((__m128i*) &pSrc[(y-1)*iSrcStride + x-2]);
            l0  = _mm_loadu_si128 ((__m128i*) &pSrc[ y   *iSrcStride + x-2]);
            lP1 = _mm_loadu_si128 ((__m128i*) &pSrc[(y+1)*iSrcStride + x-2]);
            lP2 = _mm_loadu_si128 ((__m128i*) &pSrc[(y+2)*iSrcStride + x-2]);
            lP3 = _mm_loadu_si128 ((__m128i*) &pSrc[(y+3)*iSrcStride + x-2]);

            if ( x < width-2)
            {
              sum = 0;
              tmp1 = _mm_madd_epi16 (l0, scale2);
              tmp2 = _mm_madd_epi16 (lP1, scale2);
              tmp3 = _mm_add_epi32 (tmp1, tmp2);
              tmp1 = _mm_madd_epi16 (lM1, scale1);
              tmp2 = _mm_madd_epi16 (lP2, scale1);
              tmp4 = _mm_add_epi32(tmp1,tmp2);
              tmp4 = _mm_add_epi32(tmp4,tmp3);
              tmp1 = _mm_madd_epi16 (lM2, scale3);
              tmp2 = _mm_madd_epi16 (lP3, scale3);
              tmp5 = _mm_add_epi32(tmp1,tmp2);
              tmp4 = _mm_add_epi32(tmp4,tmp5);
              tmp1 = _mm_hadd_epi32 (tmp4, tmp4);
              tmp1 = _mm_hadd_epi32 (tmp1, tmp1);
              tmp1 = _mm_abs_epi32(tmp1);
              sum += _mm_extract_epi32 (tmp1, 0);

              saAct += (uint64_t) sum;
             }
            if (x + 2 < width-2)
            {
              sum = 0;
              l0  = _mm_bsrli_si128 (l0 , 4);
              lP1 = _mm_bsrli_si128 (lP1, 4);
              tmp1 = _mm_madd_epi16 (l0, scale2);
              tmp2 = _mm_madd_epi16 (lP1, scale2);
              tmp3 = _mm_add_epi32 (tmp1, tmp2);

              lM1 = _mm_bsrli_si128 (lM1, 4);
              lP2 = _mm_bsrli_si128 (lP2, 4);
              tmp1 = _mm_madd_epi16 (lM1, scale1);
              tmp2 = _mm_madd_epi16 (lP2, scale1);
              tmp4 = _mm_add_epi32(tmp1,tmp2);
              tmp4 = _mm_add_epi32(tmp4,tmp3);

              lM2 = _mm_bsrli_si128 (lM2, 4);
              lP3 = _mm_bsrli_si128 (lP3, 4);
              tmp1 = _mm_madd_epi16 (lM2, scale3);
              tmp2 = _mm_madd_epi16 (lP3, scale3);
              tmp5 = _mm_add_epi32(tmp1,tmp2);
              tmp4 = _mm_add_epi32(tmp4,tmp5);
              tmp1 = _mm_hadd_epi32 (tmp4, tmp4);
              tmp1 = _mm_hadd_epi32 (tmp1, tmp1);
              tmp1 = _mm_abs_epi32(tmp1);
              sum += _mm_extract_epi32 (tmp1, 0);
              saAct += (uint64_t) sum;
              }
          }
        }
      }
    }
 }
  return saAct;
}
template<X86_VEXT vext>
uint64_t AvgHighPassWithDownsamplingDiff1st_SIMD (const int width, const int height, const Pel *pSrc,const Pel *pSrcM1, const int iSrcStride, const int iSrcM1Stride)
{
  uint64_t taAct = 0;
  uint16_t act = 0;
  pSrc -= iSrcStride;
  pSrc -= iSrcStride;
  pSrcM1-=iSrcM1Stride;
  pSrcM1-=iSrcM1Stride;
  int32_t x;
  int32_t y;
  const __m128i scale1 = _mm_set_epi16 (1,1,1,1,1,1,1,1);
  for (y = 2; y < height-2; y += 2)
  {
    for (x = 2; x < width-2-10; x += 8)
    {
      __m128i lineM0u = _mm_loadu_si128 ((__m128i*) &pSrc  [ y   *iSrcStride + x]); /* load 8 16-bit values */
      __m128i lineM0d = _mm_loadu_si128 ((__m128i*) &pSrc  [(y+1)*iSrcStride + x]);
      __m128i lineM1u = _mm_loadu_si128 ((__m128i*) &pSrcM1[ y   *iSrcM1Stride + x]);
      __m128i lineM1d = _mm_loadu_si128 ((__m128i*) &pSrcM1[(y+1)*iSrcM1Stride + x]);
      __m128i M0 = _mm_add_epi16 (lineM0u, lineM0d);
      __m128i M1 = _mm_add_epi16 (lineM1u, lineM1d);

      M1 = _mm_sub_epi16 (M0, M1); /* abs (sum (o[u0, u1, d0, d1]) - sum (oM1[u0, u1, d0, d1])) */
      M1 = _mm_hadd_epi16 (M1, M1);
      M1 = _mm_abs_epi16 (M1);

      //  (1 + 3 * abs (t)) >> 1
      M0 = _mm_add_epi16(M1,M1);
      M1 = _mm_add_epi16(M0,M1);
      M1 = _mm_add_epi16(M1,scale1);
      M1 = _mm_srai_epi16 (M1,1);

      M1 = _mm_hadds_epi16 (M1, M1);
      M1 = _mm_hadds_epi16 (M1, M1);
      _mm_storeu_si16 (&act, M1);
      taAct += (uint64_t)act;
    }
    // last collum
    {
      __m128i lineM0u = _mm_loadu_si128 ((__m128i*) &pSrc  [ y   *iSrcStride + x]); /* load 8 16-bit values */
      __m128i lineM0d = _mm_loadu_si128 ((__m128i*) &pSrc  [(y+1)*iSrcStride + x]);
      __m128i lineM1u = _mm_loadu_si128 ((__m128i*) &pSrcM1[ y   *iSrcM1Stride + x]);
      __m128i lineM1d = _mm_loadu_si128 ((__m128i*) &pSrcM1[(y+1)*iSrcM1Stride + x]);
      __m128i M0 = _mm_add_epi16 (lineM0u, lineM0d);
      __m128i M1 = _mm_add_epi16 (lineM1u, lineM1d);
      M1 = _mm_sub_epi16 (M0, M1); /* abs (sum (o[u0, u1, d0, d1]) - sum (oM1[u0, u1, d0, d1])) */

      int n=8-width+2+x;
      if (n > 0)
      {
        //remove n Pixel
        if (n==2)
        {
          M1 = _mm_slli_si128 (M1, 4);
          M1 = _mm_srli_si128 (M1,4);
        }
        else if  (n==4)
        {
          M1 = _mm_slli_si128 (M1, 8);
          M1 = _mm_srli_si128 (M1,8);
        }
        else if  (n==6)
        {
          M1 = _mm_slli_si128 (M1, 12);
          M1 = _mm_srli_si128 (M1,12);
        }
      }
      M1 = _mm_hadd_epi16 (M1, M1);
      M1 = _mm_abs_epi16 (M1);

      //  (1 + 3 * abs (t)) >> 1
      M0 = _mm_add_epi16(M1,M1);
      M1 = _mm_add_epi16(M0,M1);
      M1 = _mm_add_epi16(M1,scale1);
      M1 = _mm_srai_epi16 (M1,1);

      M1 = _mm_hadds_epi16 (M1, M1);
      M1 = _mm_hadds_epi16 (M1, M1);
      _mm_storeu_si16 (&act, M1);

      taAct += (uint64_t)act;
    }
  }
  return (taAct);
}
template<X86_VEXT vext>
uint64_t AvgHighPassWithDownsamplingDiff2nd_SIMD (const int width,const int height,const Pel* pSrc,const Pel* pSrcM1,const Pel* pSrcM2,const int iSrcStride,const int iSM1Stride,const int iSM2Stride)
{
  uint64_t taAct = 0;
  uint16_t act = 0;
  int32_t y;
  int32_t x;
  pSrc -= iSrcStride;
  pSrc -= iSrcStride;
  pSrcM1-=iSM1Stride;
  pSrcM1-=iSM1Stride;
  pSrcM2-=iSM2Stride;
  pSrcM2-=iSM2Stride;

  for (y = 2; y < height-2; y += 2)
  {
    for (x = 2; x < width-2-10; x += 8)
    {
      __m128i lineM0u = _mm_loadu_si128 ((__m128i*) &pSrc  [ y   *iSrcStride + x]); /* load 8 16-bit values */
      __m128i lineM0d = _mm_loadu_si128 ((__m128i*) &pSrc  [(y+1)*iSrcStride + x]);
      __m128i lineM1u = _mm_loadu_si128 ((__m128i*) &pSrcM1[ y   *iSM1Stride + x]);
      __m128i lineM1d = _mm_loadu_si128 ((__m128i*) &pSrcM1[(y+1)*iSM1Stride + x]);
      __m128i lineM2u = _mm_loadu_si128 ((__m128i*) &pSrcM2[ y   *iSM2Stride + x]);
      __m128i lineM2d = _mm_loadu_si128 ((__m128i*) &pSrcM2[(y+1)*iSM2Stride + x]);

      __m128i M0 = _mm_add_epi16 (lineM0u, lineM0d);
      __m128i M1 = _mm_add_epi16 (lineM1u, lineM1d);
      __m128i M2 = _mm_add_epi16 (lineM2u, lineM2d);

      M0 = _mm_add_epi16 (M0, M2);
      M0 = _mm_hadd_epi16 (M0, M1);
      M1 = _mm_shuffle_epi32 (M0, 0xee);
      M1 = _mm_slli_epi16 (M1, 0x1);
      M1 = _mm_sub_epi16 (M0, M1);
      M1 = _mm_abs_epi16 (M1);
      M1 = _mm_hadds_epi16 (M1, M1);
      M1 = _mm_hadds_epi16 (M1, M1);

      _mm_storeu_si16 (&act, M1);
      taAct += (uint64_t) act;
    }
    // last collum
    {
      __m128i lineM0u = _mm_loadu_si128 ((__m128i*) &pSrc  [ y   *iSrcStride + x]); /* load 8 16-bit values */
      __m128i lineM0d = _mm_loadu_si128 ((__m128i*) &pSrc  [(y+1)*iSrcStride + x]);
      __m128i lineM1u = _mm_loadu_si128 ((__m128i*) &pSrcM1[ y   *iSM1Stride + x]);
      __m128i lineM1d = _mm_loadu_si128 ((__m128i*) &pSrcM1[(y+1)*iSM1Stride + x]);
      __m128i lineM2u = _mm_loadu_si128 ((__m128i*) &pSrcM2[ y   *iSM2Stride + x]);
      __m128i lineM2d = _mm_loadu_si128 ((__m128i*) &pSrcM2[(y+1)*iSM2Stride + x]);

      __m128i M0 = _mm_add_epi16 (lineM0u, lineM0d);
      __m128i M1 = _mm_add_epi16 (lineM1u, lineM1d);
      __m128i M2 = _mm_add_epi16 (lineM2u, lineM2d);

      M0 = _mm_add_epi16 (M0, M2);
      int n=8-width+2+x;
      if (n > 0)
      {
        //remove n Pixel
        if (n==2)
        {
          M0 = _mm_slli_si128 (M0, 4);
          M0 = _mm_srli_si128 (M0,4);
          M1 = _mm_slli_si128 (M1, 4);
          M1 = _mm_srli_si128 (M1,4);
        }
        else if  (n==4)
        {
          M0 = _mm_slli_si128 (M0, 8);
          M0 = _mm_srli_si128 (M0,8);
          M1 = _mm_slli_si128 (M1, 8);
          M1 = _mm_srli_si128 (M1,8);
        }
        else if  (n==6)
        {
          M0 = _mm_slli_si128 (M0, 12);
          M0 = _mm_srli_si128 (M0,12);
          M1 = _mm_slli_si128 (M1, 12);
          M1 = _mm_srli_si128 (M1,12);
        }
      }

      M0 = _mm_hadd_epi16 (M0, M1);
      M1 = _mm_shuffle_epi32 (M0, 0xee);
      M1 = _mm_slli_epi16 (M1, 0x1);
      M1 = _mm_sub_epi16 (M0, M1);
      M1 = _mm_abs_epi16 (M1);
      M1 = _mm_hadds_epi16 (M1, M1);
      M1 = _mm_hadds_epi16 (M1, M1);

      _mm_storeu_si16 (&act, M1);
      taAct += (uint64_t) act;
    }
  }
  return taAct ;
}

// ---------------------------------------------------------------------------------------------------------------------
// Picture and block statistics, border padding and 2x2 downsampling.
// The sums of squares use madd on 16-bit values: for sample values in 0 ... 2^15 - 1 the differences fit 16 bit and
// the pair sums of squares are below 2^31, so they are exact; they are widened to 64 bit before they are accumulated.
// ---------------------------------------------------------------------------------------------------------------------

// adds the four non-negative 32-bit lanes of v to the two 64-bit lanes of acc
static inline __m128i _vv_addUnsigned32To64( __m128i acc, __m128i v )
{
  return _mm_add_epi64( acc, _mm_add_epi64( _mm_unpacklo_epi32( v, _mm_setzero_si128() ), _mm_unpackhi_epi32( v, _mm_setzero_si128() ) ) );
}

// adds the four signed 32-bit lanes of v to the two 64-bit lanes of acc
static inline __m128i _vv_addSigned32To64( __m128i acc, __m128i v )
{
  return _mm_add_epi64( acc, _mm_add_epi64( _mm_cvtepi32_epi64( v ), _mm_cvtepi32_epi64( _mm_unpackhi_epi64( v, v ) ) ) );
}

static inline int64_t _vv_sum_epi64( __m128i v )
{
  int64_t lanes[2];
  _mm_storeu_si128( ( __m128i* ) lanes, v );
  return lanes[0] + lanes[1];
}

#if USE_AVX2
static inline __m256i _vv_addUnsigned32To64( __m256i acc, __m256i v )
{
  return _mm256_add_epi64( acc, _mm256_add_epi64( _mm256_unpacklo_epi32( v, _mm256_setzero_si256() ), _mm256_unpackhi_epi32( v, _mm256_setzero_si256() ) ) );
}

static inline __m128i _vv_fold_epi64( __m256i v )
{
  return _mm_add_epi64( _mm256_castsi256_si128( v ), _mm256_extracti128_si256( v, 1 ) );
}
#endif

// sum over the block of ( src0 - src1 )^2, for samples in 0 ... 2^15 - 1
template<X86_VEXT vext>
uint64_t sumSquaredError_SIMD( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1, const int width, const int height )
{
  uint64_t sum = 0;
  __m128i  acc = _mm_setzero_si128();
#if USE_AVX2
  __m256i  acc256 = _mm256_setzero_si256();
#endif

  for( int y = 0; y < height; y++, src0 += stride0, src1 += stride1 )
  {
    int x = 0;
#if USE_AVX2
    if( vext >= AVX2 )
    {
      for( ; x + 16 <= width; x += 16 )
      {
        const __m256i diff = _mm256_sub_epi16( _mm256_loadu_si256( ( const __m256i* ) &src0[x] ), _mm256_loadu_si256( ( const __m256i* ) &src1[x] ) );
        acc256 = _vv_addUnsigned32To64( acc256, _mm256_madd_epi16( diff, diff ) );
      }
    }
#endif
    for( ; x + 8 <= width; x += 8 )
    {
      const __m128i diff = _mm_sub_epi16( _mm_loadu_si128( ( const __m128i* ) &src0[x] ), _mm_loadu_si128( ( const __m128i* ) &src1[x] ) );
      acc = _vv_addUnsigned32To64( acc, _mm_madd_epi16( diff, diff ) );
    }
    if( x + 4 <= width )
    {
      const __m128i diff = _mm_sub_epi16( _vv_loadl_epi64( ( const __m128i* ) &src0[x] ), _vv_loadl_epi64( ( const __m128i* ) &src1[x] ) );
      acc = _vv_addUnsigned32To64( acc, _mm_madd_epi16( diff, diff ) );
      x += 4;
    }
    for( ; x < width; x++ )
    {
      const int diff = src0[x] - src1[x];
      sum += uint64_t( diff * diff );
    }
  }
#if USE_AVX2
  acc = _mm_add_epi64( acc, _vv_fold_epi64( acc256 ) );
#endif
  return sum + uint64_t( _vv_sum_epi64( acc ) );
}

// sum of the samples of a block and, if sumSquares is not null, of their squares
template<X86_VEXT vext, bool withSquares>
static inline void sumBlock_SIMD( const Pel* src, const ptrdiff_t stride, const int width, const int height, int64_t& sum, int64_t& sumSquares )
{
  const __m128i ones  = _mm_set1_epi16( 1 );
  __m128i       accSum = _mm_setzero_si128();
  __m128i       accSq  = _mm_setzero_si128();
  int64_t       tail   = 0, tailSq = 0;

  for( int y = 0; y < height; y++, src += stride )
  {
    // per row, the 32-bit pair sums cannot overflow: |sum| <= width * 2^15
    __m128i rowSum = _mm_setzero_si128();
    int x = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      const __m128i v = _mm_loadu_si128( ( const __m128i* ) &src[x] );
      rowSum = _mm_add_epi32( rowSum, _mm_madd_epi16( v, ones ) );
      if( withSquares )
      {
        // the pair sums of squares are at most 2^31, i.e. unsigned 32 bit
        accSq = _vv_addUnsigned32To64( accSq, _mm_madd_epi16( v, v ) );
      }
    }
    if( x + 4 <= width )
    {
      const __m128i v = _vv_loadl_epi64( ( const __m128i* ) &src[x] );
      rowSum = _mm_add_epi32( rowSum, _mm_madd_epi16( v, ones ) );
      if( withSquares )
      {
        accSq = _vv_addUnsigned32To64( accSq, _mm_madd_epi16( v, v ) );
      }
      x += 4;
    }
    accSum = _vv_addSigned32To64( accSum, rowSum );
    for( ; x < width; x++ )
    {
      tail   += src[x];
      tailSq += src[x] * src[x];
    }
  }
  sum = tail + _vv_sum_epi64( accSum );
  if( withSquares )
  {
    sumSquares = tailSq + _vv_sum_epi64( accSq );
  }
}

template<X86_VEXT vext>
int64_t sum_SIMD( const Pel* src, const ptrdiff_t stride, const int width, const int height )
{
  int64_t sum = 0, unused = 0;
  sumBlock_SIMD<vext, false>( src, stride, width, height, sum, unused );
  return sum;
}

template<X86_VEXT vext>
void sumAndSumSquares_SIMD( const Pel* src, const ptrdiff_t stride, const int width, const int height, int64_t& sum, int64_t& sumSquares )
{
  sumBlock_SIMD<vext, true>( src, stride, width, height, sum, sumSquares );
}

// sets n samples to value
template<X86_VEXT vext>
static inline void fillRun_SIMD( Pel* dst, const int n, const Pel value )
{
  if( n < 8 )
  {
    // short runs (e.g. the ALF padding): one 64-bit store of four samples instead of a vector broadcast
    const uint64_t four = uint64_t( uint16_t( value ) ) * 0x0001000100010001ull;
    int x = 0;
    if( n >= 4 )
    {
      memcpy( dst, &four, sizeof( four ) );
      x = 4;
    }
    for( ; x < n; x++ )
    {
      dst[x] = value;
    }
    return;
  }

  int x = 0;
#if USE_AVX2
  if( vext >= AVX2 && n >= 16 )
  {
    const __m256i v = _mm256_set1_epi16( value );
    for( ; x + 16 <= n; x += 16 )
    {
      _mm256_storeu_si256( ( __m256i* ) &dst[x], v );
    }
  }
#endif
  const __m128i v = _mm_set1_epi16( value );
  for( ; x + 8 <= n; x += 8 )
  {
    _mm_storeu_si128( ( __m128i* ) &dst[x], v );
  }
  if( x + 4 <= n )
  {
    _vv_storel_epi64( ( __m128i* ) &dst[x], v );
    x += 4;
  }
  for( ; x < n; x++ )
  {
    dst[x] = value;
  }
}

// replicates the first / last sample of each row into marginLeft / marginRight samples left / right of the row
template<X86_VEXT vext>
void padHorizontal_SIMD( Pel* src, const ptrdiff_t stride, const int width, const int height, const int marginLeft, const int marginRight )
{
  for( int y = 0; y < height; y++, src += stride )
  {
    fillRun_SIMD<vext>( src - marginLeft, marginLeft, src[0] );
    fillRun_SIMD<vext>( src + width, marginRight, src[width - 1] );
  }
}

// dst = ( sum of the 2x2 source samples + 2 ) >> 2
template<X86_VEXT vext>
void downsample2x2_SIMD( const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, const int dstWidth, const int dstHeight )
{
  const __m128i ones = _mm_set1_epi16( 1 );
  const __m128i vrnd = _mm_set1_epi32( 2 );

  for( int y = 0; y < dstHeight; y++, src += 2 * srcStride, dst += dstStride )
  {
    const Pel* below = src + srcStride;
    int x = 0;
#if USE_AVX2
    if( vext >= AVX2 )
    {
      const __m256i ones256 = _mm256_set1_epi16( 1 );
      const __m256i vrnd256 = _mm256_set1_epi32( 2 );
      for( ; x + 16 <= dstWidth; x += 16 )
      {
        // madd with ones adds the horizontal pairs
        __m256i lo = _mm256_add_epi32( _mm256_madd_epi16( _mm256_loadu_si256( ( const __m256i* ) &src  [2 * x] ), ones256 ),
                                       _mm256_madd_epi16( _mm256_loadu_si256( ( const __m256i* ) &below[2 * x] ), ones256 ) );
        __m256i hi = _mm256_add_epi32( _mm256_madd_epi16( _mm256_loadu_si256( ( const __m256i* ) &src  [2 * x + 16] ), ones256 ),
                                       _mm256_madd_epi16( _mm256_loadu_si256( ( const __m256i* ) &below[2 * x + 16] ), ones256 ) );
        lo = _mm256_srai_epi32( _mm256_add_epi32( lo, vrnd256 ), 2 );
        hi = _mm256_srai_epi32( _mm256_add_epi32( hi, vrnd256 ), 2 );
        // packs works per 128-bit lane: reorder the 64-bit quarters to restore the sample order
        _mm256_storeu_si256( ( __m256i* ) &dst[x], _mm256_permute4x64_epi64( _mm256_packs_epi32( lo, hi ), 0xD8 ) );
      }
    }
#endif
    for( ; x + 8 <= dstWidth; x += 8 )
    {
      __m128i lo = _mm_add_epi32( _mm_madd_epi16( _mm_loadu_si128( ( const __m128i* ) &src  [2 * x] ), ones ),
                                  _mm_madd_epi16( _mm_loadu_si128( ( const __m128i* ) &below[2 * x] ), ones ) );
      __m128i hi = _mm_add_epi32( _mm_madd_epi16( _mm_loadu_si128( ( const __m128i* ) &src  [2 * x + 8] ), ones ),
                                  _mm_madd_epi16( _mm_loadu_si128( ( const __m128i* ) &below[2 * x + 8] ), ones ) );
      lo = _mm_srai_epi32( _mm_add_epi32( lo, vrnd ), 2 );
      hi = _mm_srai_epi32( _mm_add_epi32( hi, vrnd ), 2 );
      _mm_storeu_si128( ( __m128i* ) &dst[x], _mm_packs_epi32( lo, hi ) );
    }
    for( ; x < dstWidth; x++ )
    {
      dst[x] = Pel( ( src[2 * x] + below[2 * x] + src[2 * x + 1] + below[2 * x + 1] + 2 ) >> 2 );
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// MIP upsampling (MatrixIntraPrediction::predBlock): linear interpolation between the reduced prediction and the
// boundary, out = ( ( before << log2Factor ) + rounding + ( pos + 1 ) * ( behind - before ) ) >> log2Factor.
// For sample values in 0 ... 2^12 - 1 every intermediate value is in 0 ... 2^15 - 1, so 16-bit lanes are exact.
// ---------------------------------------------------------------------------------------------------------------------

// the upsampling factor values of one vector of "before" / "behind" samples, phase p in out[p]
template<unsigned log2Factor>
static inline void mipInterpolate_SSE( const __m128i before, const __m128i behind, __m128i* out )
{
  const __m128i diff = _mm_sub_epi16( behind, before );
  __m128i scaled     = _mm_add_epi16( _mm_slli_epi16( before, log2Factor ), _mm_set1_epi16( 1 << ( log2Factor - 1 ) ) );
  for( int p = 0; p < ( 1 << log2Factor ); p++ )
  {
    scaled = _mm_add_epi16( scaled, diff );
    out[p] = _mm_srai_epi16( scaled, log2Factor );
  }
}

// vertical upsampling: each input row k gives the output rows k * factor ... k * factor + factor - 1; the lanes run
// over the columns
template<unsigned log2Factor>
static void mipUpsampleVer_SSE( Pel* dst, const Pel* src, const Pel* bndry, const SizeType outWidth, const SizeType srcStep, const SizeType inHeight )
{
  const int factor = 1 << log2Factor;
  CHECKD( outWidth & 3, "MIP blocks are at least 4 samples wide" );

  for( SizeType x = 0; x < outWidth; x += 8 )
  {
    // 4 or 8 columns
    const bool four = x + 8 > outWidth;
    auto load  = [four]( const Pel* p ) { return four ? _vv_loadl_epi64( ( const __m128i* ) p ) : _mm_loadu_si128( ( const __m128i* ) p ); };
    auto store = [four]( Pel* p, __m128i v ) { if( four ) _vv_storel_epi64( ( __m128i* ) p, v ); else _mm_storeu_si128( ( __m128i* ) p, v ); };

    __m128i before = load( bndry + x );
    for( SizeType k = 0; k < inHeight; k++ )
    {
      // the source rows can be the output rows k * factor + factor - 1 (in-place upsampling), so load before storing
      const __m128i behind = load( src + k * srcStep + x );
      __m128i out[8];
      mipInterpolate_SSE<log2Factor>( before, behind, out );
      for( int p = 0; p < factor; p++ )
      {
        store( dst + ( k * factor + p ) * outWidth + x, out[p] );
      }
      before = behind;
    }
  }
}

// horizontal upsampling of predSize (4 or 8) rows of predSize samples; "before" of the first sample of a row is the
// left boundary sample of the row
template<unsigned log2Factor>
static void mipUpsampleHor_SSE( Pel* dst, const Pel* src, const Pel* bndry, const SizeType dstStride, const SizeType bndryStep, const SizeType predSize )
{
  CHECKD( predSize != 4 && predSize != 8, "MIP reduced prediction is 4x4 or 8x8" );
  const Pel* bndryLine = bndry + bndryStep - 1;

  for( SizeType row = 0; row < predSize; row++, src += predSize, dst += dstStride, bndryLine += bndryStep )
  {
    const __m128i behind = predSize == 8 ? _mm_loadu_si128( ( const __m128i* ) src ) : _vv_loadl_epi64( ( const __m128i* ) src );
    const __m128i before = _mm_insert_epi16( _mm_slli_si128( behind, 2 ), *bndryLine, 0 );
    __m128i v[8];
    mipInterpolate_SSE<log2Factor>( before, behind, v );

    // interleave the phases: out[i * factor + p] = v[p][i]
    if( log2Factor == 1 )
    {
      _mm_storeu_si128( ( __m128i* ) dst, _mm_unpacklo_epi16( v[0], v[1] ) );
      if( predSize == 8 )
      {
        _mm_storeu_si128( ( __m128i* ) &dst[8], _mm_unpackhi_epi16( v[0], v[1] ) );
      }
    }
    else if( log2Factor == 2 )
    {
      const __m128i lo01 = _mm_unpacklo_epi16( v[0], v[1] ), lo23 = _mm_unpacklo_epi16( v[2], v[3] );
      _mm_storeu_si128( ( __m128i* ) &dst[0], _mm_unpacklo_epi32( lo01, lo23 ) );
      _mm_storeu_si128( ( __m128i* ) &dst[8], _mm_unpackhi_epi32( lo01, lo23 ) );
      if( predSize == 8 )
      {
        const __m128i hi01 = _mm_unpackhi_epi16( v[0], v[1] ), hi23 = _mm_unpackhi_epi16( v[2], v[3] );
        _mm_storeu_si128( ( __m128i* ) &dst[16], _mm_unpacklo_epi32( hi01, hi23 ) );
        _mm_storeu_si128( ( __m128i* ) &dst[24], _mm_unpackhi_epi32( hi01, hi23 ) );
      }
    }
    else
    {
      // 8x8 transpose of 16-bit values: output vector i holds the 8 phases of sample i
      const __m128i a0 = _mm_unpacklo_epi16( v[0], v[1] ), a1 = _mm_unpacklo_epi16( v[2], v[3] );
      const __m128i a2 = _mm_unpacklo_epi16( v[4], v[5] ), a3 = _mm_unpacklo_epi16( v[6], v[7] );
      const __m128i a4 = _mm_unpackhi_epi16( v[0], v[1] ), a5 = _mm_unpackhi_epi16( v[2], v[3] );
      const __m128i a6 = _mm_unpackhi_epi16( v[4], v[5] ), a7 = _mm_unpackhi_epi16( v[6], v[7] );
      const __m128i b0 = _mm_unpacklo_epi32( a0, a1 ), b1 = _mm_unpackhi_epi32( a0, a1 );
      const __m128i b2 = _mm_unpacklo_epi32( a2, a3 ), b3 = _mm_unpackhi_epi32( a2, a3 );
      const __m128i b4 = _mm_unpacklo_epi32( a4, a5 ), b5 = _mm_unpackhi_epi32( a4, a5 );
      const __m128i b6 = _mm_unpacklo_epi32( a6, a7 ), b7 = _mm_unpackhi_epi32( a6, a7 );
      _mm_storeu_si128( ( __m128i* ) &dst[ 0], _mm_unpacklo_epi64( b0, b2 ) );
      _mm_storeu_si128( ( __m128i* ) &dst[ 8], _mm_unpackhi_epi64( b0, b2 ) );
      _mm_storeu_si128( ( __m128i* ) &dst[16], _mm_unpacklo_epi64( b1, b3 ) );
      _mm_storeu_si128( ( __m128i* ) &dst[24], _mm_unpackhi_epi64( b1, b3 ) );
      if( predSize == 8 )
      {
        _mm_storeu_si128( ( __m128i* ) &dst[32], _mm_unpacklo_epi64( b4, b6 ) );
        _mm_storeu_si128( ( __m128i* ) &dst[40], _mm_unpackhi_epi64( b4, b6 ) );
        _mm_storeu_si128( ( __m128i* ) &dst[48], _mm_unpacklo_epi64( b5, b7 ) );
        _mm_storeu_si128( ( __m128i* ) &dst[56], _mm_unpackhi_epi64( b5, b7 ) );
      }
    }
  }
}

template<X86_VEXT vext>
void mipUpsampleHor_SIMD( Pel* dst, const Pel* src, const Pel* bndry, const SizeType dstStride, const SizeType bndryStep, const SizeType predSize, const unsigned log2Factor )
{
  switch( log2Factor )
  {
  case 1:  mipUpsampleHor_SSE<1>( dst, src, bndry, dstStride, bndryStep, predSize ); break;
  case 2:  mipUpsampleHor_SSE<2>( dst, src, bndry, dstStride, bndryStep, predSize ); break;
  default: mipUpsampleHor_SSE<3>( dst, src, bndry, dstStride, bndryStep, predSize ); break;
  }
}

template<X86_VEXT vext>
void mipUpsampleVer_SIMD( Pel* dst, const Pel* src, const Pel* bndry, const SizeType outWidth, const SizeType srcStep, const SizeType inHeight, const unsigned log2Factor )
{
  switch( log2Factor )
  {
  case 1:  mipUpsampleVer_SSE<1>( dst, src, bndry, outWidth, srcStep, inHeight ); break;
  case 2:  mipUpsampleVer_SSE<2>( dst, src, bndry, outWidth, srcStep, inHeight ); break;
  default: mipUpsampleVer_SSE<3>( dst, src, bndry, outWidth, srcStep, inHeight ); break;
  }
}

// Sum over the block of hp( cb ) * hp( cr ) with the 3x3 high-pass hp = 12 * c - 2 * ( 4 direct neighbours ) - ( 4
// diagonal neighbours ), which decides the joint Cb-Cr sign of a picture (setJointCbCrModes). For samples of up to
// 10 bits |hp| <= 24 * 1023 fits 16 bits and a pair of products fits the 32-bit lanes of madd; the sums are
// accumulated in 64 bit. Higher bit depths use the C code.
template<typename V>
static inline V highPass3x3_SIMD( const Pel* p, const ptrdiff_t stride );

template<>
inline __m128i highPass3x3_SIMD<__m128i>( const Pel* p, const ptrdiff_t stride )
{
  auto ld = [&]( ptrdiff_t off ) { return _mm_loadu_si128( ( const __m128i* ) ( p + off ) ); };
  const __m128i c       = ld( 0 );
  const __m128i direct  = _mm_add_epi16( _mm_add_epi16( ld( -1 ), ld( 1 ) ), _mm_add_epi16( ld( -stride ), ld( stride ) ) );
  const __m128i diag    = _mm_add_epi16( _mm_add_epi16( ld( -1 - stride ), ld( 1 - stride ) ), _mm_add_epi16( ld( -1 + stride ), ld( 1 + stride ) ) );
  const __m128i c12     = _mm_add_epi16( _mm_slli_epi16( c, 3 ), _mm_slli_epi16( c, 2 ) );
  return _mm_sub_epi16( _mm_sub_epi16( c12, _mm_slli_epi16( direct, 1 ) ), diag );
}

#if USE_AVX2
template<>
inline __m256i highPass3x3_SIMD<__m256i>( const Pel* p, const ptrdiff_t stride )
{
  auto ld = [&]( ptrdiff_t off ) { return _mm256_loadu_si256( ( const __m256i* ) ( p + off ) ); };
  const __m256i c       = ld( 0 );
  const __m256i direct  = _mm256_add_epi16( _mm256_add_epi16( ld( -1 ), ld( 1 ) ), _mm256_add_epi16( ld( -stride ), ld( stride ) ) );
  const __m256i diag    = _mm256_add_epi16( _mm256_add_epi16( ld( -1 - stride ), ld( 1 - stride ) ), _mm256_add_epi16( ld( -1 + stride ), ld( 1 + stride ) ) );
  const __m256i c12     = _mm256_add_epi16( _mm256_slli_epi16( c, 3 ), _mm256_slli_epi16( c, 2 ) );
  return _mm256_sub_epi16( _mm256_sub_epi16( c12, _mm256_slli_epi16( direct, 1 ) ), diag );
}
#endif

template<X86_VEXT vext>
int64_t highPassCorrelation_SIMD( const Pel* cb, const ptrdiff_t cbStride, const Pel* cr, const ptrdiff_t crStride, const int width, const int height, const int bitDepth )
{
  if( bitDepth > 10 )
  {
    return highPassCorrelationCore( cb, cbStride, cr, crStride, width, height, bitDepth );
  }

  int64_t sum = 0;
  __m128i acc = _mm_setzero_si128();
#if USE_AVX2
  __m256i acc256 = _mm256_setzero_si256();
#endif

  for( int y = 0; y < height; y++, cb += cbStride, cr += crStride )
  {
    int x = 0;
#if USE_AVX2
    if( vext >= AVX2 )
    {
      for( ; x + 16 <= width; x += 16 )
      {
        const __m256i prod = _mm256_madd_epi16( highPass3x3_SIMD<__m256i>( cb + x, cbStride ), highPass3x3_SIMD<__m256i>( cr + x, crStride ) );
        acc256 = _mm256_add_epi64( acc256, _mm256_add_epi64( _mm256_cvtepi32_epi64( _mm256_castsi256_si128( prod ) ), _mm256_cvtepi32_epi64( _mm256_extracti128_si256( prod, 1 ) ) ) );
      }
    }
#endif
    for( ; x + 8 <= width; x += 8 )
    {
      const __m128i prod = _mm_madd_epi16( highPass3x3_SIMD<__m128i>( cb + x, cbStride ), highPass3x3_SIMD<__m128i>( cr + x, crStride ) );
      acc = _mm_add_epi64( acc, _mm_add_epi64( _mm_cvtepi32_epi64( prod ), _mm_cvtepi32_epi64( _mm_unpackhi_epi64( prod, prod ) ) ) );
    }
    sum += highPassCorrelationCore( cb + x, cbStride, cr + x, crStride, width - x, 1, bitDepth );
  }

#if USE_AVX2
  acc = _mm_add_epi64( acc, _vv_fold_epi64( acc256 ) );
#endif
  return sum + _vv_sum_epi64( acc );
}

// Statistics of a row of 4x4 blocks for the screen content detection (PreProcess::xDetectScc): per block the sample
// sum s and the absolute deviation sum( | s - 16 * p | ). For samples of up to 10 bits all values fit 16 bits; the
// deviations are added in 32 bit. Higher bit depths use the C code.
template<X86_VEXT vext>
void blockStats4x4_SIMD( const Pel* src, const ptrdiff_t stride, const int numBlocks, const int bitDepth, int* sum, int* absDev )
{
  if( bitDepth > 10 )
  {
    blockStats4x4Core( src, stride, numBlocks, bitDepth, sum, absDev );
    return;
  }

  int b = 0;
#if USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i ones = _mm256_set1_epi16( 1 );
    for( ; b + 4 <= numBlocks; b += 4, src += 16 )
    {
      __m256i rows[4];
      for( int h = 0; h < 4; h++ )
      {
        rows[h] = _mm256_loadu_si256( ( const __m256i* ) &src[h * stride] );
      }
      const __m256i cols  = _mm256_add_epi16( _mm256_add_epi16( rows[0], rows[1] ), _mm256_add_epi16( rows[2], rows[3] ) );
      const __m256i pairs = _mm256_madd_epi16( cols, ones );
      const __m256i sums  = _mm256_hadd_epi32( pairs, pairs ); // [ s0 s1 s0 s1 | s2 s3 s2 s3 ]
      // the sum of each block in its four lanes
      __m256i s16 = _mm256_packs_epi32( sums, sums );
      s16         = _mm256_unpacklo_epi16( s16, s16 );
      s16         = _mm256_unpacklo_epi32( s16, s16 );
      __m256i dev = _mm256_setzero_si256();
      for( int h = 0; h < 4; h++ )
      {
        dev = _mm256_add_epi32( dev, _mm256_madd_epi16( _mm256_abs_epi16( _mm256_sub_epi16( s16, _mm256_slli_epi16( rows[h], 4 ) ) ), ones ) );
      }
      dev = _mm256_hadd_epi32( dev, dev ); // [ d0 d1 d0 d1 | d2 d3 d2 d3 ]
      const __m128i s = _mm_unpacklo_epi64( _mm256_castsi256_si128( sums ), _mm256_extracti128_si256( sums, 1 ) );
      const __m128i d = _mm_unpacklo_epi64( _mm256_castsi256_si128( dev ), _mm256_extracti128_si256( dev, 1 ) );
      _mm_storeu_si128( ( __m128i* ) &sum[b], s );
      _mm_storeu_si128( ( __m128i* ) &absDev[b], d );
    }
  }
#endif
  const __m128i ones = _mm_set1_epi16( 1 );
  for( ; b + 2 <= numBlocks; b += 2, src += 8 )
  {
    __m128i rows[4];
    for( int h = 0; h < 4; h++ )
    {
      rows[h] = _mm_loadu_si128( ( const __m128i* ) &src[h * stride] );
    }
    const __m128i cols  = _mm_add_epi16( _mm_add_epi16( rows[0], rows[1] ), _mm_add_epi16( rows[2], rows[3] ) );
    const __m128i sums  = _mm_hadd_epi32( _mm_madd_epi16( cols, ones ), _mm_setzero_si128() ); // [ s0 s1 0 0 ]
    __m128i s16 = _mm_packs_epi32( sums, sums );
    s16         = _mm_unpacklo_epi16( s16, s16 );
    s16         = _mm_unpacklo_epi32( s16, s16 );
    __m128i dev = _mm_setzero_si128();
    for( int h = 0; h < 4; h++ )
    {
      dev = _mm_add_epi32( dev, _mm_madd_epi16( _mm_abs_epi16( _mm_sub_epi16( s16, _mm_slli_epi16( rows[h], 4 ) ) ), ones ) );
    }
    dev = _mm_hadd_epi32( dev, dev );
    _vv_storel_epi64( ( __m128i* ) &sum[b], sums );
    _vv_storel_epi64( ( __m128i* ) &absDev[b], dev );
  }
  blockStats4x4Core( src, stride, numBlocks - b, bitDepth, sum + b, absDev + b );
}

template<X86_VEXT vext>
void PelBufferOps::_initPelBufOpsX86()
{
  addAvg   = addAvg_SSE<vext>;
  reco     = recoCore_SSE<vext>;
  copyClip = copyClip_SSE<vext>;
  roundGeo = roundGeo_SSE<vext>;

  addAvg4  = addAvg_SSE<vext, 4>;
  addAvg8  = addAvg_SSE<vext, 8>;
  addAvg16 = addAvg_SSE<vext, 16>;

  sub4 = sub_SSE<vext, 4>;
  sub8 = sub_SSE<vext, 8>;

  copyClip4 = copyClip_SSE<vext, 4>;
  copyClip8 = copyClip_SSE<vext, 8>;

  reco4 = reco_SSE<vext, 4>;
  reco8 = reco_SSE<vext, 8>;

  linTf4 = linTf_SSE_entry<vext, 4>;
  linTf8 = linTf_SSE_entry<vext, 8>;

  copyBuffer = copyBufferSimd<vext>;

#if ENABLE_SIMD_OPT_BCW
  removeHighFreq8 = removeHighFreq_SSE<vext, 8>;
  removeHighFreq4 = removeHighFreq_SSE<vext, 4>;

  wghtAvg4 = addWghtAvg_SSE<vext, 4>;
  wghtAvg8 = addWghtAvg_SSE<vext, 8>;

#endif
  transpose4x4   = transposeNxN_SSE<vext, 4>;
  transpose8x8   = transposeNxN_SSE<vext, 8>;
  roundIntVector = roundIntVector_SIMD<vext>;

  mipMatrixMul_4_4 = mipMatrixMul_SSE<vext, 4, 4>;
  mipMatrixMul_8_4 = mipMatrixMul_SSE<vext, 8, 4>;
  mipMatrixMul_8_8 = mipMatrixMul_SSE<vext, 8, 8>;
  mipUpsampleHor   = mipUpsampleHor_SIMD<vext>;
  mipUpsampleVer   = mipUpsampleVer_SIMD<vext>;
  highPassCorrelation = highPassCorrelation_SIMD<vext>;
  blockStats4x4       = blockStats4x4_SIMD<vext>;

  weightCiip = weightCiip_SSE<vext>;

  fillPtrMap = fillPtrMap_SIMD<vext>;

  AvgHighPassWithDownsampling = AvgHighPassWithDownsampling_SIMD<vext>;
  AvgHighPass = AvgHighPass_SIMD<vext>;
  AvgHighPassWithDownsamplingDiff1st = AvgHighPassWithDownsamplingDiff1st_SIMD<vext>;
  AvgHighPassWithDownsamplingDiff2nd = AvgHighPassWithDownsamplingDiff2nd_SIMD<vext>;
  HDHighPass = HDHighPass_SIMD<vext>;
  HDHighPass2 = HDHighPass2_SIMD<vext>;

  sumSquaredError  = sumSquaredError_SIMD<vext>;
  sum              = sum_SIMD<vext>;
  sumAndSumSquares = sumAndSumSquares_SIMD<vext>;
  padHorizontal    = padHorizontal_SIMD<vext>;
  downsample2x2    = downsample2x2_SIMD<vext>;
}

template void PelBufferOps::_initPelBufOpsX86<SIMDX86>();


} // namespace vvenc

//! \}

#endif // ENABLE_SIMD_OPT_BUFFER
#endif // TARGET_SIMD_X86


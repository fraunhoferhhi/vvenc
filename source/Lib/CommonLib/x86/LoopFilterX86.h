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
/** \file     LoopFilterX86.h
    \brief    deblocking filter simd code
*/

#pragma once

#include "CommonDefX86.h"
#include "Rom.h"
#include "LoopFilter.h"

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {
  

#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

template<X86_VEXT vext>
inline void xPelLumaCore( int64_t m0, int64_t& m1, int64_t& m2, int64_t& m3, int64_t& m4, int64_t& m5, int64_t& m6, int64_t m7, const int tc )
{
  static constexpr int64_t val4  = 4ll | ( 4ll << 32ll ) | ( ( 4ll | ( 4ll << 32ll ) ) << 16ll );
  static constexpr int64_t val2  = 2ll | ( 2ll << 32ll ) | ( ( 2ll | ( 2ll << 32ll ) ) << 16ll );

  const int64_t m1234        = m1 + m2 + m3 + m4;
  const int64_t m3456        = m3 + m4 + m5 + m6;

  const int64_t r1           = m1234 + val4 + ( ( m0 + m1 ) << 1 );
  const int64_t r2           = m1234 + val2;
  const int64_t r3           = ( m1234 << 1 ) + m5 + val4 - m1;
  const int64_t r4           = ( m3456 << 1 ) + m2 + val4 - m6;
  const int64_t r5           = m3456 + val2;
  const int64_t r6           = m3456 + val4 + ( ( m6 + m7 ) << 1 );

  const char tc3[3]          = { 3, 2, 1 };

  const __m128i vtc0  = _mm_set1_epi16   ( tc3[0] * tc );
  const __m128i vtc1  = _mm_set1_epi16   ( tc3[1] * tc );
  const __m128i vtc2  = _mm_set1_epi16   ( tc3[2] * tc );
  const __m128i vzero = _mm_setzero_si128();

  __m128i vmax  = vtc1;
  __m128i vmin  = _mm_sub_epi16    ( vzero, vmax );

  __m128i org   = _mm_set_epi64x   ( m2,    m5 );
  __m128i vec   = _mm_set_epi64x   ( r2,    r5 );
  vec           = _mm_srli_epi16   ( vec,   2 );
  vec           = _mm_sub_epi16    ( vec,   org );
  vec           = _mm_min_epi16    ( vmax,  _mm_max_epi16( vmin, vec ) );
  vec           = _mm_add_epi16    ( vec,   org );
  m5            = _mm_cvtsi128_si64( vec );
  m2            = _mm_extract_epi64( vec,   1 );

  vmax          = _mm_blend_epi16  ( vtc0, vtc2, 0xf0 );
  vmin          = _mm_sub_epi16    ( vzero, vmax );
  org           = _mm_set_epi64x   ( m1,    m3 );
  vec           = _mm_set_epi64x   ( r1,    r3 );
  vec           = _mm_srli_epi16   ( vec,   3 );
  vec           = _mm_sub_epi16    ( vec,   org );
  vec           = _mm_min_epi16    ( vmax,  _mm_max_epi16( vmin, vec ) );
  vec           = _mm_add_epi16    ( vec,   org );
  m3            = _mm_cvtsi128_si64( vec );
  m1            = _mm_extract_epi64( vec,   1 );
  
  vmax          = _mm_blend_epi16  ( vtc2, vtc0, 0xf0 );
  vmin          = _mm_sub_epi16    ( vzero, vmax );
  org           = _mm_set_epi64x   ( m4,    m6 );
  vec           = _mm_set_epi64x   ( r4,    r6 );
  vec           = _mm_srli_epi16   ( vec,   3 );
  vec           = _mm_sub_epi16    ( vec,   org );
  vec           = _mm_min_epi16    ( vmax,  _mm_max_epi16( vmin, vec ) );
  vec           = _mm_add_epi16    ( vec,   org );
  m6            = _mm_cvtsi128_si64( vec );
  m4            = _mm_extract_epi64( vec,   1 );
}

template<X86_VEXT vext>
static inline void xPelFilterLumaLoopHor( Pel* piSrc, const ptrdiff_t offset, const int tc )
{
  int64_t  m0 = *( ( int64_t* ) &piSrc[-4 * offset] );
  int64_t &m1 = *( ( int64_t* ) &piSrc[-3 * offset] );
  int64_t &m2 = *( ( int64_t* ) &piSrc[-2 * offset] );
  int64_t &m3 = *( ( int64_t* ) &piSrc[-1 * offset] );
  int64_t &m4 = *( ( int64_t* ) &piSrc[ 0         ] );
  int64_t &m5 = *( ( int64_t* ) &piSrc[ 1 * offset] );
  int64_t &m6 = *( ( int64_t* ) &piSrc[ 2 * offset] );
  int64_t  m7 = *( ( int64_t* ) &piSrc[ 3 * offset] );

  xPelLumaCore<vext>( m0, m1, m2, m3, m4, m5, m6, m7, tc );
}

template<X86_VEXT vext>
static inline void xPelFilterLumaLoopVer( Pel* piSrc, const ptrdiff_t step, const int tc )
{
  //// Transpose the 8x4 matrix:  a0 a4 (ax in N^(1,4)) into 4x8: [m0 m1 m2 m3 m4 m5 m6 m7] (mx in N^(4,1))
  ////                            a1 a5
  ////                            a2 a6
  ////                            a3 a7

  __m128i va01      = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 0 * step] );
  __m128i va23      = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 1 * step] );
  __m128i va45      = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 2 * step] );
  __m128i va67      = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 3 * step] );

  __m128i va01a23lo = _mm_unpacklo_epi16( va01, va23 );
  __m128i va01a23hi = _mm_unpackhi_epi16( va01, va23 );
  __m128i va45a67lo = _mm_unpacklo_epi16( va45, va67 );
  __m128i va45a67hi = _mm_unpackhi_epi16( va45, va67 );

  va01 = _mm_unpacklo_epi32( va01a23lo, va45a67lo );
  va23 = _mm_unpackhi_epi32( va01a23lo, va45a67lo );
  va45 = _mm_unpacklo_epi32( va01a23hi, va45a67hi );
  va67 = _mm_unpackhi_epi32( va01a23hi, va45a67hi );

  int64_t m0 = _mm_extract_epi64( va01, 0 );
  int64_t m1 = _mm_extract_epi64( va01, 1 );
  int64_t m2 = _mm_extract_epi64( va23, 0 );
  int64_t m3 = _mm_extract_epi64( va23, 1 );
  int64_t m4 = _mm_extract_epi64( va45, 0 );
  int64_t m5 = _mm_extract_epi64( va45, 1 );
  int64_t m6 = _mm_extract_epi64( va67, 0 );
  int64_t m7 = _mm_extract_epi64( va67, 1 );

  // do the loop filter of the the 4x8 matrix
  xPelLumaCore<vext>( m0, m1, m2, m3, m4, m5, m6, m7, tc );

  // Transpose back
  va01 = _mm_set_epi64x( m4, m0 );
  va23 = _mm_set_epi64x( m5, m1 );
  va45 = _mm_set_epi64x( m6, m2 );
  va67 = _mm_set_epi64x( m7, m3 );

  va01a23lo = _mm_unpacklo_epi16( va01, va23 );
  va01a23hi = _mm_unpackhi_epi16( va01, va23 );
  va45a67lo = _mm_unpacklo_epi16( va45, va67 );
  va45a67hi = _mm_unpackhi_epi16( va45, va67 );

  va01 = _mm_unpacklo_epi32( va01a23lo, va45a67lo );
  va23 = _mm_unpackhi_epi32( va01a23lo, va45a67lo );
  va45 = _mm_unpacklo_epi32( va01a23hi, va45a67hi );
  va67 = _mm_unpackhi_epi32( va01a23hi, va45a67hi );

  __m128i vzr = _mm_setzero_si128();

  _mm_storel_epi64( ( __m128i* ) &piSrc[-4 + 0 * step], _mm_unpacklo_epi64( va01, vzr ) );
  _mm_storel_epi64( ( __m128i* ) &piSrc[-4 + 1 * step], _mm_unpackhi_epi64( va01, vzr ) );
  _mm_storel_epi64( ( __m128i* ) &piSrc[-4 + 2 * step], _mm_unpacklo_epi64( va23, vzr ) );
  _mm_storel_epi64( ( __m128i* ) &piSrc[-4 + 3 * step], _mm_unpackhi_epi64( va23, vzr ) );
  _mm_storel_epi64( ( __m128i* ) &piSrc[ 0 + 0 * step], _mm_unpacklo_epi64( va45, vzr ) );
  _mm_storel_epi64( ( __m128i* ) &piSrc[ 0 + 1 * step], _mm_unpackhi_epi64( va45, vzr ) );
  _mm_storel_epi64( ( __m128i* ) &piSrc[ 0 + 2 * step], _mm_unpacklo_epi64( va67, vzr ) );
  _mm_storel_epi64( ( __m128i* ) &piSrc[ 0 + 3 * step], _mm_unpackhi_epi64( va67, vzr ) );
}

template<X86_VEXT vext>
static inline void xPelFilterLumaWeakCore( __m128i &vm1, __m128i &vm2, __m128i &vm3, __m128i &vm4, __m128i &vm5, __m128i &vm6, const int tc, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng )
{
  __m128i vmin = _mm_set1_epi16( clpRng.min );
  __m128i vmax = _mm_set1_epi16( clpRng.max );

  __m128i vtmp0 = _mm_sub_epi16( vm4, vm3 );
  __m128i vtmp1 = _mm_sub_epi16( vm5, vm2 );
  __m128i vm3s  = vm3;
  __m128i vm4s  = vm4;

  vtmp0 = _mm_unpacklo_epi16( vtmp0, vtmp1 );
  vtmp1 = _mm_set1_epi32( 0xfffd0009 );

  vtmp0 = _mm_madd_epi16( vtmp0, vtmp1 );
  vtmp1 = _mm_add_epi32( vtmp0, _mm_set1_epi32( 8 ) );
  __m128i vdlt = _mm_srai_epi32( vtmp1, 4 );
  vdlt = _mm_packs_epi32( vdlt, _mm_setzero_si128() );

  short deltaV[4] = { 0, 0, 0, 0 };
  _mm_storel_epi64( ( __m128i* ) deltaV, vdlt );

  __m128i vmsk = _mm_cmpgt_epi16( _mm_set1_epi16( iThrCut ), _mm_abs_epi16( vdlt ) );
  __m128i vtc  = _mm_set1_epi16( -tc );
  vdlt = _mm_max_epi16( vdlt, vtc );
  vtc  = _mm_set1_epi16( tc );
  vdlt = _mm_min_epi16( vdlt, vtc );
  vtc  = _mm_set1_epi16( tc >> 1 );
  __m128i vtcn = _mm_set1_epi16( -( tc >> 1 ) );

  vtmp0 = _mm_add_epi16( vm3, vdlt );
  vtmp1 = _mm_min_epi16( _mm_max_epi16( vmin, vtmp0 ), vmax );
  vtmp0 = _mm_blendv_epi8( vm3, vtmp1, vmsk );
  vm3   = _mm_unpacklo_epi64( vtmp0, _mm_setzero_si128() );
  //_mm_storel_epi64( ( __m128i* ) &m3, vtmp0 );

  if( bFilterSecondP )
  {
    vtmp0 = _mm_srli_epi16( _mm_add_epi16( _mm_add_epi16( vm1, vm3s ), _mm_set1_epi16( 1 ) ), 1 );
    vtmp1 = _mm_srai_epi16( _mm_add_epi16( _mm_sub_epi16( vtmp0, vm2 ), vdlt ), 1 );
    vtmp0 = _mm_min_epi16( _mm_max_epi16( vtmp1, vtcn ), vtc );

    vtmp1 = _mm_add_epi16( vm2, vtmp0 );
    vtmp0 = _mm_min_epi16( _mm_max_epi16( vmin, vtmp1 ), vmax );
    vtmp1 = _mm_blendv_epi8( vm2, vtmp0, vmsk );
    vm2   = _mm_unpacklo_epi64( vtmp1, _mm_setzero_si128() );
    //_mm_storel_epi64( ( __m128i* ) &m2, vtmp1 );
  }

  vtmp0 = _mm_sub_epi16( vm4, vdlt );
  vtmp1 = _mm_min_epi16( _mm_max_epi16( vmin, vtmp0 ), vmax );
  vtmp0 = _mm_blendv_epi8( vm4, vtmp1, vmsk );
  vm4   = _mm_unpacklo_epi64( vtmp0, _mm_setzero_si128() );
  //_mm_storel_epi64( ( __m128i* ) &m4, vtmp0 );

  if( bFilterSecondQ )
  {
    vtmp0 = _mm_srli_epi16( _mm_add_epi16( _mm_add_epi16( vm6, vm4s ), _mm_set1_epi16( 1 ) ), 1 );
    vtmp1 = _mm_srai_epi16( _mm_sub_epi16( _mm_sub_epi16( vtmp0, vm5 ), vdlt ), 1 );
    vtmp0 = _mm_min_epi16( _mm_max_epi16( vtmp1, vtcn ), vtc );
        
    vtmp1 = _mm_add_epi16( vm5, vtmp0 );
    vtmp0 = _mm_min_epi16( _mm_max_epi16( vmin, vtmp1 ), vmax );
    vtmp1 = _mm_blendv_epi8( vm5, vtmp0, vmsk );
    vm5   = _mm_unpacklo_epi64( vtmp1, _mm_setzero_si128() );
    //_mm_storel_epi64( ( __m128i* ) &m5, vtmp1 );
  }
}

template<X86_VEXT vext>
static void xPelFilterLumaX86( Pel* piSrc, const ptrdiff_t step, const ptrdiff_t offset, const int tc, const bool sw, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng )
{
  if( sw )
  {
    if( offset == 1 )
    {
      xPelFilterLumaLoopVer<vext>( piSrc, step,   tc );
    }
    else
    {
      xPelFilterLumaLoopHor<vext>( piSrc, offset, tc );
    }
  }
  else
  {
    if( offset == 1 )
    {
      //// Transpose the 8x4 matrix:  a0 a4 (ax in N^(1,4)) into 4x8: [m0 m1 m2 m3 m4 m5 m6 m7] (mx in N^(4,1))
      ////                            a1 a5
      ////                            a2 a6
      ////                            a3 a7

      __m128i va01 = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 0 * step] );
      __m128i va23 = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 1 * step] );
      __m128i va45 = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 2 * step] );
      __m128i va67 = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 3 * step] );

      __m128i va01a23lo = _mm_unpacklo_epi16( va01, va23 );
      __m128i va01a23hi = _mm_unpackhi_epi16( va01, va23 );
      __m128i va45a67lo = _mm_unpacklo_epi16( va45, va67 );
      __m128i va45a67hi = _mm_unpackhi_epi16( va45, va67 );

      va01 = _mm_unpacklo_epi32( va01a23lo, va45a67lo );
      va23 = _mm_unpackhi_epi32( va01a23lo, va45a67lo );
      va45 = _mm_unpacklo_epi32( va01a23hi, va45a67hi );
      va67 = _mm_unpackhi_epi32( va01a23hi, va45a67hi );

      __m128i vzr = _mm_setzero_si128();
      __m128i vm1 = _mm_unpackhi_epi64( va01, vzr );
      __m128i vm2 = _mm_unpacklo_epi64( va23, vzr );
      __m128i vm3 = _mm_unpackhi_epi64( va23, vzr );
      __m128i vm4 = _mm_unpacklo_epi64( va45, vzr );
      __m128i vm5 = _mm_unpackhi_epi64( va45, vzr );
      __m128i vm6 = _mm_unpacklo_epi64( va67, vzr );

      // do the loop filter of the the 4x8 matrix
      xPelFilterLumaWeakCore<vext>( vm1, vm2, vm3, vm4, vm5, vm6,
                                    tc, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );

      va01a23lo = _mm_unpacklo_epi16( vm2, vm3 );
      va01a23hi = _mm_unpackhi_epi16( vm2, vm3 );
      va45a67lo = _mm_unpacklo_epi16( vm4, vm5 );
      va45a67hi = _mm_unpackhi_epi16( vm4, vm5 );

      va01 = _mm_unpacklo_epi32( va01a23lo, va45a67lo );
      va23 = _mm_unpackhi_epi32( va01a23lo, va45a67lo );

      _mm_storel_epi64( ( __m128i * ) &piSrc[-2 + 0 * step], _mm_unpacklo_epi64( va01, vzr ) );
      _mm_storel_epi64( ( __m128i * ) &piSrc[-2 + 1 * step], _mm_unpackhi_epi64( va01, vzr ) );
      _mm_storel_epi64( ( __m128i * ) &piSrc[-2 + 2 * step], _mm_unpacklo_epi64( va23, vzr ) );
      _mm_storel_epi64( ( __m128i * ) &piSrc[-2 + 3 * step], _mm_unpackhi_epi64( va23, vzr ) );
    }
    else
    {
      __m128i vm1 = _mm_loadl_epi64( ( const __m128i * ) &piSrc[-3 * offset] );
      __m128i vm2 = _mm_loadl_epi64( ( const __m128i * ) &piSrc[-2 * offset] );
      __m128i vm3 = _mm_loadl_epi64( ( const __m128i * ) &piSrc[-1 * offset] );
      __m128i vm4 = _mm_loadl_epi64( ( const __m128i * ) &piSrc[ 0 * offset] );
      __m128i vm5 = _mm_loadl_epi64( ( const __m128i * ) &piSrc[ 1 * offset] );
      __m128i vm6 = _mm_loadl_epi64( ( const __m128i * ) &piSrc[ 2 * offset] );

      xPelFilterLumaWeakCore<vext>( vm1, vm2, vm3, vm4, vm5, vm6,
                                    tc, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );

      _mm_storel_epi64( ( __m128i * ) &piSrc[-2 * offset], vm2 );
      _mm_storel_epi64( ( __m128i * ) &piSrc[-1 * offset], vm3 );
      _mm_storel_epi64( ( __m128i * ) &piSrc[ 0 * offset], vm4 );
      _mm_storel_epi64( ( __m128i * ) &piSrc[ 1 * offset], vm5 );
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

static const int  dbCoeffs7[7] = { 59, 50, 41, 32, 23, 14,  5 };
static const int  dbCoeffs5[5] = { 58, 45, 32, 19,  6 };
static const int  dbCoeffs3[3] = { 53, 32, 11 };
static const int  tc7[7]       = { 6, 5, 4, 3, 2, 1, 1 };
static const int  tc3[3]       = { 6, 4, 2 };

template<X86_VEXT vext>
static inline void xFilteringPandQX86Hor( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc )
{
  CHECKD( step != 1, "Offset has to be '1' for vertical edge filtering!" );

  const int* dbCoeffsP  = numberPSide == 7 ? dbCoeffs7 : ( numberPSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const int* dbCoeffsQ  = numberQSide == 7 ? dbCoeffs7 : ( numberQSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const int* tcP        = numberPSide == 3 ? tc3 : tc7;
  const int* tcQ        = numberQSide == 3 ? tc3 : tc7;

  uint64_t refPx4 = 0, refQx4 = 0, refMiddlex4 = 0;

  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    Pel* srcP = src + step * i - offset;
    Pel* srcQ = src + step * i;

    Pel refMiddle = 0;
    const Pel refP = ( srcP[-numberPSide * offset] + srcP[-( numberPSide - 1 ) * offset] + 1 ) >> 1;
    const Pel refQ = ( srcQ[ numberQSide * offset] + srcQ[ ( numberQSide - 1 ) * offset] + 1 ) >> 1;

    if( numberPSide == numberQSide )
    {
      if( numberPSide == 5 )
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] ) + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + 8 ) >> 4;
      }
      else
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] ) + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + +srcP[-6 * offset] + srcQ[6 * offset] + 8 ) >> 4;
      }
    }
    else
    {
      Pel* srcPt = srcP;
      Pel* srcQt = srcQ;
      ptrdiff_t offsetP = -offset;
      ptrdiff_t offsetQ = offset;

      int newNumberQSide = numberQSide;
      int newNumberPSide = numberPSide;

      if( numberQSide > numberPSide )
      {
        std::swap( srcPt, srcQt );
        std::swap( offsetP, offsetQ );
        newNumberQSide = numberPSide;
        newNumberPSide = numberQSide;
      }

      if( newNumberPSide == 7 && newNumberQSide == 5 )
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] ) + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + 8 ) >> 4;
      }
      else if( newNumberPSide == 7 && newNumberQSide == 3 )
      {
        refMiddle = ( 2 * ( srcPt[0] + srcQt[0] ) + srcQt[0] + 2 * ( srcQt[offsetQ] + srcQt[2 * offsetQ] ) + srcPt[offsetP] + srcQt[offsetQ] + srcPt[2 * offsetP] + srcPt[3 * offsetP] + srcPt[4 * offsetP] + srcPt[5 * offsetP] + srcPt[6 * offsetP] + 8 ) >> 4;
      }
      else
      {
        refMiddle = ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + 4 ) >> 3;
      }
    }

    refPx4      |= ( int64_t( refP      ) << ( int64_t( i ) << 4 ) );
    refQx4      |= ( int64_t( refQ      ) << ( int64_t( i ) << 4 ) );
    refMiddlex4 |= ( int64_t( refMiddle ) << ( int64_t( i ) << 4 ) );
  }

  Pel* srcP = src - offset;
  Pel* srcQ = src;

  for( int pos = 0; pos < numberPSide; pos++ )
  {
    __m128i vref1 = _mm_set_epi64x( 0, refPx4 );
    __m128i vref0 = _mm_set_epi64x( 0, refMiddlex4 );
    __m128i vsrc  = _mm_loadl_epi64( ( const __m128i* ) &srcP[-offset * pos] );
    __m128i vmax  = _mm_set1_epi16( ( tc * tcP[pos] ) >> 1 );
    __m128i vmin  = _mm_sub_epi16( vsrc, vmax );
    vmax          = _mm_add_epi16( vsrc, vmax );
    vref0         = _mm_unpacklo_epi16( vref0, vref1 );
    __m128i vtmp  = _mm_set1_epi32( dbCoeffsP[pos] | ( ( 64 - dbCoeffsP[pos] ) << 16 ) );
    vtmp          = _mm_madd_epi16( vref0, vtmp );
    vtmp          = _mm_add_epi32( vtmp, _mm_set1_epi32( 32 ) );
    vtmp          = _mm_srli_epi32( vtmp, 6 );
    vtmp          = _mm_packs_epi32( vtmp, vtmp );
    vtmp          = _mm_min_epi16( _mm_max_epi16( vtmp, vmin ), vmax );
    _mm_storel_epi64( ( __m128i* ) &srcP[-offset * pos], vtmp );
  }

  for( int pos = 0; pos < numberQSide; pos++ )
  {
    __m128i vref1 = _mm_set_epi64x( 0, refQx4 );
    __m128i vref0 = _mm_set_epi64x( 0, refMiddlex4 );
    __m128i vsrc  = _mm_loadl_epi64( ( const __m128i* ) &srcQ[offset * pos] );
    __m128i vmax  = _mm_set1_epi16( ( tc * tcQ[pos] ) >> 1 );
    __m128i vmin  = _mm_sub_epi16( vsrc, vmax );
    vmax          = _mm_add_epi16( vsrc, vmax );
    vref0         = _mm_unpacklo_epi16( vref0, vref1 );
    __m128i vtmp  = _mm_set1_epi32( dbCoeffsQ[pos] | ( ( 64 - dbCoeffsQ[pos] ) << 16 ) );
    vtmp          = _mm_madd_epi16( vref0, vtmp );
    vtmp          = _mm_add_epi32( vtmp, _mm_set1_epi32( 32 ) );
    vtmp          = _mm_srli_epi32( vtmp, 6 );
    vtmp          = _mm_packs_epi32( vtmp, vtmp );
    vtmp          = _mm_min_epi16( _mm_max_epi16( vtmp, vmin ), vmax );
    _mm_storel_epi64( ( __m128i* ) &srcQ[offset * pos], vtmp );
  }
}


template<X86_VEXT vext>
static inline void xFilteringPandQX86Ver( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc )
{
  CHECKD( offset != 1, "Offset has to be '1' for vertical edge filtering!" );

  const int* dbCoeffsP = numberPSide == 7 ? dbCoeffs7 : ( numberPSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const int* dbCoeffsQ = numberQSide == 7 ? dbCoeffs7 : ( numberQSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const int* tcP       = numberPSide == 3 ? tc3 : tc7;
  const int* tcQ       = numberQSide == 3 ? tc3 : tc7;

#if USE_AVX2
  short tcQarr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  short tcParr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  short dbQarr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  short dbParr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  for( int i = 0; i < numberQSide; i++ ) tcQarr[    i] = static_cast<short>( ( tcQ      [i] * tc ) >> 1 );
  for( int i = 0; i < numberPSide; i++ ) tcParr[7 - i] = static_cast<short>( ( tcP      [i] * tc ) >> 1 );
  for( int i = 0; i < numberQSide; i++ ) dbQarr[    i] = static_cast<short>( dbCoeffsQ[i]               );
  for( int i = 0; i < numberPSide; i++ ) dbParr[7 - i] = static_cast<short>( dbCoeffsP[i]               );

#endif
  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    Pel* srcP = src + step * i - offset;
    Pel* srcQ = src + step * i;

    Pel refMiddle = 0;
    const Pel refP = ( srcP[-numberPSide * offset] + srcP[-( numberPSide - 1 ) * offset] + 1 ) >> 1;
    const Pel refQ = ( srcQ[ numberQSide * offset] + srcQ[ ( numberQSide - 1 ) * offset] + 1 ) >> 1;

    if( numberPSide == numberQSide )
    {
      if( numberPSide == 5 )
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] ) + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + 8 ) >> 4;
      }
      else
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] ) + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + +srcP[-6 * offset] + srcQ[6 * offset] + 8 ) >> 4;
      }
    }
    else
    {
      Pel* srcPt = srcP;
      Pel* srcQt = srcQ;
      ptrdiff_t offsetP = -offset;
      ptrdiff_t offsetQ = offset;

      int newNumberQSide = numberQSide;
      int newNumberPSide = numberPSide;

      if( numberQSide > numberPSide )
      {
        std::swap( srcPt, srcQt );
        std::swap( offsetP, offsetQ );
        newNumberQSide = numberPSide;
        newNumberPSide = numberQSide;
      }

      if( newNumberPSide == 7 && newNumberQSide == 5 )
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] ) + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + 8 ) >> 4;
      }
      else if( newNumberPSide == 7 && newNumberQSide == 3 )
      {
        refMiddle = ( 2 * ( srcPt[0] + srcQt[0] ) + srcQt[0] + 2 * ( srcQt[offsetQ] + srcQt[2 * offsetQ] ) + srcPt[offsetP] + srcQt[offsetQ] + srcPt[2 * offsetP] + srcPt[3 * offsetP] + srcPt[4 * offsetP] + srcPt[5 * offsetP] + srcPt[6 * offsetP] + 8 ) >> 4;
      }
      else
      {
        refMiddle = ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + 4 ) >> 3;
      }
    }

#if USE_AVX2
    if( vext >= AVX2 )
    {
      __m256i ydbp, ytmp, ydst;
      __m128i vtmp, vmin, vmax, vsrcq;

      // P-part
      vsrcq
           = _mm_loadu_si128      ( ( const __m128i * ) srcQ );
      vtmp = _mm_loadu_si128      ( ( const __m128i * ) ( srcP - 7 * offset ) );
      vmax = _mm_loadu_si128      ( ( const __m128i * ) tcParr );
      vmin = _mm_sub_epi16        ( vtmp, vmax );
      vmax = _mm_add_epi16        ( vtmp, vmax );
      vtmp = _mm_loadu_si128      ( ( const __m128i * ) dbParr );
      ytmp = _mm256_cvtepu16_epi32( vtmp );
      ytmp = _mm256_or_si256      ( ytmp, _mm256_slli_si256( ytmp, 2 ) );
      ydbp = _mm256_abs_epi16     ( _mm256_sub_epi16( _mm256_set1_epi32( 64 ), ytmp ) );
      ytmp = _mm256_set1_epi32    ( refP | ( refMiddle << 16 ) );
      ydst = _mm256_madd_epi16    ( ydbp, ytmp );
      ydst = _mm256_add_epi32     ( ydst, _mm256_set1_epi32( 32 ) );
      ydst = _mm256_srli_epi32    ( ydst, 6 );
      vtmp = _mm256_cvtepi32_epi16x( ydst );

      vtmp = _mm_max_epi16( _mm_min_epi16( vtmp, vmax ), vmin );
      if( numberPSide == 7 )
      {
        vtmp = _mm_srli_si128( vtmp, 2 );
        _mm_storeu_si128( ( __m128i * ) ( srcP - 6 ), vtmp );
      }
      else if( numberPSide == 5 )
      {
        vtmp = _mm_srli_si128( vtmp, 6 );
        _mm_storeu_si128( ( __m128i * ) ( srcP - 4 ), vtmp );
      }
      else
      {
        vtmp = _mm_srli_si128( vtmp, 10 );
        _mm_storel_epi64( ( __m128i * ) ( srcP - 2 ), vtmp );
      }
      
      // Q-part
      vmax = _mm_loadu_si128      ( ( const __m128i * ) tcQarr );
      vmin = _mm_sub_epi16        ( vsrcq, vmax );
      vmax = _mm_add_epi16        ( vsrcq, vmax );
      vtmp = _mm_loadu_si128      ( ( const __m128i * ) dbQarr );
      ytmp = _mm256_cvtepu16_epi32( vtmp );
      ytmp = _mm256_or_si256      ( ytmp, _mm256_slli_si256( ytmp, 2 ) );
      ydbp = _mm256_abs_epi16     ( _mm256_sub_epi16( _mm256_set1_epi32( 64 ), ytmp ) );
      ytmp = _mm256_set1_epi32    ( refQ | ( refMiddle << 16 ) );
      ydst = _mm256_madd_epi16    ( ydbp, ytmp );
      ydst = _mm256_add_epi32     ( ydst, _mm256_set1_epi32( 32 ) );
      ydst = _mm256_srli_epi32    ( ydst, 6 );
      vtmp = _mm256_cvtepi32_epi16x( ydst );

      vtmp = _mm_max_epi16( _mm_min_epi16( vtmp, vmax ), vmin );
      if( numberQSide != 3 )
      {
        _mm_storeu_si128( ( __m128i * ) srcQ, vtmp );
      }
      else
      {
        _mm_storel_epi64( ( __m128i * ) srcQ, vtmp );
      }
    }
    else
#endif
    {
      int srcval;

      for( int pos = 0; pos < numberPSide; pos++ )
      {
        srcval = srcP[-offset * pos];
        int cvalue = ( tc * tcP[pos] ) >> 1;
        srcP[-offset * pos] = Clip3( srcval - cvalue, srcval + cvalue, ( ( refMiddle * dbCoeffsP[pos] + refP * ( 64 - dbCoeffsP[pos] ) + 32 ) >> 6 ) );
      }

      for( int pos = 0; pos < numberQSide; pos++ )
      {
        srcval = srcQ[offset * pos];
        int cvalue = ( tc * tcQ[pos] ) >> 1;
        srcQ[offset * pos] = Clip3( srcval - cvalue, srcval + cvalue, ( ( refMiddle * dbCoeffsQ[pos] + refQ * ( 64 - dbCoeffsQ[pos] ) + 32 ) >> 6 ) );
      }
    }
  }
}

static inline void xBilinearFilter( Pel* srcP, Pel* srcQ, ptrdiff_t offset, int refMiddle, int refP, int refQ, int numberPSide, int numberQSide, const int* dbCoeffsP, const int* dbCoeffsQ, int tc )
{
  int src;
  const char tc7[7] = { 6, 5, 4, 3, 2, 1, 1 };
  const char tc3[3] = { 6, 4, 2 };
  const char *tcP = ( numberPSide == 3 ) ? tc3 : tc7;
  const char *tcQ = ( numberQSide == 3 ) ? tc3 : tc7;

  for( int pos = 0; pos < numberPSide; pos++ )
  {
    src = srcP[-offset * pos];
    int cvalue = ( tc * tcP[pos] ) >> 1;
    srcP[-offset * pos] = Clip3( src - cvalue, src + cvalue, ( ( refMiddle * dbCoeffsP[pos] + refP * ( 64 - dbCoeffsP[pos] ) + 32 ) >> 6 ) );
  }

  for( int pos = 0; pos < numberQSide; pos++ )
  {
    src = srcQ[offset * pos];
    int cvalue = ( tc * tcQ[pos] ) >> 1;
    srcQ[offset * pos] = Clip3( src - cvalue, src + cvalue, ( ( refMiddle * dbCoeffsQ[pos] + refQ * ( 64 - dbCoeffsQ[pos] ) + 32 ) >> 6 ) );
  }
}

template<X86_VEXT vext>
static void xFilteringPandQX86( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc )
{
  CHECKD( numberPSide <= 3 && numberQSide <= 3, "Short filtering in long filtering function" );
  if( step == 1 )
  {
    xFilteringPandQX86Hor<vext>( src, step, offset, numberPSide, numberQSide, tc );
  }
  else
  {
    xFilteringPandQX86Ver<vext>( src, step, offset, numberPSide, numberQSide, tc );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template <X86_VEXT vext>
void LoopFilter::_initLoopFilterX86()
{
  xPelFilterLuma  = xPelFilterLumaX86<vext>;
  xFilteringPandQ = xFilteringPandQX86<vext>;
}

template void LoopFilter::_initLoopFilterX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


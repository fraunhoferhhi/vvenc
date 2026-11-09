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
/** \file     RdCostX86.cpp
    \brief    RD cost computation class, SIMD version
*/

#pragma once

#include "CommonDefX86.h"
#include "Rom.h"
#include "RdCost.h"

#include <math.h>

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT_DIST

//! \ingroup CommonLib
//! \{

namespace vvenc {

typedef Pel Torg;
typedef Pel Tcur;

#ifdef USE_AVX2
// The sum of a vector's eight 32-bit lanes, wrapping as 32-bit adds do: the two 128-bit halves added, then within the half by
// two shuffles and adds. This takes fewer shuffle uops than two hadds and a cross-lane permute (on Intel the shuffles share
// port 5 with the kernels' own shuffles).
static ALWAYS_INLINE int32_t xSumOfLanes32_AVX2( const __m256i v )
{
  __m128i sum4 = _mm_add_epi32( _mm256_castsi256_si128( v ), _mm256_extracti128_si256( v, 1 ) );
  sum4         = _mm_add_epi32( sum4, _mm_shuffle_epi32( sum4, 0x4e ) );
  sum4         = _mm_add_epi32( sum4, _mm_shuffle_epi32( sum4, 0xb1 ) );
  return _mm_cvtsi128_si32( sum4 );
}
#endif

template<X86_VEXT vext >
Distortion RdCost::xGetSSE_SIMD( const DistParam &rcDtParam )
{
  const Torg* pSrc1     = (const Torg*)rcDtParam.org.buf;
  const Tcur* pSrc2     = (const Tcur*)rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iCols            = rcDtParam.org.width;
  const int iStrideSrc1 = rcDtParam.org.stride;
  const int iStrideSrc2 = rcDtParam.cur.stride;

  const uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  Distortion uiRet = 0;

  if( vext >= AVX2 && ( iCols & 15 ) == 0 )
  {
#ifdef USE_AVX2
    __m256i Sum = _mm256_setzero_si256();
    for( int iY = 0; iY < iRows; iY++ )
    {
      for( int iX = 0; iX < iCols; iX+=16 )
      {
        __m256i Src1 = ( _mm256_lddqu_si256( ( __m256i* )( &pSrc1[iX] ) ) );
        __m256i Src2 = ( _mm256_lddqu_si256( ( __m256i* )( &pSrc2[iX] ) ) );
        __m256i Diff = _mm256_sub_epi16( Src1, Src2 );
        __m256i Res = _mm256_madd_epi16( Diff, Diff );
        Sum = _mm256_add_epi32( Sum, Res );
      }
      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;
    }
    uiRet = xSumOfLanes32_AVX2( Sum ) >> uiShift;
#endif
  }
  else if( ( iCols & 7 ) == 0 )
  {
    __m128i Sum = _mm_setzero_si128();
    for( int iY = 0; iY < iRows; iY++ )
    {
      for( int iX = 0; iX < iCols; iX += 8 )
      {
        __m128i Src1 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadu_si128 ( ( const __m128i* )( &pSrc1[iX] ) ) ) : ( _mm_unpacklo_epi8( _vv_loadl_epi64( ( const __m128i* )( &pSrc1[iX] ) ), _mm_setzero_si128() ) );
        __m128i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadu_si128( ( const __m128i* )( &pSrc2[iX] ) ) ) : ( _mm_unpacklo_epi8( _vv_loadl_epi64( ( const __m128i* )( &pSrc2[iX] ) ), _mm_setzero_si128() ) );
        __m128i Diff = _mm_sub_epi16( Src1, Src2 );
        __m128i Res = _mm_madd_epi16( Diff, Diff );
        Sum = _mm_add_epi32( Sum, Res );
      }
      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;
    }
    Sum = _mm_hadd_epi32( Sum, Sum );
    Sum = _mm_hadd_epi32( Sum, Sum );
    uiRet = _mm_cvtsi128_si32( Sum )>>uiShift;
  }
  else
  {
    __m128i Sum = _mm_setzero_si128();
    for( int iY = 0; iY < iRows; iY++ )
    {
      for( int iX = 0; iX < iCols; iX += 4 )
      {
        __m128i Src1 = ( sizeof( Torg ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&pSrc1[iX] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&pSrc1[iX] ), _mm_setzero_si128() ) );
        __m128i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&pSrc2[iX] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&pSrc2[iX] ), _mm_setzero_si128() ) );
        __m128i Diff = _mm_sub_epi16( Src1, Src2 );
        __m128i Res = _mm_madd_epi16( Diff, Diff );
        Sum = _mm_add_epi32( Sum, Res );
      }
      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;
    }
    Sum = _mm_hadd_epi32( Sum, Sum );
    uiRet = _mm_cvtsi128_si32( Sum )>>uiShift;
  }

  return uiRet;
}


// four rows of two samples
static inline __m128i _vv_load2x4( const Pel* src, const ptrdiff_t stride )
{
  const __m128i r01 = _mm_unpacklo_epi32( _mm_loadu_si32( src ),              _mm_loadu_si32( &src[    stride] ) );
  const __m128i r23 = _mm_unpacklo_epi32( _mm_loadu_si32( &src[2 * stride] ), _mm_loadu_si32( &src[3 * stride] ) );
  return _mm_unpacklo_epi64( r01, r23 );
}

// The 2-wide case of RdCost::xGetSAD: the absolute differences of four rows per vector. Like xGetSAD_NxN_SIMD for
// blocks below 64 wide, it sums all rows instead of stopping early once maximumDistortionForEarlyExit is exceeded: the
// checks would cost more than they save, and every caller only compares the result with that threshold, which the
// full sum exceeds whenever the partial one does.
template<X86_VEXT vext>
Distortion xGetSAD2_SIMD( const DistParam& rcDtParam )
{
  CHECKD( rcDtParam.org.width != 2, "Invalid width" );
  CHECKD( rcDtParam.applyWeight, "Weighted SAD is not supported" );

  const Pel*      org        = rcDtParam.org.buf;
  const Pel*      cur        = rcDtParam.cur.buf;
  const int       subShift   = rcDtParam.subShift;
  const ptrdiff_t orgStride  = rcDtParam.org.stride << subShift;
  const ptrdiff_t curStride  = rcDtParam.cur.stride << subShift;
  const int       numRows    = rcDtParam.org.height >> subShift;
  const uint32_t  distShift  = DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
  const __m128i   ones       = _mm_set1_epi16( 1 );

  // the 32-bit lane sums cannot overflow: each adds at most 2 * 2^15 per four rows
  __m128i acc = _mm_setzero_si128();
  int row = 0;
  for( ; row + 4 <= numRows; row += 4, org += 4 * orgStride, cur += 4 * curStride )
  {
    const __m128i absDiff = _mm_abs_epi16( _mm_sub_epi16( _vv_load2x4( org, orgStride ), _vv_load2x4( cur, curStride ) ) );
    acc = _mm_add_epi32( acc, _mm_madd_epi16( absDiff, ones ) );
  }
  acc = _mm_add_epi32( acc, _mm_unpackhi_epi64( acc, acc ) );
  acc = _mm_add_epi32( acc, _mm_shuffle_epi32( acc, 1 ) );
  Distortion sum = Distortion( _mm_cvtsi128_si32( acc ) );
  for( ; row < numRows; row++, org += orgStride, cur += curStride )
  {
    sum += abs( org[0] - cur[0] ) + abs( org[1] - cur[1] );
  }

  sum <<= subShift;
  return sum >> distShift;
}

template<int iWidth, X86_VEXT vext >
Distortion RdCost::xGetSSE_NxN_SIMD( const DistParam &rcDtParam )
{
  const Torg* pSrc1     = (const Torg*)rcDtParam.org.buf;
  const Tcur* pSrc2     = (const Tcur*)rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  const int iStrideSrc1 = rcDtParam.org.stride;
  const int iStrideSrc2 = rcDtParam.cur.stride;

  const uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;
  Distortion uiRet = 0;

  if( 2 == iWidth )
  {
    // four rows per vector
    __m128i Sum = _mm_setzero_si128();
    int iY = 0;
    for( ; iY + 4 <= iRows; iY += 4 )
    {
      __m128i Diff = _mm_sub_epi16( _vv_load2x4( pSrc1, iStrideSrc1 ), _vv_load2x4( pSrc2, iStrideSrc2 ) );
      Sum = _mm_add_epi32( Sum, _mm_madd_epi16( Diff, Diff ) );
      pSrc1 += 4 * iStrideSrc1;
      pSrc2 += 4 * iStrideSrc2;
    }
    for( ; iY < iRows; iY++ )
    {
      const int iDiff0 = pSrc1[0] - pSrc2[0], iDiff1 = pSrc1[1] - pSrc2[1];
      uiRet += iDiff0 * iDiff0 + iDiff1 * iDiff1;
      pSrc1 += iStrideSrc1;
      pSrc2 += iStrideSrc2;
    }
    Sum = _mm_hadd_epi32( Sum, Sum );
    Sum = _mm_hadd_epi32( Sum, Sum );
    uiRet = ( uiRet + _mm_cvtsi128_si32( Sum ) ) >> uiShift;
  }
  else if( 4 == iWidth )
  {
    __m128i Sum = _mm_setzero_si128();
    for( int iY = 0; iY < iRows; iY++ )
    {
      __m128i Src1 = ( sizeof( Torg ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )pSrc1 ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)pSrc1 ), _mm_setzero_si128() ) );
      __m128i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )pSrc2 ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)pSrc2 ), _mm_setzero_si128() ) );
      pSrc1 += iStrideSrc1;
      pSrc2 += iStrideSrc2;
      __m128i Diff = _mm_sub_epi16( Src1, Src2 );
      __m128i Res  = _mm_madd_epi16( Diff, Diff );
      Sum = _mm_add_epi32( Sum, Res );
    }
    Sum = _mm_hadd_epi32( Sum, Sum );
    uiRet = _mm_cvtsi128_si32( Sum )>>uiShift;
  }
  else
  {
    if( vext >= AVX2 && iWidth >= 16 )
    {
#ifdef USE_AVX2
      __m256i Sum = _mm256_setzero_si256();
      for( int iY = 0; iY < iRows; iY++ )
      {
        for( int iX = 0; iX < iWidth; iX+=16 )
        {
          __m256i Src1 = ( sizeof( Torg ) > 1 ) ? ( _mm256_lddqu_si256( ( __m256i* )( &pSrc1[iX] ) ) ) : ( _mm256_unpacklo_epi8( _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_loadu_si128( ( __m128i* )( &pSrc1[iX] ) ) ), 0xD8 ), _mm256_setzero_si256() ) );
          __m256i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _mm256_lddqu_si256( ( __m256i* )( &pSrc2[iX] ) ) ) : ( _mm256_unpacklo_epi8( _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_loadu_si128( ( __m128i* )( &pSrc2[iX] ) ) ), 0xD8 ), _mm256_setzero_si256() ) );
          __m256i Diff = _mm256_sub_epi16( Src1, Src2 );
          __m256i Res = _mm256_madd_epi16( Diff, Diff );
          Sum = _mm256_add_epi32( Sum, Res );
        }
        pSrc1   += iStrideSrc1;
        pSrc2   += iStrideSrc2;
      }

      __m256i vzero = _mm256_setzero_si256();
      Sum = _mm256_add_epi64( _mm256_unpacklo_epi32( Sum, vzero ), _mm256_unpackhi_epi32( Sum, vzero )); 
      Sum = _mm256_add_epi64( Sum, _mm256_permute4x64_epi64( Sum, 14 ) ); 
      Sum = _mm256_add_epi64( Sum, _mm256_permute4x64_epi64( Sum, 1 ) ); 
      uiRet = _mm_cvtsi128_si64( _mm256_castsi256_si128( Sum ))>>uiShift;
#endif
    }
    else
    {
      __m128i Sum = _mm_setzero_si128();
      for( int iY = 0; iY < iRows; iY++ )
      {
        for( int iX = 0; iX < iWidth; iX+=8 )
        {
          __m128i Src1 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadu_si128( ( const __m128i* )( &pSrc1[iX] ) ) ) : ( _mm_unpacklo_epi8( _vv_loadl_epi64( ( const __m128i* )( &pSrc1[iX] ) ), _mm_setzero_si128() ) );
          __m128i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadu_si128( ( const __m128i* )( &pSrc2[iX] ) ) ) : ( _mm_unpacklo_epi8( _vv_loadl_epi64( ( const __m128i* )( &pSrc2[iX] ) ), _mm_setzero_si128() ) );
          __m128i Diff = _mm_sub_epi16( Src1, Src2 );
          __m128i Res = _mm_madd_epi16( Diff, Diff );
          Sum = _mm_add_epi32( Sum, Res );
        }
        pSrc1 += iStrideSrc1;
        pSrc2 += iStrideSrc2;
      }

      __m128i vzero = _mm_setzero_si128();
      Sum = _mm_add_epi64( _mm_unpacklo_epi32( Sum, vzero ), _mm_unpackhi_epi32( Sum, vzero ));
      uiRet = (_mm_cvtsi128_si64( Sum ) + _mm_extract_epi64( Sum, 1 ))>>uiShift;
    }
  }

  return uiRet;
}

template< X86_VEXT vext >
Distortion RdCost::xGetSAD_SIMD( const DistParam &rcDtParam )
{
  if( rcDtParam.org.width < 4 )
    return RdCost::xGetSAD( rcDtParam );

  const short* pSrc1   = (const short*)rcDtParam.org.buf;
  const short* pSrc2   = (const short*)rcDtParam.cur.buf;
  int  iRows           = rcDtParam.org.height;
  int  iCols           = rcDtParam.org.width;
  int  iSubShift       = rcDtParam.subShift;
  int  iSubStep        = ( 1 << iSubShift );
  const int iStrideSrc1 = rcDtParam.org.stride * iSubStep;
  const int iStrideSrc2 = rcDtParam.cur.stride * iSubStep;

  uint32_t uiSum = 0;
  if( vext >= AVX2 && ( iCols & 15 ) == 0 )
  {
#ifdef USE_AVX2
    // Do for width that multiple of 16
    __m256i vzero = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for( int iY = 0; iY < iRows; iY+=iSubStep )
    {
      __m256i vsum16 = vzero;
      for( int iX = 0; iX < iCols; iX+=16 )
      {
        __m256i vsrc1 = _mm256_lddqu_si256( ( __m256i* )( &pSrc1[iX] ) );
        __m256i vsrc2 = _mm256_lddqu_si256( ( __m256i* )( &pSrc2[iX] ) );
        vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );
      }
      __m256i vsumtemp = _mm256_add_epi32( _mm256_unpacklo_epi16( vsum16, vzero ), _mm256_unpackhi_epi16( vsum16, vzero ) );
      vsum32 = _mm256_add_epi32( vsum32, vsumtemp );
      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;
    }
    vsum32 = _mm256_hadd_epi32( vsum32, vzero );
    vsum32 = _mm256_hadd_epi32( vsum32, vzero );
    uiSum =  _mm_cvtsi128_si32( _mm256_castsi256_si128( vsum32 ) ) + _mm_cvtsi128_si32( _mm256_castsi256_si128( _mm256_permute2x128_si256( vsum32, vsum32, 0x11 ) ) );
#endif
  }
  else if( ( iCols & 7 ) == 0 )
  {
    // Do with step of 8
    __m128i vzero = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for( int iY = 0; iY < iRows; iY+=iSubStep )
    {
      __m128i vsum16 = vzero;
      for( int iX = 0; iX < iCols; iX+=8 )
      {
        __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* )( &pSrc1[iX] ) );
        __m128i vsrc2 = _mm_loadu_si128( ( const __m128i* )( &pSrc2[iX] ) );
        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
      }
      __m128i vsumtemp = _mm_add_epi32( _mm_unpacklo_epi16( vsum16, vzero ), _mm_unpackhi_epi16( vsum16, vzero ) );
      vsum32 = _mm_add_epi32( vsum32, vsumtemp );
      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;
    }
    vsum32 = _mm_hadd_epi32( vsum32, vzero );
    vsum32 = _mm_hadd_epi32( vsum32, vzero );
    uiSum  =  _mm_cvtsi128_si32( vsum32 );
  }
  else
  {
    // Do with step of 4
    CHECK( ( iCols & 3 ) != 0, "Not divisible by 4: " << iCols );
    __m128i vzero = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for( int iY = 0; iY < iRows; iY += iSubStep )
    {
      __m128i vsum16 = vzero;
      for( int iX = 0; iX < iCols; iX+=4 )
      {
        __m128i vsrc1 = _vv_loadl_epi64( ( const __m128i* )&pSrc1[iX] );
        __m128i vsrc2 = _vv_loadl_epi64( ( const __m128i* )&pSrc2[iX] );
        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
      }
      __m128i vsumtemp = _mm_add_epi32( _mm_unpacklo_epi16( vsum16, vzero ), _mm_unpackhi_epi16( vsum16, vzero ) );
      vsum32 = _mm_add_epi32( vsum32, vsumtemp );
      pSrc1 += iStrideSrc1;
      pSrc2 += iStrideSrc2;
    }
    vsum32 = _mm_hadd_epi32( vsum32, vzero );
    vsum32 = _mm_hadd_epi32( vsum32, vzero );
    uiSum  = _mm_cvtsi128_si32( vsum32 );
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}


template< int iWidth, X86_VEXT vext >
Distortion RdCost::xGetSAD_NxN_SIMD( const DistParam &rcDtParam )
{
  //  assert( rcDtParam.iCols == iWidth);
  const short* pSrc1   = (const short*)rcDtParam.org.buf;
  const short* pSrc2   = (const short*)rcDtParam.cur.buf;
  int  iRows           = rcDtParam.org.height;
  int  iSubShift       = rcDtParam.subShift;
  int  iSubStep        = ( 1 << iSubShift );
  const int iStrideSrc1 = rcDtParam.org.stride * iSubStep;
  const int iStrideSrc2 = rcDtParam.cur.stride * iSubStep;

  uint32_t uiSum = 0;

  if( iWidth == 4 )
  {
    if( iRows == 4 && iSubShift == 0 )
    {
      __m128i vzero = _mm_setzero_si128();
      __m128i vsrc1 = _mm_or_si128( _vv_loadl_epi64( ( const __m128i* )pSrc1 ), _mm_slli_si128( _vv_loadl_epi64( ( const __m128i* )( &pSrc1[iStrideSrc1] ) ), 8 ) );
      __m128i vsrc2 = _mm_or_si128( _vv_loadl_epi64( ( const __m128i* )pSrc2 ), _mm_slli_si128( _vv_loadl_epi64( ( const __m128i* )( &pSrc2[iStrideSrc2] ) ), 8 ) );
      __m128i vsum  = _mm_cvtepi16_epi32( _mm_hadd_epi16( _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ), vzero ) );

      vsrc1 = _mm_or_si128( _vv_loadl_epi64( ( const __m128i* )( &pSrc1[2 * iStrideSrc1] ) ), _mm_slli_si128( _vv_loadl_epi64( ( const __m128i* )( &pSrc1[3 * iStrideSrc1] ) ), 8 ) );
      vsrc2 = _mm_or_si128( _vv_loadl_epi64( ( const __m128i* )( &pSrc2[2 * iStrideSrc2] ) ), _mm_slli_si128( _vv_loadl_epi64( ( const __m128i* )( &pSrc2[3 * iStrideSrc2] ) ), 8 ) );
      vsum  = _mm_add_epi32( vsum, _mm_cvtepi16_epi32( _mm_hadd_epi16( _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ), vzero ) ) );
      vsum  = _mm_hadd_epi32( vsum, vzero );
      vsum  = _mm_hadd_epi32( vsum, vzero );

      uiSum = _mm_cvtsi128_si32( vsum );
    }
    else
    {
      __m128i vone = _mm_set1_epi16( 1 );
      __m128i vsum32 = _mm_setzero_si128();
      for( int iY = 0; iY < iRows; iY += iSubStep )
      {
        __m128i vsrc1 = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( const __m128i* )pSrc1 ) );
        __m128i vsrc2 = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( const __m128i* )pSrc2 ) );
        vsum32 = _mm_add_epi32( vsum32, _mm_abs_epi32( _mm_sub_epi32( vsrc1, vsrc2 ) ) );

        pSrc1 += iStrideSrc1;
        pSrc2 += iStrideSrc2;
      }
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      uiSum = _mm_cvtsi128_si32( vsum32 );
    }
  }
  else
  {
#ifdef USE_AVX2
    if( vext >= AVX2 && iWidth >= 16 )
    {
      static constexpr bool earlyExitAllowed = iWidth >= 64;
      // Do for width that multiple of 16
      __m256i vone   = _mm256_set1_epi16( 1 );
      __m256i vsum32 = _mm256_setzero_si256();

      int checkExit = 3;

      for( int iY = 0; iY < iRows; iY+=iSubStep )
      {
        __m256i vsrc1  = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) );
        __m256i vsrc2  = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );
        __m256i vsum16 = _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) );

        for( int iX = 16; iX < iWidth; iX+=16 )
        {
          vsrc1  = _mm256_loadu_si256( ( __m256i* )( &pSrc1[iX] ) );
          vsrc2  = _mm256_loadu_si256( ( __m256i* )( &pSrc2[iX] ) );
          vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );
        }

        __m256i vsumtemp = _mm256_madd_epi16( vsum16, vone );
        if( earlyExitAllowed ) vsum32 = _mm256_hadd_epi32( vsum32, vsumtemp );
        else                   vsum32 = _mm256_add_epi32 ( vsum32, vsumtemp );

        pSrc1   += iStrideSrc1;
        pSrc2   += iStrideSrc2;

        if( earlyExitAllowed && checkExit == 0 )
        {
          Distortion distTemp = _mm256_extract_epi32( vsum32, 0 ) + _mm256_extract_epi32( vsum32, 4 );
          distTemp <<= iSubShift;
          distTemp >>= DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
          if( distTemp > rcDtParam.maximumDistortionForEarlyExit ) return distTemp;
          checkExit = 3;
        }
        else if( earlyExitAllowed )
        {
          checkExit--;
        }
      }

      __m128i
      xsum32 = _mm_add_epi32( _mm256_castsi256_si128( vsum32 ), _mm256_extracti128_si256( vsum32, 1 ) );
      xsum32 = _mm_hadd_epi32( xsum32, xsum32 );
      xsum32 = _mm_hadd_epi32( xsum32, xsum32 );
      uiSum  = _mm_cvtsi128_si32( xsum32 );
    }
    else
#endif
    if( iRows == 16 && ( iWidth == 16 || iWidth == 8 ) && iSubShift == 1 && rcDtParam.bitDepth <= 10 )
    {
      static constexpr bool isWdt16 = iWidth >= 16;

      __m128i vone   = _mm_set1_epi16( 1 );
      __m128i vsum32 = _mm_setzero_si128();

      for( int i = 0; i < 2; i++ )
      {
        //0
        __m128i vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1) );
        __m128i vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2) );

        __m128i vsum16 = _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) );

        if( isWdt16 )
        {
          vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1 + 8) );
          vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2 + 8) );

          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
        }

        pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

        // 1
        vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1) );
        vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2) );

        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );

        if( isWdt16 )
        {
          vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1 + 8) );
          vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2 + 8) );

          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
        }

        pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

        // 2
        vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1) );
        vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2) );

        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );

        if( isWdt16 )
        {
          vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1 + 8) );
          vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2 + 8) );

          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
        }

        pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

        // 3
        vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1) );
        vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2) );

        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );

        if( isWdt16 )
        {
          vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1 + 8) );
          vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2 + 8) );

          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
        }

        pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

        vsum32 = _mm_add_epi32( vsum32, _mm_madd_epi16( vsum16, vone ) );
      }

      vsum32 = _mm_hadd_epi32( vsum32, vone );
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      uiSum = _mm_cvtsi128_si32( vsum32 );

      uiSum <<= 1;
      return uiSum >> DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
    }
    else
    {
      static constexpr bool earlyExitAllowed = iWidth >= 64;

      // For width that multiple of 8
      __m128i vone   = _mm_set1_epi16( 1 );
      __m128i vsum32 = _mm_setzero_si128();

      int checkExit = 3;

      for( int iY = 0; iY < iRows; iY+=iSubStep )
      {
        __m128i vsrc1  = _mm_loadu_si128( ( const __m128i* )( pSrc1 ) );
        __m128i vsrc2  = _mm_loadu_si128( ( const __m128i* )( pSrc2 ) );
        __m128i vsum16 = _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) );

        if( iWidth >= 16 )
        {
          vsrc1  = _mm_loadu_si128( ( const __m128i* )( &pSrc1[8] ) );
          vsrc2  = _mm_loadu_si128( ( const __m128i* )( &pSrc2[8] ) );
          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );

          for( int iX = 16; iX < iWidth; iX += 16 )
          {
            vsrc1  = _mm_loadu_si128( ( const __m128i* )( &pSrc1[iX] ) );
            vsrc2  = _mm_loadu_si128( ( const __m128i* )( &pSrc2[iX] ) );
            vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
            
            vsrc1  = _mm_loadu_si128( ( const __m128i* )( &pSrc1[iX + 8] ) );
            vsrc2  = _mm_loadu_si128( ( const __m128i* )( &pSrc2[iX + 8] ) );
            vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
          }
        }

        __m128i vsumtemp = _mm_madd_epi16( vsum16, vone );
        if( earlyExitAllowed ) vsum32 = _mm_hadd_epi32( vsum32, vsumtemp );
        else                   vsum32 = _mm_add_epi32 ( vsum32, vsumtemp );

        pSrc1   += iStrideSrc1;
        pSrc2   += iStrideSrc2;

        if( earlyExitAllowed && checkExit == 0 )
        {
          Distortion distTemp = _mm_cvtsi128_si32( vsum32 );
          distTemp <<= iSubShift;
          distTemp >>= DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
          if( distTemp > rcDtParam.maximumDistortionForEarlyExit ) return distTemp;
          checkExit = 3;
        }
        else if( earlyExitAllowed )
        {
          checkExit--;
        }
      }
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      uiSum =  _mm_cvtsi128_si32( vsum32 );
    }
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

#ifdef USE_AVX2
// The SADs of four reference positions, for the full search and the TZ search: the same values as four calls of
// xGetSAD_NxN_SIMD. One call per position would load each original row four times and sum
// across the vector four times; here both happen once. From width 64 on it stops early like that kernel, once all four
// sums are above maximumDistortionForEarlyExit: the callers only compare the results with that threshold.
// An absolute difference is below 2^11 (10-bit samples, and the doubled original minus the other prediction in
// bi-prediction search), so 16 of them added in one element stay below 2^15, the limit of madd's signed input.

static inline void xStoreFourSads( const __m128i sums, const DistParam& rcDtParam, Distortion* sads )
{
  uint32_t values[4];
  _mm_storeu_si128( ( __m128i* ) values, sums );
  for( int k = 0; k < 4; k++ )
  {
    sads[k] = ( values[k] << rcDtParam.subShift ) >> DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
  }
}

static inline __m256i xAddAbsDiff_AVX2( const __m256i sum16, const __m256i orgRow, const short* ref )
{
  return _mm256_add_epi16( sum16, _mm256_abs_epi16( _mm256_sub_epi16( orgRow, _mm256_loadu_si256( ( const __m256i* ) ref ) ) ) );
}

static inline __m128i xAddAbsDiff_SSE( const __m128i sum16, const __m128i orgRow, const short* ref )
{
  return _mm_add_epi16( sum16, _mm_abs_epi16( _mm_sub_epi16( orgRow, _mm_loadu_si128( ( const __m128i* ) ref ) ) ) );
}

// widths 16 and up
template<int iWidth>
static void xGetFourSads_AVX2( const DistParam& rcDtParam, const Pel* const* refs, Distortion* sads )
{
  const short* org       = ( const short* ) rcDtParam.org.buf;
  const short* ref0 = refs[0], *ref1 = refs[1], *ref2 = refs[2], *ref3 = refs[3];
  const int    rows      = rcDtParam.org.height;
  const int    rowStep   = 1 << rcDtParam.subShift;
  const int    orgStride = rcDtParam.org.stride * rowStep;
  const int    refStride = rcDtParam.cur.stride * rowStep;
  const int    rowsPerWidening = 16 * 16 / iWidth;   // each row adds iWidth / 16 differences to one element
  const __m256i one = _mm256_set1_epi16( 1 );

  // two horizontal adds leave the sum of each position's lane in element k of both lanes
  const auto storeSums = [&]( const __m256i s0, const __m256i s1, const __m256i s2, const __m256i s3 )
  {
    const __m256i sums = _mm256_hadd_epi32( _mm256_hadd_epi32( s0, s1 ), _mm256_hadd_epi32( s2, s3 ) );
    xStoreFourSads( _mm_add_epi32( _mm256_castsi256_si128( sums ), _mm256_extracti128_si256( sums, 1 ) ), rcDtParam, sads );
  };

  int blocksUntilCheck = 8 / rowsPerWidening;
  __m256i sum32_0 = _mm256_setzero_si256(), sum32_1 = sum32_0, sum32_2 = sum32_0, sum32_3 = sum32_0;
  for( int y = 0; y < rows; )
  {
    const int yEnd = std::min( rows, y + rowsPerWidening * rowStep );
    __m256i sum16_0 = _mm256_setzero_si256(), sum16_1 = sum16_0, sum16_2 = sum16_0, sum16_3 = sum16_0;
    for( ; y < yEnd; y += rowStep )
    {
      for( int x = 0; x < iWidth; x += 16 )
      {
        const __m256i orgRow = _mm256_loadu_si256( ( const __m256i* ) ( org + x ) );
        sum16_0 = xAddAbsDiff_AVX2( sum16_0, orgRow, ref0 + x );
        sum16_1 = xAddAbsDiff_AVX2( sum16_1, orgRow, ref1 + x );
        sum16_2 = xAddAbsDiff_AVX2( sum16_2, orgRow, ref2 + x );
        sum16_3 = xAddAbsDiff_AVX2( sum16_3, orgRow, ref3 + x );
      }
      org += orgStride;
      ref0 += refStride;
      ref1 += refStride;
      ref2 += refStride;
      ref3 += refStride;
    }
    sum32_0 = _mm256_add_epi32( sum32_0, _mm256_madd_epi16( sum16_0, one ) );
    sum32_1 = _mm256_add_epi32( sum32_1, _mm256_madd_epi16( sum16_1, one ) );
    sum32_2 = _mm256_add_epi32( sum32_2, _mm256_madd_epi16( sum16_2, one ) );
    sum32_3 = _mm256_add_epi32( sum32_3, _mm256_madd_epi16( sum16_3, one ) );

    if( iWidth >= 64 && --blocksUntilCheck == 0 && y < rows )
    {
      blocksUntilCheck = 8 / rowsPerWidening;
      storeSums( sum32_0, sum32_1, sum32_2, sum32_3 );
      if( std::min( std::min( sads[0], sads[1] ), std::min( sads[2], sads[3] ) ) > rcDtParam.maximumDistortionForEarlyExit )
      {
        return;   // all four lose whatever the remaining rows add, and the caller only compares
      }
    }
  }
  storeSums( sum32_0, sum32_1, sum32_2, sum32_3 );
}

static void xGetFourSads8_AVX2( const DistParam& rcDtParam, const Pel* const* refs, Distortion* sads )
{
  const short* org       = ( const short* ) rcDtParam.org.buf;
  const short* ref0 = refs[0], *ref1 = refs[1], *ref2 = refs[2], *ref3 = refs[3];
  const int    rows      = rcDtParam.org.height;
  const int    rowStep   = 1 << rcDtParam.subShift;
  const int    orgStride = rcDtParam.org.stride * rowStep;
  const int    refStride = rcDtParam.cur.stride * rowStep;
  const __m128i one = _mm_set1_epi16( 1 );

  __m128i sum32_0 = _mm_setzero_si128(), sum32_1 = sum32_0, sum32_2 = sum32_0, sum32_3 = sum32_0;
  for( int y = 0; y < rows; )
  {
    const int yEnd = std::min( rows, y + 16 * rowStep );
    __m128i sum16_0 = _mm_setzero_si128(), sum16_1 = sum16_0, sum16_2 = sum16_0, sum16_3 = sum16_0;
    for( ; y < yEnd; y += rowStep )
    {
      const __m128i orgRow = _mm_loadu_si128( ( const __m128i* ) org );
      sum16_0 = xAddAbsDiff_SSE( sum16_0, orgRow, ref0 );
      sum16_1 = xAddAbsDiff_SSE( sum16_1, orgRow, ref1 );
      sum16_2 = xAddAbsDiff_SSE( sum16_2, orgRow, ref2 );
      sum16_3 = xAddAbsDiff_SSE( sum16_3, orgRow, ref3 );
      org += orgStride;
      ref0 += refStride;
      ref1 += refStride;
      ref2 += refStride;
      ref3 += refStride;
    }
    sum32_0 = _mm_add_epi32( sum32_0, _mm_madd_epi16( sum16_0, one ) );
    sum32_1 = _mm_add_epi32( sum32_1, _mm_madd_epi16( sum16_1, one ) );
    sum32_2 = _mm_add_epi32( sum32_2, _mm_madd_epi16( sum16_2, one ) );
    sum32_3 = _mm_add_epi32( sum32_3, _mm_madd_epi16( sum16_3, one ) );
  }
  xStoreFourSads( _mm_hadd_epi32( _mm_hadd_epi32( sum32_0, sum32_1 ), _mm_hadd_epi32( sum32_2, sum32_3 ) ), rcDtParam, sads );
}
#endif // USE_AVX2

static uint32_t xCalcHAD4x4_SSE( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur )
{
  __m128i r0 = ( sizeof( Torg ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&piOrg[0] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piOrg[0] ), _mm_setzero_si128() ) );
  __m128i r1 = ( sizeof( Torg ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&piOrg[iStrideOrg] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piOrg[iStrideOrg] ), _mm_setzero_si128() ) );
  __m128i r2 = ( sizeof( Torg ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&piOrg[2 * iStrideOrg] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piOrg[2 * iStrideOrg] ), _mm_setzero_si128() ) );
  __m128i r3 = ( sizeof( Torg ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&piOrg[3 * iStrideOrg] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piOrg[3 * iStrideOrg] ), _mm_setzero_si128() ) );
  __m128i r4 = ( sizeof( Tcur ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&piCur[0] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piCur[0] ), _mm_setzero_si128() ) );
  __m128i r5 = ( sizeof( Tcur ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&piCur[iStrideCur] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piCur[iStrideCur] ), _mm_setzero_si128() ) );
  __m128i r6 = ( sizeof( Tcur ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&piCur[2 * iStrideCur] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piCur[2 * iStrideCur] ), _mm_setzero_si128() ) );
  __m128i r7 = ( sizeof( Tcur ) > 1 ) ? ( _vv_loadl_epi64( ( const __m128i* )&piCur[3 * iStrideCur] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piCur[3 * iStrideCur] ), _mm_setzero_si128() ) );

  r0 = _mm_sub_epi16( r0, r4 );
  r1 = _mm_sub_epi16( r1, r5 );
  r2 = _mm_sub_epi16( r2, r6 );
  r3 = _mm_sub_epi16( r3, r7 );

  // first stage
  r4 = r0;
  r5 = r1;

  r0 = _mm_add_epi16( r0, r3 );
  r1 = _mm_add_epi16( r1, r2 );

  r4 = _mm_sub_epi16( r4, r3 );
  r5 = _mm_sub_epi16( r5, r2 );

  r2 = r0;
  r3 = r4;

  r0 = _mm_add_epi16( r0, r1 );
  r2 = _mm_sub_epi16( r2, r1 );
  r3 = _mm_sub_epi16( r3, r5 );
  r5 = _mm_add_epi16( r5, r4 );

  // shuffle - flip matrix for vertical transform
  r0 = _mm_unpacklo_epi16( r0, r5 );
  r2 = _mm_unpacklo_epi16( r2, r3 );

  r3 = r0;
  r0 = _mm_unpacklo_epi32( r0, r2 );
  r3 = _mm_unpackhi_epi32( r3, r2 );

  r1 = r0;
  r2 = r3;
  r1 = _mm_srli_si128( r1, 8 );
  r3 = _mm_srli_si128( r3, 8 );

  // second stage
  r4 = r0;
  r5 = r1;

  r0 = _mm_add_epi16( r0, r3 );
  r1 = _mm_add_epi16( r1, r2 );

  r4 = _mm_sub_epi16( r4, r3 );
  r5 = _mm_sub_epi16( r5, r2 );

  r2 = r0;
  r3 = r4;

  r0 = _mm_add_epi16( r0, r1 );
  r2 = _mm_sub_epi16( r2, r1 );
  r3 = _mm_sub_epi16( r3, r5 );
  r5 = _mm_add_epi16( r5, r4 );

  // abs
  __m128i Sum = _mm_abs_epi16( r0 );
  uint32_t absDc = _mm_cvtsi128_si32( Sum ) & 0x0000ffff;
  Sum = _mm_add_epi16( Sum, _mm_abs_epi16( r2 ) );
  Sum = _mm_add_epi16( Sum, _mm_abs_epi16( r3 ) );
  Sum = _mm_add_epi16( Sum, _mm_abs_epi16( r5 ) );

  __m128i iZero = _mm_set1_epi16( 0 );
  Sum = _mm_unpacklo_epi16( Sum, iZero );
  Sum = _mm_hadd_epi32( Sum, Sum );
  Sum = _mm_hadd_epi32( Sum, Sum );

  uint32_t sad = _mm_cvtsi128_si32( Sum );
  
  sad -= absDc;
  sad += absDc >> 2;
  sad = ( ( sad + 1 ) >> 1 );

  return sad;
}

//working up to 12-bit
static uint32_t xCalcHAD8x8_SSE( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  __m128i m1[2][8], m2[2][8];

  CHECK( iBitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  for( int k = 0; k < 8; k++ )
  {
    __m128i r0 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadu_si128( ( __m128i* )piOrg ) ) : ( _mm_unpacklo_epi8( _vv_loadl_epi64( ( const __m128i* )piOrg ), _mm_setzero_si128() ) );
    __m128i r1 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadu_si128( ( __m128i* )piCur ) ) : ( _mm_unpacklo_epi8( _vv_loadl_epi64( ( const __m128i* )piCur ), _mm_setzero_si128() ) );
    m2[0][k] = _mm_sub_epi16( r0, r1 ); // 11bit
    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  m1[0][0] = _mm_add_epi16( m2[0][0], m2[0][4] );
  m1[0][1] = _mm_add_epi16( m2[0][1], m2[0][5] );
  m1[0][2] = _mm_add_epi16( m2[0][2], m2[0][6] );
  m1[0][3] = _mm_add_epi16( m2[0][3], m2[0][7] );
  m1[0][4] = _mm_sub_epi16( m2[0][0], m2[0][4] );
  m1[0][5] = _mm_sub_epi16( m2[0][1], m2[0][5] );
  m1[0][6] = _mm_sub_epi16( m2[0][2], m2[0][6] );
  m1[0][7] = _mm_sub_epi16( m2[0][3], m2[0][7] ); // 12 bit

  m2[0][0] = _mm_add_epi16( m1[0][0], m1[0][2] );
  m2[0][1] = _mm_add_epi16( m1[0][1], m1[0][3] );
  m2[0][2] = _mm_sub_epi16( m1[0][0], m1[0][2] );
  m2[0][3] = _mm_sub_epi16( m1[0][1], m1[0][3] );
  m2[0][4] = _mm_add_epi16( m1[0][4], m1[0][6] );
  m2[0][5] = _mm_add_epi16( m1[0][5], m1[0][7] );
  m2[0][6] = _mm_sub_epi16( m1[0][4], m1[0][6] );
  m2[0][7] = _mm_sub_epi16( m1[0][5], m1[0][7] ); // 13 bit

  m1[0][0] = _mm_add_epi16( m2[0][0], m2[0][1] );
  m1[0][1] = _mm_sub_epi16( m2[0][0], m2[0][1] );
  m1[0][2] = _mm_add_epi16( m2[0][2], m2[0][3] );
  m1[0][3] = _mm_sub_epi16( m2[0][2], m2[0][3] );
  m1[0][4] = _mm_add_epi16( m2[0][4], m2[0][5] );
  m1[0][5] = _mm_sub_epi16( m2[0][4], m2[0][5] );
  m1[0][6] = _mm_add_epi16( m2[0][6], m2[0][7] );
  m1[0][7] = _mm_sub_epi16( m2[0][6], m2[0][7] ); // 14 bit

  m2[0][0] = _mm_unpacklo_epi16( m1[0][0], m1[0][1] );
  m2[0][1] = _mm_unpacklo_epi16( m1[0][2], m1[0][3] );
  m2[0][2] = _mm_unpackhi_epi16( m1[0][0], m1[0][1] );
  m2[0][3] = _mm_unpackhi_epi16( m1[0][2], m1[0][3] );
  m2[0][4] = _mm_unpacklo_epi16( m1[0][4], m1[0][5] );
  m2[0][5] = _mm_unpacklo_epi16( m1[0][6], m1[0][7] );
  m2[0][6] = _mm_unpackhi_epi16( m1[0][4], m1[0][5] );
  m2[0][7] = _mm_unpackhi_epi16( m1[0][6], m1[0][7] );

  m1[0][0] = _mm_unpacklo_epi32( m2[0][0], m2[0][1] );
  m1[0][1] = _mm_unpackhi_epi32( m2[0][0], m2[0][1] );
  m1[0][2] = _mm_unpacklo_epi32( m2[0][2], m2[0][3] );
  m1[0][3] = _mm_unpackhi_epi32( m2[0][2], m2[0][3] );
  m1[0][4] = _mm_unpacklo_epi32( m2[0][4], m2[0][5] );
  m1[0][5] = _mm_unpackhi_epi32( m2[0][4], m2[0][5] );
  m1[0][6] = _mm_unpacklo_epi32( m2[0][6], m2[0][7] );
  m1[0][7] = _mm_unpackhi_epi32( m2[0][6], m2[0][7] );
  
  m1[1][0] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][0], 8 ) );
  m1[0][0] = _mm_cvtepi16_epi32(                 m1[0][0]      );
  m1[1][1] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][1], 8 ) );
  m1[0][1] = _mm_cvtepi16_epi32(                 m1[0][1]      );
  m1[1][2] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][2], 8 ) );
  m1[0][2] = _mm_cvtepi16_epi32(                 m1[0][2]      );
  m1[1][3] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][3], 8 ) );
  m1[0][3] = _mm_cvtepi16_epi32(                 m1[0][3]      );
  m1[1][4] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][4], 8 ) );
  m1[0][4] = _mm_cvtepi16_epi32(                 m1[0][4]      );
  m1[1][5] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][5], 8 ) );
  m1[0][5] = _mm_cvtepi16_epi32(                 m1[0][5]      );
  m1[1][6] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][6], 8 ) );
  m1[0][6] = _mm_cvtepi16_epi32(                 m1[0][6]      );
  m1[1][7] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][7], 8 ) );
  m1[0][7] = _mm_cvtepi16_epi32(                 m1[0][7]      );

  for( int i = 0; i < 8; i++ )
  {
    int ii = i % 4;
    int ij = i >> 2;

    m2[0][i] = m1[ij][ii    ];
    m2[1][i] = m1[ij][ii + 4];
  }

  for( int i = 0; i < 2; i++ )
  {
    m1[i][0] = _mm_add_epi32( m2[i][0], m2[i][4] );
    m1[i][1] = _mm_add_epi32( m2[i][1], m2[i][5] );
    m1[i][2] = _mm_add_epi32( m2[i][2], m2[i][6] );
    m1[i][3] = _mm_add_epi32( m2[i][3], m2[i][7] );
    m1[i][4] = _mm_sub_epi32( m2[i][0], m2[i][4] );
    m1[i][5] = _mm_sub_epi32( m2[i][1], m2[i][5] );
    m1[i][6] = _mm_sub_epi32( m2[i][2], m2[i][6] );
    m1[i][7] = _mm_sub_epi32( m2[i][3], m2[i][7] );

    m2[i][0] = _mm_add_epi32( m1[i][0], m1[i][2] );
    m2[i][1] = _mm_add_epi32( m1[i][1], m1[i][3] );
    m2[i][2] = _mm_sub_epi32( m1[i][0], m1[i][2] );
    m2[i][3] = _mm_sub_epi32( m1[i][1], m1[i][3] );
    m2[i][4] = _mm_add_epi32( m1[i][4], m1[i][6] );
    m2[i][5] = _mm_add_epi32( m1[i][5], m1[i][7] );
    m2[i][6] = _mm_sub_epi32( m1[i][4], m1[i][6] );
    m2[i][7] = _mm_sub_epi32( m1[i][5], m1[i][7] );

    m1[i][0] = _mm_abs_epi32( _mm_add_epi32( m2[i][0], m2[i][1] ) );
    m1[i][1] = _mm_abs_epi32( _mm_sub_epi32( m2[i][0], m2[i][1] ) );
    m1[i][2] = _mm_abs_epi32( _mm_add_epi32( m2[i][2], m2[i][3] ) );
    m1[i][3] = _mm_abs_epi32( _mm_sub_epi32( m2[i][2], m2[i][3] ) );
    m1[i][4] = _mm_abs_epi32( _mm_add_epi32( m2[i][4], m2[i][5] ) );
    m1[i][5] = _mm_abs_epi32( _mm_sub_epi32( m2[i][4], m2[i][5] ) );
    m1[i][6] = _mm_abs_epi32( _mm_add_epi32( m2[i][6], m2[i][7] ) );
    m1[i][7] = _mm_abs_epi32( _mm_sub_epi32( m2[i][6], m2[i][7] ) );
  }
  m2[0][0] = m1[0][0];
  for( int i = 0; i < 8; i++ )
  {
    m1[0][i] = _mm_add_epi32( m1[0][i], m1[1][i] );
  }

  m1[0][0] = _mm_add_epi32( m1[0][0], m1[0][1] );
  m1[0][2] = _mm_add_epi32( m1[0][2], m1[0][3] );
  m1[0][4] = _mm_add_epi32( m1[0][4], m1[0][5] );
  m1[0][6] = _mm_add_epi32( m1[0][6], m1[0][7] );

  m1[0][0] = _mm_add_epi32( m1[0][0], m1[0][2] );
  m1[0][4] = _mm_add_epi32( m1[0][4], m1[0][6] );
  __m128i iSum = _mm_add_epi32( m1[0][0], m1[0][4] );

  iSum = _mm_hadd_epi32( iSum, iSum );
  iSum = _mm_hadd_epi32( iSum, iSum );

  uint32_t sad = _mm_cvtsi128_si32( iSum );
  uint32_t absDc = _mm_cvtsi128_si32( m2[0][0] );
  sad -= absDc;
  sad += absDc >> 2;
  sad = ( ( sad + 2 ) >> 2 );

  return sad;
}


//working up to 12-bit
static uint32_t xCalcHAD16x16_fast_SSE( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  __m128i m1[2][8], m2[2][8];

  CHECK( iBitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  for( int k = 0; k < 8; k++ )
  {
    __m128i r0 = _mm_loadu_si128( ( __m128i* )piOrg );
    __m128i r1 = _mm_loadu_si128( ( __m128i* )piCur );
    __m128i r2 = _mm_loadu_si128( ( __m128i* )( piOrg + iStrideOrg ) );
    __m128i r3 = _mm_loadu_si128( ( __m128i* )( piCur + iStrideCur ) );

    r0 = _mm_add_epi16( r0, r2 );
    r1 = _mm_add_epi16( r1, r3 );

    r2 = _mm_loadu_si128( ( __m128i* )( piOrg + 8 ) );
    r3 = _mm_loadu_si128( ( __m128i* )( piCur + 8 ) );
    __m128i r4 = _mm_loadu_si128( ( __m128i* )( piOrg + iStrideOrg + 8 ) );
    __m128i r5 = _mm_loadu_si128( ( __m128i* )( piCur + iStrideCur + 8 ) );

    r2 = _mm_add_epi16( r2, r4 );
    r3 = _mm_add_epi16( r3, r5 );

    r0 = _mm_hadd_epi16( r0, r2 );
    r1 = _mm_hadd_epi16( r1, r3 );

    r0 = _mm_add_epi16( r0, _mm_set1_epi16( 2 ) );
    r1 = _mm_add_epi16( r1, _mm_set1_epi16( 2 ) );
    r0 = _mm_srai_epi16( r0, 2 );
    r1 = _mm_srai_epi16( r1, 2 );

    m2[0][k] = _mm_sub_epi16( r0, r1 ); // 11bit
    //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
    //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
    piCur += iStrideCur * 2;
    piOrg += iStrideOrg * 2;
  }

  //horizontal
  m1[0][0] = _mm_add_epi16( m2[0][0], m2[0][4] );
  m1[0][1] = _mm_add_epi16( m2[0][1], m2[0][5] );
  m1[0][2] = _mm_add_epi16( m2[0][2], m2[0][6] );
  m1[0][3] = _mm_add_epi16( m2[0][3], m2[0][7] );
  m1[0][4] = _mm_sub_epi16( m2[0][0], m2[0][4] );
  m1[0][5] = _mm_sub_epi16( m2[0][1], m2[0][5] );
  m1[0][6] = _mm_sub_epi16( m2[0][2], m2[0][6] );
  m1[0][7] = _mm_sub_epi16( m2[0][3], m2[0][7] ); // 12 bit

  m2[0][0] = _mm_add_epi16( m1[0][0], m1[0][2] );
  m2[0][1] = _mm_add_epi16( m1[0][1], m1[0][3] );
  m2[0][2] = _mm_sub_epi16( m1[0][0], m1[0][2] );
  m2[0][3] = _mm_sub_epi16( m1[0][1], m1[0][3] );
  m2[0][4] = _mm_add_epi16( m1[0][4], m1[0][6] );
  m2[0][5] = _mm_add_epi16( m1[0][5], m1[0][7] );
  m2[0][6] = _mm_sub_epi16( m1[0][4], m1[0][6] );
  m2[0][7] = _mm_sub_epi16( m1[0][5], m1[0][7] ); // 13 bit

  m1[0][0] = _mm_add_epi16( m2[0][0], m2[0][1] );
  m1[0][1] = _mm_sub_epi16( m2[0][0], m2[0][1] );
  m1[0][2] = _mm_add_epi16( m2[0][2], m2[0][3] );
  m1[0][3] = _mm_sub_epi16( m2[0][2], m2[0][3] );
  m1[0][4] = _mm_add_epi16( m2[0][4], m2[0][5] );
  m1[0][5] = _mm_sub_epi16( m2[0][4], m2[0][5] );
  m1[0][6] = _mm_add_epi16( m2[0][6], m2[0][7] );
  m1[0][7] = _mm_sub_epi16( m2[0][6], m2[0][7] ); // 14 bit

  m2[0][0] = _mm_unpacklo_epi16( m1[0][0], m1[0][1] );
  m2[0][1] = _mm_unpacklo_epi16( m1[0][2], m1[0][3] );
  m2[0][2] = _mm_unpackhi_epi16( m1[0][0], m1[0][1] );
  m2[0][3] = _mm_unpackhi_epi16( m1[0][2], m1[0][3] );
  m2[0][4] = _mm_unpacklo_epi16( m1[0][4], m1[0][5] );
  m2[0][5] = _mm_unpacklo_epi16( m1[0][6], m1[0][7] );
  m2[0][6] = _mm_unpackhi_epi16( m1[0][4], m1[0][5] );
  m2[0][7] = _mm_unpackhi_epi16( m1[0][6], m1[0][7] );

  m1[0][0] = _mm_unpacklo_epi32( m2[0][0], m2[0][1] );
  m1[0][1] = _mm_unpackhi_epi32( m2[0][0], m2[0][1] );
  m1[0][2] = _mm_unpacklo_epi32( m2[0][2], m2[0][3] );
  m1[0][3] = _mm_unpackhi_epi32( m2[0][2], m2[0][3] );
  m1[0][4] = _mm_unpacklo_epi32( m2[0][4], m2[0][5] );
  m1[0][5] = _mm_unpackhi_epi32( m2[0][4], m2[0][5] );
  m1[0][6] = _mm_unpacklo_epi32( m2[0][6], m2[0][7] );
  m1[0][7] = _mm_unpackhi_epi32( m2[0][6], m2[0][7] );
  
  m1[1][0] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][0], 8 ) );
  m1[0][0] = _mm_cvtepi16_epi32(                 m1[0][0]      );
  m1[1][1] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][1], 8 ) );
  m1[0][1] = _mm_cvtepi16_epi32(                 m1[0][1]      );
  m1[1][2] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][2], 8 ) );
  m1[0][2] = _mm_cvtepi16_epi32(                 m1[0][2]      );
  m1[1][3] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][3], 8 ) );
  m1[0][3] = _mm_cvtepi16_epi32(                 m1[0][3]      );
  m1[1][4] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][4], 8 ) );
  m1[0][4] = _mm_cvtepi16_epi32(                 m1[0][4]      );
  m1[1][5] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][5], 8 ) );
  m1[0][5] = _mm_cvtepi16_epi32(                 m1[0][5]      );
  m1[1][6] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][6], 8 ) );
  m1[0][6] = _mm_cvtepi16_epi32(                 m1[0][6]      );
  m1[1][7] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][7], 8 ) );
  m1[0][7] = _mm_cvtepi16_epi32(                 m1[0][7]      );

  for( int i = 0; i < 8; i++ )
  {
    int ii = i % 4;
    int ij = i >> 2;

    m2[0][i] = m1[ij][ii    ];
    m2[1][i] = m1[ij][ii + 4];
  }

  for( int i = 0; i < 2; i++ )
  {
    m1[i][0] = _mm_add_epi32( m2[i][0], m2[i][4] );
    m1[i][1] = _mm_add_epi32( m2[i][1], m2[i][5] );
    m1[i][2] = _mm_add_epi32( m2[i][2], m2[i][6] );
    m1[i][3] = _mm_add_epi32( m2[i][3], m2[i][7] );
    m1[i][4] = _mm_sub_epi32( m2[i][0], m2[i][4] );
    m1[i][5] = _mm_sub_epi32( m2[i][1], m2[i][5] );
    m1[i][6] = _mm_sub_epi32( m2[i][2], m2[i][6] );
    m1[i][7] = _mm_sub_epi32( m2[i][3], m2[i][7] );

    m2[i][0] = _mm_add_epi32( m1[i][0], m1[i][2] );
    m2[i][1] = _mm_add_epi32( m1[i][1], m1[i][3] );
    m2[i][2] = _mm_sub_epi32( m1[i][0], m1[i][2] );
    m2[i][3] = _mm_sub_epi32( m1[i][1], m1[i][3] );
    m2[i][4] = _mm_add_epi32( m1[i][4], m1[i][6] );
    m2[i][5] = _mm_add_epi32( m1[i][5], m1[i][7] );
    m2[i][6] = _mm_sub_epi32( m1[i][4], m1[i][6] );
    m2[i][7] = _mm_sub_epi32( m1[i][5], m1[i][7] );

    m1[i][0] = _mm_abs_epi32( _mm_add_epi32( m2[i][0], m2[i][1] ) );
    m1[i][1] = _mm_abs_epi32( _mm_sub_epi32( m2[i][0], m2[i][1] ) );
    m1[i][2] = _mm_abs_epi32( _mm_add_epi32( m2[i][2], m2[i][3] ) );
    m1[i][3] = _mm_abs_epi32( _mm_sub_epi32( m2[i][2], m2[i][3] ) );
    m1[i][4] = _mm_abs_epi32( _mm_add_epi32( m2[i][4], m2[i][5] ) );
    m1[i][5] = _mm_abs_epi32( _mm_sub_epi32( m2[i][4], m2[i][5] ) );
    m1[i][6] = _mm_abs_epi32( _mm_add_epi32( m2[i][6], m2[i][7] ) );
    m1[i][7] = _mm_abs_epi32( _mm_sub_epi32( m2[i][6], m2[i][7] ) );
  }
  m2[0][0] = m1[0][0];
  for( int i = 0; i < 8; i++ )
  {
    m1[0][i] = _mm_add_epi32( m1[0][i], m1[1][i] );
  }

  m1[0][0] = _mm_add_epi32( m1[0][0], m1[0][1] );
  m1[0][2] = _mm_add_epi32( m1[0][2], m1[0][3] );
  m1[0][4] = _mm_add_epi32( m1[0][4], m1[0][5] );
  m1[0][6] = _mm_add_epi32( m1[0][6], m1[0][7] );

  m1[0][0] = _mm_add_epi32( m1[0][0], m1[0][2] );
  m1[0][4] = _mm_add_epi32( m1[0][4], m1[0][6] );
  __m128i iSum = _mm_add_epi32( m1[0][0], m1[0][4] );

  iSum = _mm_hadd_epi32( iSum, iSum );
  iSum = _mm_hadd_epi32( iSum, iSum );

  uint32_t sad = _mm_cvtsi128_si32( iSum );
  uint32_t absDc = _mm_cvtsi128_si32( m2[0][0] );
  sad -= absDc;
  sad += absDc >> 2;
  sad = ( ( sad + 2 ) >> 2 );

  return ( sad << 2 );
}


//working up to 12-bit
static uint32_t xCalcHAD16x8_SSE( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  __m128i m1[16][2][2], m2[16][2][2];
  __m128i iSum = _mm_setzero_si128();

  for( int l = 0; l < 2; l++ )
  {
    const Torg *piOrgPtr = piOrg + l*8;
    const Tcur *piCurPtr = piCur + l*8;
    for( int k = 0; k < 8; k++ )
    {
      __m128i r0 = _mm_loadu_si128( (__m128i*) piOrgPtr );
      __m128i r1 = _mm_loadu_si128( (__m128i*) piCurPtr );
      m2[k][l][0] = _mm_sub_epi16( r0, r1 );
      m2[k][l][1] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[k][l][0], 8 ) );
      m2[k][l][0] = _mm_cvtepi16_epi32( m2[k][l][0] );
      piCurPtr += iStrideCur;
      piOrgPtr += iStrideOrg;
    }

    for( int i = 0; i < 2; i++ )
    {
      //vertical
      m1[0][l][i] = _mm_add_epi32( m2[0][l][i], m2[4][l][i] );
      m1[1][l][i] = _mm_add_epi32( m2[1][l][i], m2[5][l][i] );
      m1[2][l][i] = _mm_add_epi32( m2[2][l][i], m2[6][l][i] );
      m1[3][l][i] = _mm_add_epi32( m2[3][l][i], m2[7][l][i] );
      m1[4][l][i] = _mm_sub_epi32( m2[0][l][i], m2[4][l][i] );
      m1[5][l][i] = _mm_sub_epi32( m2[1][l][i], m2[5][l][i] );
      m1[6][l][i] = _mm_sub_epi32( m2[2][l][i], m2[6][l][i] );
      m1[7][l][i] = _mm_sub_epi32( m2[3][l][i], m2[7][l][i] );

      m2[0][l][i] = _mm_add_epi32( m1[0][l][i], m1[2][l][i] );
      m2[1][l][i] = _mm_add_epi32( m1[1][l][i], m1[3][l][i] );
      m2[2][l][i] = _mm_sub_epi32( m1[0][l][i], m1[2][l][i] );
      m2[3][l][i] = _mm_sub_epi32( m1[1][l][i], m1[3][l][i] );
      m2[4][l][i] = _mm_add_epi32( m1[4][l][i], m1[6][l][i] );
      m2[5][l][i] = _mm_add_epi32( m1[5][l][i], m1[7][l][i] );
      m2[6][l][i] = _mm_sub_epi32( m1[4][l][i], m1[6][l][i] );
      m2[7][l][i] = _mm_sub_epi32( m1[5][l][i], m1[7][l][i] );

      m1[0][l][i] = _mm_add_epi32( m2[0][l][i], m2[1][l][i] );
      m1[1][l][i] = _mm_sub_epi32( m2[0][l][i], m2[1][l][i] );
      m1[2][l][i] = _mm_add_epi32( m2[2][l][i], m2[3][l][i] );
      m1[3][l][i] = _mm_sub_epi32( m2[2][l][i], m2[3][l][i] );
      m1[4][l][i] = _mm_add_epi32( m2[4][l][i], m2[5][l][i] );
      m1[5][l][i] = _mm_sub_epi32( m2[4][l][i], m2[5][l][i] );
      m1[6][l][i] = _mm_add_epi32( m2[6][l][i], m2[7][l][i] );
      m1[7][l][i] = _mm_sub_epi32( m2[6][l][i], m2[7][l][i] );
    }
  }

  // 4 x 8x4 blocks
  // 0 1
  // 2 3
  uint32_t absDc = 0;

  // transpose and do horizontal in two steps
  for( int l = 0; l < 2; l++ )
  {
    int off = l * 4;

    __m128i n1[16];
    __m128i n2[16];

    m2[0][0][0] = _mm_unpacklo_epi32( m1[0 + off][0][0], m1[1 + off][0][0] );
    m2[1][0][0] = _mm_unpacklo_epi32( m1[2 + off][0][0], m1[3 + off][0][0] );
    m2[2][0][0] = _mm_unpackhi_epi32( m1[0 + off][0][0], m1[1 + off][0][0] );
    m2[3][0][0] = _mm_unpackhi_epi32( m1[2 + off][0][0], m1[3 + off][0][0] );

    m2[0][0][1] = _mm_unpacklo_epi32( m1[0 + off][0][1], m1[1 + off][0][1] );
    m2[1][0][1] = _mm_unpacklo_epi32( m1[2 + off][0][1], m1[3 + off][0][1] );
    m2[2][0][1] = _mm_unpackhi_epi32( m1[0 + off][0][1], m1[1 + off][0][1] );
    m2[3][0][1] = _mm_unpackhi_epi32( m1[2 + off][0][1], m1[3 + off][0][1] );

    n1[0]       = _mm_unpacklo_epi64( m2[0][0][0], m2[1][0][0] );
    n1[1]       = _mm_unpackhi_epi64( m2[0][0][0], m2[1][0][0] );
    n1[2]       = _mm_unpacklo_epi64( m2[2][0][0], m2[3][0][0] );
    n1[3]       = _mm_unpackhi_epi64( m2[2][0][0], m2[3][0][0] );
    n1[4]       = _mm_unpacklo_epi64( m2[0][0][1], m2[1][0][1] );
    n1[5]       = _mm_unpackhi_epi64( m2[0][0][1], m2[1][0][1] );
    n1[6]       = _mm_unpacklo_epi64( m2[2][0][1], m2[3][0][1] );
    n1[7]       = _mm_unpackhi_epi64( m2[2][0][1], m2[3][0][1] );

    // transpose 8x4 -> 4x8, block 1(3)
    m2[8+0][0][0] = _mm_unpacklo_epi32( m1[0 + off][1][0], m1[1 + off][1][0] );
    m2[8+1][0][0] = _mm_unpacklo_epi32( m1[2 + off][1][0], m1[3 + off][1][0] );
    m2[8+2][0][0] = _mm_unpackhi_epi32( m1[0 + off][1][0], m1[1 + off][1][0] );
    m2[8+3][0][0] = _mm_unpackhi_epi32( m1[2 + off][1][0], m1[3 + off][1][0] );

    m2[8+0][0][1] = _mm_unpacklo_epi32( m1[0 + off][1][1], m1[1 + off][1][1] );
    m2[8+1][0][1] = _mm_unpacklo_epi32( m1[2 + off][1][1], m1[3 + off][1][1] );
    m2[8+2][0][1] = _mm_unpackhi_epi32( m1[0 + off][1][1], m1[1 + off][1][1] );
    m2[8+3][0][1] = _mm_unpackhi_epi32( m1[2 + off][1][1], m1[3 + off][1][1] );

    n1[8+0]       = _mm_unpacklo_epi64( m2[8+0][0][0], m2[8+1][0][0] );
    n1[8+1]       = _mm_unpackhi_epi64( m2[8+0][0][0], m2[8+1][0][0] );
    n1[8+2]       = _mm_unpacklo_epi64( m2[8+2][0][0], m2[8+3][0][0] );
    n1[8+3]       = _mm_unpackhi_epi64( m2[8+2][0][0], m2[8+3][0][0] );
    n1[8+4]       = _mm_unpacklo_epi64( m2[8+0][0][1], m2[8+1][0][1] );
    n1[8+5]       = _mm_unpackhi_epi64( m2[8+0][0][1], m2[8+1][0][1] );
    n1[8+6]       = _mm_unpacklo_epi64( m2[8+2][0][1], m2[8+3][0][1] );
    n1[8+7]       = _mm_unpackhi_epi64( m2[8+2][0][1], m2[8+3][0][1] );

    n2[0] = _mm_add_epi32( n1[0], n1[8] );
    n2[1] = _mm_add_epi32( n1[1], n1[9] );
    n2[2] = _mm_add_epi32( n1[2], n1[10] );
    n2[3] = _mm_add_epi32( n1[3], n1[11] );
    n2[4] = _mm_add_epi32( n1[4], n1[12] );
    n2[5] = _mm_add_epi32( n1[5], n1[13] );
    n2[6] = _mm_add_epi32( n1[6], n1[14] );
    n2[7] = _mm_add_epi32( n1[7], n1[15] );
    n2[8] = _mm_sub_epi32( n1[0], n1[8] );
    n2[9] = _mm_sub_epi32( n1[1], n1[9] );
    n2[10] = _mm_sub_epi32( n1[2], n1[10] );
    n2[11] = _mm_sub_epi32( n1[3], n1[11] );
    n2[12] = _mm_sub_epi32( n1[4], n1[12] );
    n2[13] = _mm_sub_epi32( n1[5], n1[13] );
    n2[14] = _mm_sub_epi32( n1[6], n1[14] );
    n2[15] = _mm_sub_epi32( n1[7], n1[15] );

    n1[0] = _mm_add_epi32( n2[0], n2[4] );
    n1[1] = _mm_add_epi32( n2[1], n2[5] );
    n1[2] = _mm_add_epi32( n2[2], n2[6] );
    n1[3] = _mm_add_epi32( n2[3], n2[7] );
    n1[4] = _mm_sub_epi32( n2[0], n2[4] );
    n1[5] = _mm_sub_epi32( n2[1], n2[5] );
    n1[6] = _mm_sub_epi32( n2[2], n2[6] );
    n1[7] = _mm_sub_epi32( n2[3], n2[7] );
    n1[8] = _mm_add_epi32( n2[8], n2[12] );
    n1[9] = _mm_add_epi32( n2[9], n2[13] );
    n1[10] = _mm_add_epi32( n2[10], n2[14] );
    n1[11] = _mm_add_epi32( n2[11], n2[15] );
    n1[12] = _mm_sub_epi32( n2[8], n2[12] );
    n1[13] = _mm_sub_epi32( n2[9], n2[13] );
    n1[14] = _mm_sub_epi32( n2[10], n2[14] );
    n1[15] = _mm_sub_epi32( n2[11], n2[15] );

    n2[0] = _mm_add_epi32( n1[0], n1[2] );
    n2[1] = _mm_add_epi32( n1[1], n1[3] );
    n2[2] = _mm_sub_epi32( n1[0], n1[2] );
    n2[3] = _mm_sub_epi32( n1[1], n1[3] );
    n2[4] = _mm_add_epi32( n1[4], n1[6] );
    n2[5] = _mm_add_epi32( n1[5], n1[7] );
    n2[6] = _mm_sub_epi32( n1[4], n1[6] );
    n2[7] = _mm_sub_epi32( n1[5], n1[7] );
    n2[8] = _mm_add_epi32( n1[8], n1[10] );
    n2[9] = _mm_add_epi32( n1[9], n1[11] );
    n2[10] = _mm_sub_epi32( n1[8], n1[10] );
    n2[11] = _mm_sub_epi32( n1[9], n1[11] );
    n2[12] = _mm_add_epi32( n1[12], n1[14] );
    n2[13] = _mm_add_epi32( n1[13], n1[15] );
    n2[14] = _mm_sub_epi32( n1[12], n1[14] );
    n2[15] = _mm_sub_epi32( n1[13], n1[15] );

    n1[0] = _mm_abs_epi32( _mm_add_epi32( n2[0], n2[1] ) );
    n1[1] = _mm_abs_epi32( _mm_sub_epi32( n2[0], n2[1] ) );
    n1[2] = _mm_abs_epi32( _mm_add_epi32( n2[2], n2[3] ) );
    n1[3] = _mm_abs_epi32( _mm_sub_epi32( n2[2], n2[3] ) );
    n1[4] = _mm_abs_epi32( _mm_add_epi32( n2[4], n2[5] ) );
    n1[5] = _mm_abs_epi32( _mm_sub_epi32( n2[4], n2[5] ) );
    n1[6] = _mm_abs_epi32( _mm_add_epi32( n2[6], n2[7] ) );
    n1[7] = _mm_abs_epi32( _mm_sub_epi32( n2[6], n2[7] ) );
    n1[8] = _mm_abs_epi32( _mm_add_epi32( n2[8], n2[9] ) );
    n1[9] = _mm_abs_epi32( _mm_sub_epi32( n2[8], n2[9] ) );
    n1[10] = _mm_abs_epi32( _mm_add_epi32( n2[10], n2[11] ) );
    n1[11] = _mm_abs_epi32( _mm_sub_epi32( n2[10], n2[11] ) );
    n1[12] = _mm_abs_epi32( _mm_add_epi32( n2[12], n2[13] ) );
    n1[13] = _mm_abs_epi32( _mm_sub_epi32( n2[12], n2[13] ) );
    n1[14] = _mm_abs_epi32( _mm_add_epi32( n2[14], n2[15] ) );
    n1[15] = _mm_abs_epi32( _mm_sub_epi32( n2[14], n2[15] ) );
    
    if (l == 0)
      absDc = _mm_cvtsi128_si32( n1[0] );

    // sum up
    n1[0] = _mm_add_epi32( n1[0], n1[1] );
    n1[2] = _mm_add_epi32( n1[2], n1[3] );
    n1[4] = _mm_add_epi32( n1[4], n1[5] );
    n1[6] = _mm_add_epi32( n1[6], n1[7] );
    n1[8] = _mm_add_epi32( n1[8], n1[9] );
    n1[10] = _mm_add_epi32( n1[10], n1[11] );
    n1[12] = _mm_add_epi32( n1[12], n1[13] );
    n1[14] = _mm_add_epi32( n1[14], n1[15] );

    n1[0] = _mm_add_epi32( n1[0], n1[2] );
    n1[4] = _mm_add_epi32( n1[4], n1[6] );
    n1[8] = _mm_add_epi32( n1[8], n1[10] );
    n1[12] = _mm_add_epi32( n1[12], n1[14] );

    n1[0] = _mm_add_epi32( n1[0], n1[4] );
    n1[8] = _mm_add_epi32( n1[8], n1[12] );

    n1[0] = _mm_add_epi32( n1[0], n1[8] );
    iSum = _mm_add_epi32( iSum, n1[0] );
  }

  iSum = _mm_hadd_epi32( iSum, iSum );
  iSum = _mm_hadd_epi32( iSum, iSum );

  uint32_t sad = _mm_cvtsi128_si32( iSum );
  sad -= absDc;
  sad += absDc >> 2;
  sad = (uint32_t)(sad / sqrt(16.0 * 8) * 2);

  return sad;
}


//working up to 12-bit
static uint32_t xCalcHAD8x16_SSE( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  __m128i m1[2][16], m2[2][16];
  __m128i iSum = _mm_setzero_si128();

  for( int k = 0; k < 16; k++ )
  {
    __m128i r0 =_mm_loadu_si128( (__m128i*)piOrg );
    __m128i r1 =_mm_loadu_si128( (__m128i*)piCur );
    m1[0][k] = _mm_sub_epi16( r0, r1 );
    m1[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m1[0][k], 8 ) );
    m1[0][k] = _mm_cvtepi16_epi32( m1[0][k] );
    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  for( int i = 0; i < 2; i++ )
  {
    // vertical
    m2[i][ 0] = _mm_add_epi32( m1[i][ 0], m1[i][ 8] );
    m2[i][ 1] = _mm_add_epi32( m1[i][ 1], m1[i][ 9] );
    m2[i][ 2] = _mm_add_epi32( m1[i][ 2], m1[i][10] );
    m2[i][ 3] = _mm_add_epi32( m1[i][ 3], m1[i][11] );
    m2[i][ 4] = _mm_add_epi32( m1[i][ 4], m1[i][12] );
    m2[i][ 5] = _mm_add_epi32( m1[i][ 5], m1[i][13] );
    m2[i][ 6] = _mm_add_epi32( m1[i][ 6], m1[i][14] );
    m2[i][ 7] = _mm_add_epi32( m1[i][ 7], m1[i][15] );
    m2[i][ 8] = _mm_sub_epi32( m1[i][ 0], m1[i][ 8] );
    m2[i][ 9] = _mm_sub_epi32( m1[i][ 1], m1[i][ 9] );
    m2[i][10] = _mm_sub_epi32( m1[i][ 2], m1[i][10] );
    m2[i][11] = _mm_sub_epi32( m1[i][ 3], m1[i][11] );
    m2[i][12] = _mm_sub_epi32( m1[i][ 4], m1[i][12] );
    m2[i][13] = _mm_sub_epi32( m1[i][ 5], m1[i][13] );
    m2[i][14] = _mm_sub_epi32( m1[i][ 6], m1[i][14] );
    m2[i][15] = _mm_sub_epi32( m1[i][ 7], m1[i][15] );

    m1[i][ 0] = _mm_add_epi32( m2[i][ 0], m2[i][ 4] );
    m1[i][ 1] = _mm_add_epi32( m2[i][ 1], m2[i][ 5] );
    m1[i][ 2] = _mm_add_epi32( m2[i][ 2], m2[i][ 6] );
    m1[i][ 3] = _mm_add_epi32( m2[i][ 3], m2[i][ 7] );
    m1[i][ 4] = _mm_sub_epi32( m2[i][ 0], m2[i][ 4] );
    m1[i][ 5] = _mm_sub_epi32( m2[i][ 1], m2[i][ 5] );
    m1[i][ 6] = _mm_sub_epi32( m2[i][ 2], m2[i][ 6] );
    m1[i][ 7] = _mm_sub_epi32( m2[i][ 3], m2[i][ 7] );
    m1[i][ 8] = _mm_add_epi32( m2[i][ 8], m2[i][12] );
    m1[i][ 9] = _mm_add_epi32( m2[i][ 9], m2[i][13] );
    m1[i][10] = _mm_add_epi32( m2[i][10], m2[i][14] );
    m1[i][11] = _mm_add_epi32( m2[i][11], m2[i][15] );
    m1[i][12] = _mm_sub_epi32( m2[i][ 8], m2[i][12] );
    m1[i][13] = _mm_sub_epi32( m2[i][ 9], m2[i][13] );
    m1[i][14] = _mm_sub_epi32( m2[i][10], m2[i][14] );
    m1[i][15] = _mm_sub_epi32( m2[i][11], m2[i][15] );

    m2[i][ 0] = _mm_add_epi32( m1[i][ 0], m1[i][ 2] );
    m2[i][ 1] = _mm_add_epi32( m1[i][ 1], m1[i][ 3] );
    m2[i][ 2] = _mm_sub_epi32( m1[i][ 0], m1[i][ 2] );
    m2[i][ 3] = _mm_sub_epi32( m1[i][ 1], m1[i][ 3] );
    m2[i][ 4] = _mm_add_epi32( m1[i][ 4], m1[i][ 6] );
    m2[i][ 5] = _mm_add_epi32( m1[i][ 5], m1[i][ 7] );
    m2[i][ 6] = _mm_sub_epi32( m1[i][ 4], m1[i][ 6] );
    m2[i][ 7] = _mm_sub_epi32( m1[i][ 5], m1[i][ 7] );
    m2[i][ 8] = _mm_add_epi32( m1[i][ 8], m1[i][10] );
    m2[i][ 9] = _mm_add_epi32( m1[i][ 9], m1[i][11] );
    m2[i][10] = _mm_sub_epi32( m1[i][ 8], m1[i][10] );
    m2[i][11] = _mm_sub_epi32( m1[i][ 9], m1[i][11] );
    m2[i][12] = _mm_add_epi32( m1[i][12], m1[i][14] );
    m2[i][13] = _mm_add_epi32( m1[i][13], m1[i][15] );
    m2[i][14] = _mm_sub_epi32( m1[i][12], m1[i][14] );
    m2[i][15] = _mm_sub_epi32( m1[i][13], m1[i][15] );

    m1[i][ 0] = _mm_add_epi32( m2[i][ 0], m2[i][ 1] );
    m1[i][ 1] = _mm_sub_epi32( m2[i][ 0], m2[i][ 1] );
    m1[i][ 2] = _mm_add_epi32( m2[i][ 2], m2[i][ 3] );
    m1[i][ 3] = _mm_sub_epi32( m2[i][ 2], m2[i][ 3] );
    m1[i][ 4] = _mm_add_epi32( m2[i][ 4], m2[i][ 5] );
    m1[i][ 5] = _mm_sub_epi32( m2[i][ 4], m2[i][ 5] );
    m1[i][ 6] = _mm_add_epi32( m2[i][ 6], m2[i][ 7] );
    m1[i][ 7] = _mm_sub_epi32( m2[i][ 6], m2[i][ 7] );
    m1[i][ 8] = _mm_add_epi32( m2[i][ 8], m2[i][ 9] );
    m1[i][ 9] = _mm_sub_epi32( m2[i][ 8], m2[i][ 9] );
    m1[i][10] = _mm_add_epi32( m2[i][10], m2[i][11] );
    m1[i][11] = _mm_sub_epi32( m2[i][10], m2[i][11] );
    m1[i][12] = _mm_add_epi32( m2[i][12], m2[i][13] );
    m1[i][13] = _mm_sub_epi32( m2[i][12], m2[i][13] );
    m1[i][14] = _mm_add_epi32( m2[i][14], m2[i][15] );
    m1[i][15] = _mm_sub_epi32( m2[i][14], m2[i][15] );
  }

  // process horizontal in two steps ( 2 x 8x8 blocks )

  for( int l = 0; l < 4; l++ )
  {
    int off = l * 4;

    for( int i = 0; i < 2; i++ )
    {
      // transpose 4x4
      m2[i][0 + off] = _mm_unpacklo_epi32( m1[i][0 + off], m1[i][1 + off] );
      m2[i][1 + off] = _mm_unpackhi_epi32( m1[i][0 + off], m1[i][1 + off] );
      m2[i][2 + off] = _mm_unpacklo_epi32( m1[i][2 + off], m1[i][3 + off] );
      m2[i][3 + off] = _mm_unpackhi_epi32( m1[i][2 + off], m1[i][3 + off] );

      m1[i][0 + off] = _mm_unpacklo_epi64( m2[i][0 + off], m2[i][2 + off] );
      m1[i][1 + off] = _mm_unpackhi_epi64( m2[i][0 + off], m2[i][2 + off] );
      m1[i][2 + off] = _mm_unpacklo_epi64( m2[i][1 + off], m2[i][3 + off] );
      m1[i][3 + off] = _mm_unpackhi_epi64( m2[i][1 + off], m2[i][3 + off] );
    }
  }

  uint32_t absDc = 0;

  for( int l = 0; l < 2; l++ )
  {
    int off = l * 8;

    __m128i n1[2][8];
    __m128i n2[2][8];

    for( int i = 0; i < 8; i++ )
    {
      int ii = i % 4;
      int ij = i >> 2;

      n2[0][i] = m1[ij][off + ii    ];
      n2[1][i] = m1[ij][off + ii + 4];
    }

    for( int i = 0; i < 2; i++ )
    {
      n1[i][0] = _mm_add_epi32( n2[i][0], n2[i][4] );
      n1[i][1] = _mm_add_epi32( n2[i][1], n2[i][5] );
      n1[i][2] = _mm_add_epi32( n2[i][2], n2[i][6] );
      n1[i][3] = _mm_add_epi32( n2[i][3], n2[i][7] );
      n1[i][4] = _mm_sub_epi32( n2[i][0], n2[i][4] );
      n1[i][5] = _mm_sub_epi32( n2[i][1], n2[i][5] );
      n1[i][6] = _mm_sub_epi32( n2[i][2], n2[i][6] );
      n1[i][7] = _mm_sub_epi32( n2[i][3], n2[i][7] );

      n2[i][0] = _mm_add_epi32( n1[i][0], n1[i][2] );
      n2[i][1] = _mm_add_epi32( n1[i][1], n1[i][3] );
      n2[i][2] = _mm_sub_epi32( n1[i][0], n1[i][2] );
      n2[i][3] = _mm_sub_epi32( n1[i][1], n1[i][3] );
      n2[i][4] = _mm_add_epi32( n1[i][4], n1[i][6] );
      n2[i][5] = _mm_add_epi32( n1[i][5], n1[i][7] );
      n2[i][6] = _mm_sub_epi32( n1[i][4], n1[i][6] );
      n2[i][7] = _mm_sub_epi32( n1[i][5], n1[i][7] );

      n1[i][0] = _mm_abs_epi32( _mm_add_epi32( n2[i][0], n2[i][1] ) );
      n1[i][1] = _mm_abs_epi32( _mm_sub_epi32( n2[i][0], n2[i][1] ) );
      n1[i][2] = _mm_abs_epi32( _mm_add_epi32( n2[i][2], n2[i][3] ) );
      n1[i][3] = _mm_abs_epi32( _mm_sub_epi32( n2[i][2], n2[i][3] ) );
      n1[i][4] = _mm_abs_epi32( _mm_add_epi32( n2[i][4], n2[i][5] ) );
      n1[i][5] = _mm_abs_epi32( _mm_sub_epi32( n2[i][4], n2[i][5] ) );
      n1[i][6] = _mm_abs_epi32( _mm_add_epi32( n2[i][6], n2[i][7] ) );
      n1[i][7] = _mm_abs_epi32( _mm_sub_epi32( n2[i][6], n2[i][7] ) );
      
      if ( l + i == 0 )
        absDc = _mm_cvtsi128_si32( n1[i][0] );
    }

    for( int i = 0; i < 8; i++ )
    {
      n2[0][i] = _mm_add_epi32( n1[0][i], n1[1][i] );
    }

    n2[0][0] = _mm_add_epi32( n2[0][0], n2[0][1] );
    n2[0][2] = _mm_add_epi32( n2[0][2], n2[0][3] );
    n2[0][4] = _mm_add_epi32( n2[0][4], n2[0][5] );
    n2[0][6] = _mm_add_epi32( n2[0][6], n2[0][7] );

    n2[0][0] = _mm_add_epi32( n2[0][0], n2[0][2] );
    n2[0][4] = _mm_add_epi32( n2[0][4], n2[0][6] );
    iSum = _mm_add_epi32( iSum, _mm_add_epi32( n2[0][0], n2[0][4] ) );
  }

  iSum = _mm_hadd_epi32( iSum, iSum );
  iSum = _mm_hadd_epi32( iSum, iSum );

  uint32_t sad = _mm_cvtsi128_si32( iSum );
  sad -= absDc;
  sad += absDc >> 2;
  sad = (uint32_t)(sad / sqrt(16.0 * 8) * 2);

  return sad;
}


template< typename Torg, typename Tcur >
static uint32_t xCalcHAD8x4_SSE( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  __m128i m1[8], m2[8];
  __m128i vzero = _mm_setzero_si128();

  for( int k = 0; k < 4; k++ )
  {
    __m128i r0 = (sizeof( Torg ) > 1) ? (_mm_loadu_si128 ( (__m128i*)piOrg )) : (_mm_unpacklo_epi8( _vv_loadl_epi64( (const __m128i*)piOrg ), _mm_setzero_si128() ));
    __m128i r1 = (sizeof( Tcur ) > 1) ? (_mm_loadu_si128( (__m128i*)piCur )) : (_mm_unpacklo_epi8( _vv_loadl_epi64( (const __m128i*)piCur ), _mm_setzero_si128() )); // th  _mm_loadu_si128( (__m128i*)piCur )
    m1[k] = _mm_sub_epi16( r0, r1 );
    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //vertical
  m2[0] = _mm_add_epi16( m1[0], m1[2] );
  m2[1] = _mm_add_epi16( m1[1], m1[3] );
  m2[2] = _mm_sub_epi16( m1[0], m1[2] );
  m2[3] = _mm_sub_epi16( m1[1], m1[3] );

  m1[0] = _mm_add_epi16( m2[0], m2[1] );
  m1[1] = _mm_sub_epi16( m2[0], m2[1] );
  m1[2] = _mm_add_epi16( m2[2], m2[3] );
  m1[3] = _mm_sub_epi16( m2[2], m2[3] );

  // transpose, partially
  {
    m2[0] = _mm_unpacklo_epi16( m1[0], m1[1] );
    m2[1] = _mm_unpacklo_epi16( m1[2], m1[3] );
    m2[2] = _mm_unpackhi_epi16( m1[0], m1[1] );
    m2[3] = _mm_unpackhi_epi16( m1[2], m1[3] );

    m1[0] = _mm_unpacklo_epi32( m2[0], m2[1] );
    m1[1] = _mm_unpackhi_epi32( m2[0], m2[1] );
    m1[2] = _mm_unpacklo_epi32( m2[2], m2[3] );
    m1[3] = _mm_unpackhi_epi32( m2[2], m2[3] );
  }

  // horizontal
  if( iBitDepth >= 10 /*sizeof( Torg ) > 1 || sizeof( Tcur ) > 1*/ )
  {
    // finish transpose
    m2[0] = _mm_unpacklo_epi64( m1[0], vzero );
    m2[1] = _mm_unpackhi_epi64( m1[0], vzero );
    m2[2] = _mm_unpacklo_epi64( m1[1], vzero );
    m2[3] = _mm_unpackhi_epi64( m1[1], vzero );
    m2[4] = _mm_unpacklo_epi64( m1[2], vzero );
    m2[5] = _mm_unpackhi_epi64( m1[2], vzero );
    m2[6] = _mm_unpacklo_epi64( m1[3], vzero );
    m2[7] = _mm_unpackhi_epi64( m1[3], vzero );

    for( int i = 0; i < 8; i++ )
    {
      m2[i] = _mm_cvtepi16_epi32( m2[i] );
    }

    m1[0] = _mm_add_epi32( m2[0], m2[4] );
    m1[1] = _mm_add_epi32( m2[1], m2[5] );
    m1[2] = _mm_add_epi32( m2[2], m2[6] );
    m1[3] = _mm_add_epi32( m2[3], m2[7] );
    m1[4] = _mm_sub_epi32( m2[0], m2[4] );
    m1[5] = _mm_sub_epi32( m2[1], m2[5] );
    m1[6] = _mm_sub_epi32( m2[2], m2[6] );
    m1[7] = _mm_sub_epi32( m2[3], m2[7] );

    m2[0] = _mm_add_epi32( m1[0], m1[2] );
    m2[1] = _mm_add_epi32( m1[1], m1[3] );
    m2[2] = _mm_sub_epi32( m1[0], m1[2] );
    m2[3] = _mm_sub_epi32( m1[1], m1[3] );
    m2[4] = _mm_add_epi32( m1[4], m1[6] );
    m2[5] = _mm_add_epi32( m1[5], m1[7] );
    m2[6] = _mm_sub_epi32( m1[4], m1[6] );
    m2[7] = _mm_sub_epi32( m1[5], m1[7] );

    m1[0] = _mm_abs_epi32( _mm_add_epi32( m2[0], m2[1] ) );
    m1[1] = _mm_abs_epi32( _mm_sub_epi32( m2[0], m2[1] ) );
    m1[2] = _mm_abs_epi32( _mm_add_epi32( m2[2], m2[3] ) );
    m1[3] = _mm_abs_epi32( _mm_sub_epi32( m2[2], m2[3] ) );
    m1[4] = _mm_abs_epi32( _mm_add_epi32( m2[4], m2[5] ) );
    m1[5] = _mm_abs_epi32( _mm_sub_epi32( m2[4], m2[5] ) );
    m1[6] = _mm_abs_epi32( _mm_add_epi32( m2[6], m2[7] ) );
    m1[7] = _mm_abs_epi32( _mm_sub_epi32( m2[6], m2[7] ) );
  }
  else
  {
    m2[0] = _mm_add_epi16( m1[0], m1[2] );
    m2[1] = _mm_add_epi16( m1[1], m1[3] );
    m2[2] = _mm_sub_epi16( m1[0], m1[2] );
    m2[3] = _mm_sub_epi16( m1[1], m1[3] );

    m1[0] = _mm_add_epi16( m2[0], m2[1] );
    m1[1] = _mm_sub_epi16( m2[0], m2[1] );
    m1[2] = _mm_add_epi16( m2[2], m2[3] );
    m1[3] = _mm_sub_epi16( m2[2], m2[3] );

    // finish transpose
    m2[0] = _mm_unpacklo_epi64( m1[0], vzero );
    m2[1] = _mm_unpackhi_epi64( m1[0], vzero );
    m2[2] = _mm_unpacklo_epi64( m1[1], vzero );
    m2[3] = _mm_unpackhi_epi64( m1[1], vzero );
    m2[4] = _mm_unpacklo_epi64( m1[2], vzero );
    m2[5] = _mm_unpackhi_epi64( m1[2], vzero );
    m2[6] = _mm_unpacklo_epi64( m1[3], vzero );
    m2[7] = _mm_unpackhi_epi64( m1[3], vzero );

    m1[0] = _mm_abs_epi16( _mm_add_epi16( m2[0], m2[1] ) );
    m1[1] = _mm_abs_epi16( _mm_sub_epi16( m2[0], m2[1] ) );
    m1[2] = _mm_abs_epi16( _mm_add_epi16( m2[2], m2[3] ) );
    m1[3] = _mm_abs_epi16( _mm_sub_epi16( m2[2], m2[3] ) );
    m1[4] = _mm_abs_epi16( _mm_add_epi16( m2[4], m2[5] ) );
    m1[5] = _mm_abs_epi16( _mm_sub_epi16( m2[4], m2[5] ) );
    m1[6] = _mm_abs_epi16( _mm_add_epi16( m2[6], m2[7] ) );
    m1[7] = _mm_abs_epi16( _mm_sub_epi16( m2[6], m2[7] ) );

    for( int i = 0; i < 8; i++ )
    {
      m1[i] = _mm_unpacklo_epi16( m1[i], vzero );
    }
  }
  
  uint32_t absDc = _mm_cvtsi128_si32( m1[0] );

  m1[0] = _mm_add_epi32( m1[0], m1[1] );
  m1[1] = _mm_add_epi32( m1[2], m1[3] );
  m1[2] = _mm_add_epi32( m1[4], m1[5] );
  m1[3] = _mm_add_epi32( m1[6], m1[7] );

  m1[0] = _mm_add_epi32( m1[0], m1[1] );
  m1[1] = _mm_add_epi32( m1[2], m1[3] );

  __m128i iSum = _mm_add_epi32( m1[0], m1[1] );

  iSum = _mm_hadd_epi32( iSum, iSum );
  iSum = _mm_hadd_epi32( iSum, iSum );

  uint32_t sad = _mm_cvtsi128_si32( iSum );
  sad -= absDc;
  sad += absDc >> 2;
  sad = (uint32_t)(sad / sqrt(4.0 * 8) * 2);
  return sad;
}

static uint32_t xCalcHAD4x8_SSE( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  __m128i m1[8], m2[8];

  for( int k = 0; k < 8; k++ )
  {
    __m128i r0 = (sizeof( Torg ) > 1) ? (_vv_loadl_epi64( (__m128i*)piOrg )) : (_mm_cvtsi32_si128( *(const int*)piOrg ));
    __m128i r1 = (sizeof( Tcur ) > 1) ? (_vv_loadl_epi64( (__m128i*)piCur )) : (_mm_cvtsi32_si128( *(const int*)piCur ));
    m2[k] = _mm_sub_epi16( r0, r1 );
    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }


  // vertical

  m1[0] = _mm_add_epi16( m2[0], m2[4] );
  m1[1] = _mm_add_epi16( m2[1], m2[5] );
  m1[2] = _mm_add_epi16( m2[2], m2[6] );
  m1[3] = _mm_add_epi16( m2[3], m2[7] );
  m1[4] = _mm_sub_epi16( m2[0], m2[4] );
  m1[5] = _mm_sub_epi16( m2[1], m2[5] );
  m1[6] = _mm_sub_epi16( m2[2], m2[6] );
  m1[7] = _mm_sub_epi16( m2[3], m2[7] );

  m2[0] = _mm_add_epi16( m1[0], m1[2] );
  m2[1] = _mm_add_epi16( m1[1], m1[3] );
  m2[2] = _mm_sub_epi16( m1[0], m1[2] );
  m2[3] = _mm_sub_epi16( m1[1], m1[3] );
  m2[4] = _mm_add_epi16( m1[4], m1[6] );
  m2[5] = _mm_add_epi16( m1[5], m1[7] );
  m2[6] = _mm_sub_epi16( m1[4], m1[6] );
  m2[7] = _mm_sub_epi16( m1[5], m1[7] );

  m1[0] = _mm_add_epi16( m2[0], m2[1] );
  m1[1] = _mm_sub_epi16( m2[0], m2[1] );
  m1[2] = _mm_add_epi16( m2[2], m2[3] );
  m1[3] = _mm_sub_epi16( m2[2], m2[3] );
  m1[4] = _mm_add_epi16( m2[4], m2[5] );
  m1[5] = _mm_sub_epi16( m2[4], m2[5] );
  m1[6] = _mm_add_epi16( m2[6], m2[7] );
  m1[7] = _mm_sub_epi16( m2[6], m2[7] );


  // horizontal
  // transpose
  {
    m2[0] = _mm_unpacklo_epi16( m1[0], m1[1] );
    m2[1] = _mm_unpacklo_epi16( m1[2], m1[3] );
    m2[2] = _mm_unpacklo_epi16( m1[4], m1[5] );
    m2[3] = _mm_unpacklo_epi16( m1[6], m1[7] );

    m1[0] = _mm_unpacklo_epi32( m2[0], m2[1] );
    m1[1] = _mm_unpackhi_epi32( m2[0], m2[1] );
    m1[2] = _mm_unpacklo_epi32( m2[2], m2[3] );
    m1[3] = _mm_unpackhi_epi32( m2[2], m2[3] );

    m2[0] = _mm_unpacklo_epi64( m1[0], m1[2] );
    m2[1] = _mm_unpackhi_epi64( m1[0], m1[2] );
    m2[2] = _mm_unpacklo_epi64( m1[1], m1[3] );
    m2[3] = _mm_unpackhi_epi64( m1[1], m1[3] );
  }

  uint32_t absDc = 0;

  if( iBitDepth >= 10 /*sizeof( Torg ) > 1 || sizeof( Tcur ) > 1*/ )
  {
    __m128i n1[4][2];
    __m128i n2[4][2];

    for( int i = 0; i < 4; i++ )
    {
      n1[i][0] = _mm_cvtepi16_epi32( m2[i] );
      n1[i][1] = _mm_cvtepi16_epi32( _mm_shuffle_epi32( m2[i], 0xEE ) );
    }

    for( int i = 0; i < 2; i++ )
    {
      n2[0][i] = _mm_add_epi32( n1[0][i], n1[2][i] );
      n2[1][i] = _mm_add_epi32( n1[1][i], n1[3][i] );
      n2[2][i] = _mm_sub_epi32( n1[0][i], n1[2][i] );
      n2[3][i] = _mm_sub_epi32( n1[1][i], n1[3][i] );

      n1[0][i] = _mm_abs_epi32( _mm_add_epi32( n2[0][i], n2[1][i] ) );
      n1[1][i] = _mm_abs_epi32( _mm_sub_epi32( n2[0][i], n2[1][i] ) );
      n1[2][i] = _mm_abs_epi32( _mm_add_epi32( n2[2][i], n2[3][i] ) );
      n1[3][i] = _mm_abs_epi32( _mm_sub_epi32( n2[2][i], n2[3][i] ) );
    }
    for( int i = 0; i < 4; i++ )
    {
      m1[i] = _mm_add_epi32( n1[i][0], n1[i][1] );
    }

    absDc = _mm_cvtsi128_si32( n1[0][0] );
  }
  else
  {
    m1[0] = _mm_add_epi16( m2[0], m2[2] );
    m1[1] = _mm_add_epi16( m2[1], m2[3] );
    m1[2] = _mm_sub_epi16( m2[0], m2[2] );
    m1[3] = _mm_sub_epi16( m2[1], m2[3] );

    m2[0] = _mm_abs_epi16( _mm_add_epi16( m1[0], m1[1] ) );
    m2[1] = _mm_abs_epi16( _mm_sub_epi16( m1[0], m1[1] ) );
    m2[2] = _mm_abs_epi16( _mm_add_epi16( m1[2], m1[3] ) );
    m2[3] = _mm_abs_epi16( _mm_sub_epi16( m1[2], m1[3] ) );

    __m128i ma1, ma2;
    __m128i vzero = _mm_setzero_si128();

    for( int i = 0; i < 4; i++ )
    {
      ma1 = _mm_unpacklo_epi16( m2[i], vzero );
      ma2 = _mm_unpackhi_epi16( m2[i], vzero );
      m1[i] = _mm_add_epi32( ma1, ma2 );
    }

    absDc = _mm_cvtsi128_si32( m2[0] ) & 0x0000ffff;
  }

  m1[0] = _mm_add_epi32( m1[0], m1[1] );
  m1[2] = _mm_add_epi32( m1[2], m1[3] );

  __m128i iSum = _mm_add_epi32( m1[0], m1[2] );

  iSum = _mm_hadd_epi32( iSum, iSum );
  iSum = _mm_hadd_epi32( iSum, iSum );

  uint32_t sad = _mm_cvtsi128_si32( iSum );
  
  sad -= absDc;
  sad += absDc >> 2;
  sad = (uint32_t)(sad / sqrt(4.0 * 8) * 2);

  return sad;
}

#ifdef USE_AVX2
// The helpers below compute SATDs of residuals of at most 12 bits (|value| <= 2047): 10-bit input gives 11 bits,
// and bi-prediction motion estimation compares against 2 * org - pred, which is not clipped, so its residuals reach
// +-2046. Each butterfly stage can double the values, so four stages always fit 16-bit values (16 * 2047 = 32752).
// The later stages stay in 16 bits when the values after those four allow it, which nearly all blocks do, and run
// on 32-bit values otherwise.

// The normalization ( uint32_t )( sad / sqrt( n ) * 2 ) of the n = 16 * 8 and n = 4 * 8 SATDs without the double
// division: ( sad * M ) >> 42 with M = ceil( 2^43 / c ), c the double sqrt( n ). The result is identical for every sad
// below 7095941 (checked exhaustively), and these SATDs stay far below: the transform is orthogonal up to sqrt( n ), so
// the sum of absolute coefficients is at most n * sqrt( n ) * 2047 < 3 million.
static constexpr uint32_t HAD_SCALE_MAX_SAD   = 7095940;
static constexpr uint64_t HAD_SCALE_16x8_MULT = 777472127994ull;    // n = 128
static constexpr uint64_t HAD_SCALE_8x4_MULT  = 1554944255988ull;   // n = 32

static inline uint32_t xScaleHAD( const uint32_t sad, const uint64_t mult )
{
  CHECKD( sad > HAD_SCALE_MAX_SAD, "SATD outside the range of the fixed-point normalization" );
  return uint32_t( ( uint64_t( sad ) * mult ) >> 42 );
}

// Transposes the 8x8 block of 16-bit values in each 128-bit lane: afterwards register k holds column k of both blocks.
static inline void xTranspose8x8Lanes_AVX2( __m256i r[8] )
{
  __m256i t[8];
  t[0] = _mm256_unpacklo_epi16( r[0], r[1] );
  t[1] = _mm256_unpacklo_epi16( r[2], r[3] );
  t[2] = _mm256_unpacklo_epi16( r[4], r[5] );
  t[3] = _mm256_unpacklo_epi16( r[6], r[7] );
  t[4] = _mm256_unpackhi_epi16( r[0], r[1] );
  t[5] = _mm256_unpackhi_epi16( r[2], r[3] );
  t[6] = _mm256_unpackhi_epi16( r[4], r[5] );
  t[7] = _mm256_unpackhi_epi16( r[6], r[7] );

  r[0] = _mm256_unpacklo_epi32( t[0], t[1] );
  r[1] = _mm256_unpackhi_epi32( t[0], t[1] );
  r[2] = _mm256_unpacklo_epi32( t[2], t[3] );
  r[3] = _mm256_unpackhi_epi32( t[2], t[3] );
  r[4] = _mm256_unpacklo_epi32( t[4], t[5] );
  r[5] = _mm256_unpackhi_epi32( t[4], t[5] );
  r[6] = _mm256_unpacklo_epi32( t[6], t[7] );
  r[7] = _mm256_unpackhi_epi32( t[6], t[7] );

  t[0] = _mm256_unpacklo_epi64( r[0], r[2] );
  t[1] = _mm256_unpackhi_epi64( r[0], r[2] );
  t[2] = _mm256_unpacklo_epi64( r[1], r[3] );
  t[3] = _mm256_unpackhi_epi64( r[1], r[3] );
  t[4] = _mm256_unpacklo_epi64( r[4], r[6] );
  t[5] = _mm256_unpackhi_epi64( r[4], r[6] );
  t[6] = _mm256_unpacklo_epi64( r[5], r[7] );
  t[7] = _mm256_unpackhi_epi64( r[5], r[7] );

  for( int k = 0; k < 8; k++ )
  {
    r[k] = t[k];
  }
}

// One butterfly stage of a Hadamard transform on 16-bit values: registers k and k + distance become their sum and
// their difference, for every k that has no bit of distance set.
static inline void xHadamardStage16_AVX2( __m256i r[8], const int distance )
{
  for( int k = 0; k < 8; k++ )
  {
    if( k & distance )
    {
      continue;
    }
    const __m256i sum  = _mm256_add_epi16( r[k], r[k + distance] );
    const __m256i diff = _mm256_sub_epi16( r[k], r[k + distance] );
    r[k]               = sum;
    r[k + distance]    = diff;
  }
}

// The same butterfly stage on 32-bit values.
static inline void xHadamardStage32_AVX2( __m256i r[8], const int distance )
{
  for( int k = 0; k < 8; k++ )
  {
    if( k & distance )
    {
      continue;
    }
    const __m256i sum  = _mm256_add_epi32( r[k], r[k + distance] );
    const __m256i diff = _mm256_sub_epi32( r[k], r[k + distance] );
    r[k]               = sum;
    r[k + distance]    = diff;
  }
}

// Adds max( |a|, |b| ) of the 32-bit values of registers 0 and 1, 2 and 3, 4 and 5, 6 and 7 to sum. Twice that is the
// sum of absolute values after a last butterfly stage between these registers, so that stage is never computed:
// |a + b| + |a - b| = 2 * max( |a|, |b| ).
static inline void xAddMaxAbsOfPairs32_AVX2( const __m256i r[8], __m256i& sum )
{
  for( int k = 0; k < 8; k += 2 )
  {
    sum = _mm256_add_epi32( sum, _mm256_max_epi32( _mm256_abs_epi32( r[k] ), _mm256_abs_epi32( r[k + 1] ) ) );
  }
}

// The same on 16-bit values, which must be below 2^15 in magnitude. The sum is still 32 bits per value.
static inline void xAddMaxAbsOfPairs16_AVX2( const __m256i r[8], __m256i& sum )
{
  const __m256i one = _mm256_set1_epi16( 1 );
  for( int k = 0; k < 8; k += 2 )
  {
    const __m256i maxAbs = _mm256_max_epi16( _mm256_abs_epi16( r[k] ), _mm256_abs_epi16( r[k + 1] ) );
    sum = _mm256_add_epi32( sum, _mm256_madd_epi16( maxAbs, one ) );
  }
}

// Whether all 16-bit values have a magnitude below limit, a power of two: no magnitude has a bit from limit upwards.
static inline bool xAllMagnitudesBelow_AVX2( const __m256i r[8], const int16_t limit )
{
  __m256i bits = _mm256_setzero_si256();
  for( int k = 0; k < 8; k++ )
  {
    bits = _mm256_or_si256( bits, _mm256_abs_epi16( r[k] ) );
  }
  return _mm256_testz_si256( bits, _mm256_set1_epi16( -limit ) ) != 0;
}

// a + b as 32 bits, for the first 16-bit value of a and of b in each lane, in the first 32-bit value of that lane.
static inline __m256i xAddFirstValues16_AVX2( const __m256i a, const __m256i b )
{
  return _mm256_madd_epi16( _mm256_unpacklo_epi16( a, b ), _mm256_set1_epi16( 1 ) );
}

// The SATDs of two 8x8 blocks of residuals, one per 128-bit lane (register k holds row k of both), added up as the
// 16x16 and 32x32 kernels do: ( SATD + 2 ) >> 2 per block.
static ALWAYS_INLINE uint32_t xCalcHAD8x8Pair_AVX2( __m256i rows[8] )
{
  // the vertical transform and the first horizontal stage on 16-bit values
  xHadamardStage16_AVX2( rows, 4 );
  xHadamardStage16_AVX2( rows, 2 );
  xHadamardStage16_AVX2( rows, 1 );
  xTranspose8x8Lanes_AVX2( rows );
  xHadamardStage16_AVX2( rows, 4 );

  // the DC coefficient of each block is the first value of its lane in registers 0 and 1, added after the next stage
  __m256i dc, iSum = _mm256_setzero_si256();
  if( xAllMagnitudesBelow_AVX2( rows, 1 << 14 ) )
  {
    // the next stage stays below 2^15
    xHadamardStage16_AVX2( rows, 2 );
    dc = xAddFirstValues16_AVX2( rows[0], rows[1] );
    xAddMaxAbsOfPairs16_AVX2( rows, iSum );
  }
  else
  {
    // on 32-bit values, each block staying in its lane: low[k] holds values 0 to 3 of both lanes of rows[k], high[k]
    // values 4 to 7
    __m256i low[8], high[8];
    for( int k = 0; k < 8; k++ )
    {
      const __m256i sign = _mm256_srai_epi16( rows[k], 15 );
      low[k]  = _mm256_unpacklo_epi16( rows[k], sign );
      high[k] = _mm256_unpackhi_epi16( rows[k], sign );
    }
    xHadamardStage32_AVX2( low, 2 );
    xHadamardStage32_AVX2( high, 2 );
    dc = _mm256_add_epi32( low[0], low[1] );
    xAddMaxAbsOfPairs32_AVX2( low, iSum );
    xAddMaxAbsOfPairs32_AVX2( high, iSum );
  }
  const uint32_t absDc[2] = { (uint32_t) abs( _mm256_extract_epi32( dc, 0 ) ), (uint32_t) abs( _mm256_extract_epi32( dc, 4 ) ) };

  iSum = _mm256_slli_epi32( iSum, 1 );      // the sum of absolute values after the last horizontal stage
  iSum = _mm256_hadd_epi32( iSum, iSum );
  iSum = _mm256_hadd_epi32( iSum, iSum );   // the sum of each block in the first value of its lane

  uint32_t sad = 0;
  for( int lane = 0; lane < 2; lane++ )
  {
    uint32_t tmp = lane ? _mm256_extract_epi32( iSum, 4 ) : _mm256_extract_epi32( iSum, 0 );
    tmp -= absDc[lane];
    tmp += absDc[lane] >> 2;
    sad += ( tmp + 2 ) >> 2;
  }
  return sad;
}

// The SATD of a 16x8 or an 8x16 block. rows[] holds its residuals as two 8x8 blocks, one per 128-bit lane: for 16x8,
// register k is row k, columns 0 to 7 in the low lane and 8 to 15 in the high lane; for 8x16, register k is row k in
// the low lane and row k + 8 in the high lane. Either way the block's Hadamard transform is the 8x8 transform of each
// lane plus one stage between the lanes, and the stages may run in any order.
static ALWAYS_INLINE uint32_t xCalcHAD16x8Lanes_AVX2( __m256i rows[8] )
{
  // the vertical transform of both lanes and the first horizontal stage on 16-bit values
  xHadamardStage16_AVX2( rows, 4 );
  xHadamardStage16_AVX2( rows, 2 );
  xHadamardStage16_AVX2( rows, 1 );
  xTranspose8x8Lanes_AVX2( rows );
  xHadamardStage16_AVX2( rows, 4 );

  // Next the stage between the lanes, their sum and their difference, and one more stage. The DC coefficient is the
  // first value of the sums in registers 0 and 1 after them.
  uint32_t absDc;
  __m256i iSum = _mm256_setzero_si256();
  if( xAllMagnitudesBelow_AVX2( rows, 1 << 13 ) )
  {
    // Both stages stay below 2^15. With r = [a|b], r + [b|-a] = [a + b|b - a] needs no second register: the low lane
    // gets the sum, the high lane the difference, whose sign does not matter as only absolute values are used.
    const __m256i lowPlusHighMinus = _mm256_setr_epi16( 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1 );
    for( int k = 0; k < 8; k++ )
    {
      const __m256i swapped = _mm256_permute2x128_si256( rows[k], rows[k], 0x01 );
      rows[k] = _mm256_add_epi16( rows[k], _mm256_sign_epi16( swapped, lowPlusHighMinus ) );
    }
    xHadamardStage16_AVX2( rows, 2 );
    absDc = abs( _mm256_extract_epi32( xAddFirstValues16_AVX2( rows[0], rows[1] ), 0 ) );
    xAddMaxAbsOfPairs16_AVX2( rows, iSum );
  }
  else
  {
    __m256i sum[8], dif[8];
    for( int k = 0; k < 8; k++ )
    {
      const __m256i low  = _mm256_cvtepi16_epi32( _mm256_castsi256_si128   ( rows[k] ) );
      const __m256i high = _mm256_cvtepi16_epi32( _mm256_extracti128_si256( rows[k], 1 ) );
      sum[k] = _mm256_add_epi32( low, high );
      dif[k] = _mm256_sub_epi32( low, high );
    }
    xHadamardStage32_AVX2( sum, 2 );
    xHadamardStage32_AVX2( dif, 2 );
    absDc = abs( _mm256_extract_epi32( sum[0], 0 ) + _mm256_extract_epi32( sum[1], 0 ) );
    xAddMaxAbsOfPairs32_AVX2( sum, iSum );
    xAddMaxAbsOfPairs32_AVX2( dif, iSum );
  }
  // The 32-bit adds of the eight partial sums wrap as the uint32 arithmetic does, so doubling the total equals doubling each
  // partial sum.
  uint32_t sad = ( uint32_t ) xSumOfLanes32_AVX2( iSum ) << 1;   // the sum of absolute values after the last horizontal stage
  sad -= absDc;
  sad += absDc >> 2;
  return xScaleHAD( sad, HAD_SCALE_16x8_MULT );
}
#endif //USE_AVX2

static uint32_t xCalcHAD32x32_fast_AVX2( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  CHECK( iBitDepth > 10, "Only bitdepths up to 10 supported!" );

  // The 32x32 block is subsampled to 16x16 by averaging 2x2 samples, and the SATD of the 16x16 block is the sum of
  // its four 8x8 SATDs, two rows of two blocks.
  for( int l = 0; l < 2; l++ )
  {
    __m256i rows[8];
    for( int k = 0; k < 8; k++ )
    {
      __m256i r0 = _mm256_loadu_si256( ( __m256i* ) piOrg );
      __m256i r1 = _mm256_loadu_si256( ( __m256i* ) piCur );
      __m256i r2 = _mm256_loadu_si256( ( __m256i* ) ( piOrg + iStrideOrg ) );
      __m256i r3 = _mm256_loadu_si256( ( __m256i* ) ( piCur + iStrideCur ) );

      r0 = _mm256_add_epi16( r0, r2 );
      r1 = _mm256_add_epi16( r1, r3 );

      __m256i r4 = _mm256_loadu_si256( ( __m256i* ) ( piOrg + 16 ) );
      __m256i r5 = _mm256_loadu_si256( ( __m256i* ) ( piCur + 16 ) );
      r2 = _mm256_loadu_si256( ( __m256i* ) ( piOrg + iStrideOrg + 16 ) );
      r3 = _mm256_loadu_si256( ( __m256i* ) ( piCur + iStrideCur + 16 ) );

      r2 = _mm256_add_epi16( r4, r2 );
      r3 = _mm256_add_epi16( r5, r3 );

      r0 = _mm256_hadd_epi16( r0, r2 );
      r1 = _mm256_hadd_epi16( r1, r3 );

      r0 = _mm256_add_epi16( r0, _mm256_set1_epi16( 2 ) );
      r1 = _mm256_add_epi16( r1, _mm256_set1_epi16( 2 ) );

      r0 = _mm256_srai_epi16( r0, 2 );
      r1 = _mm256_srai_epi16( r1, 2 );

      rows[k] = _mm256_permute4x64_epi64( _mm256_sub_epi16( r0, r1 ), 0 + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );
      piCur += iStrideCur * 2;
      piOrg += iStrideOrg * 2;
    }
    sad += xCalcHAD8x8Pair_AVX2( rows );
  }

#endif
  return ( sad << 2 );
}

static uint32_t xCalcHAD16x16_AVX2( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  CHECK( iBitDepth > 10, "Only bitdepths up to 10 supported!" );

  // the SATD of a 16x16 block is the sum of its four 8x8 SATDs, two rows of two blocks
  for( int l = 0; l < 2; l++ )
  {
    __m256i rows[8];
    for( int k = 0; k < 8; k++ )
    {
      rows[k] = _mm256_sub_epi16( _mm256_loadu_si256( ( const __m256i* ) piOrg ), _mm256_loadu_si256( ( const __m256i* ) piCur ) );
      piCur += iStrideCur;
      piOrg += iStrideOrg;
    }
    sad += xCalcHAD8x8Pair_AVX2( rows );
  }

#endif
  return ( sad );
}

static uint32_t xCalcHAD16x8_AVX2( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  CHECK( iBitDepth > 10, "Only bitdepths up to 10 supported!" );

  __m256i rows[8];
  for( int k = 0; k < 8; k++ )
  {
    rows[k] = _mm256_sub_epi16( _mm256_loadu_si256( ( const __m256i* ) &piOrg[k * iStrideOrg] ),
                                _mm256_loadu_si256( ( const __m256i* ) &piCur[k * iStrideCur] ) );
  }
  sad = xCalcHAD16x8Lanes_AVX2( rows );

#endif //USE_AVX2

  return (sad);
}

static uint32_t xCalcHAD8x16_AVX2( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  CHECK( iBitDepth > 10, "Only bitdepths up to 10 supported!" );

  __m256i rows[8];
  for( int k = 0; k < 8; k++ )
  {
    const __m128i orgK  = _mm_loadu_si128( ( const __m128i* ) &piOrg[ k      * iStrideOrg] );
    const __m128i orgK8 = _mm_loadu_si128( ( const __m128i* ) &piOrg[( k + 8 ) * iStrideOrg] );
    const __m128i curK  = _mm_loadu_si128( ( const __m128i* ) &piCur[ k      * iStrideCur] );
    const __m128i curK8 = _mm_loadu_si128( ( const __m128i* ) &piCur[( k + 8 ) * iStrideCur] );
    rows[k] = _mm256_sub_epi16( _mm256_inserti128_si256( _mm256_castsi128_si256( orgK ), orgK8, 1 ),
                                _mm256_inserti128_si256( _mm256_castsi128_si256( curK ), curK8, 1 ) );
  }
  sad = xCalcHAD16x8Lanes_AVX2( rows );

#endif //USE_AVX2

  return (sad);
}

#ifdef USE_AVX2
// The 8x4, 4x8 and 8x8 kernels below hold their whole block in two or four registers, two or four rows per register.
// The butterfly stages between values of one register are a shuffle, a sign change and an add. Four stages run on
// 16-bit values (see the bound above), and the last stage is never computed: |a + b| + |a - b| = 2 * max( |a|, |b| ).

// The rows y and y + 1 of org - cur of a block 8 wide, one row per 128-bit lane.
static inline __m256i xResidualRows8x2_AVX2( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur )
{
  const __m256i org = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* ) piOrg ) ),
                                               _mm_loadu_si128( ( const __m128i* ) ( piOrg + iStrideOrg ) ), 1 );
  const __m256i cur = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* ) piCur ) ),
                                               _mm_loadu_si128( ( const __m128i* ) ( piCur + iStrideCur ) ), 1 );
  return _mm256_sub_epi16( org, cur );
}

// The rows y to y + 3 of org - cur of a block 4 wide: rows y and y + 1 in the low lane, y + 2 and y + 3 in the high lane.
static inline __m256i xResidualRows4x4_AVX2( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur )
{
  auto loadRows = []( const Pel* p, const int stride )
  {
    const __m128i rows01 = _mm_unpacklo_epi64( _vv_loadl_epi64( ( const __m128i* ) p ),
                                               _vv_loadl_epi64( ( const __m128i* ) ( p + stride ) ) );
    const __m128i rows23 = _mm_unpacklo_epi64( _vv_loadl_epi64( ( const __m128i* ) ( p + 2 * stride ) ),
                                               _vv_loadl_epi64( ( const __m128i* ) ( p + 3 * stride ) ) );
    return _mm256_inserti128_si256( _mm256_castsi128_si256( rows01 ), rows23, 1 );
  };
  return _mm256_sub_epi16( loadRows( piOrg, iStrideOrg ), loadRows( piCur, iStrideCur ) );
}

// The butterfly stage between the two 128-bit lanes: the low lane becomes low + high, the high lane low - high.
static inline __m256i xHadamardLaneStage16_AVX2( const __m256i x )
{
  const __m256i negateHighLane = _mm256_setr_epi16( 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1 );
  return _mm256_add_epi16( _mm256_permute2x128_si256( x, x, 0x01 ), _mm256_sign_epi16( x, negateHighLane ) );
}

// The butterfly stage between the two 64-bit halves of each lane: the low half becomes low + high, the high half
// low - high.
static inline __m256i xHadamardHalfStage16_AVX2( const __m256i x )
{
  const __m256i negateHighHalves = _mm256_setr_epi16( 1, 1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1 );
  return _mm256_add_epi16( _mm256_shuffle_epi32( x, 0x4E ), _mm256_sign_epi16( x, negateHighHalves ) );
}

// The butterfly stage between neighbouring 32-bit pairs of 16-bit values: values 0 and 1 become their sums with
// values 2 and 3, values 2 and 3 the differences, and so on.
static inline __m256i xHadamardPairStage16_AVX2( const __m256i x )
{
  const __m256i negateOddPairs = _mm256_setr_epi16( 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1 );
  return _mm256_add_epi16( _mm256_shuffle_epi32( x, 0xB1 ), _mm256_sign_epi16( x, negateOddPairs ) );
}

// The sum of absolute values after a last butterfly stage between neighbouring 16-bit values, as 32-bit partial sums:
// each pair's max( |a|, |b| ) is in both of its values, so the pair adds 2 * max( |a|, |b| ).
static inline __m256i xSumAbsAfterNeighbourStage16_AVX2( const __m256i x )
{
  const __m256i swapNeighbours = _mm256_setr_epi8( 2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13,
                                                   2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13 );
  const __m256i absX   = _mm256_abs_epi16( x );
  const __m256i maxAbs = _mm256_max_epi16( absX, _mm256_shuffle_epi8( absX, swapNeighbours ) );
  return _mm256_madd_epi16( maxAbs, _mm256_set1_epi16( 1 ) );
}

// The same on 32-bit values: each pair's max( |a|, |b| ) in both of its values.
static inline __m256i xMaxAbsOfNeighbours32_AVX2( const __m256i x )
{
  const __m256i absX = _mm256_abs_epi32( x );
  return _mm256_max_epi32( absX, _mm256_shuffle_epi32( absX, 0xB1 ) );
}

static inline uint32_t xSumOfValues32_AVX2( const __m256i x )
{
  __m128i sum = _mm_add_epi32( _mm256_castsi256_si128( x ), _mm256_extracti128_si256( x, 1 ) );
  sum = _mm_add_epi32( sum, _mm_shuffle_epi32( sum, 0x4E ) );
  sum = _mm_add_epi32( sum, _mm_shuffle_epi32( sum, 0xB1 ) );
  return _mm_cvtsi128_si32( sum );
}

// The absolute DC coefficient when the first two 16-bit values of x are the last stage's DC pair.
static inline uint32_t xAbsDcOfFirstPair16_AVX2( const __m256i x )
{
  const int pair = _mm_cvtsi128_si32( _mm256_castsi256_si128( x ) );
  return abs( ( int16_t ) pair + ( int16_t ) ( pair >> 16 ) );
}

// The 8x4 and 4x8 SATDs, normalized as xCalcHAD8x4_SSE and xCalcHAD4x8_SSE do.
static inline uint32_t xNormalizedHAD32Samples( uint32_t sad, const uint32_t absDc )
{
  sad -= absDc;
  sad += absDc >> 2;
  return xScaleHAD( sad, HAD_SCALE_8x4_MULT );
}
#endif //USE_AVX2

static uint32_t xCalcHAD8x4_AVX2( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  const __m256i rows01 = xResidualRows8x2_AVX2( piOrg, piCur, iStrideOrg, iStrideCur );
  const __m256i rows23 = xResidualRows8x2_AVX2( piOrg + 2 * iStrideOrg, piCur + 2 * iStrideCur, iStrideOrg, iStrideCur );

  // the vertical transform: the stage between the registers (rows k and k + 2), then the one between the lanes (rows
  // k and k + 1); then two horizontal stages, and the last one folded into the sum
  __m256i sum = xHadamardLaneStage16_AVX2( _mm256_add_epi16( rows01, rows23 ) );
  __m256i dif = xHadamardLaneStage16_AVX2( _mm256_sub_epi16( rows01, rows23 ) );
  sum = xHadamardPairStage16_AVX2( xHadamardHalfStage16_AVX2( sum ) );
  dif = xHadamardPairStage16_AVX2( xHadamardHalfStage16_AVX2( dif ) );

  sad = xSumOfValues32_AVX2( _mm256_add_epi32( xSumAbsAfterNeighbourStage16_AVX2( sum ),
                                               xSumAbsAfterNeighbourStage16_AVX2( dif ) ) );
  sad = xNormalizedHAD32Samples( sad, xAbsDcOfFirstPair16_AVX2( sum ) );

#endif //USE_AVX2

  return sad;
}

static uint32_t xCalcHAD4x8_AVX2( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  const __m256i rows0to3 = xResidualRows4x4_AVX2( piOrg, piCur, iStrideOrg, iStrideCur );
  const __m256i rows4to7 = xResidualRows4x4_AVX2( piOrg + 4 * iStrideOrg, piCur + 4 * iStrideCur, iStrideOrg, iStrideCur );

  // the vertical transform: the stages between the registers (rows k and k + 4), between the lanes (k and k + 2) and
  // between the halves (k and k + 1); then one horizontal stage, and the last one folded into the sum
  __m256i sum = xHadamardHalfStage16_AVX2( xHadamardLaneStage16_AVX2( _mm256_add_epi16( rows0to3, rows4to7 ) ) );
  __m256i dif = xHadamardHalfStage16_AVX2( xHadamardLaneStage16_AVX2( _mm256_sub_epi16( rows0to3, rows4to7 ) ) );
  sum = xHadamardPairStage16_AVX2( sum );
  dif = xHadamardPairStage16_AVX2( dif );

  sad = xSumOfValues32_AVX2( _mm256_add_epi32( xSumAbsAfterNeighbourStage16_AVX2( sum ),
                                               xSumAbsAfterNeighbourStage16_AVX2( dif ) ) );
  sad = xNormalizedHAD32Samples( sad, xAbsDcOfFirstPair16_AVX2( sum ) );

#endif //USE_AVX2

  return sad;
}

static uint32_t xCalcHAD8x8_AVX2( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  // r[k] holds the rows 2k and 2k + 1
  __m256i r[4];
  for( int k = 0; k < 4; k++ )
  {
    r[k] = xResidualRows8x2_AVX2( piOrg + 2 * k * iStrideOrg, piCur + 2 * k * iStrideCur, iStrideOrg, iStrideCur );
  }

  // the vertical transform: the stages between the registers (rows k and k + 4, then k and k + 2), then the one
  // between the lanes (k and k + 1); then the first horizontal stage, between the halves
  const __m256i sum02 = _mm256_add_epi16( r[0], r[2] );
  const __m256i sum13 = _mm256_add_epi16( r[1], r[3] );
  const __m256i dif02 = _mm256_sub_epi16( r[0], r[2] );
  const __m256i dif13 = _mm256_sub_epi16( r[1], r[3] );
  r[0] = _mm256_add_epi16( sum02, sum13 );
  r[1] = _mm256_sub_epi16( sum02, sum13 );
  r[2] = _mm256_add_epi16( dif02, dif13 );
  r[3] = _mm256_sub_epi16( dif02, dif13 );

  // the fifth stage, between 16-bit neighbours, as the sums and differences of madd, which are 32-bit; the last
  // stage, between their 32-bit neighbours, folded into the sum
  const __m256i plus      = _mm256_set1_epi16( 1 );
  const __m256i plusMinus = _mm256_setr_epi16( 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1 );
  __m256i absSum = _mm256_setzero_si256();
  uint32_t absDc = 0;
  for( int k = 0; k < 4; k++ )
  {
    const __m256i x    = xHadamardHalfStage16_AVX2( xHadamardLaneStage16_AVX2( r[k] ) );
    const __m256i sums = _mm256_madd_epi16( x, plus );
    const __m256i difs = _mm256_madd_epi16( x, plusMinus );
    absSum = _mm256_add_epi32( absSum, xMaxAbsOfNeighbours32_AVX2( sums ) );
    absSum = _mm256_add_epi32( absSum, xMaxAbsOfNeighbours32_AVX2( difs ) );
    if( k == 0 )
    {
      absDc = abs( _mm_cvtsi128_si32( _mm256_castsi256_si128( sums ) ) + _mm256_extract_epi32( sums, 1 ) );
    }
  }

  sad = xSumOfValues32_AVX2( absSum );
  sad -= absDc;
  sad += absDc >> 2;
  sad = ( ( sad + 2 ) >> 2 );   // as xCalcHAD8x8_SSE

#endif //USE_AVX2

  return sad;
}

template<X86_VEXT vext >
Distortion RdCost::xGetHAD2SADs_SIMD( const DistParam &rcDtParam )
{
  Distortion distHad = xGetHADs_SIMD<vext, false>( rcDtParam );
  Distortion distSad = 0;

  {
    const short* pSrc1   = (const short*)rcDtParam.org.buf;
    const short* pSrc2   = (const short*)rcDtParam.cur.buf;
    const int iStrideSrc1 = rcDtParam.org.stride<<2;
    const int iStrideSrc2 = rcDtParam.cur.stride<<2;
    const int  iRows      = rcDtParam.org.height>>2;
    const int  iCols      = rcDtParam.org.width<<2;

    uint32_t uiSum = 0;
    CHECKD( (rcDtParam.org.width != rcDtParam.org.stride) || (rcDtParam.cur.stride != rcDtParam.org.stride) , "this functions assumes compact, aligned buffering");

#ifdef USE_AVX2 
    if( vext >= AVX2 )
    {
      // Do for width that multiple of 16
      __m256i vone   = _mm256_set1_epi16( 1 );
      __m256i vsum32 = _mm256_setzero_si256();
      for( int iY = 0; iY < iRows; iY++ )
      {
        __m256i vsum16 = _mm256_setzero_si256();
        for( int iX = 0; iX < iCols; iX+=16 )
        {
          __m256i vsrc1 = _mm256_load_si256( ( __m256i* )( &pSrc1[iX] ) );
          __m256i vsrc2 = _mm256_load_si256( ( __m256i* )( &pSrc2[iX] ) );
          vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );
        }
        __m256i vsumtemp = _mm256_madd_epi16( vsum16, vone );
        vsum32 = _mm256_add_epi32( vsum32, vsumtemp );
        pSrc1   += iStrideSrc1;
        pSrc2   += iStrideSrc2;
      }
      vsum32 = _mm256_hadd_epi32( vsum32, vone );
      vsum32 = _mm256_hadd_epi32( vsum32, vone );
      uiSum =  _mm_cvtsi128_si32( _mm256_castsi256_si128( vsum32 ) ) + _mm_cvtsi128_si32( _mm256_extracti128_si256( vsum32, 1 ) );
    }
    else
#endif
    {
      // For width that multiple of 8
      __m128i vone = _mm_set1_epi16( 1 );
      __m128i vsum32 = _mm_setzero_si128();
      for( int iY = 0; iY < iRows; iY++ )
      {
        __m128i vsum16 = _mm_setzero_si128();
        for( int iX = 0; iX < iCols; iX+=8 )
        {
          __m128i vsrc1 = _mm_load_si128( ( const __m128i* )( &pSrc1[iX] ) );
          __m128i vsrc2 = _mm_load_si128( ( const __m128i* )( &pSrc2[iX] ) );
          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
        }
        __m128i vsumtemp = _mm_madd_epi16( vsum16, vone );
        vsum32 = _mm_add_epi32( vsum32, vsumtemp );
        pSrc1   += iStrideSrc1;
        pSrc2   += iStrideSrc2;
      }
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      uiSum =  _mm_cvtsi128_si32( vsum32 );
    }
    distSad = uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
  }

  return std::min( distHad, 2*distSad);
}

template<X86_VEXT vext> 
Distortion RdCost::xGetSADwMask_SIMD(const DistParam &rcDtParam)
{
  if (rcDtParam.org.width < 4 || rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
    return RdCost::xGetSADwMask(rcDtParam);

  const short *src1       = (const short *) rcDtParam.org.buf;
  const short *src2       = (const short *) rcDtParam.cur.buf;
  const short *weightMask = (const short *) rcDtParam.mask;
  int          rows       = rcDtParam.org.height;
  int          cols       = rcDtParam.org.width;
  int          subShift   = rcDtParam.subShift;
  int          subStep    = (1 << subShift);
  const int    strideSrc1 = rcDtParam.org.stride * subStep;
  const int    strideSrc2 = rcDtParam.cur.stride * subStep;
  const int    strideMask = rcDtParam.maskStride * subStep;

  Distortion sum = 0;
  if (vext >= AVX2 && (cols & 15) == 0)
  {
#if defined( USE_AVX2 )
    // Do for width that multiple of 16
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < rows; y += subStep)
    {
      for (int x = 0; x < cols; x += 16)
      {
        __m256i vsrc1 = _mm256_loadu_si256((__m256i *) (&src1[x]));
        __m256i vsrc2 = _mm256_loadu_si256((__m256i *) (&src2[x]));
        __m256i vmask;

        if (rcDtParam.stepX == -1)
        {
          vmask                      = _mm256_loadu_si256((__m256i *) ((&weightMask[x]) - (x << 1) - (16 - 1)));
          const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2,
                                                       5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vmask                      = _mm256_shuffle_epi8(vmask, shuffle_mask);
          vmask                      = _mm256_permute4x64_epi64(vmask, _MM_SHUFFLE(1, 0, 3, 2));
        }
        else
        {
          vmask                      = _mm256_loadu_si256( ( __m256i * ) ( &weightMask[x] ) );
        }

        vsum32 = _mm256_add_epi32(vsum32, _mm256_madd_epi16(vmask, _mm256_abs_epi16(_mm256_sub_epi16(vsrc1, vsrc2))));
      }
      src1       += strideSrc1;
      src2       += strideSrc2;
      weightMask += strideMask;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32))
           + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
#endif
  }
  else
  {
    // Do with step of 8
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < rows; y += subStep)
    {
      for (int x = 0; x < cols; x += 8)
      {
        __m128i vsrc1 = _mm_loadu_si128((const __m128i *) (&src1[x]));
        __m128i vsrc2 = _mm_loadu_si128((const __m128i *) (&src2[x]));
        __m128i vmask;
        if (rcDtParam.stepX == -1)
        {
          vmask                      = _mm_loadu_si128((__m128i *) ((&weightMask[x]) - (x << 1) - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vmask                      = _mm_shuffle_epi8(vmask, shuffle_mask);
        }
        else
        {
          vmask = _mm_loadu_si128((const __m128i *) (&weightMask[x]));
        }
        vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vmask, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2))));
      }
      src1 += strideSrc1;
      src2 += strideSrc2;
      weightMask += strideMask;
    }
    vsum32 = _mm_hadd_epi32(vsum32, vzero);
    vsum32 = _mm_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(vsum32);
  }
  sum <<= subShift;
  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template<X86_VEXT vext, bool fastHad>
Distortion RdCost::xGetHADs_SIMD( const DistParam &rcDtParam )
{
  const Pel*  piOrg = rcDtParam.org.buf;
  const Pel*  piCur = rcDtParam.cur.buf;
  const int iRows = rcDtParam.org.height;
  const int iCols = rcDtParam.org.width;
  const int iStrideCur = rcDtParam.cur.stride;
  const int iStrideOrg = rcDtParam.org.stride;
  const int iBitDepth  = rcDtParam.bitDepth;

  int  x, y;
  Distortion uiSum = 0;

  if( iCols > iRows && ( iCols & 15 ) == 0 && ( iRows & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        if( vext >= AVX2 )
          uiSum += xCalcHAD16x8_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
        else
          uiSum += xCalcHAD16x8_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 8*iStrideOrg;
      piCur += 8*iStrideCur;
    }
  }
  else if( iCols < iRows && ( iRows & 15 ) == 0 && ( iCols & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        if( vext >= AVX2 )
          uiSum += xCalcHAD8x16_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
        else
          uiSum += xCalcHAD8x16_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 16*iStrideOrg;
      piCur += 16*iStrideCur;
    }
  }
  else if( iCols > iRows && ( iCols & 7 ) == 0 && ( iRows & 3 ) == 0 )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        if( vext >= AVX2 && iBitDepth <= 10 )
          uiSum += xCalcHAD8x4_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
        else
          uiSum += xCalcHAD8x4_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 4*iStrideOrg;
      piCur += 4*iStrideCur;
    }
  }
  else if( iCols < iRows && ( iRows & 7 ) == 0 && ( iCols & 3 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        if( vext >= AVX2 && iBitDepth <= 10 )
          uiSum += xCalcHAD4x8_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
        else
          uiSum += xCalcHAD4x8_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 8*iStrideOrg;
      piCur += 8*iStrideCur;
    }
  }
  else if( fastHad && vext >= AVX2 && ( ( ( iRows | iCols ) & 31 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y < iRows; y += 32 )
    {
      for( x = 0; x < iCols; x += 32 )
      {
        uiSum += xCalcHAD32x32_fast_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 32 * iStrideOrg;
      piCur += 32 * iStrideCur;
    }
  }
  else if( fastHad && ( ( ( iRows | iCols ) & 31 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x16_fast_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
    }
  }
  else if( vext >= AVX2 && ( ( ( iRows | iCols ) & 15 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x16_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 16*iStrideOrg;
      piCur += 16*iStrideCur;
    }
  }
  else if( ( ( ( iRows | iCols ) & 7 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y<iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        if( vext >= AVX2 && iBitDepth <= 10 )
          uiSum += xCalcHAD8x8_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
        else
          uiSum += xCalcHAD8x8_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 8*iStrideOrg;
      piCur += 8*iStrideCur;
    }
  }
  else if( ( iRows % 4 == 0 ) && ( iCols % 4 == 0 ) )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHAD4x4_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4*iStrideOrg;
      piCur += 4*iStrideCur;
    }
  }
  else if( ( iRows % 2 == 0 ) && ( iCols % 2 == 0 ) )
  {
    for( y = 0; y < iRows; y += 2 )
    {
      for( x = 0; x < iCols; x += 2 )
      {
        uiSum += xCalcHADs2x2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 2*iStrideOrg;
      piCur += 2*iStrideCur;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }

  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

inline Distortion getWeightedMSE_SIMD(const Pel org, const Pel cur, const int64_t fixedPTweight, unsigned uiShift)
{
  const Intermediate_Int iTemp = org - cur;
  return Intermediate_Int((fixedPTweight*(iTemp*iTemp) + (1 << 15)) >> uiShift);
}

template<X86_VEXT vext>
static Distortion fixWeightedSSE_SIMD( const DistParam& rcDtParam, uint32_t fixedPTweight )
{
        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iCols = rcDtParam.org.width;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;

  Distortion uiSum       = 0;
  const uint32_t uiShift = 16 + ( DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth ) << 1 );
  const __m128i vShift   = _mm_cvtsi32_si128(uiShift);

  if( ( iCols & 3 ) == 0 )
  {
    const __m128i xfxdw = _mm_set1_epi32 ( fixedPTweight );
    const __m128i xoffs = _mm_set1_epi64x( 1 << 15 );
          __m128i xsum  = _mm_setzero_si128();

    for( ; iRows != 0; iRows-- )
    {
      for( int n = 0; n < iCols; n += 4 )
      {
        __m128i xorg = _vv_loadl_epi64( ( const __m128i* ) &piOrg[n] );
        __m128i xcur = _vv_loadl_epi64( ( const __m128i* ) &piCur[n] );

        xcur = _mm_sub_epi16     ( xorg, xcur );

        const __m128i
        xmlo = _mm_mullo_epi16   ( xcur, xcur ),
        xmhi = _mm_mulhi_epi16   ( xcur, xcur );

        __m128i
        xmul = _mm_unpacklo_epi16( xmlo, xmhi );
        __m128i
        xtmp = _mm_mul_epi32     ( xmul, xfxdw );
        xtmp = _mm_add_epi64     ( xtmp, xoffs );
        xtmp = _mm_srl_epi64     ( xtmp, vShift );
        xsum = _mm_add_epi64     ( xsum, xtmp );

        xmul = _mm_shuffle_epi32 ( xmul, 1 + 0 + 48 + 128 );
        xtmp = _mm_mul_epi32     ( xmul, xfxdw );
        xtmp = _mm_add_epi64     ( xtmp, xoffs );
        xtmp = _mm_srl_epi64     ( xtmp, vShift );
        xsum = _mm_add_epi64     ( xsum, xtmp );
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }

    uiSum += _mm_extract_epi64( xsum, 0 );
    uiSum += _mm_extract_epi64( xsum, 1 );

    return uiSum;
  }
  else if( iCols == 2 )
  {
    for( ; iRows != 0; iRows-- )
    {
      for( int n = 0; n < iCols; n += 2 )
      {
        uiSum += getWeightedMSE_SIMD( piOrg[n    ], piCur[n    ], fixedPTweight, uiShift );
        uiSum += getWeightedMSE_SIMD( piOrg[n + 1], piCur[n + 1], fixedPTweight, uiShift );
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }

    return uiSum;
  }
  else
  {
    for( ; iRows != 0; iRows-- )
    {
      for( int n = 0; n < iCols; n++ )
      {
        uiSum += getWeightedMSE_SIMD( piOrg[n], piCur[n], fixedPTweight, uiShift );
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }

    return uiSum;
  }

  return 0;
}


template <X86_VEXT vext, bool isCalCentrePos>
void xGetSADX5_8xN_SIMDImp(const DistParam& rcDtParam, Distortion* cost) {
  int i;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf - 4;
  int height = rcDtParam.org.height;
  int iSubShift = rcDtParam.subShift;
  int iSubStep = (1 << iSubShift);
  ptrdiff_t iStrideCur = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t iStrideOrg = rcDtParam.org.stride * iSubStep;

  __m128i sum0 = _mm_setzero_si128();
  __m128i sum1 = _mm_setzero_si128();
  __m128i sum2 = _mm_setzero_si128();
  __m128i sum3 = _mm_setzero_si128();
  __m128i sum4 = _mm_setzero_si128();

  __m128i vone = _mm_set1_epi16(1);
  for (i = 0; i < height; i += iSubStep) {
    __m128i s0 = _mm_loadu_si128((__m128i*)piOrg);
    __m128i s1 = _mm_loadu_si128((__m128i*)piCur);
    __m128i s2 = _vv_loadl_epi64((__m128i*)(piOrg + 8));
    __m128i s3 = _vv_loadl_epi64((__m128i*)(piCur + 8));

    __m128i org0, org1, org2, org3, org4;
    org0 = s0;
    org1 = _mm_alignr_epi8(s2, s0, 2);
    if (isCalCentrePos) org2 = _mm_alignr_epi8(s2, s0, 4);
    org3 = _mm_alignr_epi8(s2, s0, 6);
    org4 = _mm_alignr_epi8(s2, s0, 8);

    __m128i cur0, cur1, cur2, cur3, cur4;
    cur4 = s1;
    cur0 = _mm_alignr_epi8(s3, s1, 8);
    cur1 = _mm_alignr_epi8(s3, s1, 6);
    if (isCalCentrePos) cur2 = _mm_alignr_epi8(s3, s1, 4);
    cur3 = _mm_alignr_epi8(s3, s1, 2);

    __m128i diff0, diff1, diff2, diff3, diff4;
    diff0 = _mm_sub_epi16(org0, cur0);
    diff1 = _mm_sub_epi16(org1, cur1);
    if (isCalCentrePos) diff2 = _mm_sub_epi16(org2, cur2);
    diff3 = _mm_sub_epi16(org3, cur3);
    diff4 = _mm_sub_epi16(org4, cur4);

    diff0 = _mm_abs_epi16(diff0);
    diff1 = _mm_abs_epi16(diff1);
    if (isCalCentrePos) diff2 = _mm_abs_epi16(diff2);
    diff3 = _mm_abs_epi16(diff3);
    diff4 = _mm_abs_epi16(diff4);

    sum0 = _mm_add_epi16(sum0, diff0);
    sum1 = _mm_add_epi16(sum1, diff1);
    if (isCalCentrePos) sum2 = _mm_add_epi32(sum2, diff2);
    sum3 = _mm_add_epi16(sum3, diff3);
    sum4 = _mm_add_epi16(sum4, diff4);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  sum0 = _mm_madd_epi16( sum0, vone );
  sum1 = _mm_madd_epi16( sum1, vone );
  if( isCalCentrePos ) sum2 = _mm_madd_epi16( sum2, vone );
  sum3 = _mm_madd_epi16( sum3, vone );
  sum4 = _mm_madd_epi16( sum4, vone );

  sum0 = _mm_hadd_epi32(sum0, sum1);
  sum3 = _mm_hadd_epi32(sum3, sum4);
  if (isCalCentrePos) sum2 = _mm_hadd_epi32(sum2, sum2);

  sum0 = _mm_hadd_epi32(sum0, sum3);
  if (isCalCentrePos) sum2 = _mm_hadd_epi32(sum2, sum2);

  const __m128i vSubShift = _mm_cvtsi32_si128(iSubShift);
  sum0 = _mm_sll_epi32(sum0, vSubShift);
  if (isCalCentrePos) sum2 = _mm_sll_epi32(sum2, vSubShift);

  sum0 = _mm_srl_epi32(sum0, _mm_cvtsi32_si128(1 + (DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth))));
  if (isCalCentrePos) sum2 = _mm_srl_epi32(sum2, _mm_cvtsi32_si128(1 + (DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth))));

  _mm_storeu_si128( ( __m128i* ) &cost[0], _mm_unpacklo_epi32( sum0, _mm_setzero_si128() ) );
  if (isCalCentrePos) cost[2] = (_mm_cvtsi128_si32(sum2));
  _mm_storeu_si128( ( __m128i* ) &cost[3], _mm_unpackhi_epi32( sum0, _mm_setzero_si128() ) );
}

template <X86_VEXT vext>
void RdCost::xGetSADX5_8xN_SIMD(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos) {
  if( rcDtParam.bitDepth > 10 ){
    RdCost::xGetSAD8X5( rcDtParam, cost, isCalCentrePos );
    return;
  }
  
  if (isCalCentrePos)
    xGetSADX5_8xN_SIMDImp<vext, true>(rcDtParam, cost);
  else
    xGetSADX5_8xN_SIMDImp<vext, false>(rcDtParam, cost);
}

template <X86_VEXT vext, bool isCalCentrePos>
void xGetSADX5_16xN_SIMDImp_X86(const DistParam& rcDtParam, Distortion* cost) {
  int i, j;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf - 4;
  int height = rcDtParam.org.height;
  int iSubShift = rcDtParam.subShift;
  int iSubStep = (1 << iSubShift);
  ptrdiff_t iStrideCur = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t iStrideOrg = rcDtParam.org.stride * iSubStep;

#  ifdef USE_AVX2
  if (vext >= AVX2) {
    // sum of 8 unsigned 10-bit ints (0-1023) can maximally be 3 + 10 bits, i.e. fits into 16 bit

    __m256i sum0 = _mm256_setzero_si256();
    __m256i sum1 = _mm256_setzero_si256();
    __m256i sum2 = _mm256_setzero_si256();
    __m256i sum3 = _mm256_setzero_si256();
    __m256i sum4 = _mm256_setzero_si256();

    __m256i vone = _mm256_set1_epi16(1);

    for (int i = 0; i < ( height >> 3 ); i++) {
      __m256i s0 = _mm256_loadu_si256((__m256i*)piOrg);
      __m256i s1 = _mm256_loadu_si256((__m256i*)piCur);
      __m256i s2 = _mm256_castsi128_si256(_vv_loadl_epi64((__m128i*)(piOrg + 16)));
      __m256i s3 = _mm256_castsi128_si256(_vv_loadl_epi64((__m128i*)(piCur + 16)));
      s2 = _mm256_permute2x128_si256(s0, s2, 0x21);
      s3 = _mm256_permute2x128_si256(s1, s3, 0x21);

      piOrg += iStrideOrg;
      piCur += iStrideCur;

      __m256i org0, org1, org2, org3, org4;
      org0 = s0;
      org1 = _mm256_alignr_epi8(s2, s0, 2);
      if (isCalCentrePos) org2 = _mm256_alignr_epi8(s2, s0, 4);
      org3 = _mm256_alignr_epi8(s2, s0, 6);
      org4 = _mm256_alignr_epi8(s2, s0, 8);

      __m256i cur0, cur1, cur2, cur3, cur4;
      cur4 = s1;
      cur0 = _mm256_alignr_epi8(s3, s1, 8);
      cur1 = _mm256_alignr_epi8(s3, s1, 6);
      if (isCalCentrePos) cur2 = _mm256_alignr_epi8(s3, s1, 4);
      cur3 = _mm256_alignr_epi8(s3, s1, 2);

      __m256i diff0, diff1, diff2, diff3, diff4;
      diff0 = _mm256_sub_epi16(org0, cur0);
      diff1 = _mm256_sub_epi16(org1, cur1);
      if (isCalCentrePos) diff2 = _mm256_sub_epi16(org2, cur2);
      diff3 = _mm256_sub_epi16(org3, cur3);
      diff4 = _mm256_sub_epi16(org4, cur4);

      diff0 = _mm256_abs_epi16( diff0 );
      diff1 = _mm256_abs_epi16( diff1 );
      if( isCalCentrePos ) diff2 = _mm256_abs_epi16( diff2 );
      diff3 = _mm256_abs_epi16( diff3 );
      diff4 = _mm256_abs_epi16( diff4 );

      sum0 = _mm256_add_epi16( diff0, sum0 );
      sum1 = _mm256_add_epi16( diff1, sum1 );
      if( isCalCentrePos ) sum2 = _mm256_add_epi16( diff2, sum2 );
      sum3 = _mm256_add_epi16( diff3, sum3 );
      sum4 = _mm256_add_epi16( diff4, sum4 );

      s0 = _mm256_loadu_si256((__m256i*)piOrg);
      s1 = _mm256_loadu_si256((__m256i*)piCur);
      s2 = _mm256_castsi128_si256(_vv_loadl_epi64((__m128i*)(piOrg + 16)));
      s3 = _mm256_castsi128_si256(_vv_loadl_epi64((__m128i*)(piCur + 16)));
      s2 = _mm256_permute2x128_si256(s0, s2, 0x21);
      s3 = _mm256_permute2x128_si256(s1, s3, 0x21);

      piOrg += iStrideOrg;
      piCur += iStrideCur;

      org0 = s0;
      org1 = _mm256_alignr_epi8(s2, s0, 2);
      if (isCalCentrePos) org2 = _mm256_alignr_epi8(s2, s0, 4);
      org3 = _mm256_alignr_epi8(s2, s0, 6);
      org4 = _mm256_alignr_epi8(s2, s0, 8);

      cur4 = s1;
      cur0 = _mm256_alignr_epi8(s3, s1, 8);
      cur1 = _mm256_alignr_epi8(s3, s1, 6);
      if (isCalCentrePos) cur2 = _mm256_alignr_epi8(s3, s1, 4);
      cur3 = _mm256_alignr_epi8(s3, s1, 2);

      diff0 = _mm256_sub_epi16(org0, cur0);
      diff1 = _mm256_sub_epi16(org1, cur1);
      if (isCalCentrePos) diff2 = _mm256_sub_epi16(org2, cur2);
      diff3 = _mm256_sub_epi16(org3, cur3);
      diff4 = _mm256_sub_epi16(org4, cur4);

      diff0 = _mm256_abs_epi16(diff0);
      diff1 = _mm256_abs_epi16(diff1);
      if (isCalCentrePos) diff2 = _mm256_abs_epi16(diff2);
      diff3 = _mm256_abs_epi16(diff3);
      diff4 = _mm256_abs_epi16(diff4);

      sum0 = _mm256_add_epi16(diff0, sum0);
      sum1 = _mm256_add_epi16(diff1, sum1);
      if (isCalCentrePos) sum2 = _mm256_add_epi16(diff2, sum2);
      sum3 = _mm256_add_epi16(diff3, sum3);
      sum4 = _mm256_add_epi16(diff4, sum4);

      s0 = _mm256_loadu_si256((__m256i*)piOrg);
      s1 = _mm256_loadu_si256((__m256i*)piCur);
      s2 = _mm256_castsi128_si256(_vv_loadl_epi64((__m128i*)(piOrg + 16)));
      s3 = _mm256_castsi128_si256(_vv_loadl_epi64((__m128i*)(piCur + 16)));
      s2 = _mm256_permute2x128_si256(s0, s2, 0x21);
      s3 = _mm256_permute2x128_si256(s1, s3, 0x21);

      piOrg += iStrideOrg;
      piCur += iStrideCur;

      org0 = s0;
      org1 = _mm256_alignr_epi8(s2, s0, 2);
      if (isCalCentrePos) org2 = _mm256_alignr_epi8(s2, s0, 4);
      org3 = _mm256_alignr_epi8(s2, s0, 6);
      org4 = _mm256_alignr_epi8(s2, s0, 8);

      cur4 = s1;
      cur0 = _mm256_alignr_epi8(s3, s1, 8);
      cur1 = _mm256_alignr_epi8(s3, s1, 6);
      if (isCalCentrePos) cur2 = _mm256_alignr_epi8(s3, s1, 4);
      cur3 = _mm256_alignr_epi8(s3, s1, 2);

      diff0 = _mm256_sub_epi16(org0, cur0);
      diff1 = _mm256_sub_epi16(org1, cur1);
      if (isCalCentrePos) diff2 = _mm256_sub_epi16(org2, cur2);
      diff3 = _mm256_sub_epi16(org3, cur3);
      diff4 = _mm256_sub_epi16(org4, cur4);

      diff0 = _mm256_abs_epi16(diff0);
      diff1 = _mm256_abs_epi16(diff1);
      if (isCalCentrePos) diff2 = _mm256_abs_epi16(diff2);
      diff3 = _mm256_abs_epi16(diff3);
      diff4 = _mm256_abs_epi16(diff4);

      sum0 = _mm256_add_epi16( diff0, sum0 );
      sum1 = _mm256_add_epi16( diff1, sum1 );
      if( isCalCentrePos ) sum2 = _mm256_add_epi16( diff2, sum2 );
      sum3 = _mm256_add_epi16( diff3, sum3 );
      sum4 = _mm256_add_epi16( diff4, sum4 );

      s0 = _mm256_loadu_si256((__m256i*)piOrg);
      s1 = _mm256_loadu_si256((__m256i*)piCur);
      s2 = _mm256_castsi128_si256(_vv_loadl_epi64((__m128i*)(piOrg + 16)));
      s3 = _mm256_castsi128_si256(_vv_loadl_epi64((__m128i*)(piCur + 16)));
      s2 = _mm256_permute2x128_si256(s0, s2, 0x21);
      s3 = _mm256_permute2x128_si256(s1, s3, 0x21);

      piOrg += iStrideOrg;
      piCur += iStrideCur;

      org0 = s0;
      org1 = _mm256_alignr_epi8(s2, s0, 2);
      if (isCalCentrePos) org2 = _mm256_alignr_epi8(s2, s0, 4);
      org3 = _mm256_alignr_epi8(s2, s0, 6);
      org4 = _mm256_alignr_epi8(s2, s0, 8);

      cur4 = s1;
      cur0 = _mm256_alignr_epi8(s3, s1, 8);
      cur1 = _mm256_alignr_epi8(s3, s1, 6);
      if (isCalCentrePos) cur2 = _mm256_alignr_epi8(s3, s1, 4);
      cur3 = _mm256_alignr_epi8(s3, s1, 2);

      diff0 = _mm256_sub_epi16(org0, cur0);
      diff1 = _mm256_sub_epi16(org1, cur1);
      if (isCalCentrePos) diff2 = _mm256_sub_epi16(org2, cur2);
      diff3 = _mm256_sub_epi16(org3, cur3);
      diff4 = _mm256_sub_epi16(org4, cur4);

      diff0 = _mm256_abs_epi16(diff0);
      diff1 = _mm256_abs_epi16(diff1);
      if (isCalCentrePos) diff2 = _mm256_abs_epi16(diff2);
      diff3 = _mm256_abs_epi16(diff3);
      diff4 = _mm256_abs_epi16(diff4);

      sum0 = _mm256_add_epi16(diff0, sum0);
      sum1 = _mm256_add_epi16(diff1, sum1);
      if (isCalCentrePos) sum2 = _mm256_add_epi16(diff2, sum2);
      sum3 = _mm256_add_epi16(diff3, sum3);
      sum4 = _mm256_add_epi16(diff4, sum4);
    }

    sum0 = _mm256_madd_epi16( sum0, vone );
    sum1 = _mm256_madd_epi16( sum1, vone );
    if( isCalCentrePos ) sum2 = _mm256_madd_epi16( sum2, vone );
    sum3 = _mm256_madd_epi16( sum3, vone );
    sum4 = _mm256_madd_epi16( sum4, vone );

    sum0 = _mm256_hadd_epi32(sum0, sum1);
    sum3 = _mm256_hadd_epi32(sum3, sum4);
    if (isCalCentrePos) sum2 = _mm256_hadd_epi32(sum2, sum2);

    sum0 = _mm256_hadd_epi32(sum0, sum3);
    if (isCalCentrePos) sum2 = _mm256_hadd_epi32(sum2, sum2);

    __m128i sum0134 = _mm_add_epi32(_mm256_castsi256_si128(sum0), _mm256_extracti128_si256(sum0, 1));

    sum0134 = _mm_sll_epi32(sum0134, _mm_cvtsi32_si128(iSubShift));

    sum0134 = _mm_srl_epi32(sum0134, _mm_cvtsi32_si128(1 + (DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth))));

    _mm_storeu_si128( ( __m128i* ) &cost[0], _mm_unpacklo_epi32( sum0134, _mm_setzero_si128() ) );
    if (isCalCentrePos) {
      int tmp = _mm_cvtsi128_si32(_mm256_castsi256_si128(sum2)) + _mm256_extract_epi32(sum2, 4);
      tmp <<= iSubShift;
      tmp >>= (1 + (DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth)));
      cost[2] = tmp;
    }
    _mm_storeu_si128( ( __m128i* ) &cost[3], _mm_unpackhi_epi32( sum0134, _mm_setzero_si128() ) );
  }
  else
#  endif
  {
    // sum of 16 unsigned 10-bit ints (0-1023) can maximally be 4 + 10 bits, i.e. fits into 16 bit

    __m128i sum0 = _mm_setzero_si128();
    __m128i sum1 = _mm_setzero_si128();
    __m128i sum2 = _mm_setzero_si128();
    __m128i sum3 = _mm_setzero_si128();
    __m128i sum4 = _mm_setzero_si128();

    __m128i vone = _mm_set1_epi16(1);
    for (i = 0; i < height; i += iSubStep) {
      for (j = 0; j < 16; j += 8) {
        __m128i s0 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(piOrg + j + 0));
        __m128i s1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(piCur + j + 0));
        __m128i s2 = _vv_loadl_epi64(reinterpret_cast<const __m128i*>(piOrg + j + 8));
        __m128i s3 = _vv_loadl_epi64(reinterpret_cast<const __m128i*>(piCur + j + 8));

        __m128i org0, org1, org2, org3, org4;
        org0 = s0;
        org1 = _mm_alignr_epi8(s2, s0, 2);
        if (isCalCentrePos) org2 = _mm_alignr_epi8(s2, s0, 4);
        org3 = _mm_alignr_epi8(s2, s0, 6);
        org4 = _mm_alignr_epi8(s2, s0, 8);

        __m128i cur0, cur1, cur2, cur3, cur4;
        cur4 = s1;
        cur0 = _mm_alignr_epi8(s3, s1, 8);
        cur1 = _mm_alignr_epi8(s3, s1, 6);
        if (isCalCentrePos) cur2 = _mm_alignr_epi8(s3, s1, 4);
        cur3 = _mm_alignr_epi8(s3, s1, 2);

        __m128i diff0, diff1, diff2, diff3, diff4;
        diff0 = _mm_sub_epi16(org0, cur0);
        diff1 = _mm_sub_epi16(org1, cur1);
        if (isCalCentrePos) diff2 = _mm_sub_epi16(org2, cur2);
        diff3 = _mm_sub_epi16(org3, cur3);
        diff4 = _mm_sub_epi16(org4, cur4);

        diff0 = _mm_abs_epi16(diff0);
        diff1 = _mm_abs_epi16(diff1);
        if (isCalCentrePos) diff2 = _mm_abs_epi16(diff2);
        diff3 = _mm_abs_epi16(diff3);
        diff4 = _mm_abs_epi16(diff4);

        sum0 = _mm_add_epi16(sum0, diff0);
        sum1 = _mm_add_epi16(sum1, diff1);
        if (isCalCentrePos) sum2 = _mm_add_epi16(sum2, diff2);
        sum3 = _mm_add_epi16(sum3, diff3);
        sum4 = _mm_add_epi16(sum4, diff4);
      }

      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }

    sum0 = _mm_madd_epi16( sum0, vone );
    sum1 = _mm_madd_epi16( sum1, vone );
    if( isCalCentrePos ) sum2 = _mm_madd_epi16( sum2, vone );
    sum3 = _mm_madd_epi16( sum3, vone );
    sum4 = _mm_madd_epi16( sum4, vone );

    sum0 = _mm_hadd_epi32(sum0, sum1);
    sum3 = _mm_hadd_epi32(sum3, sum4);
    if (isCalCentrePos) sum2 = _mm_hadd_epi32(sum2, sum2);

    sum0 = _mm_hadd_epi32(sum0, sum3);
    if (isCalCentrePos) sum2 = _mm_hadd_epi32(sum2, sum2);

    const __m128i vSubShift = _mm_cvtsi32_si128(iSubShift);
    sum0 = _mm_sll_epi32(sum0, vSubShift);
    if (isCalCentrePos) sum2 = _mm_sll_epi32(sum2, vSubShift);

    sum0 = _mm_srl_epi32(sum0, _mm_cvtsi32_si128(1 + (DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth))));
    if (isCalCentrePos) sum2 = _mm_srl_epi32(sum2, _mm_cvtsi32_si128(1 + (DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth))));

    _mm_storeu_si128( ( __m128i* ) &cost[0], _mm_unpacklo_epi32( sum0, _mm_setzero_si128() ) );
    if (isCalCentrePos) cost[2] = (_mm_cvtsi128_si32(sum2));
    _mm_storeu_si128( ( __m128i* ) &cost[3], _mm_unpackhi_epi32( sum0, _mm_setzero_si128() ) );
  }
}

template <X86_VEXT vext>
void RdCost::xGetSADX5_16xN_SIMD_X86(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos) {
  if( rcDtParam.bitDepth > 10 ){
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }
  
  if (isCalCentrePos)
    xGetSADX5_16xN_SIMDImp_X86<vext, true>(rcDtParam, cost);
  else
    xGetSADX5_16xN_SIMDImp_X86<vext, false>(rcDtParam, cost);
}

// ---------------------------------------------------------------------------------------------------------------------
// IBC block vector bits (RdCost::getBitsMultiplePredsIBC) for a line of vectors ( x0 + i * xStep, y ). The bits of a
// vector difference component val are 2 * floorLog2( u ) + 1 with u = val > 0 ? 2 * val : 1 - 2 * val. floorLog2 is
// read from the exponent of the float conversion of u, which is exact for u < 2^24, i.e. for differences of up to 22
// bits, more than the 18 bits of an Mv component.
// ---------------------------------------------------------------------------------------------------------------------

// internal linkage: this header is compiled once per instruction set, and the copies must not be merged by the linker
namespace
{
// the 32-bit lane operations of the block vector bits, for 4 and 8 lanes
struct BvOps128
{
  typedef __m128i V;
  static const int lanes = 4;
  static V    set1  ( int v )                     { return _mm_set1_epi32( v ); }
  static V    lane  ()                            { return _mm_setr_epi32( 0, 1, 2, 3 ); }
  static V    add   ( V a, V b )                  { return _mm_add_epi32( a, b ); }
  static V    sub   ( V a, V b )                  { return _mm_sub_epi32( a, b ); }
  static V    mul   ( V a, V b )                  { return _mm_mullo_epi32( a, b ); }
  static V    abs   ( V a )                       { return _mm_abs_epi32( a ); }
  static V    gt    ( V a, V b )                  { return _mm_cmpgt_epi32( a, b ); }
  static V    eq    ( V a, V b )                  { return _mm_cmpeq_epi32( a, b ); }
  static V    andV  ( V a, V b )                  { return _mm_and_si128( a, b ); }
  static V    minU  ( V a, V b )                  { return _mm_min_epu32( a, b ); }
  static V    sra2  ( V a )                       { return _mm_srai_epi32( a, 2 ); }
  static V    select( V mask, V ifSet, V ifNot )  { return _mm_blendv_epi8( ifNot, ifSet, mask ); }
  static V    floatExponent( V u )                { return _mm_srli_epi32( _mm_castps_si128( _mm_cvtepi32_ps( u ) ), 23 ); }
  static void store ( unsigned* p, V a )          { _mm_storeu_si128( ( __m128i* ) p, a ); }
};

#if USE_AVX2
struct BvOps256
{
  typedef __m256i V;
  static const int lanes = 8;
  static V    set1  ( int v )                     { return _mm256_set1_epi32( v ); }
  static V    lane  ()                            { return _mm256_setr_epi32( 0, 1, 2, 3, 4, 5, 6, 7 ); }
  static V    add   ( V a, V b )                  { return _mm256_add_epi32( a, b ); }
  static V    sub   ( V a, V b )                  { return _mm256_sub_epi32( a, b ); }
  static V    mul   ( V a, V b )                  { return _mm256_mullo_epi32( a, b ); }
  static V    abs   ( V a )                       { return _mm256_abs_epi32( a ); }
  static V    gt    ( V a, V b )                  { return _mm256_cmpgt_epi32( a, b ); }
  static V    eq    ( V a, V b )                  { return _mm256_cmpeq_epi32( a, b ); }
  static V    andV  ( V a, V b )                  { return _mm256_and_si256( a, b ); }
  static V    minU  ( V a, V b )                  { return _mm256_min_epu32( a, b ); }
  static V    sra2  ( V a )                       { return _mm256_srai_epi32( a, 2 ); }
  static V    select( V mask, V ifSet, V ifNot )  { return _mm256_blendv_epi8( ifNot, ifSet, mask ); }
  static V    floatExponent( V u )                { return _mm256_srli_epi32( _mm256_castps_si256( _mm256_cvtepi32_ps( u ) ), 23 ); }
  static void store ( unsigned* p, V a )          { _mm256_storeu_si256( ( __m256i* ) p, a ); }
};
#endif

template<typename Ops>
static inline typename Ops::V bvComponentBits( const typename Ops::V val )
{
  const typename Ops::V twice = Ops::add( val, val );
  const typename Ops::V u     = Ops::select( Ops::gt( val, Ops::set1( 0 ) ), twice, Ops::sub( Ops::set1( 1 ), twice ) );
  const typename Ops::V log2  = Ops::sub( Ops::floatExponent( u ), Ops::set1( 127 ) );
  return Ops::add( Ops::add( log2, log2 ), Ops::set1( 1 ) );
}

// the bits of the cheaper of two predictors for the horizontal differences rmvH0 / rmvH1 of all lanes and a vertical
// difference that is the same in all lanes; the first predictor is used if its sum of absolute differences is smaller
template<typename Ops>
struct BvVerticalPart
{
  typedef typename Ops::V V;
  V absDiff[2];
  V bits[2];

  BvVerticalPart( const int rmvV0, const int rmvV1 )
  {
    const int rmvV[2] = { rmvV0, rmvV1 };
    for( int k = 0; k < 2; k++ )
    {
      absDiff[k] = Ops::set1( std::abs( rmvV[k] ) );
      bits   [k] = bvComponentBits<Ops>( Ops::set1( rmvV[k] ) );
    }
  }

  V selectBits( const V rmvH0, const V rmvH1 ) const
  {
    const V abs0  = Ops::add( Ops::abs( rmvH0 ), absDiff[0] );
    const V abs1  = Ops::add( Ops::abs( rmvH1 ), absDiff[1] );
    const V bits0 = Ops::add( bvComponentBits<Ops>( rmvH0 ), bits[0] );
    const V bits1 = Ops::add( bvComponentBits<Ops>( rmvH1 ), bits[1] );
    return Ops::select( Ops::gt( abs1, abs0 ), bits0, bits1 );
  }
};

template<typename Ops>
static void bvBitsLine( const Mv* predictors, const int x0, const int xStep, const int count, const int y, const bool useIMV, unsigned* bits )
{
  typedef typename Ops::V V;

  // with useIMV, vectors of whole samples ( x and y multiples of 4 ) may also be coded in sample precision
  const bool                samplePrecRow = useIMV && ( y & 3 ) == 0;
  const BvVerticalPart<Ops> vert          ( y - predictors[0].ver, y - predictors[1].ver );
  const BvVerticalPart<Ops> vertSamplePrec( ( y >> 2 ) - ( ( predictors[0].ver + 2 ) >> 2 ), ( y >> 2 ) - ( ( predictors[1].ver + 2 ) >> 2 ) );
  const V                   predH0        = Ops::set1( predictors[0].hor );
  const V                   predH1        = Ops::set1( predictors[1].hor );
  const V                   predH0Sample  = Ops::set1( ( predictors[0].hor + 2 ) >> 2 );
  const V                   predH1Sample  = Ops::set1( ( predictors[1].hor + 2 ) >> 2 );

  V       xs        = Ops::add( Ops::set1( x0 ), Ops::mul( Ops::lane(), Ops::set1( xStep ) ) );
  const V lanesStep = Ops::set1( Ops::lanes * xStep );

  for( int i = 0; i < count; i += Ops::lanes, xs = Ops::add( xs, lanesStep ) )
  {
    V result = vert.selectBits( Ops::sub( xs, predH0 ), Ops::sub( xs, predH1 ) );
    if( samplePrecRow )
    {
      const V xSample    = Ops::sra2( xs );
      const V sampleBits = vertSamplePrec.selectBits( Ops::sub( xSample, predH0Sample ), Ops::sub( xSample, predH1Sample ) );
      const V wholeX     = Ops::eq( Ops::andV( xs, Ops::set1( 3 ) ), Ops::set1( 0 ) );
      result = Ops::select( wholeX, Ops::minU( result, sampleBits ), result );
    }

    if( i + Ops::lanes <= count )
    {
      Ops::store( &bits[i], result );
    }
    else
    {
      unsigned last[Ops::lanes];
      Ops::store( last, result );
      std::copy( last, last + count - i, &bits[i] );
    }
  }
}
} // namespace

template<X86_VEXT vext>
static void bvBitsLine_SIMD( const Mv* predictors, const int x0, const int xStep, const int count, const int y, const bool useIMV, unsigned* bits )
{
#if USE_AVX2
  if( vext >= AVX2 )
  {
    bvBitsLine<BvOps256>( predictors, x0, xStep, count, y, useIMV, bits );
    return;
  }
#endif
  bvBitsLine<BvOps128>( predictors, x0, xStep, count, y, useIMV, bits );
}

template <X86_VEXT vext>
void RdCost::_initRdCostX86()
{
  /* SIMD SSE implementation shifts the final sum instead of every addend
   * resulting in slightly different result compared to the scalar impl. */

  m_afpDistortFunc[0][DF_SSE    ] = xGetSSE_SIMD<vext>;
  m_afpDistortFunc[0][DF_SSE2   ] = xGetSSE_NxN_SIMD<2,  vext>;
  m_afpDistortFunc[0][DF_SSE4   ] = xGetSSE_NxN_SIMD<4,  vext>;
  m_afpDistortFunc[0][DF_SSE8   ] = xGetSSE_NxN_SIMD<8,  vext>;
  m_afpDistortFunc[0][DF_SSE16  ] = xGetSSE_NxN_SIMD<16, vext>;
  m_afpDistortFunc[0][DF_SSE32  ] = xGetSSE_NxN_SIMD<32, vext>;
  m_afpDistortFunc[0][DF_SSE64  ] = xGetSSE_NxN_SIMD<64, vext>;
  m_afpDistortFunc[0][DF_SSE128]  = xGetSSE_NxN_SIMD<128, vext>;

  m_afpDistortFunc[0][DF_SAD    ] = xGetSAD_SIMD<vext>;
  m_afpDistortFunc[0][DF_SAD2   ] = xGetSAD2_SIMD<vext>;
  m_afpDistortFunc[0][DF_SAD4   ] = xGetSAD_NxN_SIMD<4,  vext>;
  m_afpDistortFunc[0][DF_SAD8   ] = xGetSAD_NxN_SIMD<8,  vext>;
  m_afpDistortFunc[0][DF_SAD16  ] = xGetSAD_NxN_SIMD<16, vext>;
  m_afpDistortFunc[0][DF_SAD32  ] = xGetSAD_NxN_SIMD<32, vext>;
  m_afpDistortFunc[0][DF_SAD64  ] = xGetSAD_NxN_SIMD<64, vext>;
  m_afpDistortFunc[0][DF_SAD128]  = xGetSAD_NxN_SIMD<128, vext>;

  m_afpDistortFunc[0][DF_HAD]     = RdCost::xGetHADs_SIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD2]    = RdCost::xGetHADs_SIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD4]    = RdCost::xGetHADs_SIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD8]    = RdCost::xGetHADs_SIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD16]   = RdCost::xGetHADs_SIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD32]   = RdCost::xGetHADs_SIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD64]   = RdCost::xGetHADs_SIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD128]  = RdCost::xGetHADs_SIMD<vext, false>;

  m_afpDistortFunc[0][DF_HAD_fast]     = RdCost::xGetHADs_SIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD2_fast]    = RdCost::xGetHADs_SIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD4_fast]    = RdCost::xGetHADs_SIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD8_fast]    = RdCost::xGetHADs_SIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD16_fast]   = RdCost::xGetHADs_SIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD32_fast]   = RdCost::xGetHADs_SIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD64_fast]   = RdCost::xGetHADs_SIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD128_fast]  = RdCost::xGetHADs_SIMD<vext, true>;

  m_afpDistortFunc[0][DF_HAD_2SAD ]     = RdCost::xGetHAD2SADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_SAD_WITH_MASK] = xGetSADwMask_SIMD<vext>;

  m_fxdWtdPredPtr = fixWeightedSSE_SIMD <vext>;
  m_bvBitsLineIBC = bvBitsLine_SIMD<vext>;

  m_afpDistortFuncX5[0] = xGetSADX5_8xN_SIMD <vext>;
  m_afpDistortFuncX5[1] = xGetSADX5_16xN_SIMD_X86<vext>;

#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    m_afpSadFourPositions[3] = xGetFourSads8_AVX2;
    m_afpSadFourPositions[4] = xGetFourSads_AVX2<16>;
    m_afpSadFourPositions[5] = xGetFourSads_AVX2<32>;
    m_afpSadFourPositions[6] = xGetFourSads_AVX2<64>;
    m_afpSadFourPositions[7] = xGetFourSads_AVX2<128>;
  }
#endif
}

template void RdCost::_initRdCostX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


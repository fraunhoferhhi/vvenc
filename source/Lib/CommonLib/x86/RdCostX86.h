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
    Sum = _mm256_hadd_epi32( Sum, Sum );
    Sum = _mm256_hadd_epi32( Sum, Sum );
    uiRet = ( _mm_cvtsi128_si32( _mm256_castsi256_si128( Sum ) ) + _mm_cvtsi128_si32( _mm256_castsi256_si128( _mm256_permute2x128_si256( Sum, Sum, 0x11 ) ) ) ) >> uiShift;
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

// The 2-wide case of RdCost::xGetSAD, including its early exit after each row: the absolute differences of four rows
// are summed per row in one vector, then the row sums are added and checked one after the other.
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
  const Distortion maxDist   = rcDtParam.maximumDistortionForEarlyExit;
  const __m128i   ones       = _mm_set1_epi16( 1 );

  Distortion sum = 0;
  int row = 0;
  for( ; row + 4 <= numRows; row += 4, org += 4 * orgStride, cur += 4 * curStride )
  {
    const __m128i absDiff = _mm_abs_epi16( _mm_sub_epi16( _vv_load2x4( org, orgStride ), _vv_load2x4( cur, curStride ) ) );
    int32_t rowSums[4];
    _mm_storeu_si128( ( __m128i* ) rowSums, _mm_madd_epi16( absDiff, ones ) );
    for( int r = 0; r < 4; r++ )
    {
      sum += rowSums[r];
      if( maxDist < ( sum >> distShift ) )
      {
        return sum >> distShift;
      }
    }
  }
  for( ; row < numRows; row++, org += orgStride, cur += curStride )
  {
    sum += abs( org[0] - cur[0] ) + abs( org[1] - cur[1] );
    if( maxDist < ( sum >> distShift ) )
    {
      return sum >> distShift;
    }
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
// +-2046. Each butterfly stage can double the values, so at most four stages run on 16-bit values
// (16 * 2047 = 32752) and the others on 32-bit values.

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

// The SATDs of two 8x8 blocks of residuals, one per 128-bit lane (register k holds row k of both), added up as the
// 16x16 and 32x32 kernels do: ( SATD + 2 ) >> 2 per block.
static inline uint32_t xCalcHAD8x8Pair_AVX2( __m256i rows[8] )
{
  // the vertical transform and the first horizontal stage on 16-bit values
  xHadamardStage16_AVX2( rows, 4 );
  xHadamardStage16_AVX2( rows, 2 );
  xHadamardStage16_AVX2( rows, 1 );
  xTranspose8x8Lanes_AVX2( rows );
  xHadamardStage16_AVX2( rows, 4 );

  // the rest on 32-bit values, each block staying in its lane: low[k] holds values 0 to 3 of both lanes of rows[k],
  // high[k] values 4 to 7
  __m256i low[8], high[8];
  for( int k = 0; k < 8; k++ )
  {
    const __m256i sign = _mm256_srai_epi16( rows[k], 15 );
    low[k]  = _mm256_unpacklo_epi16( rows[k], sign );
    high[k] = _mm256_unpackhi_epi16( rows[k], sign );
  }
  xHadamardStage32_AVX2( low, 2 );
  xHadamardStage32_AVX2( high, 2 );

  // the DC coefficient of each block is the lane's first value of low[0] + low[1]
  const __m256i dc       = _mm256_add_epi32( low[0], low[1] );
  const uint32_t absDc[2] = { (uint32_t) abs( _mm256_extract_epi32( dc, 0 ) ), (uint32_t) abs( _mm256_extract_epi32( dc, 4 ) ) };

  __m256i iSum = _mm256_setzero_si256();
  xAddMaxAbsOfPairs32_AVX2( low, iSum );
  xAddMaxAbsOfPairs32_AVX2( high, iSum );
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
static inline uint32_t xCalcHAD16x8Lanes_AVX2( __m256i rows[8] )
{
  // the vertical transform of both lanes and the first horizontal stage on 16-bit values
  xHadamardStage16_AVX2( rows, 4 );
  xHadamardStage16_AVX2( rows, 2 );
  xHadamardStage16_AVX2( rows, 1 );
  xTranspose8x8Lanes_AVX2( rows );
  xHadamardStage16_AVX2( rows, 4 );

  // the rest on 32-bit values, starting with the stage between the lanes: their sum and their difference
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

  // the DC coefficient is the first value of sum[0] + sum[1]
  const uint32_t absDc = abs( _mm256_extract_epi32( sum[0], 0 ) + _mm256_extract_epi32( sum[1], 0 ) );

  __m256i iSum = _mm256_setzero_si256();
  xAddMaxAbsOfPairs32_AVX2( sum, iSum );
  xAddMaxAbsOfPairs32_AVX2( dif, iSum );
  iSum = _mm256_slli_epi32( iSum, 1 );      // the sum of absolute values after the last horizontal stage
  iSum = _mm256_hadd_epi32( iSum, iSum );
  iSum = _mm256_hadd_epi32( iSum, iSum );
  iSum = _mm256_add_epi32( iSum, _mm256_permute2x128_si256( iSum, iSum, 0x11 ) );

  uint32_t sad = _mm_cvtsi128_si32( _mm256_castsi256_si128( iSum ) );
  sad -= absDc;
  sad += absDc >> 2;
  return (uint32_t)(sad / sqrt(16.0 * 8) * 2);
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

  m_afpDistortFuncX5[0] = xGetSADX5_8xN_SIMD <vext>;
  m_afpDistortFuncX5[1] = xGetSADX5_16xN_SIMD_X86<vext>;
}

template void RdCost::_initRdCostX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


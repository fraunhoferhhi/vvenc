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
/** \file     RdCostX86.cpp
    \brief    RD cost computation class, SIMD version
*/

#pragma once

#include "CommonDefX86.h"
#include "Rom.h"
#include "RdCost.h"

#include <math.h>

#ifdef TARGET_SIMD_X86

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
    uiRet = ( _mm_cvtsi128_si64( _mm256_castsi256_si128( Sum ) ) + _mm_cvtsi128_si32( _mm256_castsi256_si128( _mm256_permute2x128_si256( Sum, Sum, 0x11 ) ) ) ) >> uiShift;
#endif
  }
  else if( ( iCols & 7 ) == 0 )
  {
    __m128i Sum = _mm_setzero_si128();
    for( int iY = 0; iY < iRows; iY++ )
    {
      for( int iX = 0; iX < iCols; iX += 8 )
      {
        __m128i Src1 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadu_si128 ( ( const __m128i* )( &pSrc1[iX] ) ) ) : ( _mm_unpacklo_epi8( _mm_loadl_epi64( ( const __m128i* )( &pSrc1[iX] ) ), _mm_setzero_si128() ) );
        __m128i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _mm_lddqu_si128( ( const __m128i* )( &pSrc2[iX] ) ) ) : ( _mm_unpacklo_epi8( _mm_loadl_epi64( ( const __m128i* )( &pSrc2[iX] ) ), _mm_setzero_si128() ) );
        __m128i Diff = _mm_sub_epi16( Src1, Src2 );
        __m128i Res = _mm_madd_epi16( Diff, Diff );
        Sum = _mm_add_epi32( Sum, Res );
      }
      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;
    }
    Sum = _mm_hadd_epi32( Sum, Sum );
    Sum = _mm_hadd_epi32( Sum, Sum );
    uiRet = _mm_cvtsi128_si64( Sum )>>uiShift;
  }
  else
  {
    __m128i Sum = _mm_setzero_si128();
    for( int iY = 0; iY < iRows; iY++ )
    {
      for( int iX = 0; iX < iCols; iX += 4 )
      {
        __m128i Src1 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&pSrc1[iX] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&pSrc1[iX] ), _mm_setzero_si128() ) );
        __m128i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&pSrc2[iX] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&pSrc2[iX] ), _mm_setzero_si128() ) );
        __m128i Diff = _mm_sub_epi16( Src1, Src2 );
        __m128i Res = _mm_madd_epi16( Diff, Diff );
        Sum = _mm_add_epi32( Sum, Res );
      }
      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;
    }
    Sum = _mm_hadd_epi32( Sum, Sum );
    uiRet = _mm_cvtsi128_si64( Sum )>>uiShift;
  }

  return uiRet;
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

  if( 4 == iWidth )
  {
    __m128i Sum = _mm_setzero_si128();
    for( int iY = 0; iY < iRows; iY++ )
    {
      __m128i Src1 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )pSrc1 ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)pSrc1 ), _mm_setzero_si128() ) );
      __m128i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )pSrc2 ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)pSrc2 ), _mm_setzero_si128() ) );
      pSrc1 += iStrideSrc1;
      pSrc2 += iStrideSrc2;
      __m128i Diff = _mm_sub_epi16( Src1, Src2 );
      __m128i Res  = _mm_madd_epi16( Diff, Diff );
      Sum = _mm_add_epi32( Sum, Res );
    }
    Sum = _mm_hadd_epi32( Sum, Sum );
    uiRet = _mm_cvtsi128_si64( Sum )>>uiShift;
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
          __m256i Src1 = ( sizeof( Torg ) > 1 ) ? ( _mm256_lddqu_si256( ( __m256i* )( &pSrc1[iX] ) ) ) : ( _mm256_unpacklo_epi8( _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_lddqu_si128( ( __m128i* )( &pSrc1[iX] ) ) ), 0xD8 ), _mm256_setzero_si256() ) );
          __m256i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _mm256_lddqu_si256( ( __m256i* )( &pSrc2[iX] ) ) ) : ( _mm256_unpacklo_epi8( _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_lddqu_si128( ( __m128i* )( &pSrc2[iX] ) ) ), 0xD8 ), _mm256_setzero_si256() ) );
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
          __m128i Src1 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadu_si128( ( const __m128i* )( &pSrc1[iX] ) ) ) : ( _mm_unpacklo_epi8( _mm_loadl_epi64( ( const __m128i* )( &pSrc1[iX] ) ), _mm_setzero_si128() ) );
          __m128i Src2 = ( sizeof( Tcur ) > 1 ) ? ( _mm_lddqu_si128( ( const __m128i* )( &pSrc2[iX] ) ) ) : ( _mm_unpacklo_epi8( _mm_loadl_epi64( ( const __m128i* )( &pSrc2[iX] ) ), _mm_setzero_si128() ) );
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
        __m128i vsrc2 = _mm_lddqu_si128( ( const __m128i* )( &pSrc2[iX] ) );
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
        __m128i vsrc1 = _mm_loadl_epi64( ( const __m128i* )&pSrc1[iX] );
        __m128i vsrc2 = _mm_loadl_epi64( ( const __m128i* )&pSrc2[iX] );
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
      __m128i vsrc1 = _mm_or_si128( _mm_loadl_epi64( ( const __m128i* )pSrc1 ), _mm_slli_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc1[iStrideSrc1] ) ), 8 ) );
      __m128i vsrc2 = _mm_or_si128( _mm_loadl_epi64( ( const __m128i* )pSrc2 ), _mm_slli_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc2[iStrideSrc2] ) ), 8 ) );
      __m128i vsum  = _mm_cvtepi16_epi32( _mm_hadd_epi16( _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ), vzero ) );

      vsrc1 = _mm_or_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc1[2 * iStrideSrc1] ) ), _mm_slli_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc1[3 * iStrideSrc1] ) ), 8 ) );
      vsrc2 = _mm_or_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc2[2 * iStrideSrc2] ) ), _mm_slli_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc2[3 * iStrideSrc2] ) ), 8 ) );
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
        __m128i vsrc1 = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* )pSrc1 ) );
        __m128i vsrc2 = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* )pSrc2 ) );
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
      // Do for width that multiple of 16
      __m256i vone   = _mm256_set1_epi16( 1 );
      __m256i vsum32 = _mm256_setzero_si256();

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
      __m128i vone   = _mm_set1_epi16( 1 );
      __m128i vsum32 = _mm_setzero_si128();

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
        vsum32 = _mm_add_epi32( vsum32, vsumtemp );

        pSrc1   += iStrideSrc1;
        pSrc2   += iStrideSrc2;
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
  __m128i r0 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&piOrg[0] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piOrg[0] ), _mm_setzero_si128() ) );
  __m128i r1 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&piOrg[iStrideOrg] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piOrg[iStrideOrg] ), _mm_setzero_si128() ) );
  __m128i r2 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&piOrg[2 * iStrideOrg] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piOrg[2 * iStrideOrg] ), _mm_setzero_si128() ) );
  __m128i r3 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&piOrg[3 * iStrideOrg] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piOrg[3 * iStrideOrg] ), _mm_setzero_si128() ) );
  __m128i r4 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&piCur[0] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piCur[0] ), _mm_setzero_si128() ) );
  __m128i r5 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&piCur[iStrideCur] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piCur[iStrideCur] ), _mm_setzero_si128() ) );
  __m128i r6 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&piCur[2 * iStrideCur] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piCur[2 * iStrideCur] ), _mm_setzero_si128() ) );
  __m128i r7 = ( sizeof( Tcur ) > 1 ) ? ( _mm_loadl_epi64( ( const __m128i* )&piCur[3 * iStrideCur] ) ) : ( _mm_unpacklo_epi8( _mm_cvtsi32_si128( *(const int*)&piCur[3 * iStrideCur] ), _mm_setzero_si128() ) );

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
    __m128i r0 = ( sizeof( Torg ) > 1 ) ? ( _mm_loadu_si128( ( __m128i* )piOrg ) ) : ( _mm_unpacklo_epi8( _mm_loadl_epi64( ( const __m128i* )piOrg ), _mm_setzero_si128() ) );
    __m128i r1 = ( sizeof( Tcur ) > 1 ) ? ( _mm_lddqu_si128( ( __m128i* )piCur ) ) : ( _mm_unpacklo_epi8( _mm_loadl_epi64( ( const __m128i* )piCur ), _mm_setzero_si128() ) ); // th  _mm_loadu_si128( (__m128i*)piCur )
    m2[0][k] = _mm_sub_epi16( r0, r1 ); // 11bit
    //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
    //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

#if 0
  for( int i = 0; i < 2; i++ )
  {
    //horizontal
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

    m1[i][0] = _mm_add_epi32( m2[i][0], m2[i][1] );
    m1[i][1] = _mm_sub_epi32( m2[i][0], m2[i][1] );
    m1[i][2] = _mm_add_epi32( m2[i][2], m2[i][3] );
    m1[i][3] = _mm_sub_epi32( m2[i][2], m2[i][3] );
    m1[i][4] = _mm_add_epi32( m2[i][4], m2[i][5] );
    m1[i][5] = _mm_sub_epi32( m2[i][4], m2[i][5] );
    m1[i][6] = _mm_add_epi32( m2[i][6], m2[i][7] );
    m1[i][7] = _mm_sub_epi32( m2[i][6], m2[i][7] );

    m2[i][0] = _mm_unpacklo_epi32( m1[i][0], m1[i][1] );
    m2[i][1] = _mm_unpacklo_epi32( m1[i][2], m1[i][3] );
    m2[i][2] = _mm_unpackhi_epi32( m1[i][0], m1[i][1] );
    m2[i][3] = _mm_unpackhi_epi32( m1[i][2], m1[i][3] );
    m2[i][4] = _mm_unpacklo_epi32( m1[i][4], m1[i][5] );
    m2[i][5] = _mm_unpacklo_epi32( m1[i][6], m1[i][7] );
    m2[i][6] = _mm_unpackhi_epi32( m1[i][4], m1[i][5] );
    m2[i][7] = _mm_unpackhi_epi32( m1[i][6], m1[i][7] );

    m1[i][0] = _mm_unpacklo_epi64( m2[i][0], m2[i][1] );
    m1[i][1] = _mm_unpackhi_epi64( m2[i][0], m2[i][1] );
    m1[i][2] = _mm_unpacklo_epi64( m2[i][2], m2[i][3] );
    m1[i][3] = _mm_unpackhi_epi64( m2[i][2], m2[i][3] );
    m1[i][4] = _mm_unpacklo_epi64( m2[i][4], m2[i][5] );
    m1[i][5] = _mm_unpackhi_epi64( m2[i][4], m2[i][5] );
    m1[i][6] = _mm_unpacklo_epi64( m2[i][6], m2[i][7] );
    m1[i][7] = _mm_unpackhi_epi64( m2[i][6], m2[i][7] );
  }
#else
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
#endif

  __m128i n1[2][8];
  __m128i n2[2][8];

  for( int i = 0; i < 8; i++ )
  {
    int ii = i % 4;
    int ij = i >> 2;

    n2[0][i] = m1[ij][ii    ];
    n2[1][i] = m1[ij][ii + 4];
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
  }
  for( int i = 0; i < 8; i++ )
  {
    m1[0][i] = _mm_add_epi32( n1[0][i], n1[1][i] );
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
  uint32_t absDc = _mm_cvtsi128_si32( n1[0][0] );
  sad -= absDc;
  sad += absDc >> 2;
  sad = ( ( sad + 2 ) >> 2 );

  return sad;
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
      __m128i r1 = _mm_lddqu_si128( (__m128i*) piCurPtr );
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
    __m128i r1 =_mm_lddqu_si128( (__m128i*)piCur );
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
    __m128i r0 = (sizeof( Torg ) > 1) ? (_mm_loadu_si128 ( (__m128i*)piOrg )) : (_mm_unpacklo_epi8( _mm_loadl_epi64( (const __m128i*)piOrg ), _mm_setzero_si128() ));
    __m128i r1 = (sizeof( Tcur ) > 1) ? (_mm_lddqu_si128( (__m128i*)piCur )) : (_mm_unpacklo_epi8( _mm_loadl_epi64( (const __m128i*)piCur ), _mm_setzero_si128() )); // th  _mm_loadu_si128( (__m128i*)piCur )
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
    __m128i r0 = (sizeof( Torg ) > 1) ? (_mm_loadl_epi64( (__m128i*)piOrg )) : (_mm_cvtsi32_si128( *(const int*)piOrg ));
    __m128i r1 = (sizeof( Tcur ) > 1) ? (_mm_loadl_epi64( (__m128i*)piCur )) : (_mm_cvtsi32_si128( *(const int*)piCur ));
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

static uint32_t xCalcHAD16x16_AVX2( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  const int iLoops = 2;
  __m256i m1[2][8], m2[2][8];

  CHECK( iBitDepth > 10, "Only bitdepths up to 10 supported!" );

  for( int l = 0; l < iLoops; l++ )
  {
    for( int k = 0; k < 8; k++ )
    {
      __m256i r0 = _mm256_loadu_si256( ( __m256i* ) piOrg );
      __m256i r1 = _mm256_loadu_si256( ( __m256i* ) piCur );
      m2[0][k] = _mm256_sub_epi16( r0, r1 ); // 11 bit
      //m2[1][k] = _mm256_cvtepi16_epi32( _mm256_extracti128_si256( m2[0][k], 1 ) );
      //m2[0][k] = _mm256_cvtepi16_epi32( _mm256_castsi256_si128( m2[0][k] ) );
      piCur += iStrideCur;
      piOrg += iStrideOrg;
    }

    constexpr int perm_unpacklo_epi128 = ( 0 << 0 ) + ( 2 << 4 );
    constexpr int perm_unpackhi_epi128 = ( 1 << 0 ) + ( 3 << 4 );
#if 0
    for( int i = 0; i < 2; i++ )
    {
      m1[i][0] = _mm256_add_epi32( m2[i][0], m2[i][4] );
      m1[i][1] = _mm256_add_epi32( m2[i][1], m2[i][5] );
      m1[i][2] = _mm256_add_epi32( m2[i][2], m2[i][6] );
      m1[i][3] = _mm256_add_epi32( m2[i][3], m2[i][7] );
      m1[i][4] = _mm256_sub_epi32( m2[i][0], m2[i][4] );
      m1[i][5] = _mm256_sub_epi32( m2[i][1], m2[i][5] );
      m1[i][6] = _mm256_sub_epi32( m2[i][2], m2[i][6] );
      m1[i][7] = _mm256_sub_epi32( m2[i][3], m2[i][7] );

      m2[i][0] = _mm256_add_epi32( m1[i][0], m1[i][2] );
      m2[i][1] = _mm256_add_epi32( m1[i][1], m1[i][3] );
      m2[i][2] = _mm256_sub_epi32( m1[i][0], m1[i][2] );
      m2[i][3] = _mm256_sub_epi32( m1[i][1], m1[i][3] );
      m2[i][4] = _mm256_add_epi32( m1[i][4], m1[i][6] );
      m2[i][5] = _mm256_add_epi32( m1[i][5], m1[i][7] );
      m2[i][6] = _mm256_sub_epi32( m1[i][4], m1[i][6] );
      m2[i][7] = _mm256_sub_epi32( m1[i][5], m1[i][7] );

      m1[i][0] = _mm256_add_epi32( m2[i][0], m2[i][1] );
      m1[i][1] = _mm256_sub_epi32( m2[i][0], m2[i][1] );
      m1[i][2] = _mm256_add_epi32( m2[i][2], m2[i][3] );
      m1[i][3] = _mm256_sub_epi32( m2[i][2], m2[i][3] );
      m1[i][4] = _mm256_add_epi32( m2[i][4], m2[i][5] );
      m1[i][5] = _mm256_sub_epi32( m2[i][4], m2[i][5] );
      m1[i][6] = _mm256_add_epi32( m2[i][6], m2[i][7] );
      m1[i][7] = _mm256_sub_epi32( m2[i][6], m2[i][7] );

      // transpose
      // 8x8
      m2[i][0] = _mm256_unpacklo_epi32( m1[i][0], m1[i][1] );
      m2[i][1] = _mm256_unpacklo_epi32( m1[i][2], m1[i][3] );
      m2[i][2] = _mm256_unpacklo_epi32( m1[i][4], m1[i][5] );
      m2[i][3] = _mm256_unpacklo_epi32( m1[i][6], m1[i][7] );
      m2[i][4] = _mm256_unpackhi_epi32( m1[i][0], m1[i][1] );
      m2[i][5] = _mm256_unpackhi_epi32( m1[i][2], m1[i][3] );
      m2[i][6] = _mm256_unpackhi_epi32( m1[i][4], m1[i][5] );
      m2[i][7] = _mm256_unpackhi_epi32( m1[i][6], m1[i][7] );

      m1[i][0] = _mm256_unpacklo_epi64( m2[i][0], m2[i][1] );
      m1[i][1] = _mm256_unpackhi_epi64( m2[i][0], m2[i][1] );
      m1[i][2] = _mm256_unpacklo_epi64( m2[i][2], m2[i][3] );
      m1[i][3] = _mm256_unpackhi_epi64( m2[i][2], m2[i][3] );
      m1[i][4] = _mm256_unpacklo_epi64( m2[i][4], m2[i][5] );
      m1[i][5] = _mm256_unpackhi_epi64( m2[i][4], m2[i][5] );
      m1[i][6] = _mm256_unpacklo_epi64( m2[i][6], m2[i][7] );
      m1[i][7] = _mm256_unpackhi_epi64( m2[i][6], m2[i][7] );

      m2[i][0] = _mm256_permute2x128_si256( m1[i][0], m1[i][2], perm_unpacklo_epi128 );
      m2[i][1] = _mm256_permute2x128_si256( m1[i][0], m1[i][2], perm_unpackhi_epi128 );
      m2[i][2] = _mm256_permute2x128_si256( m1[i][1], m1[i][3], perm_unpacklo_epi128 );
      m2[i][3] = _mm256_permute2x128_si256( m1[i][1], m1[i][3], perm_unpackhi_epi128 );
      m2[i][4] = _mm256_permute2x128_si256( m1[i][4], m1[i][6], perm_unpacklo_epi128 );
      m2[i][5] = _mm256_permute2x128_si256( m1[i][4], m1[i][6], perm_unpackhi_epi128 );
      m2[i][6] = _mm256_permute2x128_si256( m1[i][5], m1[i][7], perm_unpacklo_epi128 );
      m2[i][7] = _mm256_permute2x128_si256( m1[i][5], m1[i][7], perm_unpackhi_epi128 );
    }
#else
    m1[0][0] = _mm256_add_epi16( m2[0][0], m2[0][4] );
    m1[0][1] = _mm256_add_epi16( m2[0][1], m2[0][5] );
    m1[0][2] = _mm256_add_epi16( m2[0][2], m2[0][6] );
    m1[0][3] = _mm256_add_epi16( m2[0][3], m2[0][7] );
    m1[0][4] = _mm256_sub_epi16( m2[0][0], m2[0][4] );
    m1[0][5] = _mm256_sub_epi16( m2[0][1], m2[0][5] );
    m1[0][6] = _mm256_sub_epi16( m2[0][2], m2[0][6] );
    m1[0][7] = _mm256_sub_epi16( m2[0][3], m2[0][7] ); // 12 bit

    m2[0][0] = _mm256_add_epi16( m1[0][0], m1[0][2] );
    m2[0][1] = _mm256_add_epi16( m1[0][1], m1[0][3] );
    m2[0][2] = _mm256_sub_epi16( m1[0][0], m1[0][2] );
    m2[0][3] = _mm256_sub_epi16( m1[0][1], m1[0][3] );
    m2[0][4] = _mm256_add_epi16( m1[0][4], m1[0][6] );
    m2[0][5] = _mm256_add_epi16( m1[0][5], m1[0][7] );
    m2[0][6] = _mm256_sub_epi16( m1[0][4], m1[0][6] );
    m2[0][7] = _mm256_sub_epi16( m1[0][5], m1[0][7] ); // 13 bit

    m1[0][0] = _mm256_add_epi16( m2[0][0], m2[0][1] );
    m1[0][1] = _mm256_sub_epi16( m2[0][0], m2[0][1] );
    m1[0][2] = _mm256_add_epi16( m2[0][2], m2[0][3] );
    m1[0][3] = _mm256_sub_epi16( m2[0][2], m2[0][3] );
    m1[0][4] = _mm256_add_epi16( m2[0][4], m2[0][5] );
    m1[0][5] = _mm256_sub_epi16( m2[0][4], m2[0][5] );
    m1[0][6] = _mm256_add_epi16( m2[0][6], m2[0][7] );
    m1[0][7] = _mm256_sub_epi16( m2[0][6], m2[0][7] ); // 14 bit

    // transpose
    // 8x8
    m2[0][0] = _mm256_unpacklo_epi16( m1[0][0], m1[0][1] );
    m2[0][1] = _mm256_unpacklo_epi16( m1[0][2], m1[0][3] );
    m2[0][2] = _mm256_unpacklo_epi16( m1[0][4], m1[0][5] );
    m2[0][3] = _mm256_unpacklo_epi16( m1[0][6], m1[0][7] );
    m2[0][4] = _mm256_unpackhi_epi16( m1[0][0], m1[0][1] );
    m2[0][5] = _mm256_unpackhi_epi16( m1[0][2], m1[0][3] );
    m2[0][6] = _mm256_unpackhi_epi16( m1[0][4], m1[0][5] );
    m2[0][7] = _mm256_unpackhi_epi16( m1[0][6], m1[0][7] );

    m1[0][0] = _mm256_unpacklo_epi32( m2[0][0], m2[0][1] );
    m1[0][1] = _mm256_unpackhi_epi32( m2[0][0], m2[0][1] );
    m1[0][2] = _mm256_unpacklo_epi32( m2[0][2], m2[0][3] );
    m1[0][3] = _mm256_unpackhi_epi32( m2[0][2], m2[0][3] );
    m1[0][4] = _mm256_unpacklo_epi32( m2[0][4], m2[0][5] );
    m1[0][5] = _mm256_unpackhi_epi32( m2[0][4], m2[0][5] );
    m1[0][6] = _mm256_unpacklo_epi32( m2[0][6], m2[0][7] );
    m1[0][7] = _mm256_unpackhi_epi32( m2[0][6], m2[0][7] );

    m2[0][0] = _mm256_unpacklo_epi64( m1[0][0], m1[0][2] );
    m2[0][1] = _mm256_unpackhi_epi64( m1[0][0], m1[0][2] );
    m2[0][2] = _mm256_unpacklo_epi64( m1[0][1], m1[0][3] );
    m2[0][3] = _mm256_unpackhi_epi64( m1[0][1], m1[0][3] );
    m2[0][4] = _mm256_unpacklo_epi64( m1[0][4], m1[0][6] );
    m2[0][5] = _mm256_unpackhi_epi64( m1[0][4], m1[0][6] );
    m2[0][6] = _mm256_unpacklo_epi64( m1[0][5], m1[0][7] );
    m2[0][7] = _mm256_unpackhi_epi64( m1[0][5], m1[0][7] );

    for( int k = 0; k < 8; k++ )
    {
      m2[1][k] = _mm256_cvtepi16_epi32( _mm256_extracti128_si256( m2[0][k], 1 ) );
      m2[0][k] = _mm256_cvtepi16_epi32( _mm256_castsi256_si128  ( m2[0][k]    ) );
    }
#endif

    m1[0][0] = _mm256_permute2x128_si256( m2[0][0], m2[1][0], perm_unpacklo_epi128 );
    m1[0][1] = _mm256_permute2x128_si256( m2[0][1], m2[1][1], perm_unpacklo_epi128 );
    m1[0][2] = _mm256_permute2x128_si256( m2[0][2], m2[1][2], perm_unpacklo_epi128 );
    m1[0][3] = _mm256_permute2x128_si256( m2[0][3], m2[1][3], perm_unpacklo_epi128 );
    m1[0][4] = _mm256_permute2x128_si256( m2[0][4], m2[1][4], perm_unpacklo_epi128 );
    m1[0][5] = _mm256_permute2x128_si256( m2[0][5], m2[1][5], perm_unpacklo_epi128 );
    m1[0][6] = _mm256_permute2x128_si256( m2[0][6], m2[1][6], perm_unpacklo_epi128 );
    m1[0][7] = _mm256_permute2x128_si256( m2[0][7], m2[1][7], perm_unpacklo_epi128 );

    m1[1][0] = _mm256_permute2x128_si256( m2[0][0], m2[1][0], perm_unpackhi_epi128 );
    m1[1][1] = _mm256_permute2x128_si256( m2[0][1], m2[1][1], perm_unpackhi_epi128 );
    m1[1][2] = _mm256_permute2x128_si256( m2[0][2], m2[1][2], perm_unpackhi_epi128 );
    m1[1][3] = _mm256_permute2x128_si256( m2[0][3], m2[1][3], perm_unpackhi_epi128 );
    m1[1][4] = _mm256_permute2x128_si256( m2[0][4], m2[1][4], perm_unpackhi_epi128 );
    m1[1][5] = _mm256_permute2x128_si256( m2[0][5], m2[1][5], perm_unpackhi_epi128 );
    m1[1][6] = _mm256_permute2x128_si256( m2[0][6], m2[1][6], perm_unpackhi_epi128 );
    m1[1][7] = _mm256_permute2x128_si256( m2[0][7], m2[1][7], perm_unpackhi_epi128 );

    for( int i = 0; i < 2; i++ )
    {
      m2[i][0] = _mm256_add_epi32( m1[i][0], m1[i][4] );
      m2[i][1] = _mm256_add_epi32( m1[i][1], m1[i][5] );
      m2[i][2] = _mm256_add_epi32( m1[i][2], m1[i][6] );
      m2[i][3] = _mm256_add_epi32( m1[i][3], m1[i][7] );
      m2[i][4] = _mm256_sub_epi32( m1[i][0], m1[i][4] );
      m2[i][5] = _mm256_sub_epi32( m1[i][1], m1[i][5] );
      m2[i][6] = _mm256_sub_epi32( m1[i][2], m1[i][6] );
      m2[i][7] = _mm256_sub_epi32( m1[i][3], m1[i][7] );

      m1[i][0] = _mm256_add_epi32( m2[i][0], m2[i][2] );
      m1[i][1] = _mm256_add_epi32( m2[i][1], m2[i][3] );
      m1[i][2] = _mm256_sub_epi32( m2[i][0], m2[i][2] );
      m1[i][3] = _mm256_sub_epi32( m2[i][1], m2[i][3] );
      m1[i][4] = _mm256_add_epi32( m2[i][4], m2[i][6] );
      m1[i][5] = _mm256_add_epi32( m2[i][5], m2[i][7] );
      m1[i][6] = _mm256_sub_epi32( m2[i][4], m2[i][6] );
      m1[i][7] = _mm256_sub_epi32( m2[i][5], m2[i][7] );

      m2[i][0] = _mm256_abs_epi32( _mm256_add_epi32( m1[i][0], m1[i][1] ) );
      m2[i][1] = _mm256_abs_epi32( _mm256_sub_epi32( m1[i][0], m1[i][1] ) );
      m2[i][2] = _mm256_abs_epi32( _mm256_add_epi32( m1[i][2], m1[i][3] ) );
      m2[i][3] = _mm256_abs_epi32( _mm256_sub_epi32( m1[i][2], m1[i][3] ) );
      m2[i][4] = _mm256_abs_epi32( _mm256_add_epi32( m1[i][4], m1[i][5] ) );
      m2[i][5] = _mm256_abs_epi32( _mm256_sub_epi32( m1[i][4], m1[i][5] ) );
      m2[i][6] = _mm256_abs_epi32( _mm256_add_epi32( m1[i][6], m1[i][7] ) );
      m2[i][7] = _mm256_abs_epi32( _mm256_sub_epi32( m1[i][6], m1[i][7] ) );
    }

    uint32_t absDc0 = _mm_cvtsi128_si32( _mm256_castsi256_si128( m2[0][0] ) );
    uint32_t absDc1 = _mm_cvtsi128_si32( _mm256_castsi256_si128( _mm256_permute2x128_si256( m2[0][0], m2[0][0], 0x11 ) ) );

    for( int i = 0; i < 8; i++ )
    {
      m1[0][i] = _mm256_add_epi32( m2[0][i], m2[1][i] );
    }

    m1[0][0] = _mm256_add_epi32( m1[0][0], m1[0][1] );
    m1[0][2] = _mm256_add_epi32( m1[0][2], m1[0][3] );
    m1[0][4] = _mm256_add_epi32( m1[0][4], m1[0][5] );
    m1[0][6] = _mm256_add_epi32( m1[0][6], m1[0][7] );

    m1[0][0] = _mm256_add_epi32( m1[0][0], m1[0][2] );
    m1[0][4] = _mm256_add_epi32( m1[0][4], m1[0][6] );

    __m256i iSum = _mm256_add_epi32( m1[0][0], m1[0][4] );

    iSum = _mm256_hadd_epi32( iSum, iSum );
    iSum = _mm256_hadd_epi32( iSum, iSum );

    uint32_t tmp;
    tmp = _mm_cvtsi128_si32( _mm256_castsi256_si128( iSum ) );
    // 16x16 block is done by adding together 4 8x8 SATDs
    tmp -= absDc0;
    tmp += absDc0 >> 2;
    tmp = ( ( tmp + 2 ) >> 2 );
    sad += tmp;

    tmp = _mm_cvtsi128_si32( _mm256_castsi256_si128( _mm256_permute2x128_si256( iSum, iSum, 0x11 ) ) );
    // 16x16 block is done by adding together 4 8x8 SATDs
    tmp -= absDc1;
    tmp += absDc1 >> 2;
    tmp = ( ( tmp + 2 ) >> 2 );
    sad += tmp;
  }

#endif
  return ( sad );
}

static uint32_t xCalcHAD16x8_AVX2( const Torg *piOrg, const Tcur *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  __m256i m1[16], m2[16];

  CHECK( iBitDepth > 10, "Only bitdepths up to 10 supported!" );

  {
    for( int k = 0; k < 8; k++ )
    {
      __m256i r0 = _mm256_lddqu_si256( (__m256i*)piOrg );
      __m256i r1 = _mm256_lddqu_si256( (__m256i*)piCur );
      m1[k]   = _mm256_sub_epi16( r0, r1 ); // 11 bit
      //m1[k+8] = _mm256_cvtepi16_epi32( _mm256_extracti128_si256( m1[k], 1 ) );
      //m1[k]   = _mm256_cvtepi16_epi32( _mm256_castsi256_si128  ( m1[k]    ) );
      piCur += iStrideCur;
      piOrg += iStrideOrg;
    }

    // vertical, first 8x8
#if 0
    m2[0] = _mm256_add_epi32( m1[0], m1[4] );
    m2[1] = _mm256_add_epi32( m1[1], m1[5] );
    m2[2] = _mm256_add_epi32( m1[2], m1[6] );
    m2[3] = _mm256_add_epi32( m1[3], m1[7] );
    m2[4] = _mm256_sub_epi32( m1[0], m1[4] );
    m2[5] = _mm256_sub_epi32( m1[1], m1[5] );
    m2[6] = _mm256_sub_epi32( m1[2], m1[6] );
    m2[7] = _mm256_sub_epi32( m1[3], m1[7] );

    m1[0] = _mm256_add_epi32( m2[0], m2[2] );
    m1[1] = _mm256_add_epi32( m2[1], m2[3] );
    m1[2] = _mm256_sub_epi32( m2[0], m2[2] );
    m1[3] = _mm256_sub_epi32( m2[1], m2[3] );
    m1[4] = _mm256_add_epi32( m2[4], m2[6] );
    m1[5] = _mm256_add_epi32( m2[5], m2[7] );
    m1[6] = _mm256_sub_epi32( m2[4], m2[6] );
    m1[7] = _mm256_sub_epi32( m2[5], m2[7] );

    m2[0] = _mm256_add_epi32( m1[0], m1[1] );
    m2[1] = _mm256_sub_epi32( m1[0], m1[1] );
    m2[2] = _mm256_add_epi32( m1[2], m1[3] );
    m2[3] = _mm256_sub_epi32( m1[2], m1[3] );
    m2[4] = _mm256_add_epi32( m1[4], m1[5] );
    m2[5] = _mm256_sub_epi32( m1[4], m1[5] );
    m2[6] = _mm256_add_epi32( m1[6], m1[7] );
    m2[7] = _mm256_sub_epi32( m1[6], m1[7] );

    // vertical, second 8x8
    m2[8+0] = _mm256_add_epi32( m1[8+0], m1[8+4] );
    m2[8+1] = _mm256_add_epi32( m1[8+1], m1[8+5] );
    m2[8+2] = _mm256_add_epi32( m1[8+2], m1[8+6] );
    m2[8+3] = _mm256_add_epi32( m1[8+3], m1[8+7] );
    m2[8+4] = _mm256_sub_epi32( m1[8+0], m1[8+4] );
    m2[8+5] = _mm256_sub_epi32( m1[8+1], m1[8+5] );
    m2[8+6] = _mm256_sub_epi32( m1[8+2], m1[8+6] );
    m2[8+7] = _mm256_sub_epi32( m1[8+3], m1[8+7] );

    m1[8+0] = _mm256_add_epi32( m2[8+0], m2[8+2] );
    m1[8+1] = _mm256_add_epi32( m2[8+1], m2[8+3] );
    m1[8+2] = _mm256_sub_epi32( m2[8+0], m2[8+2] );
    m1[8+3] = _mm256_sub_epi32( m2[8+1], m2[8+3] );
    m1[8+4] = _mm256_add_epi32( m2[8+4], m2[8+6] );
    m1[8+5] = _mm256_add_epi32( m2[8+5], m2[8+7] );
    m1[8+6] = _mm256_sub_epi32( m2[8+4], m2[8+6] );
    m1[8+7] = _mm256_sub_epi32( m2[8+5], m2[8+7] );

    m2[8+0] = _mm256_add_epi32( m1[8+0], m1[8+1] );
    m2[8+1] = _mm256_sub_epi32( m1[8+0], m1[8+1] );
    m2[8+2] = _mm256_add_epi32( m1[8+2], m1[8+3] );
    m2[8+3] = _mm256_sub_epi32( m1[8+2], m1[8+3] );
    m2[8+4] = _mm256_add_epi32( m1[8+4], m1[8+5] );
    m2[8+5] = _mm256_sub_epi32( m1[8+4], m1[8+5] );
    m2[8+6] = _mm256_add_epi32( m1[8+6], m1[8+7] );
    m2[8+7] = _mm256_sub_epi32( m1[8+6], m1[8+7] );

    // transpose
    constexpr int perm_unpacklo_epi128 = ( 0 << 0 ) + ( 2 << 4 );
    constexpr int perm_unpackhi_epi128 = ( 1 << 0 ) + ( 3 << 4 );

    m1[0] = _mm256_unpacklo_epi32( m2[0], m2[1] );
    m1[1] = _mm256_unpacklo_epi32( m2[2], m2[3] );
    m1[2] = _mm256_unpacklo_epi32( m2[4], m2[5] );
    m1[3] = _mm256_unpacklo_epi32( m2[6], m2[7] );
    m1[4] = _mm256_unpackhi_epi32( m2[0], m2[1] );
    m1[5] = _mm256_unpackhi_epi32( m2[2], m2[3] );
    m1[6] = _mm256_unpackhi_epi32( m2[4], m2[5] );
    m1[7] = _mm256_unpackhi_epi32( m2[6], m2[7] );

    m2[0] = _mm256_unpacklo_epi64( m1[0], m1[1] );
    m2[1] = _mm256_unpackhi_epi64( m1[0], m1[1] );
    m2[2] = _mm256_unpacklo_epi64( m1[2], m1[3] );
    m2[3] = _mm256_unpackhi_epi64( m1[2], m1[3] );
    m2[4] = _mm256_unpacklo_epi64( m1[4], m1[5] );
    m2[5] = _mm256_unpackhi_epi64( m1[4], m1[5] );
    m2[6] = _mm256_unpacklo_epi64( m1[6], m1[7] );
    m2[7] = _mm256_unpackhi_epi64( m1[6], m1[7] );

    m1[0] = _mm256_permute2x128_si256( m2[0], m2[2], perm_unpacklo_epi128 );
    m1[1] = _mm256_permute2x128_si256( m2[0], m2[2], perm_unpackhi_epi128 );
    m1[2] = _mm256_permute2x128_si256( m2[1], m2[3], perm_unpacklo_epi128 );
    m1[3] = _mm256_permute2x128_si256( m2[1], m2[3], perm_unpackhi_epi128 );
    m1[4] = _mm256_permute2x128_si256( m2[4], m2[6], perm_unpacklo_epi128 );
    m1[5] = _mm256_permute2x128_si256( m2[4], m2[6], perm_unpackhi_epi128 );
    m1[6] = _mm256_permute2x128_si256( m2[5], m2[7], perm_unpacklo_epi128 );
    m1[7] = _mm256_permute2x128_si256( m2[5], m2[7], perm_unpackhi_epi128 );

    m1[8+0] = _mm256_unpacklo_epi32( m2[8+0], m2[8+1] );
    m1[8+1] = _mm256_unpacklo_epi32( m2[8+2], m2[8+3] );
    m1[8+2] = _mm256_unpacklo_epi32( m2[8+4], m2[8+5] );
    m1[8+3] = _mm256_unpacklo_epi32( m2[8+6], m2[8+7] );
    m1[8+4] = _mm256_unpackhi_epi32( m2[8+0], m2[8+1] );
    m1[8+5] = _mm256_unpackhi_epi32( m2[8+2], m2[8+3] );
    m1[8+6] = _mm256_unpackhi_epi32( m2[8+4], m2[8+5] );
    m1[8+7] = _mm256_unpackhi_epi32( m2[8+6], m2[8+7] );

    m2[8+0] = _mm256_unpacklo_epi64( m1[8+0], m1[8+1] );
    m2[8+1] = _mm256_unpackhi_epi64( m1[8+0], m1[8+1] );
    m2[8+2] = _mm256_unpacklo_epi64( m1[8+2], m1[8+3] );
    m2[8+3] = _mm256_unpackhi_epi64( m1[8+2], m1[8+3] );
    m2[8+4] = _mm256_unpacklo_epi64( m1[8+4], m1[8+5] );
    m2[8+5] = _mm256_unpackhi_epi64( m1[8+4], m1[8+5] );
    m2[8+6] = _mm256_unpacklo_epi64( m1[8+6], m1[8+7] );
    m2[8+7] = _mm256_unpackhi_epi64( m1[8+6], m1[8+7] );

    m1[8+0] = _mm256_permute2x128_si256( m2[8+0], m2[8+2], perm_unpacklo_epi128 );
    m1[8+1] = _mm256_permute2x128_si256( m2[8+0], m2[8+2], perm_unpackhi_epi128 );
    m1[8+2] = _mm256_permute2x128_si256( m2[8+1], m2[8+3], perm_unpacklo_epi128 );
    m1[8+3] = _mm256_permute2x128_si256( m2[8+1], m2[8+3], perm_unpackhi_epi128 );
    m1[8+4] = _mm256_permute2x128_si256( m2[8+4], m2[8+6], perm_unpacklo_epi128 );
    m1[8+5] = _mm256_permute2x128_si256( m2[8+4], m2[8+6], perm_unpackhi_epi128 );
    m1[8+6] = _mm256_permute2x128_si256( m2[8+5], m2[8+7], perm_unpacklo_epi128 );
    m1[8+7] = _mm256_permute2x128_si256( m2[8+5], m2[8+7], perm_unpackhi_epi128 );
#else
    m2[0] = _mm256_add_epi16( m1[0], m1[4] );
    m2[1] = _mm256_add_epi16( m1[1], m1[5] );
    m2[2] = _mm256_add_epi16( m1[2], m1[6] );
    m2[3] = _mm256_add_epi16( m1[3], m1[7] );
    m2[4] = _mm256_sub_epi16( m1[0], m1[4] );
    m2[5] = _mm256_sub_epi16( m1[1], m1[5] );
    m2[6] = _mm256_sub_epi16( m1[2], m1[6] );
    m2[7] = _mm256_sub_epi16( m1[3], m1[7] ); // 12 bit

    m1[0] = _mm256_add_epi16( m2[0], m2[2] );
    m1[1] = _mm256_add_epi16( m2[1], m2[3] );
    m1[2] = _mm256_sub_epi16( m2[0], m2[2] );
    m1[3] = _mm256_sub_epi16( m2[1], m2[3] );
    m1[4] = _mm256_add_epi16( m2[4], m2[6] );
    m1[5] = _mm256_add_epi16( m2[5], m2[7] );
    m1[6] = _mm256_sub_epi16( m2[4], m2[6] );
    m1[7] = _mm256_sub_epi16( m2[5], m2[7] ); // 13 bit

    m2[0] = _mm256_add_epi16( m1[0], m1[1] );
    m2[1] = _mm256_sub_epi16( m1[0], m1[1] );
    m2[2] = _mm256_add_epi16( m1[2], m1[3] );
    m2[3] = _mm256_sub_epi16( m1[2], m1[3] );
    m2[4] = _mm256_add_epi16( m1[4], m1[5] );
    m2[5] = _mm256_sub_epi16( m1[4], m1[5] );
    m2[6] = _mm256_add_epi16( m1[6], m1[7] );
    m2[7] = _mm256_sub_epi16( m1[6], m1[7] ); // 14 bit

    m1[0] = _mm256_unpacklo_epi16( m2[0], m2[1] );
    m1[1] = _mm256_unpacklo_epi16( m2[2], m2[3] );
    m1[2] = _mm256_unpacklo_epi16( m2[4], m2[5] );
    m1[3] = _mm256_unpacklo_epi16( m2[6], m2[7] );
    m1[4] = _mm256_unpackhi_epi16( m2[0], m2[1] );
    m1[5] = _mm256_unpackhi_epi16( m2[2], m2[3] );
    m1[6] = _mm256_unpackhi_epi16( m2[4], m2[5] );
    m1[7] = _mm256_unpackhi_epi16( m2[6], m2[7] );

    m2[0] = _mm256_unpacklo_epi32( m1[0], m1[1] );
    m2[1] = _mm256_unpackhi_epi32( m1[0], m1[1] );
    m2[2] = _mm256_unpacklo_epi32( m1[2], m1[3] );
    m2[3] = _mm256_unpackhi_epi32( m1[2], m1[3] );
    m2[4] = _mm256_unpacklo_epi32( m1[4], m1[5] );
    m2[5] = _mm256_unpackhi_epi32( m1[4], m1[5] );
    m2[6] = _mm256_unpacklo_epi32( m1[6], m1[7] );
    m2[7] = _mm256_unpackhi_epi32( m1[6], m1[7] );

    m1[0] = _mm256_unpacklo_epi64( m2[0], m2[2] );
    m1[1] = _mm256_unpackhi_epi64( m2[0], m2[2] );
    m1[2] = _mm256_unpacklo_epi64( m2[1], m2[3] );
    m1[3] = _mm256_unpackhi_epi64( m2[1], m2[3] );
    m1[4] = _mm256_unpacklo_epi64( m2[4], m2[6] );
    m1[5] = _mm256_unpackhi_epi64( m2[4], m2[6] );
    m1[6] = _mm256_unpacklo_epi64( m2[5], m2[7] );
    m1[7] = _mm256_unpackhi_epi64( m2[5], m2[7] );
    
    for( int k = 0; k < 8; k++ )
    {
      m1[k+8] = _mm256_cvtepi16_epi32( _mm256_extracti128_si256( m1[k], 1 ) );
      m1[k]   = _mm256_cvtepi16_epi32( _mm256_castsi256_si128  ( m1[k]    ) );
    }
#endif

    // horizontal
    {
      m2[ 0] = _mm256_add_epi32( m1[0], m1[ 8] );
      m2[ 1] = _mm256_add_epi32( m1[1], m1[ 9] );
      m2[ 2] = _mm256_add_epi32( m1[2], m1[10] );
      m2[ 3] = _mm256_add_epi32( m1[3], m1[11] );
      m2[ 4] = _mm256_add_epi32( m1[4], m1[12] );
      m2[ 5] = _mm256_add_epi32( m1[5], m1[13] );
      m2[ 6] = _mm256_add_epi32( m1[6], m1[14] );
      m2[ 7] = _mm256_add_epi32( m1[7], m1[15] );
      m2[ 8] = _mm256_sub_epi32( m1[0], m1[ 8] );
      m2[ 9] = _mm256_sub_epi32( m1[1], m1[ 9] );
      m2[10] = _mm256_sub_epi32( m1[2], m1[10] );
      m2[11] = _mm256_sub_epi32( m1[3], m1[11] );
      m2[12] = _mm256_sub_epi32( m1[4], m1[12] );
      m2[13] = _mm256_sub_epi32( m1[5], m1[13] );
      m2[14] = _mm256_sub_epi32( m1[6], m1[14] );
      m2[15] = _mm256_sub_epi32( m1[7], m1[15] );

      m1[ 0] = _mm256_add_epi32( m2[ 0], m2[ 4] );
      m1[ 1] = _mm256_add_epi32( m2[ 1], m2[ 5] );
      m1[ 2] = _mm256_add_epi32( m2[ 2], m2[ 6] );
      m1[ 3] = _mm256_add_epi32( m2[ 3], m2[ 7] );
      m1[ 4] = _mm256_sub_epi32( m2[ 0], m2[ 4] );
      m1[ 5] = _mm256_sub_epi32( m2[ 1], m2[ 5] );
      m1[ 6] = _mm256_sub_epi32( m2[ 2], m2[ 6] );
      m1[ 7] = _mm256_sub_epi32( m2[ 3], m2[ 7] );
      m1[ 8] = _mm256_add_epi32( m2[ 8], m2[12] );
      m1[ 9] = _mm256_add_epi32( m2[ 9], m2[13] );
      m1[10] = _mm256_add_epi32( m2[10], m2[14] );
      m1[11] = _mm256_add_epi32( m2[11], m2[15] );
      m1[12] = _mm256_sub_epi32( m2[ 8], m2[12] );
      m1[13] = _mm256_sub_epi32( m2[ 9], m2[13] );
      m1[14] = _mm256_sub_epi32( m2[10], m2[14] );
      m1[15] = _mm256_sub_epi32( m2[11], m2[15] );

      m2[ 0] = _mm256_add_epi32( m1[ 0], m1[ 2] );
      m2[ 1] = _mm256_add_epi32( m1[ 1], m1[ 3] );
      m2[ 2] = _mm256_sub_epi32( m1[ 0], m1[ 2] );
      m2[ 3] = _mm256_sub_epi32( m1[ 1], m1[ 3] );
      m2[ 4] = _mm256_add_epi32( m1[ 4], m1[ 6] );
      m2[ 5] = _mm256_add_epi32( m1[ 5], m1[ 7] );
      m2[ 6] = _mm256_sub_epi32( m1[ 4], m1[ 6] );
      m2[ 7] = _mm256_sub_epi32( m1[ 5], m1[ 7] );
      m2[ 8] = _mm256_add_epi32( m1[ 8], m1[10] );
      m2[ 9] = _mm256_add_epi32( m1[ 9], m1[11] );
      m2[10] = _mm256_sub_epi32( m1[ 8], m1[10] );
      m2[11] = _mm256_sub_epi32( m1[ 9], m1[11] );
      m2[12] = _mm256_add_epi32( m1[12], m1[14] );
      m2[13] = _mm256_add_epi32( m1[13], m1[15] );
      m2[14] = _mm256_sub_epi32( m1[12], m1[14] );
      m2[15] = _mm256_sub_epi32( m1[13], m1[15] );

      m1[ 0] = _mm256_abs_epi32( _mm256_add_epi32( m2[ 0], m2[ 1] ) );
      m1[ 1] = _mm256_abs_epi32( _mm256_sub_epi32( m2[ 0], m2[ 1] ) );
      m1[ 2] = _mm256_abs_epi32( _mm256_add_epi32( m2[ 2], m2[ 3] ) );
      m1[ 3] = _mm256_abs_epi32( _mm256_sub_epi32( m2[ 2], m2[ 3] ) );
      m1[ 4] = _mm256_abs_epi32( _mm256_add_epi32( m2[ 4], m2[ 5] ) );
      m1[ 5] = _mm256_abs_epi32( _mm256_sub_epi32( m2[ 4], m2[ 5] ) );
      m1[ 6] = _mm256_abs_epi32( _mm256_add_epi32( m2[ 6], m2[ 7] ) );
      m1[ 7] = _mm256_abs_epi32( _mm256_sub_epi32( m2[ 6], m2[ 7] ) );
      m1[ 8] = _mm256_abs_epi32( _mm256_add_epi32( m2[ 8], m2[ 9] ) );
      m1[ 9] = _mm256_abs_epi32( _mm256_sub_epi32( m2[ 8], m2[ 9] ) );
      m1[10] = _mm256_abs_epi32( _mm256_add_epi32( m2[10], m2[11] ) );
      m1[11] = _mm256_abs_epi32( _mm256_sub_epi32( m2[10], m2[11] ) );
      m1[12] = _mm256_abs_epi32( _mm256_add_epi32( m2[12], m2[13] ) );
      m1[13] = _mm256_abs_epi32( _mm256_sub_epi32( m2[12], m2[13] ) );
      m1[14] = _mm256_abs_epi32( _mm256_add_epi32( m2[14], m2[15] ) );
      m1[15] = _mm256_abs_epi32( _mm256_sub_epi32( m2[14], m2[15] ) );
    }

    uint32_t absDc = _mm_cvtsi128_si32( _mm256_castsi256_si128( m1[0] ) );

    // sum up
    m1[ 0] = _mm256_add_epi32( m1[ 0], m1[ 1] );
    m1[ 2] = _mm256_add_epi32( m1[ 2], m1[ 3] );
    m1[ 4] = _mm256_add_epi32( m1[ 4], m1[ 5] );
    m1[ 6] = _mm256_add_epi32( m1[ 6], m1[ 7] );
    m1[ 8] = _mm256_add_epi32( m1[ 8], m1[ 9] );
    m1[10] = _mm256_add_epi32( m1[10], m1[11] );
    m1[12] = _mm256_add_epi32( m1[12], m1[13] );
    m1[14] = _mm256_add_epi32( m1[14], m1[15] );

    m1[ 0] = _mm256_add_epi32( m1[ 0], m1[ 2] );
    m1[ 4] = _mm256_add_epi32( m1[ 4], m1[ 6] );
    m1[ 8] = _mm256_add_epi32( m1[ 8], m1[10] );
    m1[12] = _mm256_add_epi32( m1[12], m1[14] );

    m1[0] = _mm256_add_epi32( m1[0], m1[ 4] );
    m1[8] = _mm256_add_epi32( m1[8], m1[12] );

    __m256i iSum = _mm256_add_epi32( m1[0], m1[8] );
    iSum = _mm256_hadd_epi32( iSum, iSum );
    iSum = _mm256_hadd_epi32( iSum, iSum );
    iSum = _mm256_add_epi32( iSum, _mm256_permute2x128_si256( iSum, iSum, 0x11 ) );

    sad = _mm_cvtsi128_si32( _mm256_castsi256_si128( iSum ) );
    
    sad -= absDc;
    sad += absDc >> 2;
    sad = (uint32_t)(sad / sqrt(16.0 * 8) * 2);
  }

#endif //USE_AVX2

  return (sad);
}

static uint32_t xCalcHAD8x16_AVX2( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  __m256i m1[16], m2[16];

  {
    {
      for( int k = 0; k < 16; k++ )
      {
        __m256i r0 = _mm256_cvtepi16_epi32( _mm_lddqu_si128( (__m128i*)piOrg ) );
        __m256i r1 = _mm256_cvtepi16_epi32( _mm_lddqu_si128( (__m128i*)piCur ) );
        m1[k] = _mm256_sub_epi32( r0, r1 );
        piCur += iStrideCur;
        piOrg += iStrideOrg;
      }
    }

    // vertical

    m2[ 0] = _mm256_add_epi32( m1[0], m1[ 8] );
    m2[ 1] = _mm256_add_epi32( m1[1], m1[ 9] );
    m2[ 2] = _mm256_add_epi32( m1[2], m1[10] );
    m2[ 3] = _mm256_add_epi32( m1[3], m1[11] );
    m2[ 4] = _mm256_add_epi32( m1[4], m1[12] );
    m2[ 5] = _mm256_add_epi32( m1[5], m1[13] );
    m2[ 6] = _mm256_add_epi32( m1[6], m1[14] );
    m2[ 7] = _mm256_add_epi32( m1[7], m1[15] );
    m2[ 8] = _mm256_sub_epi32( m1[0], m1[ 8] );
    m2[ 9] = _mm256_sub_epi32( m1[1], m1[ 9] );
    m2[10] = _mm256_sub_epi32( m1[2], m1[10] );
    m2[11] = _mm256_sub_epi32( m1[3], m1[11] );
    m2[12] = _mm256_sub_epi32( m1[4], m1[12] );
    m2[13] = _mm256_sub_epi32( m1[5], m1[13] );
    m2[14] = _mm256_sub_epi32( m1[6], m1[14] );
    m2[15] = _mm256_sub_epi32( m1[7], m1[15] );

    m1[ 0] = _mm256_add_epi32( m2[ 0], m2[ 4] );
    m1[ 1] = _mm256_add_epi32( m2[ 1], m2[ 5] );
    m1[ 2] = _mm256_add_epi32( m2[ 2], m2[ 6] );
    m1[ 3] = _mm256_add_epi32( m2[ 3], m2[ 7] );
    m1[ 4] = _mm256_sub_epi32( m2[ 0], m2[ 4] );
    m1[ 5] = _mm256_sub_epi32( m2[ 1], m2[ 5] );
    m1[ 6] = _mm256_sub_epi32( m2[ 2], m2[ 6] );
    m1[ 7] = _mm256_sub_epi32( m2[ 3], m2[ 7] );
    m1[ 8] = _mm256_add_epi32( m2[ 8], m2[12] );
    m1[ 9] = _mm256_add_epi32( m2[ 9], m2[13] );
    m1[10] = _mm256_add_epi32( m2[10], m2[14] );
    m1[11] = _mm256_add_epi32( m2[11], m2[15] );
    m1[12] = _mm256_sub_epi32( m2[ 8], m2[12] );
    m1[13] = _mm256_sub_epi32( m2[ 9], m2[13] );
    m1[14] = _mm256_sub_epi32( m2[10], m2[14] );
    m1[15] = _mm256_sub_epi32( m2[11], m2[15] );

    m2[ 0] = _mm256_add_epi32( m1[ 0], m1[ 2] );
    m2[ 1] = _mm256_add_epi32( m1[ 1], m1[ 3] );
    m2[ 2] = _mm256_sub_epi32( m1[ 0], m1[ 2] );
    m2[ 3] = _mm256_sub_epi32( m1[ 1], m1[ 3] );
    m2[ 4] = _mm256_add_epi32( m1[ 4], m1[ 6] );
    m2[ 5] = _mm256_add_epi32( m1[ 5], m1[ 7] );
    m2[ 6] = _mm256_sub_epi32( m1[ 4], m1[ 6] );
    m2[ 7] = _mm256_sub_epi32( m1[ 5], m1[ 7] );
    m2[ 8] = _mm256_add_epi32( m1[ 8], m1[10] );
    m2[ 9] = _mm256_add_epi32( m1[ 9], m1[11] );
    m2[10] = _mm256_sub_epi32( m1[ 8], m1[10] );
    m2[11] = _mm256_sub_epi32( m1[ 9], m1[11] );
    m2[12] = _mm256_add_epi32( m1[12], m1[14] );
    m2[13] = _mm256_add_epi32( m1[13], m1[15] );
    m2[14] = _mm256_sub_epi32( m1[12], m1[14] );
    m2[15] = _mm256_sub_epi32( m1[13], m1[15] );

    m1[ 0] = _mm256_add_epi32( m2[ 0], m2[ 1] );
    m1[ 1] = _mm256_sub_epi32( m2[ 0], m2[ 1] );
    m1[ 2] = _mm256_add_epi32( m2[ 2], m2[ 3] );
    m1[ 3] = _mm256_sub_epi32( m2[ 2], m2[ 3] );
    m1[ 4] = _mm256_add_epi32( m2[ 4], m2[ 5] );
    m1[ 5] = _mm256_sub_epi32( m2[ 4], m2[ 5] );
    m1[ 6] = _mm256_add_epi32( m2[ 6], m2[ 7] );
    m1[ 7] = _mm256_sub_epi32( m2[ 6], m2[ 7] );
    m1[ 8] = _mm256_add_epi32( m2[ 8], m2[ 9] );
    m1[ 9] = _mm256_sub_epi32( m2[ 8], m2[ 9] );
    m1[10] = _mm256_add_epi32( m2[10], m2[11] );
    m1[11] = _mm256_sub_epi32( m2[10], m2[11] );
    m1[12] = _mm256_add_epi32( m2[12], m2[13] );
    m1[13] = _mm256_sub_epi32( m2[12], m2[13] );
    m1[14] = _mm256_add_epi32( m2[14], m2[15] );
    m1[15] = _mm256_sub_epi32( m2[14], m2[15] );

    // transpose
    constexpr int perm_unpacklo_epi128 = ( 0 << 0 ) + ( 2 << 4 );
    constexpr int perm_unpackhi_epi128 = ( 1 << 0 ) + ( 3 << 4 );

    // 1. 8x8
    m2[0] = _mm256_unpacklo_epi32( m1[0], m1[1] );
    m2[1] = _mm256_unpacklo_epi32( m1[2], m1[3] );
    m2[2] = _mm256_unpacklo_epi32( m1[4], m1[5] );
    m2[3] = _mm256_unpacklo_epi32( m1[6], m1[7] );
    m2[4] = _mm256_unpackhi_epi32( m1[0], m1[1] );
    m2[5] = _mm256_unpackhi_epi32( m1[2], m1[3] );
    m2[6] = _mm256_unpackhi_epi32( m1[4], m1[5] );
    m2[7] = _mm256_unpackhi_epi32( m1[6], m1[7] );

    m1[0] = _mm256_unpacklo_epi64( m2[0], m2[1] );
    m1[1] = _mm256_unpackhi_epi64( m2[0], m2[1] );
    m1[2] = _mm256_unpacklo_epi64( m2[2], m2[3] );
    m1[3] = _mm256_unpackhi_epi64( m2[2], m2[3] );
    m1[4] = _mm256_unpacklo_epi64( m2[4], m2[5] );
    m1[5] = _mm256_unpackhi_epi64( m2[4], m2[5] );
    m1[6] = _mm256_unpacklo_epi64( m2[6], m2[7] );
    m1[7] = _mm256_unpackhi_epi64( m2[6], m2[7] );

    m2[0] = _mm256_permute2x128_si256( m1[0], m1[2], perm_unpacklo_epi128 );
    m2[1] = _mm256_permute2x128_si256( m1[0], m1[2], perm_unpackhi_epi128 );
    m2[2] = _mm256_permute2x128_si256( m1[1], m1[3], perm_unpacklo_epi128 );
    m2[3] = _mm256_permute2x128_si256( m1[1], m1[3], perm_unpackhi_epi128 );
    m2[4] = _mm256_permute2x128_si256( m1[4], m1[6], perm_unpacklo_epi128 );
    m2[5] = _mm256_permute2x128_si256( m1[4], m1[6], perm_unpackhi_epi128 );
    m2[6] = _mm256_permute2x128_si256( m1[5], m1[7], perm_unpacklo_epi128 );
    m2[7] = _mm256_permute2x128_si256( m1[5], m1[7], perm_unpackhi_epi128 );

    // 2. 8x8
    m2[0+8] = _mm256_unpacklo_epi32( m1[0+8], m1[1+8] );
    m2[1+8] = _mm256_unpacklo_epi32( m1[2+8], m1[3+8] );
    m2[2+8] = _mm256_unpacklo_epi32( m1[4+8], m1[5+8] );
    m2[3+8] = _mm256_unpacklo_epi32( m1[6+8], m1[7+8] );
    m2[4+8] = _mm256_unpackhi_epi32( m1[0+8], m1[1+8] );
    m2[5+8] = _mm256_unpackhi_epi32( m1[2+8], m1[3+8] );
    m2[6+8] = _mm256_unpackhi_epi32( m1[4+8], m1[5+8] );
    m2[7+8] = _mm256_unpackhi_epi32( m1[6+8], m1[7+8] );

    m1[0+8] = _mm256_unpacklo_epi64( m2[0+8], m2[1+8] );
    m1[1+8] = _mm256_unpackhi_epi64( m2[0+8], m2[1+8] );
    m1[2+8] = _mm256_unpacklo_epi64( m2[2+8], m2[3+8] );
    m1[3+8] = _mm256_unpackhi_epi64( m2[2+8], m2[3+8] );
    m1[4+8] = _mm256_unpacklo_epi64( m2[4+8], m2[5+8] );
    m1[5+8] = _mm256_unpackhi_epi64( m2[4+8], m2[5+8] );
    m1[6+8] = _mm256_unpacklo_epi64( m2[6+8], m2[7+8] );
    m1[7+8] = _mm256_unpackhi_epi64( m2[6+8], m2[7+8] );

    m2[0+8] = _mm256_permute2x128_si256( m1[0+8], m1[2+8], perm_unpacklo_epi128 );
    m2[1+8] = _mm256_permute2x128_si256( m1[0+8], m1[2+8], perm_unpackhi_epi128 );
    m2[2+8] = _mm256_permute2x128_si256( m1[1+8], m1[3+8], perm_unpacklo_epi128 );
    m2[3+8] = _mm256_permute2x128_si256( m1[1+8], m1[3+8], perm_unpackhi_epi128 );
    m2[4+8] = _mm256_permute2x128_si256( m1[4+8], m1[6+8], perm_unpacklo_epi128 );
    m2[5+8] = _mm256_permute2x128_si256( m1[4+8], m1[6+8], perm_unpackhi_epi128 );
    m2[6+8] = _mm256_permute2x128_si256( m1[5+8], m1[7+8], perm_unpacklo_epi128 );
    m2[7+8] = _mm256_permute2x128_si256( m1[5+8], m1[7+8], perm_unpackhi_epi128 );

    // horizontal
    m1[0] = _mm256_add_epi32( m2[0], m2[4] );
    m1[1] = _mm256_add_epi32( m2[1], m2[5] );
    m1[2] = _mm256_add_epi32( m2[2], m2[6] );
    m1[3] = _mm256_add_epi32( m2[3], m2[7] );
    m1[4] = _mm256_sub_epi32( m2[0], m2[4] );
    m1[5] = _mm256_sub_epi32( m2[1], m2[5] );
    m1[6] = _mm256_sub_epi32( m2[2], m2[6] );
    m1[7] = _mm256_sub_epi32( m2[3], m2[7] );

    m2[0] = _mm256_add_epi32( m1[0], m1[2] );
    m2[1] = _mm256_add_epi32( m1[1], m1[3] );
    m2[2] = _mm256_sub_epi32( m1[0], m1[2] );
    m2[3] = _mm256_sub_epi32( m1[1], m1[3] );
    m2[4] = _mm256_add_epi32( m1[4], m1[6] );
    m2[5] = _mm256_add_epi32( m1[5], m1[7] );
    m2[6] = _mm256_sub_epi32( m1[4], m1[6] );
    m2[7] = _mm256_sub_epi32( m1[5], m1[7] );

    m1[0] = _mm256_abs_epi32( _mm256_add_epi32( m2[0], m2[1] ) );
    m1[1] = _mm256_abs_epi32( _mm256_sub_epi32( m2[0], m2[1] ) );
    m1[2] = _mm256_abs_epi32( _mm256_add_epi32( m2[2], m2[3] ) );
    m1[3] = _mm256_abs_epi32( _mm256_sub_epi32( m2[2], m2[3] ) );
    m1[4] = _mm256_abs_epi32( _mm256_add_epi32( m2[4], m2[5] ) );
    m1[5] = _mm256_abs_epi32( _mm256_sub_epi32( m2[4], m2[5] ) );
    m1[6] = _mm256_abs_epi32( _mm256_add_epi32( m2[6], m2[7] ) );
    m1[7] = _mm256_abs_epi32( _mm256_sub_epi32( m2[6], m2[7] ) );

    int absDc = _mm_cvtsi128_si32( _mm256_castsi256_si128( m1[0] ) );

    m1[0 + 8] = _mm256_add_epi32( m2[0 + 8], m2[4 + 8] );
    m1[1 + 8] = _mm256_add_epi32( m2[1 + 8], m2[5 + 8] );
    m1[2 + 8] = _mm256_add_epi32( m2[2 + 8], m2[6 + 8] );
    m1[3 + 8] = _mm256_add_epi32( m2[3 + 8], m2[7 + 8] );
    m1[4 + 8] = _mm256_sub_epi32( m2[0 + 8], m2[4 + 8] );
    m1[5 + 8] = _mm256_sub_epi32( m2[1 + 8], m2[5 + 8] );
    m1[6 + 8] = _mm256_sub_epi32( m2[2 + 8], m2[6 + 8] );
    m1[7 + 8] = _mm256_sub_epi32( m2[3 + 8], m2[7 + 8] );

    m2[0 + 8] = _mm256_add_epi32( m1[0 + 8], m1[2 + 8] );
    m2[1 + 8] = _mm256_add_epi32( m1[1 + 8], m1[3 + 8] );
    m2[2 + 8] = _mm256_sub_epi32( m1[0 + 8], m1[2 + 8] );
    m2[3 + 8] = _mm256_sub_epi32( m1[1 + 8], m1[3 + 8] );
    m2[4 + 8] = _mm256_add_epi32( m1[4 + 8], m1[6 + 8] );
    m2[5 + 8] = _mm256_add_epi32( m1[5 + 8], m1[7 + 8] );
    m2[6 + 8] = _mm256_sub_epi32( m1[4 + 8], m1[6 + 8] );
    m2[7 + 8] = _mm256_sub_epi32( m1[5 + 8], m1[7 + 8] );

    m1[0 + 8] = _mm256_abs_epi32( _mm256_add_epi32( m2[0 + 8], m2[1 + 8] ) );
    m1[1 + 8] = _mm256_abs_epi32( _mm256_sub_epi32( m2[0 + 8], m2[1 + 8] ) );
    m1[2 + 8] = _mm256_abs_epi32( _mm256_add_epi32( m2[2 + 8], m2[3 + 8] ) );
    m1[3 + 8] = _mm256_abs_epi32( _mm256_sub_epi32( m2[2 + 8], m2[3 + 8] ) );
    m1[4 + 8] = _mm256_abs_epi32( _mm256_add_epi32( m2[4 + 8], m2[5 + 8] ) );
    m1[5 + 8] = _mm256_abs_epi32( _mm256_sub_epi32( m2[4 + 8], m2[5 + 8] ) );
    m1[6 + 8] = _mm256_abs_epi32( _mm256_add_epi32( m2[6 + 8], m2[7 + 8] ) );
    m1[7 + 8] = _mm256_abs_epi32( _mm256_sub_epi32( m2[6 + 8], m2[7 + 8] ) );

    // sum up
    m1[0] = _mm256_add_epi32( m1[0], m1[1] );
    m1[1] = _mm256_add_epi32( m1[2], m1[3] );
    m1[2] = _mm256_add_epi32( m1[4], m1[5] );
    m1[3] = _mm256_add_epi32( m1[6], m1[7] );
    m1[4] = _mm256_add_epi32( m1[8], m1[9] );
    m1[5] = _mm256_add_epi32( m1[10], m1[11] );
    m1[6] = _mm256_add_epi32( m1[12], m1[13] );
    m1[7] = _mm256_add_epi32( m1[14], m1[15] );

    // sum up
    m1[ 0] = _mm256_add_epi32( m1[ 0], m1[ 1] );
    m1[ 1] = _mm256_add_epi32( m1[ 2], m1[ 3] );
    m1[ 2] = _mm256_add_epi32( m1[ 4], m1[ 5] );
    m1[ 3] = _mm256_add_epi32( m1[ 6], m1[ 7] );

    m1[ 0] = _mm256_add_epi32( m1[ 0], m1[ 1] );
    m1[ 1] = _mm256_add_epi32( m1[ 2], m1[ 3] );

    __m256i iSum = _mm256_add_epi32( m1[0], m1[1] );

    iSum = _mm256_hadd_epi32( iSum, iSum );
    iSum = _mm256_hadd_epi32( iSum, iSum );
    iSum = _mm256_add_epi32( iSum, _mm256_permute2x128_si256( iSum, iSum, 0x11 ) );

    int sad2 = _mm_cvtsi128_si32( _mm256_castsi256_si128( iSum ) );
    sad2 -= absDc;
    sad2 += absDc >> 2;
    sad = (uint32_t)(sad2 / sqrt(16.0 * 8) * 2);
  }

#endif //USE_AVX2

  return (sad);
}

template<X86_VEXT vext >
Distortion RdCost::xGetHAD2SADs_SIMD( const DistParam &rcDtParam )
{
  Distortion distHad = xGetHADs_SIMD<vext>( rcDtParam );
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
#ifdef USE_AVX2
    // Do for width that multiple of 16
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < rows; y += subStep)
    {
      for (int x = 0; x < cols; x += 16)
      {
        __m256i vsrc1 = _mm256_lddqu_si256((__m256i *) (&src1[x]));
        __m256i vsrc2 = _mm256_lddqu_si256((__m256i *) (&src2[x]));
        __m256i vmask;
        if (rcDtParam.stepX == -1)
        {
          vmask                      = _mm256_lddqu_si256((__m256i *) ((&weightMask[x]) - (x << 1) - (16 - 1)));
          const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2,
                                                       5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vmask                      = _mm256_shuffle_epi8(vmask, shuffle_mask);
          vmask                      = _mm256_permute4x64_epi64(vmask, _MM_SHUFFLE(1, 0, 3, 2));
        }
        else
        {
          vmask = _mm256_lddqu_si256((__m256i *) (&weightMask[x]));
        }
        vsum32 = _mm256_add_epi32(vsum32, _mm256_madd_epi16(vmask, _mm256_abs_epi16(_mm256_sub_epi16(vsrc1, vsrc2))));
      }
      src1 += strideSrc1;
      src2 += strideSrc2;
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
        __m128i vsrc2 = _mm_lddqu_si128((const __m128i *) (&src2[x]));
        __m128i vmask;
        if (rcDtParam.stepX == -1)
        {
          vmask                      = _mm_lddqu_si128((__m128i *) ((&weightMask[x]) - (x << 1) - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vmask                      = _mm_shuffle_epi8(vmask, shuffle_mask);
        }
        else
        {
          vmask = _mm_lddqu_si128((const __m128i *) (&weightMask[x]));
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

template<X86_VEXT vext >
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

template <X86_VEXT vext>
void RdCost::_initRdCostX86()
{
  /* SIMD SSE implementation shifts the final sum instead of every addend
   * resulting in slightly different result compared to the scalar impl. */

  m_afpDistortFunc[0][DF_SSE    ] = xGetSSE_SIMD<vext>;
//m_afpDistortFunc[0][DF_SSE2   ] = xGetSSE_SIMD<vext>;
  m_afpDistortFunc[0][DF_SSE4   ] = xGetSSE_NxN_SIMD<4,  vext>;
  m_afpDistortFunc[0][DF_SSE8   ] = xGetSSE_NxN_SIMD<8,  vext>;
  m_afpDistortFunc[0][DF_SSE16  ] = xGetSSE_NxN_SIMD<16, vext>;
  m_afpDistortFunc[0][DF_SSE32  ] = xGetSSE_NxN_SIMD<32, vext>;
  m_afpDistortFunc[0][DF_SSE64  ] = xGetSSE_NxN_SIMD<64, vext>;
  m_afpDistortFunc[0][DF_SSE128]  = xGetSSE_NxN_SIMD<128, vext>;

  m_afpDistortFunc[0][DF_SAD    ] = xGetSAD_SIMD<vext>;
//m_afpDistortFunc[0][DF_SAD2   ] = xGetSAD_SIMD<vext>;
  m_afpDistortFunc[0][DF_SAD4   ] = xGetSAD_NxN_SIMD<4,  vext>;
  m_afpDistortFunc[0][DF_SAD8   ] = xGetSAD_NxN_SIMD<8,  vext>;
  m_afpDistortFunc[0][DF_SAD16  ] = xGetSAD_NxN_SIMD<16, vext>;
  m_afpDistortFunc[0][DF_SAD32  ] = xGetSAD_NxN_SIMD<32, vext>;
  m_afpDistortFunc[0][DF_SAD64  ] = xGetSAD_NxN_SIMD<64, vext>;
  m_afpDistortFunc[0][DF_SAD128]  = xGetSAD_NxN_SIMD<128, vext>;

  m_afpDistortFunc[0][DF_HAD]     = RdCost::xGetHADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_HAD2]    = RdCost::xGetHADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_HAD4]    = RdCost::xGetHADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_HAD8]    = RdCost::xGetHADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_HAD16]   = RdCost::xGetHADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_HAD32]   = RdCost::xGetHADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_HAD64]   = RdCost::xGetHADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_HAD128]  = RdCost::xGetHADs_SIMD<vext>;

  m_afpDistortFunc[0][DF_HAD_2SAD ] = RdCost::xGetHAD2SADs_SIMD<vext>;
  m_afpDistortFunc[0][DF_SAD_WITH_MASK] = xGetSADwMask_SIMD<vext>;
}

template void RdCost::_initRdCostX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


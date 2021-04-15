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
 * \brief Implementation of quantization functions
 */
//#define USE_AVX2
// ====================================================================================================================
// Includes
// ====================================================================================================================

#pragma once

#include "CommonDefX86.h"
#include "Rom.h"
#include "QuantRDOQ2.h"

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if USE_AVX2 && !defined( _mm256_set_m128i )
#define VVCLIB_OWN_mm256_set_m128i
#define _mm256_set_m128i( v0, v1 ) _mm256_insertf128_si256( _mm256_castsi128_si256( v1 ), ( v0 ), 1 )

#endif
#define cond_mm_prefetch(a,b) _mm_prefetch(a,b)
//#define cond_mm_prefetch(a,b)


#define QUANT_8PIX( _dst, _src, _qc, _abssum ){\
  __m128i Coeff = _mm_load_si128( (__m128i*)(_src) ); \
  __m128i AbsCoeff = _mm_abs_epi16( Coeff ); \
  __m128i DQ1 = _mm_srai_epi32( _mm_add_epi32( _mm_mullo_epi32( (__m128i&)(*_qc  )/*piQuantCoeff[n]*/,   _mm_cvtepi16_epi32( AbsCoeff )                         ), Add), iShift ); \
  __m128i DQ2 = _mm_srai_epi32( _mm_add_epi32( _mm_mullo_epi32( (__m128i&)(*(_qc+4)), _mm_cvtepi16_epi32( _mm_shuffle_epi32(AbsCoeff, 0xEE) )), Add), iShift ); \
  __m128i Comb = _mm_packs_epi32( DQ1, DQ2); \
  _abssum = _mm_add_epi16( _abssum, Comb); \
  _mm_store_si128( (__m128i*)_dst, Comb ); \
}


template< X86_VEXT vext, int iLog2TrSize >
int xQuantCGWise_SSE( short* piQCoef, short* piCbf, const short* piCoef, const int* piQuantCoeff, const int iAdd, const int iShift, const int iSize )
{
  __m128i Add = _mm_set1_epi32( iAdd );
  __m128i AbsSum = _mm_setzero_si128();
  if( iLog2TrSize == 2 )
  {
    for( int n = 0; n < iSize; n+=8 )
    {
      __m128i Coeff = _mm_load_si128( (__m128i*)&piCoef[n] );
      __m128i AbsCoeff = _mm_abs_epi16( Coeff );
      __m128i DQ1 = _mm_srai_epi32( _mm_add_epi32( _mm_mullo_epi32( (__m128i&)piQuantCoeff[n]  , _mm_cvtepi16_epi32( AbsCoeff )                         ), Add), iShift );
      __m128i DQ2 = _mm_srai_epi32( _mm_add_epi32( _mm_mullo_epi32( (__m128i&)piQuantCoeff[n+4], _mm_cvtepi16_epi32( _mm_shuffle_epi32(AbsCoeff, 0xEE) )), Add), iShift );
      __m128i Comb = _mm_packs_epi32( DQ1, DQ2);
      AbsSum = _mm_add_epi16( AbsSum, Comb);
      _mm_store_si128( (__m128i*)&piQCoef[n], Comb );
    }
    AbsSum = _mm_hadd_epi16( AbsSum, AbsSum);
    AbsSum = _mm_hadd_epi16( AbsSum, AbsSum);
    AbsSum = _mm_hadd_epi16( AbsSum, AbsSum);
    int iAbsSum = 0xffff & _mm_cvtsi128_si32( AbsSum );
    piCbf[0] = iAbsSum;
    return iAbsSum;
  }
  else if( iLog2TrSize == 3 )
  {
    //for( int n = 0, int k = 0; k < iTrSize; k+=4, n+=4*iTrSize )
    __m128i AbsSum2 = _mm_setzero_si128();
    int n = 0;
    for( ; n < iSize/2; n+=8 )
    {
      QUANT_8PIX( &piQCoef[n], &piCoef[n], &piQuantCoeff[n], AbsSum );
    }
    for( ; n < iSize; n+=8 )
    {
      QUANT_8PIX( &piQCoef[n], &piCoef[n], &piQuantCoeff[n], AbsSum2 );
    }
    __m128i Cbf = _mm_packs_epi16( AbsSum, AbsSum2 );
    _mm_store_si128( (__m128i*)piCbf, _mm_packs_epi32( Cbf, Cbf ) );
    AbsSum = _mm_add_epi16( AbsSum, AbsSum2 );
  }
  else
  {
    int stride;
    assert( iLog2TrSize == 4 || iLog2TrSize == 5 );
    if ( iLog2TrSize == 4 )
      stride = 16;
    else
      stride = 32;
    for( int n = 0; n < iSize; )
    {
      __m128i AbsSum1 = _mm_setzero_si128();
      __m128i AbsSum2 = _mm_setzero_si128();
      QUANT_8PIX( &piQCoef[n  ], &piCoef[n  ], &piQuantCoeff[n  ], AbsSum1 );
      QUANT_8PIX( &piQCoef[n+8], &piCoef[n+8], &piQuantCoeff[n+8], AbsSum2 ); n += stride;
      QUANT_8PIX( &piQCoef[n  ], &piCoef[n  ], &piQuantCoeff[n  ], AbsSum1 );
      QUANT_8PIX( &piQCoef[n+8], &piCoef[n+8], &piQuantCoeff[n+8], AbsSum2 ); n += stride;
      QUANT_8PIX( &piQCoef[n  ], &piCoef[n  ], &piQuantCoeff[n  ], AbsSum1 );
      QUANT_8PIX( &piQCoef[n+8], &piCoef[n+8], &piQuantCoeff[n+8], AbsSum2 ); n += stride;
      QUANT_8PIX( &piQCoef[n  ], &piCoef[n  ], &piQuantCoeff[n  ], AbsSum1 );
      QUANT_8PIX( &piQCoef[n+8], &piCoef[n+8], &piQuantCoeff[n+8], AbsSum2 ); 
      if( iLog2TrSize == 4 ) n += 16;
      else                   n += 32 + 16 - (32*4);
      AbsSum = _mm_add_epi16( AbsSum, AbsSum1 );
      AbsSum = _mm_add_epi16( AbsSum, AbsSum2 );
      __m128i Cbf = _mm_packs_epi16( AbsSum1, AbsSum2 );
      AbsSum1 = _mm_setzero_si128();
      AbsSum2 = _mm_setzero_si128();
      QUANT_8PIX( &piQCoef[n  ], &piCoef[n  ], &piQuantCoeff[n  ], AbsSum1 );
      QUANT_8PIX( &piQCoef[n+8], &piCoef[n+8], &piQuantCoeff[n+8], AbsSum2 ); n += stride;
      QUANT_8PIX( &piQCoef[n  ], &piCoef[n  ], &piQuantCoeff[n  ], AbsSum1 );
      QUANT_8PIX( &piQCoef[n+8], &piCoef[n+8], &piQuantCoeff[n+8], AbsSum2 ); n += stride;
      QUANT_8PIX( &piQCoef[n  ], &piCoef[n  ], &piQuantCoeff[n  ], AbsSum1 );
      QUANT_8PIX( &piQCoef[n+8], &piCoef[n+8], &piQuantCoeff[n+8], AbsSum2 ); n += stride;
      QUANT_8PIX( &piQCoef[n  ], &piCoef[n  ], &piQuantCoeff[n  ], AbsSum1 );
      QUANT_8PIX( &piQCoef[n+8], &piCoef[n+8], &piQuantCoeff[n+8], AbsSum2 ); n += 16;
      AbsSum = _mm_add_epi16( AbsSum, AbsSum1 );
      AbsSum = _mm_add_epi16( AbsSum, AbsSum2 );
      Cbf = _mm_packs_epi32( Cbf, _mm_packs_epi16( AbsSum1, AbsSum2 ) );
      _mm_store_si128( (__m128i*)&piCbf[(n-128)>>4], Cbf );
    }
  }

  AbsSum = _mm_hadd_epi16( AbsSum, AbsSum);
  AbsSum = _mm_hadd_epi16( AbsSum, AbsSum);
  AbsSum = _mm_hadd_epi16( AbsSum, AbsSum);
  return (0xffff & _mm_cvtsi128_si32( AbsSum ));
}

static constexpr unsigned short levmask[16] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0,0,0,0,0,0};
template<X86_VEXT vext>
static void DeQuantCoreSIMD(const int maxX,const int maxY,const int scale,const TCoeff   *const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum)
{
  const int inputMinimum = -(inputMaximum+1);
  const TCoeff transformMinimum = -(transformMaximum+1);
  const int width = maxX+1;

  __m128i vlevmask;
  if (maxX<7)
    vlevmask = _mm_loadu_si128( ( __m128i const * )&levmask[7-maxX] );
  else
    vlevmask = _mm_set_epi64x(0xffffffffffffffff,0xffffffffffffffff);

  if (rightShift>0)
  {
    const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);

    __m128i v_max =  _mm_set1_epi16 ((short)inputMaximum);
    __m128i v_min =  _mm_set1_epi16 ((short)inputMinimum);
    __m128i v_Tmax =  _mm_set1_epi32 ((short)transformMaximum);
    __m128i v_Tmin =  _mm_set1_epi32 ((short)transformMinimum);
    __m128i v_scale = _mm_set1_epi16 ((short)scale);
    __m128i v_add = _mm_set1_epi32 (iAdd);
    __m128i v_rshift = _mm_set1_epi64x (rightShift);

    if (maxX<4)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = maxX == 1 ? _mm_loadl_epi64( (__m128i const*) & piQCoef[y * piQCfStride] ) : _mm_loadu_si128( (__m128i const*) & piQCoef[y * piQCfStride] );
        v_level = _mm_packs_epi32 (v_level,v_level);
        v_level = _mm_and_si128(v_level,vlevmask);
        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level =  _mm_add_epi32(v_level,v_add);
        v_level = _mm_sra_epi32(v_level,v_rshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        if( maxX == 1 )
          _mm_storel_epi64( (__m128i*)(piCoef + y * width), v_level );
        else
          _mm_storeu_si128( (__m128i*)(piCoef + y * width), v_level );
      }
    }
    else
    {
      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=8)
        {
          __m128i v_levell = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+ y * piQCfStride]  );
          __m128i v_levelh = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+4 + y * piQCfStride]  );
          __m128i v_level = _mm_packs_epi32 (v_levell,v_levelh);
          v_level = _mm_and_si128(v_level,vlevmask);
          v_level = _mm_max_epi16 (v_level, v_min);
          v_level = _mm_min_epi16 (v_level, v_max);
          __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
          __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

          v_level = _mm_unpacklo_epi16(v_low,v_high);
          v_level =  _mm_add_epi32(v_level,v_add);
          v_level = _mm_sra_epi32(v_level,v_rshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+x+y*width ), v_level );

          v_level = _mm_unpackhi_epi16(v_low,v_high);
          v_level =  _mm_add_epi32(v_level,v_add);
          v_level = _mm_sra_epi32(v_level,v_rshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+4+x+y*width ), v_level );
        }
      }
    }
  }
  else  // rightshift <0
  {
    __m128i v_max =  _mm_set1_epi16 ((short)inputMaximum);
    __m128i v_min =  _mm_set1_epi16 ((short)inputMinimum);
    __m128i v_Tmax =  _mm_set1_epi32 ((short)transformMaximum);
    __m128i v_Tmin =  _mm_set1_epi32 ((short)transformMinimum);
    __m128i v_scale = _mm_set1_epi16 ((short)scale);
    __m128i v_lshift = _mm_set1_epi64x (-rightShift);

    if (maxX<4)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = maxX == 1 ? _mm_loadl_epi64( (__m128i const*) & piQCoef[y * piQCfStride] ) : _mm_loadu_si128( ( __m128i const * )&piQCoef[y * piQCfStride]  );
        v_level = _mm_packs_epi32 (v_level,v_level);
        v_level = _mm_and_si128(v_level,vlevmask);

        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level = _mm_sll_epi32(v_level,v_lshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        if( maxX == 1 )
        {
          _mm_storel_epi64( (__m128i*)(piCoef + y * width), v_level );
        }
        else
          _mm_storeu_si128(( __m128i * )(piCoef+y*width ), v_level );
      }
    }
    else
    {
      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=8)
        {
          __m128i v_levell = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+ y * piQCfStride]  );
          __m128i v_levelh = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+4 + y * piQCfStride]  );
          __m128i v_level = _mm_packs_epi32 (v_levell,v_levelh);
          v_level = _mm_and_si128(v_level,vlevmask);
          v_level = _mm_max_epi16 (v_level, v_min);
          v_level = _mm_min_epi16 (v_level, v_max);
          __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
          __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

          v_level = _mm_unpacklo_epi16(v_low,v_high);
          v_level = _mm_sll_epi32(v_level,v_lshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+x+y*width ), v_level );

          v_level = _mm_unpackhi_epi16(v_low,v_high);
          v_level = _mm_sll_epi32(v_level,v_lshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+4+x+y*width ), v_level );
        }
      }
    }
  }
}


template <X86_VEXT vext>
void QuantRDOQ2::_initQuantX86()
{
  m_pQuantToNearestInt[0] = xQuantCGWise_SSE<vext, 2>;
  m_pQuantToNearestInt[1] = xQuantCGWise_SSE<vext, 3>;
  m_pQuantToNearestInt[2] = xQuantCGWise_SSE<vext, 4>;
  m_pQuantToNearestInt[3] = xQuantCGWise_SSE<vext, 5>;
}

template void QuantRDOQ2::_initQuantX86<SIMDX86>();

template<X86_VEXT vext>
void Quant::_initQuantX86()
{
  DeQuant = DeQuantCoreSIMD<vext>;
}
template void Quant::_initQuantX86<SIMDX86>();


} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


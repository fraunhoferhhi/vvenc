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

#define cond_mm_prefetch(a,b) _mm_prefetch(a,b)
//#define cond_mm_prefetch(a,b)

static constexpr unsigned short levmask[16] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0,0,0,0,0,0};
template<X86_VEXT vext>
static void DeQuantCoreSIMD(const int maxX,const int maxY,const int scale,const TCoeffSig *const piQCoef,const size_t piQCfStride,TCoeff *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum)
{
  // TODO: TCoeffSig!!!
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
        __m128i v_level = maxX == 1 ? _mm_set1_epi32( *( ( int const* ) & piQCoef[y * piQCfStride] ) ) : _mm_loadl_epi64( ( __m128i const* ) &piQCoef[y * piQCfStride] );
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
          __m128i v_level = _mm_loadu_si128( ( __m128i const* ) &piQCoef[x + y * piQCfStride] );
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
        __m128i v_level = maxX == 1 ? _mm_set1_epi32( *( ( int const* ) & piQCoef[y * piQCfStride] ) ) : _mm_loadl_epi64( ( __m128i const* ) &piQCoef[y * piQCfStride] );
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
          __m128i v_level = _mm_loadu_si128( ( __m128i const* ) &piQCoef[x + y * piQCfStride] );
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
template<X86_VEXT vext>
static void QuantCoreSIMD(const CCoeffBuf&  piCoef,CoeffSigBuf piQCoef,TCoeff &uiAbsSum,TCoeff *deltaU,const int maxNumberOfCoeffs,const int defaultQuantisationCoefficient,const int iQBits,const int64_t iAdd,const TCoeff entropyCodingMinimum,const TCoeff entropyCodingMaximum ,const bool signHiding)
{
  const int qBits8 = iQBits - 8;

  piQCoef.memset( 0 );

  if( maxNumberOfCoeffs >= 8 )
  {
    if( vext >= AVX2 )
    {
#ifdef USE_AVX2
      printf("AVX2 maxNumberOfCoeffs %d \n",maxNumberOfCoeffs);
#endif
    }  //AVX2
    else
    {
      __m128i vNull = _mm_set_epi32(0,0,0,0);
      __m128i vQuantCoeff = _mm_set_epi32(defaultQuantisationCoefficient,defaultQuantisationCoefficient,defaultQuantisationCoefficient,defaultQuantisationCoefficient);
      __m128i vAdd = _mm_set_epi64(iAdd,iAdd);
      __m128i vShift = _mm_set_epi64(iQBits,iQBits);
      __m128i vqBits8 = _mm_set_epi64(qBits8,qBits8);
      __m128i vMin = _mm_set_epi32(entropyCodingMinimum,entropyCodingMinimum,entropyCodingMinimum,entropyCodingMinimum);
      __m128i vMax = _mm_set_epi32(entropyCodingMaximum,entropyCodingMaximum,entropyCodingMaximum,entropyCodingMaximum);



      printf("SSE maxNumberOfCoeffs %d \n",maxNumberOfCoeffs);
      for (int uiBlockPos = 0; uiBlockPos < maxNumberOfCoeffs; uiBlockPos+=4 )
      {
        __m128i vLevel = _mm_lddqu_si128((__m128i*)&piCoef.buf[uiBlockPos]);     // coeff3,coeff2,coeff1,coeff0,
        __m128i vSign = _mm_cmpgt_epi32 (vNull,vLevel);                                            // sign3,sign2,sign1,sign0 FFFF or 0000
        vLevel = _mm_abs_epi32 (vLevel);                                                                        // abs(vLevel3,vLevel2,vLevel1,vLevel0)
        __m128i vdeltaU0 = _mm_mul_epu32(vLevel,vQuantCoeff);                      // Tmp2,Tmp0
        __m128i vdeltaU1 = _mm_bsrli_si128(vLevel,4)                                            // abs(0,vLevel3,vLevel2,vLevel1)
        __m128i vTmpLevel_1 = _mm_mul_epu32(vdeltaU1,vQuantCoeff);                          // Tmp3,Tmp1
        __m128i vTmpLevel_0 = _mm_add_epi64(vdeltaU0,vAdd)
        vTmpLevel_1 = _mm_add_epi64(vTmpLevel_1,vAdd)
        vTmpLevel_0 = _mm_srl_epi64(vTmpLevel_0,iQBits);                                         // Int32 Tmp2,Tmp0
        vTmpLevel_1 = _mm_srl_epi64(vTmpLevel_1,iQBits);                                         // Int32 Tmp3,Tmp1
        if (signHiding)
        {
          __m128i vBS0 = _mm_sll_epi32(vTmpLevel_0,iQBits);
          __m128i vBS1 = _mm_sll_epi32(vTmpLevel_1,iQBits);
          vdeltaU0 = _mm_sub_epi64(vdeltaU0,vBS0);
          vdeltaU1 = _mm_sub_epi64(vdeltaU1,vBS1);
          vdeltaU0 = _mm_srl_epi64(vdeltaU0,vqBits8);
          vdeltaU1 = _mm_srl_epi64(vdeltaU1,vqBits8);
          vdeltaU0 = _mm_unpacklo_epi32(vdeltaU0,vdeltaU1);
          _mm_storeu_si128( ( __m128i * )&deltaU[uiBlockPos],vdeltaU0);

        }
        vTmpLevel_0 =  _mm_unpacklo_epi32(vTmpLevel_0,vTmpLevel_1);                 // quantisedMagnitude Int32 Tmp3,Tmp2,Tmp1,Tmp0
        __m128i vAbsSum = _mm_hadd_epi32(vTmpLevel_0,vTmpLevel_0);
        vAbsSum = _mm_hadd_epi32(vTmpLevel_0,vTmpLevel_0);
        _mm_storeu_si32 (( __m128i * )&uiAbsSum,vAbsSum);
        vTmpLevel_1 =   _mm_and_si128(vTmpLevel_0,vSign);                                       // mask only neg values
        vTmpLevel_0 =   _mm_andnot_si128(vSign,vTmpLevel_0);                                 // mask only pos values
        vTmpLevel_0 =   _mm_sub_epi32(vTmpLevel_0,vTmpLevel_1);
        vTmpLevel_0 =  _mm_min_epi32(vMax, _mm_max_epi32(vMin,vTmpLevel_0))  // clip
        _mm_storeu_si128( ( __m128i * )&piQCoef.buf[uiBlockPos],vTmpLevel_0);
      }

    }
  }
  else
    printf("scalar maxNumberOfCoeffs %d \n",maxNumberOfCoeffs);
#if 0
  for (int uiBlockPos = 0; uiBlockPos < maxNumberOfCoeffs; uiBlockPos++ )
  {
    const TCoeff iLevel   = piCoef.buf[uiBlockPos];
    const TCoeff iSign    = (iLevel < 0 ? -1: 1);

    const int64_t  tmpLevel = (int64_t)abs(iLevel) * defaultQuantisationCoefficient;
    const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);
    if (signHiding)
      deltaU[uiBlockPos] = (TCoeff)((tmpLevel - ((int64_t)quantisedMagnitude<<iQBits) )>> qBits8);

    uiAbsSum += quantisedMagnitude;
    const TCoeff quantisedCoefficient = quantisedMagnitude * iSign;

    piQCoef.buf[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
  } // for n
#endif
}

template<X86_VEXT vext>
void Quant::_initQuantX86()
{
  DeQuant = DeQuantCoreSIMD<vext>;
  xQuant=QuantCoreSIMD<vext>;
}
template void Quant::_initQuantX86<SIMDX86>();


} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


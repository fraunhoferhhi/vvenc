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
void print128_16(__m128i var)
{
    uint16_t val[8];
    memcpy(val, &var, sizeof(val));
    printf("Numerical: %i %i %i %i %i %i %i %i \n",
           val[0], val[1], val[2], val[3], val[4], val[5],
           val[6], val[7]);
}
template<X86_VEXT vext>
void print128_32(__m128i var)
{
    uint32_t val[4];
    memcpy(val, &var, sizeof(val));
    printf("%x %x %x %x\n",
           val[0], val[1], val[2], val[3]);
}
template<X86_VEXT vext>
void print128_64(__m128i var)
{
    uint64_t val[2];
    memcpy(val, &var, sizeof(val));
    printf("%lx %lx\n",
           val[0], val[1]);
}
template<X86_VEXT vext>
void print256_32(__m256i var)
{
    uint32_t val[8];
    memcpy(val, &var, sizeof(val));
    printf("%x %x %x %x %x %x %x %x\n",
        val[0], val[1], val[2], val[3],val[4], val[5], val[6], val[7]);
}
template<X86_VEXT vext>
void print256_16_1(__m256i var)
{
    uint16_t val[16];
    memcpy(val, &var, sizeof(val));
    printf("%x %x %x %x %x %x %x %x  \n",
        val[0], val[1], val[2], val[3],val[4], val[5], val[6], val[7]);
}
template<X86_VEXT vext>
void print256_64(__m256i var)
{
    uint64_t val[4];
    memcpy(val, &var, sizeof(val));
    printf("%lx %lx %lx %lx\n",
        val[0], val[1],val[2], val[3]);
}
#define _mm_storeu_si32(p, a) (void)(*(int*)(p) = _mm_cvtsi128_si32((a)))
static int MyCount=0;
template<X86_VEXT vext>
static void QuantCoreSIMD(const CCoeffBuf&  piCoef,CoeffSigBuf piQCoef,TCoeff &uiAbsSum,TCoeff *deltaU,const int maxNumberOfCoeffs,const int defaultQuantisationCoefficient,const int iQBits,const int64_t iAdd,const TCoeff entropyCodingMinimum,const TCoeff entropyCodingMaximum ,const bool signHiding)
{
  const int qBits8 = iQBits - 8;
  int16_t tmpArray[4096];

  int tmpDelta[4096];

  piQCoef.memset( 0 );
//#if 1
  if( maxNumberOfCoeffs >= 8 )
  {
    if( vext >= AVX2 )
    {
#ifdef USE_AVX2
      __m256i vNull = _mm256_set1_epi32(0);
      __m256i vQuantCoeff = _mm256_set1_epi32(defaultQuantisationCoefficient);
      __m256i vAdd = _mm256_set_epi64x(iAdd,iAdd,iAdd,iAdd);
      __m128i vQBits = _mm_set_epi64x(iQBits,iQBits);
      __m256i vMask = _mm256_set_epi64x(0xffffffff,0xffffffff,0xffffffff,0xffffffff);
      __m256i vMax = _mm256_set1_epi32(entropyCodingMaximum);
      __m256i vMin = _mm256_set1_epi32(entropyCodingMinimum);
      __m128i vqBits8 = _mm_set_epi64x(qBits8,qBits8);

      //printf("AVX2 maxNumberOfCoeffs %d \n",maxNumberOfCoeffs);
      for (int uiBlockPos = 0; uiBlockPos < maxNumberOfCoeffs; uiBlockPos+=8 )
      {
        __m256i  vLevel = _mm256_loadu_si256((__m256i *)&piCoef.buf[uiBlockPos]);     // coeff7,coeff6,coeff5,coeff4,coeff3,coeff2,coeff1,coeff0,
        __m256i vSign = _mm256_cmpgt_epi32 (vNull,vLevel);                                            // sign3,sign2,sign1,sign0 FFFF or 0000
        vLevel = _mm256_abs_epi32 (vLevel);
        __m256i vdeltaU0 = _mm256_mul_epu32(vLevel,vQuantCoeff);                      // Tmp2,Tmp0
        __m256i  vdeltaU1 = _mm256_srli_si256(vLevel,4);                                            // abs(0,vLevel3,vLevel2,vLevel1)
        vdeltaU1 = _mm256_mul_epu32(vdeltaU1,vQuantCoeff);                          // Tmp3,Tmp1
        __m256i vTmpLevel_0 = _mm256_add_epi64(vdeltaU0,vAdd);
        __m256i vTmpLevel_1 = _mm256_add_epi64(vdeltaU1,vAdd);
        vTmpLevel_0 = _mm256_srl_epi64 (vTmpLevel_0,vQBits);                                         // Int32 Tmp2,Tmp0
        vTmpLevel_1 = _mm256_srl_epi64 (vTmpLevel_1,vQBits);                                         // Int32 Tmp3,Tmp1
        //printf("\n");
        //print256_64<vext>(vTmpLevel_0);print256_64<vext>(vTmpLevel_1);

        if (signHiding)
        {
         // printf("signHiding\n");
          __m256i vBS0 = _mm256_sll_epi64(vTmpLevel_0,vQBits);
          __m256i vBS1 = _mm256_sll_epi64(vTmpLevel_1,vQBits);

          //print128_32<vext>(vTmpLevel_0);print128_32<vext>(vTmpLevel_1);

          //print128_64<vext>(vBS0);print128_64<vext>(vBS1);
         // print128_64<vext>(vdeltaU0);print128_64<vext>(vdeltaU1);
          vdeltaU0 = _mm256_sub_epi64(vdeltaU0,vBS0);
          vdeltaU1 = _mm256_sub_epi64(vdeltaU1,vBS1);
          //print128_64<vext0000>(vdeltaU0);print128_64<vext>(vdeltaU1);

          vdeltaU0 = _mm256_srl_epi64(vdeltaU0,vqBits8);
          vdeltaU1 = _mm256_srl_epi64(vdeltaU1,vqBits8);
          //print128_64<vext>(vdeltaU0);print128_64<vext>(vdeltaU1);

          vdeltaU0 =  _mm256_and_si256(vdeltaU0,vMask);
          vdeltaU1 =  _mm256_and_si256(vdeltaU1,vMask);
          vdeltaU1 =   _mm256_slli_epi64(vdeltaU1,32);
          vdeltaU0 = _mm256_or_si256(vdeltaU0,vdeltaU1);
          //print256_32<vext>(vdeltaU0);
          _mm256_storeu_si256( ( __m256i * )&deltaU[uiBlockPos],vdeltaU0);
          //_mm256_storeu_si256( ( __m256i * )&tmpDelta[uiBlockPos],vdeltaU0);
          //_mm_storeu_si128( ( __m128i * )&tmpDelta[upiQCoef.bufiBlockPos],vdeltaU0);
          //print128_32<vext>(vdeltaU0);
        }
        __m256i vquantMag0 =  _mm256_and_si256(vTmpLevel_0,vMask);
        __m256i vquantMag1 =  _mm256_and_si256(vTmpLevel_1,vMask);
        vquantMag1 =   _mm256_slli_epi64(vquantMag1,32);
        vTmpLevel_0 = _mm256_or_si256(vquantMag0,vquantMag1);
        //print128_32<vext>(vTmpLevel_0);
        __m256i vAbsSum = _mm256_hadd_epi32(vTmpLevel_0,vTmpLevel_0);
        //print256_32<vext>(vAbsSum);
        vAbsSum = _mm256_hadd_epi32(vAbsSum,vAbsSum);
        //print256_32<vext>(vAbsSum);

        uiAbsSum+=_mm256_extract_epi32(vAbsSum,0);
        //printf("sum %x \n",uiAbsSum);
        uiAbsSum+=_mm256_extract_epi32(vAbsSum,4);

        //printf("pos %d %x\n",uiBlockPos+7,uiAbsSum);
        vTmpLevel_1 =   _mm256_and_si256(vTmpLevel_0,vSign);                                       // mask only neg values
        vTmpLevel_0 =   _mm256_andnot_si256(vSign,vTmpLevel_0);                                 // mask only pos values
        vTmpLevel_0 =   _mm256_sub_epi32(vTmpLevel_0,vTmpLevel_1);
        vTmpLevel_0 =  _mm256_min_epi32(vMax, _mm256_max_epi32(vMin,vTmpLevel_0));  // clip to 16 Bit
        vTmpLevel_0 =  _mm256_packs_epi32(vTmpLevel_0,vTmpLevel_0);
        //print256_16<vext>(vTmpLevel_0);
        vTmpLevel_0 =   _mm256_insert_epi64 ( vTmpLevel_0 , _mm256_extract_epi64(vTmpLevel_0, 2),1);
        _mm_storeu_si128( ( __m128i * )&piQCoef.buf[uiBlockPos],_mm256_castsi256_si128(vTmpLevel_0));
        //_mm_storeu_si128( ( __m128i * )&tmpArray[uiBlockPos],_mm256_castsi256_si128(vTmpLevel_0));
        //print256_16_1<vext>(vTmpLevel_0);
       //printf("%x %x %x %x   %x %x %x %x \n",piQCoef.buf[uiBlockPos],piQCoef.buf[uiBlockPos+1],piQCoef.buf[uiBlockPos+2],piQCoef.buf[uiBlockPos+3],piQCoef.buf[uiBlockPos+4],piQCoef.buf[uiBlockPos+5],piQCoef.buf[uiBlockPos+6],piQCoef.buf[uiBlockPos+7]);


      }
#endif
    }  //AVX2
    else
    {
      printf("SSE maxNumberOfCoeffs %d \n",maxNumberOfCoeffs);
      //printf("Block %d \n",MyCount++);
      __m128i vNull = _mm_set_epi32(0,0,0,0);
      __m128i vQuantCoeff = _mm_set_epi32(defaultQuantisationCoefficient,defaultQuantisationCoefficient,defaultQuantisationCoefficient,defaultQuantisationCoefficient);
      __m128i vAdd = _mm_set_epi64x(iAdd,iAdd);
      __m128i vQBits = _mm_set_epi64x(iQBits,iQBits);
      __m128i vqBits8 = _mm_set_epi64x(qBits8,qBits8);
      __m128i vMin = _mm_set_epi32(entropyCodingMinimum,entropyCodingMinimum,entropyCodingMinimum,entropyCodingMinimum);
      __m128i vMax = _mm_set_epi32(entropyCodingMaximum,entropyCodingMaximum,entropyCodingMaximum,entropyCodingMaximum);
      __m128i vMask = _mm_set_epi64x(0xffffffff,0xffffffff);



      //printf("SSE maxNumberOfCoeffs %d \n",maxNumberOfCoeffs);
      for (int uiBlockPos = 0; uiBlockPos < maxNumberOfCoeffs; uiBlockPos+=4 )
      {
        __m128i vLevel = _mm_lddqu_si128((__m128i*)&piCoef.buf[uiBlockPos]);     // coeff3,coeff2,coeff1,coeff0,
        //print128_32<vext>(vLevel);
        __m128i vSign = _mm_cmpgt_epi32 (vNull,vLevel);                                            // sign3,sign2,sign1,sign0 FFFF or 0000
        //print128_32<vext>(vSign);
        vLevel = _mm_abs_epi32 (vLevel);                                                                        // abs(vLevel3,vLevel2,vLevel1,vLevel0)
        __m128i vdeltaU0 = _mm_mul_epu32(vLevel,vQuantCoeff);                      // Tmp2,Tmp0
        __m128i vdeltaU1 = _mm_bsrli_si128(vLevel,4);                                            // abs(0,vLevel3,vLevel2,vLevel1)
        vdeltaU1 = _mm_mul_epu32(vdeltaU1,vQuantCoeff);                          // Tmp3,Tmp1
        //print128_64<vext>(vdeltaU0);print128_64<vext>(vTmpLevel_1);
        __m128i vTmpLevel_0 = _mm_add_epi64(vdeltaU0,vAdd);
        __m128i vTmpLevel_1 = _mm_add_epi64(vdeltaU1,vAdd);
        vTmpLevel_0 = _mm_srl_epi64(vTmpLevel_0,vQBits);                                         // Int32 Tmp2,Tmp0
        vTmpLevel_1 = _mm_srl_epi64(vTmpLevel_1,vQBits);                                         // Int32 Tmp3,Tmp1
        //if (uiBlockPos%8 ==0 )
        //      printf("\n");
       // print128_64<vext>(vTmpLevel_0);print128_64<vext>(vTmpLevel_1);

        if (signHiding)
        {

         // printf("signHiding\n");
          __m128i vBS0 = _mm_sll_epi64(vTmpLevel_0,vQBits);
          __m128i vBS1 = _mm_sll_epi64(vTmpLevel_1,vQBits);

          //print128_32<vext>(vTmpLevel_0);print128_32<vext>(vTmpLevel_1);

          //print128_64<vext>(vBS0);print128_64<vext>(vBS1);
         // print128_64<vext>(vdeltaU0);print128_64<vext>(vdeltaU1);
          vdeltaU0 = _mm_sub_epi64(vdeltaU0,vBS0);
          vdeltaU1 = _mm_sub_epi64(vdeltaU1,vBS1);
          //print128_64<vext0000>(vdeltaU0);print128_64<vext>(vdeltaU1);

          vdeltaU0 = _mm_srl_epi64(vdeltaU0,vqBits8);
          vdeltaU1 = _mm_srl_epi64(vdeltaU1,vqBits8);
          //print128_64<vext>(vdeltaU0);print128_64<vext>(vdeltaU1);

          vdeltaU0 =  _mm_and_si128(vdeltaU0,vMask);
          vdeltaU1 =  _mm_and_si128(vdeltaU1,vMask);
          vdeltaU1 =   _mm_slli_epi64(vdeltaU1,32);
          vdeltaU0 = _mm_or_si128(vdeltaU0,vdeltaU1);
         _mm_storeu_si128( ( __m128i * )&deltaU[uiBlockPos],vdeltaU0);
          //_mm_storeu_si128( ( __m128i * )&tmpDelta[uiBlockPos],vdeltaU0);
          //print128_32<vext>(vdeltaU0);
        }

        __m128i vquantMag0 =  _mm_and_si128(vTmpLevel_0,vMask);
        __m128i vquantMag1 =  _mm_and_si128(vTmpLevel_1,vMask);
        vquantMag1 =   _mm_slli_epi64(vquantMag1,32);
        vTmpLevel_0 = _mm_or_si128(vquantMag0,vquantMag1);
        //print128_32<vext>(vTmpLevel_0);
        __m128i vAbsSum = _mm_hadd_epi32(vTmpLevel_0,vTmpLevel_0);
        vAbsSum += _mm_hadd_epi32(vAbsSum,vAbsSum);
        uiAbsSum+=_mm_extract_epi32(vAbsSum,0);
        //printf("tmpDeltapos %d %x\n",uiBlockPos+3,uiAbsSum);
        vTmpLevel_1 =   _mm_and_si128(vTmpLevel_0,vSign);                                       // mask only neg values
        vTmpLevel_0 =   _mm_andnot_si128(vSign,vTmpLevel_0);                                 // mask only pos values
        vTmpLevel_0 =   _mm_sub_epi32(vTmpLevel_0,vTmpLevel_1);
        vTmpLevel_0 =  _mm_min_epi32(vMax, _mm_max_epi32(vMin,vTmpLevel_0));  // clip to 16 Bit
        vTmpLevel_0 =  _mm_packs_epi32(vTmpLevel_0,vTmpLevel_0);
       _mm_storeu_si64( ( __m128i * )&piQCoef.buf[uiBlockPos],vTmpLevel_0);
        //_mm_storeu_si64( ( __m128i * )&tmpArray[uiBlockPos],vTmpLevel_0);
        //print128_16<vext>(vTmpLevel_0);
        //printf("%x %x %x %x \n",tmpArray[uiBlockPos],tmpArray[uiBlockPos+1],tmpArray[uiBlockPos+2],tmpArray[uiBlockPos+3]);
        //printf("%x %x %x %x \n",piQCoef.buf[uiBlockPos],piQCoef.buf[uiBlockPos+1],piQCoef.buf[uiBlockPos+2],piQCoef.buf[uiBlockPos+3]);
        //exit(1);
      }

    }
  }
  else
    printf("scalar maxNumberOfCoeffs %d \n",maxNumberOfCoeffs);
//#else

  for (int uiBlockPos = 0; uiBlockPos < maxNumberOfCoeffs; uiBlockPos++ )
  {
    const TCoeff iLevel   = piCoef.buf[uiBlockPos];
    const TCoeff iSign    = (iLevel < 0 ? -1: 1);
//    if (uiBlockPos%4 ==0 )
//      printf("\n");
    const int64_t  tmpLevel = (int64_t)abs(iLevel) * defaultQuantisationCoefficient;
    const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);
    //printf("%lx ",tmpLevel);

    if (signHiding)
    {
      //printf("signHiding\n");
      //printf("%x ",quantisedMagnitude<<iQBits);
      //printf("%lx ",(tmpLevel - ((int64_t)quantisedMagnitude<<iQBits) ));

      //deltaU[uiBlockPos] = (TCoeff)((tmpLevel - ((int64_t)quantisedMagnitude<<iQBits) )>> qBits8);
      //tmpDelta[uiBlockPos] = (TCoeff)((tmpLevel - ((int64_t)quantisedMagnitude<<iQBits) )>> qBits8);
      //printf("%x ",deltaU[uiBlockPos]);

    }
    //uiAbsSum += quantisedMagnitude;
    //printf("pos %d %x\n",uiBlockPos,uiAbsSum);
    const TCoeff quantisedCoefficient = quantisedMagnitude * iSign;

    //piQCoef.buf[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
    //tmpArray[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
    //printf("%x ",piQCoef.buf[uiBlockPos] );
  } // for n
//#endif
#if 0

  for (int uiBlockPos = 0; uiBlockPos < maxNumberOfCoeffs; uiBlockPos++ )
  {
    if (tmpArray[uiBlockPos] != piQCoef.buf[uiBlockPos] )
    {
      printf("ERROR: pos %d  %x %x \n",uiBlockPos,tmpArray[uiBlockPos],piQCoef.buf[uiBlockPos]);
      exit(1);
    }
  }
   for (int uiBlockPos = 0; uiBlockPos < maxNumberOfCoeffs; uiBlockPos++ )
    {
      if (tmpDelta[uiBlockPos] != deltaU[uiBlockPos])
      {
        printf("Delta ERROR: pos %d %x %x \n",uiBlockPos,tmpDelta[uiBlockPos],deltaU[uiBlockPos]);
        exit(1);
      }
  }
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


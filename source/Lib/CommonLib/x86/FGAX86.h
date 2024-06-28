/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     FGAX86.h
    \brief    SIMD for FilmGrainAnalyse
*/

//! \ingroup CommonLib
//! \{

//#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "SEIFilmGrainAnalyzer.h"

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {
#ifdef USE_AVX2
/* -----------------------------------------------------------------------------
atan2 aproximation taken from:
https://mazzo.li/posts/vectorized-atan2.html
------------------------------------------------------------------------------------------- */
inline __m256 atan_avx_approximation(__m256 x) {
  // Store the coefficients -- `_mm256_set1_ps` creates a vector
  // with the same value in every element.
  __m256 a1  = _mm256_set1_ps( 0.99997726f);
  __m256 a3  = _mm256_set1_ps(-0.33262347f);
  __m256 a5  = _mm256_set1_ps( 0.19354346f);
  __m256 a7  = _mm256_set1_ps(-0.11643287f);
  __m256 a9  = _mm256_set1_ps( 0.05265332f);
  __m256 a11 = _mm256_set1_ps(-0.01172120f);
  // Compute the polynomial on an 8-vector with FMA.
  __m256 x_sq = _mm256_mul_ps(x, x);
  __m256 result;
  result = a11;
  result = _mm256_add_ps(_mm256_mul_ps(x_sq, result), a9);
  result = _mm256_add_ps(_mm256_mul_ps(x_sq, result), a7);
  result = _mm256_add_ps(_mm256_mul_ps(x_sq, result), a5);
  result = _mm256_add_ps(_mm256_mul_ps(x_sq, result), a3);
  result = _mm256_add_ps(_mm256_mul_ps(x_sq, result), a1);
  result = _mm256_mul_ps(x, result);
  return result;
}
#endif
inline __m128 atan_avx_approximation(__m128 x) {
  // Store the coefficients -- `_mm256_set1_ps` creates a vector
  // with the same value in every element.
  __m128 a1  = _mm_set1_ps( 0.99997726f);
  __m128 a3  = _mm_set1_ps(-0.33262347f);
  __m128 a5  = _mm_set1_ps( 0.19354346f);
  __m128 a7  = _mm_set1_ps(-0.11643287f);
  __m128 a9  = _mm_set1_ps( 0.05265332f);
  __m128 a11 = _mm_set1_ps(-0.01172120f);
  // Compute the polynomial on an 8-vector with FMA.
  __m128 x_sq = _mm_mul_ps(x, x);
  __m128 result;
  result = a11;
  result = _mm_add_ps(_mm_mul_ps(x_sq, result), a9);
  result = _mm_add_ps(_mm_mul_ps(x_sq, result), a7);
  result = _mm_add_ps(_mm_mul_ps(x_sq, result), a5);
  result = _mm_add_ps(_mm_mul_ps(x_sq, result), a3);
  result = _mm_add_ps(_mm_mul_ps(x_sq, result), a1);
  result = _mm_mul_ps(x, result);
  return result;
}

template<X86_VEXT vext>
void gradient_SIMD (PelStorage *buff1, PelStorage *buff2,
                    PelStorage *AccGxBuf, PelStorage *AccGyBuf,
                    unsigned int width, unsigned int height,
                    unsigned int bitDepth, ComponentID compID)
{
  // buff1 - magnitude; buff2 - orientation (Only luma in buff2)
  const unsigned int convWidthS=CONV_WIDTH_S;
  const int maxClpRange = (1 << bitDepth) - 1;
  const int padding     = convWidthS / 2;
  Pel* p_buf1;
  Pel* p_buf1_up;
  Pel* p_buf1_down;
  int stride  = buff1->Y().stride;
  Pel* p_ACC = AccGxBuf->Y().buf;
  Pel* p_ACC_Y = AccGyBuf->Y().buf;

  int res16 = width & 0xf;
  // avoid compiler warnings
  __m128i v0_mid =  _mm_set1_epi16 (0);
  __m128i vold_down =  _mm_set1_epi16 (0);
  __m128i vold_up =  _mm_set1_epi16 (0);
  __m128i v0_down =  _mm_set1_epi16 (0);
  __m128i v0_up =  _mm_set1_epi16 (0);
  __m128i vold_mid =  _mm_set1_epi16 (0);

  for (int y = 0; y < height; y++)
  {
    p_buf1=buff1->Y().buf + y*stride;
    if (y==0)
    {
      p_buf1_up = p_buf1;
      p_buf1_down = p_buf1_up+stride;
    }
    else if (y==height-1)
    {
      p_buf1_down = p_buf1;
      p_buf1_up = p_buf1_down - stride;
    }
    else
    {
      p_buf1_up = p_buf1 - stride;  //starts at 1 now
      p_buf1_down = p_buf1+stride;
    }
    if( vext >= AVX2 && !res16)
    {
#ifdef USE_AVX2
      __m256i  v0_up;
      __m256i  v0_down;
      __m256i  v0_mid;
      int x;
      for (x=0; x < width-16; x+=16)
      {
        if (x==0)
        {
          v0_up = _mm256_lddqu_si256((const __m256i *)(p_buf1_up));
          __m256i  vr_up = _mm256_lddqu_si256((const __m256i *)(p_buf1_up+1));
          v0_down = _mm256_lddqu_si256((const __m256i *)(p_buf1_down));
          __m256i  vr_down = _mm256_lddqu_si256((const __m256i *)(p_buf1_down+1));

          __m256i  vl_up = _mm256_slli_si256 (v0_up,2);  // jeweils der unterste fehlt, aus vold holen
          __m256i  tmp = _mm256_permute4x64_epi64 (v0_up,0x10);
          tmp = _mm256_bsrli_epi128(tmp,6);
          tmp = _mm256_inserti128_si256(tmp,_mm256_castsi256_si128 (v0_up),0);
          vl_up = _mm256_blend_epi16(vl_up,tmp,1);
          __m256i  vm_up = _mm256_slli_epi16 (v0_up,1);  // middle *2
          __m256i  acc_up = _mm256_adds_epi16 (vm_up,vl_up);
          acc_up = _mm256_adds_epi16 (acc_up,vr_up);

          __m256i  vl_down = _mm256_slli_si256 (v0_down,2);  // jeweils der unterste fehlt, aus vold holen
          tmp = _mm256_permute4x64_epi64 (v0_down,0x10);
          tmp = _mm256_bsrli_epi128(tmp,6);
          tmp = _mm256_inserti128_si256(tmp,_mm256_castsi256_si128 (v0_down),0);
          vl_down = _mm256_blend_epi16(vl_down,tmp,1);
          __m256i  vm_down = _mm256_slli_epi16 (v0_down,1);  // middle *2
          __m256i  acc_down = _mm256_adds_epi16 (vm_down,vl_down);
          acc_down = _mm256_adds_epi16 (acc_down,vr_down);

          __m256i  acc = _mm256_subs_epi16 (acc_down,acc_up);
          _mm256_storeu_si256((__m256i *)&p_ACC[0], acc);

          // mid
          v0_mid = _mm256_lddqu_si256((const __m256i *)(p_buf1));
          __m256i  vr_mid = _mm256_lddqu_si256((const __m256i *)(p_buf1+1));
          __m256i  vl_mid = _mm256_slli_si256 (v0_mid,2);  // jeweils der unterste fehlt, aus vold holen
          tmp = _mm256_permute4x64_epi64 (v0_mid,0x10);
          tmp = _mm256_bsrli_epi128(tmp,6);
          tmp = _mm256_inserti128_si256(tmp,_mm256_castsi256_si128 (v0_mid),0);
          vl_mid = _mm256_blend_epi16(vl_mid,tmp,1);

          __m256i acc_right = _mm256_adds_epi16 (vr_up,vr_down);
          vr_mid = _mm256_slli_epi16 (vr_mid,1);  // middle *2
          acc_right = _mm256_adds_epi16 (acc_right,vr_mid);

          __m256i acc_left = _mm256_adds_epi16 (vl_up,vl_down);
          vl_mid = _mm256_slli_epi16 (vl_mid,1);  // middle *2
          acc_left = _mm256_adds_epi16 (acc_left,vl_mid);
          acc = _mm256_subs_epi16 (acc_right,acc_left);
          _mm256_storeu_si256((__m256i *)&p_ACC_Y[x], acc);
        }
        else
        {
          v0_up = _mm256_lddqu_si256((const __m256i *)(p_buf1_up+x));
          __m256i  vr_up = _mm256_lddqu_si256((const __m256i *)(p_buf1_up+x+1));
          __m256i  vl_up = _mm256_lddqu_si256((const __m256i *)(p_buf1_up+x-1));

          v0_down = _mm256_lddqu_si256((const __m256i *)(p_buf1_down+x));
          __m256i  vr_down = _mm256_lddqu_si256((const __m256i *)(p_buf1_down+x+1));
          __m256i  vl_down = _mm256_lddqu_si256((const __m256i *)(p_buf1_down+x-1));

          __m256i  vm_up = _mm256_slli_epi16 (v0_up,1);  // middle *2
          __m256i  acc_up = _mm256_adds_epi16 (vm_up,vl_up);
          acc_up = _mm256_adds_epi16 (acc_up,vr_up);

          __m256i  vm_down = _mm256_slli_epi16 (v0_down,1);  // middle *2
          __m256i  acc_down = _mm256_adds_epi16 (vm_down,vl_down);
          acc_down = _mm256_adds_epi16 (acc_down,vr_down);

          __m256i  acc = _mm256_subs_epi16 (acc_down,acc_up);
          _mm256_storeu_si256((__m256i *)&p_ACC[x], acc);

          // mid
          v0_mid = _mm256_lddqu_si256((const __m256i *)(p_buf1+x));
          __m256i  vr_mid = _mm256_lddqu_si256((const __m256i *)(p_buf1+x+1));
          __m256i  vl_mid = _mm256_lddqu_si256((const __m256i *)(p_buf1+x-1));

          __m256i acc_right = _mm256_adds_epi16 (vr_up,vr_down);
          vr_mid = _mm256_slli_epi16 (vr_mid,1);  // middle *2
          acc_right = _mm256_adds_epi16 (acc_right,vr_mid);

          __m256i acc_left = _mm256_adds_epi16 (vl_up,vl_down);
          vl_mid = _mm256_slli_epi16 (vl_mid,1);  // middle *2
          acc_left = _mm256_adds_epi16 (acc_left,vl_mid);
          acc = _mm256_subs_epi16 (acc_right,acc_left);
          _mm256_storeu_si256((__m256i *)&p_ACC_Y[x], acc);
        }
      }  //for x
      // last collum
      {
        v0_up = _mm256_lddqu_si256((const __m256i *)(p_buf1_up+x));
        __m256i  vl_up = _mm256_lddqu_si256((const __m256i *)(p_buf1_up+x-1));

        v0_down = _mm256_lddqu_si256((const __m256i *)(p_buf1_down+x));
        __m256i  vl_down = _mm256_lddqu_si256((const __m256i *)(p_buf1_down+x-1));

        __m256i  vr_up = _mm256_srli_si256 (v0_up,2);  // jeweils der oberste fehlt
        vr_up = _mm256_insert_epi16 (vr_up,_mm256_extract_epi16 (v0_up,8), 7);
        vr_up = _mm256_insert_epi16 (vr_up,_mm256_extract_epi16 (v0_up,15), 15);

        __m256i  vr_down = _mm256_srli_si256 (v0_down,2);  // jeweils der oberste fehlt
        vr_down = _mm256_insert_epi16 (vr_down,_mm256_extract_epi16 (v0_down,8), 7);
        vr_down = _mm256_insert_epi16 (vr_down,_mm256_extract_epi16 (v0_down,15), 15);

        __m256i  vm_up = _mm256_slli_epi16 (v0_up,1);  // middle *2
        __m256i  acc_up = _mm256_adds_epi16 (vm_up,vl_up);
        acc_up = _mm256_adds_epi16 (acc_up,vr_up);

        __m256i  vm_down = _mm256_slli_epi16 (v0_down,1);  // middle *2
        __m256i  acc_down = _mm256_adds_epi16 (vm_down,vl_down);
        acc_down = _mm256_adds_epi16 (acc_down,vr_down);

        __m256i  acc = _mm256_subs_epi16 (acc_down,acc_up);
        _mm256_storeu_si256((__m256i *)&p_ACC[x], acc);

        // mid
        v0_mid = _mm256_lddqu_si256((const __m256i *)(p_buf1+x));
        __m256i  vl_mid = _mm256_lddqu_si256((const __m256i *)(p_buf1+x-1));

        __m256i  vr_mid = _mm256_srli_si256 (v0_mid,2);  // jeweils der oberste fehlt
        vr_mid = _mm256_insert_epi16 (vr_mid,_mm256_extract_epi16 (v0_mid,8), 7);
        vr_mid = _mm256_insert_epi16 (vr_mid,_mm256_extract_epi16 (v0_mid,15), 15);

        __m256i acc_right = _mm256_adds_epi16 (vr_up,vr_down);
        vr_mid = _mm256_slli_epi16 (vr_mid,1);  // middle *2
        acc_right = _mm256_adds_epi16 (acc_right,vr_mid);

        __m256i acc_left = _mm256_adds_epi16 (vl_up,vl_down);
        vl_mid = _mm256_slli_epi16 (vl_mid,1);  // middle *2
        acc_left = _mm256_adds_epi16 (acc_left,vl_mid);
        acc = _mm256_subs_epi16 (acc_right,acc_left);
        _mm256_storeu_si256((__m256i *)&p_ACC_Y[x], acc);
      }
#endif
    }  //AVX2
    else
    {
      __m128i v1_up;
      __m128i v1_down;
      __m128i v1_mid;
      int x;
      for (x=0; x < width-8; x+=8)
      {
        if (x==0)
        {
          v0_up = _mm_loadu_si128((const __m128i*)(p_buf1_up));
          v1_up = _mm_loadu_si128((const __m128i*)(p_buf1_up+8));
          v0_down = _mm_loadu_si128((const __m128i*)(p_buf1_down));
          v1_down = _mm_loadu_si128((const __m128i*)(p_buf1_down+8));
          v0_mid = _mm_loadu_si128((const __m128i*)(p_buf1));
          v1_mid = _mm_loadu_si128((const __m128i*)(p_buf1+8));

          __m128i vl_up = _mm_slli_si128 (v0_up,2);  // der unterste fehlt, aus vold holen
          vl_up = _mm_blend_epi16 (vl_up,v0_up,1);
          __m128i vr_up = _mm_srli_si128 (v0_up,2);  // der oberste fehlt, aus v1 holen
          vr_up = _mm_blend_epi16 (vr_up,_mm_slli_si128 (v1_up,14),0x80);

          __m128i vl_down = _mm_slli_si128 (v0_down,2);  // der unterste fehlt,
          vl_down = _mm_blend_epi16 (vl_down,v0_down,1);
          __m128i vr_down = _mm_srli_si128 (v0_down,2);  // der oberste fehlt, aus v1 holen
          vr_down = _mm_blend_epi16 (vr_down,_mm_slli_si128 (v1_down,14),0x80);

          __m128i vl_mid = _mm_slli_si128 (v0_mid,2);  // der unterste fehlt,
          vl_mid = _mm_blend_epi16 (vl_mid,v0_mid,1);
          __m128i vr_mid = _mm_srli_si128 (v0_mid,2);  // der oberste fehlt, aus v1 holen
          vr_mid = _mm_blend_epi16 (vr_mid,_mm_slli_si128 (v1_mid,14),0x80);

          __m128i vm_up = _mm_slli_epi16 (v0_up,1);  // middle *2
          __m128i acc_up = _mm_adds_epi16 (vm_up,vl_up);
          acc_up = _mm_adds_epi16 (acc_up,vr_up);

          __m128i vm_down = _mm_slli_epi16 (v0_down,1);  // middle *2
          __m128i acc_down = _mm_adds_epi16 (vm_down,vl_down);
          acc_down = _mm_adds_epi16 (acc_down,vr_down);

          __m128i acc = _mm_subs_epi16 (acc_down,acc_up);
          _mm_storeu_si128((__m128i*)&p_ACC[x], acc);

          __m128i acc_right = _mm_adds_epi16 (vr_up,vr_down);
          vr_mid = _mm_slli_epi16 (vr_mid,1);  // middle *2
          acc_right = _mm_adds_epi16 (acc_right,vr_mid);

          __m128i acc_left = _mm_adds_epi16 (vl_up,vl_down);
          vl_mid = _mm_slli_epi16 (vl_mid,1);  // middle *2
          acc_left = _mm_adds_epi16 (acc_left,vl_mid);
          acc = _mm_subs_epi16 (acc_right,acc_left);
          _mm_storeu_si128((__m128i*)&p_ACC_Y[x], acc);
        }
        else
        {
          v1_up = _mm_loadu_si128((const __m128i*)(p_buf1_up+x+8));
          v1_down = _mm_loadu_si128((const __m128i*)(p_buf1_down+x+8));
          v1_mid = _mm_loadu_si128((const __m128i*)(p_buf1+x+8));

          __m128i vl_up = _mm_slli_si128 (v0_up,2);  // der unterste fehlt, aus vold holen
          vl_up = _mm_blend_epi16 (vl_up,_mm_srli_si128 (vold_up,14),1);
          __m128i vr_up = _mm_srli_si128 (v0_up,2);  // der oberste fehlt, aus v1 holen
          vr_up = _mm_blend_epi16 (vr_up,_mm_slli_si128 (v1_up,14),0x80);

          __m128i vl_down = _mm_slli_si128 (v0_down,2);  // der unterste fehlt, aus vold holen
          vl_down = _mm_blend_epi16 (vl_down,_mm_srli_si128 (vold_down,14),1);
          __m128i vr_down = _mm_srli_si128 (v0_down,2);  // der oberste fehlt, aus v1 holen
          vr_down = _mm_blend_epi16 (vr_down,_mm_slli_si128 (v1_down,14),0x80);

          __m128i vl_mid = _mm_slli_si128 (v0_mid,2);  // der unterste fehlt, aus vold holen
          vl_mid = _mm_blend_epi16 (vl_mid,_mm_srli_si128 (vold_mid,14),1);
          __m128i vr_mid = _mm_srli_si128 (v0_mid,2);  // der oberste fehlt, aus v1 holen
          vr_mid = _mm_blend_epi16 (vr_mid,_mm_slli_si128 (v1_mid,14),0x80);

          __m128i vm_up = _mm_slli_epi16 (v0_up,1);  // middle *2
          __m128i acc_up = _mm_adds_epi16 (vm_up,vl_up);
          acc_up = _mm_adds_epi16 (acc_up,vr_up);

          __m128i vm_down = _mm_slli_epi16 (v0_down,1);  // middle *2
          __m128i acc_down = _mm_adds_epi16 (vm_down,vl_down);
          acc_down = _mm_adds_epi16 (acc_down,vr_down);
          __m128i acc = _mm_subs_epi16 (acc_down,acc_up);
          _mm_storeu_si128((__m128i*)&p_ACC[x], acc);

          __m128i acc_right = _mm_adds_epi16 (vr_up,vr_down);
          vr_mid = _mm_slli_epi16 (vr_mid,1);  // middle *2
          acc_right = _mm_adds_epi16 (acc_right,vr_mid);

          __m128i acc_left = _mm_adds_epi16 (vl_up,vl_down);
          vl_mid = _mm_slli_epi16 (vl_mid,1);  // middle *2
          acc_left = _mm_adds_epi16 (acc_left,vl_mid);
          acc = _mm_subs_epi16 (acc_right,acc_left);
          _mm_storeu_si128((__m128i*)&p_ACC_Y[x], acc);
        }
        vold_up = v0_up;
        vold_down = v0_down;
        vold_mid = v0_mid;
        v0_up = v1_up;
        v0_down = v1_down;
        v0_mid = v1_mid;
      }  //for x
      // last collum
      {
        __m128i vl_up = _mm_slli_si128 (v0_up,2);  // der unterste fehlt, aus vold holen
        vl_up = _mm_blend_epi16 (vl_up,_mm_srli_si128 (vold_up,14),1);
        __m128i vr_up = _mm_srli_si128 (v0_up,2);  // der oberste fehlt, aus v0 holen
        vr_up = _mm_blend_epi16 (vr_up,v0_up,0x80);

        __m128i vl_down = _mm_slli_si128 (v0_down,2);  // der unterste fehlt, aus vold holen
        vl_down = _mm_blend_epi16 (vl_down,_mm_srli_si128 (vold_down,14),1);
        __m128i vr_down = _mm_srli_si128 (v0_down,2);  // der oberste fehlt, aus v0 holen
        vr_down = _mm_blend_epi16(vr_down,v0_down,0x80);

        __m128i vl_mid = _mm_slli_si128 (v0_mid,2);  // der unterste fehlt, aus vold holen
        vl_mid = _mm_blend_epi16 (vl_mid,_mm_srli_si128 (vold_mid,14),1);
        __m128i vr_mid = _mm_srli_si128 (v0_mid,2);  // der oberste fehlt, aus v0 holen
        vr_mid = _mm_blend_epi16 (vr_mid,v0_mid,0x80);

        __m128i vm_up = _mm_slli_epi16 (v0_up,1);  // middle *2
        __m128i acc_up = _mm_adds_epi16 (vm_up,vl_up);
        acc_up = _mm_adds_epi16 (acc_up,vr_up);

        __m128i vm_down = _mm_slli_epi16 (v0_down,1);  // middle *2
        __m128i acc_down = _mm_adds_epi16 (vm_down,vl_down);
        acc_down = _mm_adds_epi16 (acc_down,vr_down);

        __m128i acc = _mm_subs_epi16 (acc_down,acc_up);
        _mm_storeu_si128((__m128i*)&p_ACC[x], acc);

        __m128i acc_right = _mm_adds_epi16 (vr_up,vr_down);
        vr_mid = _mm_slli_epi16 (vr_mid,1);  // middle *2
        acc_right = _mm_adds_epi16 (acc_right,vr_mid);

        __m128i acc_left = _mm_adds_epi16 (vl_up,vl_down);
        vl_mid = _mm_slli_epi16 (vl_mid,1);  // middle *2
        acc_left = _mm_adds_epi16 (acc_left,vl_mid);
        acc = _mm_subs_epi16 (acc_right,acc_left);
        _mm_storeu_si128((__m128i*)&p_ACC_Y[x], acc);
      }
    }
    p_ACC+=width;
    p_ACC_Y+=width;
  } // y

  // magnitude
  p_ACC = AccGxBuf->Y().buf;
  p_ACC_Y = AccGyBuf->Y().buf;

  for (int y = 0; y < height; y++)
  {
    p_buf1=buff1->Y().buf + y*stride;

    if( vext >= AVX2 && !res16)
    {
#ifdef USE_AVX2
      int x;
      __m256i vbdmax  = _mm256_set1_epi16   ( maxClpRange);

      for (x=0; x < width; x+=16)
      {
        __m256i GX = _mm256_loadu_si256((const __m256i*)&p_ACC[x]);
        __m256i GY = _mm256_loadu_si256((const __m256i*)&p_ACC_Y[x]);
        GX = _mm256_abs_epi16(GX);
        GY = _mm256_abs_epi16(GY);
        GX = _mm256_add_epi16(GX,GY);
        GX = _mm256_srli_epi16(GX,1);
        GX = _mm256_min_epi16 (GX,vbdmax);
        _mm256_storeu_si256((__m256i*)&p_buf1[x], GX);
      }
#endif
    }  //AVX2
    else
    {
      int x;
      __m128i vbdmax  = _mm_set1_epi16   ( maxClpRange);

      for (x=0; x < width; x+=8)
      {
        __m128i GX = _mm_loadu_si128((const __m128i*)&p_ACC[x]);
        __m128i GY = _mm_loadu_si128((const __m128i*)&p_ACC_Y[x]);
        GX = _mm_abs_epi16(GX);
        GY = _mm_abs_epi16(GY);
        GX = _mm_add_epi16(GX,GY);
        GX = _mm_srli_epi16(GX,1);
        GX = _mm_min_epi16 (GX,vbdmax);
        _mm_storeu_si128((__m128i*)&p_buf1[x], GX);
      }
    }
    p_ACC+=width;
    p_ACC_Y+=width;
    p_buf1+=stride;
  }

  // Loop through each pixel
  Pel* pX = AccGxBuf->Y().buf;
  Pel* pY = AccGyBuf->Y().buf;
  int strideX = AccGxBuf->Y().stride;
  int strideY = AccGyBuf->Y().stride;

  Pel* pQD = buff2->Y().buf;

  for (int y = 0; y < height; y++)
  {
    if( vext >= AVX2 && !res16)
    {
#ifdef USE_AVX2
      // Store pi and pi/2 as constants
      const __m256 pi = _mm256_set1_ps((float)PI);
      const __m256 pi_2 = _mm256_set1_ps((float)PI_2);
      const __m256 vpi_8 = _mm256_set1_ps((float)pi_8);
      const __m256 vpi_3_8 = _mm256_set1_ps((float)pi_3_8);
      const __m256 vpi_5_8 = _mm256_set1_ps((float)pi_5_8);
      const __m256 vpi_7_8 = _mm256_set1_ps((float)pi_7_8);

      const __m256 abs_mask = _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF));;
      const __m256 sign_mask = _mm256_castsi256_ps(_mm256_set1_epi32(0x80000000));

      for (int x = 0; x < width; x+=16)
      {
        for (int n=0; n<16;n+=8)
        {
          __m128i Ix = _mm_loadu_si128((const __m128i*)&pX[x+n]);
          __m128i Iy = _mm_loadu_si128((const __m128i*)&pY[x+n]);

          __m256 vx = _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(Ix));
          __m256 vy = _mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(Iy));
          __m256 swap_mask = _mm256_cmp_ps(_mm256_and_ps(vy, abs_mask),_mm256_and_ps(vx, abs_mask),_CMP_GT_OS);
          __m256 atan_input = _mm256_div_ps(_mm256_blendv_ps(vy, vx, swap_mask),_mm256_blendv_ps(vx, vy, swap_mask));
          __m256 result = atan_avx_approximation(atan_input);

          result = _mm256_blendv_ps(result,_mm256_sub_ps(_mm256_or_ps(pi_2, _mm256_and_ps(atan_input, sign_mask)),result),swap_mask);
          __m256 x_sign_mask = _mm256_castsi256_ps(_mm256_srai_epi32(_mm256_castps_si256(vx), 31));
          result = _mm256_add_ps(_mm256_and_ps(_mm256_xor_ps(pi, _mm256_and_ps(sign_mask, vy)),x_sign_mask),result);

          // take abs value
          result = _mm256_andnot_ps(sign_mask,result);
          // compare
          __m256 QD0 = _mm256_cmp_ps (result,vpi_8,_CMP_LE_OS);
          QD0 = _mm256_or_ps(QD0,_mm256_cmp_ps (result,vpi_7_8,_CMP_GE_OS));
          __m256 QD90 = _mm256_cmp_ps (result,vpi_3_8,_CMP_GT_OS);
          QD90 = _mm256_and_ps(QD90,_mm256_cmp_ps (result,vpi_5_8,_CMP_LE_OS));
          __m256 QD45 = _mm256_cmp_ps (result,vpi_8,_CMP_GT_OS);
          QD45 = _mm256_and_ps(QD45,_mm256_cmp_ps (result,vpi_3_8,_CMP_LE_OS));
          __m256 QD135 = _mm256_cmp_ps (result,vpi_5_8,_CMP_GT_OS);
          QD135 = _mm256_and_ps(QD135,_mm256_cmp_ps (result,vpi_7_8,_CMP_LE_OS));
          // Dy > 0
          __m256 Neg = _mm256_cmp_ps (vy,_mm256_set1_ps(0.0),_CMP_LT_OS);
          QD45 = _mm256_xor_ps(QD45,_mm256_and_ps(Neg,QD135));
          QD135 = _mm256_xor_ps(QD135,_mm256_and_ps(Neg,QD45));

          __m256 FQD = _mm256_set1_ps(0.0);
          FQD =  _mm256_blendv_ps(FQD,_mm256_set1_ps(90.0),QD90);
          FQD =  _mm256_blendv_ps(FQD,_mm256_set1_ps(45.0),QD45);
          FQD =  _mm256_blendv_ps(FQD,_mm256_set1_ps(135.0),QD135);
          // integer 32 bit
          __m256i QD0I = _mm256_cvtps_epi32(FQD);
          // integer 16 bit
          QD0I = _mm256_packus_epi32(QD0I,QD0I);
          QD0I = _mm256_permute4x64_epi64(QD0I,0x8);
          _mm_storeu_si128((__m128i*)&pQD[x+n], _mm256_castsi256_si128(QD0I));
        }
      }
#endif
    }
    else      //SSE
    {
      // Store pi and pi/2 as constants
      const __m128 pi = _mm_set1_ps((float)PI);
      const __m128 pi_2 = _mm_set1_ps((float)PI_2);
      const __m128 vpi_8 = _mm_set1_ps((float)pi_8);
      const __m128 vpi_3_8 = _mm_set1_ps((float)pi_3_8);
      const __m128 vpi_5_8 = _mm_set1_ps((float)pi_5_8);
      const __m128 vpi_7_8 = _mm_set1_ps((float)pi_7_8);

      const __m128 abs_mask = _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF));;
      const __m128 sign_mask = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));

      for (int x = 0; x < width; x+=8)
      {
        for (int n=0; n<8;n+=4)
        {
          __m128i Ix = _mm_loadu_si128((const __m128i*)&pX[x+n]);
          __m128i Iy = _mm_loadu_si128((const __m128i*)&pY[x+n]);

          __m128 vx = _mm_cvtepi32_ps(_mm_cvtepi16_epi32(Ix));
          __m128 vy = _mm_cvtepi32_ps(_mm_cvtepi16_epi32(Iy));
          __m128 swap_mask = _mm_cmpgt_ps(_mm_and_ps(vy, abs_mask),_mm_and_ps(vx, abs_mask));
          __m128 atan_input = _mm_div_ps(_mm_blendv_ps(vy, vx, swap_mask),_mm_blendv_ps(vx, vy, swap_mask));
          __m128 result = atan_avx_approximation(atan_input);

          result = _mm_blendv_ps(result,_mm_sub_ps(_mm_or_ps(pi_2, _mm_and_ps(atan_input, sign_mask)),result),swap_mask);
          __m128 x_sign_mask = _mm_castsi128_ps(_mm_srai_epi32(_mm_castps_si128(vx), 31));
          result = _mm_add_ps(_mm_and_ps(_mm_xor_ps(pi, _mm_and_ps(sign_mask, vy)),x_sign_mask),result);

          // take abs value
          result = _mm_andnot_ps(sign_mask,result);
          // compare
          __m128 QD0 = _mm_cmple_ps (result,vpi_8);
          QD0 = _mm_or_ps(QD0,_mm_cmpge_ps (result,vpi_7_8));
          __m128 QD90 = _mm_cmpgt_ps (result,vpi_3_8);
          QD90 = _mm_and_ps(QD90,_mm_cmple_ps (result,vpi_5_8));
          __m128 QD45 = _mm_cmpgt_ps (result,vpi_8);
          QD45 = _mm_and_ps(QD45,_mm_cmple_ps (result,vpi_3_8));
          __m128 QD135 = _mm_cmpgt_ps (result,vpi_5_8);
          QD135 = _mm_and_ps(QD135,_mm_cmple_ps (result,vpi_7_8));
          // Dy > 0
          __m128 Neg = _mm_cmplt_ps (vy,_mm_set1_ps(0.0));
          QD45 = _mm_xor_ps(QD45,_mm_and_ps(Neg,QD135));
          QD135 = _mm_xor_ps(QD135,_mm_and_ps(Neg,QD45));

          __m128 FQD = _mm_set1_ps(0.0);
          FQD =  _mm_blendv_ps(FQD,_mm_set1_ps(90.0),QD90);
          FQD =  _mm_blendv_ps(FQD,_mm_set1_ps(45.0),QD45);
          FQD =  _mm_blendv_ps(FQD,_mm_set1_ps(135.0),QD135);
          // integer 32 bit
          __m128i QD0I = _mm_cvtps_epi32(FQD);
          // integer 16 bit
          QD0I = _mm_packus_epi32(QD0I,QD0I);
          _mm_storeu_si64((__m128i*)&pQD[x+n],QD0I);
        }
      }
    }
    pX+=strideX;
    pY+=strideY;
    pQD+=buff2->Y().stride;;
  }

  buff1->get(compID).extendBorderPel(padding, padding);   // extend border for the next steps
}

template<X86_VEXT vext>
int dilation_SIMD ( PelStorage *buff,
                    PelStorage *Wbuf,
                    unsigned int bitDepth,
                    ComponentID compID,
                    int numIter,
                    int iter,
                    Pel Value)
{
  if ( iter == numIter )
  {
    return iter;
  }
  unsigned int width      = buff->get(compID).width,
               height     = buff->get(compID).height;   // Width and Height of current frame
  unsigned int windowSize = KERNELSIZE;
  unsigned int padding    = windowSize / 2;

  Wbuf->bufs[0].copyFrom( buff->get(compID) );

  Pel* p_buf;
  Pel* p_buf_up;
  Pel* p_buf_down;
  int stride  = buff->Y().stride;
  Pel* p_tmpBuf = Wbuf->Y().buf;

  int res16 = width & 0xf;

  // avoid compiler warnings
  __m128i v0_mid =  _mm_set1_epi16 (0);
  __m128i vold_down =  _mm_set1_epi16 (0);
  __m128i vold_up =  _mm_set1_epi16 (0);
  __m128i v0_down =  _mm_set1_epi16 (0);
  __m128i v0_up =  _mm_set1_epi16 (0);
  __m128i vold_mid =  _mm_set1_epi16 (0);

  for (int y = 0; y < height; y++)
  {
    p_buf=buff->Y().buf + y*stride;
    if (y==0)
    {
      p_buf_up = p_buf;
      p_buf_down = p_buf_up+stride;
    }
    else if (y==height-1)
    {
      p_buf_down = p_buf;
      p_buf_up = p_buf_down - stride;
    }
    else
    {
      p_buf_up = p_buf - stride;  //starts at 1 now
      p_buf_down = p_buf+stride;
    }
    if( vext >= AVX2 && !res16)
    {
#ifdef USE_AVX2
      __m256i  v0_up;
      __m256i  v0_down;
      __m256i  v0_mid;
      __m256i vstrong =  _mm256_set1_epi16 (Value);

      int x;
      for (x=0; x < width-16; x+=16)
      {
        if (x==0)
        {
          v0_up = _mm256_lddqu_si256((const __m256i *)(p_buf_up));
          __m256i  vr_up = _mm256_lddqu_si256((const __m256i *)(p_buf_up+1));
          v0_down = _mm256_lddqu_si256((const __m256i *)(p_buf_down));
          __m256i  vr_down = _mm256_lddqu_si256((const __m256i *)(p_buf_down+1));

          __m256i  vl_up = _mm256_slli_si256 (v0_up,2);  // jeweils der unterste fehlt, aus vold holen
          __m256i  tmp = _mm256_permute4x64_epi64 (v0_up,0x10);
          tmp = _mm256_bsrli_epi128(tmp,6);
          tmp = _mm256_inserti128_si256(tmp,_mm256_castsi256_si128 (v0_up),0);
          vl_up = _mm256_blend_epi16(vl_up,tmp,1);

          __m256i  vl_down = _mm256_slli_si256 (v0_down,2);  // jeweils der unterste fehlt, aus vold holen
          tmp = _mm256_permute4x64_epi64 (v0_down,0x10);
          tmp = _mm256_bsrli_epi128(tmp,6);
          tmp = _mm256_inserti128_si256(tmp,_mm256_castsi256_si128 (v0_down),0);
          vl_down = _mm256_blend_epi16(vl_down,tmp,1);

          // mid
          v0_mid = _mm256_lddqu_si256((const __m256i *)(p_buf));
          __m256i  vr_mid = _mm256_lddqu_si256((const __m256i *)(p_buf+1));
          __m256i  vl_mid = _mm256_slli_si256 (v0_mid,2);  // jeweils der unterste fehlt, aus vold holen
          tmp = _mm256_permute4x64_epi64 (v0_mid,0x10);
          tmp = _mm256_bsrli_epi128(tmp,6);
          tmp = _mm256_inserti128_si256(tmp,_mm256_castsi256_si128 (v0_mid),0);
          vl_mid = _mm256_blend_epi16(vl_mid,tmp,1);

          __m256i v_mask = _mm256_cmpeq_epi16(vl_up,vstrong);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_up,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_up,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vl_mid,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_mid,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_mid,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vl_down,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_down,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_down,vstrong),v_mask);

          __m256i vres = _mm256_blendv_epi8(v0_mid,vstrong,v_mask);
          _mm256_storeu_si256((__m256i*)&p_tmpBuf[x], vres);
        }
        else
        {
          v0_up = _mm256_lddqu_si256((const __m256i *)(p_buf_up+x));
          __m256i  vr_up = _mm256_lddqu_si256((const __m256i *)(p_buf_up+x+1));
          __m256i  vl_up = _mm256_lddqu_si256((const __m256i *)(p_buf_up+x-1));

          v0_down = _mm256_lddqu_si256((const __m256i *)(p_buf_down+x));
          __m256i  vr_down = _mm256_lddqu_si256((const __m256i *)(p_buf_down+x+1));
          __m256i  vl_down = _mm256_lddqu_si256((const __m256i *)(p_buf_down+x-1));

          // mid
          v0_mid = _mm256_lddqu_si256((const __m256i *)(p_buf+x));
          __m256i  vr_mid = _mm256_lddqu_si256((const __m256i *)(p_buf+x+1));
          __m256i  vl_mid = _mm256_lddqu_si256((const __m256i *)(p_buf+x-1));

          __m256i v_mask = _mm256_cmpeq_epi16(vl_up,vstrong);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_up,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_up,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vl_mid,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_mid,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_mid,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vl_down,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_down,vstrong),v_mask);
          v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_down,vstrong),v_mask);

          __m256i vres = _mm256_blendv_epi8(v0_mid,vstrong,v_mask);
          _mm256_storeu_si256((__m256i*)&p_tmpBuf[x], vres);
         }
      }  //for x
      // last collum
      {
        v0_up = _mm256_lddqu_si256((const __m256i *)(p_buf_up+x));
        __m256i  vl_up = _mm256_lddqu_si256((const __m256i *)(p_buf_up+x-1));

        v0_down = _mm256_lddqu_si256((const __m256i *)(p_buf_down+x));
        __m256i  vl_down = _mm256_lddqu_si256((const __m256i *)(p_buf_down+x-1));

        __m256i  vr_up = _mm256_srli_si256 (v0_up,2);  // jeweils der oberste fehlt
        vr_up = _mm256_insert_epi16 (vr_up,_mm256_extract_epi16 (v0_up,8), 7);
        vr_up = _mm256_insert_epi16 (vr_up,_mm256_extract_epi16 (v0_up,15), 15);

        __m256i  vr_down = _mm256_srli_si256 (v0_down,2);  // jeweils der oberste fehlt
        vr_down = _mm256_insert_epi16 (vr_down,_mm256_extract_epi16 (v0_down,8), 7);
        vr_down = _mm256_insert_epi16 (vr_down,_mm256_extract_epi16 (v0_down,15), 15);

        // mid
        v0_mid = _mm256_lddqu_si256((const __m256i *)(p_buf+x));
        __m256i  vl_mid = _mm256_lddqu_si256((const __m256i *)(p_buf+x-1));
        __m256i  vr_mid = _mm256_srli_si256 (v0_mid,2);  // jeweils der oberste fehlt
        vr_mid = _mm256_insert_epi16 (vr_mid,_mm256_extract_epi16 (v0_mid,8), 7);
        vr_mid = _mm256_insert_epi16 (vr_mid,_mm256_extract_epi16 (v0_mid,15), 15);

        __m256i v_mask = _mm256_cmpeq_epi16(vl_up,vstrong);
        v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_up,vstrong),v_mask);
        v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_up,vstrong),v_mask);
        v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vl_mid,vstrong),v_mask);
        v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_mid,vstrong),v_mask);
        v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_mid,vstrong),v_mask);
        v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vl_down,vstrong),v_mask);
        v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(v0_down,vstrong),v_mask);
        v_mask = _mm256_or_si256(_mm256_cmpeq_epi16(vr_down,vstrong),v_mask);

        __m256i vres = _mm256_blendv_epi8(v0_mid,vstrong,v_mask);
        _mm256_storeu_si256((__m256i*)&p_tmpBuf[x], vres);
      }
#endif
    }  //AVX2
    else
    {
      __m128i v1_up;
      __m128i v1_down;
      __m128i v1_mid;
      __m128i vstrong =  _mm_set1_epi16 (Value);
      int x;
      for (x=0; x < width-8; x+=8)
      {
        if (x==0)
        {
          v0_up = _mm_loadu_si128((const __m128i*)(p_buf_up));
          v1_up = _mm_loadu_si128((const __m128i*)(p_buf_up+8));
          v0_down = _mm_loadu_si128((const __m128i*)(p_buf_down));
          v1_down = _mm_loadu_si128((const __m128i*)(p_buf_down+8));
          v0_mid = _mm_loadu_si128((const __m128i*)(p_buf));
          v1_mid = _mm_loadu_si128((const __m128i*)(p_buf+8));

          __m128i vl_up = _mm_slli_si128 (v0_up,2);  // der unterste fehlt, aus vold holen
          vl_up = _mm_blend_epi16 (vl_up,v0_up,1);
          __m128i vr_up = _mm_srli_si128 (v0_up,2);  // der oberste fehlt, aus v1 holen
          vr_up = _mm_blend_epi16 (vr_up,_mm_slli_si128 (v1_up,14),0x80);

          __m128i vl_down = _mm_slli_si128 (v0_down,2);  // der unterste fehlt,
          vl_down = _mm_blend_epi16 (vl_down,v0_down,1);
          __m128i vr_down = _mm_srli_si128 (v0_down,2);  // der oberste fehlt, aus v1 holen
          vr_down = _mm_blend_epi16 (vr_down,_mm_slli_si128 (v1_down,14),0x80);

          __m128i vl_mid = _mm_slli_si128 (v0_mid,2);  // der unterste fehlt,
          vl_mid = _mm_blend_epi16 (vl_mid,v0_mid,1);
          __m128i vr_mid = _mm_srli_si128 (v0_mid,2);  // der oberste fehlt, aus v1 holen
          vr_mid = _mm_blend_epi16 (vr_mid,_mm_slli_si128 (v1_mid,14),0x80);

          __m128i v_mask = _mm_cmpeq_epi16(vl_up,vstrong);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_up,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_up,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vl_mid,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_mid,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_mid,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vl_down,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_down,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_down,vstrong),v_mask);

          __m128i vres = _mm_blendv_epi8(v0_mid,vstrong,v_mask);
          _mm_storeu_si128((__m128i*)&p_tmpBuf[x], vres);
        }  //x==0
        else
        {
          v1_up = _mm_loadu_si128((const __m128i*)(p_buf_up+x+8));
          v1_down = _mm_loadu_si128((const __m128i*)(p_buf_down+x+8));
          v1_mid = _mm_loadu_si128((const __m128i*)(p_buf+x+8));

          __m128i vl_up = _mm_slli_si128 (v0_up,2);  // der unterste fehlt, aus vold holen
          vl_up = _mm_blend_epi16 (vl_up,_mm_srli_si128 (vold_up,14),1);
          __m128i vr_up = _mm_srli_si128 (v0_up,2);  // der oberste fehlt, aus v1 holen
          vr_up = _mm_blend_epi16 (vr_up,_mm_slli_si128 (v1_up,14),0x80);

          __m128i vl_down = _mm_slli_si128 (v0_down,2);  // der unterste fehlt, aus vold holen
          vl_down = _mm_blend_epi16 (vl_down,_mm_srli_si128 (vold_down,14),1);
          __m128i vr_down = _mm_srli_si128 (v0_down,2);  // der oberste fehlt, aus v1 holen
          vr_down = _mm_blend_epi16 (vr_down,_mm_slli_si128 (v1_down,14),0x80);

          __m128i vl_mid = _mm_slli_si128 (v0_mid,2);  // der unterste fehlt, aus vold holen
          vl_mid = _mm_blend_epi16 (vl_mid,_mm_srli_si128 (vold_mid,14),1);
          __m128i vr_mid = _mm_srli_si128 (v0_mid,2);  // der oberste fehlt, aus v1 holen
          vr_mid = _mm_blend_epi16 (vr_mid,_mm_slli_si128 (v1_mid,14),0x80);

          __m128i v_mask = _mm_cmpeq_epi16(vl_up,vstrong);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_up,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_up,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vl_mid,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_mid,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_mid,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vl_down,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_down,vstrong),v_mask);
          v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_down,vstrong),v_mask);

          __m128i vres = _mm_blendv_epi8(v0_mid,vstrong,v_mask);
          _mm_storeu_si128((__m128i*)&p_tmpBuf[x], vres);
        }
        vold_up = v0_up;
        vold_down = v0_down;
        vold_mid = v0_mid;
        v0_up = v1_up;
        v0_down = v1_down;
        v0_mid = v1_mid;
      }  //for x
      // last collum
      {
        __m128i vl_up = _mm_slli_si128 (v0_up,2);  // der unterste fehlt, aus vold holen
        vl_up = _mm_blend_epi16 (vl_up,_mm_srli_si128 (vold_up,14),1);
        __m128i vr_up = _mm_srli_si128 (v0_up,2);  // der oberste fehlt, aus v0 holen
        vr_up = _mm_blend_epi16 (vr_up,v0_up,0x80);

        __m128i vl_down = _mm_slli_si128 (v0_down,2);  // der unterste fehlt, aus vold holen
        vl_down = _mm_blend_epi16 (vl_down,_mm_srli_si128 (vold_down,14),1);
        __m128i vr_down = _mm_srli_si128 (v0_down,2);  // der oberste fehlt, aus v0 holen
        vr_down = _mm_blend_epi16(vr_down,v0_down,0x80);

        __m128i vl_mid = _mm_slli_si128 (v0_mid,2);  // der unterste fehlt, aus vold holen
        vl_mid = _mm_blend_epi16 (vl_mid,_mm_srli_si128 (vold_mid,14),1);
        __m128i vr_mid = _mm_srli_si128 (v0_mid,2);  // der oberste fehlt, aus v0 holen
        vr_mid = _mm_blend_epi16 (vr_mid,v0_mid,0x80);

        __m128i v_mask = _mm_cmpeq_epi16(vl_up,vstrong);
        v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_up,vstrong),v_mask);
        v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_up,vstrong),v_mask);
        v_mask = _mm_or_si128(_mm_cmpeq_epi16(vl_mid,vstrong),v_mask);
        v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_mid,vstrong),v_mask);
        v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_mid,vstrong),v_mask);
        v_mask = _mm_or_si128(_mm_cmpeq_epi16(vl_down,vstrong),v_mask);
        v_mask = _mm_or_si128(_mm_cmpeq_epi16(v0_down,vstrong),v_mask);
        v_mask = _mm_or_si128(_mm_cmpeq_epi16(vr_down,vstrong),v_mask);

        __m128i vres = _mm_blendv_epi8(v0_mid,vstrong,v_mask);
        _mm_storeu_si128((__m128i*)&p_tmpBuf[x], vres);
      }
    }  //!AVX
    p_tmpBuf+=Wbuf->get(compID).stride;
  }//y

  buff->get(compID).extendBorderPel( padding, padding );
  buff->get(compID).copyFrom( Wbuf->bufs[0] );

  iter++;

  iter = dilation_SIMD<vext> ( buff,Wbuf,
                    bitDepth,
                    compID,
                    numIter,
                    iter,
                    Value);

  return iter;
}

template<X86_VEXT vext>
double calcVarSse( const Pel* org, const ptrdiff_t origStride, const int w, const int h );


template<X86_VEXT vext>
int calcMeanSSE ( const Pel* org, const ptrdiff_t origStride, const int w, const int h )
{
  int avg;
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
  avg = _mm_extract_epi32 (xavg32, 0);
  return avg;
}

template<X86_VEXT vext>
void Canny::_initFGACannyX86()
{
  gradient  = gradient_SIMD<vext>;
}
template void Canny::_initFGACannyX86<SIMDX86>();

template<X86_VEXT vext>
void Morph::_initFGAMorphX86()
{
  dilation  = dilation_SIMD<vext>;
}
template void Morph::_initFGAMorphX86<SIMDX86>();


template<X86_VEXT vext>
void FGAnalyzer::_initFGAnalyzerX86()
{
  calcVar  = calcVarSse<vext>;
  calcMean = calcMeanSSE<vext>;
}
template void FGAnalyzer::_initFGAnalyzerX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86
//! \}

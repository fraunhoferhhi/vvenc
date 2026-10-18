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
/** \file     FGAX86.h
    \brief    SIMD for FilmGrainAnalyse
*/

//! \ingroup CommonLib
//! \{

//#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "SEIFilmGrainAnalyzer.h"


//#ifdef TARGET_SIMD_X86
#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT_FGA

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

// Sobel gradients of 8 samples with the sums of gradient_core: gx = down - up, gy = right - left. up, mid and down
// point at the samples in the rows above, at and below them; the samples to their left and right must be readable.
static inline void sobel8( const Pel* up, const Pel* mid, const Pel* down, Pel* gx, Pel* gy )
{
  const __m128i ul = _mm_loadu_si128( ( const __m128i* ) ( up - 1 ) );
  const __m128i um = _mm_loadu_si128( ( const __m128i* ) up );
  const __m128i ur = _mm_loadu_si128( ( const __m128i* ) ( up + 1 ) );
  const __m128i ml = _mm_loadu_si128( ( const __m128i* ) ( mid - 1 ) );
  const __m128i mr = _mm_loadu_si128( ( const __m128i* ) ( mid + 1 ) );
  const __m128i dl = _mm_loadu_si128( ( const __m128i* ) ( down - 1 ) );
  const __m128i dm = _mm_loadu_si128( ( const __m128i* ) down );
  const __m128i dr = _mm_loadu_si128( ( const __m128i* ) ( down + 1 ) );

  const __m128i accUp    = _mm_adds_epi16( _mm_adds_epi16( _mm_slli_epi16( um, 1 ), ul ), ur );
  const __m128i accDown  = _mm_adds_epi16( _mm_adds_epi16( _mm_slli_epi16( dm, 1 ), dl ), dr );
  const __m128i accLeft  = _mm_adds_epi16( _mm_adds_epi16( ul, dl ), _mm_slli_epi16( ml, 1 ) );
  const __m128i accRight = _mm_adds_epi16( _mm_adds_epi16( ur, dr ), _mm_slli_epi16( mr, 1 ) );
  _mm_storeu_si128( ( __m128i* ) gx, _mm_subs_epi16( accDown, accUp ) );
  _mm_storeu_si128( ( __m128i* ) gy, _mm_subs_epi16( accRight, accLeft ) );
}

#ifdef USE_AVX2
// sobel8 for 16 samples
static inline void sobel16( const Pel* up, const Pel* mid, const Pel* down, Pel* gx, Pel* gy )
{
  const __m256i ul = _mm256_loadu_si256( ( const __m256i* ) ( up - 1 ) );
  const __m256i um = _mm256_loadu_si256( ( const __m256i* ) up );
  const __m256i ur = _mm256_loadu_si256( ( const __m256i* ) ( up + 1 ) );
  const __m256i ml = _mm256_loadu_si256( ( const __m256i* ) ( mid - 1 ) );
  const __m256i mr = _mm256_loadu_si256( ( const __m256i* ) ( mid + 1 ) );
  const __m256i dl = _mm256_loadu_si256( ( const __m256i* ) ( down - 1 ) );
  const __m256i dm = _mm256_loadu_si256( ( const __m256i* ) down );
  const __m256i dr = _mm256_loadu_si256( ( const __m256i* ) ( down + 1 ) );

  const __m256i accUp    = _mm256_adds_epi16( _mm256_adds_epi16( _mm256_slli_epi16( um, 1 ), ul ), ur );
  const __m256i accDown  = _mm256_adds_epi16( _mm256_adds_epi16( _mm256_slli_epi16( dm, 1 ), dl ), dr );
  const __m256i accLeft  = _mm256_adds_epi16( _mm256_adds_epi16( ul, dl ), _mm256_slli_epi16( ml, 1 ) );
  const __m256i accRight = _mm256_adds_epi16( _mm256_adds_epi16( ur, dr ), _mm256_slli_epi16( mr, 1 ) );
  _mm256_storeu_si256( ( __m256i* ) gx, _mm256_subs_epi16( accDown, accUp ) );
  _mm256_storeu_si256( ( __m256i* ) gy, _mm256_subs_epi16( accRight, accLeft ) );
}
#endif

template<X86_VEXT vext>
void gradient_SIMD (PelStorage *buff1, PelStorage *buff2,
                    PelStorage *AccGxBuf, PelStorage *AccGyBuf,
                    unsigned int width, unsigned int height,
                    unsigned int bitDepth, ComponentID compID)
{
  // The vectors below hold at least 8 samples. Narrower planes only occur in the quarter-size mask of pictures
  // under 32 samples wide.
  if( width < 8 )
  {
    gradient_core( buff1, buff2, AccGxBuf, AccGyBuf, width, height, bitDepth, compID );
    return;
  }

  // buff1 - magnitude; buff2 - orientation (Only luma in buff2)
  const unsigned int convWidthS=CONV_WIDTH_S;
  const int maxClpRange = (1 << bitDepth) - 1;
  const int padding     = convWidthS / 2;
  PelBuf    plane       = buff1->get( compID );
  const int stride      = plane.stride;

  // Each row is processed in vectors of 16 (AVX2) or 8 samples. When the width is not a multiple of that, the last
  // vector of a row starts at width - step and overlaps the one before it, which computes a few samples twice.
  const bool useAVX2 = vext >= AVX2 && width >= 16;
  const int  step    = useAVX2 ? 16 : 8;

  // As in gradient_core, the border samples are repeated into the margin, so that the 3x3 neighbourhood of every
  // sample is loaded as it is, also at the picture border.
  plane.extendBorderPel( padding, padding );

  Pel* p_ACC   = AccGxBuf->Y().buf;
  Pel* p_ACC_Y = AccGyBuf->Y().buf;
  for( int y = 0; y < height; y++ )
  {
    const Pel* mid  = plane.bufAt( 0, y );
    const Pel* up   = mid - stride;
    const Pel* down = mid + stride;
    for( int x = 0; x < width; x += step )
    {
      const int xv = std::min<int>( x, width - step );
#ifdef USE_AVX2
      if( useAVX2 )
      {
        sobel16( up + xv, mid + xv, down + xv, p_ACC + xv, p_ACC_Y + xv );
        continue;
      }
#endif
      sobel8( up + xv, mid + xv, down + xv, p_ACC + xv, p_ACC_Y + xv );
    }
    p_ACC   += width;
    p_ACC_Y += width;
  }

  // magnitude
  p_ACC   = AccGxBuf->Y().buf;
  p_ACC_Y = AccGyBuf->Y().buf;

  for (int y = 0; y < height; y++)
  {
    Pel* p_buf1 = plane.bufAt( 0, y );

    if( useAVX2 )
    {
#ifdef USE_AVX2
      __m256i vbdmax  = _mm256_set1_epi16   ( maxClpRange);

      for( int x = 0; x < width; x += step )
      {
        const int xv = std::min<int>( x, width - step );
        __m256i GX = _mm256_loadu_si256((const __m256i*)&p_ACC[xv]);
        __m256i GY = _mm256_loadu_si256((const __m256i*)&p_ACC_Y[xv]);
        GX = _mm256_abs_epi16(GX);
        GY = _mm256_abs_epi16(GY);
        GX = _mm256_add_epi16(GX,GY);
        GX = _mm256_srli_epi16(GX,1);
        GX = _mm256_min_epi16 (GX,vbdmax);
        _mm256_storeu_si256((__m256i*)&p_buf1[xv], GX);
      }
#endif
    }  //AVX2
    else
    {
      __m128i vbdmax  = _mm_set1_epi16   ( maxClpRange);

      for( int x = 0; x < width; x += step )
      {
        const int xv = std::min<int>( x, width - step );
        __m128i GX = _mm_loadu_si128((const __m128i*)&p_ACC[xv]);
        __m128i GY = _mm_loadu_si128((const __m128i*)&p_ACC_Y[xv]);
        GX = _mm_abs_epi16(GX);
        GY = _mm_abs_epi16(GY);
        GX = _mm_add_epi16(GX,GY);
        GX = _mm_srli_epi16(GX,1);
        GX = _mm_min_epi16 (GX,vbdmax);
        _mm_storeu_si128((__m128i*)&p_buf1[xv], GX);
      }
    }
    p_ACC+=width;
    p_ACC_Y+=width;
  }

  // Loop through each pixel. The gradients above were stored with a row pitch of width, which is less than the
  // buffers' stride for the subsampled planes of FGAnalyzer::findMask, so they are read back with the same pitch.
  Pel* pX = AccGxBuf->Y().buf;
  Pel* pY = AccGyBuf->Y().buf;

  Pel* pQD = buff2->Y().buf;

  for (int y = 0; y < height; y++)
  {
    if( useAVX2 )
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

      for( int x = 0; x < width; x += step )
      {
        const int xv = std::min<int>( x, width - step );
        for (int n=0; n<16;n+=8)
        {
          __m128i Ix = _mm_loadu_si128((const __m128i*)&pX[xv+n]);
          __m128i Iy = _mm_loadu_si128((const __m128i*)&pY[xv+n]);

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
          _mm_storeu_si128((__m128i*)&pQD[xv+n], _mm256_castsi256_si128(QD0I));
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

      for( int x = 0; x < width; x += step )
      {
        const int xv = std::min<int>( x, width - step );
        for (int n=0; n<8;n+=4)
        {
          __m128i Ix = _mm_loadu_si128((const __m128i*)&pX[xv+n]);
          __m128i Iy = _mm_loadu_si128((const __m128i*)&pY[xv+n]);

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
          _mm_storeu_si64((__m128i*)&pQD[xv+n],QD0I);
        }
      }
    }
    pX+=width;
    pY+=width;
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

  // The vectorized path below requires width to be a multiple of eight and
  // strictly greater than sixteen: at exactly 16, the AVX2 branch's 16-wide
  // main loop runs zero times, leaving its tail write to read an
  // uninitialized column -1 (width 16 is fine under SSE alone, but it's not
  // worth special-casing vext for a single width, so it's excluded either
  // way). It also assumes at least two rows: at height == 1 the y == 0 case
  // makes p_buf_down point past the single row, an out-of-bounds read.
  // Anything outside those bounds falls back to the scalar reference, which
  // handles all remaining iterations itself.
  if( width <= 16 || ( width % 8 ) != 0 || height == 1 )
  {
    return dilation_core( buff, Wbuf, bitDepth, compID, numIter, iter, Value );
  }

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

// Canny::suppressNonMaxPlane: eight samples per step; the four neighbour pairs of the directions 0, 45, 90 and 135
// degrees are all compared, and the direction of each sample selects its pair
template<X86_VEXT vext>
static void suppressNonMax_SIMD( const Pel* mag, const ptrdiff_t magStride, Pel* dir, const ptrdiff_t dirStride, const int width, const int height )
{
  auto suppressOne = [&]( const Pel* m, Pel& d )
  {
    int dx = 0, dy = 0;
    switch( d )
    {
    case 0:   dx =  1; dy = 0; break;
    case 45:  dx =  1; dy = 1; break;
    case 90:  dx =  0; dy = 1; break;
    case 135: dx = -1; dy = 1; break;
    default: THROW( "Unsupported gradient direction." ); break;
    }
    d = ( m[0] < m[dy * magStride + dx] || m[0] < m[-dy * magStride - dx] ) ? 0 : m[0];
  };

  const __m128i d0   = _mm_setzero_si128();
  const __m128i d45  = _mm_set1_epi16( 45 );
  const __m128i d90  = _mm_set1_epi16( 90 );
  const __m128i d135 = _mm_set1_epi16( 135 );

  for( int y = 0; y < height; y++, mag += magStride, dir += dirStride )
  {
    int x = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      const Pel* m    = &mag[x];
      const __m128i cur  = _mm_loadu_si128( ( const __m128i* ) m );
      const __m128i lft  = _mm_loadu_si128( ( const __m128i* ) &m[-1] );
      const __m128i rgt  = _mm_loadu_si128( ( const __m128i* ) &m[1] );
      const __m128i upL  = _mm_loadu_si128( ( const __m128i* ) &m[-magStride - 1] );
      const __m128i up   = _mm_loadu_si128( ( const __m128i* ) &m[-magStride] );
      const __m128i upR  = _mm_loadu_si128( ( const __m128i* ) &m[-magStride + 1] );
      const __m128i dnL  = _mm_loadu_si128( ( const __m128i* ) &m[magStride - 1] );
      const __m128i dn   = _mm_loadu_si128( ( const __m128i* ) &m[magStride] );
      const __m128i dnR  = _mm_loadu_si128( ( const __m128i* ) &m[magStride + 1] );
      const __m128i dirs = _mm_loadu_si128( ( const __m128i* ) &dir[x] );

      const __m128i is0   = _mm_cmpeq_epi16( dirs, d0 );
      const __m128i is45  = _mm_cmpeq_epi16( dirs, d45 );
      const __m128i is90  = _mm_cmpeq_epi16( dirs, d90 );
      const __m128i is135 = _mm_cmpeq_epi16( dirs, d135 );
      const __m128i valid = _mm_or_si128( _mm_or_si128( is0, is45 ), _mm_or_si128( is90, is135 ) );
      if( _mm_movemask_epi8( valid ) != 0xffff )
      {
        THROW( "Unsupported gradient direction." );
      }

      // a neighbour along the direction is larger: ( x + 1, y ) / ( x - 1, y ) for 0, ( x + 1, y + 1 ) / ( x - 1, y - 1 )
      // for 45, ( x, y + 1 ) / ( x, y - 1 ) for 90 and ( x - 1, y + 1 ) / ( x + 1, y - 1 ) for 135 degrees
      auto larger = [&cur]( const __m128i a, const __m128i b ) { return _mm_or_si128( _mm_cmpgt_epi16( a, cur ), _mm_cmpgt_epi16( b, cur ) ); };
      const __m128i suppress = _mm_or_si128( _mm_or_si128( _mm_and_si128( is0,  larger( rgt, lft ) ), _mm_and_si128( is45,  larger( dnR, upL ) ) ),
                                             _mm_or_si128( _mm_and_si128( is90, larger( dn,  up  ) ), _mm_and_si128( is135, larger( dnL, upR ) ) ) );
      _mm_storeu_si128( ( __m128i* ) &dir[x], _mm_andnot_si128( suppress, cur ) );
    }
    for( ; x < width; x++ )
    {
      suppressOne( &mag[x], dir[x] );
    }
  }
}

// Canny::doubleThresholdPlane: the largest magnitude and the classification of the samples, eight per step
template<X86_VEXT vext>
static void doubleThreshold_SIMD( Pel* buf, const ptrdiff_t stride, const int width, const int height, const int bitDepth, const double lowThresholdRatio, const int highThresholdRatio )
{
  const Pel strongPel = ( static_cast<Pel>( 1 ) << bitDepth ) - 1;
  const Pel weakPel   = ( static_cast<Pel>( 1 ) << ( bitDepth - 1 ) ) - 1;

  __m128i vmax    = _mm_setzero_si128();
  Pel     maxTail = 0;
  for( int y = 0; y < height; y++ )
  {
    const Pel* row = &buf[y * stride];
    int x = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      vmax = _mm_max_epi16( vmax, _mm_loadu_si128( ( const __m128i* ) &row[x] ) );
    }
    for( ; x < width; x++ )
    {
      maxTail = std::max<Pel>( maxTail, row[x] );
    }
  }
  vmax = _mm_max_epi16( vmax, _mm_shuffle_epi32( vmax, 0x4e ) );
  vmax = _mm_max_epi16( vmax, _mm_shuffle_epi32( vmax, 0xb1 ) );
  vmax = _mm_max_epi16( vmax, _mm_shufflelo_epi16( vmax, 0xb1 ) );
  const Pel highest = std::max<Pel>( maxTail, Pel( _mm_extract_epi16( vmax, 0 ) ) );

  // the thresholds as in the C version
  const Pel lowThreshold  = static_cast<Pel>( lowThresholdRatio * highest );
  const Pel highThreshold = Clip3( 0, ( 1 << bitDepth ) - 1, highThresholdRatio * lowThreshold );

  const __m128i vlow    = _mm_set1_epi16( lowThreshold );
  const __m128i vhigh   = _mm_set1_epi16( highThreshold );
  const __m128i vstrong = _mm_set1_epi16( strongPel );
  const __m128i vweak   = _mm_set1_epi16( weakPel );
  for( int y = 0; y < height; y++ )
  {
    Pel* row = &buf[y * stride];
    int x = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      const __m128i v      = _mm_loadu_si128( ( const __m128i* ) &row[x] );
      const __m128i weak   = _mm_and_si128( _mm_cmpgt_epi16( v, vlow ), vweak );
      const __m128i result = _mm_blendv_epi8( weak, vstrong, _mm_cmpgt_epi16( v, vhigh ) );
      _mm_storeu_si128( ( __m128i* ) &row[x], result );
    }
    for( ; x < width; x++ )
    {
      row[x] = row[x] > highThreshold ? strongPel : row[x] > lowThreshold ? weakPel : 0;
    }
  }
}

#if ENABLE_SIMD_OPT_FGA
template<X86_VEXT vext>
void Canny::_initFGACannyX86()
{
  gradient             = gradient_SIMD<vext>;
  suppressNonMaxPlane  = suppressNonMax_SIMD<vext>;
  doubleThresholdPlane = doubleThreshold_SIMD<vext>;
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
#if  ENABLE_SIMD_OPT_MCTF
  calcVar  = calcVarSse<vext>;
  calcMean = calcMeanSSE<vext>;
#endif
}

template void FGAnalyzer::_initFGAnalyzerX86<SIMDX86>();
#endif

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86
//! \}

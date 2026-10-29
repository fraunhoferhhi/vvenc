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
/** \file     IntraPredX86.h
    \brief    SIMD for IntraPrediction
*/

#pragma once

#include "CommonDefX86.h"
#include "Rom.h"
#include "IntraPrediction.h"
#include "InterpolationFilter.h"

#include "Unit.h"

#if ENABLE_SIMD_OPT_INTRAPRED
#ifdef TARGET_SIMD_X86
//! \ingroup CommonLib
//! \{

namespace vvenc {

//#define USE_AVX2
template< X86_VEXT vext >
void IntraPredAngleChroma_SIMD(int16_t* pDst,const ptrdiff_t dstStride,int16_t* pBorder,int width,int height,int deltaPos,int intraPredAngle)
{
  int deltaInt;
  int deltaFract;
  int refMainIndex;
  __m128i voffset = _mm_set1_epi16(16);
  if( width >= 8 )
  {
    if( vext >= AVX2 )
    {
#ifdef USE_AVX2
      if (( width & 15 ) == 0 )
      {
       int deltaInt;
        int deltaFract;
        int refMainIndex;

        __m256i voffset = _mm256_set1_epi16(16);
        for (int k=0; k<height; k++) {

          deltaInt   = deltaPos >> 5;
          deltaFract = deltaPos & (32 - 1);

          __m256i vfract = _mm256_set1_epi16(deltaFract);
          __m256i v32minfract = _mm256_set1_epi16(32-deltaFract);
          // Do linear filtering
          for (int l=0; l<width; l+=16) {
            refMainIndex   = l+ deltaInt+1;
            __m256i vpred0 = _mm256_lddqu_si256((__m256i*)&pBorder[refMainIndex]);
            __m256i vpred1 = _mm256_lddqu_si256((__m256i*)&pBorder[refMainIndex+1]);
            vpred0 = _mm256_mullo_epi16(v32minfract, vpred0);
            vpred1 = _mm256_mullo_epi16(vfract, vpred1);
            __m256i vpred = _mm256_srli_epi16(_mm256_add_epi16(_mm256_add_epi16(vpred0, vpred1), voffset), 5);
            _mm256_storeu_si256((__m256i*)&pDst[l], vpred);
          }
          pDst+=dstStride;
          deltaPos += intraPredAngle;
        }
      }
      else // width==8
      {
        for (int k=0; k<height; k++)
        {
          deltaInt   = deltaPos >> 5;
          deltaFract = deltaPos & (32 - 1);

          __m128i vfract = _mm_set1_epi16(deltaFract);
          __m128i v32minfract = _mm_set1_epi16(32-deltaFract);
          // Do linear filtering
          for (int l=0; l<width; l+=8) {
            refMainIndex        = l+ deltaInt+1;
            __m128i vpred0 = _mm_loadu_si128((__m128i*)&pBorder[refMainIndex]);
            __m128i vpred1 = _mm_loadu_si128((__m128i*)&pBorder[refMainIndex+1]);
            vpred0 = _mm_mullo_epi16(v32minfract, vpred0);
            vpred1 = _mm_mullo_epi16(vfract, vpred1);
            __m128i vpred = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(vpred0, vpred1), voffset), 5);
            _mm_storeu_si128((__m128i*)&pDst[l], vpred);
          }
          deltaPos += intraPredAngle;

          pDst+=dstStride;
        }

      }
#endif
    }  //AVX2
    else
    {
      for (int k=0; k<height; k++) {
        deltaInt   = deltaPos >> 5;
        deltaFract = deltaPos & (32 - 1);

        __m128i vfract = _mm_set1_epi16(deltaFract);
        __m128i v32minfract = _mm_set1_epi16(32-deltaFract);
        // Do linear filtering
        for (int l=0; l<width; l+=8) {
          refMainIndex        = l+ deltaInt+1;
          __m128i vpred0 = _mm_loadu_si128((__m128i*)&pBorder[refMainIndex]);
          __m128i vpred1 = _mm_loadu_si128((__m128i*)&pBorder[refMainIndex+1]);
          vpred0 = _mm_mullo_epi16(v32minfract, vpred0);
          vpred1 = _mm_mullo_epi16(vfract, vpred1);
          __m128i vpred = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(vpred0, vpred1), voffset), 5);
          _mm_storeu_si128((__m128i*)&pDst[l], vpred);
        }
        deltaPos += intraPredAngle;
        pDst+=dstStride;
      }
    }
  }
  else if( width == 4 )
  {
    for (int k=0; k<height; k++) {
      deltaInt   = deltaPos >> 5;
      deltaFract = deltaPos & (32 - 1);

      __m128i vfract = _mm_set1_epi16(deltaFract);
      __m128i v32minfract = _mm_set1_epi16(32-deltaFract);
      // Do linear filtering
      refMainIndex        = deltaInt+1;
      __m128i vpred0 = _mm_loadu_si128((__m128i*)&pBorder[refMainIndex]);
      __m128i vpred1 = _mm_loadu_si128((__m128i*)&pBorder[refMainIndex+1]);
      vpred0 = _mm_mullo_epi16(v32minfract, vpred0);
      vpred1 = _mm_mullo_epi16(vfract, vpred1);
      __m128i vpred = _mm_srli_epi16(_mm_add_epi16(_mm_add_epi16(vpred0, vpred1), voffset), 5);
      _vv_storel_epi64( ( __m128i * )(pDst ), vpred);
      deltaPos += intraPredAngle;
      pDst+=dstStride;
    }
  }
  else
  {
    for (int y = 0; y<height; y++)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & (32 - 1);

      // Do linear filtering
      const Pel* pRM = pBorder + deltaInt + 1;
      int lastRefMainPel = *pRM++;

      for( int x = 0; x < 2; pRM++, x++ )
      {
        int thisRefMainPel = *pRM;
        pDst[x + 0] = ( Pel ) ( ( ( 32 - deltaFract )*lastRefMainPel + deltaFract*thisRefMainPel + 16 ) >> 5 );
        lastRefMainPel = thisRefMainPel;
      }
      deltaPos += intraPredAngle;
      pDst += dstStride;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

// Angular luma prediction, as IntraPredAngleLuma_Core: row y (deltaPos += intraPredAngle per row) interpolates
//   pred[x] = ( f0 * r[x - 1] + f1 * r[x] + f2 * r[x + 1] + f3 * r[x + 2] + 32 ) >> 6,  r = refMain + ( deltaPos >> 5 ) + 1,
// with the 4-tap filter f of the row's fraction deltaPos & 31: the cubic one (InterpolationFilter's chroma table, then
// clipped, as only it has negative taps) or the Gaussian ( 16 - h, 32 - h, 16 + h, h ), h = fraction >> 1.
// Per group of 8 (16 with AVX2) outputs: four unaligned loads at r - 1, r, r + 1, r + 2; unpacklo / unpackhi pair them as
// (r[x - 1], r[x]) and (r[x + 1], r[x + 2]); two pmaddwd with the row's coefficient pairs (f0, f1) and (f2, f3), broadcast
// once per row; +32, >> 6, packs. unpack and packs both work within 128-bit lanes, so the outputs come out in order
// without horizontal adds or permutes. Exact: the sums stay far inside int32 for samples up to 15 bit.
// Width 4: two rows per register with 64-bit loads, so nothing beyond r[width + 1] is read (as for wider blocks).

// the row's coefficient pairs (f0, f1) and (f2, f3), each as one int32 with f0 / f2 in the low half
static inline void intraAngleCoeffPairs( int deltaFract, bool useCubicFilter, int32_t& f01, int32_t& f23 )
{
  if( useCubicFilter )
  {
    const TFilterCoeff* f = InterpolationFilter::getChromaFilterTable( deltaFract );
    memcpy( &f01, &f[0], sizeof( f01 ) );
    memcpy( &f23, &f[2], sizeof( f23 ) );
  }
  else
  {
    const int h = deltaFract >> 1;
    f01 = ( 16 - h ) | ( ( 32 - h ) << 16 );
    f23 = ( 16 + h ) | ( h << 16 );
  }
}

// 4 outputs ( lo: x0..3 of the 128-bit lane ) or 4 more ( hi ), as int32 before rounding
static inline __m128i intraAngleMadd( const __m128i& p01, const __m128i& p23, const __m128i& f01, const __m128i& f23 )
{
  return _mm_add_epi32( _mm_madd_epi16( p01, f01 ), _mm_madd_epi16( p23, f23 ) );
}

template< X86_VEXT vext >
void IntraPredAngleLumaCore_SIMD( int16_t* pDstBuf, const ptrdiff_t dstStride, int16_t* refMain, int width, int height, int deltaPos, int intraPredAngle, const TFilterCoeff* ff_unused, const bool useCubicFilter, const ClpRng& clpRng )
{
  const __m128i voffset = _mm_set1_epi32( 32 );
  const __m128i vmin    = _mm_set1_epi16( clpRng.min() );
  const __m128i vmax    = _mm_set1_epi16( clpRng.max() );

  if( width == 4 )
  {
    // two rows per register: the low 64 bits hold row y, the high 64 bits row y + 1
    for( int y = 0; y < height; y += 2, pDstBuf += 2 * dstStride )
    {
      const bool    two = y + 1 < height;
      const int     d0  = deltaPos, d1 = two ? deltaPos + intraPredAngle : deltaPos;
      const int16_t* r0 = refMain + ( d0 >> 5 ) + 1;
      const int16_t* r1 = refMain + ( d1 >> 5 ) + 1;
      int32_t a01, a23, b01, b23;
      intraAngleCoeffPairs( d0 & 31, useCubicFilter, a01, a23 );
      intraAngleCoeffPairs( d1 & 31, useCubicFilter, b01, b23 );

      // [ r0[k - 1 .. k + 2] | r1[k - 1 .. k + 2] ] for the four taps k = 0 .. 3
      const __m128i t0 = _mm_unpacklo_epi64( _vv_loadl_epi64( ( const __m128i* ) &r0[-1] ), _vv_loadl_epi64( ( const __m128i* ) &r1[-1] ) );
      const __m128i t1 = _mm_unpacklo_epi64( _vv_loadl_epi64( ( const __m128i* ) &r0[ 0] ), _vv_loadl_epi64( ( const __m128i* ) &r1[ 0] ) );
      const __m128i t2 = _mm_unpacklo_epi64( _vv_loadl_epi64( ( const __m128i* ) &r0[ 1] ), _vv_loadl_epi64( ( const __m128i* ) &r1[ 1] ) );
      const __m128i t3 = _mm_unpacklo_epi64( _vv_loadl_epi64( ( const __m128i* ) &r0[ 2] ), _vv_loadl_epi64( ( const __m128i* ) &r1[ 2] ) );

      // unpacklo pairs row y, unpackhi row y + 1
      __m128i s0 = intraAngleMadd( _mm_unpacklo_epi16( t0, t1 ), _mm_unpacklo_epi16( t2, t3 ), _mm_set1_epi32( a01 ), _mm_set1_epi32( a23 ) );
      __m128i s1 = intraAngleMadd( _mm_unpackhi_epi16( t0, t1 ), _mm_unpackhi_epi16( t2, t3 ), _mm_set1_epi32( b01 ), _mm_set1_epi32( b23 ) );
      s0 = _mm_srai_epi32( _mm_add_epi32( s0, voffset ), 6 );
      s1 = _mm_srai_epi32( _mm_add_epi32( s1, voffset ), 6 );
      __m128i v = _mm_packs_epi32( s0, s1 );
      if( useCubicFilter ) v = _mm_min_epi16( vmax, _mm_max_epi16( vmin, v ) );

      _vv_storel_epi64( ( __m128i* ) pDstBuf, v );
      if( two ) _vv_storel_epi64( ( __m128i* ) &pDstBuf[dstStride], _mm_unpackhi_epi64( v, v ) );
      deltaPos += 2 * intraPredAngle;
    }
    return;
  }

  if( width < 8 )
  {
    THROW( "Unsupported size in IntraPredAngleCore_SIMD" );
  }

#if USE_AVX2
  if( vext >= AVX2 && width >= 16 )
  {
    const __m256i voffset256 = _mm256_set1_epi32( 32 );
    const __m256i vmin256    = _mm256_set1_epi16( clpRng.min() );
    const __m256i vmax256    = _mm256_set1_epi16( clpRng.max() );

    for( int y = 0; y < height; y++, pDstBuf += dstStride, deltaPos += intraPredAngle )
    {
      const int16_t* r = refMain + ( deltaPos >> 5 ) + 1;
      int32_t f01, f23;
      intraAngleCoeffPairs( deltaPos & 31, useCubicFilter, f01, f23 );
      const __m256i vf01 = _mm256_set1_epi32( f01 );
      const __m256i vf23 = _mm256_set1_epi32( f23 );

      for( int x = 0; x < width; x += 16 )
      {
        const __m256i t0 = _mm256_loadu_si256( ( const __m256i* ) &r[x - 1] );
        const __m256i t1 = _mm256_loadu_si256( ( const __m256i* ) &r[x    ] );
        const __m256i t2 = _mm256_loadu_si256( ( const __m256i* ) &r[x + 1] );
        const __m256i t3 = _mm256_loadu_si256( ( const __m256i* ) &r[x + 2] );

        __m256i lo = _mm256_add_epi32( _mm256_madd_epi16( _mm256_unpacklo_epi16( t0, t1 ), vf01 ), _mm256_madd_epi16( _mm256_unpacklo_epi16( t2, t3 ), vf23 ) );
        __m256i hi = _mm256_add_epi32( _mm256_madd_epi16( _mm256_unpackhi_epi16( t0, t1 ), vf01 ), _mm256_madd_epi16( _mm256_unpackhi_epi16( t2, t3 ), vf23 ) );
        lo = _mm256_srai_epi32( _mm256_add_epi32( lo, voffset256 ), 6 );
        hi = _mm256_srai_epi32( _mm256_add_epi32( hi, voffset256 ), 6 );
        __m256i v = _mm256_packs_epi32( lo, hi );
        if( useCubicFilter ) v = _mm256_min_epi16( vmax256, _mm256_max_epi16( vmin256, v ) );
        _mm256_storeu_si256( ( __m256i* ) &pDstBuf[x], v );
      }
    }
    _mm256_zeroupper();
    return;
  }
#endif

  for( int y = 0; y < height; y++, pDstBuf += dstStride, deltaPos += intraPredAngle )
  {
    const int16_t* r = refMain + ( deltaPos >> 5 ) + 1;
    int32_t f01, f23;
    intraAngleCoeffPairs( deltaPos & 31, useCubicFilter, f01, f23 );
    const __m128i vf01 = _mm_set1_epi32( f01 );
    const __m128i vf23 = _mm_set1_epi32( f23 );

    for( int x = 0; x < width; x += 8 )
    {
      const __m128i t0 = _mm_loadu_si128( ( const __m128i* ) &r[x - 1] );
      const __m128i t1 = _mm_loadu_si128( ( const __m128i* ) &r[x    ] );
      const __m128i t2 = _mm_loadu_si128( ( const __m128i* ) &r[x + 1] );
      const __m128i t3 = _mm_loadu_si128( ( const __m128i* ) &r[x + 2] );

      __m128i lo = intraAngleMadd( _mm_unpacklo_epi16( t0, t1 ), _mm_unpacklo_epi16( t2, t3 ), vf01, vf23 );
      __m128i hi = intraAngleMadd( _mm_unpackhi_epi16( t0, t1 ), _mm_unpackhi_epi16( t2, t3 ), vf01, vf23 );
      lo = _mm_srai_epi32( _mm_add_epi32( lo, voffset ), 6 );
      hi = _mm_srai_epi32( _mm_add_epi32( hi, voffset ), 6 );
      __m128i v = _mm_packs_epi32( lo, hi );
      if( useCubicFilter ) v = _mm_min_epi16( vmax, _mm_max_epi16( vmin, v ) );
      _mm_storeu_si128( ( __m128i* ) &pDstBuf[x], v );
    }
  }
}

template< X86_VEXT vext, int W >
void  IntraPredSampleFilter_SIMD(PelBuf& dstBuf, const CPelBuf& Src)
{
  const int iWidth  = dstBuf.width;
  const int iHeight = dstBuf.height;
  Pel* pDst         = dstBuf.buf;
  const ptrdiff_t  dstStride=dstBuf.stride;

  const Pel* ptrSrc = Src.buf;
  const ptrdiff_t  srcStride=Src.stride;

  const int scale = ((floorLog2(iWidth * iHeight) - 2) >> 2);
  CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 2");

#if USE_AVX2
  if( W > 8 )
  {
    __m256i tmplo,tmphi;
    __m256i w32 = _mm256_set_epi32(32,32,32,32,32,32,32,32);
    __m256i wl16,wl16start;

    wl16start = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,0,0,0,2,8,32);
    
    if (scale==1)
    {
      wl16start = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,1,2,4,8,16,32);
    }
    else if (scale==2)
    {
      wl16start = _mm256_set_epi16(0,0,0,0,1,1,2,2,4,4,8,8,16,16,32,32);
    }
    
    for (int y = 0; y < iHeight; y++)
    {
      int wT = 32 >> std::min(31, ((y << 1) >> scale));

      __m256i wt16 = _mm256_set_epi16(wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT,wT);
      __m256i x16left = _mm256_broadcastw_epi16(_mm_loadu_si128 ((__m128i const *) (ptrSrc+((y+1)+srcStride))));
      if (wT)
      {
        for (int x = 0; x < iWidth; x+=16)
        {
          if (x==0)
          {
            wl16=wl16start;

            __m256i x16top = _mm256_loadu_si256((__m256i *) (ptrSrc+x+1)); // load top
            __m256i x16dst = _mm256_loadu_si256((const __m256i *) (pDst+y*dstStride+x)); // load dst

            tmphi = _mm256_sub_epi16(x16left,x16dst);
            tmplo = _mm256_mullo_epi16(tmphi,wl16);  //wL * left-val
            tmphi = _mm256_mulhi_epi16(tmphi,wl16);  //wL * left-val
            __m256i leftlo = _mm256_unpacklo_epi16(tmplo,tmphi);
            __m256i lefthi = _mm256_unpackhi_epi16(tmplo,tmphi);

            x16top = _mm256_sub_epi16(x16top,x16dst);
            tmplo = _mm256_mullo_epi16(x16top,wt16);    // wT*top-val
            tmphi = _mm256_mulhi_epi16(x16top,wt16);    // wT*top-val
            __m256i toplo = _mm256_unpacklo_epi16(tmplo,tmphi);
            __m256i tophi = _mm256_unpackhi_epi16(tmplo,tmphi);

            __m256i dstlo = _mm256_add_epi32(leftlo,toplo);
            __m256i dsthi = _mm256_add_epi32(lefthi,tophi);
            dstlo = _mm256_add_epi32(dstlo,w32);
            dsthi = _mm256_add_epi32(dsthi,w32);

            dstlo =  _mm256_srai_epi32(dstlo,6);
            dsthi =  _mm256_srai_epi32(dsthi,6);

            dstlo =  _mm256_packs_epi32(dstlo,dsthi);
            dstlo =  _mm256_permute4x64_epi64 ( dstlo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

            dstlo = _mm256_adds_epi16(dstlo,x16dst);
            _mm256_storeu_si256( ( __m256i * )(pDst+y*dstStride+x), dstlo );
          }
          else
          {
            __m256i x16top = _mm256_loadu_si256((__m256i *) (ptrSrc+x+1)); // load top
            __m256i x16dst = _mm256_loadu_si256((const __m256i *) (pDst+y*dstStride+x)); // load dst

            x16top = _mm256_sub_epi16(x16top,x16dst);
            tmplo = _mm256_mullo_epi16(x16top,wt16);    // wT*top-val
            tmphi = _mm256_mulhi_epi16(x16top,wt16);    // wT*top-val
            __m256i toplo = _mm256_unpacklo_epi16(tmplo,tmphi);
            __m256i tophi = _mm256_unpackhi_epi16(tmplo,tmphi);

            __m256i dstlo = _mm256_add_epi32(toplo,w32);
            __m256i dsthi = _mm256_add_epi32(tophi,w32);

            dstlo =  _mm256_srai_epi32(dstlo,6);
            dsthi =  _mm256_srai_epi32(dsthi,6);

            dstlo =  _mm256_packs_epi32(dstlo,dsthi);
            dstlo =  _mm256_permute4x64_epi64 ( dstlo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

            dstlo = _mm256_adds_epi16(dstlo,x16dst);
            _mm256_storeu_si256( ( __m256i * )(pDst+y*dstStride+x), dstlo );
          }
        }  // for x
      }
      else
      { // wT =0
        wl16=wl16start;

        __m256i x16dst = _mm256_loadu_si256((const __m256i *) (pDst+y*dstStride)); // load dst

        tmphi = _mm256_sub_epi16(x16left,x16dst);
        tmplo = _mm256_mullo_epi16(tmphi,wl16);  //wL * left-val
        tmphi = _mm256_mulhi_epi16(tmphi,wl16);  //wL * left-val
        __m256i leftlo = _mm256_unpacklo_epi16(tmplo,tmphi);
        __m256i lefthi = _mm256_unpackhi_epi16(tmplo,tmphi);

        __m256i dstlo = _mm256_add_epi32(leftlo,w32);
        __m256i dsthi = _mm256_add_epi32(lefthi,w32);

        dstlo =  _mm256_srai_epi32(dstlo,6);
        dsthi =  _mm256_srai_epi32(dsthi,6);

        dstlo =  _mm256_packs_epi32(dstlo,dsthi);
        dstlo =  _mm256_permute4x64_epi64 ( dstlo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

        dstlo = _mm256_adds_epi16(dstlo,x16dst);
        _mm256_storeu_si256( ( __m256i * )(pDst+y*dstStride), dstlo );
      }
    }
  }
  else
#endif
  {
    __m128i tmplo8,tmphi8;
    __m128i w32_8 = _mm_set_epi32(32,32,32,32);
    __m128i wl8start,wl8start2;
    CHECK(scale < 0 || scale > 2, "PDPC: scale < 0 || scale > 2");

    wl8start = _mm_set_epi16(0,0,0,0,0,2,8,32);
    wl8start2 = _mm_set_epi16(0,0,0,0,0,0,0,0);
    
    if (scale==1)
    {
      wl8start = _mm_set_epi16(0,0,1,2,4,8,16,32);
      wl8start2 = _mm_set_epi16(0,0,0,0,0,0,0,0);
    }
    else if (scale==2)
    {
      wl8start = _mm_set_epi16(4,4,8,8,16,16,32,32);
      wl8start2 = _mm_set_epi16(0,0,0,0,1,1,2,2);
    }

    __m128i wl8 = wl8start;
    for (int y = 0; y < iHeight; y++)
    {
      int wT = 32 >> std::min(31, ((y << 1) >> scale));

      __m128i wt8 = _mm_set_epi16(wT,wT,wT,wT,wT,wT,wT,wT);
      __m128i x8left;


      if ( W == 4 )
      {
        x8left = _mm_loadu_si64 ((__m128i const *) (ptrSrc+((y+1)+srcStride)));
      }
      else if ( W == 2 )
      {
        x8left = _mm_loadu_si32 ((__m128i const *) (ptrSrc+((y+1)+srcStride)));
      }
      else
      {
        x8left = _mm_loadu_si128 ((__m128i const *) (ptrSrc+((y+1)+srcStride)));
      }
      x8left =_mm_shufflelo_epi16(x8left,0);
      x8left =_mm_shuffle_epi32(x8left,0);


      if (wT)
      {
        for (int x = 0; x < iWidth; x+=8)
        {
          if (x>8)
          {
            __m128i x8top;
            __m128i x8dst;

            if ( W == 4 )
            {
              x8top =  _mm_loadu_si64((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si64((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            else if ( W == 2 )
            {
              x8top =  _mm_loadu_si32((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si32((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            else
            {
              x8top =  _mm_loadu_si128((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si128((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }

            tmphi8 = _mm_sub_epi16(x8top,x8dst);
            tmplo8 = _mm_mullo_epi16(tmphi8,wt8);    // wT*top-val
            tmphi8 = _mm_mulhi_epi16(tmphi8,wt8);    // wT*top-val
            __m128i toplo8 = _mm_unpacklo_epi16(tmplo8,tmphi8);
            __m128i tophi8 = _mm_unpackhi_epi16(tmplo8,tmphi8);

            __m128i dstlo8 = _mm_add_epi32(toplo8,w32_8);
            __m128i dsthi8 = _mm_add_epi32(tophi8,w32_8);

            dstlo8 =  _mm_srai_epi32(dstlo8,6);
            dsthi8 =  _mm_srai_epi32(dsthi8,6);

            dstlo8 =  _mm_packs_epi32(dstlo8,dsthi8);
            dstlo8 =  _mm_adds_epi16(dstlo8,x8dst);

            _mm_storeu_si128(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );

          }
          else // x<=8
          {
            
            if (x==0)
              wl8=wl8start;
            else if (x==8)
              wl8=wl8start2;

            __m128i x8top;
            __m128i x8dst;

            if ( W == 4 )
            {
              x8top =  _mm_loadu_si64((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si64((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            else if ( W == 2 )
            {
              x8top =  _mm_loadu_si32((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si32((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            else
            {
              x8top =  _mm_loadu_si128((__m128i *) (ptrSrc+x+1)); // load top
              x8dst =  _mm_loadu_si128((const __m128i *) (pDst+y*dstStride+x)); // load dst
            }
            tmphi8 = _mm_sub_epi16(x8left,x8dst);
            tmplo8 = _mm_mullo_epi16(tmphi8,wl8);  //wL * left-val
            tmphi8 = _mm_mulhi_epi16(tmphi8,wl8);  //wL * left-val
            __m128i leftlo8 = _mm_unpacklo_epi16(tmplo8,tmphi8);
            __m128i lefthi8 = _mm_unpackhi_epi16(tmplo8,tmphi8);

            tmphi8 = _mm_sub_epi16(x8top,x8dst);
            tmplo8 = _mm_mullo_epi16(tmphi8,wt8);    // wT*top-val
            tmphi8 = _mm_mulhi_epi16(tmphi8,wt8);    // wT*top-val
            __m128i toplo8 = _mm_unpacklo_epi16(tmplo8,tmphi8);
            __m128i tophi8 = _mm_unpackhi_epi16(tmplo8,tmphi8);

            __m128i dstlo8 = _mm_add_epi32(leftlo8,toplo8);
            __m128i dsthi8 = _mm_add_epi32(lefthi8,tophi8);
            dstlo8 = _mm_add_epi32(dstlo8,w32_8);
            dsthi8 = _mm_add_epi32(dsthi8,w32_8);

            dstlo8 =  _mm_srai_epi32(dstlo8,6);
            dsthi8 =  _mm_srai_epi32(dsthi8,6);

            dstlo8 =  _mm_packs_epi32(dstlo8,dsthi8);
            dstlo8 =  _mm_adds_epi16(dstlo8,x8dst);

            if (W>=8)
              _mm_storeu_si128(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );
            else if (W==4)
              _vv_storel_epi64(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );
            else if (W==2)
              _mm_storeu_si32(( __m128i * )(pDst+y*dstStride+x),(dstlo8) );
          }
        }
      }
      else //wT =0
      {
        for (int x = 0; x < std::min(iWidth,16); x+=8)
        {
          if (x==0)
            wl8=wl8start;
          else
            wl8=wl8start2;

          __m128i x8dst ;

          if ( W == 4 )
          {
            x8dst =  _mm_loadu_si64((const __m128i *) (pDst+y*dstStride+x)); // load dst
          }
          else if ( W == 2 )
          {
            x8dst =  _mm_loadu_si32((const __m128i *) (pDst+y*dstStride+x)); // load dst
          }
          else
          {
            x8dst =  _mm_loadu_si128((const __m128i *) (pDst+y*dstStride+x)); // load dst
          }
          tmphi8 = _mm_sub_epi16(x8left,x8dst);
          tmplo8 = _mm_mullo_epi16(tmphi8,wl8);  //wL * left-val
          tmphi8 = _mm_mulhi_epi16(tmphi8,wl8);  //wL * left-val
          __m128i leftlo8 = _mm_unpacklo_epi16(tmplo8,tmphi8);
          __m128i lefthi8 = _mm_unpackhi_epi16(tmplo8,tmphi8);

          __m128i dstlo8 = _mm_add_epi32(leftlo8,w32_8);
          __m128i dsthi8 = _mm_add_epi32(lefthi8,w32_8);

          dstlo8 =  _mm_srai_epi32(dstlo8,6);
          dsthi8 =  _mm_srai_epi32(dsthi8,6);

          dstlo8 =  _mm_packs_epi32(dstlo8,dsthi8);
          dstlo8 =  _mm_adds_epi16(dstlo8,x8dst);

          if (W>=8)
            _mm_storeu_si128(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );
          else if (W==4)
            _vv_storel_epi64(( __m128i * )(pDst+y*dstStride+x), (dstlo8) );
          else if (W==2)
            _mm_storeu_si32(( __m128i * )(pDst+y*dstStride+x),(dstlo8) );
        }
      }
    }
  }


}

template< X86_VEXT vext >
void  IntraPredSampleFilter_SIMD(PelBuf& dstBuf, const CPelBuf& srcBuf)
{
  const int iWidth  = dstBuf.width;

  if (iWidth>8)
    IntraPredSampleFilter_SIMD<vext,16>(dstBuf, srcBuf);
  else  if (iWidth==8)
    IntraPredSampleFilter_SIMD<vext,8>(dstBuf, srcBuf);
  else  if (iWidth==4)
    IntraPredSampleFilter_SIMD<vext,4>(dstBuf, srcBuf);
  else
    IntraPredSampleFilter_SIMD<vext,2>(dstBuf, srcBuf);

#if USE_AVX2
  _mm256_zeroupper();
#endif
}


// Planar, as xPredIntraPlanar_Core. Its running sums written out (( L << log2W ) + ( x + 1 ) * ( TR - L ) ==
// ( W - 1 - x ) * L + ( x + 1 ) * TR, and the same for the vertical term), every sample is
//   pred[y][x] = ( ( W - 1 - x ) * H * L[y] + ( x + 1 ) * H * TR + ( H - 1 - y ) * W * T[x] + ( y + 1 ) * W * BL + W * H )
//                >> ( 1 + log2W + log2H ),
// with T / L the above / left reference samples, TR = T[W] and BL = L[H]. Each of the two halves is one pmaddwd on
// interleaved (sample, corner) pairs with pre-shifted weights <= 64 * 64 = 4096 (the SMOOTH predictor form of dav1d and
// libaom): the pairs (T[x], BL) and the column weights ( ( W - 1 - x ) * H, ( x + 1 ) * H ) are set up once per block, a
// row only broadcasts (L[y], TR) and its weight pair ( ( H - 1 - y ) * W, ( y + 1 ) * W ). No per-row multiplies; the
// sums stay below 2^26 for samples up to 12 bit and below 2^31 for any int16 sample.
// unpacklo / unpackhi pair the columns 0..3 / 4..7 of each 128-bit half, and packs puts them back in order.

// The pairs (T[x], BL) and the column weights of 8 columns, as the two pmaddwd operands for columns 0..3 (lo) and 4..7 (hi)
struct PlanarCols8 { __m128i tbLo, tbHi, wLo, wHi; };

template<X86_VEXT vext>
void xPredIntraPlanar_SIMD( PelBuf& pDst, const CPelBuf& pSrc )
{
  const int width  = pDst.width;
  const int height = pDst.height;
  const int log2W  = floorLog2( width );
  const int log2H  = floorLog2( height );
  const Pel* top   = pSrc.buf + 1;                 // T[x] = pSrc.at( x + 1, 0 )
  const Pel* left  = pSrc.buf + pSrc.stride + 1;   // L[y] = pSrc.at( y + 1, 1 )
  const int  tr    = top[width];
  const int  bl    = left[height];
  const __m128i vshift = _mm_cvtsi32_si128( 1 + log2W + log2H );
  const __m128i vlog2H = _mm_cvtsi32_si128( log2H );
  Pel* pred = pDst.buf;

#if USE_AVX2
  if( vext >= AVX2 && width >= 16 )
  {
    // groups of 16 columns: in each 128-bit half, unpacklo pairs its columns 0..3 and unpackhi its columns 4..7
    __m256i tbLo[MAX_CU_SIZE >> 4], tbHi[MAX_CU_SIZE >> 4], wLo[MAX_CU_SIZE >> 4], wHi[MAX_CU_SIZE >> 4];
    const __m256i vbl = _mm256_set1_epi16( bl );
    for( int g = 0; g < ( width >> 4 ); g++ )
    {
      const __m256i x  = _mm256_add_epi16( _mm256_setr_epi16( 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 ), _mm256_set1_epi16( g << 4 ) );
      const __m256i wa = _mm256_sll_epi16( _mm256_sub_epi16( _mm256_set1_epi16( width - 1 ), x ), vlog2H );   // ( W - 1 - x ) * H
      const __m256i wb = _mm256_sll_epi16( _mm256_add_epi16( x, _mm256_set1_epi16( 1 ) ), vlog2H );           // ( x + 1 ) * H
      const __m256i t  = _mm256_loadu_si256( ( const __m256i* ) &top[g << 4] );
      tbLo[g] = _mm256_unpacklo_epi16( t, vbl );
      tbHi[g] = _mm256_unpackhi_epi16( t, vbl );
      wLo[g]  = _mm256_unpacklo_epi16( wa, wb );
      wHi[g]  = _mm256_unpackhi_epi16( wa, wb );
    }
    const __m256i voffset = _mm256_set1_epi32( 1 << ( log2W + log2H ) );

    for( int y = 0; y < height; y++, pred += pDst.stride )
    {
      const __m256i vlt = _mm256_set1_epi32( int( uint32_t( uint16_t( left[y] ) ) | ( uint32_t( uint16_t( tr ) ) << 16 ) ) );                                         // (L[y], TR)
      const __m256i vrw = _mm256_set1_epi32( ( ( height - 1 - y ) << log2W ) | ( ( ( y + 1 ) << log2W ) << 16 ) );            // row weights
      for( int g = 0; g < ( width >> 4 ); g++ )
      {
        __m256i lo = _mm256_add_epi32( _mm256_madd_epi16( wLo[g], vlt ), _mm256_madd_epi16( tbLo[g], vrw ) );
        __m256i hi = _mm256_add_epi32( _mm256_madd_epi16( wHi[g], vlt ), _mm256_madd_epi16( tbHi[g], vrw ) );
        lo = _mm256_sra_epi32( _mm256_add_epi32( lo, voffset ), vshift );
        hi = _mm256_sra_epi32( _mm256_add_epi32( hi, voffset ), vshift );
        _mm256_storeu_si256( ( __m256i* ) &pred[g << 4], _mm256_packs_epi32( lo, hi ) );
      }
    }
    _mm256_zeroupper();
    return;
  }
#endif

  // groups of 8 columns (one group, partly stored, for widths 1, 2 and 4)
  PlanarCols8 cols[MAX_CU_SIZE >> 3];
  const int groups = std::max( 1, width >> 3 );
  const __m128i vbl = _mm_set1_epi16( bl );
  for( int g = 0; g < groups; g++ )
  {
    const __m128i x  = _mm_add_epi16( _mm_setr_epi16( 0, 1, 2, 3, 4, 5, 6, 7 ), _mm_set1_epi16( g << 3 ) );
    const __m128i wa = _mm_sll_epi16( _mm_sub_epi16( _mm_set1_epi16( width - 1 ), x ), vlog2H );   // ( W - 1 - x ) * H
    const __m128i wb = _mm_sll_epi16( _mm_add_epi16( x, _mm_set1_epi16( 1 ) ), vlog2H );           // ( x + 1 ) * H
    const __m128i t  = _mm_loadu_si128( ( const __m128i* ) &top[g << 3] );   // reads up to T[7] also for narrow blocks, as before
    cols[g].tbLo = _mm_unpacklo_epi16( t, vbl );
    cols[g].tbHi = _mm_unpackhi_epi16( t, vbl );
    cols[g].wLo  = _mm_unpacklo_epi16( wa, wb );
    cols[g].wHi  = _mm_unpackhi_epi16( wa, wb );
  }
  const __m128i voffset = _mm_set1_epi32( 1 << ( log2W + log2H ) );

  for( int y = 0; y < height; y++, pred += pDst.stride )
  {
    const __m128i vlt = _mm_set1_epi32( int( uint32_t( uint16_t( left[y] ) ) | ( uint32_t( uint16_t( tr ) ) << 16 ) ) );                                         // (L[y], TR)
    const __m128i vrw = _mm_set1_epi32( ( ( height - 1 - y ) << log2W ) | ( ( ( y + 1 ) << log2W ) << 16 ) );            // row weights
    for( int g = 0; g < groups; g++ )
    {
      __m128i lo = _mm_add_epi32( _mm_madd_epi16( cols[g].wLo, vlt ), _mm_madd_epi16( cols[g].tbLo, vrw ) );
      __m128i hi = _mm_add_epi32( _mm_madd_epi16( cols[g].wHi, vlt ), _mm_madd_epi16( cols[g].tbHi, vrw ) );
      lo = _mm_sra_epi32( _mm_add_epi32( lo, voffset ), vshift );
      hi = _mm_sra_epi32( _mm_add_epi32( hi, voffset ), vshift );
      const __m128i p = _mm_packs_epi32( lo, hi );
      if( width >= 8 )      _mm_storeu_si128( ( __m128i* ) &pred[g << 3], p );
      else if( width == 4 ) _vv_storel_epi64( ( __m128i* ) pred, p );
      else if( width == 2 ) _mm_storeu_si32( ( __m128i* ) pred, p );
      else                  pred[0] = ( Pel ) _mm_extract_epi16( p, 0 );
    }
  }
}

template< X86_VEXT vext>
void GetLumaRecPixel420SIMD (const int width,const int height, const Pel* pRecSrc0,const ptrdiff_t iRecStride,Pel* pDst0,const ptrdiff_t iDstStride)
{
#ifdef USE_AVX2
  if( ( width & 15 ) == 0 )    // width>=16
  {
    __m256i vzero = _mm256_set1_epi8(0);
    __m256i vfour = _mm256_set1_epi32(4);
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x += 16 )
      {
        int x2=x<<1;
        __m256i vsrc_l = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2-1]);      // -1 0 1 2 3 4 5 6
        __m256i vsrc = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2]);          // 0 1 2 3 4 5 6 7

        __m256i vsrc01 = _mm256_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
        __m256i vsrc0 = _mm256_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
        __m256i vsrc10 = _mm256_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
        vsrc10 = _mm256_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm256_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm256_add_epi32(vsrc0,vsrc10);
        __m256i vdst0 = _mm256_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile fehlt noch

        vsrc_l = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2 +15]);      // 7 8 9 10 11 12 13 14
        vsrc = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2 + 16 ]);          // 8 9 10 11 12 13 14 15

        x2+= (int)iRecStride;

        vsrc01 = _mm256_blend_epi16(vzero,vsrc_l,0x55);
        vsrc0 = _mm256_blend_epi16(vzero,vsrc,0x55);
        vsrc10 = _mm256_blend_epi16(vzero,vsrc,0xAA);
        vsrc10 = _mm256_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm256_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm256_add_epi32(vsrc0,vsrc10);
        __m256i vdst1 = _mm256_add_epi32(vsrc0,vsrc01);   // dst 4 5 6 7 32 Bit, untere Zeile fehlt noch

        // jetzt die nächste Zeile dazu
        vsrc_l = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2-1]);      // -1 0 1 2 3 4 5 6
        vsrc = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2]);          // 0 1 2 3 4 5 6 7

        vsrc01 = _mm256_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
        vsrc0 = _mm256_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
        vsrc10 = _mm256_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
        vsrc10 = _mm256_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm256_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm256_add_epi32(vsrc0,vsrc10);
        __m256i vdst01 = _mm256_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile

        vsrc_l = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2 + 15]);      // 7 8 9 10 11 12 13 14
        vsrc = _mm256_loadu_si256((__m256i*)&pRecSrc0[x2 + 16 ]);          // 8 9 10 11 12 13 14 15

        vsrc01 = _mm256_blend_epi16(vzero,vsrc_l,0x55);
        vsrc0 = _mm256_blend_epi16(vzero,vsrc,0x55);
        vsrc10 = _mm256_blend_epi16(vzero,vsrc,0xAA);
        vsrc10 = _mm256_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm256_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm256_add_epi32(vsrc0,vsrc10);
        __m256i vdst11 = _mm256_add_epi32(vsrc0,vsrc01);   // dst 4 5 6 7 32 Bit, untere Zeile

        vdst0 = _mm256_add_epi32(vdst0,vdst01);
        vdst1 = _mm256_add_epi32(vdst1,vdst11);
        vdst0 =  _mm256_add_epi32(vdst0,vfour);
        vdst1 =  _mm256_add_epi32(vdst1,vfour);
        vdst0 = _mm256_srli_epi32(vdst0,3);
        vdst1 = _mm256_srli_epi32(vdst1,3);
        vdst0 = _mm256_packus_epi32 (vdst0,vdst1);   // 16 bit
        vdst0 = _mm256_permute4x64_epi64(vdst0,0xd8);

        _mm256_storeu_si256((__m256i*)&pDst0[x], vdst0);
        //        _mm_storeu_si128((__m128i*)&pDstTmp[x], vdst0);
      }
      pDst0 += iDstStride;
      pRecSrc0 += (iRecStride<<1);
    }
  }
  else
#endif
    if( ( width & 7 ) == 0 )    // width>=8
    {
      __m128i vzero = _mm_set1_epi8(0);
      __m128i vfour = _mm_set1_epi32(4);


      for( int y = 0; y < height; y++ )
      {

        for( int x = 0; x < width; x += 8 )
        {
          int x2=x<<1;
          __m128i vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[x2-1]);      // -1 0 1 2 3 4 5 6
          __m128i vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[x2]);          // 0 1 2 3 4 5 6 7

          __m128i vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
          __m128i vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
          __m128i vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
          vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
          vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

          vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
          __m128i vdst0 = _mm_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile fehlt noch

          vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[x2 +7]);      // 7 8 9 10 11 12 13 14
          vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[x2 + 8 ]);          // 8 9 10 11 12 13 14 15

          x2+=(int)iRecStride;

          vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);
          vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);
          vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);
          vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
          vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

          vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
          __m128i vdst1 = _mm_add_epi32(vsrc0,vsrc01);   // dst 4 5 6 7 32 Bit, untere Zeile fehlt noch

          // jetzt die nächste Zeile dazu
          vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[x2-1]);      // -1 0 1 2 3 4 5 6
          vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[x2]);          // 0 1 2 3 4 5 6 7

          vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
          vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
          vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
          vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
          vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

          vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
          __m128i vdst01 = _mm_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile

          vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[x2 + 7]);      // 7 8 9 10 11 12 13 14
          vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[x2 + 8 ]);          // 8 9 10 11 12 13 14 15

          vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);
          vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);
          vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);
          vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
          vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

          vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
          __m128i vdst11 = _mm_add_epi32(vsrc0,vsrc01);   // dst 4 5 6 7 32 Bit, untere Zeile

          vdst0 = _mm_add_epi32(vdst0,vdst01);
          vdst1 = _mm_add_epi32(vdst1,vdst11);
          vdst0 =  _mm_add_epi32(vdst0,vfour);
          vdst1 =  _mm_add_epi32(vdst1,vfour);
          vdst0 = _mm_srli_epi32(vdst0,3);
          vdst1 = _mm_srli_epi32(vdst1,3);
          vdst0 = _mm_packus_epi32 (vdst0,vdst1);   // 16 bit__m256i wl16start;

          _mm_storeu_si128((__m128i*)&pDst0[x], vdst0);
          //        _mm_storeu_si128((__m128i*)&pDstTmp[x], vdst0);
        }
        pDst0 += iDstStride;
        pRecSrc0 += (iRecStride<<1);
      }
    }
    else     // width<=4
    {
      __m128i vzero = _mm_set1_epi8(0);
      __m128i vfour = _mm_set1_epi32(4);

      for( int y = 0; y < height; y++ )
      {
        __m128i vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[-1]);      // -1 0 1 2 3 4 5 6
        __m128i vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[0]);          // 0 1 2 3 4 5 6 7

        __m128i vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
        __m128i vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
        __m128i vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
        vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
        __m128i vdst0 = _mm_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile fehlt noch

        // jetzt die nächste Zeile dazu
        vsrc_l = _mm_loadu_si128((__m128i*)&pRecSrc0[iRecStride-1]);      // -1 0 1 2 3 4 5 6
        vsrc = _mm_loadu_si128((__m128i*)&pRecSrc0[iRecStride]);          // 0 1 2 3 4 5 6_mm_storeu_si32 7

        vsrc01 = _mm_blend_epi16(vzero,vsrc_l,0x55);      // -1 1 3 5  32 Bit
        vsrc0 = _mm_blend_epi16(vzero,vsrc,0x55);      // 0 2 4 6  32 Bit
        vsrc10 = _mm_blend_epi16(vzero,vsrc,0xAA);      // 1 3 5 7 32 Bit
        vsrc10 = _mm_srli_epi32(vsrc10,16);      // 1 3 5 7 32 Bit
        vsrc0 =  _mm_slli_epi32 (vsrc0,1);      // 0  2 4 6 *2

        vsrc0 =  _mm_add_epi32(vsrc0,vsrc10);
        __m128i vdst01 = _mm_add_epi32(vsrc0,vsrc01);   // dst 0 1 2 3 32 Bit, untere Zeile


        vdst0 = _mm_add_epi32(vdst0,vdst01);
        vdst0 =  _mm_add_epi32(vdst0,vfour);
        vdst0 = _mm_srli_epi32(vdst0,3);
        vdst0 = _mm_packus_epi32 (vdst0,vdst0);   // 16 bit

        if (width==4)
          _vv_storel_epi64(( __m128i * )&pDst0[0], (vdst0) );
        else if (width==2)
          _mm_storeu_si32(( __m128i * )&pDst0[0], (vdst0) );
        else
        {
          int tmp = _mm_cvtsi128_si32(vdst0);
          pDst0[0] = (Pel) tmp;
        }

        pDst0 += iDstStride;
        pRecSrc0 += (iRecStride<<1);
      }
    }
}


// Angular PDPC (positive angles), as IntraAnglePDPC_Core: for the first n = min( 3 << scale, width ) columns
//   dst[x] += ( wL[x] * ( refSide[y + off[x] + 1] - dst[x] ) + 32 ) >> 6,  wL[x] = 32 >> ( 2 * x >> scale ),
//   off[x] = ( ( x + 1 ) * invAngle + 256 ) >> 9  (the scalar code's running invAngleSum).
// The offsets depend only on the column, so they are derived once per block. Each row's left samples are built in
// registers, never through a stack array (narrow stores read back with one wide load stall store forwarding, which is
// what the previous version paid on every row): when a group of up to 8 columns reads at most 8 consecutive samples
// (always for blocks with up to 8 rows), one unaligned load of that window and a byte shuffle made once per block,
// otherwise one lane insert per column.
// The blend is one pmulhrsw: mulhrs( d, wL << 9 ) == ( wL * d + 32 ) >> 6 exactly for every int16 d (wL << 9 <= 16384,
// so | d * ( wL << 9 ) | < 2^29, and the rounding of both is the floor of the same quotient). d = left - dst cannot wrap
// for samples up to 15 bit. Lanes from n on have weight 0 and keep their value; their shuffle index stays inside the
// window and the lanes the inserts leave out are 0.

// The PDPC of one group of up to 8 columns (firstCol .. firstCol + 7) of a block: the weights wL << 9, and how to build
// the left samples of a row (window shuffle or offsets for the lane inserts).
#if defined( _MSC_VER )
#pragma warning( push )
#pragma warning( disable : 4324 )   // padded to the vector members' alignment (SIMDE's types carry an alignas on Arm)
#endif
struct PdpcColumns
{
  __m128i weights;
  __m128i shuffle;   // byte shuffle of the window starting at off[firstCol] (dense only)
  int     off[8];    // off[firstCol + i] for the lane inserts (written and read as scalars)
  int     base;      // off[firstCol]
  int     count;     // columns of this group that PDPC changes
  bool    dense;     // all of them read within 8 consecutive samples
};
#if defined( _MSC_VER )
#pragma warning( pop )
#endif

static inline void pdpcInitColumns( PdpcColumns& g, int invAngle, int scale, int firstCol, int count )
{
  // wL << 9 for columns 0 .. 15 at each scale (columns from 3 << scale on have weight 0)
  static const int16_t weights[3][16] = {
    { 32 << 9,  8 << 9,  2 << 9,       0,      0,      0,      0,      0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 32 << 9, 16 << 9,  8 << 9,  4 << 9, 2 << 9, 1 << 9,      0,      0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 32 << 9, 32 << 9, 16 << 9, 16 << 9, 8 << 9, 8 << 9, 4 << 9, 4 << 9, 2 << 9, 2 << 9, 1 << 9, 1 << 9, 0, 0, 0, 0 } };

  g.weights = _mm_loadu_si128( ( const __m128i* ) &weights[scale][firstCol] );
  g.count   = count;
  for( int i = 0; i < 8; i++ )
  {
    g.off[i] = ( ( firstCol + std::min( i, count - 1 ) + 1 ) * invAngle + 256 ) >> 9;   // lanes from count on: the last one
  }
  g.base  = g.off[0];
  g.dense = g.off[count - 1] - g.base < 8;

  // the same offsets in registers (the array above is only read as scalars): columns firstCol + 1 .. firstCol + 8,
  // clamped to the last changed column
  const __m128i vinv  = _mm_set1_epi32( invAngle );
  const __m128i vlast = _mm_set1_epi32( firstCol + count );
  const __m128i vcol0 = _mm_min_epi32( _mm_setr_epi32( firstCol + 1, firstCol + 2, firstCol + 3, firstCol + 4 ), vlast );
  const __m128i vcol1 = _mm_min_epi32( _mm_setr_epi32( firstCol + 5, firstCol + 6, firstCol + 7, firstCol + 8 ), vlast );
  const __m128i voff0 = _mm_srai_epi32( _mm_add_epi32( _mm_mullo_epi32( vcol0, vinv ), _mm_set1_epi32( 256 ) ), 9 );
  const __m128i voff1 = _mm_srai_epi32( _mm_add_epi32( _mm_mullo_epi32( vcol1, vinv ), _mm_set1_epi32( 256 ) ), 9 );

  // lane i takes sample off[i] - base of the window: the bytes 2 * ( off[i] - base ) and the next one
  const __m128i vidx  = _mm_sub_epi16( _mm_packs_epi32( voff0, voff1 ), _mm_set1_epi16( g.base ) );
  const __m128i vbyte = _mm_add_epi16( vidx, vidx );
  g.shuffle = _mm_or_si128( vbyte, _mm_slli_epi16( _mm_add_epi16( vbyte, _mm_set1_epi16( 1 ) ), 8 ) );
}

// The left samples of one row for a group of columns; r = refSide + y + 1.
static inline __m128i pdpcLeft( const PdpcColumns& g, const Pel* r )
{
  if( g.dense )
  {
    return _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i* ) &r[g.base] ), g.shuffle );
  }

  __m128i v = _mm_cvtsi32_si128( uint16_t( r[g.off[0]] ) );
  switch( g.count )   // lanes from count on stay 0 (weight 0)
  {
  case 8: v = _mm_insert_epi16( v, r[g.off[7]], 7 ); // fall through
  case 7: v = _mm_insert_epi16( v, r[g.off[6]], 6 ); // fall through
  case 6: v = _mm_insert_epi16( v, r[g.off[5]], 5 ); // fall through
  case 5: v = _mm_insert_epi16( v, r[g.off[4]], 4 ); // fall through
  case 4: v = _mm_insert_epi16( v, r[g.off[3]], 3 ); // fall through
  case 3: v = _mm_insert_epi16( v, r[g.off[2]], 2 ); // fall through
  case 2: v = _mm_insert_epi16( v, r[g.off[1]], 1 ); // fall through
  default: break;
  }
  return v;
}

// dst += mulhrs( left - dst, wL << 9 ) for 8 samples
static inline __m128i pdpcBlend( const __m128i& dst, const __m128i& left, const __m128i& weights )
{
  return _mm_add_epi16( dst, _mm_mulhrs_epi16( _mm_sub_epi16( left, dst ), weights ) );
}

template<X86_VEXT vext>
void IntraAnglePDPC_SIMD( Pel* pDsty, const int dstStride, Pel* refSide, const int width, const int height, int scale, int invAngle )
{
  const int n = std::min( 3 << scale, width );

  if( width < 4 )   // not used by the encoder (PDPC needs 4x4 blocks), kept for completeness
  {
    for( int y = 0; y < height; y++, pDsty += dstStride )
    {
      int invAngleSum = 256;
      for( int x = 0; x < n; x++ )
      {
        invAngleSum += invAngle;
        const int wL   = 32 >> ( 2 * x >> scale );
        const Pel left = refSide[y + ( invAngleSum >> 9 ) + 1];
        pDsty[x] = pDsty[x] + ( ( wL * ( left - pDsty[x] ) + 32 ) >> 6 );
      }
    }
    return;
  }

  PdpcColumns g0;
  pdpcInitColumns( g0, invAngle, scale, 0, std::min( n, 8 ) );

  if( n > 8 )   // scale 2 and width >= 16: columns 0 .. 11 in two groups of 8
  {
    PdpcColumns g1;
    pdpcInitColumns( g1, invAngle, scale, 8, n - 8 );

    for( int y = 0; y < height; y++, pDsty += dstStride )
    {
      const Pel* r = refSide + y + 1;
      _mm_storeu_si128( ( __m128i* ) &pDsty[0], pdpcBlend( _mm_loadu_si128( ( const __m128i* ) &pDsty[0] ), pdpcLeft( g0, r ), g0.weights ) );
      _mm_storeu_si128( ( __m128i* ) &pDsty[8], pdpcBlend( _mm_loadu_si128( ( const __m128i* ) &pDsty[8] ), pdpcLeft( g1, r ), g1.weights ) );
    }
  }
  else if( width >= 8 )
  {
    for( int y = 0; y < height; y++, pDsty += dstStride )
    {
      _mm_storeu_si128( ( __m128i* ) pDsty, pdpcBlend( _mm_loadu_si128( ( const __m128i* ) pDsty ), pdpcLeft( g0, refSide + y + 1 ), g0.weights ) );
    }
  }
  else   // width 4
  {
    for( int y = 0; y < height; y++, pDsty += dstStride )
    {
      _vv_storel_epi64( ( __m128i* ) pDsty, pdpcBlend( _vv_loadl_epi64( ( const __m128i* ) pDsty ), pdpcLeft( g0, refSide + y + 1 ), g0.weights ) );
    }
  }
}

#if USE_AVX2
// Horizontal luma modes predicted in the block's own orientation, with no temporary block and no transpose (AVX2):
//   pred[y][x] = sum_k f_k( x ) * refMain[i_x + y + k],  k = 0..3,  i_x = ( a * ( x + 1 + mri ) ) >> 5,
// with the 4-tap filter f of column x's fraction ( a * ( x + 1 + mri ) ) & 31: what IntraPredAngleLumaCore_SIMD computes
// for the transposed block (its row x, column y). The fractions, taps and offsets are per column, constant over the block.
// Per group of 8 columns and pair of rows (y, y + 1):
// - each half (columns 0..3 and 4..7 of the group) loads one 8-sample window at refMain + base + y, base = min( i_first,
//   i_last ) of the half (either sign of a); d_j = i_j - base <= 3 for |a| <= 29, so the taps of row y (window samples
//   d_j .. d_j + 3) and of row y + 1 (d_j + 1 .. d_j + 4) are all in it; the window goes to both 128-bit lanes;
// - two pshufb with per-group masks pair the taps (w[d], w[d + 1]) and (w[d + 2], w[d + 3]) of each column, the high
//   lane's masks 2 bytes further (row y + 1); two pmaddwd with the columns' coefficient pairs, + 32, >> 6;
// - packs( A, B ) gives row y (low lane) and row y + 1 (high lane) in column order; the cubic filter's clip;
// - with PDPC (positive angles, reference line 0) rows y < n = min( 3 << scale, height ) get IntraAnglePDPC's blend in
//   the block's orientation, pred[y][x] += ( wL( y ) * ( refSide[x + off( y ) + 1] - pred[y][x] ) + 32 ) >> 6, whose left
//   samples are consecutive (one load per row and group); the row after an odd n gets weight 0.
// Exact: the same int32 sums as the kernel on the transposed block, + 32 >> 6, packs (which does not saturate for intra
// samples), clip, then IntraAnglePDPC_SIMD's pmulhrsw blend. Reads: at most 3 samples past the last one the C code
// reads in each row pair, inside the extended reference rows (IntraPrediction::ExtRefRows), whose samples are always
// initialised. width >= 8 and height >= 4 (multiples of 8 and 4 by block size), |a| <= 29, not a multiple of 32.

// the two pshufb masks of one half: bytes 2 d_j + ( 0..3 ) and 2 d_j + ( 4..7 ) for column j, 2 more in the high lane
static inline void intraHorMasks( const int d0, const int d1, const int d2, const int d3, __m256i& m01, __m256i& m23 )
{
  const __m128i d2x  = _mm_slli_epi32( _mm_setr_epi32( d0, d1, d2, d3 ), 1 );
  const __m128i rep  = _mm_shuffle_epi8( d2x, _mm_setr_epi8( 0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12 ) );
  const __m128i lo01 = _mm_add_epi8( rep, _mm_setr_epi8( 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3 ) );
  const __m128i lo23 = _mm_add_epi8( lo01, _mm_set1_epi8( 4 ) );
  const __m128i two  = _mm_set1_epi8( 2 );
  m01 = _mm256_set_m128i( _mm_add_epi8( lo01, two ), lo01 );
  m23 = _mm256_set_m128i( _mm_add_epi8( lo23, two ), lo23 );
}

template<bool cubic, bool pdpc>
static void intraPredAngleLumaHorAVX2( Pel* dst, const ptrdiff_t dstStride, const Pel* refMain, const Pel* refSide, const int width, const int height,
                                       const int intraPredAngle, const int multiRefIdx, const int scale, const int invAngle, const ClpRng& clpRng )
{
  const __m256i vrnd = _mm256_set1_epi32( 32 );
  const __m256i vmin = _mm256_set1_epi16( clpRng.min() );
  const __m256i vmax = _mm256_set1_epi16( clpRng.max() );

  // PDPC: rows y < n with their left sample offset (+ 1) and weight << 9; the row after an odd n: weight 0, valid address
  int n = 0, off[( 3 << 2 ) + 1], wgt[( 3 << 2 ) + 1];   // n <= 3 << 2 (scale <= 2), and the row after it
  if( pdpc )
  {
    n = std::min( 3 << scale, height );
    for( int y = 0; y < n; y++ )
    {
      off[y] = ( ( ( y + 1 ) * invAngle + 256 ) >> 9 ) + 1;
      wgt[y] = ( 32 >> ( 2 * y >> scale ) ) << 9;
    }
    off[n] = off[n - 1];
    wgt[n] = 0;
  }

  for( int x0 = 0; x0 < width; x0 += 8 )
  {
    int     i[8];
    int32_t f01[8], f23[8];
    for( int j = 0; j < 8; j++ )
    {
      const int deltaPos = intraPredAngle * ( x0 + j + 1 + multiRefIdx );
      i[j] = deltaPos >> 5;
      intraAngleCoeffPairs( deltaPos & 31, cubic, f01[j], f23[j] );
    }
    const int baseA = std::min( i[0], i[3] ), baseB = std::min( i[4], i[7] );
    CHECKD( std::max( i[0], i[3] ) - baseA > 3 || std::max( i[4], i[7] ) - baseB > 3, "angle too steep for IntraPredAngleLumaHor" );
    __m256i mA01, mA23, mB01, mB23;
    intraHorMasks( i[0] - baseA, i[1] - baseA, i[2] - baseA, i[3] - baseA, mA01, mA23 );
    intraHorMasks( i[4] - baseB, i[5] - baseB, i[6] - baseB, i[7] - baseB, mB01, mB23 );
    const __m256i cA01 = _mm256_broadcastsi128_si256( _mm_setr_epi32( f01[0], f01[1], f01[2], f01[3] ) );
    const __m256i cA23 = _mm256_broadcastsi128_si256( _mm_setr_epi32( f23[0], f23[1], f23[2], f23[3] ) );
    const __m256i cB01 = _mm256_broadcastsi128_si256( _mm_setr_epi32( f01[4], f01[5], f01[6], f01[7] ) );
    const __m256i cB23 = _mm256_broadcastsi128_si256( _mm_setr_epi32( f23[4], f23[5], f23[6], f23[7] ) );
    const Pel* rA = refMain + baseA;
    const Pel* rB = refMain + baseB;
    Pel*       d  = dst + x0;

    for( int y = 0; y < height; y += 2, d += 2 * dstStride )
    {
      const __m256i wA = _mm256_broadcastsi128_si256( _mm_loadu_si128( ( const __m128i* ) ( rA + y ) ) );
      const __m256i wB = _mm256_broadcastsi128_si256( _mm_loadu_si128( ( const __m128i* ) ( rB + y ) ) );
      __m256i a = _mm256_add_epi32( _mm256_madd_epi16( _mm256_shuffle_epi8( wA, mA01 ), cA01 ), _mm256_madd_epi16( _mm256_shuffle_epi8( wA, mA23 ), cA23 ) );
      __m256i b = _mm256_add_epi32( _mm256_madd_epi16( _mm256_shuffle_epi8( wB, mB01 ), cB01 ), _mm256_madd_epi16( _mm256_shuffle_epi8( wB, mB23 ), cB23 ) );
      a = _mm256_srai_epi32( _mm256_add_epi32( a, vrnd ), 6 );
      b = _mm256_srai_epi32( _mm256_add_epi32( b, vrnd ), 6 );
      __m256i v = _mm256_packs_epi32( a, b );
      if( cubic )
      {
        v = _mm256_min_epi16( vmax, _mm256_max_epi16( vmin, v ) );
      }
      if( pdpc && y < n )
      {
        const __m256i l = _mm256_set_m128i( _mm_loadu_si128( ( const __m128i* ) ( refSide + x0 + off[y + 1] ) ),
                                            _mm_loadu_si128( ( const __m128i* ) ( refSide + x0 + off[y] ) ) );
        const __m256i w = _mm256_set_m128i( _mm_set1_epi16( wgt[y + 1] ), _mm_set1_epi16( wgt[y] ) );
        v = _mm256_add_epi16( v, _mm256_mulhrs_epi16( _mm256_sub_epi16( l, v ), w ) );
      }
      _mm_storeu_si128( ( __m128i* ) d, _mm256_castsi256_si128( v ) );
      _mm_storeu_si128( ( __m128i* ) ( d + dstStride ), _mm256_extracti128_si256( v, 1 ) );
    }
  }
  _mm256_zeroupper();
}

template<X86_VEXT vext>
void IntraPredAngleLumaHor_SIMD( Pel* dst, const ptrdiff_t dstStride, const Pel* refMain, const Pel* refSide, const int width, const int height,
                                 const int intraPredAngle, const int multiRefIdx, const int scale, const int invAngle, const bool useCubicFilter,
                                 const bool applyPDPC, const ClpRng& clpRng )
{
  if( useCubicFilter )
  {
    if( applyPDPC ) intraPredAngleLumaHorAVX2<true, true >( dst, dstStride, refMain, refSide, width, height, intraPredAngle, multiRefIdx, scale, invAngle, clpRng );
    else            intraPredAngleLumaHorAVX2<true, false>( dst, dstStride, refMain, refSide, width, height, intraPredAngle, multiRefIdx, scale, invAngle, clpRng );
  }
  else
  {
    if( applyPDPC ) intraPredAngleLumaHorAVX2<false, true >( dst, dstStride, refMain, refSide, width, height, intraPredAngle, multiRefIdx, scale, invAngle, clpRng );
    else            intraPredAngleLumaHorAVX2<false, false>( dst, dstStride, refMain, refSide, width, height, intraPredAngle, multiRefIdx, scale, invAngle, clpRng );
  }
}
#endif   // USE_AVX2

template<X86_VEXT vext>
void IntraHorVerPDPC_SIMD(Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height,int scale,const Pel* refMain, const ClpRng& clpRng)
{
   const Pel topLeft = refMain[0];

   if (width>=16)
   {
#ifdef USE_AVX2
     __m256i v32 = _mm256_set1_epi32(32);
     __m256i vbdmin   = _mm256_set1_epi16( clpRng.min() );
     __m256i vbdmax   = _mm256_set1_epi16( clpRng.max() );

     __m256i wl16;
     if (scale==0)
     {
       wl16 = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,0,0,0,2,8,32);
     }
     else if (scale==1)
     {
       wl16 = _mm256_set_epi16(0,0,0,0,0,0,0,0,0,0,1,2,4,8,16,32);
     }
     else
     {
       wl16 = _mm256_set_epi16(0,0,0,0,1,1,2,2,4,4,8,8,16,16,32,32);
     }
     __m256i xtopLeft = _mm256_set_epi16(topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft);

     for (int y = 0; y<height; y++, pDsty += dstStride)
     {
       // first column
       const Pel left    = refSide[1 + y];
       __m256i xleft= _mm256_set_epi16(left,left,left,left,left,left,left,left,left,left,left,left,left,left,left,left);

       __m256i xdst= _mm256_loadu_si256((__m256i*)&refMain[1]);
       xleft = _mm256_sub_epi16(xleft,xtopLeft);

       __m256i tmplo = _mm256_mullo_epi16(xleft,wl16);
       __m256i tmphi = _mm256_mulhi_epi16(xleft,wl16);
       xleft = _mm256_unpacklo_epi16(tmplo,tmphi);  //low
       tmphi = _mm256_unpackhi_epi16(tmplo,tmphi);  // high

       tmplo = _mm256_add_epi32(xleft,v32);
       tmphi = _mm256_add_epi32(tmphi,v32);
       tmplo = _mm256_srai_epi32(tmplo,6);
       tmphi = _mm256_srai_epi32(tmphi,6);

       tmplo =  _mm256_packs_epi32(tmplo,tmphi);
       tmplo =  _mm256_permute4x64_epi64 ( tmplo, ( 0 << 0 ) + ( 1 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

       xdst = _mm256_adds_epi16(tmplo,xdst);
       xdst = _mm256_min_epi16( vbdmax, _mm256_max_epi16( vbdmin, xdst ) );
       _mm256_storeu_si256( ( __m256i * )(pDsty), xdst );

       // rest memcpy
       for (int x = 16; x < width; x+=16)
       {
         __m256i xdst= _mm256_loadu_si256((__m256i*)&refMain[x+1]);
         _mm256_storeu_si256( ( __m256i * )(&pDsty[x]), xdst );
       }
     }
#else
     for( int y = 0; y < height; y++ )
     {
       memcpy(pDsty,&refMain[1],width*sizeof(Pel));
       const Pel left    = refSide[1 + y];
       for (int x = 0; x < std::min(3 << scale, width); x++)
       {
         const int wL  = 32 >> (2 * x >> scale);
         const Pel val = pDsty[x];
         pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
       }
       pDsty += dstStride;
     }
#endif
   }
   else      //width <= 8
   {
     __m128i vbdmin   = _mm_set1_epi16( clpRng.min() );
     __m128i vbdmax   = _mm_set1_epi16( clpRng.max() );
     __m128i wl16;

     if (scale==0)
     {
       wl16 = _mm_set_epi16(0,0,0,0,0,2,8,32);
     }
     else if (scale==1)
     {
       wl16 = _mm_set_epi16(0,0,1,2,4,8,16,32);
     }
     else
     {
       wl16 = _mm_set_epi16(4,4,8,8,16,16,32,32);
     }

     __m128i v32 = _mm_set1_epi32(32);
     __m128i xtopLeft = _mm_set_epi16(topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft);

     for (int y = 0; y<height; y++, pDsty += dstStride)
     {
       // first column
        const Pel left    = refSide[1 + y];
        __m128i xleft= _mm_set_epi16(left,left,left,left,left,left,left,left);

        __m128i xdst= _mm_loadu_si128((__m128i*)&refMain[1]);
        xleft = _mm_sub_epi16(xleft,xtopLeft);

        __m128i tmplo = _mm_mullo_epi16(xleft,wl16);
        __m128i tmphi = _mm_mulhi_epi16(xleft,wl16);
        xleft = _mm_unpacklo_epi16(tmplo,tmphi);  //low
        tmphi = _mm_unpackhi_epi16(tmplo,tmphi);  // high

        tmplo = _mm_add_epi32(xleft,v32);
        tmphi = _mm_add_epi32(tmphi,v32);
        tmplo = _mm_srai_epi32(tmplo,6);
        tmphi = _mm_srai_epi32(tmphi,6);

        tmplo =  _mm_packs_epi32(tmplo,tmphi);

        xdst = _mm_adds_epi16(tmplo,xdst);
        xdst = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, xdst ) );

        if (width==8)
           _mm_storeu_si128( ( __m128i * )(pDsty), xdst );
         else if (width==4)
           _vv_storel_epi64( ( __m128i * )(pDsty), xdst );
         else
         {
           _mm_storeu_si32( ( __m128i * )(pDsty), xdst );
         }
     }
   }
#if USE_AVX2
  _mm256_zeroupper();
#endif
}


// CCLM luma downsampling for 4:2:0 without vertical collocation (IntraPrediction::loadLMLumaRecPels):
// dst[i] = ( 2 * a[2i] + a[2i + 1] + a[2i - 1] + 2 * b[2i] + b[2i + 1] + b[2i - 1] + 4 ) >> 3 for the luma rows a, b of
// each chroma row; without a left neighbour a[-1] and b[-1] are replaced by a[0] and b[0]. madd adds the pairs of
// samples ( a[2i], a[2i + 1] ) with the weights ( 2, 1 ) and ( a[2i - 1], a[2i] ) with ( 1, 0 ).
template<X86_VEXT vext>
void LMLumaDownsample420_SIMD( const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, const int width, const int height, const bool leftAvailable )
{
  const __m128i w21  = _mm_setr_epi16( 2, 1, 2, 1, 2, 1, 2, 1 );
  const __m128i w10  = _mm_setr_epi16( 1, 0, 1, 0, 1, 0, 1, 0 );
  const __m128i vrnd = _mm_set1_epi32( 4 );

  // the sum of the six taps for four chroma samples, from the luma rows a and b
  auto sum4 = [&]( const Pel* a, const Pel* b )
  {
    __m128i s = _mm_add_epi32( _mm_madd_epi16( _mm_loadu_si128( ( const __m128i* ) a ), w21 ), _mm_madd_epi16( _mm_loadu_si128( ( const __m128i* ) ( a - 1 ) ), w10 ) );
    s         = _mm_add_epi32( s, _mm_madd_epi16( _mm_loadu_si128( ( const __m128i* ) b ), w21 ) );
    s         = _mm_add_epi32( s, _mm_madd_epi16( _mm_loadu_si128( ( const __m128i* ) ( b - 1 ) ), w10 ) );
    return _mm_srai_epi32( _mm_add_epi32( s, vrnd ), 3 );
  };

  for( int y = 0; y < height; y++, src += 2 * srcStride, dst += dstStride )
  {
    const Pel* a = src;
    const Pel* b = src + srcStride;
    int x = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      _mm_storeu_si128( ( __m128i* ) &dst[x], _mm_packs_epi32( sum4( a + 2 * x, b + 2 * x ), sum4( a + 2 * x + 8, b + 2 * x + 8 ) ) );
    }
    for( ; x + 4 <= width; x += 4 )
    {
      const __m128i s = sum4( a + 2 * x, b + 2 * x );
      _vv_storel_epi64( ( __m128i* ) &dst[x], _mm_packs_epi32( s, s ) );
    }
    for( ; x < width; x++ )
    {
      dst[x] = ( 2 * a[2 * x] + a[2 * x + 1] + a[2 * x - 1] + 2 * b[2 * x] + b[2 * x + 1] + b[2 * x - 1] + 4 ) >> 3;
    }
    if( !leftAvailable )
    {
      // the vector code read the samples left of the block, which are replaced by the first column here
      dst[0] = ( 3 * a[0] + a[1] + 3 * b[0] + b[1] + 4 ) >> 3;
    }
  }
}

// [1 2 1] smoothing of the reference samples (IntraPrediction::xFilterReferenceSamples):
// dst[i] = ( src[i - 1] + 2 * src[i] + src[i + 1] + 2 ) >> 2 for 0 < i < length. The sums are computed as unsigned
// 16 bit, which is exact for samples in 0 ... 2^14 - 1.
template<X86_VEXT vext>
void IntraFilterRefSamples_SIMD( const Pel* src, Pel* dst, const int length )
{
  const __m128i vrnd = _mm_set1_epi16( 2 );
  int i = 1;
#if USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i vrnd256 = _mm256_set1_epi16( 2 );
    for( ; i + 16 <= length; i += 16 )
    {
      const __m256i l = _mm256_loadu_si256( ( const __m256i* ) &src[i - 1] );
      const __m256i c = _mm256_loadu_si256( ( const __m256i* ) &src[i] );
      const __m256i r = _mm256_loadu_si256( ( const __m256i* ) &src[i + 1] );
      const __m256i s = _mm256_add_epi16( _mm256_add_epi16( l, r ), _mm256_add_epi16( _mm256_slli_epi16( c, 1 ), vrnd256 ) );
      _mm256_storeu_si256( ( __m256i* ) &dst[i], _mm256_srli_epi16( s, 2 ) );
    }
  }
#endif
  for( ; i + 8 <= length; i += 8 )
  {
    const __m128i l = _mm_loadu_si128( ( const __m128i* ) &src[i - 1] );
    const __m128i c = _mm_loadu_si128( ( const __m128i* ) &src[i] );
    const __m128i r = _mm_loadu_si128( ( const __m128i* ) &src[i + 1] );
    const __m128i s = _mm_add_epi16( _mm_add_epi16( l, r ), _mm_add_epi16( _mm_slli_epi16( c, 1 ), vrnd ) );
    _mm_storeu_si128( ( __m128i* ) &dst[i], _mm_srli_epi16( s, 2 ) );
  }
  if( i + 4 <= length )
  {
    const __m128i l = _vv_loadl_epi64( ( const __m128i* ) &src[i - 1] );
    const __m128i c = _vv_loadl_epi64( ( const __m128i* ) &src[i] );
    const __m128i r = _vv_loadl_epi64( ( const __m128i* ) &src[i + 1] );
    const __m128i s = _mm_add_epi16( _mm_add_epi16( l, r ), _mm_add_epi16( _mm_slli_epi16( c, 1 ), vrnd ) );
    _vv_storel_epi64( ( __m128i* ) &dst[i], _mm_srli_epi16( s, 2 ) );
    i += 4;
  }
  for( ; i < length; i++ )
  {
    dst[i] = ( src[i - 1] + 2 * src[i] + src[i + 1] + 2 ) >> 2;
  }
}

template<X86_VEXT vext>
void IntraPrediction::_initIntraPredictionX86()
{
  IntraPredAngleLuma    = IntraPredAngleLumaCore_SIMD<vext>;
  IntraPredAngleChroma  = IntraPredAngleChroma_SIMD<vext>;
  IntraAnglePDPC        = IntraAnglePDPC_SIMD<vext>;
  IntraHorVerPDPC       = IntraHorVerPDPC_SIMD<vext>;
  IntraPredSampleFilter = IntraPredSampleFilter_SIMD<vext>;
  xPredIntraPlanar      = xPredIntraPlanar_SIMD<vext>;
  LMLumaDownsample420   = LMLumaDownsample420_SIMD<vext>;
  IntraFilterRefSamples = IntraFilterRefSamples_SIMD<vext>;
#if USE_AVX2 && REAL_TARGET_X86   // not through SIMDE: there the other targets keep the temporary block and transpose
  if( vext >= AVX2 )
  {
    IntraPredAngleLumaHor = IntraPredAngleLumaHor_SIMD<vext>;
  }
#endif
}
template void IntraPrediction::_initIntraPredictionX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86
#endif
//! \}

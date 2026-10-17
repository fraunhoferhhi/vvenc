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
/** \file     TrafoX86.h
    \brief    SIMD averaging.
*/

//! \ingroup CommonLib
//! \{

#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#if ENABLE_SIMD_TRAFO
#ifdef TARGET_SIMD_X86

namespace vvenc {

template<X86_VEXT vext, unsigned trSize>
void fastInv_SSE( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned lines, unsigned reducedLines, unsigned rows )
{
  unsigned maxLoopL = std::min<int>( reducedLines, 4 );

#if USE_AVX2
  if( trSize >= 8 && vext >= AVX2 )
  {
    if( ( trSize & 15 ) == 0 )
    {
      static constexpr unsigned trLoops = trSize >> 4 ? trSize >> 4 : 1;

      for( int k = 0; k < rows; k += 2 )
      {
              TCoeff* dstPtr =  dst;

        const TCoeff* srcPtr0 = &src[ k      * lines];
        const TCoeff* srcPtr1 = &src[(k + 1) * lines];

        __m256i vsrc1v[trLoops][2];
        
        const TMatrixCoeff*  itPtr0 = &it[ k      * trSize];
        const TMatrixCoeff*  itPtr1 = &it[(k + 1) * trSize];

        for( int col = 0; col < trLoops; col++, itPtr0 += 16, itPtr1 += 16 )
        {
#if defined( _MSC_VER ) && _MSC_VER > 1900
          __m256i vit16_0 = _mm256_permute4x64_epi64( _mm256_stream_load_si256( ( const __m256i * ) itPtr0 ), ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
          __m256i vit16_1 = _mm256_permute4x64_epi64( _mm256_stream_load_si256( ( const __m256i * ) itPtr1 ), ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
#else
          __m256i vit16_0 = _mm256_permute4x64_epi64( _mm256_stream_load_si256( (       __m256i * ) itPtr0 ), ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
          __m256i vit16_1 = _mm256_permute4x64_epi64( _mm256_stream_load_si256( (       __m256i * ) itPtr1 ), ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
#endif

          vsrc1v[col][0] = _mm256_unpacklo_epi16( vit16_0, vit16_1 );
          vsrc1v[col][1] = _mm256_unpackhi_epi16( vit16_0, vit16_1 );
        }

        for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
        {
          __m128i xscale = maxLoopL == 4
                         ? _mm_packs_epi32( _mm_loadu_si128( ( const __m128i* )srcPtr0 ), _mm_loadu_si128( ( const __m128i* )srcPtr1 ) )
                         : _mm_packs_epi32( _vv_loadl_epi64( ( const __m128i* )srcPtr0 ), _vv_loadl_epi64( ( const __m128i* )srcPtr1 ) );
          xscale = _mm_shuffle_epi8( xscale, _mm_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 ) );

          if( _mm_test_all_zeros( xscale, xscale ) ) { dstPtr += ( trSize * maxLoopL ); continue; }

          for( int l = 0; l < maxLoopL; l++ )
          {
            __m256i
            vscale = _mm256_broadcastd_epi32( xscale );
            xscale = _mm_bsrli_si128( xscale, 4 );

            for( int col = 0; col < trLoops; col++, dstPtr += 16 )
            {
              __m256i vsrc0 = _mm256_load_si256       ( ( const __m256i * ) dstPtr );

              __m256i
              vsrc1 = vsrc1v[col][0];
              vsrc1 = _mm256_madd_epi16    ( vsrc1, vscale );
              vsrc0 = _mm256_add_epi32     ( vsrc0, vsrc1 );

              _mm256_store_si256           ( ( __m256i * ) dstPtr, vsrc0 );
            
              vsrc0 = _mm256_load_si256    ( ( const __m256i * ) &dstPtr[8] );

              vsrc1 = vsrc1v[col][1];
              vsrc1 = _mm256_madd_epi16    ( vsrc1, vscale );
              vsrc0 = _mm256_add_epi32     ( vsrc0, vsrc1 );

              _mm256_store_si256           ( ( __m256i * ) &dstPtr[8], vsrc0 );
            }
          }
        }
      }
    }
    else
    {
      for( int k = 0; k < rows; k += 2 )
      {
              TCoeff* dstPtr  =  dst;

        const TCoeff* srcPtr0 = &src[ k      * lines];
        const TCoeff* srcPtr1 = &src[(k + 1) * lines];

        const TMatrixCoeff*  itPtr0 = &it[  k      * trSize];
        const TMatrixCoeff*  itPtr1 = &it[( k + 1 ) * trSize];

        __m256i vit;

        {
#if defined( _MSC_VER ) && _MSC_VER > 1900
          __m256i vsrc1 = _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_stream_load_si128( ( const __m128i * ) itPtr0 ) ), ( 0 << 0 ) + ( 1 << 4 ) );
#else
          __m256i vsrc1 = _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_stream_load_si128( ( __m128i * ) itPtr0 ) ), ( 0 << 0 ) + ( 1 << 4 ) );
#endif
#if defined( _MSC_VER ) && _MSC_VER > 1900
          __m256i vsrc2 = _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_stream_load_si128( ( const __m128i * ) itPtr1 ) ), ( 0 << 0 ) + ( 1 << 4 ) );
#else
          __m256i vsrc2 = _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_stream_load_si128( ( __m128i * ) itPtr1 ) ), ( 0 << 0 ) + ( 1 << 4 ) );
#endif

          vit = _mm256_unpacklo_epi16( vsrc1, vsrc2 );
        }
        
        for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
        {
          __m128i xscale = maxLoopL == 4
                         ? _mm_packs_epi32( _mm_loadu_si128( ( const __m128i* )srcPtr0 ), _mm_loadu_si128( ( const __m128i* )srcPtr1 ) )
                         : _mm_packs_epi32( _vv_loadl_epi64( ( const __m128i* )srcPtr0 ), _vv_loadl_epi64( ( const __m128i* )srcPtr1 ) );
          xscale = _mm_shuffle_epi8( xscale, _mm_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 ) );

          if( _mm_test_all_zeros( xscale, xscale ) ) { dstPtr += ( trSize * maxLoopL ); continue; }

          for( int l = 0; l < maxLoopL; l++ )
          {
            __m256i
            vscale = _mm256_broadcastd_epi32( xscale );
            xscale = _mm_bsrli_si128( xscale, 4 );

            for( int col = 0; col < trSize; col += 8, dstPtr += 8, itPtr0 += 8, itPtr1 += 8 )
            {
              __m256i
              vsrc0 = _mm256_load_si256    ( ( const __m256i * ) dstPtr );
              __m256i
              vsrc1 = _mm256_madd_epi16    ( vit, vscale );
              vsrc0 = _mm256_add_epi32     ( vsrc0, vsrc1 );

              _mm256_store_si256           ( ( __m256i * ) dstPtr, vsrc0 );
            }
          }
        }
      }
    }
  }
#else
  if( trSize >= 8 )
  {
    for( int k = 0; k < rows; k += 2 )
    {
            TCoeff* dstPtr  =  dst;

      const TCoeff* srcPtr0 = &src[ k      * lines];
      const TCoeff* srcPtr1 = &src[(k + 1) * lines];
        
      for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
      {
        __m128i xscale = maxLoopL == 4
                        ? _mm_packs_epi32( _mm_loadu_si128( ( const __m128i* )srcPtr0 ), _mm_loadu_si128( ( const __m128i* )srcPtr1 ) )
                        : _mm_packs_epi32( _vv_loadl_epi64( ( const __m128i* )srcPtr0 ), _vv_loadl_epi64( ( const __m128i* )srcPtr1 ) );
        xscale = _mm_shuffle_epi8( xscale, _mm_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 ) );

        if( _mm_test_all_zeros( xscale, xscale ) ) { dstPtr += ( trSize * maxLoopL ); continue; }

        for( int l = 0; l < maxLoopL; l++ )
        {
          const TMatrixCoeff*  itPtr0 = &it[k      * trSize];
          const TMatrixCoeff*  itPtr1 = &it[( k + 1 ) * trSize];

          __m128i
          vscale = _mm_set1_epi32( _mm_cvtsi128_si32( xscale ) );
          xscale = _mm_bsrli_si128( xscale, 4 );

          for( int col = 0; col < trSize; col += 8, dstPtr += 8, itPtr0 += 8, itPtr1 += 8 )
          {
            __m128i vsrc0   = _mm_load_si128       ( ( const __m128i * ) dstPtr );
#if defined( _MSC_VER ) && _MSC_VER > 1900
            __m128i vit16_0 = _mm_stream_load_si128( ( const __m128i * ) itPtr0 );
            __m128i vit16_1 = _mm_stream_load_si128( ( const __m128i * ) itPtr1 );
#else
            __m128i vit16_0 = _mm_stream_load_si128( (       __m128i * ) itPtr0 );
            __m128i vit16_1 = _mm_stream_load_si128( (       __m128i * ) itPtr1 );
#endif

            __m128i vsrc1 = _mm_unpacklo_epi16( vit16_0, vit16_1 );

            vsrc1 = _mm_madd_epi16 ( vsrc1, vscale );
            vsrc0 = _mm_add_epi32  ( vsrc0, vsrc1 );

            _mm_store_si128        ( ( __m128i * ) dstPtr, vsrc0 );
          
            vsrc0 = _mm_load_si128 ( ( const __m128i * ) &dstPtr[4] );
          
            vsrc1 = _mm_unpackhi_epi16( vit16_0, vit16_1 );

            vsrc1 = _mm_madd_epi16 ( vsrc1, vscale );
            vsrc0 = _mm_add_epi32  ( vsrc0, vsrc1 );
          
            _mm_store_si128        ( ( __m128i * ) &dstPtr[4], vsrc0 );
          }
        }
      }
    }
  }
#endif
  else if( trSize >= 4 )
  {
    CHECKD( trSize != 4, "trSize needs to be '4'!" );

    for( int k = 0; k < rows; k += 2 )
    {
            TCoeff* dstPtr  =  dst;

      const TCoeff* srcPtr0 = &src[ k      * lines];
      const TCoeff* srcPtr1 = &src[(k + 1) * lines];

      const TMatrixCoeff*  itPtr0 = &it[  k       * trSize];
      const TMatrixCoeff*  itPtr1 = &it[( k + 1 ) * trSize];

      __m128i vit = _mm_unpacklo_epi16( _vv_loadl_epi64( ( const __m128i * ) itPtr0 ), _vv_loadl_epi64( ( const __m128i * ) itPtr1 ) );
 
      for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
      {
        __m128i xscale = maxLoopL == 4
                        ? _mm_packs_epi32( _mm_loadu_si128( ( const __m128i* )srcPtr0 ), _mm_loadu_si128( ( const __m128i* )srcPtr1 ) )
                        : _mm_packs_epi32( _vv_loadl_epi64( ( const __m128i* )srcPtr0 ), _vv_loadl_epi64( ( const __m128i* )srcPtr1 ) );
        xscale = _mm_shuffle_epi8( xscale, _mm_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 ) );

        if( _mm_test_all_zeros( xscale, xscale ) ) { dstPtr += ( trSize * maxLoopL ); continue; }

        for( int l = 0; l < maxLoopL; l++ )
        {
          __m128i
          vscale = _mm_set1_epi32( _mm_cvtsi128_si32( xscale ) );
          xscale = _mm_bsrli_si128( xscale, 4 );

          for( int col = 0; col < trSize; col += 4, dstPtr += 4 )
          {
            __m128i
            vsrc0 = _mm_load_si128 ( ( const __m128i * ) dstPtr );
            __m128i 
            vsrc1 = _mm_madd_epi16 ( vit, vscale );
            vsrc0 = _mm_add_epi32  ( vsrc0, vsrc1 );

            _mm_store_si128        ( ( __m128i * ) dstPtr, vsrc0 );
          }
        }
      }
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext, int trSize>
void fastFwd_SSE( const TMatrixCoeff* tc, const TCoeff* src, TCoeff* dst, unsigned line, unsigned reducedLine, unsigned cutoff, int shift )
{
  const int rnd_factor = 1 << ( shift - 1 );
  
  //for( int i = 0; i < reducedLine; i++ )
  //{
  //        TCoeff*       dstPtr = dst;
  //  const TMatrixCoeff* iT     = tc;
  //
  //  for( int j = 0; j < cutoff; j++ )
  //  {
  //    int sum = 0;
  //
  //    for( int k = 0; k < trSize; k++ )
  //    {
  //      // dst[j * line + i] += src[i * trSize + k] * t[j * trSize + k]
  //      sum += src[k] * iT[k];
  //    }
  //
  //    dstPtr[i] = ( sum + rnd_factor ) >> shift;
  //    dstPtr   += line;
  //    iT       += trSize;
  //  }
  //
  //  src += trSize;
  //}

  if( trSize >= 8 )
  {
#if USE_AVX2
    if( vext >= AVX2 && ( trSize & 15 ) == 0 )
    {
#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ ) && !defined( __clang__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
      // vsrcarr[2] and vsrcarr[3] might be unitialized for nlx4==0, but in that case they will not be used, so discard the warning!
#endif
      static constexpr unsigned trLoops = trSize >> 4 ? trSize >> 4 : 1;

      // is number of lines a multiplier of 4
      const int nlx4 = reducedLine == 2 ? 0 : 1;

      for( int i = 0; i < reducedLine; i += ( 2 << nlx4 ) )
      {
              TCoeff*       dstPtr = dst + i;
        const TMatrixCoeff* itPtr  = tc;
        
        __m256i vsrcarr[trLoops][4];
          
        for( int k = 0; k < trLoops; k++ )
        {
          __m256i vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[(k << 4) + 0] );
          __m256i vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[(k << 4) + 8] );
          __m256i vsrc  = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[k][0] = vsrc;
          
          vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[(k << 4) + 0 + trSize] );
          vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[(k << 4) + 8 + trSize] );
          vsrc  = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc  = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[k][1] = vsrc;

          if( !nlx4 ) continue;

          vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[(k << 4) + 0 + 2 * trSize] );
          vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[(k << 4) + 8 + 2 * trSize] );
          vsrc = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[k][2] = vsrc;

          vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[(k << 4) + 0 + 3 * trSize] );
          vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[(k << 4) + 8 + 3 * trSize] );
          vsrc = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[k][3] = vsrc;
        }

        for( int j = 0; j < cutoff; j += 4 )
        {
          __m256i vsum00 = _mm256_setzero_si256();
          __m256i vsum02 = _mm256_setzero_si256();

          __m256i vsum10 = _mm256_setzero_si256();
          __m256i vsum12 = _mm256_setzero_si256();
          
          __m256i vsum20 = _mm256_setzero_si256();
          __m256i vsum22 = _mm256_setzero_si256();

          __m256i vsum30 = _mm256_setzero_si256();
          __m256i vsum32 = _mm256_setzero_si256();

          for( int k = 0; k < trLoops; k++ )
          {
            // dst[j * line + i] += src[i * trSize + k] * t[j * trSize + k]

#if 0
#if defined( _MSC_VER ) && _MSC_VER > 1900
            __m256i vit0  = _mm256_stream_load_si256( ( const __m256i* ) &itPtr[k + 0 * trSize] );
            __m256i vit1  = _mm256_stream_load_si256( ( const __m256i* ) &itPtr[k + 1 * trSize] );
            __m256i vit2  = _mm256_stream_load_si256( ( const __m256i* ) &itPtr[k + 2 * trSize] );
            __m256i vit3  = _mm256_stream_load_si256( ( const __m256i* ) &itPtr[k + 3 * trSize] );
#else
            __m256i vit0  = _mm256_stream_load_si256( (       __m256i* ) &itPtr[k + 0 * trSize] );
            __m256i vit1  = _mm256_stream_load_si256( (       __m256i* ) &itPtr[k + 1 * trSize] );
            __m256i vit2  = _mm256_stream_load_si256( (       __m256i* ) &itPtr[k + 2 * trSize] );
            __m256i vit3  = _mm256_stream_load_si256( (       __m256i* ) &itPtr[k + 3 * trSize] );
#endif
#else
            __m256i vit0  = _mm256_load_si256( ( const __m256i* ) &itPtr[(k << 4) + 0 * trSize] );
            __m256i vit1  = _mm256_load_si256( ( const __m256i* ) &itPtr[(k << 4) + 1 * trSize] );
            __m256i vit2  = _mm256_load_si256( ( const __m256i* ) &itPtr[(k << 4) + 2 * trSize] );
            __m256i vit3  = _mm256_load_si256( ( const __m256i* ) &itPtr[(k << 4) + 3 * trSize] );
#endif

            __m256i
            vsrc  = vsrcarr[k][0];

            vsum00 = _mm256_add_epi32( vsum00, _mm256_hadd_epi32( _mm256_madd_epi16( vit0, vsrc ), _mm256_madd_epi16( vit1, vsrc ) ) );
            vsum02 = _mm256_add_epi32( vsum02, _mm256_hadd_epi32( _mm256_madd_epi16( vit2, vsrc ), _mm256_madd_epi16( vit3, vsrc ) ) );
     
            vsrc  = vsrcarr[k][1];

            vsum10 = _mm256_add_epi32( vsum10, _mm256_hadd_epi32( _mm256_madd_epi16( vit0, vsrc ), _mm256_madd_epi16( vit1, vsrc ) ) );
            vsum12 = _mm256_add_epi32( vsum12, _mm256_hadd_epi32( _mm256_madd_epi16( vit2, vsrc ), _mm256_madd_epi16( vit3, vsrc ) ) );

            // skip branching
            //if( !nlx4 ) continue;
     
            vsrc  = vsrcarr[k][2];

            vsum20 = _mm256_add_epi32( vsum20, _mm256_hadd_epi32( _mm256_madd_epi16( vit0, vsrc ), _mm256_madd_epi16( vit1, vsrc ) ) );
            vsum22 = _mm256_add_epi32( vsum22, _mm256_hadd_epi32( _mm256_madd_epi16( vit2, vsrc ), _mm256_madd_epi16( vit3, vsrc ) ) );
            
            vsrc  = vsrcarr[k][3];

            vsum30 = _mm256_add_epi32( vsum30, _mm256_hadd_epi32( _mm256_madd_epi16( vit0, vsrc ), _mm256_madd_epi16( vit1, vsrc ) ) );
            vsum32 = _mm256_add_epi32( vsum32, _mm256_hadd_epi32( _mm256_madd_epi16( vit2, vsrc ), _mm256_madd_epi16( vit3, vsrc ) ) );
          }

          vsum00 = _mm256_hadd_epi32( vsum00, vsum02 );

          __m128i xsum00 = _mm_add_epi32( _mm256_castsi256_si128( vsum00 ), _mm256_extracti128_si256( vsum00, 1 ) );
          xsum00 = _mm_add_epi32 ( xsum00, _mm_set1_epi32( rnd_factor ) );
          xsum00 = _mm_srai_epi32( xsum00, shift );

          vsum10 = _mm256_hadd_epi32( vsum10, vsum12 );
          
          __m128i xsum10 = _mm_add_epi32( _mm256_castsi256_si128( vsum10 ), _mm256_extracti128_si256( vsum10, 1 ) );
          xsum10 = _mm_add_epi32 ( xsum10, _mm_set1_epi32( rnd_factor ) );
          xsum10 = _mm_srai_epi32( xsum10, shift );

          if( nlx4 )
          {
            vsum20 = _mm256_hadd_epi32( vsum20, vsum22 );

            __m128i xsum20 = _mm_add_epi32( _mm256_castsi256_si128( vsum20 ), _mm256_extracti128_si256( vsum20, 1 ) );
            xsum20 = _mm_add_epi32( xsum20, _mm_set1_epi32( rnd_factor ) );
            xsum20 = _mm_srai_epi32( xsum20, shift );

            vsum30 = _mm256_hadd_epi32( vsum30, vsum32 );

            __m128i xsum30 = _mm_add_epi32( _mm256_castsi256_si128( vsum30 ), _mm256_extracti128_si256( vsum30, 1 ) );
            xsum30 = _mm_add_epi32( xsum30, _mm_set1_epi32( rnd_factor ) );
            xsum30 = _mm_srai_epi32( xsum30, shift );

            __m128i xtmp0 = _mm_unpacklo_epi32( xsum00, xsum10 );
            __m128i xtmp1 = _mm_unpacklo_epi32( xsum20, xsum30 );

            _mm_store_si128( ( __m128i* ) dstPtr, _mm_unpacklo_epi64( xtmp0, xtmp1 ) ); dstPtr += line;
            _mm_store_si128( ( __m128i* ) dstPtr, _mm_unpackhi_epi64( xtmp0, xtmp1 ) ); dstPtr += line;

            xtmp0 = _mm_unpackhi_epi32( xsum00, xsum10 );
            xtmp1 = _mm_unpackhi_epi32( xsum20, xsum30 );

            _mm_store_si128( ( __m128i* ) dstPtr, _mm_unpacklo_epi64( xtmp0, xtmp1 ) ); dstPtr += line;
            _mm_store_si128( ( __m128i* ) dstPtr, _mm_unpackhi_epi64( xtmp0, xtmp1 ) ); dstPtr += line;
          }
          else
          {
            __m128i xtmp = _mm_unpacklo_epi32( xsum00, xsum10 );

            _vv_storel_epi64( ( __m128i* ) dstPtr,                     xtmp );         dstPtr += line;
            _vv_storel_epi64( ( __m128i* ) dstPtr, _mm_unpackhi_epi64( xtmp, xtmp ) ); dstPtr += line;

            xtmp = _mm_unpackhi_epi32( xsum00, xsum10 );

            _vv_storel_epi64( ( __m128i* ) dstPtr,                     xtmp );         dstPtr += line;
            _vv_storel_epi64( ( __m128i* ) dstPtr, _mm_unpackhi_epi64( xtmp, xtmp ) ); dstPtr += line;
          }

          itPtr  += ( trSize << 2 );
        }

        src += ( trSize << ( 1 + nlx4 ) );
      }
#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ ) && !defined( __clang__ )
#pragma GCC diagnostic pop
#endif
    }
    else
#endif
    {
      static constexpr unsigned trLoops = trSize >> 3 ? trSize >> 3 : 1;

      for( int i = 0; i < reducedLine; i += 2 )
      {
              TCoeff*       dstPtr = dst + i;
        const TMatrixCoeff* itPtr  = tc;
     
        __m128i vsrcarr[trLoops][2];
          
        for( int k = 0; k < trLoops; k++ )
        {
          __m128i vsrc0 = _mm_load_si128( ( const __m128i* ) &src[(k << 3) + 0] );
          __m128i vsrc1 = _mm_load_si128( ( const __m128i* ) &src[(k << 3) + 4] );
          __m128i vsrc  = _mm_packs_epi32( vsrc0, vsrc1 );

          vsrcarr[k][0] = vsrc;
          
          vsrc0 = _mm_load_si128( ( const __m128i* ) &src[(k << 3) + 0 + trSize] );
          vsrc1 = _mm_load_si128( ( const __m128i* ) &src[(k << 3) + 4 + trSize] );
          vsrc  = _mm_packs_epi32( vsrc0, vsrc1 );

          vsrcarr[k][1] = vsrc;
        }

        for( int j = 0; j < cutoff; j += 4 )
        {
          __m128i vsum00 = _mm_setzero_si128();
          __m128i vsum02 = _mm_setzero_si128();
        
          __m128i vsum10 = _mm_setzero_si128();
          __m128i vsum12 = _mm_setzero_si128();

          for( int k = 0; k < trLoops; k++ )
          {
            // dst[j * line + i] += src[i * trSize + k] * t[j * trSize + k]

  #if 0
  #if defined( _MSC_VER ) && _MSC_VER > 1900
            __m128i vit0  = _mm_stream_load_si128( ( const __m128i* ) &itPtr[k + 0 * trSize] );
            __m128i vit1  = _mm_stream_load_si128( ( const __m128i* ) &itPtr[k + 1 * trSize] );
            __m128i vit2  = _mm_stream_load_si128( ( const __m128i* ) &itPtr[k + 2 * trSize] );
            __m128i vit3  = _mm_stream_load_si128( ( const __m128i* ) &itPtr[k + 3 * trSize] );
  #else
            __m128i vit0  = _mm_stream_load_si128( (       __m128i* ) &itPtr[k + 0 * trSize] );
            __m128i vit1  = _mm_stream_load_si128( (       __m128i* ) &itPtr[k + 1 * trSize] );
            __m128i vit2  = _mm_stream_load_si128( (       __m128i* ) &itPtr[k + 2 * trSize] );
            __m128i vit3  = _mm_stream_load_si128( (       __m128i* ) &itPtr[k + 3 * trSize] );
  #endif
  #else
            __m128i vit0  = _mm_load_si128( ( const __m128i* ) &itPtr[(k << 3) + 0 * trSize] );
            __m128i vit1  = _mm_load_si128( ( const __m128i* ) &itPtr[(k << 3) + 1 * trSize] );
            __m128i vit2  = _mm_load_si128( ( const __m128i* ) &itPtr[(k << 3) + 2 * trSize] );
            __m128i vit3  = _mm_load_si128( ( const __m128i* ) &itPtr[(k << 3) + 3 * trSize] );
  #endif
            
            // fist source line
            __m128i vsrc  = vsrcarr[k][0];

            vsum00 = _mm_add_epi32( vsum00, _mm_hadd_epi32( _mm_madd_epi16( vit0, vsrc ), _mm_madd_epi16( vit1, vsrc ) ) );
            vsum02 = _mm_add_epi32( vsum02, _mm_hadd_epi32( _mm_madd_epi16( vit2, vsrc ), _mm_madd_epi16( vit3, vsrc ) ) );
          
            // second source line
            vsrc   = vsrcarr[k][1];

            vsum10 = _mm_add_epi32( vsum10, _mm_hadd_epi32( _mm_madd_epi16( vit0, vsrc ), _mm_madd_epi16( vit1, vsrc ) ) );
            vsum12 = _mm_add_epi32( vsum12, _mm_hadd_epi32( _mm_madd_epi16( vit2, vsrc ), _mm_madd_epi16( vit3, vsrc ) ) );
          }

          vsum00 = _mm_hadd_epi32( vsum00, vsum02 );
          vsum00 = _mm_add_epi32 ( vsum00, _mm_set1_epi32( rnd_factor ) );
          vsum00 = _mm_srai_epi32( vsum00, shift );

          vsum10 = _mm_hadd_epi32( vsum10, vsum12 );
          vsum10 = _mm_add_epi32 ( vsum10, _mm_set1_epi32( rnd_factor ) );
          vsum10 = _mm_srai_epi32( vsum10, shift );

          __m128i xtmp = _mm_unpacklo_epi32( vsum00, vsum10 );
          _vv_storel_epi64( ( __m128i* ) dstPtr, xtmp ); dstPtr += line;

          xtmp = _mm_shuffle_epi32( xtmp, ( 2 << 0 ) + ( 3 << 2 ) );
          _vv_storel_epi64( ( __m128i* ) dstPtr, xtmp ); dstPtr += line;
          
          xtmp = _mm_unpackhi_epi32( vsum00, vsum10 );
          _vv_storel_epi64( ( __m128i* ) dstPtr, xtmp ); dstPtr += line;

          xtmp = _mm_shuffle_epi32( xtmp, ( 2 << 0 ) + ( 3 << 2 ) );
          _vv_storel_epi64( ( __m128i* ) dstPtr, xtmp ); dstPtr += line;

          itPtr  += ( trSize << 2 );
        }

        src += ( trSize << 1 );
      }
    }
  }
  else
  {
    __m128i vzero = _mm_setzero_si128();

    for( int i = 0; i < reducedLine; i++ )
    {
            TCoeff*       dstPtr = dst;
      const TMatrixCoeff* itPtr  = tc;

      for( int j = 0; j < cutoff; j++ )
      {
        __m128i vit   = _vv_loadl_epi64( ( const __m128i* ) itPtr );
        __m128i vsrc0 = _mm_load_si128 ( ( const __m128i* ) src );

        __m128i vsrc = _mm_packs_epi32( vsrc0, vzero );
        __m128i vsum = _mm_madd_epi16 ( vit, vsrc );

        dstPtr[i] = ( _mm_extract_epi32( vsum, 0 ) + _mm_extract_epi32( vsum, 1 ) + rnd_factor ) >> shift;

        dstPtr += line;
        itPtr  += trSize;
      }

      src += trSize;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext, int W >
void roundClip_SSE( TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift )
{
#if USE_AVX2
  if( W >= 8 && vext >= AVX2 )
  {
    __m256i vmin = _mm256_set1_epi32( outputMin );
    __m256i vmax = _mm256_set1_epi32( outputMax );
    __m256i vrnd = _mm256_set1_epi32( round );

    while( height-- )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i
        vdst = _mm256_load_si256( ( __m256i * ) &dst[col] );
        vdst = _mm256_add_epi32 ( vdst, vrnd );
        vdst = _mm256_srai_epi32( vdst, shift );
        vdst = _mm256_max_epi32 ( vdst, vmin );
        vdst = _mm256_min_epi32 ( vdst, vmax );
        _mm256_store_si256      ( ( __m256i * ) &dst[col], vdst );
      }

      dst += stride;
    }
  }
  else
#endif
  if( W >= 4 )
  {
    __m128i vmin = _mm_set1_epi32( outputMin );
    __m128i vmax = _mm_set1_epi32( outputMax );
    __m128i vrnd = _mm_set1_epi32( round );

    while( height-- )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i
        vdst = _mm_load_si128 ( ( __m128i * ) &dst[col] );
        vdst = _mm_add_epi32  ( vdst, vrnd );
        vdst = _mm_srai_epi32 ( vdst, shift );
        vdst = _mm_max_epi32  ( vdst, vmin );
        vdst = _mm_min_epi32  ( vdst, vmax );
        _mm_store_si128       ( ( __m128i * ) &dst[col], vdst );
      }

      dst += stride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext, int W >
void cpyResi_SSE( const TCoeff* src, Pel* dst, ptrdiff_t stride, unsigned width, unsigned height )
{
#if USE_AVX2
  if( W >= 8 && vext >= AVX2 )
  {
    while( height-- )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i
        vsrc = _mm256_load_si256        ( ( const __m256i * ) &src[col] );
        __m128i
        vdst = _mm_packs_epi32          ( _mm256_castsi256_si128( vsrc ), _mm256_extracti128_si256( vsrc, 1 ) );
        _mm_storeu_si128                ( ( __m128i * ) &dst[col], vdst );
      }

      src += width;
      dst += stride;
    }
  }
  else
#endif
  if( W >= 4 )
  {
    __m128i vzero = _mm_setzero_si128();
    __m128i vdst;

    while( height-- )
    {
      for( int col = 0; col < width; col += 4 )
      {
        vdst = _mm_load_si128 ( ( const __m128i * ) &src[col] );
        vdst = _mm_packs_epi32( vdst, vzero );
        _vv_storel_epi64      ( ( __m128i * ) &dst[col], vdst );
      }

      src += width;
      dst += stride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext, int W >
void cpyCoeff_SSE( const Pel* src, ptrdiff_t stride, TCoeff* dst, unsigned width, unsigned height )
{
#if USE_AVX2
  if( W >= 8 && vext >= AVX2 )
  {
    while( height-- )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i vtmp = _mm256_cvtepi16_epi32( _mm_loadu_si128( ( const __m128i * ) &src[col] ) );
        _mm256_store_si256( ( __m256i * ) &dst[col], vtmp );
      }

      src += stride;
      dst += width;
    }
  }
  else
#endif
  if( W >= 4 )
  {
    while( height-- )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vtmp = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( const __m128i * ) &src[col] ) );
        _mm_store_si128( ( __m128i * ) &dst[col], vtmp );
      }

      src += stride;
      dst += width;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void simdInvLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  CHECK( index > 2 || ( zeroOutSize != 8 && zeroOutSize != 16 ), "Wrong parameters" );

  static constexpr int maxLog2TrDynamicRange = 15;
  const TCoeff    outputMinimum = -( 1 << maxLog2TrDynamicRange );
  const TCoeff    outputMaximum =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const int8_t*   trMat         = ( size > 4 ) ? g_lfnstInv8x8[mode][index][0] : g_lfnstInv4x4[mode][index][0];
  const int       trSize        = ( size > 4 ) ? 48 : 16;
  int*            out           = dst;

  const __m128i vzero = _mm_setzero_si128();
  const __m128i vmin  = _mm_set1_epi32( outputMinimum );
  const __m128i vmax  = _mm_set1_epi32( outputMaximum );

  for( int j = 0; j < trSize; j += 4, out += 4 )
  {
    __m128i       vsum[4];

    for( int k = 0; k < 4; k++, trMat += 16 )
    {
      const int8_t* trMatTmp = trMat;
      int* srcPtr = src;

      __m128i vsrc;
      __m128i vtr;
      __m128i vtmp;
      __m128i vcur = vzero;

      for( int i = 0; i < zeroOutSize; i += 8, srcPtr += 8, trMatTmp += 8 )
      {
        vsrc = _mm_loadu_si128( ( const __m128i* ) srcPtr );
        vtr  = _vv_loadl_epi64( ( const __m128i* ) trMatTmp );
        vtr  = _mm_cvtepi8_epi16( vtr );
        vtmp = _mm_cvtepi16_epi32( vtr );

        vtmp = _mm_mullo_epi32( vsrc, vtmp );
        vcur = _mm_add_epi32( vtmp, vcur );

        vsrc = _mm_loadu_si128( ( const __m128i* ) &srcPtr[4] );
        vtmp = _mm_cvtepi16_epi32( _mm_unpackhi_epi64( vtr, vzero ) );
      
        vtmp = _mm_mullo_epi32( vsrc, vtmp );
        vcur = _mm_add_epi32( vtmp, vcur );
      }

      vsum[k] = vcur;
    }

    __m128i vout = _mm_hadd_epi32( _mm_hadd_epi32( vsum[0], vsum[1] ), _mm_hadd_epi32( vsum[2], vsum[3] ) );
    vout = _mm_add_epi32( vout, _mm_set1_epi32( 64 ) );
    vout = _mm_srai_epi32( vout, 7 );
    vout = _mm_min_epi32( _mm_max_epi32( vmin, vout ), vmax );

    _mm_storeu_si128( ( __m128i* ) out, vout );
  }
}

template<X86_VEXT vext>
void simdFwdLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  const int8_t *trMat  = ( size > 4 ) ? g_lfnstFwd8x8[mode][index][0] : g_lfnstFwd4x4[mode][index][0];
  const int     trSize = ( size > 4 ) ? 48 : 16;
  int *         out    = dst;

  const __m128i vzero  = _mm_setzero_si128();

  for( int j = 0; j < zeroOutSize; j += 4, out += 4 )
  {
    __m128i vout[4];

    for( int k = 0; k < 4; k++ )
    {
      int* srcPtr = src;
      const int8_t* trMatTmp = trMat;

      __m128i vsum = vzero;

      for( int i = 0; i < trSize; i += 16, srcPtr += 16, trMatTmp += 16 )
      {
        __m128i vtrc = _mm_loadu_si128( ( const __m128i* ) trMatTmp );
        __m128i vtrl = _mm_cvtepi8_epi16( vtrc );
        __m128i vtrh = _mm_cvtepi8_epi16( _mm_unpackhi_epi64( vtrc, vzero ) );

        __m128i vsrc0 = _mm_loadu_si128( ( const __m128i* ) &srcPtr[0] );
                vtrc  = _mm_cvtepi16_epi32( vtrl );
                vsrc0 = _mm_mullo_epi32( vsrc0, vtrc );
              
        __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* ) &srcPtr[4] );
                vtrc  = _mm_cvtepi16_epi32( _mm_unpackhi_epi64( vtrl, vzero ) );
                vsrc1 = _mm_mullo_epi32( vsrc1, vtrc );
              
        __m128i vsrc2 = _mm_loadu_si128( ( const __m128i* ) &srcPtr[8] );
                vtrc  = _mm_cvtepi16_epi32( vtrh );
                vsrc2 = _mm_mullo_epi32( vsrc2, vtrc );
              
        __m128i vsrc3 = _mm_loadu_si128( ( const __m128i* ) &srcPtr[12] );
                vtrc  = _mm_cvtepi16_epi32( _mm_unpackhi_epi64( vtrh, vzero ) );
                vsrc3 = _mm_mullo_epi32( vsrc3, vtrc );

        vsrc0 = _mm_add_epi32( vsrc0, vsrc1 );
        vsrc2 = _mm_add_epi32( vsrc2, vsrc3 );

        vsum = _mm_add_epi32( vsum, _mm_add_epi32( vsrc0, vsrc2 ) );
      }

      vout[k] = vsum;
      trMat += trSize;
    }

    __m128i vdst = _mm_hadd_epi32( _mm_hadd_epi32( vout[0], vout[1] ), _mm_hadd_epi32( vout[2], vout[3] ) );
    vdst = _mm_add_epi32( vdst, _mm_set1_epi32( 64 ) );
    vdst = _mm_srai_epi32( vdst, 7 );

    _mm_storeu_si128( ( __m128i* ) out, vdst );
  }

  ::memset( out, 0, ( trSize - zeroOutSize ) * sizeof( int ) );
}

// ---------------------------------------------------------------------------------------------------------------------
// Partial-butterfly 1D transforms (DCT-II 2/4/8-pt inverse, DCT-II 2/4-pt and DST-VII/DCT-VIII 4-pt forward) and the
// 1D (single line) matrix-multiplication forward core. The lanes run over the transform lines. Every vector expression
// mirrors the scalar expression tree in TrQuant_EMT.cpp, so the 32-bit results are identical to the scalar code.
// The DCT-II coefficients of the even-even part are +-64 by definition (g_trCoreDCT2P*); 64 * a + 64 * b and
// 64 * a - 64 * b are computed as ( a << 6 ) +- ( b << 6 ), which is the same modulo 2^32.
// ---------------------------------------------------------------------------------------------------------------------

static inline void _vv_transpose4x4_epi32( __m128i& r0, __m128i& r1, __m128i& r2, __m128i& r3 )
{
  const __m128i t0 = _mm_unpacklo_epi32( r0, r1 );
  const __m128i t1 = _mm_unpacklo_epi32( r2, r3 );
  const __m128i t2 = _mm_unpackhi_epi32( r0, r1 );
  const __m128i t3 = _mm_unpackhi_epi32( r2, r3 );
  r0 = _mm_unpacklo_epi64( t0, t1 );
  r1 = _mm_unpackhi_epi64( t0, t1 );
  r2 = _mm_unpacklo_epi64( t2, t3 );
  r3 = _mm_unpackhi_epi64( t2, t3 );
}

static inline __m128i _vv_roundClip_epi32( __m128i v, __m128i vadd, __m128i vshift, __m128i vmin, __m128i vmax )
{
  return _mm_min_epi32( vmax, _mm_max_epi32( vmin, _mm_sra_epi32( _mm_add_epi32( v, vadd ), vshift ) ) );
}

static inline void _vv_store_lines4x4( TCoeff* dst, __m128i d0, __m128i d1, __m128i d2, __m128i d3 )
{
  _vv_transpose4x4_epi32( d0, d1, d2, d3 );
  _mm_storeu_si128( ( __m128i* ) &dst[ 0], d0 );
  _mm_storeu_si128( ( __m128i* ) &dst[ 4], d1 );
  _mm_storeu_si128( ( __m128i* ) &dst[ 8], d2 );
  _mm_storeu_si128( ( __m128i* ) &dst[12], d3 );
}

static inline void _vv_load_lines4x4( const TCoeff* src, __m128i& s0, __m128i& s1, __m128i& s2, __m128i& s3 )
{
  s0 = _mm_loadu_si128( ( const __m128i* ) &src[ 0] );
  s1 = _mm_loadu_si128( ( const __m128i* ) &src[ 4] );
  s2 = _mm_loadu_si128( ( const __m128i* ) &src[ 8] );
  s3 = _mm_loadu_si128( ( const __m128i* ) &src[12] );
  _vv_transpose4x4_epi32( s0, s1, s2, s3 );
}

#if USE_AVX2
// transposes the 4x4 blocks in both 128-bit lanes
static inline void _vv_transpose4x4x2_epi32( __m256i& r0, __m256i& r1, __m256i& r2, __m256i& r3 )
{
  const __m256i t0 = _mm256_unpacklo_epi32( r0, r1 );
  const __m256i t1 = _mm256_unpacklo_epi32( r2, r3 );
  const __m256i t2 = _mm256_unpackhi_epi32( r0, r1 );
  const __m256i t3 = _mm256_unpackhi_epi32( r2, r3 );
  r0 = _mm256_unpacklo_epi64( t0, t1 );
  r1 = _mm256_unpackhi_epi64( t0, t1 );
  r2 = _mm256_unpacklo_epi64( t2, t3 );
  r3 = _mm256_unpackhi_epi64( t2, t3 );
}

static inline __m256i _vv_roundClip_epi32( __m256i v, __m256i vadd, __m128i vshift, __m256i vmin, __m256i vmax )
{
  return _mm256_min_epi32( vmax, _mm256_max_epi32( vmin, _mm256_sra_epi32( _mm256_add_epi32( v, vadd ), vshift ) ) );
}

// stores d0..d3 (lane j = line j) as 8 consecutive lines of 4 values
static inline void _vv_store_lines4x8( TCoeff* dst, __m256i d0, __m256i d1, __m256i d2, __m256i d3 )
{
  _vv_transpose4x4x2_epi32( d0, d1, d2, d3 );
  _mm256_storeu_si256( ( __m256i* ) &dst[ 0], _mm256_permute2x128_si256( d0, d1, 0x20 ) );
  _mm256_storeu_si256( ( __m256i* ) &dst[ 8], _mm256_permute2x128_si256( d2, d3, 0x20 ) );
  _mm256_storeu_si256( ( __m256i* ) &dst[16], _mm256_permute2x128_si256( d0, d1, 0x31 ) );
  _mm256_storeu_si256( ( __m256i* ) &dst[24], _mm256_permute2x128_si256( d2, d3, 0x31 ) );
}

// loads 8 consecutive lines of 4 values and returns them transposed (s_k lane j = value k of line j)
static inline void _vv_load_lines4x8( const TCoeff* src, __m256i& s0, __m256i& s1, __m256i& s2, __m256i& s3 )
{
  const __m256i v0 = _mm256_loadu_si256( ( const __m256i* ) &src[ 0] );
  const __m256i v1 = _mm256_loadu_si256( ( const __m256i* ) &src[ 8] );
  const __m256i v2 = _mm256_loadu_si256( ( const __m256i* ) &src[16] );
  const __m256i v3 = _mm256_loadu_si256( ( const __m256i* ) &src[24] );
  s0 = _mm256_permute2x128_si256( v0, v2, 0x20 );
  s1 = _mm256_permute2x128_si256( v0, v2, 0x31 );
  s2 = _mm256_permute2x128_si256( v1, v3, 0x20 );
  s3 = _mm256_permute2x128_si256( v1, v3, 0x31 );
  _vv_transpose4x4x2_epi32( s0, s1, s2, s3 );
}

#endif

// Element-wise int32 operations, so that each butterfly is written once and used for 4 lanes, 8 lanes and the
// scalar tail.
struct Int32x4Ops
{
  static inline __m128i set1( int v )                  { return _mm_set1_epi32( v ); }
  static inline __m128i add ( __m128i a, __m128i b )   { return _mm_add_epi32( a, b ); }
  static inline __m128i sub ( __m128i a, __m128i b )   { return _mm_sub_epi32( a, b ); }
  static inline __m128i mul ( __m128i a, __m128i b )   { return _mm_mullo_epi32( a, b ); }
  static inline __m128i shl6( __m128i a )              { return _mm_slli_epi32( a, 6 ); }
};

struct Int32Ops
{
  static inline int set1( int v )                      { return v; }
  static inline int add ( int a, int b )               { return a + b; }
  static inline int sub ( int a, int b )               { return a - b; }
  static inline int mul ( int a, int b )               { return a * b; }
  static inline int shl6( int a )                      { return int( unsigned( a ) << 6 ); }
};

#if USE_AVX2
struct Int32x8Ops
{
  static inline __m256i set1( int v )                  { return _mm256_set1_epi32( v ); }
  static inline __m256i add ( __m256i a, __m256i b )   { return _mm256_add_epi32( a, b ); }
  static inline __m256i sub ( __m256i a, __m256i b )   { return _mm256_sub_epi32( a, b ); }
  static inline __m256i mul ( __m256i a, __m256i b )   { return _mm256_mullo_epi32( a, b ); }
  static inline __m256i shl6( __m256i a )              { return _mm256_slli_epi32( a, 6 ); }
};
#endif

// inverse 2-pt DCT-II butterfly, d0/d1 before rounding
template<typename V, typename OPS>
static inline void invButterflyDCT2_2( const TMatrixCoeff* iT, const V& s0, const V& s1, V& d0, V& d1 )
{
  CHECKD( iT[0] != 64 || iT[2] != 64, "unexpected DCT-II coefficients" );
  d0 = OPS::shl6( OPS::add( s0, s1 ) );
  d1 = OPS::shl6( OPS::sub( s0, s1 ) );
}

// inverse 4-pt DCT-II butterfly, d0..d3 before rounding
template<typename V, typename OPS>
static inline void invButterflyDCT2_4( const TMatrixCoeff* iT, const V* s, V* d )
{
  const V O0 = OPS::add( OPS::mul( OPS::set1( iT[1 * 4 + 0] ), s[1] ), OPS::mul( OPS::set1( iT[3 * 4 + 0] ), s[3] ) );
  const V O1 = OPS::add( OPS::mul( OPS::set1( iT[1 * 4 + 1] ), s[1] ), OPS::mul( OPS::set1( iT[3 * 4 + 1] ), s[3] ) );
  CHECKD( iT[0 * 4 + 0] != 64 || iT[2 * 4 + 0] != 64 || iT[0 * 4 + 1] != 64 || iT[2 * 4 + 1] != -64, "unexpected DCT-II coefficients" );
  const V E0 = OPS::add( OPS::shl6( s[0] ), OPS::shl6( s[2] ) );
  const V E1 = OPS::sub( OPS::shl6( s[0] ), OPS::shl6( s[2] ) );
  d[0] = OPS::add( E0, O0 );
  d[1] = OPS::add( E1, O1 );
  d[2] = OPS::sub( E1, O1 );
  d[3] = OPS::sub( E0, O0 );
}

// inverse 8-pt DCT-II butterfly, d0..d7 before rounding
template<typename V, typename OPS>
static inline void invButterflyDCT2_8( const TMatrixCoeff* iT, const V* s, V* d )
{
  V O[4], E[4];
  for( int k = 0; k < 4; k++ )
  {
    O[k] = OPS::add( OPS::add( OPS::add( OPS::mul( OPS::set1( iT[1 * 8 + k] ), s[1] ),
                                         OPS::mul( OPS::set1( iT[3 * 8 + k] ), s[3] ) ),
                                         OPS::mul( OPS::set1( iT[5 * 8 + k] ), s[5] ) ),
                                         OPS::mul( OPS::set1( iT[7 * 8 + k] ), s[7] ) );
  }
  const V EO0 = OPS::add( OPS::mul( OPS::set1( iT[2 * 8 + 0] ), s[2] ), OPS::mul( OPS::set1( iT[6 * 8 + 0] ), s[6] ) );
  const V EO1 = OPS::add( OPS::mul( OPS::set1( iT[2 * 8 + 1] ), s[2] ), OPS::mul( OPS::set1( iT[6 * 8 + 1] ), s[6] ) );
  CHECKD( iT[0 * 8 + 0] != 64 || iT[4 * 8 + 0] != 64 || iT[0 * 8 + 1] != 64 || iT[4 * 8 + 1] != -64, "unexpected DCT-II coefficients" );
  const V EE0 = OPS::add( OPS::shl6( s[0] ), OPS::shl6( s[4] ) );
  const V EE1 = OPS::sub( OPS::shl6( s[0] ), OPS::shl6( s[4] ) );
  E[0] = OPS::add( EE0, EO0 );
  E[3] = OPS::sub( EE0, EO0 );
  E[1] = OPS::add( EE1, EO1 );
  E[2] = OPS::sub( EE1, EO1 );
  for( int k = 0; k < 4; k++ )
  {
    d[k    ] = OPS::add( E[    k], O[    k] );
    d[k + 4] = OPS::sub( E[3 - k], O[3 - k] );
  }
}

template<X86_VEXT vext>
void fastInverseDCT2_B2_SIMD( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  const TMatrixCoeff *iT = g_trCoreDCT2P2[TRANSFORM_INVERSE][0];
  const int add          = 1 << ( shift - 1 );
  const int reducedLine  = line - iSkipLine;
  const __m128i vshift   = _mm_cvtsi32_si128( shift );
  int j = 0;

#if USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i vadd = _mm256_set1_epi32( add );
    const __m256i vmin = _mm256_set1_epi32( outputMinimum );
    const __m256i vmax = _mm256_set1_epi32( outputMaximum );

    for( ; j + 8 <= reducedLine; j += 8 )
    {
      const __m256i s0 = _mm256_loadu_si256( ( const __m256i* ) &src[       j] );
      const __m256i s1 = _mm256_loadu_si256( ( const __m256i* ) &src[line + j] );
      __m256i e, o;
      invButterflyDCT2_2<__m256i, Int32x8Ops>( iT, s0, s1, e, o );
      e = _vv_roundClip_epi32( e, vadd, vshift, vmin, vmax );
      o = _vv_roundClip_epi32( o, vadd, vshift, vmin, vmax );
      const __m256i lo = _mm256_unpacklo_epi32( e, o );
      const __m256i hi = _mm256_unpackhi_epi32( e, o );
      _mm256_storeu_si256( ( __m256i* ) &dst[2 * j    ], _mm256_permute2x128_si256( lo, hi, 0x20 ) );
      _mm256_storeu_si256( ( __m256i* ) &dst[2 * j + 8], _mm256_permute2x128_si256( lo, hi, 0x31 ) );
    }
  }
#endif
  {
    const __m128i vadd = _mm_set1_epi32( add );
    const __m128i vmin = _mm_set1_epi32( outputMinimum );
    const __m128i vmax = _mm_set1_epi32( outputMaximum );

    for( ; j + 4 <= reducedLine; j += 4 )
    {
      const __m128i s0 = _mm_loadu_si128( ( const __m128i* ) &src[       j] );
      const __m128i s1 = _mm_loadu_si128( ( const __m128i* ) &src[line + j] );
      __m128i e, o;
      invButterflyDCT2_2<__m128i, Int32x4Ops>( iT, s0, s1, e, o );
      e = _vv_roundClip_epi32( e, vadd, vshift, vmin, vmax );
      o = _vv_roundClip_epi32( o, vadd, vshift, vmin, vmax );
      _mm_storeu_si128( ( __m128i* ) &dst[2 * j    ], _mm_unpacklo_epi32( e, o ) );
      _mm_storeu_si128( ( __m128i* ) &dst[2 * j + 4], _mm_unpackhi_epi32( e, o ) );
    }
  }

  for( ; j < reducedLine; j++ )
  {
    int E, O;
    invButterflyDCT2_2<int, Int32Ops>( iT, src[j], src[line + j], E, O );
    dst[2 * j    ] = Clip3( outputMinimum, outputMaximum, ( E + add ) >> shift );
    dst[2 * j + 1] = Clip3( outputMinimum, outputMaximum, ( O + add ) >> shift );
  }

  if( iSkipLine )
  {
    memset( dst + 2 * reducedLine, 0, ( iSkipLine << 1 ) * sizeof( TCoeff ) );
  }
}

template<X86_VEXT vext>
void fastInverseDCT2_B4_SIMD( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  const TMatrixCoeff *iT = g_trCoreDCT2P4[TRANSFORM_INVERSE][0];
  const int add          = 1 << ( shift - 1 );
  const int reducedLine  = line - iSkipLine;
  const __m128i vshift   = _mm_cvtsi32_si128( shift );
  int j = 0;

#if USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i vadd = _mm256_set1_epi32( add );
    const __m256i vmin = _mm256_set1_epi32( outputMinimum );
    const __m256i vmax = _mm256_set1_epi32( outputMaximum );

    for( ; j + 8 <= reducedLine; j += 8 )
    {
      __m256i s[4], d[4];
      for( int r = 0; r < 4; r++ )
      {
        s[r] = _mm256_loadu_si256( ( const __m256i* ) &src[r * line + j] );
      }
      invButterflyDCT2_4<__m256i, Int32x8Ops>( iT, s, d );
      for( int r = 0; r < 4; r++ )
      {
        d[r] = _vv_roundClip_epi32( d[r], vadd, vshift, vmin, vmax );
      }
      _vv_store_lines4x8( &dst[4 * j], d[0], d[1], d[2], d[3] );
    }
  }
#endif
  {
    const __m128i vadd = _mm_set1_epi32( add );
    const __m128i vmin = _mm_set1_epi32( outputMinimum );
    const __m128i vmax = _mm_set1_epi32( outputMaximum );

    for( ; j + 4 <= reducedLine; j += 4 )
    {
      __m128i s[4], d[4];
      for( int r = 0; r < 4; r++ )
      {
        s[r] = _mm_loadu_si128( ( const __m128i* ) &src[r * line + j] );
      }
      invButterflyDCT2_4<__m128i, Int32x4Ops>( iT, s, d );
      for( int r = 0; r < 4; r++ )
      {
        d[r] = _vv_roundClip_epi32( d[r], vadd, vshift, vmin, vmax );
      }
      _vv_store_lines4x4( &dst[4 * j], d[0], d[1], d[2], d[3] );
    }
  }

  for( ; j < reducedLine; j++ )
  {
    int s[4], d[4];
    for( int r = 0; r < 4; r++ )
    {
      s[r] = src[r * line + j];
    }
    invButterflyDCT2_4<int, Int32Ops>( iT, s, d );
    for( int r = 0; r < 4; r++ )
    {
      dst[4 * j + r] = Clip3( outputMinimum, outputMaximum, ( d[r] + add ) >> shift );
    }
  }

  if( iSkipLine )
  {
    memset( dst + 4 * reducedLine, 0, ( iSkipLine << 2 ) * sizeof( TCoeff ) );
  }
}

template<X86_VEXT vext>
void fastInverseDCT2_B8_SIMD( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  const TMatrixCoeff *iT = g_trCoreDCT2P8[TRANSFORM_INVERSE][0];
  const int add          = 1 << ( shift - 1 );
  const int reducedLine  = line - iSkipLine;
  const __m128i vshift   = _mm_cvtsi32_si128( shift );
  int j = 0;

#if USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i vadd = _mm256_set1_epi32( add );
    const __m256i vmin = _mm256_set1_epi32( outputMinimum );
    const __m256i vmax = _mm256_set1_epi32( outputMaximum );

    for( ; j + 8 <= reducedLine; j += 8 )
    {
      __m256i s[8], d[8];
      for( int r = 0; r < 8; r++ )
      {
        s[r] = _mm256_loadu_si256( ( const __m256i* ) &src[r * line + j] );
      }
      invButterflyDCT2_8<__m256i, Int32x8Ops>( iT, s, d );
      for( int r = 0; r < 8; r++ )
      {
        d[r] = _vv_roundClip_epi32( d[r], vadd, vshift, vmin, vmax );
      }

      // 8x8 transpose: lane r of d[n] goes to dst[( j + r ) * 8 + n]
      __m256i t[8], u[8];
      t[0] = _mm256_unpacklo_epi32( d[0], d[1] ); t[1] = _mm256_unpackhi_epi32( d[0], d[1] );
      t[2] = _mm256_unpacklo_epi32( d[2], d[3] ); t[3] = _mm256_unpackhi_epi32( d[2], d[3] );
      t[4] = _mm256_unpacklo_epi32( d[4], d[5] ); t[5] = _mm256_unpackhi_epi32( d[4], d[5] );
      t[6] = _mm256_unpacklo_epi32( d[6], d[7] ); t[7] = _mm256_unpackhi_epi32( d[6], d[7] );
      u[0] = _mm256_unpacklo_epi64( t[0], t[2] ); u[1] = _mm256_unpackhi_epi64( t[0], t[2] );
      u[2] = _mm256_unpacklo_epi64( t[1], t[3] ); u[3] = _mm256_unpackhi_epi64( t[1], t[3] );
      u[4] = _mm256_unpacklo_epi64( t[4], t[6] ); u[5] = _mm256_unpackhi_epi64( t[4], t[6] );
      u[6] = _mm256_unpacklo_epi64( t[5], t[7] ); u[7] = _mm256_unpackhi_epi64( t[5], t[7] );

      TCoeff* dd = &dst[8 * j];
      for( int r = 0; r < 4; r++ )
      {
        _mm256_storeu_si256( ( __m256i* ) &dd[8 * r     ], _mm256_permute2x128_si256( u[r], u[r + 4], 0x20 ) );
        _mm256_storeu_si256( ( __m256i* ) &dd[8 * r + 32], _mm256_permute2x128_si256( u[r], u[r + 4], 0x31 ) );
      }
    }
  }
#endif
  {
    const __m128i vadd = _mm_set1_epi32( add );
    const __m128i vmin = _mm_set1_epi32( outputMinimum );
    const __m128i vmax = _mm_set1_epi32( outputMaximum );

    for( ; j + 4 <= reducedLine; j += 4 )
    {
      __m128i s[8], d[8];
      for( int r = 0; r < 8; r++ )
      {
        s[r] = _mm_loadu_si128( ( const __m128i* ) &src[r * line + j] );
      }
      invButterflyDCT2_8<__m128i, Int32x4Ops>( iT, s, d );
      for( int r = 0; r < 8; r++ )
      {
        d[r] = _vv_roundClip_epi32( d[r], vadd, vshift, vmin, vmax );
      }

      _vv_transpose4x4_epi32( d[0], d[1], d[2], d[3] );
      _vv_transpose4x4_epi32( d[4], d[5], d[6], d[7] );

      TCoeff* dd = &dst[8 * j];
      for( int r = 0; r < 4; r++ )
      {
        _mm_storeu_si128( ( __m128i* ) &dd[8 * r    ], d[r    ] );
        _mm_storeu_si128( ( __m128i* ) &dd[8 * r + 4], d[r + 4] );
      }
    }
  }

  for( ; j < reducedLine; j++ )
  {
    int s[8], d[8];
    for( int r = 0; r < 8; r++ )
    {
      s[r] = src[r * line + j];
    }
    invButterflyDCT2_8<int, Int32Ops>( iT, s, d );
    for( int r = 0; r < 8; r++ )
    {
      dst[8 * j + r] = Clip3( outputMinimum, outputMaximum, ( d[r] + add ) >> shift );
    }
  }

  if( iSkipLine )
  {
    memset( dst + 8 * reducedLine, 0, ( iSkipLine << 3 ) * sizeof( TCoeff ) );
  }
}

template<X86_VEXT vext>
void fastForwardDCT2_B2_SIMD( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  const TMatrixCoeff *iT = g_trCoreDCT2P2[TRANSFORM_FORWARD][0];
  const TCoeff add       = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;
  CHECKD( iT[0] != 64 || iT[2] != 64, "unexpected DCT-II coefficients" );
  const int reducedLine  = line - iSkipLine;
  const __m128i vshift   = _mm_cvtsi32_si128( shift );
  int j = 0;

#if USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i vadd = _mm256_set1_epi32( add );

    for( ; j + 8 <= reducedLine; j += 8 )
    {
      // [s0 s1] pairs of 8 lines -> s0 of the 8 lines, s1 of the 8 lines
      const __m256i a  = _mm256_shuffle_epi32( _mm256_loadu_si256( ( const __m256i* ) &src[2 * j    ] ), 0xD8 );
      const __m256i b  = _mm256_shuffle_epi32( _mm256_loadu_si256( ( const __m256i* ) &src[2 * j + 8] ), 0xD8 );
      const __m256i s0 = _mm256_permute4x64_epi64( _mm256_unpacklo_epi64( a, b ), 0xD8 );
      const __m256i s1 = _mm256_permute4x64_epi64( _mm256_unpackhi_epi64( a, b ), 0xD8 );
      const __m256i E  = _mm256_add_epi32( s0, s1 );
      const __m256i O  = _mm256_sub_epi32( s0, s1 );
      _mm256_storeu_si256( ( __m256i* ) &dst[       j], _mm256_sra_epi32( _mm256_add_epi32( Int32x8Ops::shl6( E ), vadd ), vshift ) );
      _mm256_storeu_si256( ( __m256i* ) &dst[line + j], _mm256_sra_epi32( _mm256_add_epi32( Int32x8Ops::shl6( O ), vadd ), vshift ) );
    }
  }
#endif
  {
    const __m128i vadd = _mm_set1_epi32( add );

    for( ; j + 4 <= reducedLine; j += 4 )
    {
      const __m128i a  = _mm_shuffle_epi32( _mm_loadu_si128( ( const __m128i* ) &src[2 * j    ] ), 0xD8 );
      const __m128i b  = _mm_shuffle_epi32( _mm_loadu_si128( ( const __m128i* ) &src[2 * j + 4] ), 0xD8 );
      const __m128i s0 = _mm_unpacklo_epi64( a, b );
      const __m128i s1 = _mm_unpackhi_epi64( a, b );
      const __m128i E  = _mm_add_epi32( s0, s1 );
      const __m128i O  = _mm_sub_epi32( s0, s1 );
      _mm_storeu_si128( ( __m128i* ) &dst[       j], _mm_sra_epi32( _mm_add_epi32( Int32x4Ops::shl6( E ), vadd ), vshift ) );
      _mm_storeu_si128( ( __m128i* ) &dst[line + j], _mm_sra_epi32( _mm_add_epi32( Int32x4Ops::shl6( O ), vadd ), vshift ) );
    }
  }

  for( ; j < reducedLine; j++ )
  {
    const int E = src[2 * j] + src[2 * j + 1];
    const int O = src[2 * j] - src[2 * j + 1];
    dst[       j] = ( iT[0] * E + add ) >> shift;
    dst[line + j] = ( iT[2] * O + add ) >> shift;
  }

  if( iSkipLine )
  {
    for( int k = 0; k < 2; k++ )
    {
      memset( dst + k * line + reducedLine, 0, sizeof( TCoeff ) * iSkipLine );
    }
  }
}

// 4-pt forward butterflies of TrQuant_EMT.cpp (trType: DCT2, DST7 or DCT8), d0..d3 before the shift
template<int trType, typename V, typename OPS>
static inline void fwdButterfly4( const TMatrixCoeff* iT, const V& s0, const V& s1, const V& s2, const V& s3, const V& vadd, V& d0, V& d1, V& d2, V& d3 )
{
  if( trType == DCT2 )
  {
    const V E0 = OPS::add( s0, s3 ), O0 = OPS::sub( s0, s3 );
    const V E1 = OPS::add( s1, s2 ), O1 = OPS::sub( s1, s2 );
    CHECKD( iT[0] != 64 || iT[1] != 64 || iT[8] != 64 || iT[9] != -64, "unexpected DCT-II coefficients" );
    d0 = OPS::add( OPS::add( OPS::shl6( E0 ), OPS::shl6( E1 ) ), vadd );
    d2 = OPS::add( OPS::sub( OPS::shl6( E0 ), OPS::shl6( E1 ) ), vadd );
    d1 = OPS::add( OPS::add( OPS::mul( OPS::set1( iT[ 4] ), O0 ), OPS::mul( OPS::set1( iT[ 5] ), O1 ) ), vadd );
    d3 = OPS::add( OPS::add( OPS::mul( OPS::set1( iT[12] ), O0 ), OPS::mul( OPS::set1( iT[13] ), O1 ) ), vadd );
  }
  else if( trType == DST7 )
  {
    const V c0 = OPS::add( s0, s3 );
    const V c1 = OPS::add( s1, s3 );
    const V c2 = OPS::sub( s0, s1 );
    const V c3 = OPS::mul( OPS::set1( iT[2] ), s2 );
    d0 = OPS::add( OPS::add( OPS::add( OPS::mul( OPS::set1( iT[0] ), c0 ), OPS::mul( OPS::set1( iT[1] ), c1 ) ), c3 ), vadd );
    d1 = OPS::add( OPS::mul( OPS::set1( iT[2] ), OPS::sub( OPS::add( s0, s1 ), s3 ) ), vadd );
    d2 = OPS::add( OPS::sub( OPS::add( OPS::mul( OPS::set1( iT[0] ), c2 ), OPS::mul( OPS::set1( iT[1] ), c0 ) ), c3 ), vadd );
    d3 = OPS::add( OPS::add( OPS::sub( OPS::mul( OPS::set1( iT[1] ), c2 ), OPS::mul( OPS::set1( iT[0] ), c1 ) ), c3 ), vadd );
  }
  else
  {
    const V c0 = OPS::add( s0, s3 );
    const V c1 = OPS::add( s2, s0 );
    const V c2 = OPS::sub( s3, s2 );
    const V c3 = OPS::mul( OPS::set1( iT[1] ), s1 );
    d0 = OPS::add( OPS::add( OPS::add( OPS::mul( OPS::set1( iT[3] ), c0 ), OPS::mul( OPS::set1( iT[2] ), c1 ) ), c3 ), vadd );
    d1 = OPS::add( OPS::mul( OPS::set1( iT[1] ), OPS::sub( OPS::sub( s0, s2 ), s3 ) ), vadd );
    d2 = OPS::add( OPS::sub( OPS::add( OPS::mul( OPS::set1( iT[3] ), c2 ), OPS::mul( OPS::set1( iT[2] ), c0 ) ), c3 ), vadd );
    d3 = OPS::add( OPS::sub( OPS::sub( OPS::mul( OPS::set1( iT[3] ), c1 ), OPS::mul( OPS::set1( iT[2] ), c2 ) ), c3 ), vadd );
  }
}

template<X86_VEXT vext, int trType>
void fastForward4_SIMD( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  const TMatrixCoeff *iT = trType == DCT2 ? g_trCoreDCT2P4[TRANSFORM_FORWARD][0] : trType == DST7 ? g_trCoreDST7P4[TRANSFORM_FORWARD][0] : g_trCoreDCT8P4[TRANSFORM_FORWARD][0];
  // same rounding offset as the scalar code (the DCT-VIII code does not check for shift == 0)
  const TCoeff add       = trType == DCT8 ? ( 1 << ( shift - 1 ) ) : ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;
  const int reducedLine  = line - iSkipLine;
  const __m128i vshift   = _mm_cvtsi32_si128( shift );
  int j = 0;

#if USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i vadd = _mm256_set1_epi32( add );

    for( ; j + 8 <= reducedLine; j += 8 )
    {
      __m256i s0, s1, s2, s3, d0, d1, d2, d3;
      _vv_load_lines4x8( &src[4 * j], s0, s1, s2, s3 );
      fwdButterfly4<trType, __m256i, Int32x8Ops>( iT, s0, s1, s2, s3, vadd, d0, d1, d2, d3 );
      _mm256_storeu_si256( ( __m256i* ) &dst[           j], _mm256_sra_epi32( d0, vshift ) );
      _mm256_storeu_si256( ( __m256i* ) &dst[    line + j], _mm256_sra_epi32( d1, vshift ) );
      _mm256_storeu_si256( ( __m256i* ) &dst[2 * line + j], _mm256_sra_epi32( d2, vshift ) );
      _mm256_storeu_si256( ( __m256i* ) &dst[3 * line + j], _mm256_sra_epi32( d3, vshift ) );
    }
  }
#endif
  {
    const __m128i vadd = _mm_set1_epi32( add );

    for( ; j + 4 <= reducedLine; j += 4 )
    {
      __m128i s0, s1, s2, s3, d0, d1, d2, d3;
      _vv_load_lines4x4( &src[4 * j], s0, s1, s2, s3 );
      fwdButterfly4<trType, __m128i, Int32x4Ops>( iT, s0, s1, s2, s3, vadd, d0, d1, d2, d3 );
      _mm_storeu_si128( ( __m128i* ) &dst[           j], _mm_sra_epi32( d0, vshift ) );
      _mm_storeu_si128( ( __m128i* ) &dst[    line + j], _mm_sra_epi32( d1, vshift ) );
      _mm_storeu_si128( ( __m128i* ) &dst[2 * line + j], _mm_sra_epi32( d2, vshift ) );
      _mm_storeu_si128( ( __m128i* ) &dst[3 * line + j], _mm_sra_epi32( d3, vshift ) );
    }
  }

  for( ; j < reducedLine; j++ )
  {
    const TCoeff* s = &src[4 * j];
    int d0, d1, d2, d3;
    fwdButterfly4<trType, int, Int32Ops>( iT, s[0], s[1], s[2], s[3], add, d0, d1, d2, d3 );
    dst[           j] = d0 >> shift;
    dst[    line + j] = d1 >> shift;
    dst[2 * line + j] = d2 >> shift;
    dst[3 * line + j] = d3 >> shift;
  }

  if( iSkipLine )
  {
    for( int k = 0; k < 4; k++ )
    {
      memset( dst + k * line + reducedLine, 0, sizeof( TCoeff ) * iSkipLine );
    }
  }
}

// forward matrix multiplication for a single line (1D transforms of 1xN / Nx1 blocks)
template<X86_VEXT vext, unsigned trSize>
void fastFwd1D_SIMD( const TMatrixCoeff* tc, const TCoeff* src, TCoeff* dst, unsigned line, unsigned reducedLine, unsigned cutoff, int shift )
{
  const int rnd_factor = 1 << ( shift - 1 );

  for( unsigned i = 0; i < reducedLine; i++ )
  {
    const TMatrixCoeff* iT = tc;
    unsigned j = 0;

    for( ; j + 4 <= cutoff; j += 4 )
    {
      __m128i vsum;
#if USE_AVX2
      if( vext >= AVX2 && trSize >= 8 )
      {
        __m256i acc[4];
        for( int r = 0; r < 4; r++ )
        {
          acc[r] = _mm256_setzero_si256();
          for( unsigned k = 0; k < trSize; k += 8 )
          {
            const __m256i vs = _mm256_loadu_si256( ( const __m256i* ) &src[k] );
            const __m256i vt = _mm256_cvtepi16_epi32( _mm_loadu_si128( ( const __m128i* ) &iT[r * trSize + k] ) );
            acc[r] = _mm256_add_epi32( acc[r], _mm256_mullo_epi32( vs, vt ) );
          }
        }
        const __m256i h = _mm256_hadd_epi32( _mm256_hadd_epi32( acc[0], acc[1] ), _mm256_hadd_epi32( acc[2], acc[3] ) );
        vsum = _mm_add_epi32( _mm256_castsi256_si128( h ), _mm256_extracti128_si256( h, 1 ) );
      }
      else
#endif
      {
        __m128i acc[4];
        for( int r = 0; r < 4; r++ )
        {
          acc[r] = _mm_setzero_si128();
          for( unsigned k = 0; k < trSize; k += 4 )
          {
            const __m128i vs = _mm_loadu_si128( ( const __m128i* ) &src[k] );
            const __m128i vt = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( const __m128i* ) &iT[r * trSize + k] ) );
            acc[r] = _mm_add_epi32( acc[r], _mm_mullo_epi32( vs, vt ) );
          }
        }
        vsum = _mm_hadd_epi32( _mm_hadd_epi32( acc[0], acc[1] ), _mm_hadd_epi32( acc[2], acc[3] ) );
      }
      vsum = _mm_sra_epi32( _mm_add_epi32( vsum, _mm_set1_epi32( rnd_factor ) ), _mm_cvtsi32_si128( shift ) );

      if( line == 1 )
      {
        _mm_storeu_si128( ( __m128i* ) &dst[j], vsum );
      }
      else
      {
        dst[( j + 0 ) * line + i] = _mm_extract_epi32( vsum, 0 );
        dst[( j + 1 ) * line + i] = _mm_extract_epi32( vsum, 1 );
        dst[( j + 2 ) * line + i] = _mm_extract_epi32( vsum, 2 );
        dst[( j + 3 ) * line + i] = _mm_extract_epi32( vsum, 3 );
      }
      iT += 4 * trSize;
    }

    for( ; j < cutoff; j++ )
    {
      int sum = 0;
      for( unsigned k = 0; k < trSize; k++ )
      {
        sum += src[k] * iT[k];
      }
      dst[j * line + i] = ( sum + rnd_factor ) >> shift;
      iT += trSize;
    }

    src += trSize;
  }
}

template<X86_VEXT vext>
void TCoeffOps::_initTCoeffOpsX86()
{
  cpyResi4     = cpyResi_SSE  <vext, 4>;
  cpyResi8     = cpyResi_SSE  <vext, 8>;
  cpyCoeff4    = cpyCoeff_SSE <vext, 4>;
  cpyCoeff8    = cpyCoeff_SSE <vext, 8>;
  roundClip4   = roundClip_SSE<vext, 4>;
  roundClip8   = roundClip_SSE<vext, 8>;

  fastInvCore[0] = fastInv_SSE<vext,  4>;
  fastInvCore[1] = fastInv_SSE<vext,  8>;
  fastInvCore[2] = fastInv_SSE<vext, 16>;
  fastInvCore[3] = fastInv_SSE<vext, 32>;
  fastInvCore[4] = fastInv_SSE<vext, 64>;

  fastFwdCore_2D[0] = fastFwd_SSE<vext,  4>;
  fastFwdCore_2D[1] = fastFwd_SSE<vext,  8>;
  fastFwdCore_2D[2] = fastFwd_SSE<vext, 16>;
  fastFwdCore_2D[3] = fastFwd_SSE<vext, 32>;
  fastFwdCore_2D[4] = fastFwd_SSE<vext, 64>;

  fastFwdCore_1D[0] = fastFwd1D_SIMD<vext,  4>;
  fastFwdCore_1D[1] = fastFwd1D_SIMD<vext,  8>;
  fastFwdCore_1D[2] = fastFwd1D_SIMD<vext, 16>;
  fastFwdCore_1D[3] = fastFwd1D_SIMD<vext, 32>;
  fastFwdCore_1D[4] = fastFwd1D_SIMD<vext, 64>;

  fastInvDCT2[0] = fastInverseDCT2_B2_SIMD<vext>;
  fastInvDCT2[1] = fastInverseDCT2_B4_SIMD<vext>;
  fastInvDCT2[2] = fastInverseDCT2_B8_SIMD<vext>;
  fastFwdDCT2[0] = fastForwardDCT2_B2_SIMD<vext>;
  fastFwdDCT2[1] = fastForward4_SIMD<vext, DCT2>;
  fastFwdDST7_4  = fastForward4_SIMD<vext, DST7>;
  fastFwdDCT8_4  = fastForward4_SIMD<vext, DCT8>;
}

template<X86_VEXT vext>
void TrQuant::_initTrQuantX86()
{
  m_invLfnstNxN = simdInvLfnstNxN<vext>;
  m_fwdLfnstNxN  = simdFwdLfnstNxN<vext>;
}

template void TCoeffOps::_initTCoeffOpsX86<SIMDX86>();
template void TrQuant::_initTrQuantX86<SIMDX86>();

}

#endif // TARGET_SIMD_X86
#endif
//! \}

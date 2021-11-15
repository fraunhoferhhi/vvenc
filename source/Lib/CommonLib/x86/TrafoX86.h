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
/** \file     TrafoX86.h
    \brief    SIMD averaging.
*/

//! \ingroup CommonLib
//! \{

#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"

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
                         : _mm_packs_epi32( _mm_loadl_epi64( ( const __m128i* )srcPtr0 ), _mm_loadl_epi64( ( const __m128i* )srcPtr1 ) );
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
                         : _mm_packs_epi32( _mm_loadl_epi64( ( const __m128i* )srcPtr0 ), _mm_loadl_epi64( ( const __m128i* )srcPtr1 ) );
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
                        : _mm_packs_epi32( _mm_loadl_epi64( ( const __m128i* )srcPtr0 ), _mm_loadl_epi64( ( const __m128i* )srcPtr1 ) );
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

      __m128i vit = _mm_unpacklo_epi16( _mm_loadl_epi64( ( const __m128i * ) itPtr0 ), _mm_loadl_epi64( ( const __m128i * ) itPtr1 ) );
 
      for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
      {
        __m128i xscale = maxLoopL == 4
                        ? _mm_packs_epi32( _mm_loadu_si128( ( const __m128i* )srcPtr0 ), _mm_loadu_si128( ( const __m128i* )srcPtr1 ) )
                        : _mm_packs_epi32( _mm_loadl_epi64( ( const __m128i* )srcPtr0 ), _mm_loadl_epi64( ( const __m128i* )srcPtr1 ) );
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
#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ )
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
          
        for( int k = 0; k < trSize; k += 16 )
        {
          __m256i vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[k + 0] );
          __m256i vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[k + 8] );
          __m256i vsrc  = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[k >> 4][0] = vsrc;
          
          vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[k + 0 + trSize] );
          vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[k + 8 + trSize] );
          vsrc  = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc  = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[k >> 4][1] = vsrc;

          if( !nlx4 ) continue;

          vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[k + 0 + 2 * trSize] );
          vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[k + 8 + 2 * trSize] );
          vsrc = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[k >> 4][2] = vsrc;

          vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[k + 0 + 3 * trSize] );
          vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[k + 8 + 3 * trSize] );
          vsrc = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[k >> 4][3] = vsrc;
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

          for( int k = 0; k < trSize; k += 16 )
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
            __m256i vit0  = _mm256_load_si256( ( const __m256i* ) &itPtr[k + 0 * trSize] );
            __m256i vit1  = _mm256_load_si256( ( const __m256i* ) &itPtr[k + 1 * trSize] );
            __m256i vit2  = _mm256_load_si256( ( const __m256i* ) &itPtr[k + 2 * trSize] );
            __m256i vit3  = _mm256_load_si256( ( const __m256i* ) &itPtr[k + 3 * trSize] );
#endif

            // first source line
            __m256i vsrc  = vsrcarr[k >> 4][0];

            vsum00 = _mm256_add_epi32( vsum00, _mm256_hadd_epi32( _mm256_madd_epi16( vit0, vsrc ), _mm256_madd_epi16( vit1, vsrc ) ) );
            vsum02 = _mm256_add_epi32( vsum02, _mm256_hadd_epi32( _mm256_madd_epi16( vit2, vsrc ), _mm256_madd_epi16( vit3, vsrc ) ) );
     
            vsrc  = vsrcarr[k >> 4][1];

            vsum10 = _mm256_add_epi32( vsum10, _mm256_hadd_epi32( _mm256_madd_epi16( vit0, vsrc ), _mm256_madd_epi16( vit1, vsrc ) ) );
            vsum12 = _mm256_add_epi32( vsum12, _mm256_hadd_epi32( _mm256_madd_epi16( vit2, vsrc ), _mm256_madd_epi16( vit3, vsrc ) ) );

            // skip branching
            //if( !nlx4 ) continue;
     
            vsrc  = vsrcarr[k >> 4][2];

            vsum20 = _mm256_add_epi32( vsum20, _mm256_hadd_epi32( _mm256_madd_epi16( vit0, vsrc ), _mm256_madd_epi16( vit1, vsrc ) ) );
            vsum22 = _mm256_add_epi32( vsum22, _mm256_hadd_epi32( _mm256_madd_epi16( vit2, vsrc ), _mm256_madd_epi16( vit3, vsrc ) ) );
            
            vsrc  = vsrcarr[k >> 4][3];

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

            _mm_storel_epi64( ( __m128i* ) dstPtr,                     xtmp );         dstPtr += line;
            _mm_storel_epi64( ( __m128i* ) dstPtr, _mm_unpackhi_epi64( xtmp, xtmp ) ); dstPtr += line;

            xtmp = _mm_unpackhi_epi32( xsum00, xsum10 );

            _mm_storel_epi64( ( __m128i* ) dstPtr,                     xtmp );         dstPtr += line;
            _mm_storel_epi64( ( __m128i* ) dstPtr, _mm_unpackhi_epi64( xtmp, xtmp ) ); dstPtr += line;
          }

          itPtr  += ( trSize << 2 );
        }

        src += ( trSize << ( 1 + nlx4 ) );
      }
#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ )
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
          
        for( int k = 0; k < trSize; k += 8 )
        {
          __m128i vsrc0 = _mm_load_si128( ( const __m128i* ) &src[k + 0] );
          __m128i vsrc1 = _mm_load_si128( ( const __m128i* ) &src[k + 4] );
          __m128i vsrc  = _mm_packs_epi32( vsrc0, vsrc1 );

          vsrcarr[k >> 3][0] = vsrc;
          
          vsrc0 = _mm_load_si128( ( const __m128i* ) &src[k + 0 + trSize] );
          vsrc1 = _mm_load_si128( ( const __m128i* ) &src[k + 4 + trSize] );
          vsrc  = _mm_packs_epi32( vsrc0, vsrc1 );

          vsrcarr[k >> 3][1] = vsrc;
        }

        for( int j = 0; j < cutoff; j += 4 )
        {
          __m128i vsum00 = _mm_setzero_si128();
          //__m128i vsum01 = _mm_setzero_si128();
          __m128i vsum02 = _mm_setzero_si128();
          //__m128i vsum03 = _mm_setzero_si128();
        
          __m128i vsum10 = _mm_setzero_si128();
          //__m128i vsum11 = _mm_setzero_si128();
          __m128i vsum12 = _mm_setzero_si128();
          //__m128i vsum13 = _mm_setzero_si128();

          for( int k = 0; k < trSize; k += 8 )
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
            __m128i vit0  = _mm_load_si128( ( const __m128i* ) &itPtr[k + 0 * trSize] );
            __m128i vit1  = _mm_load_si128( ( const __m128i* ) &itPtr[k + 1 * trSize] );
            __m128i vit2  = _mm_load_si128( ( const __m128i* ) &itPtr[k + 2 * trSize] );
            __m128i vit3  = _mm_load_si128( ( const __m128i* ) &itPtr[k + 3 * trSize] );
  #endif
            
            // fist source line
            __m128i vsrc  = vsrcarr[k >> 3][0];

            vsum00 = _mm_add_epi32( vsum00, _mm_hadd_epi32( _mm_madd_epi16( vit0, vsrc ), _mm_madd_epi16( vit1, vsrc ) ) );
            vsum02 = _mm_add_epi32( vsum02, _mm_hadd_epi32( _mm_madd_epi16( vit2, vsrc ), _mm_madd_epi16( vit3, vsrc ) ) );
          
            // second source line
            vsrc   = vsrcarr[k >> 3][1];

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
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp ); dstPtr += line;

          xtmp = _mm_shuffle_epi32( xtmp, ( 2 << 0 ) + ( 3 << 2 ) );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp ); dstPtr += line;
          
          xtmp = _mm_unpackhi_epi32( vsum00, vsum10 );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp ); dstPtr += line;

          xtmp = _mm_shuffle_epi32( xtmp, ( 2 << 0 ) + ( 3 << 2 ) );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp ); dstPtr += line;

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
        __m128i vit   = _mm_loadl_epi64( ( const __m128i* ) itPtr );
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
        _mm_storel_epi64      ( ( __m128i * ) &dst[col], vdst );
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
        __m128i vtmp = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i * ) &src[col] ) );
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
}

template void TCoeffOps::_initTCoeffOpsX86<SIMDX86>();

}

#endif // TARGET_SIMD_X86
#endif
//! \}

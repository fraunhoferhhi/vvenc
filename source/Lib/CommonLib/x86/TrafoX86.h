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

template< X86_VEXT vext, int W >
void fastInv_SSE( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned lines, unsigned reducedLines, unsigned rows )
{
  unsigned maxLoopL = std::min<int>( reducedLines, 4 );

#if USE_AVX2
  if( W >= 8 && vext >= AVX2 )
  {
    if( ( trSize & 15 ) == 0 )
    {
      unsigned trLoops = trSize >> 4;

      for( int k = 0; k < rows; k += 2 )
      {
              TCoeff* dstPtr =  dst;

        const TCoeff* srcPtr0 = &src[ k      * lines];
        const TCoeff* srcPtr1 = &src[(k + 1) * lines];

        __m256i vsrc1v[4][2];
        
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
  if( W >= 8 )
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
  else if( W >= 4 )
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

template<X86_VEXT vext, int W>
void fastFwd_SSE( const TMatrixCoeff* tc, const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned line, unsigned reducedLine, unsigned cutoff, int shift )
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

  if( W >= 8 )
  {
#if USE_AVX2
    if( vext >= AVX2 && ( trSize & 15 ) == 0 )
    {
      for( int i = 0; i < reducedLine; i += 2 )
      {
              TCoeff*       dstPtr = dst + i;
        const TMatrixCoeff* itPtr  = tc;
        
        __m256i vsrcarr[2][4];
          
        for( int k = 0; k < trSize; k += 16 )
        {
          __m256i vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[k + 0] );
          __m256i vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[k + 8] );
          __m256i vsrc  = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[0][k >> 4] = vsrc;
          
          vsrc0 = _mm256_load_si256( ( const __m256i* ) &src[k + 0 + trSize] );
          vsrc1 = _mm256_load_si256( ( const __m256i* ) &src[k + 8 + trSize] );
          vsrc  = _mm256_packs_epi32( vsrc0, vsrc1 );
          vsrc  = _mm256_permute4x64_epi64( vsrc, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

          vsrcarr[1][k >> 4] = vsrc;
        }

        for( int j = 0; j < cutoff; j += 4 )
        {
          __m256i vsum00 = _mm256_setzero_si256();
          __m256i vsum01 = _mm256_setzero_si256();
          __m256i vsum02 = _mm256_setzero_si256();
          __m256i vsum03 = _mm256_setzero_si256();

          __m256i vsum10 = _mm256_setzero_si256();
          __m256i vsum11 = _mm256_setzero_si256();
          __m256i vsum12 = _mm256_setzero_si256();
          __m256i vsum13 = _mm256_setzero_si256();

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
            __m256i vsrc  = vsrcarr[0][k >> 4];

            __m256i
            vtmp   = _mm256_madd_epi16( vit0,   vsrc );
            vsum00 = _mm256_add_epi32 ( vsum00, vtmp );
          
            vtmp   = _mm256_madd_epi16( vit1,   vsrc );
            vsum01 = _mm256_add_epi32 ( vsum01, vtmp );
          
            vtmp   = _mm256_madd_epi16( vit2,   vsrc );
            vsum02 = _mm256_add_epi32 ( vsum02, vtmp );
          
            vtmp   = _mm256_madd_epi16( vit3,   vsrc );
            vsum03 = _mm256_add_epi32 ( vsum03, vtmp );
     
            vsrc  = vsrcarr[1][k >> 4];
          
            vtmp   = _mm256_madd_epi16( vit0,   vsrc );
            vsum10 = _mm256_add_epi32 ( vsum10, vtmp );
          
            vtmp   = _mm256_madd_epi16( vit1,   vsrc );
            vsum11 = _mm256_add_epi32 ( vsum11, vtmp );
          
            vtmp   = _mm256_madd_epi16( vit2,   vsrc );
            vsum12 = _mm256_add_epi32 ( vsum12, vtmp );
          
            vtmp   = _mm256_madd_epi16( vit3,   vsrc );
            vsum13 = _mm256_add_epi32 ( vsum13, vtmp );
          }

          vsum00 = _mm256_hadd_epi32( vsum00, vsum01 );
          vsum02 = _mm256_hadd_epi32( vsum02, vsum03 );

          vsum00 = _mm256_hadd_epi32( vsum00, vsum02 );

          __m128i xsum00 = _mm_add_epi32( _mm256_castsi256_si128( vsum00 ), _mm256_extracti128_si256( vsum00, 1 ) );
          xsum00 = _mm_add_epi32 ( xsum00, _mm_set1_epi32( rnd_factor ) );
          xsum00 = _mm_srai_epi32( xsum00, shift );
        
          vsum10 = _mm256_hadd_epi32( vsum10, vsum11 );
          vsum12 = _mm256_hadd_epi32( vsum12, vsum13 );

          vsum10 = _mm256_hadd_epi32( vsum10, vsum12 );
          
          __m128i xsum10 = _mm_add_epi32( _mm256_castsi256_si128( vsum10 ), _mm256_extracti128_si256( vsum10, 1 ) );
          xsum10 = _mm_add_epi32 ( xsum10, _mm_set1_epi32( rnd_factor ) );
          xsum10 = _mm_srai_epi32( xsum10, shift );

          __m128i xtmp = _mm_unpacklo_epi32( xsum00, xsum10 );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp );

          dstPtr += line;

          xtmp = _mm_shuffle_epi32( xtmp, ( 2 << 0 ) + ( 3 << 2 ) );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp );

          dstPtr += line;
          
          xtmp = _mm_unpackhi_epi32( xsum00, xsum10 );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp );

          dstPtr += line;

          xtmp = _mm_shuffle_epi32( xtmp, ( 2 << 0 ) + ( 3 << 2 ) );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp );

          dstPtr += line;

          itPtr  += ( trSize << 2 );
        }

        src += ( trSize << 1 );
      }
    }
    else
#endif
    {
      for( int i = 0; i < reducedLine; i += 2 )
      {
              TCoeff*       dstPtr = dst + i;
        const TMatrixCoeff* itPtr  = tc;
        
#if USE_AVX2
        __m128i vsrcarr[2][1];
#else
        __m128i vsrcarr[2][8];
#endif
          
        for( int k = 0; k < trSize; k += 8 )
        {
          __m128i vsrc0 = _mm_load_si128( ( const __m128i* ) &src[k + 0] );
          __m128i vsrc1 = _mm_load_si128( ( const __m128i* ) &src[k + 4] );
          __m128i vsrc  = _mm_packs_epi32( vsrc0, vsrc1 );

          vsrcarr[0][k >> 3] = vsrc;
          
          vsrc0 = _mm_load_si128( ( const __m128i* ) &src[k + 0 + trSize] );
          vsrc1 = _mm_load_si128( ( const __m128i* ) &src[k + 4 + trSize] );
          vsrc  = _mm_packs_epi32( vsrc0, vsrc1 );

          vsrcarr[1][k >> 3] = vsrc;
        }

        for( int j = 0; j < cutoff; j += 4 )
        {
          __m128i vsum00 = _mm_setzero_si128();
          __m128i vsum01 = _mm_setzero_si128();
          __m128i vsum02 = _mm_setzero_si128();
          __m128i vsum03 = _mm_setzero_si128();
        
          __m128i vsum10 = _mm_setzero_si128();
          __m128i vsum11 = _mm_setzero_si128();
          __m128i vsum12 = _mm_setzero_si128();
          __m128i vsum13 = _mm_setzero_si128();

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
            __m128i vsrc  = vsrcarr[0][k >> 3];

            __m128i
            vtmp   = _mm_madd_epi16( vit0,   vsrc );
            vsum00 = _mm_add_epi32 ( vsum00, vtmp );
          
            vtmp   = _mm_madd_epi16( vit1,   vsrc );
            vsum01 = _mm_add_epi32 ( vsum01, vtmp );
          
            vtmp   = _mm_madd_epi16( vit2,   vsrc );
            vsum02 = _mm_add_epi32 ( vsum02, vtmp );
          
            vtmp   = _mm_madd_epi16( vit3,   vsrc );
            vsum03 = _mm_add_epi32 ( vsum03, vtmp );
          
            // second source line
            vsrc   = vsrcarr[1][k >> 3];
          
            vtmp   = _mm_madd_epi16( vit0,   vsrc );
            vsum10 = _mm_add_epi32 ( vsum10, vtmp );
          
            vtmp   = _mm_madd_epi16( vit1,   vsrc );
            vsum11 = _mm_add_epi32 ( vsum11, vtmp );
          
            vtmp   = _mm_madd_epi16( vit2,   vsrc );
            vsum12 = _mm_add_epi32 ( vsum12, vtmp );
          
            vtmp   = _mm_madd_epi16( vit3,   vsrc );
            vsum13 = _mm_add_epi32 ( vsum13, vtmp );
          }

          vsum00 = _mm_hadd_epi32( vsum00, vsum01 );
          vsum02 = _mm_hadd_epi32( vsum02, vsum03 );

          vsum00 = _mm_hadd_epi32( vsum00, vsum02 );
          vsum00 = _mm_add_epi32 ( vsum00, _mm_set1_epi32( rnd_factor ) );
          vsum00 = _mm_srai_epi32( vsum00, shift );
        
          vsum10 = _mm_hadd_epi32( vsum10, vsum11 );
          vsum12 = _mm_hadd_epi32( vsum12, vsum13 );

          vsum10 = _mm_hadd_epi32( vsum10, vsum12 );
          vsum10 = _mm_add_epi32 ( vsum10, _mm_set1_epi32( rnd_factor ) );
          vsum10 = _mm_srai_epi32( vsum10, shift );

          __m128i xtmp = _mm_unpacklo_epi32( vsum00, vsum10 );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp );

          dstPtr += line;

          xtmp = _mm_shuffle_epi32( xtmp, ( 2 << 0 ) + ( 3 << 2 ) );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp );

          dstPtr += line;
          
          xtmp = _mm_unpackhi_epi32( vsum00, vsum10 );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp );

          dstPtr += line;

          xtmp = _mm_shuffle_epi32( xtmp, ( 2 << 0 ) + ( 3 << 2 ) );
          _mm_storel_epi64( ( __m128i* ) dstPtr, xtmp );

          dstPtr += line;

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
  fastInvCore4 = fastInv_SSE  <vext, 4>;
  fastInvCore8 = fastInv_SSE  <vext, 8>;
  fastFwdCore4_2D
               = fastFwd_SSE  <vext, 4>;
  fastFwdCore8_2D
               = fastFwd_SSE  <vext, 8>;
}

template void TCoeffOps::_initTCoeffOpsX86<SIMDX86>();

}

#endif // TARGET_SIMD_X86
#endif
//! \}

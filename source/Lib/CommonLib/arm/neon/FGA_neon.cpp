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
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/** \file     FGA_neon.cpp
    \brief    film grain analysis, Neon version
*/

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "EncoderLib/SEIFilmGrainAnalyzer.h"
#include "sum_neon.h"

#include <arm_neon.h>

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_FGA

namespace vvenc
{

// Mirrors calcMeanSse: returns the integer sum of all w*h pixels (w is a
// multiple of 8). The caller divides by the block size.
static int calcMeanNeon( const Pel* org, const ptrdiff_t origStride, const int w, const int h )
{
  // Two independent accumulators allow out-of-order execution to overlap
  // the add chains. vpadalq_s16 folds the pairwise widen and accumulate
  // into a single instruction.
  int32x4_t acc0 = vdupq_n_s32( 0 );
  int32x4_t acc1 = vdupq_n_s32( 0 );
  for( int y = 0; y < h; y++, org += origStride )
  {
    int x = 0;
    for( ; x < w - 8; x += 16 )
    {
      acc0 = vpadalq_s16( acc0, vld1q_s16( org + x ) );
      acc1 = vpadalq_s16( acc1, vld1q_s16( org + x + 8 ) );
    }
    for( ; x < w; x += 8 )
      acc0 = vpadalq_s16( acc0, vld1q_s16( org + x ) );
  }
  return horizontal_add_s32x4( vaddq_s32( acc0, acc1 ) );
}

#if ENABLE_SIMD_OPT_MCTF
// Defined in MCTF_neon.cpp and reused here, mirroring how the x86 build
// shares calcVarSse between MCTF and the film grain analyzer.
double calcVar_neon( const Pel* org, const ptrdiff_t origStride, const int w, const int h );
#endif

// Morphological dilation: a pixel becomes Value if any of its 3x3 neighbours
// (including itself) equals Value, otherwise it keeps its original value.
// buff carries the (border-extended) input, Wbuf receives the output; they
// never alias at real call sites. The two buffers generally have different
// strides (buff has a margin, Wbuf usually has none), so they are indexed
// with their own strides.
static int dilation_neon( PelStorage* buff, PelStorage* Wbuf, uint32_t bitDepth, ComponentID compID, int numIter,
                          int iter, Pel Value )
{
  CHECKD( buff == Wbuf, "buff and Wbuf must not alias" );

  // width doesn't change across iterations, so a width<8 buffer stays that
  // way for the whole call; let the scalar reference handle it rather than
  // duplicating its 3x3 test here.
  if( buff->get( compID ).width < 8 )
  {
    return dilation_core( buff, Wbuf, bitDepth, compID, numIter, iter, Value );
  }

  for( ; iter != numIter; ++iter )
  {
    Wbuf->bufs[0].copyFrom( buff->get( compID ) );
    buff->get( compID ).extendBorderPel( 1, 1 );

    const int       width  = buff->get( compID ).width;
    const int       height = buff->get( compID ).height;
    const Pel*      src    = buff->get( compID ).buf;
    const ptrdiff_t sStr   = buff->get( compID ).stride;
    Pel*            dst    = Wbuf->bufs[0].buf;
    const ptrdiff_t dStr   = Wbuf->bufs[0].stride;

    const int16x8_t vVal = vdupq_n_s16( Value );

    auto rowMask = [vVal]( const Pel* row ) -> uint16x8_t {
      int16x8_t  l = vld1q_s16( row - 1 );
      int16x8_t  c = vld1q_s16( row );
      int16x8_t  r = vld1q_s16( row + 1 );
      uint16x8_t m = vceqq_s16( c, vVal );
      m            = vorrq_u16( m, vceqq_s16( l, vVal ) );
      m            = vorrq_u16( m, vceqq_s16( r, vVal ) );
      return m;
    };

    for( int y = 0; y < height; y++ )
    {
      const Pel* srcRow = src + (ptrdiff_t)y * sStr;
      Pel*       dstRow = dst + (ptrdiff_t)y * dStr;

      for( int x = 0; x < width; x += 8 )
      {
        if( x + 8 > width ) x = width - 8;   // overlapping tail strip; safe, dst != src within an iteration
        const Pel* p = srcRow + x;

        // The centre row's vector is needed both for the compare and as
        // the blend source, so compute its mask inline instead of going
        // through rowMask() a second time for the same load.
        int16x8_t  cMid = vld1q_s16( p );
        uint16x8_t mMid = vorrq_u16( vceqq_s16( cMid, vVal ),
                                     vorrq_u16( vceqq_s16( vld1q_s16( p - 1 ), vVal ),
                                                vceqq_s16( vld1q_s16( p + 1 ), vVal ) ) );
        uint16x8_t strong = vorrq_u16( rowMask( p - sStr ), vorrq_u16( mMid, rowMask( p + sStr ) ) );
        vst1q_s16( dstRow + x, vbslq_s16( strong, vVal, cMid ) );
      }
    }

    buff->get( compID ).copyFrom( Wbuf->bufs[0] );
  }
  return iter;
}

template<>
void FGAnalyzer::_initFGAnalyzerARM<NEON>()
{
#if ENABLE_SIMD_OPT_MCTF
  calcVar  = calcVar_neon;
#endif
  calcMean = calcMeanNeon;
}

template<>
void Morph::_initFGAMorphARM<NEON>()
{
  dilation = dilation_neon;
}

}  // namespace vvenc

#endif  // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_FGA

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
/** \file     TrQuant_EMT.h
    \brief    transform and quantization class (header)
*/

#pragma once

#include "CommonDef.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_TRAFO
using namespace x86_simd;
#endif
#if defined(TARGET_SIMD_ARM)  && ENABLE_SIMD_TRAFO
using namespace arm_simd;
#endif

#if ENABLE_SIMD_TRAFO
struct TCoeffOps
{
  TCoeffOps();
  void initTCoeffOps( bool enableOpt = true );

  void( *cpyResi8 )         ( const TCoeff*      src,        Pel*    dst, ptrdiff_t stride, unsigned width, unsigned height );
  void( *cpyResi4 )         ( const TCoeff*      src,        Pel*    dst, ptrdiff_t stride, unsigned width, unsigned height );
  void( *cpyCoeff8 )        ( const Pel*         src, ptrdiff_t stride,   TCoeff* dst, unsigned width, unsigned height );
  void( *cpyCoeff4 )        ( const Pel*         src, ptrdiff_t stride,   TCoeff* dst, unsigned width, unsigned height );
  void( *fastInvCore[5] )   ( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned lines, unsigned reducedLines, unsigned rows );
  void( *fastFwdCore_2D[5] )( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned lines, unsigned reducedLines, unsigned cutoff, int shift );
  void( *fastFwdCore_1D[5] )( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned lines, unsigned reducedLines, unsigned cutoff, int shift );
  void( *roundClip4 )       (                                             TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift );
  void( *roundClip8 )       (                                             TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift );
  // 1D transforms called directly by fastForward/InverseDCT2_B*: partial butterflies (fastInvDCT2, fastFwdDCT2[0..1])
  // and the whole forward DCT-II 16/32/64 (fastFwdDCT2[2..4])
  void( *fastInvDCT2[3] )   ( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum ); // [0]: 2-pt, [1]: 4-pt, [2]: 8-pt
  // [0]: 2-pt, [1]: 4-pt (partial butterflies), [2]: 16-pt, [3]: 32-pt, [4]: 64-pt (by default fastForwardDCT2_MM)
  void( *fastFwdDCT2[5] )   ( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 );
  void( *fastFwdDST7_4 )    ( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 );
  void( *fastFwdDCT8_4 )    ( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 );

private:
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_TRAFO
  void initTCoeffOpsX86();
  template<X86_VEXT vext>
  void _initTCoeffOpsX86();
#endif
#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_TRAFO
  void initTCoeffOpsARM();
  template<ARM_VEXT vext>
  void _initTCoeffOpsARM();
#endif
};

extern TCoeffOps g_tCoeffOps;

#endif

// Forward DCT-II of size N (16, 32 or 64) as a matrix multiply (through g_tCoeffOps.fastFwdCore_* with SIMD): the
// default of TCoeffOps::fastFwdDCT2[2..4] and the fallback of their SIMD versions.
template<int N>
void fastForwardDCT2_MM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2 );

// Zeroes what a forward 1D transform leaves out: columns reducedLine .. line - 1 of the first cutoff rows, and all
// rows from cutoff on (dst holds the transform's rows of line coefficients each).
static inline void fastForwardZeroSkipped( TCoeff *dst, int line, int reducedLine, int cutoff, int iSkipLine, int iSkipLine2 )
{
  if( iSkipLine )
  {
    TCoeff *pCoef = dst + reducedLine;
    for( int j = 0; j < cutoff; j++ )
    {
      memset( pCoef, 0, sizeof( TCoeff ) * iSkipLine );
      pCoef += line;
    }
  }

  if( iSkipLine2 )
  {
    memset( dst + line * cutoff, 0, sizeof( TCoeff ) * line * iSkipLine2 );
  }
}

////DCT-II transforms
void fastForwardDCT2_B2  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B2  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B4  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B4  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B8  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B8  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B16 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B16 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B32 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B32 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B64 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B64 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);

//DST-VII transforms (EMT)
void fastForwardDST7_B4  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B4  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B8  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B8  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B16 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B16 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B32 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B32 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);

//DCT-VIII transforms (EMT)
void fastForwardDCT8_B4  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B4  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B8  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B8  (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B16 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B16 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B32 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B32 (const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum);

} // namespace vvenc

//! \}


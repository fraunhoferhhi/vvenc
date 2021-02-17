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
/** \file     TrQuant_EMT.h
    \brief    transform and quantization class (header)
*/

#pragma once

#include "CommonDef.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_TRAFO
struct TCoeffOps
{
  TCoeffOps();

  void initTCoeffOpsX86();
  template<X86_VEXT vext>
  void _initTCoeffOpsX86();

  void( *cpyResi8 )       ( const TCoeff*      src,        Pel*    dst, ptrdiff_t stride, unsigned width, unsigned height );
  void( *cpyResi4 )       ( const TCoeff*      src,        Pel*    dst, ptrdiff_t stride, unsigned width, unsigned height );
  void( *cpyCoeff8 )      ( const Pel*         src, ptrdiff_t stride,   TCoeff* dst, unsigned width, unsigned height );
  void( *cpyCoeff4 )      ( const Pel*         src, ptrdiff_t stride,   TCoeff* dst, unsigned width, unsigned height );
  void( *fastInvCore4 )   ( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned lines, unsigned reducedLines, unsigned rows );
  void( *fastInvCore8 )   ( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned lines, unsigned reducedLines, unsigned rows );
  void( *fastFwdCore4_2D )( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned lines, unsigned reducedLines, unsigned cutoff, int shift );
  void( *fastFwdCore8_2D )( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned lines, unsigned reducedLines, unsigned cutoff, int shift );
  void( *fastFwdCore4_1D )( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned lines, unsigned reducedLines, unsigned cutoff, int shift );
  void( *fastFwdCore8_1D )( const TMatrixCoeff* it,  const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned lines, unsigned reducedLines, unsigned cutoff, int shift );
  void( *roundClip4 )  (                                             TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift );
  void( *roundClip8 )  (                                             TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift );
};

extern TCoeffOps g_tCoeffOps;

#endif

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


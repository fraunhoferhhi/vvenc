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

/** \file     bench_trafo.cpp
    \brief    benchmarks of the TCoeffOps transform kernels
*/

#include "vvenc_simd_bench.h"

#include "CommonLib/TrQuant_EMT.h"

#include <memory>

using namespace vvenc;
using namespace vvenc_bench;

#if ENABLE_SIMD_TRAFO

namespace
{

typedef void InvTrFn( const TCoeff*, TCoeff*, int, int, int, int, const TCoeff, const TCoeff );
typedef void FwdTrFn( const TCoeff*, TCoeff*, int, int, int, int );
typedef void FwdCoreFn( const TMatrixCoeff*, const TCoeff*, TCoeff*, unsigned, unsigned, unsigned, int );

// the benchmark closures keep the buffers alive
typedef std::shared_ptr<std::vector<TCoeff>> CoeffVec;

CoeffVec makeCoeffs( size_t size, int minVal, int maxVal )
{
  auto v = std::make_shared<std::vector<TCoeff>>( size );
  fill_random( *v, minVal, maxVal );
  return v;
}

void addInverse( const std::string& name, InvTrFn* ref, InvTrFn* opt, int trSize, int line )
{
  const CoeffVec src = makeCoeffs( trSize * line, -2000, 2000 );
  const CoeffVec dst = makeCoeffs( trSize * line, 0, 0 );
  auto bench = [=]( InvTrFn* fn ) -> BenchFn
  {
    return [=]( int iters ) { for( int i = 0; i < iters; i++ ) fn( src->data(), dst->data(), 7, line, 0, 0, -32768, 32767 ); };
  };
  add( "TCoeffOps", name + " line=" + std::to_string( line ), bench( ref ), bench( opt ) );
}

void addForward( const std::string& name, FwdTrFn* ref, FwdTrFn* opt, int trSize, int line )
{
  const CoeffVec src = makeCoeffs( trSize * line, -512, 511 );
  const CoeffVec dst = makeCoeffs( trSize * line, 0, 0 );
  auto bench = [=]( FwdTrFn* fn ) -> BenchFn
  {
    return [=]( int iters ) { for( int i = 0; i < iters; i++ ) fn( src->data(), dst->data(), 2, line, 0, 0 ); };
  };
  add( "TCoeffOps", name + " line=" + std::to_string( line ), bench( ref ), bench( opt ) );
}

void addForwardCore1D( FwdCoreFn* ref, FwdCoreFn* opt, int trSize )
{
  auto tc = std::make_shared<std::vector<TMatrixCoeff>>( trSize * trSize );
  fill_random( *tc, -90, 90 );
  const CoeffVec src = makeCoeffs( trSize, -512, 511 );
  const CoeffVec dst = makeCoeffs( trSize, 0, 0 );
  auto bench = [=]( FwdCoreFn* fn ) -> BenchFn
  {
    return [=]( int iters ) { for( int i = 0; i < iters; i++ ) fn( tc->data(), src->data(), dst->data(), 1, 1, trSize, 5 ); };
  };
  add( "TCoeffOps", "fastFwdCore_1D " + std::to_string( trSize ) + "-pt", bench( ref ), bench( opt ) );
}

void registerTrafo()
{
  static TCoeffOps ref, opt;
  ref.initTCoeffOps( false );
  opt.initTCoeffOps( true );

  for( int line : { 4, 16 } )
  {
    addInverse( "fastInvDCT2 2-pt", ref.fastInvDCT2[0], opt.fastInvDCT2[0], 2, line );
    addInverse( "fastInvDCT2 4-pt", ref.fastInvDCT2[1], opt.fastInvDCT2[1], 4, line );
    addInverse( "fastInvDCT2 8-pt", ref.fastInvDCT2[2], opt.fastInvDCT2[2], 8, line );
    addForward( "fastFwdDCT2 2-pt", ref.fastFwdDCT2[0], opt.fastFwdDCT2[0], 2, line );
    addForward( "fastFwdDCT2 4-pt", ref.fastFwdDCT2[1], opt.fastFwdDCT2[1], 4, line );
    addForward( "fastFwdDST7 4-pt", ref.fastFwdDST7_4,  opt.fastFwdDST7_4,  4, line );
    addForward( "fastFwdDCT8 4-pt", ref.fastFwdDCT8_4,  opt.fastFwdDCT8_4,  4, line );
  }

  for( int log2Size = 2; log2Size <= 6; log2Size++ )
  {
    addForwardCore1D( ref.fastFwdCore_1D[log2Size - 2], opt.fastFwdCore_1D[log2Size - 2], 1 << log2Size );
  }
}

const int registered = addRegistrar( registerTrafo );

} // namespace

#endif

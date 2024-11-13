/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

/**
  \ingroup vvenclibtest
  \file    vvenclibtest.cpp
  \brief   This vvenclibtest.cpp file contains the main entry point of the test application.
*/

#include <iostream>
#include <stddef.h>
#include <stdio.h>
#include <vector>

#include "CommonLib/TrQuant_EMT.h"
#include "CommonLib/TypeDef.h"

using namespace vvenc;

#define NUM_CASES 100

static bool compare_values_2d( const std::string& context, const TCoeff* ref, const TCoeff* opt, unsigned rows,
                               unsigned cols, unsigned stride = 0 )
{
  stride = stride != 0 ? stride : cols;

  for( unsigned row = 0; row < rows; ++row )
  {
    for( unsigned col = 0; col < cols; ++col )
    {
      unsigned idx = row * stride + col;
      if( ref[ idx ] != opt[ idx ] )
      {
        printf( "failed: %s\n", context.c_str() );
        printf( "  mismatch:  ref[%u*%u+%u]=%d  opt[%u*%u+%u]=%d\n", row, cols, col, ref[ idx ], row, cols, col,
                opt[ idx ] );
        return false;
      }
    }
  }
  return true;
}

template<typename T>
class InputGenerator
{
public:
  explicit InputGenerator( unsigned bits ) : m_bits( bits )
  {
  }

  T operator()() const
  {
    return rand() & ( ( 1 << m_bits ) - 1 );
  }

private:
  unsigned m_bits;
};

class DimensionGenerator
{
public:
  unsigned get( unsigned min, unsigned max, unsigned mod = 1 ) const
  {
    unsigned ret = rand() % ( max - min + 1 ) + min;
    ret -= ret % mod;
    return ret;
  }
};

template<typename G>
static bool check_one_fastInvCore( TCoeffOps* ref, TCoeffOps* opt, int idx, int trSize, unsigned lines,
                                   unsigned reducedLines, unsigned rows, G input_generator )
{
  CHECK( lines == 0, "Lines must be non-zero" );
  CHECK( reducedLines > lines, "ReducedLines must be less than or equal to lines" );
  CHECK( rows == 0, "Rows must be non-zero" );
  CHECK( rows % 4, "Rows must be a multiple of four" );

  std::ostringstream sstm;
  sstm << "fastInvCore trSize=" << trSize << " lines=" << lines << " reducedLines=" << reducedLines << " rows=" << rows;

  std::vector<TMatrixCoeff> it( rows * trSize );
  std::vector<TCoeff> src( rows * lines );
  std::vector<TCoeff> dst0( lines * trSize );
  std::vector<TCoeff> dst1 = dst0;

  // Initialize source buffers.
  std::generate( it.begin(), it.end(), input_generator );
  std::generate( src.begin(), src.end(), input_generator );

  ref->fastInvCore[ idx ]( it.data(), src.data(), dst0.data(), lines, reducedLines, rows );
  opt->fastInvCore[ idx ]( it.data(), src.data(), dst1.data(), lines, reducedLines, rows );
  return compare_values_2d( sstm.str(), dst0.data(), dst1.data(), reducedLines, trSize );
}

template<typename G>
static bool check_one_fastFwdCore_2D( TCoeffOps* ref, TCoeffOps* opt, int idx, int trSize, unsigned line,
                                      unsigned reducedLine, unsigned cutoff, unsigned shift, G input_generator )
{
  CHECK( line == 0, "Line must be non-zero" );
  CHECK( reducedLine > line, "ReducedLine must be less than or equal to line" );
  CHECK( cutoff == 0, "Cutoff must be non-zero" );
  CHECK( cutoff % 4, "Cutoff must be a multiple of four" );
  CHECK( shift == 0, "Shift must be at least one" );

  std::ostringstream sstm;
  sstm << "fastFwdCore_2D trSize=" << trSize << " line=" << line << " reducedLine=" << reducedLine
       << " cutoff=" << cutoff << " shift=" << shift;

  std::vector<TMatrixCoeff> tc( trSize * cutoff );
  std::vector<TCoeff> src( trSize * reducedLine );
  std::vector<TCoeff> dst0( line * cutoff );
  std::vector<TCoeff> dst1 = dst0;

  // Initialize source and destination buffers, make sure that destination
  // buffers match in elements that are not written to by the kernel being
  // tested.
  std::generate( tc.begin(), tc.end(), input_generator );
  std::generate( src.begin(), src.end(), input_generator );

  ref->fastFwdCore_2D[ idx ]( tc.data(), src.data(), dst0.data(), line, reducedLine, cutoff, shift );
  opt->fastFwdCore_2D[ idx ]( tc.data(), src.data(), dst1.data(), line, reducedLine, cutoff, shift );
  // Don't check for over-writes past reducedLine columns here, since the
  // existing x86 implementations would fail.
  return compare_values_2d( sstm.str(), dst0.data(), dst1.data(), cutoff, reducedLine, line );
}

static bool check_fastInvCore( TCoeffOps* ref, TCoeffOps* opt, unsigned num_cases, int idx, int trSize )
{
  printf( "Testing TCoeffOps::fastInvCore trSize=%d\n", trSize );
  InputGenerator<TCoeff> g{ 10 };
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    // Clamp lines down to the next multiple of four when generating
    // reducedLines to avoid existing x86 implementations over-writing.
    unsigned lines        = rng.get( 4, 1024 );
    unsigned reducedLines = rng.get( 4, lines & ~3U );
    unsigned rows         = rng.get( 4, 128, 4 );  // Rows must be a non-zero multiple of four.
    if( !check_one_fastInvCore( ref, opt, idx, trSize, lines, reducedLines, rows, g ) )
    {
      return false;
    }
  }

  return true;
}

static bool check_fastFwdCore_2D( TCoeffOps* ref, TCoeffOps* opt, unsigned num_cases, int idx, int trSize )
{
  printf( "Testing TCoeffOps::fastFwdCore_2D trSize=%d\n", trSize );
  InputGenerator<TCoeff> g{ 10 };
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    // Clamp line down to the next multiple of four when generating reducedLine
    // to avoid existing x86 implementations over-writing.
    unsigned line        = rng.get( 1, 1024 );
    unsigned reducedLine = rng.get( 0, line & ~3U );
    unsigned cutoff      = rng.get( 4, 1024, 4 );  // Cutoff must be a non-zero multiple of four.
    unsigned shift       = rng.get( 1, 16 );       // Shift must be at least one to avoid UB.
    if( !check_one_fastFwdCore_2D( ref, opt, idx, trSize, line, reducedLine, cutoff, shift, g ) )
    {
      return false;
    }
  }

  return true;
}

static bool test_TCoeffOps()
{
  TCoeffOps ref;
  TCoeffOps opt;
#if defined( TARGET_SIMD_X86 )
  opt.initTCoeffOpsX86();
#endif
#if defined( TARGET_SIMD_ARM )
  opt.initTCoeffOpsARM();
#endif

  unsigned num_cases = NUM_CASES;
  bool passed        = true;

  passed = check_fastInvCore( &ref, &opt, num_cases, 0, 4 ) && passed;
  passed = check_fastInvCore( &ref, &opt, num_cases, 1, 8 ) && passed;
  passed = check_fastInvCore( &ref, &opt, num_cases, 2, 16 ) && passed;
  passed = check_fastInvCore( &ref, &opt, num_cases, 3, 32 ) && passed;
  passed = check_fastInvCore( &ref, &opt, num_cases, 4, 64 ) && passed;

  passed = check_fastFwdCore_2D( &ref, &opt, num_cases, 0, 4 ) && passed;
  passed = check_fastFwdCore_2D( &ref, &opt, num_cases, 1, 8 ) && passed;
  passed = check_fastFwdCore_2D( &ref, &opt, num_cases, 2, 16 ) && passed;
  passed = check_fastFwdCore_2D( &ref, &opt, num_cases, 3, 32 ) && passed;
  passed = check_fastFwdCore_2D( &ref, &opt, num_cases, 4, 64 ) && passed;
  return passed;
}

int main()
{
  bool passed = test_TCoeffOps();

  if( !passed )
  {
    printf( "\nerror: some tests failed!\n\n" );
    exit( EXIT_FAILURE );
  }
  printf( "\nsuccess: all tests passed!\n\n" );
}

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
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <time.h>

#include "CommonLib/InterPrediction.h"
#include "CommonLib/MCTF.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/TrQuant_EMT.h"
#include "CommonLib/TypeDef.h"

using namespace vvenc;

#define NUM_CASES 100

template<typename T>
static inline bool compare_value( const std::string& context, const T ref, const T opt )
{
  if( opt != ref )
  {
    std::cerr << "failed: " << context << "\n"
              << "  mismatch:  ref=" << ref << "  opt=" << opt << "\n";
  }
  return opt == ref;
}

template<typename T>
static inline bool compare_values_2d( const std::string& context, const T* ref, const T* opt, unsigned rows,
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
  explicit InputGenerator( unsigned bits, bool is_signed = true ) : m_bits( bits ), m_signed( is_signed )
  {
  }

  T operator()() const
  {
    if( !m_signed )
    {
      return static_cast<T>( rand() & ( ( 1 << m_bits ) - 1 ) );
    }
    else
    {
      return ( rand() & ( ( 1 << m_bits ) - 1 ) ) - ( 1 << m_bits >> 1 );
    }
  }

private:
  unsigned m_bits;
  bool m_signed;
};

template<typename T>
class TrafoGenerator
{
public:
  explicit TrafoGenerator( unsigned bits ) : m_bits( bits )
  {
  }

  T operator()() const
  {
    return ( rand() & ( ( 1 << m_bits ) - 1 ) ) - ( 1 << m_bits >> 1 );
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

#if ENABLE_SIMD_TRAFO
template<typename G, typename T>
static bool check_one_fastInvCore( TCoeffOps* ref, TCoeffOps* opt, unsigned idx, unsigned trSize, unsigned lines,
                                   unsigned reducedLines, unsigned cutoff, G input_generator, T trafo_generator )
{
  CHECK( lines == 0, "Lines must be non-zero" );
  CHECK( reducedLines > lines, "ReducedLines must be less than or equal to lines" );
  CHECK( cutoff == 0, "Cutoff must be non-zero" );
  CHECK( cutoff % 4, "Cutoff must be a multiple of four" );
  CHECK( cutoff > trSize, "Cutoff must not be larger than transformation size" );

  std::ostringstream sstm;
  sstm << "fastInvCore trSize=" << trSize << " lines=" << lines << " reducedLines=" << reducedLines << " cutoff=" << cutoff;

  TMatrixCoeff *it   = ( TMatrixCoeff* ) xMalloc( TMatrixCoeff, trSize * trSize );
  TCoeff       *src  = ( TCoeff* )       xMalloc( TCoeff,       trSize * lines );
  TCoeff       *dst0 = ( TCoeff* )       xMalloc( TCoeff,       trSize * lines );
  TCoeff       *dst1 = ( TCoeff* )       xMalloc( TCoeff,       trSize * lines );

  // Initialize source buffers.
  std::generate( it, it + trSize * trSize, trafo_generator );
  std::generate( src, src + trSize * lines, input_generator );
  memset( dst0, 0, trSize * lines * sizeof( TCoeff ) );
  memset( dst1, 0, trSize * lines * sizeof( TCoeff ) );

  ref->fastInvCore[ idx ]( it, src, dst0, lines, reducedLines, cutoff );
  opt->fastInvCore[ idx ]( it, src, dst1, lines, reducedLines, cutoff );

  auto ret = compare_values_2d( sstm.str(), dst0, dst1, trSize, lines );

  xFree( it );
  xFree( src );
  xFree( dst0 );
  xFree( dst1 );

  return ret;
}

template<typename G, typename T>
static bool check_one_fastFwdCore_2D( TCoeffOps* ref, TCoeffOps* opt, unsigned idx, unsigned trSize, unsigned line,
                                      unsigned reducedLine, unsigned cutoff, unsigned shift, G input_generator, T trafo_generator )
{
  CHECK( line == 0, "Line must be non-zero" );
  CHECK( reducedLine > line, "ReducedLine must be less than or equal to line" );
  CHECK( cutoff == 0, "Cutoff must be non-zero" );
  CHECK( cutoff % 4, "Cutoff must be a multiple of four" );
  CHECK( cutoff > trSize, "Cutoff must not be larger than transformation size" );
  CHECK( shift == 0, "Shift must be at least one" );

  std::ostringstream sstm;
  sstm << "fastFwdCore_2D trSize=" << trSize << " line=" << line << " reducedLine=" << reducedLine
       << " cutoff=" << cutoff << " shift=" << shift;

  TMatrixCoeff *tc   = ( TMatrixCoeff* ) xMalloc( TMatrixCoeff, trSize * trSize );
  TCoeff       *src  = ( TCoeff* )       xMalloc( TCoeff,       trSize * line );
  TCoeff       *dst0 = ( TCoeff* )       xMalloc( TCoeff,       trSize * line );
  TCoeff       *dst1 = ( TCoeff* )       xMalloc( TCoeff,       trSize * line );

  // Initialize source and destination buffers, make sure that destination
  // buffers match in elements that are not written to by the kernel being
  // tested.
  std::generate( tc,  tc + trSize * trSize, trafo_generator );
  std::generate( src, src + trSize * line,  input_generator );
  memset( dst0, 0, trSize * line * sizeof( TCoeff ) );
  memset( dst1, 0, trSize * line * sizeof( TCoeff ) );

  ref->fastFwdCore_2D[ idx ]( tc, src, dst0, line, reducedLine, cutoff, shift );
  opt->fastFwdCore_2D[ idx ]( tc, src, dst1, line, reducedLine, cutoff, shift );

  // Don't check for over-writes past reducedLine columns here, since the
  // existing x86 implementations would fail.
  auto ret = compare_values_2d( sstm.str(), dst0, dst1, trSize, reducedLine, line );

  xFree( tc );
  xFree( src );
  xFree( dst0 );
  xFree( dst1 );

  return ret;
}

static bool check_fastInvCore( TCoeffOps* ref, TCoeffOps* opt, unsigned num_cases, unsigned idx, unsigned trSize )
{
  printf( "Testing TCoeffOps::fastInvCore trSize=%d\n", trSize );
  InputGenerator<TCoeff> g{ 16 };
  TrafoGenerator<TMatrixCoeff> t{ 8 };
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    // Clamp lines down to the next multiple of four when generating
    // reducedLines to avoid existing x86 implementations over-writing.
    unsigned lines        = 1 << rng.get( 1, 6 );
    unsigned reducedLines = lines == 2 ? lines : std::min( 32u, rng.get( 4, lines, 4 ) );
    unsigned cutoff       = rng.get( 4, trSize, 4 );  // Cutoff must be a non-zero multiple of four.
    if( !check_one_fastInvCore( ref, opt, idx, trSize, lines, reducedLines, cutoff, g, t ) )
    {
      return false;
    }
  }

  return true;
}

static bool check_fastFwdCore_2D( TCoeffOps* ref, TCoeffOps* opt, unsigned num_cases, unsigned idx, unsigned trSize )
{
  printf( "Testing TCoeffOps::fastFwdCore_2D trSize=%d\n", trSize );
  InputGenerator<TCoeff> g{ 11 }; // signed 10 bit in both positive/negative, i.e. 11 bit shifted to signed
  TrafoGenerator<TMatrixCoeff> t{ 8 };
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    // Clamp line down to the next multiple of four when generating reducedLine
    // to avoid existing x86 implementations over-writing.
    unsigned line        = 1 << rng.get( 1, 6 );
    unsigned reducedLine = line == 2 ? 2 : std::min( 32u, rng.get( 4, line, 4 ) );
    unsigned cutoff      = rng.get( 4, trSize, 4 );  // Cutoff must be a non-zero multiple of four.
    unsigned shift       = rng.get( 1, 16 );          // Shift must be at least one to avoid UB.
    if( !check_one_fastFwdCore_2D( ref, opt, idx, trSize, line, reducedLine, cutoff, shift, g, t ) )
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
#endif

#if ENABLE_SIMD_OPT_MCTF
template<typename G>
static bool check_one_motionErrorLumaInt8( MCTF* ref, MCTF* opt, unsigned orgStride, unsigned bufStride, unsigned w,
                                           unsigned h, unsigned besterror, G input_generator )
{
  CHECK( orgStride < w, "OrgStride must be greater than or equal to width" );
  CHECK( bufStride < w, "BufStride must be greater than or equal to width" );
  CHECK( w % 8, "Width must be a multiple of eight" );
  CHECK( h % 8, "Height must be a multiple of eight" );

  std::ostringstream sstm;
  sstm << "motionErrorLumaInt8 orgStride=" << orgStride << " bufStride=" << bufStride << " w=" << w << " h=" << h
       << " besterror=" << besterror;

  std::vector<Pel> org( orgStride * h );
  std::vector<Pel> buf( bufStride * h );

  // Initialize source buffers.
  std::generate( org.begin(), org.end(), input_generator );
  std::generate( buf.begin(), buf.end(), input_generator );

  int error_ref = ref->m_motionErrorLumaInt8( org.data(), orgStride, buf.data(), bufStride, w, h, besterror );
  int error_opt = opt->m_motionErrorLumaInt8( org.data(), orgStride, buf.data(), bufStride, w, h, besterror );
  return compare_value( sstm.str(), error_ref, error_opt );
}

static bool check_motionErrorLumaInt8( MCTF* ref, MCTF* opt, unsigned num_cases, int w, int h )
{
  printf( "Testing MCTF::motionErrorLumaInt8 w=%d h=%d\n", w, h );
  InputGenerator<TCoeff> g{ 10 };
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    unsigned orgStride = rng.get( w, 128 );
    unsigned bufStride = rng.get( w, 128 );
    unsigned besterror = INT_MAX;
    if( !check_one_motionErrorLumaInt8( ref, opt, orgStride, bufStride, w, h, besterror, g ) )
    {
      return false;
    }
  }

  return true;
}

static bool test_MCTF()
{
  MCTF ref{ /*enableOpt=*/false };
  MCTF opt{ /*enableOpt=*/true };

  unsigned num_cases = NUM_CASES;
  bool passed        = true;

  for( unsigned w = 8; w <= 64; w += 8 )
  {
    for( unsigned h = 8; h <= 64; h += 8 )
    {
      passed = check_motionErrorLumaInt8( &ref, &opt, num_cases, w, h ) && passed;
    }
  }
  return passed;
}
#endif

#if ENABLE_SIMD_OPT_BDOF
template<typename G>
static bool check_one_biDirOptFlow( InterPredInterpolation* ref, InterPredInterpolation* opt, int width, int height,
                                    ptrdiff_t dstStride, int shift, int offset, int limit, ClpRng clpRng, int bitDepth,
                                    G input_generator )
{
  CHECK( width % 8, "Width must be a multiple of eight" );
  CHECK( height % 8, "Height must be a multiple of eight" );

  std::ostringstream sstm;
  sstm << "biDirOptFlow width=" << width << " height=" << height << " shift=" << shift << " offset=" << offset
       << " limit=" << limit;

  int srcStride = width + 2 * BDOF_EXTEND_SIZE + 2;
  int gradStride = width + 2;

  std::vector<Pel> srcY0( srcStride * ( height + 2 ) );
  std::vector<Pel> srcY1( srcStride * ( height + 2 ) );
  std::vector<Pel> gradX0( gradStride * ( height + 2 ) );
  std::vector<Pel> gradX1( gradStride * ( height + 2 ) );
  std::vector<Pel> gradY0( gradStride * ( height + 2 ) );
  std::vector<Pel> gradY1( gradStride * ( height + 2 ) );
  std::vector<Pel> dstYref( dstStride * height );
  std::vector<Pel> dstYopt( dstStride * height );

  // Initialize source buffers.
  std::generate( srcY0.begin(), srcY0.end(), input_generator );
  std::generate( srcY1.begin(), srcY1.end(), input_generator );
  std::generate( gradX0.begin(), gradX0.end(), input_generator );
  std::generate( gradX1.begin(), gradX1.end(), input_generator );
  std::generate( gradY0.begin(), gradY0.end(), input_generator );
  std::generate( gradY1.begin(), gradY1.end(), input_generator );

  ref->xFpBiDirOptFlow( srcY0.data(), srcY1.data(), gradX0.data(), gradX1.data(), gradY0.data(), gradY1.data(), width,
                        height, dstYref.data(), dstStride, shift, offset, limit, clpRng, bitDepth );
  opt->xFpBiDirOptFlow( srcY0.data(), srcY1.data(), gradX0.data(), gradX1.data(), gradY0.data(), gradY1.data(), width,
                        height, dstYopt.data(), dstStride, shift, offset, limit, clpRng, bitDepth );
  return compare_values_2d( sstm.str(), dstYref.data(), dstYopt.data(), height, dstStride );
}

static bool check_biDirOptFlow( InterPredInterpolation* ref, InterPredInterpolation* opt, unsigned num_cases, int width,
                                int height )
{
  printf( "Testing InterPred::xFpBiDirOptFlow w=%d h=%d\n", width, height );
  InputGenerator<Pel> g{ 10 };
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    // Width is either 8 or 16.
    // DstStride is a multiple of eight in the range width to 128 inclusive.
    unsigned dstStride = rng.get( width, 128, 8 );

    for( int bitDepth = 8; bitDepth <= 10; bitDepth += 2 )
    {
      const unsigned shift = IF_INTERNAL_PREC + 1 - bitDepth;
      const int offset = ( 1 << ( shift - 1 ) ) + 2 * IF_INTERNAL_OFFS;
      const int limit = ( 1 << 4 ) - 1;
      ClpRng clpRng{ bitDepth };

      if( !check_one_biDirOptFlow( ref, opt, width, height, dstStride, shift, offset, limit, clpRng, bitDepth, g ) )
      {
        return false;
      }
    }
  }

  return true;
}

static bool test_InterPred()
{
  InterPredInterpolation ref;
  InterPredInterpolation opt;

  ref.init( /*enableOpt=*/false );
  opt.init( /*enableOpt=*/true );

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  for( unsigned width = 8; width <= 16; width += 8 )
  {
    for( unsigned height = 8; height <= 16; height += 8 )
    {
      passed = check_biDirOptFlow( &ref, &opt, num_cases, width, height ) && passed;
    }
  }
  return passed;
}
#endif

#if ENABLE_SIMD_OPT_DIST
static bool check_lumaWeightedSSE( RdCost* ref, RdCost* opt, unsigned num_cases, int width, int height )
{
  printf( "Testing RdCost::lumaWeightedSSE %dx%d\n", width, height );

  std::ostringstream sstm;
  sstm << "lumaWeightedSSE" << " w=" << width << " h=" << height;

  InputGenerator<Pel> g14{ 14 };
  InputGenerator<Pel> g10{ 10, /*is_signed=*/false }; // Index range : 0 - 1023.
  InputGenerator<uint32_t> g17{ 17, /*is_signed=*/false };
  DimensionGenerator rng;

  bool passed = true;
  for( unsigned i = 0; i < num_cases; i++ )
  {
    int org_stride = rng.get( width, 1024 );
    int cur_stride = rng.get( width, 1024 );
    int luma_stride = rng.get( width, 1024 );
    std::vector<Pel> orgBuf( org_stride * height );
    std::vector<Pel> curBuf( cur_stride * height );
    std::vector<Pel> orgLumaBuf( luma_stride * height * 2 );
    std::vector<uint32_t> lumaWeights( 1024 );

    DistParam dtParam;
    dtParam.org.buf = orgBuf.data();
    dtParam.cur.buf = curBuf.data();
    dtParam.org.width = width;
    dtParam.org.height = height;
    dtParam.cur.stride = cur_stride;
    dtParam.org.stride = org_stride;
    CPelBuf pelBuf;
    pelBuf.buf = orgLumaBuf.data();
    pelBuf.stride = luma_stride;
    dtParam.orgLuma = &pelBuf;
    dtParam.bitDepth = 10;
    dtParam.compID = COMP_Y;

    std::generate( orgBuf.begin(), orgBuf.end(), g14 );
    std::generate( curBuf.begin(), curBuf.end(), g14 );
    std::generate( orgLumaBuf.begin(), orgLumaBuf.end(), g10 );
    std::generate( lumaWeights.begin(), lumaWeights.end(), g17 );

    for( unsigned csx = 0; csx < 2; csx++ )
    {
      Distortion sum_ref = ref->m_wtdPredPtr[csx]( dtParam, VVENC_CHROMA_420, lumaWeights.data() );
      Distortion sum_opt = opt->m_wtdPredPtr[csx]( dtParam, VVENC_CHROMA_420, lumaWeights.data() );
      passed = compare_value( sstm.str(), sum_ref, sum_opt ) && passed;
    }
  }

  return passed;
}

static bool check_fixWeightedSSE( RdCost* ref, RdCost* opt, unsigned num_cases, int width, int height )
{
  printf( "Testing RdCost::fixWeightedSSE %dx%d\n", width, height );

  std::ostringstream sstm;
  sstm << "fixWeightedSSE" << " w=" << width << " h=" << height;

  DimensionGenerator rng;
  InputGenerator<Pel> g14{ 14 };
  InputGenerator<uint32_t> g17{ 17, /*is_signed=*/false };

  bool passed = true;
  for( unsigned i = 0; i < num_cases; i++ )
  {
    int org_stride = rng.get( width, 1024 );
    int cur_stride = rng.get( width, 1024 );
    std::vector<Pel> orgBuf( org_stride * height );
    std::vector<Pel> curBuf( cur_stride * height );

    DistParam dtParam;
    dtParam.org.buf = orgBuf.data();
    dtParam.cur.buf = curBuf.data();
    dtParam.org.width = width;
    dtParam.org.height = height;
    dtParam.cur.stride = cur_stride;
    dtParam.org.stride = org_stride;
    dtParam.bitDepth = 10;

    std::generate( orgBuf.begin(), orgBuf.end(), g14 );
    std::generate( curBuf.begin(), curBuf.end(), g14 );
    uint32_t fixedPTweight = g17();

    Distortion sum_ref = ref->m_fxdWtdPredPtr( dtParam, fixedPTweight );
    Distortion sum_opt = opt->m_fxdWtdPredPtr( dtParam, fixedPTweight );
    passed = compare_value( sstm.str(), sum_ref, sum_opt ) && passed;
  }
  return passed;
}

static bool test_RdCost()
{
  RdCost ref;
  RdCost opt;
  ref.create( /*enableOpt=*/false );
  opt.create( /*enableOpt=*/true );

  unsigned num_cases = NUM_CASES;
  bool passed = true;
  std::array<int, 8> widths = { 1, 2, 4, 8, 16, 32, 64, 128 };
  std::array<int, 7> heights = { 2, 4, 8, 16, 32, 64, 128 };

  for( int h : heights )
  {
    for( int w : widths )
    {
      passed = check_lumaWeightedSSE( &ref, &opt, num_cases, w, h ) && passed;
    }
  }
  for( int h : heights )
  {
    for( int w : widths )
    {
      passed = check_fixWeightedSSE( &ref, &opt, num_cases, w, h ) && passed;
    }
  }
  return passed;
}
#endif // ENABLE_SIMD_OPT_DIST

int main( int argc, char** argv )
{
  unsigned seed = ( unsigned ) time( NULL );
  srand( seed );

  bool passed = true;

#if ENABLE_SIMD_TRAFO
  passed = test_TCoeffOps() && passed;
#endif
#if ENABLE_SIMD_OPT_MCTF
  passed = test_MCTF() && passed;
#endif
#if ENABLE_SIMD_OPT_BDOF
  passed = test_InterPred() && passed;
#endif
#if ENABLE_SIMD_OPT_DIST
  passed = test_RdCost() && passed;
#endif

  if( !passed )
  {
    printf( "\nerror: some tests failed for seed=%u!\n\n", seed );
    exit( EXIT_FAILURE );
  }
  printf( "\nsuccess: all tests passed!\n\n" );
}

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

#include "CommonLib/AffineGradientSearch.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/MCTF.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/TrQuant_EMT.h"
#include "CommonLib/TypeDef.h"
#include "CommonLib/Unit.h"


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
static inline bool compare_values_1d( const std::string& context, const T* ref, const T* opt, unsigned length )
{
  for( unsigned idx = 0; idx < length; ++idx )
  {
    if( ref[idx] != opt[idx] )
    {
      std::cout << "failed: " << context << "\n"
                << "  mismatch:  ref[" << idx << "]=" << ref[idx] << "  opt[" << idx << "]=" << opt[idx] << "\n";
      return false;
    }
  }
  return true;
}

template<typename T, typename U = T>
static inline bool compare_values_2d( const std::string& context, const T* ref, const T* opt, unsigned rows,
                                      unsigned cols, unsigned stride = 0, U tolerance = U( 0 ) )
{
  stride = stride != 0 ? stride : cols;

  auto abs_diff = []( T value1, T value2 ) -> U { return static_cast<U>( std::abs( value1 - value2 ) ); };

  for( unsigned row = 0; row < rows; ++row )
  {
    for( unsigned col = 0; col < cols; ++col )
    {
      unsigned idx = row * stride + col;
      if( abs_diff( ref[idx], opt[idx] ) > tolerance )
      {
        std::cout << "failed: " << context << "\n"
                  << "  mismatch:  ref[" << row << "*" << stride << "+" << col << "]=" << ref[idx]
                  << "  opt[" << row << "*" << stride << "+" << col << "]=" << opt[idx] << "\n";
        return false;
      }
    }
  }
  return true;
}

template<typename T>
class MinMaxGenerator
{
public:
  explicit MinMaxGenerator( unsigned bits, bool is_signed = true )
  {
    if( is_signed )
    {
      T half = 1 << ( bits - 1 );
      m_min = -half;
      m_max =  half - 1;
    }
    else
    {
      m_min = 0;
      m_max = ( 1 << bits ) - 1;
    }
  }

  T operator()() const
  {
    return ( rand() & 1 ) ? m_max : m_min;
  }

  std::string input_type() const
  {
    return "MinOrMax";
  }

private:
  T m_min;
  T m_max;
};

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

  std::string input_type() const
  {
    return "Rand";
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

#if ENABLE_SIMD_OPT_INTRAPRED
static bool check_IntraPredAngleLuma( IntraPrediction* ref, IntraPrediction* opt, unsigned num_cases )
{
  static constexpr unsigned int bd = 10; // default bit-depth = 10
  ClpRng clpRng{ bd };
  DimensionGenerator dim;
  InputGenerator<Pel> ref_gen{ bd, /*is_signed=*/false }; // unsigned 10-bit

  ptrdiff_t dstStride = MAX_CU_SIZE;
  static constexpr size_t refMain_size = 2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX;
  static constexpr size_t dstBuf_size = MAX_CU_SIZE * MAX_CU_SIZE;
  std::vector<Pel> refMain( refMain_size );
  std::vector<Pel> dstBuf_ref( dstBuf_size );
  std::vector<Pel> dstBuf_opt( dstBuf_size );

  bool passed = true;

  for( bool useCubic : { true, false } )
  {
    std::ostringstream sstm_test;
    sstm_test << "IntraPrediction::IntraPredAngleLuma" << " useCubic=" << std::boolalpha << useCubic;
    std::cout << "Testing " << sstm_test.str() << '\n';

    for( unsigned n = 0; n < num_cases; n++ )
    {
      int log2width = dim.get( 2, 6 );
      int log2height = dim.get( 2, 6 );
      int width = 1 << log2width;   // min: 4, max: 64
      int height = 1 << log2height; // min: 4, max: 64
      int deltaPos = dim.get( 16, 128 );
      int intraPredAngle = deltaPos;

      std::generate( refMain.begin(), refMain.end(), ref_gen );
      std::fill( dstBuf_ref.begin(), dstBuf_ref.end(), Pel{} );
      std::fill( dstBuf_opt.begin(), dstBuf_opt.end(), Pel{} );

      ref->IntraPredAngleLuma( dstBuf_ref.data(), dstStride, refMain.data(), width, height, deltaPos, intraPredAngle,
                               nullptr, useCubic, clpRng );
      opt->IntraPredAngleLuma( dstBuf_opt.data(), dstStride, refMain.data(), width, height, deltaPos, intraPredAngle,
                               nullptr, useCubic, clpRng );

      std::ostringstream sstm_subtest;
      sstm_subtest << sstm_test.str() << " width=" << width << " height=" << height << " deltaPos=" << deltaPos
                   << " intraPredAngle=" << intraPredAngle;

      passed = compare_values_1d( sstm_subtest.str(), dstBuf_ref.data(), dstBuf_opt.data(), dstBuf_size ) && passed;
    }
  }

  return passed;
}

static bool test_IntraPred()
{
  IntraPrediction ref{ /*enableOpt=*/false };
  IntraPrediction opt{ /*enableOpt=*/true };

  unsigned num_cases = NUM_CASES;
  bool passed        = true;

  passed = check_IntraPredAngleLuma( &ref, &opt, num_cases ) && passed;

  return passed;
}
#endif // ENABLE_SIMD_OPT_INTRAPRED

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

#define VVENC_MCTF_RANGE 6
template<typename G>
static bool check_one_applyBlock( MCTF* ref, MCTF* opt, unsigned srcStride, unsigned dstStride, int w, int h,
                                  int bitDepth, int numRefs, G inputGenCorrectedPics )
{
  CHECK( srcStride < w, "OrgStride must be greater than or equal to width" );
  CHECK( dstStride < w, "BufStride must be greater than or equal to width" );

  std::ostringstream sstm;
  sstm << "applyBlock srcStride=" << srcStride << " dstStride=" << dstStride << " w=" << w << " h=" << h;

  InputGenerator<TCoeff> g10{ 10, /*is_signed=*/false };
  std::vector<int> verror( 2 * VVENC_MCTF_RANGE ); // 10bit unsigned
  std::generate( verror.begin(), verror.end(), g10 );

  const double refStrengths[2][VVENC_MCTF_RANGE] = {
      // abs(POC offset)
      // 1       2       3       4       5       6
      { 0.84375, 0.6, 0.4286, 0.3333, 0.2727, 0.2308 }, // RA
      { 1.12500, 1.0, 0.7143, 0.5556, 0.4545, 0.3846 }  // LD
  };
  std::vector<double> refStr( 2 * VVENC_MCTF_RANGE );
  std::copy( &refStrengths[0][0], &refStrengths[0][0] + 2 * VVENC_MCTF_RANGE, refStr.begin() );

  DimensionGenerator dg;
  ChromaFormat chromaFormat = dg.get( 0, 1 ) ? VVENC_CHROMA_400 : VVENC_CHROMA_420;
  ComponentID compID = ( ComponentID )dg.get( 0, 2 ); // 0 to 2

  const CompArea blk( compID, chromaFormat, Area( 0, 0, w, h ) );
  const ClpRng clpRng{ bitDepth };
  const std::array<double, 3> overallStrength = { 0.5, 0.666667, 1.5 }; // Values taken from a real encoding.
  const double weightScaling = overallStrength[dg.get( 0, 2 )] * ( isChroma( compID ) ? 0.55 : 0.4 );
  const std::array<double, 3> sigmaSqVal = { 900.0, 2275.031250, 4608.0 }; // Values taken from a real encoding.
  double sigmaSq = sigmaSqVal[dg.get( 0, 2 )];

  std::vector<const Pel*> correctedPics( 2 * VVENC_MCTF_RANGE );
  std::vector<Pel> correctedPicsBuf( numRefs * w * h );
  std::generate( correctedPicsBuf.begin(), correctedPicsBuf.end(), inputGenCorrectedPics );
  for( int i = 0; i < numRefs; i++ )
  {
    correctedPics[i] = correctedPicsBuf.data() + ( i * w * h );
  }

  std::vector<Pel> src_buf( srcStride * h );
  std::generate( src_buf.begin(), src_buf.end(), g10 );

  std::vector<Pel> dst_buf_ref( dstStride * h );
  std::vector<Pel> dst_buf_opt( dstStride * h );

  CPelBuf src;
  PelBuf dst_ref, dst_opt;

  src.buf = src_buf.data();
  dst_ref.buf = dst_buf_ref.data();
  dst_opt.buf = dst_buf_opt.data();
  src.stride = srcStride;
  dst_ref.stride = dstStride;
  dst_opt.stride = dstStride;

  ref->m_applyBlock( src, dst_ref, blk, clpRng, correctedPics.data(), numRefs, verror.data(), refStr.data(),
                     weightScaling, sigmaSq );
  opt->m_applyBlock( src, dst_opt, blk, clpRng, correctedPics.data(), numRefs, verror.data(), refStr.data(),
                     weightScaling, sigmaSq );

  // The SIMDe implementation of applyBlock may differ by one bit compared to the reference implementation.
  // Adjusted tolerance to reflect this.
  return compare_values_2d( sstm.str(), dst_buf_ref.data(), dst_buf_opt.data(), h, w, dstStride, 1 );
}

static bool check_applyBlock( MCTF* ref, MCTF* opt, unsigned num_cases, int w, int h )
{
  printf( "Testing MCTF::applyBlock w=%d h=%d\n", w, h );
  InputGenerator<TCoeff> g10{ 10, /*is_signed=*/false };
  InputGenerator<TCoeff> g2{ 2, /*is_signed=*/false };
  DimensionGenerator rng;

  for( int bitDepth : { 8, 10 } )
  {
    for( int numRefs : { 6, 8 } )
    {
      for( unsigned i = 0; i < num_cases; ++i )
      {
        unsigned srcStride = rng.get( w, 128 );
        unsigned dstStride = rng.get( w, 128 );

        if( !check_one_applyBlock( ref, opt, srcStride, dstStride, w, h, bitDepth, numRefs, g10 ) )
        {
          return false;
        }
      }
      // Test scenarios with high noise (as corner case) - dst buffer having high variance from src buffer.
      unsigned srcStride = rng.get( w, 128 );
      unsigned dstStride = rng.get( w, 128 );

      if( !check_one_applyBlock( ref, opt, srcStride, dstStride, w, h, bitDepth, numRefs, g2 ) )
      {
        return false;
      }
    }
  }

  return true;
}

template<typename G>
static bool check_one_applyPlanarCorrection( MCTF* ref, MCTF* opt, unsigned orgStride, unsigned dstStride, int size,
                                             int bitDepth, uint16_t motionerror, G input_generator )
{
  CHECK( orgStride < size, "OrgStride must be greater than or equal to width" );
  CHECK( dstStride < size, "DstStride must be greater than or equal to width" );

  std::ostringstream sstm;
  sstm << "applyPlanarCorrection orgStride=" << orgStride << " dstStride=" << dstStride << " w=" << size
       << " h=" << size << " motionerror=" << motionerror;

  std::vector<Pel> org( orgStride * size );
  std::vector<Pel> dst_ref( dstStride * size );
  std::vector<Pel> dst_opt( dstStride * size );

  // Initialize source buffers.
  std::generate( org.begin(), org.end(), input_generator );
  std::generate( dst_ref.begin(), dst_ref.end(), input_generator );
  dst_opt = dst_ref;

  const ClpRng clpRng{ bitDepth };
  ref->m_applyPlanarCorrection( org.data(), orgStride, dst_ref.data(), dstStride, size, size, clpRng, motionerror );
  opt->m_applyPlanarCorrection( org.data(), orgStride, dst_opt.data(), dstStride, size, size, clpRng, motionerror );
  return compare_values_2d( sstm.str(), dst_ref.data(), dst_opt.data(), size, size, dstStride );
}

static bool check_applyPlanarCorrection( MCTF* ref, MCTF* opt, unsigned num_cases, int size )
{
  printf( "Testing MCTF::applyPlanarCorrection w=%d h=%d\n", size, size );
  InputGenerator<TCoeff> g{ 10, /*is_signed=*/false };
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    for( int bitDepth = 8; bitDepth <= 10; bitDepth += 2 )
    {
      unsigned orgStride = rng.get( size, 128 );
      unsigned dstStride = rng.get( size, 128 );
      uint16_t motionerror = rng.get( 1, 32 );
      if( !check_one_applyPlanarCorrection( ref, opt, orgStride, dstStride, size, bitDepth, motionerror, g ) )
      {
        return false;
      }
    }
  }

  return true;
}

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
  bool passed = true;

  std::vector<unsigned> sizes = { 4, 8, 16, 24, 32, 40, 48, 56, 64 };
  for( unsigned w : sizes )
  {
    for( unsigned h : sizes )
    {
      passed = check_applyBlock( &ref, &opt, num_cases, w, h ) && passed;
    }
  }

  for( int size = 4; size <= 32; size *= 2 )
  {
    passed = check_applyPlanarCorrection( &ref, &opt, num_cases, size ) && passed;
  }

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
  return compare_values_2d( sstm.str(), dstYref.data(), dstYopt.data(), height, (unsigned) dstStride );
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

#if ENABLE_SIMD_OPT_AFFINE_ME
template<typename G>
static bool check_EqualCoeffComputer( AffineGradientSearch* ref, AffineGradientSearch* opt, unsigned num_cases,
                                      G inp_gen )
{
  DimensionGenerator dim;

  static constexpr size_t buf_size = MAX_CU_SIZE * MAX_CU_SIZE;
  std::vector<Pel> residue( buf_size );
  std::vector<Pel> derivate0( buf_size );
  std::vector<Pel> derivate1( buf_size );
  Pel* pDerivate[2] = { derivate0.data(), derivate1.data() };

  static constexpr size_t coeff_size = 7;
  int64_t i64EqualCoeff_ref[coeff_size][coeff_size];
  int64_t i64EqualCoeff_opt[coeff_size][coeff_size];

  bool passed = true;

  for( int b6Param : { 0, 1 } )
  {
    std::ostringstream sstm_test;
    sstm_test << "AffineGradientSearch::EqualCoeffComputer<" << std::boolalpha << static_cast<bool>( b6Param ) << ">"
              << " Input=" << inp_gen.input_type();
    std::cout << "Testing " << sstm_test.str() << std::endl;

    // Set height and width to powers of two >= 16.
    for( int height : { 16, 32, 64, 128 } )
    {
      for( int width : { 16, 32, 64, 128 } )
      {
        for( unsigned n = 0; n < num_cases; n++ )
        {
          // Set random strides >= width.
          const int residueStride = dim.get( width, MAX_CU_SIZE );
          const int derivateStride = dim.get( width, MAX_CU_SIZE );

          // Fill input buffers with signed 10-bit data from generator.
          std::generate( residue.begin(), residue.end(), inp_gen );
          std::generate( derivate0.begin(), derivate0.end(), inp_gen );
          std::generate( derivate1.begin(), derivate1.end(), inp_gen );

          // Clear output blocks.
          std::memset( i64EqualCoeff_ref, 0, sizeof( i64EqualCoeff_ref ) );
          std::memset( i64EqualCoeff_opt, 0, sizeof( i64EqualCoeff_opt ) );

          ref->m_EqualCoeffComputer[b6Param]( residue.data(), residueStride, pDerivate, derivateStride, width, height,
                                              i64EqualCoeff_ref );
          opt->m_EqualCoeffComputer[b6Param]( residue.data(), residueStride, pDerivate, derivateStride, width, height,
                                              i64EqualCoeff_opt );

          std::ostringstream sstm_subtest;
          sstm_subtest << sstm_test.str() << " residueStride=" << residueStride << " derivateStride=" << derivateStride
                       << " width=" << width << " height=" << height;

          passed = compare_values_2d( sstm_subtest.str(), &i64EqualCoeff_ref[0][0], &i64EqualCoeff_opt[0][0],
                                      coeff_size, coeff_size ) &&
                   passed;
        }
      }
    }
  }

  return passed;
}

static bool test_AffineGradientSearch()
{
  AffineGradientSearch ref{ /*enableOpt=*/false };
  AffineGradientSearch opt{ /*enableOpt=*/true };

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  static constexpr unsigned bd = 10;
  auto random_gen = InputGenerator<Pel>{ bd, /*is_signed=*/true };
  auto minmax_gen = MinMaxGenerator<Pel>{ bd, /*is_signed=*/true };

  passed = check_EqualCoeffComputer( &ref, &opt, num_cases, random_gen ) && passed;
  passed = check_EqualCoeffComputer( &ref, &opt, num_cases, minmax_gen ) && passed;

  return passed;
}
#endif // ENABLE_SIMD_OPT_AFFINE_ME

#ifdef ENABLE_SIMD_OPT_BUFFER
static bool check_addAvg( PelBufferOps* ref, PelBufferOps* opt, unsigned num_cases )
{
  static constexpr unsigned bd = 10;
  ClpRng clpRng{ bd };
  DimensionGenerator dim;
  InputGenerator<Pel> inp_gen{ bd, /*is_signed=*/false }; // unsigned 10-bit

  const unsigned shiftNum = std::max<int>( 2, ( IF_INTERNAL_PREC - bd ) ) + 1;
  const int      offset   = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;

  static constexpr size_t buf_size = MAX_CU_SIZE * MAX_CU_SIZE;

  // Use xMalloc to create aligned buffers.
  Pel *src0     = ( Pel* ) xMalloc( Pel, buf_size );
  Pel *src1     = ( Pel* ) xMalloc( Pel, buf_size );
  Pel *dest_ref = ( Pel* ) xMalloc( Pel, buf_size );
  Pel *dest_opt = ( Pel* ) xMalloc( Pel, buf_size );

  bool passed = true;

  // Test addAvg with no strides.
  for( int size : { 4, 8, 16, 32, 48, 64, 128, 192 } )
  {
    std::ostringstream sstm_test;
    sstm_test << "PelBufferOps::addAvg" << " size=" << size;
    std::cout << "Testing " << sstm_test.str() << std::endl;

    for( unsigned n = 0; n < num_cases; n++ )
    {
      // Fill input buffers with unsigned 10-bit data from generator.
      std::generate( src0, src0 + buf_size, inp_gen );
      std::generate( src1, src1 + buf_size, inp_gen );

      // Clear output blocks.
      memset( dest_ref, 0, buf_size * sizeof( Pel ) );
      memset( dest_opt, 0, buf_size * sizeof( Pel ) );

      ref->addAvg( src0, src1, dest_ref, size, shiftNum, offset, clpRng );
      opt->addAvg( src0, src1, dest_opt, size, shiftNum, offset, clpRng );

      passed = compare_values_1d( sstm_test.str(), dest_ref, dest_opt, buf_size ) && passed;
    }
  }

  // Test addAvg with strides.
  for( int height : { 4, 8, 16, 24, 32, 64 } )
  {
    for( int width : { 4, 8, 12, 16, 20, 24, 32, 40, 48, 64 } )
    {
      std::ostringstream sstm_test;
      sstm_test << "PelBufferOps::addAvg(strided)" << " w=" << width << " h=" << height;
      std::cout << "Testing " << sstm_test.str() << std::endl;

      for( unsigned n = 0; n < num_cases; n++ )
      {
        // Set random strides >= width.
        const int src0Stride = dim.get( width, MAX_CU_SIZE );
        const int src1Stride = dim.get( width, MAX_CU_SIZE );
        const int destStride = dim.get( width, MAX_CU_SIZE );

        // Fill input buffers with unsigned 10-bit data from generator.
        std::generate( src0, src0 + buf_size, inp_gen );
        std::generate( src1, src1 + buf_size, inp_gen );

        // Clear output blocks.
        memset( dest_ref, 0, buf_size * sizeof( Pel ) );
        memset( dest_opt, 0, buf_size * sizeof( Pel ) );

        if( ( width & 15 ) == 0 )
        {
          ref->addAvg16( src0, src0Stride, src1, src1Stride, dest_ref, destStride,
                         width, height, shiftNum, offset, clpRng );
          opt->addAvg16( src0, src0Stride, src1, src1Stride, dest_opt, destStride,
                         width, height, shiftNum, offset, clpRng );
        }
        else if( ( width & 7 ) == 0 )
        {
          ref->addAvg8( src0, src0Stride, src1, src1Stride, dest_ref, destStride,
                        width, height, shiftNum, offset, clpRng );
          opt->addAvg8( src0, src0Stride, src1, src1Stride, dest_opt, destStride,
                        width, height, shiftNum, offset, clpRng );
        }
        else if( ( width & 3 ) == 0 )
        {
          ref->addAvg4( src0, src0Stride, src1, src1Stride, dest_ref, destStride,
                        width, height, shiftNum, offset, clpRng );
          opt->addAvg4( src0, src0Stride, src1, src1Stride, dest_opt, destStride,
                        width, height, shiftNum, offset, clpRng );
        }
        else // Shouldn't come here.
        {
          THROW( "Unsupported size" );
        }

        std::ostringstream sstm_subtest;
        sstm_subtest << sstm_test.str() << " src0Stride=" << src0Stride << " src1Stride=" << src1Stride
                     << " destStride=" << destStride;

        passed = compare_values_2d( sstm_subtest.str(), dest_ref, dest_opt, height, width, destStride ) && passed;
      }
    }
  }

  xFree( src0 );
  xFree( src1 );
  xFree( dest_ref );
  xFree( dest_opt );

  return passed;
}

static bool test_PelBufferOps()
{
  PelBufferOps ref;
  PelBufferOps opt;

#if defined( TARGET_SIMD_X86 )
  opt.initPelBufOpsX86();
#endif
#if defined( TARGET_SIMD_ARM )
  opt.initPelBufOpsARM();
#endif

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  passed = check_addAvg( &ref, &opt, num_cases ) && passed;

  return passed;
}
#endif // ENABLE_SIMD_OPT_BUFFER

#if ENABLE_SIMD_OPT_MCIF
template<unsigned isFirst, unsigned isLast>
static bool check_filterCopy( InterpolationFilter* ref, InterpolationFilter* opt, unsigned num_cases, bool biMCForDMVR )
{
  DimensionGenerator dim;

  static constexpr size_t buf_size = MAX_CU_SIZE * MAX_CU_SIZE;

  std::vector<Pel> src( buf_size );
  std::vector<Pel> dst_ref( buf_size );
  std::vector<Pel> dst_opt( buf_size );

  bool passed = true;

  // Test unsigned 8-bit and 10-bit.
  for( unsigned bd : { 8, 10 } )
  {
    ClpRng clpRng{ ( int )bd };
    InputGenerator<Pel> inp_gen{ bd, /*is_signed=*/false };

    std::ostringstream sstm_test;
    sstm_test << "InterpolationFilter::filterCopy[" << isFirst << "][" << isLast << "]"
              << " biMCForDMVR=" << std::boolalpha << biMCForDMVR << " bitDepth=" << bd;
    std::cout << "Testing " << sstm_test.str() << std::endl;

    for( unsigned n = 0; n < num_cases; n++ )
    {
      unsigned height    = dim.get( 1, MAX_CU_SIZE );
      unsigned width     = dim.get( 1, MAX_CU_SIZE );
      unsigned srcStride = dim.get( width, MAX_CU_SIZE );
      unsigned dstStride = dim.get( width, MAX_CU_SIZE );

      // Fill input buffers with unsigned data.
      std::generate( src.begin(), src.end(), inp_gen );

      // Clear output blocks.
      std::fill( dst_ref.begin(), dst_ref.end(), 0 );
      std::fill( dst_opt.begin(), dst_opt.end(), 0 );

      ref->m_filterCopy[isFirst][isLast]( clpRng, src.data(), ( int )srcStride, dst_ref.data(), ( int )dstStride,
                                          ( int )width, ( int )height, biMCForDMVR );
      opt->m_filterCopy[isFirst][isLast]( clpRng, src.data(), ( int )srcStride, dst_opt.data(), ( int )dstStride,
                                          ( int )width, ( int )height, biMCForDMVR );

      std::ostringstream sstm_subtest;
      sstm_subtest << sstm_test.str() << " srcStride=" << srcStride << " dstStride=" << dstStride << " w=" << width
                   << " h=" << height;

      passed =
          compare_values_2d( sstm_subtest.str(), dst_ref.data(), dst_opt.data(), height, width, dstStride ) && passed;
    }
  }

  return passed;
}

static bool test_InterpolationFilter()
{
  InterpolationFilter ref;
  InterpolationFilter opt;

  ref.initInterpolationFilter( /*enable=*/false );
  opt.initInterpolationFilter( /*enable=*/true );

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  passed = check_filterCopy<0, 0>( &ref, &opt, num_cases, false ) && passed;
  passed = check_filterCopy<0, 1>( &ref, &opt, num_cases, false ) && passed;
  passed = check_filterCopy<1, 0>( &ref, &opt, num_cases, false ) && passed;
  passed = check_filterCopy<1, 1>( &ref, &opt, num_cases, false ) && passed;
  passed = check_filterCopy<1, 0>( &ref, &opt, num_cases, true ) && passed;

  return passed;
}
#endif // ENABLE_SIMD_OPT_MCIF

int main()
{
  unsigned seed = ( unsigned ) time( NULL );
  srand( seed );

  bool passed = true;

#if ENABLE_SIMD_OPT_INTRAPRED
  passed = test_IntraPred() && passed;
#endif
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
#if ENABLE_SIMD_OPT_AFFINE_ME
  passed = test_AffineGradientSearch() && passed;
#endif
#if ENABLE_SIMD_OPT_BUFFER
  passed = test_PelBufferOps() && passed;
#endif
#if ENABLE_SIMD_OPT_MCIF
  passed = test_InterpolationFilter() && passed;
#endif

  if( !passed )
  {
    printf( "\nerror: some tests failed for seed=%u!\n\n", seed );
    exit( EXIT_FAILURE );
  }
  printf( "\nsuccess: all tests passed!\n\n" );
}

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

/** \file     vvenc_simd_bench.cpp
    \brief    micro benchmark: scalar reference vs. SIMD implementation of the kernels with function-pointer dispatch
*/

#include "vvenc_simd_bench.h"

#include "apputils/ParseArg.h"
#include "vvenc/vvenc.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#else
#include <sched.h>
#endif

namespace po = apputils::program_options;

namespace vvenc_bench
{

std::vector<Bench>& registry()
{
  static std::vector<Bench> benches;
  return benches;
}

void add( const char* group, const std::string& name, BenchFn ref, BenchFn opt )
{
  registry().push_back( Bench{ group, name, std::move( ref ), std::move( opt ) } );
}

static std::vector<void ( * )()>& registrars()
{
  static std::vector<void ( * )()> fns;
  return fns;
}

int addRegistrar( void ( *fn )() )
{
  registrars().push_back( fn );
  return 0;
}

// ns per call: the loop count is doubled until one run takes >= minTime, then the best of `reps` runs is taken
static double measure( const BenchFn& fn, double minTimeSec, int reps )
{
  using clk = std::chrono::steady_clock;
  int iters = 1;
  fn( 16 ); // warm up caches
  for( ;; )
  {
    const auto t0 = clk::now();
    fn( iters );
    const double s = std::chrono::duration<double>( clk::now() - t0 ).count();
    if( s >= minTimeSec || iters >= ( 1 << 28 ) )
    {
      break;
    }
    iters *= 2;
  }
  double best = 1e30;
  for( int r = 0; r < reps; r++ )
  {
    const auto t0 = clk::now();
    fn( iters );
    best = std::min( best, std::chrono::duration<double>( clk::now() - t0 ).count() );
  }
  return best * 1e9 / iters;
}

} // namespace vvenc_bench

int main( int argc, char* argv[] )
{
  std::string simd, filter;
  int cpu         = 0;
  double minTime  = 0.02;
  int reps        = 5;
  bool show_help  = false;

  po::Options opts;
  opts.addOptions()
    ( "help,h", show_help, "Show help", true )
    ( "SIMD", simd, "SIMD extension for the optimized kernels (SSE41, SSE42, AVX2)", false )
    ( "filter,f", filter, "only run benchmarks whose group or name contains this text", false )
    ( "cpu", cpu, "logical CPU to pin the benchmark thread to (-1: no pinning)", false )
    ( "mintime", minTime, "minimum time per measurement in seconds", false )
    ( "reps", reps, "repetitions per measurement (best is reported)", false );
  po::SilentReporter err;
  po::scanArgv( opts, argc, ( const char** ) argv, err );
  if( show_help )
  {
    std::ostringstream help_sstm;
    po::doHelp( help_sstm, opts );
    std::cout << help_sstm.str() << "\n";
    return 0;
  }

  const char* ext = vvenc_set_SIMD_extension( simd.c_str() );
  if( !ext )
  {
    std::cout << simd << " is not supported!\n";
    return 1;
  }

  if( cpu >= 0 )
  {
#ifdef _WIN32
    SetThreadAffinityMask( GetCurrentThread(), DWORD_PTR( 1 ) << cpu );
#else
    cpu_set_t set;
    CPU_ZERO( &set );
    CPU_SET( cpu, &set );
    sched_setaffinity( 0, sizeof( set ), &set );
#endif
  }

  for( auto fn : vvenc_bench::registrars() )
  {
    fn();
  }

  std::printf( "SIMD=%s cpu=%d\n", ext, cpu );
  std::printf( "%-14s %-48s %12s %12s %9s\n", "group", "kernel", "scalar ns", "simd ns", "speed-up" );

  int slower = 0;
  for( const auto& b : vvenc_bench::registry() )
  {
    if( !filter.empty() && b.group.find( filter ) == std::string::npos && b.name.find( filter ) == std::string::npos )
    {
      continue;
    }
    const double scalarNs = vvenc_bench::measure( b.ref, minTime, reps );
    const double simdNs   = vvenc_bench::measure( b.opt, minTime, reps );
    std::printf( "%-14s %-48s %12.1f %12.1f %8.2fx\n", b.group.c_str(), b.name.c_str(), scalarNs, simdNs, scalarNs / simdNs );
    std::fflush( stdout );
    slower += simdNs >= scalarNs;
  }
  std::printf( "\n%d kernel(s) not faster than scalar\n", slower );
  return 0;
}

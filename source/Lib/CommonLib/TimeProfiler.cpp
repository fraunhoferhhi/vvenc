/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD

License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     TimeProfiler.cpp
    \brief    profiling of run-time behavior
*/
#include "Rom.h"
#include "TimeProfiler.h"
//#include "StatCounter.h"

// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
// #include <time.h>
// 
// 
// #include <stack>
// #include <array>
// #include <memory>
// #include <chrono>
// #include <numeric>
// #include <ostream>
// #include <sstream>

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if ENABLE_TIME_PROFILING

#if ENABLE_TIME_PROFILING_EXTENDED
static int profilerId = 0;
#endif
TProfiler* timeProfilerCreate( const VVEncCfg& encCfg )
{
  TProfiler* tp = nullptr;
#if !ENABLE_TIME_PROFILING_EXTENDED
  tp = new TProfiler();
#else
#if ENABLE_TIME_PROFILING_PIC_TYPES
  tp = new TProfiler( 3, 1, 1, profilerId );
#elif ENABLE_TIME_PROFILING_TL
  tp = new TProfiler( Log2( encCfg.m_GOPSize ) + 2, 1, 1, profilerId );
#elif ENABLE_TIME_PROFILING_CTUS_IN_PIC
  int   widthInCTU  = ( encCfg.m_PadSourceWidth % encCfg.m_CTUSize )  ? encCfg.m_PadSourceWidth/encCfg.m_CTUSize  + 1 : encCfg.m_PadSourceWidth/encCfg.m_CTUSize;
  int   heightInCTU = ( encCfg.m_PadSourceHeight % encCfg.m_CTUSize ) ? encCfg.m_PadSourceHeight/encCfg.m_CTUSize + 1 : encCfg.m_PadSourceHeight/encCfg.m_CTUSize;
  tp = new TProfiler( widthInCTU, heightInCTU, 2, 1, profilerId );
#elif ENABLE_TIME_PROFILING_CU_SHAPES
  tp = new TProfiler( Log2(encCfg.m_CTUSize) + 1, Log2(encCfg.m_CTUSize) + 1, 2, 1, profilerId );
#endif
  profilerId++;
#endif
  return tp;
}

void timeProfilerResults( TProfiler* tp )
{
#if !ENABLE_TIME_PROFILING_EXTENDED
  if( tp )
  {
    std::cout << *tp;
    delete tp;
    tp = nullptr;
  }
#else
  if( tp )
  {
    std::cout << std::endl;
#if ENABLE_TIME_PROFILING_PIC_TYPES
    std::cout << "Run-time of selected encoder stages across picture types (0:Intra, 1:Inter)" << std::endl;
    for( int j = 0; j < tp->getCountersSet()[0].getNumCntTypes(); j++ )
    {
      tp->getCountersSet()[0][j][0][2] += tp->getCountersSet()[0][j][0][0] + tp->getCountersSet()[0][j][0][1];
    }
    StatCounters::report2D( std::cout, tp->getCountersSet()[0], false, true, false, true, true, -1 );
#elif ENABLE_TIME_PROFILING_TL
    std::cout << "Run-time of selected encoder stages across temporal levels" << std::endl;
    size_t finalCntIdx = tp->getCountersSet()[0].getDimHor() - 1;
    for( size_t i = 0; i < finalCntIdx; i++ )
    {
      for( size_t j = 0; j < tp->getCountersSet()[0].getNumCntTypes(); j++ )
      {
        tp->getCountersSet()[0][j][0][finalCntIdx] += tp->getCountersSet()[0][j][0][i];
      }
    }
    StatCounter2DSet<double> cnt = tp->getCountersSet()[0];
    cnt.getCounters().resize( P_STAGES );
    StatCounters::report2D( std::cout, cnt/*tp->getCountersSet()[0]*/, false, true, false, true, true, -1 );
#elif ENABLE_TIME_PROFILING_CTUS_IN_PIC
    for( int i = 0; i < tp->getCountersSet().size(); i++ )
    {
      std::cout << "Run-time of selected encoder stages across CTUs of all pictures " << "(" << ( i == 0 ? "Intra": "Inter" << ")" ) << std::endl;
      StatCounters::report2D( std::cout, tp->getCountersSet()[i], false, true, false, true, true, -1 );
      if( i > 0 )
        tp->getCountersSet()[0] += tp->getCountersSet()[i];
    }
    if( tp->getCountersSet().size() > 1 )
    {
      std::cout << "Run-time of selected encoder stages across CTUs of all pictures (total)" << std::endl;
      StatCounters::report2D( std::cout, tp->getCountersSet()[0], false, true, false, true, true, -1 );
    }
#elif ENABLE_TIME_PROFILING_CU_SHAPES
    for( int i = 0; i < tp->getCountersSet().size(); i++ )
    {
      std::cout << "Run-time of selected encoder stages across CU block shapes of all pictures " << "(" << ( i == 0 ? "Intra": "Inter" ) << ")"  << std::endl;
      StatCounters::report2D( std::cout, tp->getCountersSet()[i],  true, true, false, true, true, -1 );
      if( i > 0 ) tp->getCountersSet()[0] += tp->getCountersSet()[i];
    }
    if( tp->getCountersSet().size() > 1 )
    {
      std::cout << "Run-time of selected encoder stages across CU block shapes of all pictures (total)" << std::endl;
      StatCounters::report2D( std::cout, tp->getCountersSet()[0],  true, true, false, true, true, -1 );
    }
#endif
    delete tp;
    tp = nullptr;
  }
#endif
}
#endif

} // namespace vvenc

//! \}

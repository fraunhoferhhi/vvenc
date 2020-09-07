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
/** \file     TimeProfiler.h
    \brief    profiling of run-time behavior (header)
*/
#pragma once

#include "CommonDef.h"
#include "StatCounter.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>


#include <stack>
#include <array>
#include <memory>
#include <chrono>
#include <numeric>
#include <ostream>
#include <sstream>

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if ENABLE_TIME_PROFILING || ENABLE_TIME_PROFILING_EXTENDED

// Here, users can add their profiling stages
#define E_TIME_PROF_STAGES(E_) \
  E_( P_PIC_LEVEL               ) \
  E_( P_COMPRESS_SLICE          ) \
  E_( P_COMPRESS_CU             ) \
  E_( P_INTER_MVD_SEARCH        ) \
  E_( P_INTER_MVD_SEARCH_AFFINE ) \
  E_( P_INTER_MVD_SEARCH_IMV    ) \
  E_( P_FRAC_PEL                ) \
  E_( P_QPEL                    ) \
  E_( P_QPEL_INTERP             ) \
  E_( P_HPEL_INTERP             ) \
  E_( P_INTER_MRG               ) \
  E_( P_INTER_MRG_AFFINE        ) \
  E_( P_INTER_MRG_DMVR          ) \
  E_( P_INTRA                   ) \
  E_( P_INTRA_EST_RD_CAND_LUMA  ) \
  E_( P_INTRA_RD_SEARCH_LUMA    ) \
  E_( P_INTRA_CHROMA            ) \
  E_( P_QUANT                   ) \
  E_( P_TRAFO                   ) \
  E_( P_DEBLOCK_FILTER          ) \
  E_( P_ALF                     ) \
  E_( P_ALF_CLASS               ) \
  E_( P_ALF_STATS               ) \
  E_( P_ALF_ENC                 ) \
  E_( P_ALF_MERGE               ) \
  E_( P_ALF_DERIVE_COEF         ) \
  E_( P_ALF_ENC_CTB             ) \
  E_( P_ALF_REC                 ) \
  E_( P_OTHER                   ) \
  E_( P_STAGES                  ) \
  E_( P_VOID = P_STAGES         )
MAKE_ENUM_AND_STRINGS(E_TIME_PROF_STAGES, STAGE, stageNames)

template<class Rep>
std::ostream& operator<<( std::ostream& os, const std::chrono::duration<double, Rep> d )
{
  os << d.count();
  return os;
}

#if ENABLE_TIME_PROFILING
class TimeProfiler
{
public:
  using rep                 = std::milli;
  using clock               = std::chrono::steady_clock;
  using time_point          = std::chrono::time_point<clock>;
  using duration            = std::chrono::duration<double, rep>;

private:
  time_point previous = clock::now();
  STAGE    m_eStage;
  const unsigned m_numStages = P_STAGES + 1;
  int      m_iLevel;
  int      m_iExtData;
  unsigned m_numBlkHor;
  unsigned m_numBlkVer;
  unsigned m_curWId;
  unsigned m_curHId;

public:
  const time_point start_time = previous;
  std::vector<duration> durations;

  TimeProfiler() : m_eStage( P_OTHER )
  {
    init();
  }
  void init() 
  {
    durations.resize( m_numStages );
    for( size_t i = 0; i < m_numStages; ++i ) { durations[i] = durations[i].zero(); }
  }
  TimeProfiler& operator()( STAGE s ) 
  {
    time_point now = clock::now();
    durations[m_eStage] += ( now - previous );
    previous = now;
    m_eStage = s;
    return *this;
  }
  TimeProfiler& operator+=( const TimeProfiler& other ) 
  {
    auto i1 = durations.begin();
    auto i2 = other.durations.cbegin();
    for( ; i1 != durations.end() && i2 != other.durations.cend(); ++i1, ++i2 ) 
    {
      *i1 += *i2;
    }

    return *this;
  }

  void start( STAGE s )
  {
    previous = clock::now();
    m_eStage = s;
  }
  void stop( STAGE s ) 
  {
    time_point now = clock::now();
    durations[m_eStage] += ( now - previous );
  }
  STAGE curStage() { return m_eStage; }


  friend std::ostream& operator<<( std::ostream& os, const TimeProfiler& prof )
  {
    //const TimeProfiler::duration total   = TimeProfiler::clock::now() - prof.start_time;
    const TimeProfiler::duration counted = std::accumulate(prof.durations.begin(), prof.durations.end()-1, TimeProfiler::duration{});
    const double scale = 1.0;
    const int prec = 1;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(prec) << (counted / scale);
    const int ts = 1 + ss.str().size();

    os << '\n';
    os << std::setw(10) << " "
       << std::setw(30) << std::left << "stages" << std::internal
       << std::setw(ts) << "time(ms)"
       << std::setw(10) << "%"
       << '\n';

    for (size_t i=0; i < P_STAGES; ++i) 
    {
        auto v = prof.durations[i];
        if( v.count() != 0.0 )
        {
          os << std::setw( 10 ) << " "
            << std::setw( 30 ) << std::left << stageNames[i] << std::internal
            << std::fixed << std::setw( ts ) << std::setprecision( prec ) << (v / scale)/* * total*/
            << std::fixed << std::setw( 10 ) << std::setprecision( prec ) << (v / counted) * 100.0
            << '\n';
        }
    }
    os << '\n';

    os << std::setw(10) << " " 
       << std::setw(30) << std::left << "TOTAL" << std::internal
       << std::fixed << std::setw(ts) << std::setprecision(prec) << (counted / scale)/*total*/
       << std::fixed << std::setw(10) << std::setprecision(prec) << 100.00
       << '\n';

    return os;
  }

  void output( std::ostream& os )
  {
    os << *this;
  }
};

class StageTimeProfiler 
{
  STAGE m_ePrevStage;
  TimeProfiler* m_profiler;
public:
  StageTimeProfiler( TimeProfiler *pcProfiler, STAGE m_eStage )
  {
    m_profiler = pcProfiler;
    m_ePrevStage = m_profiler->curStage();
    ( *m_profiler )( m_eStage );
  }
  ~StageTimeProfiler()
  {
    ( *m_profiler )( m_ePrevStage );
  }
};
#endif

///////////////////////////////////////////////////////////////////////////////
//////////////////////////    EXTENDED TIME PROFILER  ///////////////////////// 
///////////////////////////////////////////////////////////////////////////////
#if ENABLE_TIME_PROFILING_EXTENDED
using namespace StatCounters;

class TimeProfiler2D
{
public:
  using clock               = std::chrono::steady_clock;
  using time_point          = std::chrono::time_point<clock>;

private:
  time_point m_previous = clock::now();
  const time_point m_startTime = m_previous;
  STAGE    m_stage;
  unsigned m_numX;
  unsigned m_numY;
  unsigned m_numZ;
  unsigned m_curX;
  unsigned m_curY;
  unsigned m_curZ;
  std::vector<StatCounter2DSet<double>> m_counters;

public:
  TimeProfiler2D( unsigned numX = 1, unsigned numY = 1, unsigned numZ = 1 ) : m_stage( P_OTHER ), m_numX( numX ), m_numY( numY ), m_numZ( numZ ), m_curX( 0 ), m_curY( 0 ), m_curZ( 0 )
  {
    m_counters.resize( m_numZ );
    for( int i = 0; i < m_numZ; i++ )
    {
      m_counters[i].init( std::vector<std::string> { stageNames, std::end( stageNames ) }, m_numX, m_numY );
    }
  }
  void count( STAGE s, unsigned x, unsigned y, unsigned z ) 
  {
    time_point now = clock::now();
    m_counters[z][m_stage][y][x] += ( now - m_previous ).count();
    m_previous = now;
    m_stage    = s;
    m_curX     = x;
    m_curY     = y;
    m_curZ     = z;
  }
  void start( STAGE s )
  {
    m_previous = clock::now();
    m_stage = s;
  }
  void stop( STAGE s ) 
  {
    count( m_stage, m_curX, m_curY, m_curZ );
  }
  unsigned  numStages()  { return m_counters[0].getNumCntTypes(); }
  STAGE     curStage()   { return m_stage; }
  unsigned  curX()       { return m_curX; }
  unsigned  curY()       { return m_curY; }
  unsigned  curZ()       { return m_curZ; }

  std::vector<StatCounter2DSet<double>>&       getCountersSet()       { return m_counters; }
  const std::vector<StatCounter2DSet<double>>& getCountersSet() const { return m_counters; }
};

class StageTimeProfiler2D
{
  STAGE m_prevStage;
  int   m_prevX;
  int   m_prevY;
  int   m_prevZ;
  TimeProfiler2D* m_profiler;
public:
  StageTimeProfiler2D( TimeProfiler2D *pcProfiler, STAGE m_eStage, unsigned x, unsigned y, unsigned z ) 
  {
    m_profiler   = pcProfiler;
    m_prevStage  = m_profiler->curStage();
    m_prevX      = m_profiler->curX();
    m_prevY      = m_profiler->curY();
    m_prevZ      = m_profiler->curZ();
    m_profiler->count( m_eStage, x, y, z );
  }
  ~StageTimeProfiler2D()
  {
    m_profiler->count( m_prevStage, m_prevX, m_prevY, m_prevZ );
  }
};
#endif

#define PROFILER_START(p,s)                                     (*(p)).start(s)
#define PROFILER_STOP(p)                                           

#if ENABLE_TIME_PROFILING
#define PROF_SCOPE_AND_STAGE_COND_0(p,s)
#define PROF_SCOPE_AND_STAGE_COND_1(p,s)                        StageTimeProfiler cScopedProfiler##s((p),(s))
#define PROF_SCOPE_AND_STAGE_COND(cond,p,s)                     PROF_SCOPE_AND_STAGE_COND_ ## cond (p,s)
#define PROFILER_SCOPE_AND_STAGE_(cond,p,s)                     PROF_SCOPE_AND_STAGE_COND(cond,p,s)

#elif ENABLE_TIME_PROFILING_EXTENDED
#define PROF_EXT_ACCUM_AND_START_NEW_SET_COND_0(p,s,t)
#define PROF_EXT_ACCUM_AND_START_NEW_SET_COND_1(p,s,t)          (*(p)).count(s,t,0,0)
#define PROF_EXT_ACCUM_AND_START_NEW_SET_COND(cond,p,s,t)       PROF_EXT_ACCUM_AND_START_NEW_SET_COND_ ## cond (p,s,t)
#define PROFILER_EXT_ACCUM_AND_START_NEW_SET(cond,p,s,t)        PROF_EXT_ACCUM_AND_START_NEW_SET_COND(cond,p,s,t)

#define PROF_SCOPE_AND_STAGE_EXT_COND_0(p,s,a,b,c)
#define PROF_SCOPE_AND_STAGE_EXT_COND_1(p,s,a,b,c)              StageTimeProfiler2D cScopedProfilerExt##s((p),(s),(a),(b),(c))
#define PROF_SCOPE_AND_STAGE_EXT_COND(cond,p,s,a,b,c)           PROF_SCOPE_AND_STAGE_EXT_COND_ ## cond (p,s,a,b,c)

#if ENABLE_TIME_PROFILING_PIC_TYPES
#define PROFILER_SCOPE_AND_STAGE_EXT2D_(cond,p,s,t,x,y,w,h)     PROF_SCOPE_AND_STAGE_EXT_COND(cond,p,s,t,0,0)
#elif ENABLE_TIME_PROFILING_CTUS_IN_PIC
#define PROFILER_SCOPE_AND_STAGE_EXT2D_(cond,p,s,t,x,y,w,h)     PROF_SCOPE_AND_STAGE_EXT_COND(cond,p,s,x,y,t)
#elif ENABLE_TIME_PROFILING_CU_SHAPES
#define PROFILER_SCOPE_AND_STAGE_EXT2D_(cond,p,s,t,x,y,w,h)     PROF_SCOPE_AND_STAGE_EXT_COND(cond,p,s,w,h,t)
#else
#define PROFILER_SCOPE_AND_STAGE_EXT2D_(cond,p,s,t,x,y,w,h)      
#endif
#define BX_(cs,ch)  ( ( (cs)->area.block( ComponentID(ch) ).x << getChannelTypeScaleX( ch, (cs)->pcv->chrFormat ) ) >> (cs)->pcv->maxCUSizeLog2 )
#define BY_(cs,ch)  ( ( (cs)->area.block( ComponentID(ch) ).y << getChannelTypeScaleY( ch, (cs)->pcv->chrFormat ) ) >> (cs)->pcv->maxCUSizeLog2 )
#define BW_(cs,ch)  ( Log2( ((cs)->area.block( ComponentID(ch) ).width) ) )
#define BH_(cs,ch)  ( Log2( ((cs)->area.block( ComponentID(ch) ).height) ) )
#define PROFILER_SCOPE_AND_STAGE_EXT2D(cond,p,s,cs,ch)          PROFILER_SCOPE_AND_STAGE_EXT2D_(cond,p,s,!(cs)->slice->isIntra(), BX_(cs,ch), BY_(cs,ch), BW_(cs,ch), BH_(cs,ch) )
#endif
#endif

#if ENABLE_TIME_PROFILING
#define PROFILER_ACCUM_AND_START_NEW_SET(cond,p,s)              (*(p))(s)
#define PROFILER_SCOPE_AND_STAGE(cond,p,s)                      PROFILER_SCOPE_AND_STAGE_(cond,p,s)
#define PROFILER_SCOPE_AND_STAGE_EXT(cond,p,s,cs,ch)            PROFILER_SCOPE_AND_STAGE(cond,p,s)
#elif ENABLE_TIME_PROFILING_EXTENDED
#define PROFILER_ACCUM_AND_START_NEW_SET(cond,p,s)              PROFILER_EXT_ACCUM_AND_START_NEW_SET(cond,p,s,0)
#define PROFILER_SCOPE_AND_STAGE(cond,p,s)
#define PROFILER_SCOPE_AND_STAGE_EXT(cond,p,s,cs,ch)            PROFILER_SCOPE_AND_STAGE_EXT2D(cond,p,s,cs,ch)
#else
#define PROF_START(p,s)
#define PROF_STOP(p)
#define PROFILER_ACCUM_AND_START_NEW_SET(cond,p,s)
#define PROFILER_SCOPE_AND_STAGE(cond,p,s)
#define PROFILER_SCOPE_AND_STAGE_EXT(cond,p,s,cs,ch)
#endif
} // namespace vvenc

//! \}

/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     Stats.h
    \brief    statistics calculator class (header)
*/

#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <numeric>
#include <cmath>

#if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
  #include <io.h>
#elif __linux || __APPLE__
  #include <unistd.h>
#endif

#include "vvenc/vvenc.h"


//! \ingroup Interface
//! \{

namespace apputils {

// ====================================================================================================================

class Stats
{
public:

  class AUStats
  {
    public:
    AUStats()
    {}
    ~AUStats()
    {
      reset();
    }

    void reset( double framerate = 0.0 )
    {
      m_count = 0;
      m_bytes = 0;
      m_framerate = framerate;
      m_qpCur.clear();
      m_qps.clear();
    }

    void addAU( vvencAccessUnit* au )
    {
      m_count++;
      m_bytes += au->payloadUsedSize;

      std::string info(au->infoString);
      if( !info.empty() )
      {
        std::size_t fQP = info.find("QP");
        if (fQP !=std::string::npos)
        {
          std::size_t fQPEnd = info.find(",",fQP+1);
          if (fQPEnd ==std::string::npos)
          {
            fQPEnd = info.find(")",fQP+1);
          }
          std::string qp = info.substr (fQP+3,fQPEnd-fQP-3);
          if ( !qp.empty() )
          {
            m_qpCur.push_back( std::stoi( qp ));
          }
        }
      }
    }

    void calc()
    {
      if( !m_qpCur.empty() )
      {
        // calc. average over cur. period
        m_qps.push_back(std::accumulate( m_qpCur.begin(), m_qpCur.end(), 0.0)/m_qpCur.size());
        m_qpCur.clear();
      }
    }

    uint64_t count() { return m_count; }

    double getBps()
    {
      return (m_bytes ? (m_bytes*8 * m_framerate / (double)m_count ) : NAN);
    }

    double getKbps()
    {
      return (m_bytes ? (getBps() / 1000.0 ) : NAN);
    }

    double getAvgQp()
    {
      return (m_qps.empty()) ?
      (m_qpCur.empty() ? NAN : std::accumulate( m_qpCur.begin(), m_qpCur.end(), 0.0)/m_qpCur.size() ) :
      std::accumulate( m_qps.begin(), m_qps.end(), 0.0)/m_qps.size();
    }

    int getNumDigitsKbps()
    {
      double bitrate = getKbps();
      int numDigitsB = (bitrate > 0.0) ? static_cast<int>(std::log10(std::abs(std::ceil(bitrate)))) + 4 : 4;
      return numDigitsB;
    }

   private:
      uint64_t  m_count   = 0;
      uint64_t  m_bytes   = 0;
      double    m_framerate = 0.0;

      std::vector <double>  m_qps;
      std::vector <int>     m_qpCur;
  };

  Stats()
  {
  }

  ~Stats()
  {
  }

  int init( int framerate, int framescale, int maxFrames,  vvencMsgLevel verbosity, std::string prependString = "" )
  {
    m_framerate   = (framerate/(double)framescale);
    m_maxFrames   = maxFrames;
    m_verbosity   = verbosity;
    m_preString   = prependString;
    m_bytes       = 0;
    m_bytesCur    = 0;
    m_frames      = 0;
    m_framesCur   = 0;
    m_tStart      = std::chrono::steady_clock::now();
    m_tGlobStart  = std::chrono::steady_clock::now();

    m_AUStats[VVENC_I_SLICE].reset(m_framerate);
    m_AUStats[VVENC_P_SLICE].reset(m_framerate);
    m_AUStats[VVENC_B_SLICE].reset(m_framerate);

#if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
    m_istty = _isatty( _fileno(stdout));
#elif __linux || __APPLE__
    m_istty = isatty( fileno(stdout));
#endif

    return 0;
  }

  int addAU( vvencAccessUnit* au, bool* infoReady )
  {
    if( !au ){ return -1; }

    *infoReady = false;

    m_frames++;
    m_framesCur++;
    m_AUStats[(int)au->sliceType].addAU(au);

    if( m_bytes && m_framesCur >= std::ceil(m_framerate) )
    {      
      m_AUStats[VVENC_I_SLICE].calc();
      m_AUStats[VVENC_P_SLICE].calc();
      m_AUStats[VVENC_B_SLICE].calc();
      *infoReady = true;
    }

    m_bytes    += au->payloadUsedSize;
    m_bytesCur += au->payloadUsedSize;
    return 0;
  }

  std::string getInfoString( bool finalInfo = false )
  {
    std::stringstream css;
    m_tEnd     = std::chrono::steady_clock::now();
    m_tGlobEnd = std::chrono::steady_clock::now();

    if( m_bytesCur )
    {
      double dTime      = (double)std::chrono::duration_cast<std::chrono::milliseconds>(m_tEnd-m_tStart).count() / 1000.0;
      double dGlobTime  = (double)std::chrono::duration_cast<std::chrono::milliseconds>(m_tGlobEnd-m_tGlobStart).count() / 1000.0;        
      double bitrateAvg = m_bytes*8 * m_framerate/(double)m_frames / 1000.0;
      double dFpsAvg    = dGlobTime ? (double)m_frames / dGlobTime : 0;
      
      double bitrate    = finalInfo ? bitrateAvg : (m_bytesCur*8 * m_framerate / (double)m_framesCur/ 1000.0 );     
      double dFps       = finalInfo ? dFpsAvg    : (dTime ? (double)m_framesCur / dTime : 0);
      
      if( bitrate > (double)m_maxratekbps )
      {
        m_maxratekbps = static_cast<int>(bitrate);
      }
      int setwBR = std::max( 8,  (int)std::log10(m_maxratekbps) + 4 );

      if ( m_verbosity <= VVENC_INFO && m_istty ) css << "\r";
      css << m_preString << "stats: ";
      if( m_maxFrames > 0 )
      {
        double dDone = m_frames * 100.0 / m_maxFrames;
        int numDigits = m_maxFrames ? static_cast<int>(std::log10(std::abs(m_maxFrames))) + 1 : 1;        
        css << std::fixed << std::setprecision(1) << std::setfill(' ') << std::setw(5) << dDone << "%%";
        css << " frame= " << std::setfill(' ') << std::setw(numDigits) << m_frames << "/" << m_maxFrames;
      }
      else
      {
        css << " frame= " << std::setfill(' ') << std::setw(4) << m_frames;
      }

      css << std::fixed << std::setprecision(1);
      css << " fps= "     << std::setfill(' ') << std::setw(5) << dFps;
      css << " avg_fps= " << std::setfill(' ') << std::setw(5) << dFpsAvg;
      css << std::fixed << std::setprecision(2);
      css << " bitrate= "     << std::setfill(' ') << std::setw(setwBR) << bitrate    << " kbps";
      css << " avg_bitrate= " << std::setfill(' ') << std::setw(setwBR) << bitrateAvg << " kbps";

      int sec   = std::ceil(dGlobTime);
      int days  = sec/86400;
      int hours = sec%86400/3600;
      int min   = sec%3600/60;
      sec=sec%60;

      css << " elapsed= ";
      if( days )
        css << std::setfill('0') << std::setw(2) << days << "d:";
      css << std::setfill('0') << std::setw(2) << hours << "h:";
      css << std::setfill('0') << std::setw(2) << min << "m:";
      css << std::setfill('0') << std::setw(2) << sec << "s";

      if( m_maxFrames > 0 )
      {
        double dRemaining = (dGlobTime * m_maxFrames / m_frames) - dGlobTime;
        sec   = std::ceil(dRemaining);
        days  = sec/86400;
        hours = sec%86400/3600;
        min   = sec%3600/60;
        sec=sec%60;

        css << " left= ";
        if( days )
          css << std::setfill('0') << std::setw(2) << days << "d:";
        css << std::setfill('0') << std::setw(2) << hours << "h:";
        css << std::setfill('0') << std::setw(2) << min << "m:";
        css << std::setfill('0') << std::setw(2) << sec << "s";
      }
      css << std::setprecision(-1) << "    ";

      if ( m_verbosity > VVENC_INFO || 0 == m_istty ) css << std::endl;
    }

    m_bytesCur = 0;
    m_framesCur = 0;
    m_tStart = std::chrono::steady_clock::now();

    return css.str();
  }

  std::string getFinalStats()
  {
    std::stringstream css;
    m_tGlobEnd = std::chrono::steady_clock::now();

    if( m_bytes )
    {
      double bitrate = (m_bytes*8 * m_framerate / (double)m_frames ) / 1000.0;
      double dGlobTime = (double)std::chrono::duration_cast<std::chrono::milliseconds>(m_tGlobEnd-m_tGlobStart).count() / 1000.0;
      double dFpsAvg = dGlobTime ? (double)m_frames / dGlobTime : 0;

      css << getInfoString( true );
      css << std::endl;
      css << m_preString << "stats summary:";
      css << " frame= " << m_frames;
      if( m_maxFrames > 0 )
      {
         css << "/" << m_maxFrames;  
      }
      css << std::fixed << std::setprecision(1) << " avg_fps= " << dFpsAvg;  
      css << std::fixed << std::setprecision(2) << " avg_bitrate= " << bitrate << " kbps";
      css << std::endl;

      int numDigitsF = (m_maxFrames>0) ? static_cast<int>(std::log10(std::abs(m_maxFrames))) + 1 : 4;
      int numDigitsB = m_AUStats[VVENC_I_SLICE].getNumDigitsKbps();
      if( m_AUStats[VVENC_P_SLICE].getNumDigitsKbps() > numDigitsB )
        numDigitsB = m_AUStats[VVENC_P_SLICE].getNumDigitsKbps();
      if( m_AUStats[VVENC_B_SLICE].getNumDigitsKbps() > numDigitsB )
        numDigitsB = m_AUStats[VVENC_B_SLICE].getNumDigitsKbps();

      css << m_preString << "stats summary: frame I: " << std::setfill(' ') << std::setw(numDigitsF) << m_AUStats[VVENC_I_SLICE].count() << 
      ", kbps: " << std::setfill(' ') << std::setw(numDigitsB) << m_AUStats[VVENC_I_SLICE].getKbps() <<
      ", AvgQP: " << m_AUStats[VVENC_I_SLICE].getAvgQp() << std::endl;
      css << m_preString << "stats summary: frame P: " << std::setfill(' ') << std::setw(numDigitsF) << m_AUStats[VVENC_P_SLICE].count() << 
      ", kbps: " << std::setfill(' ') << std::setw(numDigitsB) << m_AUStats[VVENC_P_SLICE].getKbps() <<
      ", AvgQP: " << m_AUStats[VVENC_P_SLICE].getAvgQp() << std::endl;
      css << m_preString << "stats summary: frame B: " << std::setfill(' ') << std::setw(numDigitsF) << m_AUStats[VVENC_B_SLICE].count() << 
      ", kbps: " << std::setfill(' ') << std::setw(numDigitsB) << m_AUStats[VVENC_B_SLICE].getKbps() <<
      ", AvgQP: " << m_AUStats[VVENC_B_SLICE].getAvgQp() << std::endl;

      css << std::setprecision(-1) << std::endl;
    }
    return css.str();
  }

private:

  double      m_framerate  = 1.0;
  int         m_maxFrames  = 0;
  vvencMsgLevel m_verbosity = VVENC_INFO;
  std::string m_preString;
  int         m_istty      = 0;

  uint64_t    m_bytes      = 0;
  uint64_t    m_bytesCur   = 0;
  int         m_maxratekbps = 0;

  int         m_frames     = 0;
  int         m_framesCur  = 0;

  AUStats     m_AUStats[3]; // stats per slice type

  std::chrono::steady_clock::time_point m_tStart;
  std::chrono::steady_clock::time_point m_tEnd;

  std::chrono::steady_clock::time_point m_tGlobStart;
  std::chrono::steady_clock::time_point m_tGlobEnd;
};

} // namespace apputils

//! \}

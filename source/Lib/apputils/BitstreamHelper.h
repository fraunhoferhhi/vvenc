/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     BitstreamHelper.h
    \brief    file I/O helper class (header)
*/

#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>

#include "vvenc/vvenc.h"


//! \ingroup Interface
//! \{

namespace apputils {

// ====================================================================================================================

class BitstreamHelper
{
public:

  BitstreamHelper()
  {
  }

  ~BitstreamHelper()
  {
  }

  int init( int framerate, int maxFrames )
  {
    mFramerate = framerate;
    mMaxFrames = maxFrames;
    mBytes = 0;
    mBytesCur = 0;
    mPeriods = 0;
    mFrames = 0;
    mFramesCur = 0;
    mRapCnt = 0;

    mTStart = std::chrono::steady_clock::now();

    return 0;
  }

  int addAU( vvencAccessUnit* au, bool* periodDone )
  {
    if( !au ){ return -1; } 
    
    *periodDone = false;

    mFrames++;
    mFramesCur++;

    if( mBytes && mFramesCur >= mFramerate )
    {
      mPeriods++;
      *periodDone = true;
    }

    if( au->rap )
      mRapCnt++;

    mBytes    += au->payloadUsedSize;
    mBytesCur += au->payloadUsedSize;

    //std::cout << "au " << au->poc << " bytes " << au->payloadUsedSize << " bitrate " << mBytesCur*8 << std::endl;
 
    return 0;
  }

  std::string getAndResetCurBitrate()
  {
    std::stringstream css;
    uint64_t bitrate = mBytesCur*8;

    mTEnd = std::chrono::steady_clock::now();

    if( mPeriods && mBytesCur )
    {
      double dTimeSec = (double)std::chrono::duration_cast<std::chrono::milliseconds>((mTEnd)-(mTStart)).count() / 1000;
      double dFps = dTimeSec ? (double)mFramesCur / dTimeSec : 0;

      css << "stats:";
      css << std::fixed << std::setprecision(2) << " frame= " << mFrames << "/" << mMaxFrames << " fps= " << dFps;
      css << std::fixed << std::setprecision(2) << " bitrate= " << (double)bitrate/1000000.0 << " Mbps";

      bitrate = mPeriods ? mBytes*8/mPeriods : 0;
      css << std::fixed << std::setprecision(2) << " avg_bitrate= " << (double)bitrate/1000000.0 << " Mbps ";


      css << std::setprecision(-1) << std::endl;
    }

    mBytesCur = 0;
    mFramesCur = 0;
    mRapCnt = 0;
    mTStart = std::chrono::steady_clock::now();

    return css.str();
  }

private:
   int mFramerate     = 1;
   int mMaxFrames     = 0;

   uint64_t mBytes    = 0;
   uint64_t mBytesCur = 0;

   int mPeriods       = 0;
   int mFrames        = 0;
   int mFramesCur     = 0;

   int mRapCnt = 0;

   std::chrono::steady_clock::time_point mTStart;
   std::chrono::steady_clock::time_point mTEnd; 
};

} // namespace apputils

//! \}


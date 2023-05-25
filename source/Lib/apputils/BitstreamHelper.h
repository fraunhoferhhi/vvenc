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
#include <string.h>

//#include "vvenc/vvencCfg.h"
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

  int init( int framerate )
  {
    mFramerate = framerate;
    mBytes = 0;
    mBytesCur = 0;
    mPeriods = 0;
    mFrames = 0;
    return 0;
  }

  int addAU( vvencAccessUnit* au, bool* periodDone )
  {
    if( !au ){ return -1; } 
    
    *periodDone = false;
    if( mBytes && au->rap )
    {
      mPeriods++;
      *periodDone = true;
    }

    mBytes    += au->payloadUsedSize;
    mBytesCur += au->payloadUsedSize;
    mFrames++;

    //std::cout << "au " << au->poc << " bytes " << au->payloadUsedSize << " bitrate " << mBytesCur*8 << std::endl;
 
    return 0;
  }

  std::string getCurBitrate()
  {
    std::stringstream css;
    uint64_t bitrate = mBytesCur*8;

    css << "BITRATE ";
    if( bitrate < 1000000 )
      css <<  (double)bitrate/1000.0 << " kbps ";
    else
      css << (double)bitrate/1000000.0 << " Mbps ";

    css << " periods " << mPeriods << " avg ";
    bitrate = mBytes*8/mPeriods;

    if( bitrate < 1000000 )
      css <<  (double)bitrate/1000.0 << " kbps ";
    else
      css << (double)bitrate/1000000.0 << " Mbps ";

    css << " frames " << mFrames << std::endl;

    mBytesCur = 0;
    return css.str();
  }

  void reset()
  {
    mBytesCur = 0;
  }

private:
   int mFramerate     = 1;
   
   uint64_t mBytes    = 0;
   uint64_t mBytesCur = 0;

   int mPeriods       = 0;
   int mFrames        = 0;
};

} // namespace apputils

//! \}


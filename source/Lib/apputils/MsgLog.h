/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */
/**
  \file    MsgLog.h
  \brief   A logger for the logging callback function
*/

#pragma once

#include <functional>
#include <stdarg.h>
#include <mutex>

#include "vvenc/vvencCfg.h"

namespace apputils {

static std::mutex m_msgMutex;

class MsgLog
{
public:

  MsgLog(){}
  MsgLog(void *msgCtx, vvencLoggingCallback msgFnc)
  {
    m_msgCtx = msgCtx;
    m_msgFnc = msgFnc;
  }

  ~MsgLog() {};

  void setCallback( void *msgCtx, vvencLoggingCallback msgFnc )
  {
    m_msgCtx = msgCtx;
    m_msgFnc = msgFnc;
  }

  void log( int level, const char* fmt, ... )
  {
    if ( this->m_msgFnc )
    {
      std::unique_lock<std::mutex> _lock( m_msgMutex );
      va_list args;
      va_start( args, fmt );
      m_msgFnc( m_msgCtx, level, fmt, args );
      va_end( args );
    }
}

private: 
  std::function<void( void*, int, const char*, va_list )> m_msgFnc{};
  void *m_msgCtx{};
};

} // namespace vvenc

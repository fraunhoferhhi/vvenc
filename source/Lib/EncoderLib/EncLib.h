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
/** \file     EncLib.h
    \brief    encoder class (header)
*/

#pragma once

#include "EncGOP.h"
#include "EncStage.h"
#include "CommonLib/MCTF.h"
#include "vvenc/vvencCfg.h"

#include <mutex>

//! \ingroup EncoderLib
//! \{

namespace vvenc {


// ====================================================================================================================
// Class definition
// ====================================================================================================================


class NoMallocThreadPool;
class MsgLog;
class GOPCfg;

/// encoder class
class EncLib
{
private:
  MsgLog&                    msg;

  std::function<void( void*, vvencYUVBuffer* )> m_recYuvBufFunc;
  void*                                         m_recYuvBufCtx;

  const VVEncCfg             m_encCfg;
  const VVEncCfg             m_orgCfg;
  VVEncCfg                   m_firstPassCfg;
  GOPCfg*                    m_gopCfg;
  RateCtrl*                  m_rateCtrl;
  MCTF*                      m_MCTF;
  EncGOP*                    m_preEncoder;
  EncGOP*                    m_gopEncoder;
  std::vector<EncStage*>     m_encStages;
  std::list<PicShared*>      m_picSharedList;
  std::deque<PicShared*>     m_prevSharedQueue;
  PicShared*                 m_prevSharedTL0;

  NoMallocThreadPool*        m_threadPool;

  int                        m_picsRcvd;
  int                        m_passInitialized;
  int                        m_maxNumPicShared;
  bool                       m_accessUnitOutputStarted;
  bool                       m_firstFlushDone;
  std::mutex                 m_stagesMutex;
  std::condition_variable    m_stagesCond;
  std::deque<AccessUnitList> m_AuList;

public:
  EncLib( MsgLog& logger );
  virtual ~EncLib();

  void     setRecYUVBufferCallback( void* ctx, vvencRecYUVBufferCallback func );
  void     initEncoderLib      ( const vvenc_config& encCfg );
  void     initPass            ( int pass, const char* statsFName );
  void     encodePicture       ( bool flush, const vvencYUVBuffer* yuvInBuf, AccessUnitList& au, bool& isQueueEmpty );
  void     uninitEncoderLib    ();
  void     printSummary        ();
  void     getParameterSets    ( AccessUnitList& au );

private:
  void     xUninitLib          ();
  void     xInitRCCfg          ();

  PicShared* xGetFreePicShared();
  void     xAssignPrevQpaBufs( PicShared* picShared );

  void     xDetectScc          ( PicShared* picShared );
 };

} // namespace vvenc

//! \}


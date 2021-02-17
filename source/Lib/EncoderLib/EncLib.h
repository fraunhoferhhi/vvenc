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
/** \file     EncLib.h
    \brief    encoder class (header)
*/

#pragma once

#include "EncGOP.h"
#include "EncHRD.h"
#include "CommonLib/MCTF.h"
#include "CommonLib/Nal.h"
#include "vvenc/vvencCfg.h"

#include <mutex>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class YUVWriterIf;
class NoMallocThreadPool;

/// encoder class
class EncLib
{
private:
  int                       m_numPicsRcvd;
  int                       m_numPicsInQueue;
  int                       m_numPicsCoded;
  int                       m_pocEncode;
  int                       m_pocRecOut;
  int                       m_GOPSizeLog2;
  int                       m_TicksPerFrameMul4;
  int                       m_numPassInitialized;

  const VVEncCfg            m_cEncCfg;
  VVEncCfg                  m_cBckCfg;
  EncGOP*                   m_cGOPEncoder;
  EncHRD                    m_cEncHRD;
  MCTF                      m_MCTF;
  PicList                   m_cListPic;
  YUVWriterIf*              m_yuvWriterIf;
  NoMallocThreadPool*       m_threadPool;
  RateCtrl                  m_cRateCtrl;                          ///< Rate control class

  VPS                       m_cVPS;
  DCI                       m_cDCI;
  ParameterSetMap<SPS>      m_spsMap;
  ParameterSetMap<PPS>      m_ppsMap;

  XUCache                   m_shrdUnitCache;
  std::mutex                m_unitCacheMutex;

  std::vector<int>          m_pocToGopId;
  std::vector<int>          m_nextPocOffset;

public:
  EncLib();
  virtual ~EncLib();

  void     initEncoderLib      ( const VVEncCfg& encCfg, YUVWriterIf* yuvWriterIf );
  void     initPass            ( int pass );
  void     encodePicture       ( bool flush, const YUVBuffer& yuvInBuf, AccessUnitList& au, bool& isQueueEmpty );
  void     uninitEncoderLib    ();
  void     printSummary        ();

private:
  void     xUninitLib          ();
  void     xResetLib           ();
  void     xSetRCEncCfg        ( int pass );

  int      xGetGopIdFromPoc    ( int poc ) const { return m_pocToGopId[ poc % m_cEncCfg.m_GOPSize ]; }
  int      xGetNextPocICO      ( int poc, bool flush, int max ) const;
  void     xCreateCodingOrder  ( int start, int max, int numInQueue, bool flush, std::vector<Picture*>& encList );
  void     xInitPicture        ( Picture& pic, int picNum, const PPS& pps, const SPS& sps, const VPS& vps, const DCI& dci );
  void     xDeletePicBuffer    ();
  Picture* xGetNewPicBuffer    ( const PPS& pps, const SPS& sps );            ///< get picture buffer which will be processed. If ppsId<0, then the ppsMap will be queried for the first match.
  Picture* xGetPictureBuffer   ( int poc );

  void     xInitVPS            ( VPS &vps )                                  const; ///< initialize VPS from encoder options
  void     xInitDCI            ( DCI &dci, const SPS &sps, const int dciId ) const; ///< initialize DCI from encoder options
  void     xInitConstraintInfo ( ConstraintInfo &ci )                        const;  ///< initialize SPS from encoder options
  void     xInitSPS            ( SPS &sps )                                  const; ///< initialize SPS from encoder options
  void     xInitPPS            ( PPS &pps, const SPS &sps )                  const;  ///< initialize PPS from encoder options
  void     xInitPPSforTiles    ( PPS &pps ) const;
  void     xInitRPL            ( SPS &sps ) const;
  void     xInitHrdParameters  ( SPS &sps );
  void     xOutputRecYuv       ();
#if SCC_MCTF
  void     xDetectScreenC      ( Picture& pic, PelUnitBuf yuvOrgBuf );
#else
  void     xDetectScreenC      ( Picture& pic , PelUnitBuf yuvOrgBuf, int useTS);
#endif
};

} // namespace vvenc

//! \}


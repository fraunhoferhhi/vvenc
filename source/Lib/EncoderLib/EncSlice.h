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

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
/** \file     EncSlice.h
    \brief    slice encoder class (header)
*/

#pragma once

#include "EncCu.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "InterSearch.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class VVEncCfg;
struct SAOStatData;
class EncSampleAdaptiveOffset;
class EncAdaptiveLoopFilter;
class EncPicture;
class NoMallocThreadPool;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct LineEncRsrc;
struct PerThreadRsrc;
struct CtuEncParam;

enum ProcessCtuState {
  CTU_ENCODE     = 0,
  RESHAPE_LF_VER,
  LF_HOR,
  SAO_FILTER,
  ALF_GET_STATISTICS,
  ALF_DERIVE_FILTER,
  ALF_RECONSTRUCT,
  FINISH_SLICE,
  PROCESS_DONE
};

/// slice encoder class
class EncSlice
{
private:
  // encoder configuration
  const VVEncCfg*              m_pcEncCfg;                           ///< encoder configuration class

  std::vector<PerThreadRsrc*>  m_CtuTaskRsrc;
  std::vector<LineEncRsrc*>    m_LineEncRsrc;
  NoMallocThreadPool*          m_threadPool;
  std::vector<ProcessCtuState> m_processStates;

  LoopFilter*                  m_pLoopFilter;
  EncAdaptiveLoopFilter*       m_pALF;
  RateCtrl*                    m_pcRateCtrl;
  BinEncoder                   m_BinEncoder;
  CABACWriter                  m_CABACWriter;

  Ctx                          m_entropyCodingSyncContextState;      ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row used for writing
  std::vector<Ctx>             m_syncPicCtx;                         ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row used for estimation
  SliceType                    m_encCABACTableIdx;
  int                          m_appliedSwitchDQQ;

  double                       m_saoDisabledRate[ MAX_NUM_COMP ][ MAX_TLAYER ];
  bool                         m_saoEnabled[ MAX_NUM_COMP ];
  bool                         m_saoAllDisabled;
  std::vector<SAOBlkParam>     m_saoReconParams;
  std::vector<SAOStatData**>   m_saoStatData;
  std::vector<CtuEncParam>     ctuEncParams;

public:
  EncSlice();
  virtual ~EncSlice();

  void    init                ( const VVEncCfg& encCfg,
                                const SPS& sps,
                                const PPS& pps,
                                std::vector<int>* const globalCtuQpVector,
                                LoopFilter& loopFilter,
                                EncAdaptiveLoopFilter& alf,
                                RateCtrl& rateCtrl,
                                NoMallocThreadPool* threadPool );

  void    initPic             ( Picture* pic, int gopId );

  // compress and encode slice
  void    compressSlice       ( Picture* pic );      ///< analysis stage of slice                     s
  void    encodeSliceData     ( Picture* pic );
  void    saoDisabledRate     ( CodingStructure& cs, SAOBlkParam* reconParams );
  void    finishCompressSlice ( Picture* pic, Slice& slice );

  void    resetQP              ( Picture* pic, int sliceQP, double lambda );

private:
  void    xInitSliceLambdaQP   ( Slice* slice, int gopId );
  double  xCalculateLambda     ( const Slice* slice, const int GOPid, const int depth, const double refQP, const double dQP, int& iQP );
  void    xProcessCtus         ( Picture* pic, const unsigned startCtuTsAddr, const unsigned boundingCtuTsAddr );
  template<bool checkReadyState=false>
  static bool xProcessCtuTask  ( int taskIdx, CtuEncParam* ctuEncParam );

  int     xGetQPForPicture     ( const Slice* slice, unsigned gopId );
};

} // namespace vvenc

//! \}


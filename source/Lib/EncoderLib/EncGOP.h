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
/** \file     EncGOP.h
    \brief    GOP encoder class (header)
*/

#pragma once

#include "EncSampleAdaptiveOffset.h"
#include "VLCWriter.h"
#include "SEIwrite.h"
#include "SEIEncoder.h"
#include "Analyze.h"
#include "EncPicture.h"
#include "EncReshape.h"
#include "CommonLib/Picture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Nal.h"
#include "RateCtrl.h"

#include <vector>
#include <list>
#include <stdlib.h>
#include <atomic>


#include "Utilities/NoMallocThreadPool.h"
#include <mutex>
#include <condition_variable>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================

class InputByteStream;
class DecLib;
class EncHRD;

struct FFwdDecoder
{
  bool bDecode1stPart;
  bool bHitFastForwardPOC;
  bool loopFiltered;
  int  iPOCLastDisplay;
  std::ifstream* bitstreamFile;
  InputByteStream* bytestream;
  DecLib *pcDecLib;

  FFwdDecoder()
    : bDecode1stPart      ( true )
      , bHitFastForwardPOC( false )
      , loopFiltered      ( false )
      , iPOCLastDisplay   ( -MAX_INT )
      , bitstreamFile     ( nullptr )
      , bytestream        ( nullptr )
      , pcDecLib          ( nullptr )
  {}
};

// ====================================================================================================================

class EncGOP;
struct FinishTaskParam {
  EncGOP*     gopEncoder;
  EncPicture* picEncoder;
  Picture*    pic;
  FinishTaskParam()                                          : gopEncoder( nullptr ), picEncoder( nullptr ), pic( nullptr ) {}
  FinishTaskParam( EncGOP* _g, EncPicture* _e, Picture* _p ) : gopEncoder( _g ),      picEncoder( _e ),      pic( _p )      {}
};

// ====================================================================================================================

class EncGOP
{
private:
  Analyze                   m_AnalyzeAll;
  Analyze                   m_AnalyzeI;
  Analyze                   m_AnalyzeP;
  Analyze                   m_AnalyzeB;

  bool                      m_bFirstInit;
  bool                      m_bFirstWrite;
  bool                      m_bRefreshPending;
  int                       m_codingOrderIdx;
  int                       m_lastIDR;
  int                       m_lastRasPoc;
  int                       m_pocCRA;
  int                       m_associatedIRAPPOC;
  NalUnitType               m_associatedIRAPType;

  const VVEncCfg*           m_pcEncCfg;
  HLSWriter                 m_HLSWriter;
  SEIWriter                 m_seiWriter;
  SEIEncoder                m_seiEncoder;
  EncReshape                m_Reshaper;
  BlkStat                   m_BlkStat;
  FFwdDecoder               m_ffwdDecoder;
  RateCtrl*                 m_pcRateCtrl;
  EncHRD*                   m_pcEncHRD;
  ParameterSetMap<APS>      m_gopApsMap;

  std::list<EncPicture*>    m_freePicEncoderList;
  std::list<Picture*>       m_gopEncListInput;

  std::vector<int>          m_globalCtuQpVector;

  NoMallocThreadPool*       m_threadPool;
  std::mutex                m_gopEncMutex;
  std::condition_variable   m_gopEncCond;

public:
  std::list<Picture*>       m_gopEncListOutput;

public:
  EncGOP();
  virtual ~EncGOP();

  void init               ( const VVEncCfg& encCfg, const SPS& sps, const PPS& pps, RateCtrl& rateCtrl, EncHRD& encHrd, NoMallocThreadPool* threadPool );
  void encodePictures     ( const std::vector<Picture*>& encList, PicList& picList, AccessUnitList& au, bool isEncodeLtRef );
  void printOutSummary    ( int numAllPicCoded, const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr, const BitDepths &bitDepths );
  void picInitRateControl ( int gopId, Picture& pic, Slice* slice, EncPicture *picEncoder );
  ParameterSetMap<APS>&       getSharedApsMap()       { return m_gopApsMap; }
  const ParameterSetMap<APS>& getSharedApsMap() const { return m_gopApsMap; }
  bool                        anyFramesInOutputQueue() { return !m_gopEncListOutput.empty(); }

private:
  void xUpdateRasInit                 ( Slice* slice );

  NalUnitType xGetNalUnitType         ( int pocCurr, int lastIdr ) const;
  int  xGetSliceDepth                 ( int poc ) const;
  bool xIsSliceTemporalSwitchingPoint ( const Slice* slice, PicList& picList, int gopId ) const;

  void xInitPicsInCodingOrder         ( const std::vector<Picture*>& encList, PicList& picList, bool isEncodeLtRef );
  void xGetProcessingLists            ( std::list<Picture*>& procList, std::list<Picture*>& rcUpdateList );
  void xInitFirstSlice                ( Picture& pic, PicList& picList, bool isEncodeLtRef );
  void xInitSliceTMVPFlag             ( PicHeader* picHeader, const Slice* slice, int gopId );
#if RPR_READY
  void xUpdateRPRtmvp                 ( PicHeader* picHeader, Slice* slice );
  void xUpdateRPRToolCtrl             ( PicHeader* picHeader, Slice* slice );
#endif
  void xInitSliceMvdL1Zero            ( PicHeader* picHeader, const Slice* slice );
  void xInitLMCS                      ( Picture& pic );
  void xSelectReferencePictureList    ( Slice* slice, int curPoc, int gopId, int ltPoc );
  void xSyncAlfAps                    ( Picture& pic, ParameterSetMap<APS>& dst, const ParameterSetMap<APS>& src );

  void xWritePicture                  ( Picture& pic, AccessUnitList& au, bool isEncodeLtRef );
  int  xWriteParameterSets            ( Picture& pic, AccessUnitList& accessUnit, HLSWriter& hlsWriter );
  int  xWritePictureSlices            ( Picture& pic, AccessUnitList& accessUnit, HLSWriter& hlsWriter );
  void xWriteLeadingSEIs              ( const Picture& pic, AccessUnitList& accessUnit );
  void xWriteTrailingSEIs             ( const Picture& pic, AccessUnitList& accessUnit, std::string& digestStr );
  int  xWriteVPS                      ( AccessUnitList &accessUnit, const VPS *vps, HLSWriter& hlsWriter );
  int  xWriteDCI                      ( AccessUnitList &accessUnit, const DCI *dci, HLSWriter& hlsWriter );
  int  xWriteSPS                      ( AccessUnitList &accessUnit, const SPS *sps, HLSWriter& hlsWriter );
  int  xWritePPS                      ( AccessUnitList &accessUnit, const PPS *pps, const SPS *sps, HLSWriter& hlsWriter );
  int  xWriteAPS                      ( AccessUnitList &accessUnit, const APS *aps, HLSWriter& hlsWriter, NalUnitType eNalUnitType );
  void xWriteAccessUnitDelimiter      ( AccessUnitList &accessUnit, Slice* slice, bool IrapOrGdr, HLSWriter& hlsWriter );
  void xWriteSEI                      ( NalUnitType naluType, SEIMessages& seiMessages, AccessUnitList &accessUnit, AccessUnitList::iterator &auPos, int temporalId, const SPS *sps );
  void xWriteSEISeparately            ( NalUnitType naluType, SEIMessages& seiMessages, AccessUnitList &accessUnit, AccessUnitList::iterator &auPos, int temporalId, const SPS *sps );
  void xAttachSliceDataToNalUnit      ( OutputNALUnit& rNalu, const OutputBitstream* pcBitstreamRedirect );
  void xCabacZeroWordPadding          ( const Picture& pic, const Slice* slice, uint32_t binCountsInNalUnits, uint32_t numBytesInVclNalUnits, std::ostringstream &nalUnitData );

  void xUpdateAfterPicRC              ( const Picture* pic );
  void xCalculateAddPSNR              ( const Picture* pic, CPelUnitBuf cPicD, AccessUnitList&, bool printFrameMSE, double* PSNR_Y, bool isEncodeLtRef );
  uint64_t xFindDistortionPlane       ( const CPelBuf& pic0, const CPelBuf& pic1, uint32_t rshift ) const;
  void xPrintPictureInfo              ( const Picture& pic, AccessUnitList& accessUnit, const std::string& digestStr, bool printFrameMSE, bool isEncodeLtRef );
};// END CLASS DEFINITION EncGOP

} // namespace vvenc

//! \}


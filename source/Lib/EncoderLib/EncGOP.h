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
#include "RateCtrl.h"

#include <vector>
#include <list>
#include <stdlib.h>
#include <atomic>
#include "vvenc/Nal.h"

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
  int                       m_lastIDR;
  int                       m_lastRasPoc;
  int                       m_pocCRA;
  int                       m_associatedIRAPPOC;
  NalUnitType               m_associatedIRAPType;

  const EncCfg*             m_pcEncCfg;
  HLSWriter                 m_HLSWriter;
  SEIWriter                 m_seiWriter;
  SEIEncoder                m_seiEncoder;
  EncReshape                m_Reshaper;
  BlkStat                   m_BlkStat;
  FFwdDecoder               m_ffwdDecoder;
  RateCtrl*                 m_pcRateCtrl;
  EncHRD*                   m_pcEncHRD;
  ParameterSetMap<APS>      m_gopApsMap;

  std::vector<EncPicture*>  m_picEncoderList;
  std::list<Picture*>       m_encodePics;
  std::atomic_int           m_numPicEncoder;
  NoMallocThreadPool*       m_gopThreadPool;
  std::mutex                m_gopEncMutex;
  std::condition_variable   m_gopEncCond;
  std::vector<int>          m_globalCtuQpVector;

  double                    m_lambda;
  int                       m_actualHeadBits;
  int                       m_actualTotalBits;
  int                       m_estimatedBits;

public:
  EncGOP();
  virtual ~EncGOP();

  void init               ( const EncCfg& encCfg, const SPS& sps, const PPS& pps, RateCtrl& rateCtrl, EncHRD& encHrd, NoMallocThreadPool* threadPool );
  void encodePicture      ( std::vector<Picture*> encList, PicList& picList, AccessUnit& au, bool isEncodeLtRef );
  void finishEncPicture   ( EncPicture* picEncoder, Picture& pic );
  void printOutSummary    ( int numAllPicCoded, const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr, const BitDepths &bitDepths );
  void picInitRateControl ( int gopId, Picture& pic, Slice* slice );

  EncPicture* getPicEncoder( int idx )
  {
    CHECK( idx > m_picEncoderList.size() || m_picEncoderList[ idx] == nullptr, "error: array index out of bounds" );
    return m_picEncoderList[ idx ];
  }

private:
  void xUpdateRasInit                 ( Slice* slice );

  NalUnitType xGetNalUnitType         ( int pocCurr, int lastIdr ) const;
  int  xGetSliceDepth                 ( int poc ) const;
  bool xIsSliceTemporalSwitchingPoint ( const Slice* slice, PicList& picList, int gopId ) const;

  void xInitPicsICO                   ( std::vector<Picture*> encList, PicList& picList, bool isEncodeLtRef );
  void xInitFirstSlice                ( Picture& pic, PicList& picList, bool isEncodeLtRef );
  void xInitSliceTMVPFlag             ( PicHeader* picHeader, const Slice* slice, int gopId );
  void xInitSliceMvdL1Zero            ( PicHeader* picHeader, const Slice* slice );
  void xInitLMCS                      ( Picture& pic );
  void xSelectReferencePictureList    ( Slice* slice, int curPoc, int gopId, int ltPoc );
  void xSyncAlfAps                    ( Picture& pic, ParameterSetMap<APS>& dst, const ParameterSetMap<APS>& src );

  void xWritePicture                  ( Picture& pic, AccessUnit& au, bool isEncodeLtRef );
  int  xWriteParameterSets            ( Picture& pic, AccessUnit& accessUnit, HLSWriter& hlsWriter );
  int  xWritePictureSlices            ( Picture& pic, AccessUnit& accessUnit, HLSWriter& hlsWriter );
  void xWriteLeadingSEIs              ( const Picture& pic, AccessUnit& accessUnit );
  void xWriteTrailingSEIs             ( const Picture& pic, AccessUnit& accessUnit, std::string& digestStr );
  int  xWriteVPS                      ( AccessUnit &accessUnit, const VPS *vps, HLSWriter& hlsWriter );
  int  xWriteDCI                      ( AccessUnit &accessUnit, const DCI *dci, HLSWriter& hlsWriter );
  int  xWriteSPS                      ( AccessUnit &accessUnit, const SPS *sps, HLSWriter& hlsWriter );
  int  xWritePPS                      ( AccessUnit &accessUnit, const PPS *pps, const SPS *sps, HLSWriter& hlsWriter );
  int  xWriteAPS                      ( AccessUnit &accessUnit, const APS *aps, HLSWriter& hlsWriter, NalUnitType eNalUnitType );
  void xWriteAccessUnitDelimiter      ( AccessUnit &accessUnit, Slice* slice, bool IrapOrGdr, HLSWriter& hlsWriter );
  void xWriteSEI                      ( NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, int temporalId, const SPS *sps );
  void xWriteSEISeparately            ( NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, int temporalId, const SPS *sps );
  void xAttachSliceDataToNalUnit      ( OutputNALUnit& rNalu, const OutputBitstream* pcBitstreamRedirect );
  void xCabacZeroWordPadding          ( const Picture& pic, const Slice* slice, uint32_t binCountsInNalUnits, uint32_t numBytesInVclNalUnits, std::ostringstream &nalUnitData );

  void xUpdateAfterPicRC              ( const Picture* pic );
  void xCalculateAddPSNR              ( const Picture* pic, CPelUnitBuf cPicD, AccessUnit&, bool printFrameMSE, double* PSNR_Y, bool isEncodeLtRef );
  uint64_t xFindDistortionPlane       ( const CPelBuf& pic0, const CPelBuf& pic1, uint32_t rshift ) const;
  void xPrintPictureInfo              ( const Picture& pic, AccessUnit& accessUnit, const std::string& digestStr, bool printFrameMSE, bool isEncodeLtRef );
};// END CLASS DEFINITION EncGOP

} // namespace vvenc

//! \}


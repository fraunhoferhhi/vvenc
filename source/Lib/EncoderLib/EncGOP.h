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
#include "../../../include/vvenc/Nal.h"

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
  ParameterSetMap<APS>      m_gopApsMap;
  RateCtrl*                 m_pcRateCtrl;
  EncHRD*                   m_pcEncHRD;

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


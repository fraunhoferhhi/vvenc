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
/** \file     DecLib.h
    \brief    decoder class (header)
*/

#pragma once

#include "DecSlice.h"
#include "CABACReader.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/Reshape.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Unit.h"
#include "CommonLib/SampleAdaptiveOffset.h"

//! \ingroup DecoderLib
//! \{

namespace vvenc {

class InputNALUnit;
struct FFwdDecoder;

bool tryDecodePicture( Picture* pic, const int expectedPoc, const std::string& bitstreamFileName, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>* apsMap, bool bDecodeUntilPocFound = false, int debugPOC = -1, bool copyToEnc = true  );

// Class definition
// ====================================================================================================================

/// decoder class
class DecLib
{
private:
  int                     m_iMaxRefPicNum;

  vvencNalUnitType        m_associatedIRAPType; ///< NAL unit type of the associated IRAP picture
  int                     m_pocCRA;            ///< POC number of the latest CRA picture
  int                     m_pocRandomAccess;   ///< POC number of the random access point (the first IDR or CRA picture)
  int                     m_lastRasPoc;

  PicList                 m_cListPic;         //  Dynamic buffer
  ParameterSetManager     m_parameterSetManager;  // storage for parameter sets
  PicHeader               m_picHeader;            // picture header
  Slice*                  m_apcSlicePilot;


  SEIMessages             m_SEIs; ///< List of SEI messages that have been received before the first slice and between slices, excluding prefix SEIs...


  // functional classes
  IntraPrediction         m_cIntraPred;
  InterPrediction         m_cInterPred;
  TrQuant                 m_cTrQuant;
  DecSlice                m_cSliceDecoder;
  DecCu                   m_cCuDecoder;
  HLSyntaxReader          m_HLSReader;
  CABACDecoder            m_CABACDecoder;
  SEIReader               m_seiReader;
  LoopFilter              m_cLoopFilter;
  SampleAdaptiveOffset    m_cSAO;
  Reshape                 m_cReshaper;
  HRD                     m_HRD;
  AdaptiveLoopFilter      m_cALF;
  RdCost                  m_cRdCost;                      ///< RD cost computation class
  XUCache                 m_unitCache;
  bool isRandomAccessSkipPicture(int& iSkipFrame,  int& iPOCLastDisplay);
  Picture*                m_pic;
  uint32_t                m_uiSliceSegmentIdx;
  uint32_t                m_prevLayerID;
  int                     m_prevPOC;
  int                     m_prevTid0POC;
  bool                    m_bFirstSliceInPicture;
  bool                    m_bFirstSliceInSequence;
  bool                    m_prevSliceSkipped;
  int                     m_skippedPOC;
  bool                    m_bFirstSliceInBitstream;
  int                     m_lastPOCNoOutputPriorPics;
  bool                    m_isNoOutputPriorPics;
  bool                    m_lastNoIncorrectPicOutputFlag;    //value of variable NoIncorrectPicOutputFlag of the last CRA / GDR pic
  std::ostream*           m_pDecodedSEIOutputStream;
  int                     m_decodedPictureHashSEIEnabled;  ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  uint32_t                m_numberOfChecksumErrorsDetected;
  bool                    m_warningMessageSkipPicture;

  std::list<InputNALUnit*> m_prefixSEINALUs; /// Buffered up prefix SEI NAL Units.
  int                     m_debugPOC;
  bool                    m_isDecoderInEncoder;
  std::vector<std::pair<vvencNalUnitType, int>> m_accessUnitNals;
  struct AccessUnitPicInfo
  {
    vvencNalUnitType m_nalUnitType; ///< nal_unit_type
    uint32_t         m_temporalId;  ///< temporal_id
    uint32_t         m_nuhLayerId;  ///< nuh_layer_id
    int              m_POC;
  };
  std::vector<AccessUnitPicInfo> m_accessUnitPicInfo;
  std::vector<int> m_accessUnitApsNals;

  VPS*                    m_vps;
  bool                    m_scalingListUpdateFlag;
  int                     m_PreScalingListAPSId;
  int                     m_maxDecSubPicIdx;
  int                     m_maxDecSliceAddrInSubPic;
  ParameterSetMap<APS>*   m_apsMapEnc;

public:
  int                     m_targetSubPicIdx;
public:
  DecLib();
  virtual ~DecLib();

  void  create  ();
  void  destroy ();

  void  setDecodedPictureHashSEIEnabled(int enabled) { m_decodedPictureHashSEIEnabled=enabled; }

  void  init();
  bool  decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay, int iTargetOlsIdx);
  void  deletePicBuffer();

  void  executeLoopFilters();
  void  finishPicture(int& poc, PicList*& rpcListPic, vvencMsgLevel msgl = VVENC_INFO);
  void  finishPictureLight(int& poc, PicList*& rpcListPic );
  void  checkNoOutputPriorPics (PicList* rpcListPic);
  void  checkNalUnitConstraints( uint32_t naluType );

  bool  getNoOutputPriorPicsFlag () const   { return m_isNoOutputPriorPics; }
  void  setNoOutputPriorPicsFlag (bool val) { m_isNoOutputPriorPics = val; }
  bool  getFirstSliceInPicture () const { return m_bFirstSliceInPicture; }
  void  setFirstSliceInPicture (bool val)  { m_bFirstSliceInPicture = val; }
  bool  getFirstSliceInSequence () const   { return m_bFirstSliceInSequence; }
  void  setFirstSliceInSequence (bool val) { m_bFirstSliceInSequence = val; }
  void  setDecodedSEIMessageOutputStream(std::ostream *pOpStream) { m_pDecodedSEIOutputStream = pOpStream; }
  uint32_t  getNumberOfChecksumErrorsDetected() const { return m_numberOfChecksumErrorsDetected; }

  int  getDebugPOC( )               const { return m_debugPOC; };
  void setDebugPOC( int debugPOC )        { m_debugPOC = debugPOC; };
  void resetAccessUnitNals()              { m_accessUnitNals.clear();    }
  void resetAccessUnitPicInfo()           { m_accessUnitPicInfo.clear();    }
  void resetAccessUnitApsNals()           { m_accessUnitApsNals.clear(); }
  void setDecoderInEncoderMode( bool m )  { m_isDecoderInEncoder = m; }
  bool isSliceNaluFirstInAU( bool newPicture, InputNALUnit &nalu );

  const VPS* getVPS()                     { return m_vps; }

  bool  getScalingListUpdateFlag() { return m_scalingListUpdateFlag; }
  void  setScalingListUpdateFlag(bool b) { m_scalingListUpdateFlag = b; }
  int   getPreScalingListAPSId() { return m_PreScalingListAPSId; }
  void  setPreScalingListAPSId(int id) { m_PreScalingListAPSId = id; }
  void  setAPSMapEnc( ParameterSetMap<APS>* apsMap ) { m_apsMapEnc = apsMap;  }
  bool  isNewPicture( std::ifstream *bitstreamFile, class InputByteStream *bytestream );
  bool  isNewAccessUnit( bool newPicture, std::ifstream *bitstreamFile, class InputByteStream *bytestream );

protected:
  void      xUpdateRasInit(Slice* slice);

  Picture * xGetNewPicBuffer(const SPS &sps, const PPS &pps, const uint32_t temporalLayer, const int layerId);
  void      xCreateLostPicture (int iLostPOC, const int layerId);
  void      xActivateParameterSets( const int layerId );
  void      xCheckParameterSetConstraints( const int layerId );
  void      xDecodePicHeader( InputNALUnit& nalu );
  bool      xDecodeSlice(InputNALUnit &nalu, int& iSkipFrame, int iPOCLastDisplay);
  void      xDecodeVPS( InputNALUnit& nalu );
  void      xDecodeDCI( InputNALUnit& nalu );
  void      xDecodeSPS( InputNALUnit& nalu );
  void      xDecodePPS( InputNALUnit& nalu );
  void      xDecodeAPS(InputNALUnit& nalu);
  void      xUpdatePreviousTid0POC(Slice* pSlice) { if ((pSlice->TLayer == 0) && (pSlice->nalUnitType!=VVENC_NAL_UNIT_CODED_SLICE_RASL) && (pSlice->nalUnitType!=VVENC_NAL_UNIT_CODED_SLICE_RADL))  { m_prevTid0POC = pSlice->poc; }  }
  void      xParsePrefixSEImessages();
  void      xParsePrefixSEIsForUnknownVCLNal();

  void      xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType );

};// END CLASS DEFINITION DecLib

} // namespace vvenc

//! \}


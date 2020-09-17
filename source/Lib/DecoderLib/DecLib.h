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

bool tryDecodePicture( Picture* pic, const int expectedPoc, const std::string& bitstreamFileName, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>& apsMap, bool bDecodeUntilPocFound = false, int debugPOC = -1 );

// Class definition
// ====================================================================================================================

/// decoder class
class DecLib
{
private:
  int                     m_iMaxRefPicNum;

  NalUnitType             m_associatedIRAPType; ///< NAL unit type of the associated IRAP picture
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
  std::vector<std::pair<NalUnitType, int>> m_accessUnitNals;
  struct AccessUnitPicInfo
  {
    NalUnitType     m_nalUnitType; ///< nal_unit_type
    uint32_t        m_temporalId;  ///< temporal_id
    uint32_t        m_nuhLayerId;  ///< nuh_layer_id
    int             m_POC;
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
  void  finishPicture(int& poc, PicList*& rpcListPic, MsgLevel msgl = INFO);
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
  void      xUpdatePreviousTid0POC(Slice* pSlice) { if ((pSlice->TLayer == 0) && (pSlice->nalUnitType!=NAL_UNIT_CODED_SLICE_RASL) && (pSlice->nalUnitType!=NAL_UNIT_CODED_SLICE_RADL))  { m_prevTid0POC = pSlice->poc; }  }
  void      xParsePrefixSEImessages();
  void      xParsePrefixSEIsForUnknownVCLNal();

  void      xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType );

};// END CLASS DEFINITION DecLib

} // namespace vvenc

//! \}


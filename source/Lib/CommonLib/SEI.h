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
#pragma once

#include "CommonDef.h"
#include "libmd5/MD5.h"

#include <list>
#include <vector>
#include <cstring>

//! \ingroup CommonLib
//! \{

namespace vvenc {

/**
 * Abstract class representing an SEI message with lightweight RTTI.
 */
class SEI
{
public:
  enum PayloadType
  {
    BUFFERING_PERIOD                     = 0,
    PICTURE_TIMING                       = 1,
    FILLER_PAYLOAD                       = 3,
    USER_DATA_REGISTERED_ITU_T_T35       = 4,
    USER_DATA_UNREGISTERED               = 5,
    FILM_GRAIN_CHARACTERISTICS           = 19,
    FRAME_PACKING                        = 45,
    PARAMETER_SETS_INCLUSION_INDICATION  = 129,
    DECODING_UNIT_INFO                   = 130,
    DECODED_PICTURE_HASH                 = 132,
    SCALABLE_NESTING                     = 133,
    MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
    DEPENDENT_RAP_INDICATION             = 145,
    EQUIRECTANGULAR_PROJECTION           = 150,
    SPHERE_ROTATION                      = 154,
    REGION_WISE_PACKING                  = 155,
    OMNI_VIEWPORT                        = 156,
    GENERALIZED_CUBEMAP_PROJECTION       = 153,
    FRAME_FIELD_INFO                     = 168,
    SUBPICTURE_LEVEL_INFO                = 203,
    SAMPLE_ASPECT_RATIO_INFO             = 204,
    CONTENT_LIGHT_LEVEL_INFO             = 144,
    ALTERNATIVE_TRANSFER_CHARACTERISTICS = 147,
    AMBIENT_VIEWING_ENVIRONMENT          = 148,
    CONTENT_COLOUR_VOLUME                = 149,
  };

  SEI() {}
  virtual ~SEI() {}

  static const char *getSEIMessageString(SEI::PayloadType payloadType);

  virtual PayloadType payloadType() const = 0;
};

struct SEIMasteringDisplay
{
  bool        colourVolumeSEIEnabled;
  uint32_t    maxLuminance;
  uint32_t    minLuminance;
  uint16_t    primaries[3][2];
  uint16_t    whitePoint[2];
};


class SEIEquirectangularProjection : public SEI
{
public:
  PayloadType payloadType() const { return EQUIRECTANGULAR_PROJECTION; }

  SEIEquirectangularProjection()  {}
  virtual ~SEIEquirectangularProjection() {}

  bool    erpCancelFlag;
  bool    erpPersistenceFlag;
  bool    erpGuardBandFlag;
  uint8_t erpGuardBandType;
  uint8_t erpLeftGuardBandWidth;
  uint8_t erpRightGuardBandWidth;
};

class SEISphereRotation : public SEI
{
public:
  PayloadType payloadType() const { return SPHERE_ROTATION; }

  SEISphereRotation()  {}
  virtual ~SEISphereRotation() {}

  bool  sphereRotationCancelFlag;
  bool  sphereRotationPersistenceFlag;
  int   sphereRotationYaw;
  int   sphereRotationPitch;
  int   sphereRotationRoll;
};

class SEIOmniViewport : public SEI
{
public:
  PayloadType payloadType() const { return OMNI_VIEWPORT; }

  SEIOmniViewport() {}
  virtual ~SEIOmniViewport() {}

  struct OmniViewport
  {
    int      azimuthCentre;
    int      elevationCentre;
    int      tiltCentre;
    uint32_t horRange;
    uint32_t verRange;
  };

  uint32_t omniViewportId;
  bool     omniViewportCancelFlag;
  bool     omniViewportPersistenceFlag;
  uint8_t  omniViewportCntMinus1;
  std::vector<OmniViewport> omniViewportRegions;
};

class SEIRegionWisePacking : public SEI
{
public:
  PayloadType payloadType() const { return REGION_WISE_PACKING; }
  SEIRegionWisePacking() {}
  virtual ~SEIRegionWisePacking() {}
  bool                  rwpCancelFlag;
  bool                  rwpPersistenceFlag;
  bool                  constituentPictureMatchingFlag;
  int                   numPackedRegions;
  int                   projPictureWidth;
  int                   projPictureHeight;
  int                   packedPictureWidth;
  int                   packedPictureHeight;
  std::vector<uint8_t>  rwpTransformType;
  std::vector<bool>     rwpGuardBandFlag;
  std::vector<uint32_t> projRegionWidth;
  std::vector<uint32_t> projRegionHeight;
  std::vector<uint32_t> rwpProjRegionTop;
  std::vector<uint32_t> projRegionLeft;
  std::vector<uint16_t> packedRegionWidth;
  std::vector<uint16_t> packedRegionHeight;
  std::vector<uint16_t> packedRegionTop;
  std::vector<uint16_t> packedRegionLeft;
  std::vector<uint8_t>  rwpLeftGuardBandWidth;
  std::vector<uint8_t>  rwpRightGuardBandWidth;
  std::vector<uint8_t>  rwpTopGuardBandHeight;
  std::vector<uint8_t>  rwpBottomGuardBandHeight;
  std::vector<bool>     rwpGuardBandNotUsedForPredFlag;
  std::vector<uint8_t>  rwpGuardBandType;
};

class SEIGeneralizedCubemapProjection : public SEI
{
public:
  PayloadType payloadType() const { return GENERALIZED_CUBEMAP_PROJECTION; }

  SEIGeneralizedCubemapProjection()  {}
  virtual ~SEIGeneralizedCubemapProjection() {}

  bool                 gcmpCancelFlag;
  bool                 gcmpPersistenceFlag;
  uint8_t              gcmpPackingType;
  uint8_t              gcmpMappingFunctionType;
  std::vector<uint8_t> gcmpFaceIndex;
  std::vector<uint8_t> gcmpFaceRotation;
  std::vector<uint8_t> gcmpFunctionCoeffU;
  std::vector<bool>    gcmpFunctionUAffectedByVFlag;
  std::vector<uint8_t> gcmpFunctionCoeffV;
  std::vector<bool>    gcmpFunctionVAffectedByUFlag;
  bool                 gcmpGuardBandFlag;
  uint8_t              gcmpGuardBandType;
  bool                 gcmpGuardBandBoundaryExteriorFlag;
  uint8_t              gcmpGuardBandSamplesMinus1;
};

class SEISampleAspectRatioInfo : public SEI
{
public:
  PayloadType payloadType() const { return SAMPLE_ASPECT_RATIO_INFO; }
  SEISampleAspectRatioInfo() {}
  virtual ~SEISampleAspectRatioInfo() {}
  bool                  sariCancelFlag;
  bool                  sariPersistenceFlag;
  int                   sariAspectRatioIdc;
  int                   sariSarWidth;
  int                   sariSarHeight;
};

static const uint32_t ISO_IEC_11578_LEN=16;

class SEIuserDataUnregistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_UNREGISTERED; }

  SEIuserDataUnregistered()
    : userData(0)
    {}

  virtual ~SEIuserDataUnregistered()
  {
    delete userData;
  }

  uint8_t   uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  uint32_t  userDataLength;
  uint8_t*  userData;
};

class SEIDecodedPictureHash : public SEI
{
public:
  PayloadType payloadType() const { return DECODED_PICTURE_HASH; }

  SEIDecodedPictureHash() {}
  virtual ~SEIDecodedPictureHash() {}

  HashType    method;
  PictureHash pictureHash;
};

class SEIDependentRAPIndication : public SEI
{
public:
  PayloadType payloadType() const { return DEPENDENT_RAP_INDICATION; }
  SEIDependentRAPIndication() { }

  virtual ~SEIDependentRAPIndication() { }
};


class SEIBufferingPeriod : public SEI
{
public:
  PayloadType payloadType() const { return BUFFERING_PERIOD; }

  SEIBufferingPeriod()
    : bpNalCpbParamsPresent (false)
    , bpVclCpbParamsPresent (false)
    , initialCpbRemovalDelayLength (0)
    , cpbRemovalDelayLength (0)
    , dpbOutputDelayLength (0)
    , bpCpbCnt(0)
    , duCpbRemovalDelayIncrementLength (0)
    , dpbOutputDelayDuLength (0)
    , cpbRemovalDelayDeltasPresent (false)
    , numCpbRemovalDelayDeltas (0)
    , bpMaxSubLayers (0)
    , bpDecodingUnitHrdParamsPresent (false)
    , decodingUnitCpbParamsInPicTimingSeiFlag (false)
    , decodingUnitDpbDuParamsInPicTimingSeiFlag(false)
    , sublayerInitialCpbRemovalDelayPresent(false)
    , additionalConcatenationInfoPresent (false)
    , maxInitialRemovalDelayForConcatenation (0)
    , sublayerDpbOutputOffsetsPresent (false)
    , altCpbParamsPresent (false)
    , useAltCpbParamsFlag (false)
  {
    ::memset( initialCpbRemovalDelay,  0, sizeof(initialCpbRemovalDelay));
    ::memset( initialCpbRemovalOffset, 0, sizeof(initialCpbRemovalOffset));
    ::memset( cpbRemovalDelayDelta,    0, sizeof(cpbRemovalDelayDelta));
    ::memset( dpbOutputTidOffset,      0, sizeof(dpbOutputTidOffset));
  }
  virtual ~SEIBufferingPeriod() {}

  bool      bpNalCpbParamsPresent;
  bool      bpVclCpbParamsPresent;
  uint32_t  initialCpbRemovalDelayLength;
  uint32_t  cpbRemovalDelayLength;
  uint32_t  dpbOutputDelayLength;
  int       bpCpbCnt;
  uint32_t  duCpbRemovalDelayIncrementLength;
  uint32_t  dpbOutputDelayDuLength;
  uint32_t  initialCpbRemovalDelay         [MAX_TLAYER][MAX_CPB_CNT][2];
  uint32_t  initialCpbRemovalOffset        [MAX_TLAYER][MAX_CPB_CNT][2];
  bool      concatenationFlag;
  uint32_t  auCpbRemovalDelayDelta;
  bool      cpbRemovalDelayDeltasPresent;
  int       numCpbRemovalDelayDeltas;
  int       bpMaxSubLayers;
  uint32_t  cpbRemovalDelayDelta    [15];
  bool      bpDecodingUnitHrdParamsPresent;
  bool      decodingUnitCpbParamsInPicTimingSeiFlag;
  bool      decodingUnitDpbDuParamsInPicTimingSeiFlag;
  bool      sublayerInitialCpbRemovalDelayPresent;
  bool      additionalConcatenationInfoPresent;
  uint32_t  maxInitialRemovalDelayForConcatenation;
  bool      sublayerDpbOutputOffsetsPresent;
  uint32_t  dpbOutputTidOffset      [MAX_TLAYER];
  bool      altCpbParamsPresent;
  bool      useAltCpbParamsFlag;
};

class SEIPictureTiming : public SEI
{
public:
  PayloadType payloadType() const { return PICTURE_TIMING; }

  SEIPictureTiming()
  : picDpbOutputDelay (0)
  , picDpbOutputDuDelay (0)
  , numDecodingUnitsMinus1 (0)
  , duCommonCpbRemovalDelayFlag (false)
  , cpbAltTimingInfoPresent (false)
  , ptDisplayElementalPeriodsMinus1(0)
  {
    ::memset(ptSubLayerDelaysPresent,     0, sizeof(ptSubLayerDelaysPresent));
    ::memset(duCommonCpbRemovalDelayMinus1,   0, sizeof(duCommonCpbRemovalDelayMinus1));
    ::memset(cpbRemovalDelayDeltaEnabledFlag, 0, sizeof(cpbRemovalDelayDeltaEnabledFlag));
    ::memset(cpbRemovalDelayDeltaIdx,         0, sizeof(cpbRemovalDelayDeltaIdx));
    ::memset(auCpbRemovalDelay,               0, sizeof(auCpbRemovalDelay));
  }
  virtual ~SEIPictureTiming() {}

  bool                                ptSubLayerDelaysPresent[MAX_TLAYER];
  bool                                cpbRemovalDelayDeltaEnabledFlag[MAX_TLAYER];
  uint32_t                            cpbRemovalDelayDeltaIdx[MAX_TLAYER];
  uint32_t                            auCpbRemovalDelay[MAX_TLAYER];
  uint32_t                            picDpbOutputDelay;
  uint32_t                            picDpbOutputDuDelay;
  uint32_t                            numDecodingUnitsMinus1;
  bool                                duCommonCpbRemovalDelayFlag;
  uint32_t                            duCommonCpbRemovalDelayMinus1[MAX_TLAYER];
  std::vector<uint32_t>               numNalusInDuMinus1;
  std::vector<uint32_t>               duCpbRemovalDelayMinus1;
  bool                                cpbAltTimingInfoPresent;
  std::vector<std::vector<uint32_t>>  nalCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>>  nalCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>               nalCpbDelayOffset;
  std::vector<uint32_t>               nalDpbDelayOffset;
  std::vector<std::vector<uint32_t>>  vclCpbAltInitialRemovalDelayDelta;
  std::vector<std::vector<uint32_t>>  vclCpbAltInitialRemovalOffsetDelta;
  std::vector<uint32_t>               vclCpbDelayOffset;
  std::vector<uint32_t>               vclDpbDelayOffset;
  int                                 ptDisplayElementalPeriodsMinus1;
};

class SEIDecodingUnitInfo : public SEI
{
public:
  PayloadType payloadType() const { return DECODING_UNIT_INFO; }

  SEIDecodingUnitInfo()
    : decodingUnitIdx(0)
    , dpbOutputDuDelayPresent(false)
    , picSptDpbOutputDuDelay(0)
  {}
  virtual ~SEIDecodingUnitInfo() {}

  int   decodingUnitIdx;
  int   duSptCpbRemovalDelayIncrement[MAX_TLAYER];
  bool  duiSubLayerDelaysPresent[MAX_TLAYER];
  bool  dpbOutputDuDelayPresent;
  int   picSptDpbOutputDuDelay;
};


class SEIFrameFieldInfo : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_FIELD_INFO; }

  SEIFrameFieldInfo()
    : fieldPicFlag(false)
    , bottomFieldFlag (false)
    , pairingIndicatedFlag (false)
    , pairedWithNextFieldFlag(false)
    , displayFieldsFromFrameFlag(false)
    , topFieldFirstFlag(false)
    , duplicateFlag(false)
    , displayElementalPeriodsMinus1(0)
    , sourceScanType(0)
  {}
  virtual ~SEIFrameFieldInfo() {}

  bool fieldPicFlag;
  bool bottomFieldFlag;
  bool pairingIndicatedFlag;
  bool pairedWithNextFieldFlag;
  bool displayFieldsFromFrameFlag;
  bool topFieldFirstFlag;
  bool duplicateFlag;
  int  displayElementalPeriodsMinus1;
  int  sourceScanType;
};


class SEIFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_PACKING; }

  SEIFramePacking() {}
  virtual ~SEIFramePacking() {}

  bool arrangementCancelFlag;
  bool quincunxSamplingFlag;
  bool spatialFlippingFlag;
  bool frame0FlippedFlag;
  bool fieldViewsFlag;
  bool currentFrameIsFrame0Flag;
  bool frame0SelfContainedFlag;
  bool frame1SelfContainedFlag;
  bool arrangementPersistenceFlag;
  bool upsampledAspectRatio;
  int  arrangementId;
  int  arrangementType;
  int  contentInterpretationType;
  int  frame0GridPositionX;
  int  frame0GridPositionY;
  int  frame1GridPositionX;
  int  frame1GridPositionY;
  int  arrangementReservedByte;
};

class SEIParameterSetsInclusionIndication : public SEI
{
public:
  PayloadType payloadType() const { return PARAMETER_SETS_INCLUSION_INDICATION; }
  SEIParameterSetsInclusionIndication() {}
  virtual ~SEIParameterSetsInclusionIndication() {}

  int selfContainedClvsFlag;
};

class SEIMasteringDisplayColourVolume : public SEI
{
public:
    PayloadType payloadType() const { return MASTERING_DISPLAY_COLOUR_VOLUME; }
    SEIMasteringDisplayColourVolume() {}
    virtual ~SEIMasteringDisplayColourVolume(){}

    SEIMasteringDisplay values;
};

typedef std::list<SEI*> SEIMessages;

/// output a selection of SEI messages by payload type. Ownership stays in original message list.
SEIMessages getSeisByType(const SEIMessages &seiList, SEI::PayloadType seiType);

/// remove a selection of SEI messages by payload type from the original list and return them in a new list.
SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// delete list of SEI messages (freeing the referenced objects)
void deleteSEIs (SEIMessages &seiList);

class SEIScalableNesting : public SEI
{
public:
  PayloadType payloadType() const { return SCALABLE_NESTING; }

  SEIScalableNesting()
  : snOlsFlag (false)
  , snSubpicFlag (false)
  , snNumOlssMinus1 (0)
  , snAllLayersFlag (false)
  , snNumLayersMinus1 (0)
  , snNumSubpics (1)
  , snSubpicIdLen (0)
  , snNumSEIs(0)
  {}

  virtual ~SEIScalableNesting()
  {
    deleteSEIs(nestedSEIs);
  }

  bool                  snOlsFlag;
  bool                  snSubpicFlag;
  uint32_t              snNumOlssMinus1;
  uint32_t              snOlsIdxDeltaMinus1[MAX_NESTING_NUM_LAYER];
  uint32_t              snOlsIdx[MAX_NESTING_NUM_LAYER];
  bool                  snAllLayersFlag;                      //value valid if m_nestingOlsFlag == 0
  uint32_t              snNumLayersMinus1;                    //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0
  uint8_t               snLayerId[MAX_NESTING_NUM_LAYER];     //value valid if m_nestingOlsFlag == 0 and m_nestingAllLayersFlag == 0. This can e.g. be a static array of 64 uint8_t values
  uint32_t              snNumSubpics;
  uint8_t               snSubpicIdLen;
  std::vector<uint16_t> snSubpicId;
  uint32_t              snNumSEIs;

  SEIMessages           nestedSEIs;
};

class SEIAlternativeTransferCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return ALTERNATIVE_TRANSFER_CHARACTERISTICS; }

  SEIAlternativeTransferCharacteristics() : preferredTransferCharacteristics(18)
  { }

  virtual ~SEIAlternativeTransferCharacteristics() {}

  uint32_t preferredTransferCharacteristics;
};

class SEIUserDataRegistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_REGISTERED_ITU_T_T35; }

  SEIUserDataRegistered() {}
  virtual ~SEIUserDataRegistered() {}

  uint16_t              ituCountryCode;
  std::vector<uint8_t>  userData;
};

class SEIFilmGrainCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return FILM_GRAIN_CHARACTERISTICS; }

  SEIFilmGrainCharacteristics() {}
  virtual ~SEIFilmGrainCharacteristics() {}

  bool      filmGrainCharacteristicsCancelFlag;
  uint8_t   filmGrainModelId;
  bool      separateColourDescriptionPresent;
  uint8_t   filmGrainBitDepthLumaMinus8;
  uint8_t   filmGrainBitDepthChromaMinus8;
  bool      filmGrainFullRangeFlag;
  uint8_t   filmGrainColourPrimaries;
  uint8_t   filmGrainTransferCharacteristics;
  uint8_t   filmGrainMatrixCoeffs;
  uint8_t   blendingModeId;
  uint8_t   log2ScaleFactor;

  struct CompModelIntensityValues
  {
    uint8_t intensityIntervalLowerBound;
    uint8_t intensityIntervalUpperBound;
    std::vector<int> compModelValue;
  };

  struct CompModel
  {
    bool     presentFlag;
    uint8_t  numModelValues;
    std::vector<CompModelIntensityValues> intensityValues;
  };

  CompModel compModel[MAX_NUM_COMP];
  bool      filmGrainCharacteristicsPersistenceFlag;
};

class SEIContentLightLevelInfo : public SEI
{
public:
  PayloadType payloadType() const { return CONTENT_LIGHT_LEVEL_INFO; }
  SEIContentLightLevelInfo() { }

  virtual ~SEIContentLightLevelInfo() { }

  uint32_t maxContentLightLevel;
  uint32_t maxPicAverageLightLevel;
};

class SEIAmbientViewingEnvironment : public SEI
{
public:
  PayloadType payloadType() const { return AMBIENT_VIEWING_ENVIRONMENT; }
  SEIAmbientViewingEnvironment() { }

  virtual ~SEIAmbientViewingEnvironment() { }

  uint32_t ambientIlluminance;
  uint16_t ambientLightX;
  uint16_t ambientLightY;
};

class SEIContentColourVolume : public SEI
{
public:
  PayloadType payloadType() const { return CONTENT_COLOUR_VOLUME; }
  SEIContentColourVolume() {}
  virtual ~SEIContentColourVolume() {}

  bool      ccvCancelFlag;
  bool      ccvPersistenceFlag;
  bool      ccvPrimariesPresent;
  bool      ccvMinLuminanceValuePresent;
  bool      ccvMaxLuminanceValuePresent;
  bool      ccvAvgLuminanceValuePresent;
  int       ccvPrimariesX[MAX_NUM_COMP];
  int       ccvPrimariesY[MAX_NUM_COMP];
  uint32_t  ccvMinLuminanceValue;
  uint32_t  ccvMaxLuminanceValue;
  uint32_t  ccvAvgLuminanceValue;
};

class SEISubpicureLevelInfo : public SEI
{
public:
  PayloadType payloadType() const { return SUBPICTURE_LEVEL_INFO; }
  SEISubpicureLevelInfo()
  : numRefLevels(0)
  , explicitFractionPresent (false)
  , cbrConstraintFlag (false)
  , numSubpics(0)
  , sliMaxSublayers(1)
  , sliSublayerInfoPresent(false)
  {}
  virtual ~SEISubpicureLevelInfo() {}

  int                                         numRefLevels;
  bool                                        explicitFractionPresent;
  bool                                        cbrConstraintFlag;
  int                                         numSubpics;
  int                                         sliMaxSublayers;
  bool                                        sliSublayerInfoPresent;
  std::vector<std::vector<int>>               nonSubpicLayersFraction;
  std::vector<std::vector<Level::Name>>       refLevelIdc;
  std::vector<std::vector<std::vector<int>>>  refLevelFraction;
};


} // namespace vvenc

//! \}


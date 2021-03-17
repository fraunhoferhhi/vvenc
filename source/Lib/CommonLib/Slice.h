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
/** \file     Slice.h
    \brief    slice header and SPS class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Rom.h"
#include "AlfParameters.h"
#include "Common.h"
#include "MotionInfo.h"
#include "HRD.h"

#include "vvenc/vvencCfgExpert.h"


#include <cstring>
#include <list>
#include <map>
#include <vector>

//! \ingroup CommonLib
//! \{

namespace vvenc {

struct  MotionInfo;
struct  Picture;
class   Pic;
class   TrQuant;
class   PreCalcValues;
typedef std::list<Picture*> PicList;

// ====================================================================================================================
// Constants
// ====================================================================================================================
static const uint32_t REF_PIC_LIST_NUM_IDX=32;


// ====================================================================================================================
// Class definition
// ====================================================================================================================
struct DpbParameters
{
  int maxDecPicBuffering     [VVENC_MAX_TLAYER] = { 0 };
  int numReorderPics         [VVENC_MAX_TLAYER] = { 0 };
  int maxLatencyIncreasePlus1[VVENC_MAX_TLAYER] = { 0 };
};

struct ReferencePictureList
{
  int       numberOfShorttermPictures;
  int       numberOfLongtermPictures;
  int       numberOfActivePictures;
  bool      isLongtermRefPic[VVENC_MAX_NUM_REF_PICS];
  int       refPicIdentifier[VVENC_MAX_NUM_REF_PICS];  //This can be delta POC for STRP or POC LSB for LTRP
  int       POC[VVENC_MAX_NUM_REF_PICS];
  uint32_t  deltaPocMSBCycleLT[VVENC_MAX_NUM_REF_PICS];
  bool      deltaPocMSBPresent[VVENC_MAX_NUM_REF_PICS];
  bool      ltrpInSliceHeader;
  bool      interLayerPresent;
  bool      isInterLayerRefPic[VVENC_MAX_NUM_REF_PICS];
  int       interLayerRefPicIdx[VVENC_MAX_NUM_REF_PICS];
  int       numberOfInterLayerPictures;

  ReferencePictureList();

  void     setRefPicIdentifier  ( int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx );
  int      getNumRefEntries     ()  const { return numberOfShorttermPictures + numberOfLongtermPictures; }
  bool     isPOCInRefPicList    ( const int poc, const int currPoc ) const;
};

/// Reference Picture List set class
typedef std::vector<ReferencePictureList> RPLList;

struct ConstraintInfo
{
  bool      gciPresent;
  bool      noRprConstraintFlag;
  bool      noResChangeInClvsConstraintFlag;
  bool      oneTilePerPicConstraintFlag;
  bool      picHeaderInSliceHeaderConstraintFlag;
  bool      oneSlicePerPicConstraintFlag;
  bool      noIdrRplConstraintFlag;
  bool      noRectSliceConstraintFlag;
  bool      oneSlicePerSubpicConstraintFlag;
  bool      noSubpicInfoConstraintFlag;
  bool      intraOnlyConstraintFlag;
  uint32_t  maxBitDepthConstraintIdc;
  int       maxChromaFormatConstraintIdc;
  bool      onePictureOnlyConstraintFlag;
  bool      lowerBitRateConstraintFlag;
  bool      allLayersIndependentConstraintFlag;
  bool      noMrlConstraintFlag;
  bool      noIspConstraintFlag;
  bool      noMipConstraintFlag;
  bool      noLfnstConstraintFlag;
  bool      noMmvdConstraintFlag;
  bool      noSmvdConstraintFlag;
  bool      noProfConstraintFlag;
  bool      noPaletteConstraintFlag;
  bool      noActConstraintFlag;
  bool      noLmcsConstraintFlag;
  bool      noExplicitScaleListConstraintFlag;
  bool      noVirtualBoundaryConstraintFlag;
  bool      noMttConstraintFlag;
  bool      noChromaQpOffsetConstraintFlag;
  bool      noQtbttDualTreeIntraConstraintFlag;
  int       maxLog2CtuSizeConstraintIdc;
  bool      noPartitionConstraintsOverrideConstraintFlag;
  bool      noSaoConstraintFlag;
  bool      noAlfConstraintFlag;
  bool      noCCAlfConstraintFlag;
  bool      noWeightedPredictionConstraintFlag;
  bool      noRefWraparoundConstraintFlag;
  bool      noTemporalMvpConstraintFlag;
  bool      noSbtmvpConstraintFlag;
  bool      noAmvrConstraintFlag;
  bool      noBdofConstraintFlag;
  bool      noDmvrConstraintFlag;
  bool      noCclmConstraintFlag;
  bool      noMtsConstraintFlag;
  bool      noSbtConstraintFlag;
  bool      noAffineMotionConstraintFlag;
  bool      noBcwConstraintFlag;
  bool      noIbcConstraintFlag;
  bool      noCiipConstraintFlag;
  bool      noGeoConstraintFlag;
  bool      noLadfConstraintFlag;
  bool      noTransformSkipConstraintFlag;
  bool      noLumaTransformSize64ConstraintFlag;
  bool      noBDPCMConstraintFlag;
  bool      noJointCbCrConstraintFlag;
  bool      noQpDeltaConstraintFlag;
  bool      noDepQuantConstraintFlag;
  bool      noSignDataHidingConstraintFlag;
  bool      noMixedNaluTypesInPicConstraintFlag;
  bool      noTrailConstraintFlag;
  bool      noStsaConstraintFlag;
  bool      noRaslConstraintFlag;
  bool      noRadlConstraintFlag;
  bool      noIdrConstraintFlag;
  bool      noCraConstraintFlag;
  bool      noGdrConstraintFlag;
  bool      noApsConstraintFlag;


  ConstraintInfo()
    : gciPresent                                      ( false )
    , noRprConstraintFlag                             ( false )
    , noResChangeInClvsConstraintFlag                 ( false )
    , oneTilePerPicConstraintFlag                     ( false )
    , picHeaderInSliceHeaderConstraintFlag            ( false )
    , oneSlicePerPicConstraintFlag                    ( false )
    , noIdrRplConstraintFlag                          ( false )
    , noRectSliceConstraintFlag                       ( false )
    , oneSlicePerSubpicConstraintFlag                 ( false )
    , noSubpicInfoConstraintFlag                      ( false )
    , intraOnlyConstraintFlag                         ( false )
    , maxBitDepthConstraintIdc                        ( false )
    , maxChromaFormatConstraintIdc                    ( CHROMA_420 )
    , onePictureOnlyConstraintFlag                    ( false )
    , lowerBitRateConstraintFlag                      ( false )
    , allLayersIndependentConstraintFlag              ( false )
    , noMrlConstraintFlag                             ( false )
    , noIspConstraintFlag                             ( false )
    , noMipConstraintFlag                             ( false )
    , noLfnstConstraintFlag                           ( false )
    , noMmvdConstraintFlag                            ( false )
    , noSmvdConstraintFlag                            ( false )
    , noProfConstraintFlag                            ( false )
    , noPaletteConstraintFlag                         ( false )
    , noActConstraintFlag                             ( false )
    , noLmcsConstraintFlag                            ( false )
    , noExplicitScaleListConstraintFlag               ( false )
    , noVirtualBoundaryConstraintFlag                 ( false )
    , noMttConstraintFlag                             ( false )
    , noChromaQpOffsetConstraintFlag                  ( false )
    , noQtbttDualTreeIntraConstraintFlag              ( false )
    , maxLog2CtuSizeConstraintIdc                     ( 0 )
    , noPartitionConstraintsOverrideConstraintFlag    ( false )
    , noSaoConstraintFlag                             ( false )
    , noAlfConstraintFlag                             ( false )
    , noCCAlfConstraintFlag                           ( false )
    , noWeightedPredictionConstraintFlag              ( false )
    , noRefWraparoundConstraintFlag                   ( false )
    , noTemporalMvpConstraintFlag                     ( false )
    , noSbtmvpConstraintFlag                          ( false )
    , noAmvrConstraintFlag                            ( false )
    , noBdofConstraintFlag                            ( false )
    , noDmvrConstraintFlag                            ( false )
    , noCclmConstraintFlag                            ( false )
    , noMtsConstraintFlag                             ( false )
    , noSbtConstraintFlag                             ( false )
    , noAffineMotionConstraintFlag                    ( false )
    , noBcwConstraintFlag                             ( false )
    , noIbcConstraintFlag                             ( false )
    , noCiipConstraintFlag                            ( false )
    , noGeoConstraintFlag                             ( false )
    , noLadfConstraintFlag                            ( false )
    , noTransformSkipConstraintFlag                   ( false )
    , noLumaTransformSize64ConstraintFlag             ( false )
    , noBDPCMConstraintFlag                           ( false )
    , noJointCbCrConstraintFlag                       ( false )
    , noQpDeltaConstraintFlag                         ( false )
    , noDepQuantConstraintFlag                        ( false )
    , noSignDataHidingConstraintFlag                  ( false )
    , noMixedNaluTypesInPicConstraintFlag             ( false )
    , noTrailConstraintFlag                           ( false )
    , noStsaConstraintFlag                            ( false )
    , noRaslConstraintFlag                            ( false )
    , noRadlConstraintFlag                            ( false )
    , noIdrConstraintFlag                             ( false )
    , noCraConstraintFlag                             ( false )
    , noGdrConstraintFlag                             ( false )
    , noApsConstraintFlag                             ( false )
  {}
};

struct ProfileTierLevel
{
  vvencTier             tierFlag;
  vvencProfile          profileIdc;
  uint8_t               numSubProfile;
  std::vector<uint32_t> subProfileIdc;
  vvencLevel                 levelIdc;
  bool                  frameOnlyConstraintFlag;
  bool                  multiLayerEnabledFlag;
  ConstraintInfo        constraintInfo;
  bool                  subLayerLevelPresent[VVENC_MAX_TLAYER - 1];
  vvencLevel            subLayerLevelIdc[VVENC_MAX_TLAYER - 1];

  ProfileTierLevel()
    : tierFlag        ( VVENC_TIER_MAIN )
    , profileIdc      ( VVENC_PROFILE_AUTO )
    , numSubProfile   (0)
    , subProfileIdc   (0)
    , levelIdc        ( VVENC_LEVEL_AUTO )
    , frameOnlyConstraintFlag ( true )
    , multiLayerEnabledFlag   ( false )
  {
    ::memset( subLayerLevelPresent, 0, sizeof(subLayerLevelPresent ));
    ::memset( subLayerLevelIdc,     0, sizeof(subLayerLevelIdc ));
  }
};


struct LmcsParam
{
  LmcsParam()
  : sliceReshaperEnabled( false )
  , sliceReshaperModelPresent( false )
  , enableChromaAdj(0)
  , reshaperModelMinBinIdx(0)
  , reshaperModelMaxBinIdx(0)
  , reshaperModelBinCWDelta { 0 }
  , maxNbitsNeededDeltaCW (0)
  , chrResScalingOffset (0)
  {
  }

  bool      sliceReshaperEnabled;
  bool      sliceReshaperModelPresent;
  unsigned  enableChromaAdj;
  uint32_t  reshaperModelMinBinIdx;
  uint32_t  reshaperModelMaxBinIdx;
  int       reshaperModelBinCWDelta[PIC_CODE_CW_BINS];
  int       maxNbitsNeededDeltaCW;
  int       chrResScalingOffset;
};


struct ChromaQpAdj
{
  union
  {
    struct {
      int CbOffset;
      int CrOffset;
      int JointCbCrOffset;
    } comp;
    int offset[3];
  } u;
};

struct ChromaQpMappingTable : vvencChromaQpMappingTableParams
{
  int       getMappedChromaQpValue(ComponentID compID, const int qpVal)  const { return m_chromaQpMappingTables[m_sameCQPTableForAllChromaFlag ? 0 : (int)compID - 1].at(qpVal + m_qpBdOffset); }
  void      derivedChromaQPMappingTables();
  void      setParams(const vvencChromaQpMappingTableParams &params, const int qpBdOffset);
private:
  std::vector<int> m_chromaQpMappingTables[VVENC_MAX_NUM_CQP_MAPPING_TABLES];
};

struct SliceMap
{
  uint32_t               sliceID;                           //!< slice identifier (slice index for rectangular slices, slice address for raser-scan slices)
  uint32_t               numTilesInSlice;                   //!< number of tiles in slice (raster-scan slices only)
  uint32_t               numCtuInSlice;                     //!< number of CTUs in the slice
  std::vector<uint32_t>  ctuAddrInSlice;                    //!< raster-scan addresses of all the CTUs in the slice

  SliceMap()
   : sliceID            ( 0 )
   , numTilesInSlice    ( 0 )
   , numCtuInSlice      ( 0 )
   , ctuAddrInSlice     ( )
  {
  }

  void  addCtusToSlice( uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY )
  {
    CHECK( startX >= stopX || startY >= stopY, "Invalid slice definition");
    for( uint32_t ctbY = startY; ctbY < stopY; ctbY++ )
    {
      for( uint32_t ctbX = startX; ctbX < stopX; ctbX++ )
      {
        ctuAddrInSlice.push_back( ctbY * picWidthInCtbsY + ctbX );
        numCtuInSlice++;
      }
    }
  }
};

struct RectSlice
{
  uint32_t         tileIdx;                           //!< tile index corresponding to the first CTU in the slice
  uint32_t         sliceWidthInTiles;                 //!< slice width in units of tiles
  uint32_t         sliceHeightInTiles;                //!< slice height in units of tiles
  uint32_t         numSlicesInTile;                   //!< number of slices in current tile for the special case of multiple slices inside a single tile
  uint32_t         sliceHeightInCtu;                  //!< slice height in units of CTUs for the special case of multiple slices inside a single tile

  RectSlice()  {}
};

struct SubPic
{
  uint32_t         subPicID;                                  //!< ID of subpicture
  uint32_t         subPicIdx;                                 //!< Index of subpicture
  uint32_t         numCTUsInSubPic;                           //!< number of CTUs contained in this sub-picture
  uint32_t         subPicCtuTopLeftX;                         //!< horizontal position of top left CTU of the subpicture in unit of CTU
  uint32_t         subPicCtuTopLeftY;                         //!< vertical position of top left CTU of the subpicture in unit of CTU
  uint32_t         subPicWidth;                               //!< the width of subpicture in units of CTU
  uint32_t         subPicHeight;                              //!< the height of subpicture in units of CTU
  uint32_t         subPicWidthInLumaSample;                   //!< the width of subpicture in units of luma sample
  uint32_t         subPicHeightInLumaSample;                  //!< the height of subpicture in units of luma sample
  uint32_t         firstCtuInSubPic;                          //!< the raster scan index of the first CTU in a subpicture
  uint32_t         lastCtuInSubPic;                           //!< the raster scan index of the last CTU in a subpicture
  uint32_t         subPicLeft;                                //!< the position of left boundary
  uint32_t         subPicRight;                               //!< the position of right boundary
  uint32_t         subPicTop;                                 //!< the position of top boundary
  uint32_t         subPicBottom;                              //!< the position of bottom boundary
  std::vector<uint32_t> ctuAddrInSubPic;                      //!< raster scan addresses of all the CTUs in the slice

  bool             treatedAsPic;                          //!< whether the subpicture is treated as a picture in the decoding process excluding in-loop filtering operations
  bool             loopFilterAcrossSubPicEnabled;         //!< whether in-loop filtering operations may be performed across the boundaries of the subpicture
  uint32_t         numSlicesInSubPic;                         //!< Number of slices contained in this subpicture

  bool             isContainingPos(const Position& pos) const
  {
    return pos.x >= subPicLeft && pos.x <= subPicRight && pos.y >= subPicTop  && pos.y <= subPicBottom;
  }

  void init( unsigned picWithInCtu, unsigned picHeightInCtu, unsigned picWithInSamples, unsigned picHeighthInSamples )
  {
    unsigned numCtus          = picWithInCtu * picHeightInCtu;
    numCTUsInSubPic           = numCtus;
    subPicWidth               = picWithInCtu;
    subPicHeight              = picHeightInCtu;
    subPicWidthInLumaSample   = picWithInSamples;
    subPicHeightInLumaSample  = picHeighthInSamples;
    lastCtuInSubPic           = numCtus;
    subPicRight               = picWithInSamples-1;
    subPicBottom              = picHeighthInSamples-1;
    ctuAddrInSubPic.resize( numCtus );
    for( unsigned i = 0; i < numCtus; i++)
    {
      ctuAddrInSubPic[i] = i;
    }
  }

  SubPic()
  : subPicID(0)
  , subPicIdx(0)
  , numCTUsInSubPic(-1)
  , subPicCtuTopLeftX(0)
  , subPicCtuTopLeftY(0)
  , subPicWidth(-1)
  , subPicHeight(-1)
  , subPicWidthInLumaSample(-1)
  , subPicHeightInLumaSample(-1)
  , firstCtuInSubPic(0)
  , lastCtuInSubPic(-1)
  , subPicLeft(0)
  , subPicRight(0)
  , subPicTop(0)
  , subPicBottom(0)
  , treatedAsPic(true)
  , loopFilterAcrossSubPicEnabled(true)
  , numSlicesInSubPic(1)
  {
  }
};

struct DCI
{
  uint32_t   dciId;
  std::vector<ProfileTierLevel> profileTierLevel;

  DCI() : dciId(0) {}
};

struct VPS
{
  uint32_t              vpsId;
  uint32_t              maxLayers;
  uint32_t              maxSubLayers;
  uint32_t              layerId[MAX_VPS_LAYERS];
  bool                  defaultPtlDpbHrdMaxTidFlag;
  bool                  allLayersSameNumSubLayers;
  bool                  allIndependentLayers;
  uint32_t              vpsCfgPredDirection[MAX_VPS_SUBLAYERS];
  bool                  independentLayer[MAX_VPS_LAYERS];
  bool                  directRefLayer[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  uint8_t               maxTidIlRefPicsPlus1[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  bool                  eachLayerIsAnOls;
  uint32_t              olsModeIdc;
  uint32_t              numOutputLayerSets;
  bool                  olsOutputLayer[MAX_NUM_OLSS][MAX_VPS_LAYERS];
  uint32_t              directRefLayerIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  uint32_t              generalLayerIdx[MAX_VPS_LAYERS];
  uint32_t              numPtls;
  bool                  ptPresent[MAX_NUM_OLSS];
  uint32_t              ptlMaxTemporalId[MAX_NUM_OLSS];
  std::vector<ProfileTierLevel> profileTierLevel;
  uint32_t              olsPtlIdx[MAX_NUM_OLSS];

  // stores index ( ilrp_idx within 0 .. NumDirectRefLayers ) of the dependent reference layers
  uint32_t              interLayerRefIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  bool                  extension;

  bool                  generalHrdParamsPresent;
  bool                  sublayerCpbParamsPresent;
  uint32_t              numOlsHrdParamsMinus1;
  uint32_t              hrdMaxTid[MAX_NUM_OLSS];
  uint32_t              olsHrdIdx[MAX_NUM_OLSS];
  GeneralHrdParams      generalHrdParams;
  OlsHrdParams          olsHrdParams[VVENC_MAX_TLAYER];
  std::vector<Size>             olsDpbPicSize;
  std::vector<int>              olsDpbParamsIdx;
  std::vector<std::vector<int>> outputLayerIdInOls;
  std::vector<std::vector<int>> numSubLayersInLayerInOLS;

  std::vector<int>              olsDpbChromaFormatIdc;
  std::vector<int>              olsDpbBitDepthMinus8;

  int                           totalNumOLSs;
  int                           numMultiLayeredOlss;
  uint32_t                      multiLayerOlsIdx[MAX_NUM_OLSS];
  int                           numDpbParams;
  std::vector<DpbParameters>    dpbParameters;
  bool                          sublayerDpbParamsPresent;
  std::vector<int>              dpbMaxTemporalId;
  std::vector<int>              targetOutputLayerIdSet;          ///< set of LayerIds to be outputted
  std::vector<int>              targetLayerIdSet;                ///< set of LayerIds to be included in the sub-bitstream extraction process.
  int                           targetOlsIdx;
  std::vector<int>              numOutputLayersInOls;
  std::vector<int>              numLayersInOls;
  std::vector<std::vector<int>> layerIdInOls;

  VPS()
  : vpsId                       ( 0 )
  , maxLayers                   ( 0 )
  , maxSubLayers                ( 0 )
  , defaultPtlDpbHrdMaxTidFlag  ( false )
  , allLayersSameNumSubLayers   ( false )
  , allIndependentLayers        ( false )
  , eachLayerIsAnOls            ( false )
  , olsModeIdc                  ( 0 )
  , numOutputLayerSets          ( 0 )
  , numPtls                     ( 0 )
  , extension                   ( false )
  , totalNumOLSs                ( 0 )
  , numDpbParams                ( 0 )
  , sublayerDpbParamsPresent    ( false)
  , targetOlsIdx                ( 0 )
  {
    memset( layerId,            0, sizeof(layerId));
    memset( independentLayer,   0, sizeof(independentLayer));
    memset( directRefLayer,     0, sizeof(directRefLayer));
    memset( maxTidIlRefPicsPlus1, 7, sizeof(maxTidIlRefPicsPlus1));
    memset( olsOutputLayer,     0, sizeof(olsOutputLayer));
    memset( directRefLayerIdx,  0, sizeof(directRefLayerIdx));
    memset( generalLayerIdx,    0, sizeof(generalLayerIdx));
    memset( ptPresent,          0, sizeof(ptPresent));
    memset( ptlMaxTemporalId,   0, sizeof(ptlMaxTemporalId));
    memset( olsPtlIdx,          0, sizeof(olsPtlIdx));
    memset( interLayerRefIdx,   0, sizeof(interLayerRefIdx));
  }
  void deriveOutputLayerSets();
  int               getMaxDecPicBuffering( int temporalId ) const        { return dpbParameters[olsDpbParamsIdx[targetOlsIdx]].maxDecPicBuffering[temporalId]; }
  int               getNumReorderPics( int temporalId ) const            { return dpbParameters[olsDpbParamsIdx[targetOlsIdx]].numReorderPics[temporalId]; }

};

struct Window
{
  bool enabledFlag;
  int  winLeftOffset;
  int  winRightOffset;
  int  winTopOffset;
  int  winBottomOffset;

  Window()
  : enabledFlag    (false)
  , winLeftOffset  (0)
  , winRightOffset (0)
  , winTopOffset   (0)
  , winBottomOffset(0)
  { }

  void setWindow(int offsetLeft, int offsetLRight, int offsetLTop, int offsetLBottom)
  {
    enabledFlag     = offsetLeft || offsetLRight || offsetLTop || offsetLBottom;
    winLeftOffset   = offsetLeft;
    winRightOffset  = offsetLRight;
    winTopOffset    = offsetLTop;
    winBottomOffset = offsetLBottom;
  }

  bool operator!=( const Window& other ) const
  {
    return (winLeftOffset   != other.winLeftOffset
         || winRightOffset  != other.winRightOffset
         || winTopOffset    != other.winTopOffset
         || winBottomOffset != other.winBottomOffset);
  }

};


struct VUI
{
  bool       progressiveSourceFlag;
  bool       interlacedSourceFlag;
  bool       nonPackedFlag;
  bool       nonProjectedFlag;
  bool       aspectRatioInfoPresent;
  bool       aspectRatioConstantFlag;
  uint32_t   aspectRatioIdc;
  uint32_t   sarWidth;
  uint32_t   sarHeight;
  bool       overscanInfoPresent;
  bool       overscanAppropriateFlag;
  bool       colourDescriptionPresent;
  uint32_t   colourPrimaries;
  uint32_t   transferCharacteristics;
  uint32_t   matrixCoefficients;
  bool       videoFullRangeFlag;
  bool       chromaLocInfoPresent;
  uint32_t   chromaSampleLocTypeTopField;
  uint32_t   chromaSampleLocTypeBottomField;
  uint32_t   chromaSampleLocType;

  VUI()
    : progressiveSourceFlag         (false)
    , interlacedSourceFlag          (false)
    , nonPackedFlag                 (false)
    , nonProjectedFlag              (false)
    , aspectRatioInfoPresent        (false) //TODO: This initialiser list contains magic numbers
    , aspectRatioConstantFlag       (false)
    , aspectRatioIdc                    (0)
    , sarWidth                          (0)
    , sarHeight                         (0)
    , overscanInfoPresent           (false)
    , overscanAppropriateFlag       (false)
    , colourDescriptionPresent      (false)
    , colourPrimaries                   (2)
    , transferCharacteristics           (2)
    , matrixCoefficients                (2)
    , videoFullRangeFlag            (false)
    , chromaLocInfoPresent          (false)
    , chromaSampleLocTypeTopField       (0)
    , chromaSampleLocTypeBottomField    (0)
    , chromaSampleLocType               (0)
  {}
};

/// SPS RExt class
struct SPSRExt // Names aligned to text specification
{
  bool             transformSkipRotationEnabled;
  bool             transformSkipContextEnabled;
  bool             implicitRdpcmEnabled;
  bool             explicitRdpcmEnabled;
  bool             extendedPrecisionProcessing;
  bool             intraSmoothingDisabled;
  bool             highPrecisionOffsetsEnabled;
  bool             persistentRiceAdaptationEnabled;
  bool             cabacBypassAlignmentEnabled;

  SPSRExt();

  bool settingsDifferFromDefaults() const
  {
    return transformSkipRotationEnabled
        || transformSkipContextEnabled
        || implicitRdpcmEnabled
        || explicitRdpcmEnabled
        || extendedPrecisionProcessing
        || intraSmoothingDisabled
        || highPrecisionOffsetsEnabled
        || persistentRiceAdaptationEnabled
        || cabacBypassAlignmentEnabled;
  }
};


/// SPS class
struct SPS
{
  int               spsId;
  int               dciId;
  int               vpsId;
  int               layerId;

  bool              AffineAmvr;
  bool              DMVR;
  bool              MMVD;
  bool              SBT;
  bool              ISP;
  ChromaFormat      chromaFormatIdc;
  bool              separateColourPlane;     //!< separate colour plane
  uint32_t          maxTLayers;           // maximum number of temporal layers
  bool              ptlDpbHrdParamsPresent;
  bool              subLayerDpbParams;

  // Structure
  uint32_t          maxPicWidthInLumaSamples;
  uint32_t          maxPicHeightInLumaSamples;
  Window            conformanceWindow;
  bool              subPicInfoPresent;
  uint8_t           numSubPics;                        //!< number of sub-pictures used
  bool              independentSubPicsFlag;
  uint32_t          subPicCtuTopLeftX[MAX_NUM_SUB_PICS];
  uint32_t          subPicCtuTopLeftY[MAX_NUM_SUB_PICS];
  uint32_t          subPicWidth[MAX_NUM_SUB_PICS];
  uint32_t          subPicHeight[MAX_NUM_SUB_PICS];
  bool              subPicTreatedAsPic[MAX_NUM_SUB_PICS];
  bool              loopFilterAcrossSubpicEnabled[MAX_NUM_SUB_PICS];
  bool              subPicIdMappingExplicitlySignalled;
  uint32_t          subPicIdLen;                       //!< sub-picture ID length in bits
  uint8_t           subPicId[MAX_NUM_SUB_PICS];        //!< sub-picture ID for each sub-picture in the sequence

  int               log2MinCodingBlockSize;
  unsigned          CTUSize;
  unsigned          partitionOverrideEnabled;       // enable partition constraints override function
  unsigned          minQTSize[3];   // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned          maxMTTDepth[3];
  unsigned          maxBTSize[3];
  unsigned          maxTTSize[3];
  bool              idrRefParamList;
  unsigned          dualITree;

  RPLList           rplList[NUM_REF_PIC_LIST_01];
  bool              rpl1CopyFromRpl0;
  bool              rpl1IdxPresent;
  bool              allRplEntriesHasSameSign;
  bool              longTermRefsPresent;
  bool              temporalMVPEnabled;
  int               numReorderPics[VVENC_MAX_TLAYER];

  // Tool list
  bool              transformSkip;
  int               log2MaxTransformSkipBlockSize;
  bool              BDPCM;
  bool              jointCbCr;
  // Parameter
  BitDepths         bitDepths;
  bool              entropyCodingSyncEnabled;
  bool              entryPointsPresent;
  int               qpBDOffset[MAX_NUM_CH];
  int               internalMinusInputBitDepth[MAX_NUM_CH]; //  max(0, internal bitdepth - input bitdepth);

  bool              SbtMvp;
  bool              BDOF;
  bool              fpelMmvd;
  bool              BdofPresent;
  bool              DmvrPresent;
  bool              ProfPresent;
  uint32_t          bitsForPOC;

  bool              pocMsbFlag;
  uint32_t          pocMsbLen;
  int               numExtraPHBitsBytes;
  int               numExtraSHBitsBytes;
  std::vector<bool> extraPHBitPresent;
  std::vector<bool> extraSHBitPresent;

  uint32_t          numLongTermRefPicSPS;
  uint32_t          ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  bool              usedByCurrPicLtSPS[MAX_NUM_LONG_TERM_REF_PICS];
  uint32_t          log2MaxTbSize;
  bool              weightPred;                     //!< Use of Weighting Prediction (P_SLICE)
  bool              weightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)

  bool              saoEnabled;

  bool              temporalIdNesting; // temporal_id_nesting_flag

  bool              scalingListEnabled;
  bool              depQuantEnabled;            //!< dependent quantization enabled flag
  bool              signDataHidingEnabled;      //!< sign data hiding enabled flag
  bool              virtualBoundariesEnabled;   //!< Enable virtual boundaries tool
  bool              virtualBoundariesPresent    ;   //!< disable loop filtering across virtual boundaries
  unsigned          numVerVirtualBoundaries;                         //!< number of vertical virtual boundaries
  unsigned          numHorVirtualBoundaries;                         //!< number of horizontal virtual boundaries
  unsigned          virtualBoundariesPosX[3];                        //!< horizontal position of each vertical virtual boundary
  unsigned          virtualBoundariesPosY[3];                        //!< vertical position of each horizontal virtual boundary
  uint32_t          maxDecPicBuffering[VVENC_MAX_TLAYER];
  uint32_t          maxLatencyIncreasePlus1[VVENC_MAX_TLAYER];


  bool              hrdParametersPresent;
  bool              subLayerParametersPresent;
  GeneralHrdParams  generalHrdParams;
  OlsHrdParams      olsHrdParams[VVENC_MAX_TLAYER];
  bool              fieldSeqFlag;
  bool              vuiParametersPresent;
  VUI               vuiParameters;

  SPSRExt           spsRExt;

//  TimingInfo        timingInfo;
  ProfileTierLevel  profileTierLevel;

  bool              alfEnabled;
  bool              ccalfEnabled;
  bool              wrapAroundEnabled;
  bool              IBC;
  bool              useColorTrans;
  bool              PLT;

  bool              lumaReshapeEnable;
  bool              AMVR;
  bool              LMChroma;
  bool              horCollocatedChroma;
  bool              verCollocatedChroma;
  bool              MTS;
  bool              MTSIntra;                   // 18
  bool              MTSInter;                   // 19
  bool              LFNST;
  bool              SMVD;
  bool              Affine;
  bool              AffineType;
  bool              PROF;
  bool              BCW;
  bool              CIIP;
  bool              GEO;
  bool              LADF;
  bool              MRL;
  bool              MIP;
  ChromaQpMappingTable chromaQpMappingTable;
  bool              GDR;
  bool              subLayerCbpParametersPresent;

  bool              rprEnabled;
  bool              resChangeInClvsEnabled;
  bool              interLayerPresent;

  uint32_t          log2ParallelMergeLevelMinus2; // th fix this
  uint32_t          maxNumMergeCand;
  uint32_t          maxNumAffineMergeCand;
  uint32_t          maxNumIBCMergeCand;
  uint32_t          maxNumGeoCand;
  bool              scalingMatrixAlternativeColourSpaceDisabled;
  bool              scalingMatrixDesignatedColourSpace;
  bool              disableScalingMatrixForLfnstBlks;

  SPS();
  int               getNumRPL( int idx) const { return (int)rplList[idx].size()-1;}
  int               getMaxLog2TrDynamicRange(ChannelType channelType)                   const { return spsRExt.extendedPrecisionProcessing ? std::max<int>(15, int(bitDepths[channelType] + 6)) : 15; }
  uint32_t          getMaxTbSize()                                                      const { return  1 << log2MaxTbSize;                                        }
  bool              getUseImplicitMTS     ()                                            const { return MTS && !MTSIntra; }

  static int        getWinUnitX (int chromaFormatIdc)
  {
    CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter");
    return (chromaFormatIdc == 1 || chromaFormatIdc == 2) ? 2 : 1;
  }
  static int        getWinUnitY (int chromaFormatIdc)
  {
    CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter");
    return (chromaFormatIdc == 1) ? 2 : 1;
  }

private:
  static const int  m_winUnitX[NUM_CHROMA_FORMAT];
  static const int  m_winUnitY[NUM_CHROMA_FORMAT];
};


/// Reference Picture Lists class

/// PPS class
struct PPS
{
  int              ppsId;                    // pic_parameter_set_id
  int              spsId;                    // seq_parameter_set_id
  int              picInitQPMinus26;
  bool             useDQP;
  bool             usePPSChromaTool;
  bool             sliceChromaQpFlag;       // slicelevel_chroma_qp_flag
  int              layerId;
  int              temporalId;

  // access channel
  int              chromaQpOffset[MAX_NUM_COMP+1]; //also includes JointCbCr
  bool             jointCbCrQpOffsetPresent;
  ChromaQpAdj      chromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  // Chroma QP Adjustments
  int              chromaQpOffsetListLen; // size (excludes the null entry used in the following array).
  ChromaQpAdj      ChromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  uint32_t         numRefIdxL0DefaultActive;
  uint32_t         numRefIdxL1DefaultActive;

  bool             rpl1IdxPresent;

  bool             weightPred;                    //!< Use of Weighting Prediction (P_SLICE)
  bool             weightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)
  bool             outputFlagPresent;             //!< Indicates the presence of output_flag in slice header
  uint8_t          numSubPics;                        //!< number of sub-p
  bool             subPicIdMappingInPps;
  uint32_t         subPicIdLen;                       //!< sub-picture ID length in bits
  uint8_t          subPicId[MAX_NUM_SUB_PICS];        //!< sub-picture ID for each sub-picture in the sequence
  bool             noPicPartition;                //!< no picture partitioning flag - single slice, single tile
  uint8_t          log2CtuSize;                       //!< log2 of the CTU size - required to match corresponding value in SPS
  uint8_t          ctuSize;                           //!< CTU size
  uint32_t         picWidthInCtu;                     //!< picture width in units of CTUs
  uint32_t         picHeightInCtu;                    //!< picture height in units of CTUs
  uint32_t         numExpTileCols;                    //!< number of explicitly specified tile columns
  uint32_t         numExpTileRows;                    //!< number of explicitly specified tile rows
  uint32_t         numTileCols;                       //!< number of tile columns
  uint32_t         numTileRows;                       //!< number of tile rows

  std::vector<uint32_t>  tileColWidth;                 //!< tile column widths in units of CTUs
  std::vector<uint32_t>  tileRowHeight;                //!< tile row heights in units of CTUs
  std::vector<uint32_t>  tileColBd;                    //!< tile column left-boundaries in units of CTUs
  std::vector<uint32_t>  tileRowBd;                    //!< tile row top-boundaries in units of CTUs
  std::vector<uint32_t>  ctuToTileCol;                 //!< mapping between CTU horizontal address and tile column index
  std::vector<uint32_t>  ctuToTileRow;                 //!< mapping between CTU vertical address and tile row index
  bool                   rectSlice;                         //!< rectangular slice flag
  bool                   singleSlicePerSubPic;          //!< single slice per sub-picture flag
  std::vector<uint32_t>  ctuToSubPicIdx;               //!< mapping between CTU and Sub-picture index
  uint32_t               numSlicesInPic;                    //!< number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool                   tileIdxDeltaPresent;               //!< tile index delta present flag
  std::vector<RectSlice> rectSlices;                  //!< list of rectangular slice signalling parameters
  std::vector<SliceMap>  sliceMap;                    //!< list of CTU maps for each slice in the picture
  std::vector<SubPic>    subPics;                   //!< list of subpictures in the picture

  bool             loopFilterAcrossTilesEnabled;        //!< loop filtering applied across tiles flag
  bool             loopFilterAcrossSlicesEnabled;       //!< loop filtering applied across slices flag

  bool             cabacInitPresent;
  bool             pictureHeaderExtensionPresent;   //< picture header extension flags present in picture headers or not

  bool             sliceHeaderExtensionPresent;
  bool             deblockingFilterControlPresent;
  bool             deblockingFilterOverrideEnabled;
  bool             deblockingFilterDisabled;
  int              deblockingFilterBetaOffsetDiv2[MAX_NUM_COMP];    //< beta offset for deblocking filter
  int              deblockingFilterTcOffsetDiv2[MAX_NUM_COMP];      //< tc offset for deblocking filter
  bool             listsModificationPresent;
  bool             rplInfoInPh;
  bool             dbfInfoInPh;
  bool             saoInfoInPh;
  bool             alfInfoInPh;
  bool             wpInfoInPh;
  bool             qpDeltaInfoInPh;
  bool             mixedNaluTypesInPic;
  uint32_t         picWidthInLumaSamples;
  uint32_t         picHeightInLumaSamples;
  Window           conformanceWindow;
  Window           scalingWindow;
  bool             wrapAroundEnabled;               //< reference wrap around enabled or not
  unsigned         picWidthMinusWrapAroundOffset;          // <pic_width_in_minCbSizeY - wraparound_offset_in_minCbSizeY
  unsigned         wrapAroundOffset;                    //< reference wrap around offset in luma samples

  PreCalcValues   *pcv;

public:

                  PPS();
  virtual         ~PPS();
  uint32_t        getNumTiles() const;
  uint32_t        getSubPicIdxFromSubPicId( uint32_t subPicId ) const;
  const ChromaQpAdj&     getChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1 ) const
  {
    CHECK(cuChromaQpOffsetIdxPlus1 >= chromaQpOffsetListLen+1, "Invalid chroma QP offset");
    return chromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1]; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  }


  void                   setChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1, int cbOffset, int crOffset, int jointCbCrOffset )
  {
    CHECK(cuChromaQpOffsetIdxPlus1 == 0 || cuChromaQpOffsetIdxPlus1 > MAX_QP_OFFSET_LIST_SIZE, "Invalid chroma QP offset");
    chromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CbOffset = cbOffset; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
    chromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CrOffset = crOffset;
    chromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.JointCbCrOffset = jointCbCrOffset;
    chromaQpOffsetListLen = std::max(chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1);
  }


  bool               isRefScaled( const PPS& pps ) const                             { return  picWidthInLumaSamples  != pps.picWidthInLumaSamples        ||
                                                                                               picHeightInLumaSamples != pps.picHeightInLumaSamples       ||
                                                                                               scalingWindow          != pps.scalingWindow; }


  uint32_t               getTileIdx( const Position& pos ) const                          { return 0; } //tbd
  const SubPic&          getSubPicFromPos(const Position& pos)  const;
  const SubPic&          getSubPicFromCU (const CodingUnit& cu) const;

  void resetTileSliceInfo();
  void initTiles();
  void initRectSlices();

};

struct APS
{
  uint32_t               apsId;                    // adaptation_parameter_set_id
  int                    temporalId;
  int                    layerId;
  uint32_t               apsType;                  // aps_params_type
  AlfParam               alfParam;
  LmcsParam              lmcsParam;
  CcAlfFilterParam       ccAlfParam;
  bool                   hasPrefixNalUnitType;
  bool                   chromaPresent;
  int                    poc;
  APS() : apsId(0), temporalId( 0 ), layerId( 0 ), apsType(0), hasPrefixNalUnitType(false), chromaPresent( false ), poc(-1)
  { }
};

struct WPScalingParam
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  bool      presentFlag;
  uint32_t  log2WeightDenom;
  int       iWeight;
  int       iOffset;

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  int       w;
  int       o;
  int       offset;
  int       shift;
  int       round;
};

struct WPACDCParam
{
  int64_t iAC;
  int64_t iDC;
};

 // picture header class
struct PicHeader
{
  Picture*                    pic;                                                    //!< pointer to picture structure
  int                         pocLsb;                                                 //!< least significant bits of picture order count
  bool                        nonRefPic;                                              //!< non-reference picture flag
  bool                        gdrOrIrapPic;                                           //!< gdr or irap picture flag
  bool                        gdrPic;                                                 //!< gradual decoding refresh picture flag
  bool                        noOutputOfPriorPics;                                    //!< no output of prior pictures flag
  uint32_t                    recoveryPocCnt;                                         //!< recovery POC count
  bool                        noOutputBeforeRecovery;                             //!< NoOutputBeforeRecoveryFlag
  bool                        handleCraAsCvsStart;                                //!< HandleCraAsCvsStartFlag
  bool                        handleGdrAsCvsStart;                                //!< HandleGdrAsCvsStartFlag
  int                         spsId;                                                  //!< sequence parameter set ID
  int                         ppsId;                                                  //!< picture parameter set ID
  bool                        pocMsbPresent;                                          //!< ph_poc_msb_present_flag
  int                         pocMsbVal;                                              //!< poc_msb_val
  bool                        virtualBoundariesEnabled;              //!< loop filtering across virtual boundaries disabled
  bool                        virtualBoundariesPresent;              //!< loop filtering across virtual boundaries disabled
  unsigned                    numVerVirtualBoundaries;                                //!< number of vertical virtual boundaries
  unsigned                    numHorVirtualBoundaries;                                //!< number of horizontal virtual boundaries
  unsigned                    virtualBoundariesPosX[3];                               //!< horizontal virtual boundary positions
  unsigned                    virtualBoundariesPosY[3];                               //!< vertical virtual boundary positions
  bool                        picOutputFlag;                                          //!< picture output flag
  const ReferencePictureList* pRPL[NUM_REF_PIC_LIST_01];                              //!< pointer to RPL for L0, either in the SPS or the local RPS in the picture header
  ReferencePictureList        localRPL[NUM_REF_PIC_LIST_01];                          //!< RPL for L0 when present in picture header
  int                         rplIdx[NUM_REF_PIC_LIST_01];                            //!< index of used RPL in the SPS or -1 for local RPL in the picture header
  bool                        picInterSliceAllowed;                                   //!< inter slice allowed flag in PH
  bool                        picIntraSliceAllowed;                                   //!< intra slice allowed flag in PH
  bool                        splitConsOverride;                                      //!< partitioning constraint override flag
  uint32_t                    cuQpDeltaSubdivIntra;                                   //!< CU QP delta maximum subdivision for intra slices
  uint32_t                    cuQpDeltaSubdivInter;                                   //!< CU QP delta maximum subdivision for inter slices
  uint32_t                    cuChromaQpOffsetSubdivIntra;                            //!< CU chroma QP offset maximum subdivision for intra slices
  uint32_t                    cuChromaQpOffsetSubdivInter;                            //!< CU chroma QP offset maximum subdivision for inter slices
  bool                        enableTMVP;                                             //!< enable temporal motion vector prediction
  bool                        picColFromL0;                                           //!< syntax element collocated_from_l0_flag
  uint32_t                    colRefIdx;
  bool                        mvdL1Zero;                                              //!< L1 MVD set to zero flag
  uint32_t                    maxNumAffineMergeCand;                                  //!< max number of sub-block merge candidates
  bool                        disFracMMVD;                                            //!< fractional MMVD offsets disabled flag
  bool                        disBdofFlag;                                            //!< picture level BDOF disable flag
  bool                        disDmvrFlag;                                            //!< picture level DMVR disable flag
  bool                        disProfFlag;                                            //!< picture level PROF disable flag
  bool                        jointCbCrSign;                                          //!< joint Cb/Cr residual sign flag
  int                         qpDelta;                                                //!< value of Qp delta
  bool                        saoEnabled[MAX_NUM_CH];                                 //!< sao enabled flags for each channel
  bool                        alfEnabled[MAX_NUM_COMP];                               //!< alf enabled flags for each component
  int                         numAlfAps;                                              //!< number of alf aps active for the picture
  std::vector<int>            alfApsId;                                               //!< list of alf aps for the picture
  int                         alfChromaApsId;                                         //!< chroma alf aps ID
  bool                        ccalfEnabled[MAX_NUM_COMP];
  int                         ccalfCbApsId;
  int                         ccalfCrApsId;
  bool                        deblockingFilterOverride;                               //!< deblocking filter override controls enabled
  bool                        deblockingFilterDisable;                                //!< deblocking filter disabled flag
  int                         deblockingFilterBetaOffsetDiv2[MAX_NUM_COMP];           //!< beta offset for deblocking filter
  int                         deblockingFilterTcOffsetDiv2[MAX_NUM_COMP];              //!< tc offset for deblocking filter
  bool                        lmcsEnabled;                                            //!< lmcs enabled flag
  int                         lmcsApsId;                                              //!< lmcs APS ID
  APS*                        lmcsAps;                                                //!< lmcs APS
  bool                        lmcsChromaResidualScale;                                //!< lmcs chroma residual scale flag
  bool                        explicitScalingListEnabled;                             //!< quantization scaling lists present
  int                         scalingListApsId;                                       //!< quantization scaling list APS ID
  APS*                        scalingListAps;                                         //!< quantization scaling list APS
  unsigned                    minQTSize[3];                                           //!< minimum quad-tree size  0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned                    maxMTTDepth[3];                                         //!< maximum MTT depth
  unsigned                    maxBTSize[3];                                           //!< maximum BT size
  unsigned                    maxTTSize[3];                                           //!< maximum TT size

  WPScalingParam              weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMP];   // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  int                         numL0Weights;                                           //!< number of weights for L0 list
  int                         numL1Weights;                                           //!< number of weights for L1 list

  PicHeader() { initPicHeader(); }
  void getWpScaling(RefPicList e, int iRefIdx, WPScalingParam *&wp) const;

  void copyPicInfo( const PicHeader* other, bool cpyAll);

  void initPicHeader()
  {
    pic                                           = nullptr;
    nonRefPic                                     = false;
    gdrPic                                        = 0;
    noOutputOfPriorPics                           = 0;
    recoveryPocCnt                                = -1;
    noOutputBeforeRecovery                        = false;
    handleCraAsCvsStart                           = false;
    handleGdrAsCvsStart                           = false;
    spsId                                         = -1;
    ppsId                                         = -1;
    pocMsbPresent                                 = 0;
    pocMsbVal                                     = 0;
    virtualBoundariesEnabled                      = 0;
    virtualBoundariesPresent                      = 0;
    numVerVirtualBoundaries                       = 0;
    numHorVirtualBoundaries                       = 0;
    picOutputFlag                                 = true;
    splitConsOverride                             = false;
    cuQpDeltaSubdivIntra                          = 0;
    cuQpDeltaSubdivInter                          = 0;
    cuChromaQpOffsetSubdivIntra                   = 0;
    cuChromaQpOffsetSubdivInter                   = 0;
    enableTMVP                                    = true;
    picColFromL0                                  = true;
    colRefIdx                                     = 0;
    mvdL1Zero                                     = 0;
    maxNumAffineMergeCand                         = AFFINE_MRG_MAX_NUM_CANDS;
    disFracMMVD                                   = 0;
    disBdofFlag                                   = 0;
    disDmvrFlag                                   = 0;
    disProfFlag                                   = 0;
    jointCbCrSign                                 = 0;
    qpDelta                                       = 0;
    numAlfAps                                     = 0;
    alfChromaApsId                                = 0;
    ccalfCbApsId                                  = 0;
    ccalfCrApsId                                  = 0;
    deblockingFilterOverride                      = 0;
    deblockingFilterDisable                       = 0;
    deblockingFilterBetaOffsetDiv2[COMP_Y]        = 0;
    deblockingFilterTcOffsetDiv2[COMP_Y]          = 0;
    deblockingFilterBetaOffsetDiv2[COMP_Cb]       = 0;
    deblockingFilterTcOffsetDiv2[COMP_Cb]         = 0;
    deblockingFilterBetaOffsetDiv2[COMP_Cr]       = 0;
    deblockingFilterTcOffsetDiv2[COMP_Cr]         = 0;
    lmcsEnabled                                   = 0;
    lmcsApsId                                     = 0;
    lmcsAps                                       = nullptr;
    lmcsChromaResidualScale                       = 0;
    explicitScalingListEnabled                    = 0;
    scalingListApsId                              = -1;
    scalingListAps                                = nullptr;
    numL0Weights                                  = 0;
    numL1Weights                                  = 0;

    for( int i = 0; i < 2; i++)
    {
      pRPL[i] = nullptr;
      rplIdx[i]  = 0;

      localRPL[i].numberOfActivePictures = (0);
      localRPL[i].numberOfShorttermPictures = (0);
      localRPL[i].numberOfLongtermPictures = (0);
      localRPL[i].ltrpInSliceHeader = false;
    }

    alfApsId.resize(0);

    memset(virtualBoundariesPosX, 0, sizeof(virtualBoundariesPosX));
    memset(virtualBoundariesPosY, 0, sizeof(virtualBoundariesPosY));
    memset(saoEnabled,            0, sizeof(saoEnabled));
    memset(alfEnabled,            0, sizeof(alfEnabled));
    memset(ccalfEnabled,          0, sizeof(ccalfEnabled));

    memset(minQTSize,             0, sizeof(minQTSize));
    memset(maxMTTDepth,           0, sizeof(maxMTTDepth));
    memset(maxBTSize,             0, sizeof(maxBTSize));
    memset(maxTTSize,             0, sizeof(maxTTSize));
  }

};

/// slice header class
class Slice
{

  public:
  //  Bitstream writing
  bool                        saoEnabled[MAX_NUM_CH];
  int                         ppsId;               ///< picture parameter set ID
  int                         poc;
  int                         lastIDR;
  int                         prevGDRInSameLayerPOC;  //< the previous GDR in the same layer
  int                         associatedIRAP;
  vvencNalUnitType            associatedIRAPType;
  bool                        enableDRAPSEI;
  bool                        useLTforDRAP;
  bool                        isDRAP;
  int                         latestDRAPPOC;

  const ReferencePictureList* rpl[NUM_REF_PIC_LIST_01];                 //< pointer to RPL for L0, either in the SPS or the local RPS in the same slice header
  ReferencePictureList        rplLocal[NUM_REF_PIC_LIST_01];            //< RPL for L0 when present in slice header
  int                         rplIdx[NUM_REF_PIC_LIST_01];              //< index of used RPL in the SPS or -1 for local RPL in the slice header
  int                         colourPlaneId;                         //!< 4:4:4 colour plane ID
  bool                        pictureHeaderInSliceHeader;
  uint32_t                    nuhLayerId;
  vvencNalUnitType            nalUnitType;         ///< Nal unit type for the slice
  vvencSliceType              sliceType;
  int                         sliceQp;
  bool                        chromaQpAdjEnabled;
  bool                        lmcsEnabled;
  bool                        explicitScalingListUsed;
  bool                        deblockingFilterDisable;
  bool                        deblockingFilterOverrideFlag;      //< offsets for deblocking filter inherit from PPS
  int                         deblockingFilterBetaOffsetDiv2[MAX_NUM_COMP];    //< beta offset for deblocking filter
  int                         deblockingFilterTcOffsetDiv2[MAX_NUM_COMP];      //< tc offset for deblocking filter
  bool                        depQuantEnabled;               //!< dependent quantization enabled flag
  bool                        signDataHidingEnabled;         //!< sign data hiding enabled flag
  bool                        tsResidualCodingDisabled;
  int                         list1IdxToList0Idx[MAX_NUM_REF];
  int                         numRefIdx   [NUM_REF_PIC_LIST_01];    //  for multiple reference of current slice
  bool                        pendingRasInit;
  bool                        checkLDC;
  bool                        biDirPred;
  bool                        lmChromaCheckDisable;
  int                         symRefIdx[2];
  
  //  Data
  int                         sliceChromaQpDelta[MAX_NUM_COMP+1];
  Picture*                    refPicList [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  int                         refPOCList  [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  bool                        isUsedAsLongTerm[NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  int                         depth;


  // access channel
  const VPS*                  vps;
  const DCI*                  dci;
  const SPS*                  sps;
  const PPS*                  pps;
  Picture*                    pic;
  PicHeader*                  picHeader;    //!< pointer to picture header structure
  bool                        colFromL0Flag;  // collocated picture from List0 flag
  uint32_t                    colRefIdx;
  double                      lambdas[MAX_NUM_COMP];
  uint32_t                    TLayer;
  bool                        TLayerSwitchingFlag;
  SliceMap                    sliceMap;
  uint32_t                    independentSliceIdx;
  WPScalingParam              weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMP];
  WPACDCParam                 weightACDCParam[MAX_NUM_COMP];
  ClpRngs                     clpRngs;
  std::vector<uint32_t>       substreamSizes;
  bool                        cabacInitFlag;
  uint32_t                    sliceSubPicId;
  vvencSliceType              encCABACTableIdx;           // Used to transmit table selection across slices.
  APS*                        alfAps[ALF_CTB_MAX_NUM_APS];
  bool                        tileGroupAlfEnabled[MAX_NUM_COMP];
  int                         tileGroupNumAps;
  std::vector<int>            tileGroupLumaApsId;
  int                         tileGroupChromaApsId;
  bool                        tileGroupCcAlfCbEnabled;
  bool                        tileGroupCcAlfCrEnabled;
  int                         tileGroupCcAlfCbApsId;
  int                         tileGroupCcAlfCrApsId;

  bool                        disableSATDForRd;
  bool                        isLossless;
  CcAlfFilterParam            ccAlfFilterParam;
  uint8_t*                    ccAlfFilterControl[2];

public:
                              Slice();
  virtual                     ~Slice();
  void                        resetSlicePart();
  void                        constructRefPicList(PicList& rcListPic, bool extBorder);
  void                        updateRefPicCounter( int step );
  bool                        checkRefPicsReconstructed() const;
  void                        setRefPOCList();
  void                        setSMVDParam();
  void                        checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic) const;
  void                        setAlfAPSs(APS** apss)                                 { memcpy(alfAps, apss, sizeof(alfAps)); }

  const Picture*              getRefPic( RefPicList e, int iRefIdx) const            { return refPicList[e][iRefIdx]; }
  int                         getRefPOC( RefPicList e, int iRefIdx) const            { return refPOCList[e][iRefIdx]; }
  int                         getNumEntryPoints(const SPS& sps, const PPS& pps) const;

  bool                        getRapPicFlag() const;
  bool                        getIdrPicFlag() const                                  { return nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL || nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  bool                        isIRAP() const { return (nalUnitType >= VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL) && (nalUnitType <= VVENC_NAL_UNIT_CODED_SLICE_CRA); }
  bool                        isIDRorBLA() const { return (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP); }
  void                        checkCRA(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, int& pocCRA, vvencNalUnitType& associatedIRAPType, PicList& rcListPic);
  void                        setDecodingRefreshMarking(int& pocCRA, bool& bRefreshPending, PicList& rcListPic);

  bool                        isIntra() const                                        { return sliceType == VVENC_I_SLICE; }
  bool                        isInterB() const                                       { return sliceType == VVENC_B_SLICE; }
  bool                        isInterP() const                                       { return sliceType == VVENC_P_SLICE; }

  void                        setLambdas( const double lambdas_[MAX_NUM_COMP] )  { for (int component = 0; component < MAX_NUM_COMP; component++) lambdas[component] = lambdas_[component]; }
  const double*               getLambdas() const                                     { return lambdas;                                             }

  static void                 sortPicList( PicList& rcListPic );
  void                        setList1IdxToList0Idx();

  void                        copySliceInfo( const Slice* slice, bool cpyAlmostAll = true);

  void                        checkLeadingPictureRestrictions( PicList& rcListPic )                                         const;
  void                        applyReferencePictureListBasedMarking( PicList& rcListPic, const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int layerId, const PPS& pps )  const;
  bool                        isTemporalLayerSwitchingPoint( PicList& rcListPic )                                           const;
  bool                        isStepwiseTemporalLayerSwitchingPointCandidate( PicList& rcListPic )                          const;
  int                         checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList* pRPL, int rplIdx, bool printErrors)                const;
  void                        createExplicitReferencePictureSetFromReference(PicList& rcListPic, const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1);
  void                        getWpScaling( RefPicList e, int iRefIdx, WPScalingParam *&wp) const;


  void                        clearSubstreamSizes( )                                 { return substreamSizes.clear();                              }
  uint32_t                    getNumberOfSubstreamSizes( )                           { return (uint32_t) substreamSizes.size();                    }
  void                        addSubstreamSize( uint32_t size )                      { substreamSizes.push_back(size);                             }
  uint32_t                    getSubstreamSize( uint32_t idx )                       { CHECK(idx>=getNumberOfSubstreamSizes(),"Invalid index"); return substreamSizes[idx]; }

  void                        setDefaultClpRng( const SPS& sps );
  unsigned                    getMinPictureDistance()                           const ;

  bool                        isPocRestrictedByDRAP( int poc, bool precedingDRAPInDecodingOrder ) const;
  void                        setAlfApsIds( const std::vector<int>& ApsIDs);
private:
  Picture*                    xGetLongTermRefPic(PicList& rcListPic, int poc, bool pocHasMsb);
};// END CLASS DEFINITION Slice

void calculateParameterSetChangedFlag(bool& bChanged, const std::vector<uint8_t>* pOldData, const std::vector<uint8_t>* pNewData);

template <class T> class ParameterSetMap
{
public:
  template <class Tm>
  struct MapData
  {
    bool                  bChanged;
    std::vector<uint8_t>* pNaluData;
    Tm*                   parameterSet;
  };


private:
  std::map<int,MapData<T> > m_paramsetMap;
  int                       m_maxId;
  std::vector<int>          m_activePsId;
  T*                        m_lastActiveParameterSet;
  int                       m_apsIdStart;

public:
  ParameterSetMap()
  : m_maxId                 ( MAX_NUM_APS * MAX_NUM_APS_TYPE )
  , m_activePsId            ()
  , m_lastActiveParameterSet( nullptr )
  , m_apsIdStart            ( ALF_CTB_MAX_NUM_APS )
  {
  }

  ParameterSetMap( int maxId )
  : m_maxId                 ( maxId )
  , m_activePsId            ()
  , m_lastActiveParameterSet( nullptr )
  , m_apsIdStart            ( ALF_CTB_MAX_NUM_APS )
  {
  }

  ~ParameterSetMap()
  {
    clearMap();
  }

  T *allocatePS( const int psId )
  {
    CHECK( psId >= m_maxId, "Invalid PS id" );
    if ( m_paramsetMap.find(psId) == m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged     = true;
      m_paramsetMap[psId].pNaluData    = nullptr;
      m_paramsetMap[psId].parameterSet = new T;
      setID( m_paramsetMap[ psId ].parameterSet, psId );
    }
    return m_paramsetMap[ psId ].parameterSet;
  }

  void clearMap()
  {
    for( typename std::map<int,MapData<T> >::iterator i = m_paramsetMap.begin(); i != m_paramsetMap.end(); i++ )
    {
      delete (*i).second.pNaluData;
      delete (*i).second.parameterSet;
    }
    delete m_lastActiveParameterSet; m_lastActiveParameterSet = nullptr;
    m_paramsetMap.clear();
    m_activePsId.clear();
  }

  void storePS( int psId, T *ps )
  {
    CHECK( psId >= m_maxId, "Invalid PS id" );
    if( m_paramsetMap.find( psId ) != m_paramsetMap.end() )
    {
      delete m_paramsetMap[psId].parameterSet;
    }
    m_paramsetMap[psId].parameterSet = ps;
    m_paramsetMap[psId].bChanged = true;
  }

  void storePS(int psId, T *ps, const std::vector<uint8_t> *pNaluData)
  {
    CHECK( psId >= m_maxId, "Invalid PS id" );
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      MapData<T> &mapData=m_paramsetMap[psId];

      // work out changed flag
      calculateParameterSetChangedFlag(mapData.bChanged, mapData.pNaluData, pNaluData);

      if( ! mapData.bChanged )
      {
        // just keep the old one
        delete ps;
        return;
      }

      if (find(m_activePsId.begin(), m_activePsId.end(), psId) != m_activePsId.end())
      {
        std::swap( m_paramsetMap[psId].parameterSet, m_lastActiveParameterSet );
      }
      delete m_paramsetMap[psId].pNaluData;
      delete m_paramsetMap[psId].parameterSet;

      m_paramsetMap[psId].parameterSet = ps;
    }
    else
    {
      m_paramsetMap[psId].parameterSet = ps;
      m_paramsetMap[psId].bChanged = false;
    }
    if (pNaluData != 0)
    {
      m_paramsetMap[psId].pNaluData=new std::vector<uint8_t>;
      *(m_paramsetMap[psId].pNaluData) = *pNaluData;
    }
    else
    {
      m_paramsetMap[psId].pNaluData=0;
    }
  }

  void setChangedFlag(int psId, bool bChanged=true)
  {
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged=bChanged;
    }
  }

  void clearChangedFlag(int psId)
  {
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged=false;
    }
  }

  bool getChangedFlag(int psId) const
  {
    const typename std::map<int,MapData<T> >::const_iterator constit=m_paramsetMap.find(psId);
    if ( constit != m_paramsetMap.end() )
    {
      return constit->second.bChanged;
    }
    return false;
  }

  T* getPS(int psId)
  {
    typename std::map<int,MapData<T> >::iterator it=m_paramsetMap.find(psId);
    return ( it == m_paramsetMap.end() ) ? NULL : (it)->second.parameterSet;
  }

  const T* getPS(int psId) const
  {
    typename std::map<int,MapData<T> >::const_iterator it=m_paramsetMap.find(psId);
    return ( it == m_paramsetMap.end() ) ? NULL : (it)->second.parameterSet;
  }

  T* getFirstPS()
  {
    return (m_paramsetMap.begin() == m_paramsetMap.end() ) ? NULL : m_paramsetMap.begin()->second.parameterSet;
  }

  void      setActive(int psId)           { m_activePsId.push_back(psId); }
  void      clearActive()                 { m_activePsId.clear(); m_apsIdStart = ALF_CTB_MAX_NUM_APS; }
  int       getApsIdStart()               { return m_apsIdStart; }
  const int getApsIdStart() const         { return m_apsIdStart; }
  void      setApsIdStart( const int id ) { m_apsIdStart = id; }
//   void      reset()                       
//   { 
//     m_apsIdStart = m_apsIdStart = ALF_CTB_MAX_NUM_APS; 
//     for( typename std::map<int,MapData<T> >::iterator i = m_paramsetMap.begin(); i != m_paramsetMap.end(); i++ )
//     {
//       (*i).second.bChanged = false;
//     }
//   }

private:
  static void setID(T* parameterSet, const int psId);
};

class ParameterSetManager
{
public:
                 ParameterSetManager();
  virtual        ~ParameterSetManager();
  void           storeVPS(VPS *vps, const std::vector<uint8_t> &naluData)    { m_vpsMap.storePS(vps->vpsId, vps, &naluData); }
  VPS*           getVPS( int vpsId )                                         { return m_vpsMap.getPS( vpsId ); };

  void           storeDCI(DCI *dci, const std::vector<uint8_t> &naluData)    { m_dciMap.storePS( dci->dciId, dci, &naluData); };
  //DCI*           getDCI(int dciId)                                           { return m_dciMap.getPS(dciId); };

  void           storeSPS(SPS *sps, const std::vector<uint8_t> &naluData) { m_spsMap.storePS( sps->spsId, sps, &naluData); };
  SPS*           getSPS(int spsId)                                           { return m_spsMap.getPS(spsId); };
  bool           getSPSChangedFlag(int spsId) const                          { return m_spsMap.getChangedFlag(spsId); }
  void           clearSPSChangedFlag(int spsId)                              { m_spsMap.clearChangedFlag(spsId); }
  SPS*           getFirstSPS()                                               { return m_spsMap.getFirstPS(); };

  void           storePPS(PPS *pps, const std::vector<uint8_t> &naluData) { m_ppsMap.storePS( pps->ppsId, pps, &naluData); };
  PPS*           getPPS(int ppsId)                                           { return m_ppsMap.getPS(ppsId); };
  bool           getPPSChangedFlag(int ppsId) const                          { return m_ppsMap.getChangedFlag(ppsId); }
  void           clearPPSChangedFlag(int ppsId)                              { m_ppsMap.clearChangedFlag(ppsId); }
  PPS*           getFirstPPS()                                               { return m_ppsMap.getFirstPS(); };

  bool           activatePPS(int ppsId, bool isIRAP);
  APS**          getAPSs() { return &m_apss[0]; }
  ParameterSetMap<APS>* getApsMap() { return &m_apsMap; }
  void           storeAPS(APS *aps, const std::vector<uint8_t> &naluData)    { m_apsMap.storePS((aps->apsId << NUM_APS_TYPE_LEN) + aps->apsType, aps, &naluData); };
  APS*           getAPS(int apsId, int apsType)                              { return m_apsMap.getPS((apsId << NUM_APS_TYPE_LEN) + apsType); };
  bool           getAPSChangedFlag(int apsId, int apsType) const             { return m_apsMap.getChangedFlag((apsId << NUM_APS_TYPE_LEN) + apsType); }
  void           clearAPSChangedFlag(int apsId, int apsType)                 { m_apsMap.clearChangedFlag((apsId << NUM_APS_TYPE_LEN) + apsType); }
  bool           activateAPS(int apsId, int apsType);
  const VPS*     getActiveVPS()const                                         { return m_vpsMap.getPS(m_activeVPSId); };
  const SPS*     getActiveSPS()const                                         { return m_spsMap.getPS(m_activeSPSId); };
  const DCI*     getActiveDCI()const                                         { return m_dciMap.getPS(m_activeDCIId); };

protected:
  ParameterSetMap<SPS> m_spsMap;
  ParameterSetMap<PPS> m_ppsMap;
  ParameterSetMap<APS> m_apsMap;
  ParameterSetMap<DCI> m_dciMap;
  ParameterSetMap<VPS> m_vpsMap;

  APS* m_apss[ALF_CTB_MAX_NUM_APS];

  int m_activeDCIId; // -1 for nothing active
  int m_activeSPSId; // -1 for nothing active
  int m_activeVPSId; // -1 for nothing active
};

class PreCalcValues
{
public:
  PreCalcValues( const SPS& sps, const PPS& pps, bool _isEncoder )
    : chrFormat           ( sps.chromaFormatIdc )
    , maxCUSize           ( sps.CTUSize )
    , maxCUSizeMask       ( maxCUSize  - 1 )
    , maxCUSizeLog2       ( Log2( maxCUSize  ) )
    , minCUSize           ( 1 << MIN_CU_LOG2 )
    , minCUSizeLog2       ( MIN_CU_LOG2 )
    , partsInCtuWidth     ( maxCUSize >> MIN_CU_LOG2 )
    , partsInCtu          ( partsInCtuWidth*partsInCtuWidth )
    , widthInCtus         ( (pps.picWidthInLumaSamples  + sps.CTUSize - 1) / sps.CTUSize )
    , heightInCtus        ( (pps.picHeightInLumaSamples + sps.CTUSize - 1) / sps.CTUSize )
    , sizeInCtus          ( widthInCtus * heightInCtus )
    , lumaWidth           ( pps.picWidthInLumaSamples )
    , lumaHeight          ( pps.picHeightInLumaSamples )
    , fastDeltaQPCuMaxSize( Clip3<unsigned>( (1 << sps.log2MinCodingBlockSize), sps.CTUSize, 32u) )
    , multiBlock422       ( false )
    , noChroma2x2         ( false )
    , isEncoder           ( _isEncoder )
    , ISingleTree         ( !sps.dualITree )
    , wrapArround         ( sps.wrapAroundEnabled )
    , maxMTTDepth         { sps.maxMTTDepth[0], sps.maxMTTDepth[1], sps.maxMTTDepth[2] }
    , minTSize            { 1u << sps.log2MinCodingBlockSize, 1u << sps.log2MinCodingBlockSize, 1u << sps.log2MinCodingBlockSize }
    , maxBtSize           { sps.maxBTSize[0], sps.maxBTSize[1], sps.maxBTSize[2] }
    , maxTtSize           { sps.maxTTSize[0], sps.maxTTSize[1], sps.maxTTSize[2] }
    , minQtSize           { sps.minQTSize[0], sps.minQTSize[1], sps.minQTSize[2] }
  {}

  const ChromaFormat chrFormat;
  const unsigned     maxCUSize;
  // to get CTU position, use (x & maxCUWidthMask) rather than (x % maxCUWidth)
  const unsigned     maxCUSizeMask;
  const unsigned     maxCUSizeLog2;
  const unsigned     minCUSize;
  const unsigned     minCUSizeLog2;
  const unsigned     partsInCtuWidth;
  const unsigned     partsInCtu;
  const unsigned     widthInCtus;
  const unsigned     heightInCtus;
  const unsigned     sizeInCtus;
  const unsigned     lumaWidth;
  const unsigned     lumaHeight;
  const unsigned     fastDeltaQPCuMaxSize;
  const bool         multiBlock422;
  const bool         noChroma2x2;
  const bool         isEncoder;
  const bool         ISingleTree;
  const bool         wrapArround;

private:
  const unsigned     maxMTTDepth[3];
  const unsigned     minTSize   [3];
  const unsigned     maxBtSize  [3];
  const unsigned     maxTtSize  [3];
  const unsigned     minQtSize  [3];

  unsigned getValIdx      ( const Slice &slice, const ChannelType chType ) const;

public:
  unsigned getMaxMTTDepth ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinTSize    ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxBtSize   ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxTtSize   ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinQtSize   ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxDepth    ( const vvencSliceType slicetype, const ChannelType chType )   const { return maxCUSizeLog2 - Log2(slicetype == VVENC_I_SLICE ? (chType == CH_L ? minQtSize[0] : minQtSize[2]) : minQtSize[1]); }
  Area     getCtuArea     ( const int ctuPosX, const int ctuPosY ) const;
};

} // namespace vvenc

//! \}


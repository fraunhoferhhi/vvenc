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
/** \file     UnitTool.h
 *  \brief    defines operations for basic units
 */

#pragma once

#include "Unit.h"
#include "UnitPartitioner.h"
#include "ContextModelling.h"
#include "InterPrediction.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

// CS tools
namespace CS
{
  UnitArea  getArea                    (const CodingStructure &cs, const UnitArea& area, const ChannelType chType, const TreeType treeType);
  bool      isDualITree                (const CodingStructure &cs);
  void      setRefinedMotionField      (      CodingStructure &cs);
}


// CU tools
namespace CU
{
  inline bool isSepTree                 (const CodingUnit &cu)                          { return cu.treeType != TREE_D || CS::isDualITree( *cu.cs ); }
  inline bool isLocalSepTree            (const CodingUnit &cu)                          { return cu.treeType != TREE_D && !CS::isDualITree(*cu.cs); }
  inline bool isConsInter               (const CodingUnit &cu)                          { return cu.modeType == MODE_TYPE_INTER; }
  inline bool isConsIntra               (const CodingUnit &cu)                          { return cu.modeType == MODE_TYPE_INTRA; }

  inline bool isIntra                   (const CodingUnit &cu)                          { return cu.predMode == MODE_INTRA; }
  inline bool isInter                   (const CodingUnit &cu)                          { return cu.predMode == MODE_INTER; }
  inline bool isIBC                     (const CodingUnit &cu)                          { return cu.predMode == MODE_IBC; }
  inline bool isPLT                     (const CodingUnit &cu)                          { return cu.predMode == MODE_PLT; }
  inline bool isRDPCMEnabled            (const CodingUnit& cu)                          { return cu.predMode == MODE_INTRA ? cu.cs->sps->spsRExt.implicitRdpcmEnabled : cu.cs->sps->spsRExt.explicitRdpcmEnabled;}
  inline bool isSameSlice               (const CodingUnit& cu, const CodingUnit& cu2)   { return cu.slice->independentSliceIdx == cu2.slice->independentSliceIdx; }
  inline bool isSameTile                (const CodingUnit& cu, const CodingUnit& cu2)   { return cu.tileIdx == cu2.tileIdx; }
  inline bool isSameSliceAndTile        (const CodingUnit& cu, const CodingUnit& cu2)   { return ( cu.slice->independentSliceIdx == cu2.slice->independentSliceIdx ) && ( cu.tileIdx == cu2.tileIdx ); }

  uint8_t   checkAllowedSbt             (const CodingUnit &cu);
  bool      checkCCLMAllowed            (const CodingUnit &cu);
  bool      isSameCtu                   (const CodingUnit &cu, const CodingUnit &cu2);
  bool      isSameSubPic                (const CodingUnit &cu, const CodingUnit &cu2);
  bool      isLastSubCUOfCtu            (const CodingUnit &cu);
  uint32_t  getCtuAddr                  (const CodingUnit &cu);
  int       predictQP                   (const CodingUnit& cu, const int prevQP);

  void saveMotionInHMVP                 (const CodingUnit& cu, const bool isToBeDone );

  PartSplit getSplitAtDepth             (const CodingUnit& cu, const unsigned depth);
  ModeType  getModeTypeAtDepth          (const CodingUnit& cu, const unsigned depth);

  bool      isPredRegDiffFromTB         (const CodingUnit& cu);
  bool      isFirstTBInPredReg          (const CodingUnit& cu, const CompArea& area);
  void      adjustPredArea              (CompArea& area);
  bool      isBcwIdxCoded               (const CodingUnit& cu);
  uint8_t   getValidBcwIdx              (const CodingUnit& cu);
  void      setBcwIdx                   (      CodingUnit& cu, uint8_t uh);
  bool      bdpcmAllowed                (const CodingUnit& cu, const ComponentID compID);
  bool      isMTSAllowed                (const CodingUnit& cu, const ComponentID compID);


  bool      divideTuInRows              (const CodingUnit &cu);
  PartSplit getISPType                  (const CodingUnit &cu,                         const ComponentID compID);
  bool      isISPLast                   (const CodingUnit &cu, const CompArea& tuArea, const ComponentID compID);
  bool      isISPFirst                  (const CodingUnit &cu, const CompArea& tuArea, const ComponentID compID);
  bool      canUseISP                   (const CodingUnit &cu,                         const ComponentID compID);
  bool      canUseISP                   (const int width, const int height, const int maxTrSize = MAX_TB_SIZEY );
  bool      canUseLfnstWithISP          (const CompArea& cuArea, const ISPType ispSplitType );
  bool      canUseLfnstWithISP          (const CodingUnit& cu, const ChannelType chType );
  uint32_t  getISPSplitDim              (const int width, const int height, const PartSplit ispType);
  bool      allLumaCBFsAreZero          (const CodingUnit& cu);

  TUTraverser  traverseTUs              (      CodingUnit& cu);
  cTUTraverser traverseTUs              (const CodingUnit& cu);

  bool      hasSubCUNonZeroMVd          (const CodingUnit& cu);
  bool      hasSubCUNonZeroAffineMVd    (const CodingUnit& cu );
  void      resetMVDandMV2Int           (      CodingUnit& cu );

  inline uint8_t   getSbtIdx            ( const uint8_t sbtInfo )                     { return ( sbtInfo >> 0 ) & 0xf; }
  inline uint8_t   getSbtPos            ( const uint8_t sbtInfo )                     { return ( sbtInfo >> 4 ) & 0x3; }

  inline uint8_t   getSbtMode           (const uint8_t sbtIdx, const uint8_t sbtPos)  { return (sbtIdx<<1) + sbtPos - 2; }
  inline uint8_t   getSbtIdxFromSbtMode (const uint8_t sbtMode)                       { return (sbtMode>>1)+1; }
  inline uint8_t   getSbtPosFromSbtMode (const uint8_t sbtMode)                       { return sbtMode&1;}
  inline uint8_t   targetSbtAllowed     (uint8_t sbtIdx, uint8_t sbtAllowed)          { return ( sbtAllowed >> sbtIdx ) & 0x1; }

  uint8_t   numSbtModeRdo               (uint8_t sbtAllowed);
  PartSplit getSbtTuSplit               ( const uint8_t sbtInfo );
  bool      isSbtMode                   (const uint8_t sbtInfo);
  bool      isSameSbtSize               (const uint8_t sbtInfo1, const uint8_t sbtInfo2);
  bool      getRprScaling               ( const SPS* sps, const PPS* curPPS, Picture* refPic, int& xScale, int& yScale );

  const CodingUnit* getLeft             (const CodingUnit& curr);
  const CodingUnit* getAbove            (const CodingUnit& curr);


  int      getLMSymbolList              (const CodingUnit& cu, int *modeList);
  bool     isMIP                        (const CodingUnit& cu, const ChannelType channelType = CH_L);
  int      getIntraMPMs                 (const CodingUnit& cu, unsigned *mpm );
  bool     isDMChromaMIP                (const CodingUnit& cu);
  uint32_t getIntraDirLuma              (const CodingUnit& cu);
  void     getIntraChromaCandModes      (const CodingUnit& cu, unsigned modeList[NUM_CHROMA_MODE]);
  const CodingUnit &getCoLocatedLumaPU  (const CodingUnit& cu);
  uint32_t getFinalIntraMode            (const CodingUnit& cu, const ChannelType chType);
  uint32_t getCoLocatedIntraLumaMode    (const CodingUnit& cu);
  void     getInterMergeCandidates      (const CodingUnit& cu, MergeCtx& mrgCtx, int mmvdList, const int mrgCandIdx = -1 );
  void     getInterMMVDMergeCandidates  (const CodingUnit& cu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
  int      getDistScaleFactor           (const int currPOC, const int currRefPOC, const int colPOC, const int colRefPOC);
  bool     isDiffMER                    (const Position &pos1, const Position &pos2, const unsigned plevel);
  bool     getColocatedMVP              (const CodingUnit& cu, const RefPicList refPicList, const Position& pos, Mv& rcMv, const int refIdx, bool sbFlag = false);
  void     fillMvpCand                  (      CodingUnit& cu, const RefPicList refPicList, const int refIdx, AMVPInfo &amvpInfo );
  bool     addMVPCandUnscaled           (const CodingUnit& cu, const RefPicList refPicList, const int iRefIdx, const Position& pos, const MvpDir dir, AMVPInfo &amvpInfo);
  bool     addMergeHMVPCand             (const CodingStructure &cs, MergeCtx& mrgCtx, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1, const MotionInfo &miLeft, const bool isAvailableB1, const MotionInfo &miAbove, const bool ibcFlag, const bool isGt4x4);
  void     addAMVPHMVPCand              (const CodingUnit& cu, const RefPicList refPicList, const int currRefPOC, AMVPInfo &info);
  bool     isBipredRestriction          (const CodingUnit& cu);
  void     spanMotionInfo               (      CodingUnit& cu, const MergeCtx &mrgCtx = MergeCtx() );
  void     restrictBiPredMergeCandsOne  (      CodingUnit& cu);

  bool     isLMCMode                    (                          unsigned mode);
  bool     isLMCModeEnabled             (const CodingUnit& cu, unsigned mode);
  void     getGeoMergeCandidates        (const CodingUnit& cu, MergeCtx &GeoMrgCtx);
  void     spanGeoMotionInfo            (CodingUnit& cu, MergeCtx &GeoMrgCtx, const uint8_t splitDir, const uint8_t candIdx0, const uint8_t candIdx1);
  bool     isBiPredFromDifferentDirEqDistPoc(const CodingUnit& cu);
  bool     checkDMVRCondition           (const CodingUnit& cu);
  void     getAffineControlPointCand    (const CodingUnit& cu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t BcwIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgCtx);
  void     getAffineMergeCand           (      CodingUnit& cu, AffineMergeCtx& affMrgCtx, const int mrgCandIdx = -1);
  void     setAllAffineMvField          (      CodingUnit& cu, const MvField *mvField, RefPicList eRefList);
  void     setAllAffineMv               (      CodingUnit& cu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs = false);
  void     xInheritedAffineMv           (const CodingUnit& cu, const CodingUnit* cuNeighbour, RefPicList refPicList, Mv rcMv[3]);
  void     fillAffineMvpCand            (      CodingUnit& cu, const RefPicList refPicList, const int refIdx, AffineAMVPInfo &affiAMVPInfo);
  bool     addAffineMVPCandUnscaled     (const CodingUnit& cu, const RefPicList refPicList, const int refIdx, const Position& pos, const MvpDir dir, AffineAMVPInfo &affiAmvpInfo);
  bool     getInterMergeSbTMVPCand      (const CodingUnit& cu, MergeCtx &mrgCtx, bool& LICFlag, const int count, int mmvdList);

  void     getIBCMergeCandidates        (const CodingUnit& cu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
  void     fillIBCMvpCand               (CodingUnit& cu, AMVPInfo& amvpInfo);
  void     getIbcMVPsEncOnly            (CodingUnit& cu, Mv* mvPred, int& nbPred);
  bool     isAddNeighborMvIBC           (const Mv& currMv, Mv* neighborMvs, int numNeighborMv);
  bool     getDerivedBVIBC              (CodingUnit& cu, const Mv& currentMv, Mv& derivedMv);
}

// TU tools
namespace TU
{
  bool isNonTransformedResidualRotated  (const TransformUnit& tu, const ComponentID compID);
  bool getCbf                           (const TransformUnit& tu, const ComponentID compID);
  bool getCbfAtDepth                    (const TransformUnit& tu, const ComponentID compID, const unsigned depth);
  void setCbfAtDepth                    (      TransformUnit& tu, const ComponentID compID, const unsigned depth, const bool cbf);
  bool isTSAllowed                      (const TransformUnit& tu, const ComponentID compID);

  bool needsSqrt2Scale                  (const TransformUnit& tu, const ComponentID compID);
  TransformUnit* getPrevTU              (const TransformUnit& tu, const ComponentID compID);
  bool           getPrevTuCbfAtDepth    (const TransformUnit& tu, const ComponentID compID, const int trDepth );
  int            getICTMode             (const TransformUnit& tu, int jointCbCr = -1);
}

uint32_t  getCtuAddr                    (const Position& pos, const PreCalcValues &pcv);
int       getNumModesMip                (const Size& block);
int       getMipSizeId                  (const Size& block);
bool      allowLfnstWithMip             (const Size& block);

template<typename T, size_t N>
uint32_t updateCandList( T uiMode, double uiCost, static_vector<T, N> &candModeList, static_vector<double, N> &candCostList, size_t uiFastCandNum = N, int *iserttPos = nullptr )
{
  CHECK( std::min( uiFastCandNum, candModeList.size() ) != std::min( uiFastCandNum, candCostList.size() ), "Sizes do not match!" );
  CHECK( uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!" );

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min( uiFastCandNum, candCostList.size() );

  while( shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift] )
  {
    shift++;
  }

  if( candModeList.size() >= uiFastCandNum && shift != 0 )
  {
    for( i = 1; i < shift; i++ )
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = uiMode;
    candCostList[currSize - shift] = uiCost;
    if (iserttPos != nullptr)
    {
      *iserttPos = int(currSize - shift);
    }
    return 1;
  }
  else if( currSize < uiFastCandNum )
  {
    candModeList.insert( candModeList.end() - shift, uiMode );
    candCostList.insert( candCostList.end() - shift, uiCost );
    if (iserttPos != nullptr)
    {
      *iserttPos = int(candModeList.size() - shift - 1);
    }
    return 1;
  }
  if (iserttPos != nullptr)
  {
    *iserttPos = -1;
  }
  return 0;
}

} // namespace vvenc

//! \}


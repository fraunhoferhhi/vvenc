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
/** \file     IntraSearch.h
    \brief    intra search class (header)
*/

#pragma once

#include "CABACWriter.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/CommonDef.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class EncPicture;
class EncCu;

/// encoder search class
class IntraSearch : public IntraPrediction
{
private:
  static const int maxCuDepth = (MAX_CU_SIZE_IDX - MIN_CU_LOG2) << 1;

  CodingStructure*    m_pTempCS;
  CodingStructure*    m_pBestCS;
  CodingStructure**   m_pSaveCS;
  bool                m_saveCuCostInSCIPU;
  uint8_t             m_numCuInSCIPU;
  Area                m_cuAreaInSCIPU[NUM_INTER_CU_INFO_SAVE];
  double              m_cuCostInSCIPU[NUM_INTER_CU_INFO_SAVE];

  struct ModeInfo
  {
    bool     mipFlg;    // CU::mipFlag
    bool     mipTrFlg;  // CU::mipTransposeFlag
    int8_t   mRefId;    // CU::multiRefIdx
    uint8_t  ispMod;    // CU::ispMode
    uint8_t  modeId;    // CU::intraDir[CH_L]

    ModeInfo() : mipFlg(false), mipTrFlg(false), mRefId(0), ispMod(NOT_INTRA_SUBPARTITIONS), modeId(0) {}
    ModeInfo(const bool mipf, const bool miptf, const int8_t mrid, const uint8_t ispm, const uint8_t mode) : mipFlg(mipf), mipTrFlg(miptf), mRefId(mrid), ispMod(ispm), modeId(mode) {}
    bool operator==(const ModeInfo cmp) const { return (0 == ::memcmp(this,&cmp,sizeof(ModeInfo))); }
  };
#if ENABLE_TIME_PROFILING_MT_MODE
  TProfiler*      m_timeProfiler = nullptr;
#endif
protected:
  // interface to option
  const VVEncCfg* m_pcEncCfg;

  // interface to classes
  TrQuant*        m_pcTrQuant;
  RdCost*         m_pcRdCost;
  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;

  SortedPelUnitBufs<SORTED_BUFS> *m_SortedPelUnitBufs;
public:
  IntraSearch();
  ~IntraSearch();

  struct ISPTestedModesInfo
  {
    int                                         numTotalParts[2];
    int                                         bestModeSoFar;
    ISPType                                     bestSplitSoFar;
    double                                      bestCost[2];
    bool                                        splitIsFinished[2];
    int                                         subTuCounter;
    PartSplit                                   IspType;
    bool                                        relatedCuIsValid;
    bool                                        intraWasTested;
    int                                         bestIntraMode;
    bool                                        isIntra;
    int                                         bestBefore[3];
    // set everything to default values
    void clear()
    {
      for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
      {
        numTotalParts[splitIdx] = 0;
        splitIsFinished[splitIdx] = false;
        bestCost[splitIdx] = MAX_DOUBLE;
      }
      bestModeSoFar = -1;
      bestSplitSoFar = NOT_INTRA_SUBPARTITIONS;
      subTuCounter = -1;
      IspType = TU_NO_ISP;
    }
    void init(const int numTotalPartsHor, const int numTotalPartsVer, bool n)
    {
      if (n)
      {
        intraWasTested = false;
        relatedCuIsValid = false;
        bestIntraMode = 0;
        isIntra   = false;
        std::memset(bestBefore,0, sizeof(bestBefore));
        clear();
      }
      else
      {
        const int horSplit = HOR_INTRA_SUBPARTITIONS - 1, verSplit = VER_INTRA_SUBPARTITIONS - 1;
        numTotalParts[horSplit] = numTotalPartsHor;
        numTotalParts[verSplit] = numTotalPartsVer;
        splitIsFinished[horSplit] = (numTotalParts[horSplit] == 0);
        splitIsFinished[verSplit] = (numTotalParts[verSplit] == 0);
        subTuCounter = -1;
        IspType = TU_NO_ISP;
      }
    }
  };

  ISPTestedModesInfo m_ispTestedModes[ NUM_LFNST_NUM_PER_SET ];
  void init                       ( const VVEncCfg &encCfg, TrQuant *pTrQuant, RdCost *pRdCost, SortedPelUnitBufs<SORTED_BUFS> *pSortedPelUnitBufs, XUCache &unitCache _TPROF_DECL );
  void setCtuEncRsrc              ( CABACWriter* cabacEstimator, CtxCache* ctxCache );
  void destroy                    ();

  CodingStructure  **getSaveCSBuf () { return m_pSaveCS; }

  bool getSaveCuCostInSCIPU       ()               { return m_saveCuCostInSCIPU; }
  void setSaveCuCostInSCIPU       ( bool b )       { m_saveCuCostInSCIPU = b;  }
  void setNumCuInSCIPU            ( uint8_t i )    { m_numCuInSCIPU = i; }
  void saveCuAreaCostInSCIPU      ( Area area, double cost );
  void initCuAreaCostInSCIPU      ();

  bool estIntraPredLumaQT         ( CodingUnit &cu, Partitioner &pm, double bestCost = MAX_DOUBLE);
  void estIntraPredChromaQT       ( CodingUnit& cu, Partitioner& partitioner, const double maxCostAllowed );

private:
  double    xFindInterCUCost          ( CodingUnit &cu );
  void      xPreCheckMTS              ( TransformUnit &tu, std::vector<TrMode> *trModes, const int maxCand, PelUnitBuf *pPred, const ComponentID& compID = COMP_Y);
  void      xEstimateLumaRdModeList   ( int& numModesForFullRD,
                                        static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& RdModeList,
                                        static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& HadModeList,
                                        static_vector<double, FAST_UDI_MAX_RDMODE_NUM>& CandCostList,
                                        static_vector<double, FAST_UDI_MAX_RDMODE_NUM>& CandHadList, CodingUnit& cu, bool testMip);

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------
  uint64_t  xFracModeBitsIntraLuma    ( const CodingUnit& cu, const unsigned* mpmLst );

  void      xEncIntraHeader           ( CodingStructure &cs, Partitioner& pm, const bool luma );
  void      xEncSubdivCbfQT           ( CodingStructure &cs, Partitioner& pm, const bool luma );
  uint64_t  xGetIntraFracBitsQT       ( CodingStructure &cs, Partitioner &pm, const bool luma, CUCtx *cuCtx = nullptr );

  uint64_t  xGetIntraFracBitsQTChroma ( const TransformUnit& tu, const ComponentID compID, CUCtx *cuCtx );

  void     xEncCoeffQT                ( CodingStructure& cs, Partitioner& pm, const ComponentID compID, CUCtx* cuCtx = nullptr, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );

  void      xIntraCodingTUBlock       ( TransformUnit &tu, const ComponentID compID, const bool checkCrossCPrediction, Distortion &ruiDist, uint32_t *numSig = nullptr, PelUnitBuf *pPred = nullptr, const bool loadTr = false);
  ChromaCbfs xIntraChromaCodingQT     ( CodingStructure& cs, Partitioner& pm );
  void     xIntraCodingLumaQT         ( CodingStructure& cs, Partitioner& pm, PelUnitBuf* pPred, const double bestCostSoFar, int numMode, bool disableMTS);
  double   xTestISP                   ( CodingStructure& cs, Partitioner& pm, double bestCostSoFar, PartSplit ispType, bool& splitcbf, uint64_t& singleFracBits, Distortion& singleDistLuma, CUCtx& cuCtx);
  int      xSpeedUpISP                ( int speed, bool& testISP, int mode, int& noISP, int& endISP, CodingUnit& cu, static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& RdModeList,const ModeInfo& bestPUMode, int bestISP, int bestLfnstIdx);
  void     xSpeedUpIntra              ( double bestcost, int& EndMode, int& speedIntra, CodingUnit& cu);

  template<typename T, size_t N, int M>
  void      xReduceHadCandList        ( static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, SortedPelUnitBufs<M>& sortedPelBuffer, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const CodingUnit& cu, const bool fastMip);

};// END CLASS DEFINITION EncSearch

} // namespace vvenc

//! \}


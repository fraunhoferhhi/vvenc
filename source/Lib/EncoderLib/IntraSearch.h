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
class EncCfg;

/// encoder search class
class IntraSearch : public IntraPrediction
{
private:
  CodingStructure***  m_pTempCS;
  CodingStructure***  m_pBestCS;
  CodingStructure**   m_pSaveCS;
  bool                m_saveCuCostInSCIPU;
  uint8_t             m_numCuInSCIPU;
  Area                m_cuAreaInSCIPU[NUM_INTER_CU_INFO_SAVE];
  double              m_cuCostInSCIPU[NUM_INTER_CU_INFO_SAVE];

  struct ModeInfo
  {
    bool     mipFlg;    // CU::mipFlag
    bool     mipTrFlg;  // PU::mipTransposeFlag
    int8_t   mRefId;    // PU::multiRefIdx
    uint8_t  ispMod;    // CU::ispMode
    uint8_t  modeId;    // PU::intraDir[CH_L]

    ModeInfo() : mipFlg(false), mipTrFlg(false), mRefId(0), ispMod(NOT_INTRA_SUBPARTITIONS), modeId(0) {}
    ModeInfo(const bool mipf, const bool miptf, const int8_t mrid, const uint8_t ispm, const uint8_t mode) : mipFlg(mipf), mipTrFlg(miptf), mRefId(mrid), ispMod(ispm), modeId(mode) {}
    bool operator==(const ModeInfo cmp) const { return (0 == ::memcmp(this,&cmp,sizeof(ModeInfo))); }
  };
#if ISP_VVC 

  struct ISPTestedModesInfo
  {
    int                                         numTotalParts[2];
    int                                         bestModeSoFar;
    ISPType                                     bestSplitSoFar;
    double                                      bestCost[2];
    bool                                        splitIsFinished[2];
    int                                         subTuCounter;
    PartSplit                                   IspType;

    // set everything to default values
    void clear()
    {
      for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
      {
        numTotalParts[splitIdx]   = 0;
        splitIsFinished[splitIdx] = false;
        bestCost[splitIdx] = MAX_DOUBLE;
      }
      bestModeSoFar      = -1;
      bestSplitSoFar     = NOT_INTRA_SUBPARTITIONS;
      subTuCounter = -1;
      IspType      = TU_NO_ISP;
    }
    void init(const int numTotalPartsHor, const int numTotalPartsVer)
    {
      clear();
      const int horSplit = HOR_INTRA_SUBPARTITIONS - 1, verSplit = VER_INTRA_SUBPARTITIONS - 1;
      numTotalParts[horSplit]   = numTotalPartsHor;
      numTotalParts[verSplit]   = numTotalPartsVer;
      splitIsFinished[horSplit] = (numTotalParts[horSplit] == 0);
      splitIsFinished[verSplit] = (numTotalParts[verSplit] == 0);
      subTuCounter = -1;
      IspType      = TU_NO_ISP;
    }
  };

  ISPTestedModesInfo                                       m_ispTestedModes[NUM_LFNST_NUM_PER_SET];
#endif

protected:
  // interface to option
  const EncCfg*   m_pcEncCfg;

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
  void init                       ( const EncCfg &encCfg, TrQuant *pTrQuant, RdCost *pRdCost, SortedPelUnitBufs<SORTED_BUFS> *pSortedPelUnitBufs, XUCache &unitCache);
  void setCtuEncRsrc              ( CABACWriter* cabacEstimator, CtxCache* ctxCache );
  void destroy                    ();

  CodingStructure  **getSaveCSBuf () { return m_pSaveCS; }

  bool getSaveCuCostInSCIPU       ()               { return m_saveCuCostInSCIPU; }
  void setSaveCuCostInSCIPU       ( bool b )       { m_saveCuCostInSCIPU = b;  }
  void setNumCuInSCIPU            ( uint8_t i )    { m_numCuInSCIPU = i; }
  void saveCuAreaCostInSCIPU      ( Area area, double cost );
  void initCuAreaCostInSCIPU      ();

  bool estIntraPredLumaQT         ( CodingUnit &cu, Partitioner &pm, double bestCost = MAX_DOUBLE);
#if ISP_VVC
  void estIntraPredChromaQT       ( CodingUnit& cu, Partitioner& partitioner, const double maxCostAllowed );
#else
  void estIntraPredChromaQT       ( CodingUnit &cu, Partitioner& pm );
#endif

private:
  double    xFindInterCUCost          ( CodingUnit &cu );
  void      xPreCheckMTS              ( TransformUnit &tu, std::vector<TrMode> *trModes, const int maxCand, PelUnitBuf *pPred);
  void      xEstimateLumaRdModeList   ( int& numModesForFullRD,
                                        static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& RdModeList,
                                        static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& HadModeList,
                                        static_vector<double, FAST_UDI_MAX_RDMODE_NUM>& CandCostList,
                                        static_vector<double, FAST_UDI_MAX_RDMODE_NUM>& CandHadList, CodingUnit& cu, bool testMip);

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------
  uint64_t  xFracModeBitsIntraLuma    ( const CodingUnit& cu );

  void      xEncIntraHeader           ( CodingStructure &cs, Partitioner& pm, const bool luma );
  void      xEncSubdivCbfQT           ( CodingStructure &cs, Partitioner& pm, const bool luma );
  uint64_t  xGetIntraFracBitsQT       ( CodingStructure &cs, Partitioner &pm, const bool luma, CUCtx *cuCtx = nullptr );

  uint64_t  xGetIntraFracBitsQTChroma ( const TransformUnit& tu, const ComponentID compID, CUCtx *cuCtx );

#if ISP_VVC
  void     xEncCoeffQT                ( CodingStructure& cs, Partitioner& pm, const ComponentID compID, CUCtx* cuCtx = nullptr, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
#else
  void      xEncCoeffQT               ( CodingStructure &cs, Partitioner &pm, const ComponentID compID, CUCtx *cuCtx = nullptr );
#endif

  void      xIntraCodingTUBlock       ( TransformUnit &tu, const ComponentID compID, const bool checkCrossCPrediction, Distortion &ruiDist, uint32_t *numSig = nullptr, PelUnitBuf *pPred = nullptr, const bool loadTr = false);
#if ISP_VVC
  ChromaCbfs xIntraChromaCodingQT     ( CodingStructure& cs, Partitioner& pm );
  void xIntraCodingLumaQT             ( CodingStructure& cs, Partitioner& pm, PelUnitBuf* pPred, const double bestCostSoFar, int numMode );
  double xTestISP                     ( CodingStructure& cs, Partitioner& pm, double bestCostSoFar, PartSplit ispType, bool& splitcbf, uint64_t& singleFracBits, Distortion& singleDistLuma, CUCtx& cuCtx);
  int  xSpeedISP                      ( int speed, bool& testISP, int mode, int& noISP, int& endISP, CodingUnit& cu, static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& RdModeList, ModeInfo uiBestPUMode, int bestISP, int bestLfnstIdx);
#else 
  void      xIntraChromaCodingQT      ( CodingStructure &cs, Partitioner& pm );
  void      xIntraCodingLumaQT        ( CodingStructure &cs, Partitioner &pm, PelUnitBuf *pPred, const double bestCostSoFar );
#endif

  template<typename T, size_t N, int M>
  void      xReduceHadCandList        ( static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, SortedPelUnitBufs<M>& sortedPelBuffer, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const CodingUnit& cu, const bool fastMip);

};// END CLASS DEFINITION EncSearch

} // namespace vvenc

//! \}


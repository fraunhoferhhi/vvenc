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


/** \file     EncSearch.cpp
 *  \brief    encoder intra search class
 */

#include "IntraSearch.h"
#include "EncPicture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/Reshape.h"
#include <math.h>
#include "../../../include/vvenc/EncCfg.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

#define PLTCtx(c) SubCtx( Ctx::Palette, c )
IntraSearch::IntraSearch()
  : m_pTempCS       (nullptr)
  , m_pBestCS       (nullptr)
  , m_pSaveCS       (nullptr)
  , m_pcEncCfg      (nullptr)
  , m_pcTrQuant     (nullptr)
  , m_pcRdCost      (nullptr)
  , m_CABACEstimator(nullptr)
  , m_CtxCache      (nullptr)
{
}

void IntraSearch::init(const EncCfg &encCfg, TrQuant *pTrQuant, RdCost *pRdCost, SortedPelUnitBufs<SORTED_BUFS> *pSortedPelUnitBufs, XUCache &unitCache )
{
  IntraPrediction::init( encCfg.m_internChromaFormat, encCfg.m_internalBitDepth[ CH_L ] );

  m_pcEncCfg          = &encCfg;
  m_pcTrQuant         = pTrQuant;
  m_pcRdCost          = pRdCost;
  m_SortedPelUnitBufs = pSortedPelUnitBufs;

  const ChromaFormat chrFormat = encCfg.m_internChromaFormat;
  const int maxCUSize          = encCfg.m_CTUSize;

  const int numWidths  = MAX_CU_SIZE_IDX;
  const int numHeights = MAX_CU_SIZE_IDX;

  m_pBestCS = new CodingStructure**[numWidths];
  m_pTempCS = new CodingStructure**[numWidths];

  for( int wIdx = 0; wIdx < numWidths; wIdx++ )
  {
    m_pBestCS[wIdx] = new CodingStructure*[numHeights];
    m_pTempCS[wIdx] = new CodingStructure*[numHeights];

    for( int hIdx = 0; hIdx < numHeights; hIdx++ )
    {
      if( wIdx < 2 || hIdx < 2 )
      {
        m_pBestCS[wIdx][hIdx] = nullptr;
        m_pTempCS[wIdx][hIdx] = nullptr;
        continue;
      }

      m_pBestCS[wIdx][hIdx] = new CodingStructure( unitCache, nullptr );
      m_pTempCS[wIdx][hIdx] = new CodingStructure( unitCache, nullptr );

      Area area = Area( 0, 0, 1<<wIdx, 1<<hIdx );
      m_pBestCS[wIdx][hIdx]->create( chrFormat, area, false );
      m_pTempCS[wIdx][hIdx]->create( chrFormat, area, false );
    }
  }

  const int uiNumSaveLayersToAllocate = 3;
  m_pSaveCS = new CodingStructure*[uiNumSaveLayersToAllocate];
  for( int layer = 0; layer < uiNumSaveLayersToAllocate; layer++ )
  {
    m_pSaveCS[ layer ] = new CodingStructure( unitCache, nullptr );
    m_pSaveCS[ layer ]->create( chrFormat, Area( 0, 0, maxCUSize, maxCUSize ), false );
    m_pSaveCS[ layer ]->initStructData();
  }

}


void IntraSearch::destroy()
{
  if ( m_pSaveCS )
  {
    const int uiNumSaveLayersToAllocate = 3;
    for( int layer = 0; layer < uiNumSaveLayersToAllocate; layer++ )
    {
      if ( m_pSaveCS[ layer ] ) { m_pSaveCS[ layer ]->destroy(); delete m_pSaveCS[ layer ]; }
    }
    delete[] m_pSaveCS;
    m_pSaveCS = nullptr;
  }

  const int numWidths  = MAX_CU_SIZE_IDX;
  const int numHeights = MAX_CU_SIZE_IDX;
  for( int wIdx = 0; wIdx < numWidths; wIdx++ )
  {
    for( int hIdx = 0; hIdx < numHeights; hIdx++ )
    {
      if ( m_pBestCS && m_pBestCS[ wIdx ] && m_pBestCS[ wIdx ][ hIdx ] ) { m_pBestCS[ wIdx ][ hIdx ]->destroy(); delete m_pBestCS[ wIdx ][ hIdx ]; }
      if ( m_pTempCS && m_pTempCS[ wIdx ] && m_pTempCS[ wIdx ][ hIdx ] ) { m_pTempCS[ wIdx ][ hIdx ]->destroy(); delete m_pTempCS[ wIdx ][ hIdx ]; }
    }
    if ( m_pBestCS  && m_pBestCS[ wIdx ]  ) { delete[] m_pBestCS[ wIdx ];  }
    if ( m_pTempCS  && m_pTempCS[ wIdx ]  ) { delete[] m_pTempCS[ wIdx ];  }
  }
  if ( m_pBestCS  ) { delete[] m_pBestCS;  m_pBestCS  = nullptr; }
  if ( m_pTempCS  ) { delete[] m_pTempCS;  m_pTempCS  = nullptr; }

}

IntraSearch::~IntraSearch()
{
  destroy();
}

void IntraSearch::setCtuEncRsrc( CABACWriter* cabacEstimator, CtxCache *ctxCache )
{
  m_CABACEstimator = cabacEstimator;
  m_CtxCache       = ctxCache;
}

//////////////////////////////////////////////////////////////////////////
// INTRA PREDICTION
//////////////////////////////////////////////////////////////////////////
static constexpr double COST_UNKNOWN = -65536.0;

double IntraSearch::xFindInterCUCost( CodingUnit &cu )
{
  if( CU::isConsIntra(cu) && !cu.slice->isIntra() )
  {
    //search corresponding inter CU cost
    for( int i = 0; i < m_numCuInSCIPU; i++ )
    {
      if( cu.lumaPos() == m_cuAreaInSCIPU[i].pos() && cu.lumaSize() == m_cuAreaInSCIPU[i].size() )
      {
        return m_cuCostInSCIPU[i];
      }
    }
  }
  return COST_UNKNOWN;
}

void IntraSearch::xEstimateLumaRdModeList(int& numModesForFullRD,
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& RdModeList,
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& HadModeList,
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM>& CandCostList,
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM>& CandHadList, CodingUnit& cu, bool testMip )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 0, g_timeProfiler, P_INTRA_EST_RD_CAND_LUMA, cu.cs, CH_L );
  const uint16_t intra_ctx_size = Ctx::IntraLumaMpmFlag.size() + Ctx::IntraLumaPlanarFlag.size() + Ctx::MultiRefLineIdx.size() + Ctx::ISPMode.size() + Ctx::MipFlag.size();
  const TempCtx  ctxStartIntraCtx(m_CtxCache, SubCtx(CtxSet(Ctx::IntraLumaMpmFlag(), intra_ctx_size), m_CABACEstimator->getCtx()));
  const double   sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  const int numModesAvailable = NUM_LUMA_MODE; // total number of Intra modes

  CHECK(numModesForFullRD >= numModesAvailable, "Too many modes for full RD search");

  const SPS& sps     = *cu.cs->sps;
  const bool fastMip = sps.MIP && m_pcEncCfg->m_useFastMIP;

  // this should always be true
  CHECK( !cu.Y().valid(), "CU is not valid" );

  const CompArea& area = cu.Y();

  const UnitArea localUnitArea(area.chromaFormat, Area(0, 0, area.width, area.height));
  if( testMip)
  {
    numModesForFullRD += fastMip? std::max(numModesForFullRD, floorLog2(std::min(cu.lwidth(), cu.lheight())) - m_pcEncCfg->m_useFastMIP) : numModesForFullRD;
    m_SortedPelUnitBufs->prepare( localUnitArea, numModesForFullRD + 1 );
  }
  else
  {
    m_SortedPelUnitBufs->prepare( localUnitArea, numModesForFullRD );
  }

  CPelBuf piOrg   = cu.cs->getOrgBuf(COMP_Y);
  PelBuf piPred  = m_SortedPelUnitBufs->getTestBuf(COMP_Y);

  const ReshapeData& reshapeData = cu.cs->picture->reshapeData;
  if (cu.cs->picHeader->lmcsEnabled && reshapeData.getCTUFlag())
  {
    piOrg = cu.cs->getRspOrgBuf();
  }
  DistParam distParam    = m_pcRdCost->setDistParam( piOrg, piPred, sps.bitDepths[ CH_L ], DF_HAD_2SAD); // Use HAD (SATD) cost

  const int numHadCand = (testMip ? 2 : 1) * 3;

  //*** Derive (regular) candidates using Hadamard
  cu.mipFlag = false;
  cu.multiRefIdx = 0;

  //===== init pattern for luma prediction =====
  initIntraPatternChType(cu, cu.Y(), true);

  bool satdChecked[NUM_INTRA_MODE] = { false };

  unsigned mpmLst[NUM_MOST_PROBABLE_MODES];
  CU::getIntraMPMs(cu, mpmLst);

  for( unsigned mode = 0; mode < numModesAvailable; mode++ )
  {
    // Skip checking extended Angular modes in the first round of SATD
    if( mode > DC_IDX && ( mode & 1 ) )
    {
      continue;
    }

    cu.intraDir[0] = mode;

    initPredIntraParams(cu, cu.Y(), sps);
    distParam.cur.buf = piPred.buf = m_SortedPelUnitBufs->getTestBuf().Y().buf;
    predIntraAng( COMP_Y, piPred, cu);

    // Use the min between SAD and HAD as the cost criterion
    // SAD is scaled by 2 to align with the scaling of HAD
    Distortion minSadHad = distParam.distFunc(distParam);

    uint64_t fracModeBits = xFracModeBitsIntraLuma( cu, mpmLst );

    //restore ctx
    m_CABACEstimator->getCtx() = SubCtx(CtxSet(Ctx::IntraLumaMpmFlag(), intra_ctx_size), ctxStartIntraCtx);

    double cost = ( double ) minSadHad + (double)fracModeBits * sqrtLambdaForFirstPass;
    DTRACE(g_trace_ctx, D_INTRA_COST, "IntraHAD: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, mode);

    int insertPos = -1;
    updateCandList( ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, RdModeList, CandCostList, numModesForFullRD, &insertPos );
    updateCandList( ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), (double)minSadHad, HadModeList, CandHadList,  numHadCand );
    m_SortedPelUnitBufs->insert( insertPos, (int)RdModeList.size() );

    satdChecked[mode] = true;
  }

  std::vector<ModeInfo> parentCandList( RdModeList.cbegin(), RdModeList.cend());

  // Second round of SATD for extended Angular modes
  for (unsigned modeIdx = 0; modeIdx < numModesForFullRD; modeIdx++)
  {
    unsigned parentMode = parentCandList[modeIdx].modeId;
    if (parentMode > (DC_IDX + 1) && parentMode < (NUM_LUMA_MODE - 1))
    {
      for (int subModeIdx = -1; subModeIdx <= 1; subModeIdx += 2)
      {
        unsigned mode = parentMode + subModeIdx;

        if( ! satdChecked[mode])
        {
          cu.intraDir[0] = mode;

          initPredIntraParams(cu, cu.Y(), sps);
          distParam.cur.buf = piPred.buf = m_SortedPelUnitBufs->getTestBuf().Y().buf;
          predIntraAng(COMP_Y, piPred, cu );

          // Use the min between SAD and SATD as the cost criterion
          // SAD is scaled by 2 to align with the scaling of HAD
          Distortion minSadHad = distParam.distFunc(distParam);

          uint64_t fracModeBits = xFracModeBitsIntraLuma( cu, mpmLst );
          //restore ctx
          m_CABACEstimator->getCtx() = SubCtx(CtxSet(Ctx::IntraLumaMpmFlag(), intra_ctx_size), ctxStartIntraCtx);

          double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;
//          DTRACE(g_trace_ctx, D_INTRA_COST, "IntraHAD2: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, mode);

          int insertPos = -1;
          updateCandList( ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, mode ), cost, RdModeList, CandCostList, numModesForFullRD, &insertPos );
          updateCandList( ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, mode ), (double)minSadHad, HadModeList, CandHadList,  numHadCand );
          m_SortedPelUnitBufs->insert(insertPos, (int)RdModeList.size());

          satdChecked[mode] = true;
        }
      }
    }
  }

  const bool isFirstLineOfCtu = (((cu.block(COMP_Y).y)&((cu.cs->sps)->CTUSize - 1)) == 0);
  if( m_pcEncCfg->m_MRL && ! isFirstLineOfCtu )
  {
    cu.multiRefIdx = 1;
    unsigned  multiRefMPM [NUM_MOST_PROBABLE_MODES];
    CU::getIntraMPMs(cu, multiRefMPM);

    for (int mRefNum = 1; mRefNum < MRL_NUM_REF_LINES; mRefNum++)
    {
      int multiRefIdx = MULTI_REF_LINE_IDX[mRefNum];

      cu.multiRefIdx = multiRefIdx;
      initIntraPatternChType(cu, cu.Y(), true);

      for (int x = 1; x < NUM_MOST_PROBABLE_MODES; x++)
      {
        cu.intraDir[0] = multiRefMPM[x];
        initPredIntraParams(cu, cu.Y(), sps);
        distParam.cur.buf = piPred.buf = m_SortedPelUnitBufs->getTestBuf().Y().buf;
        predIntraAng(COMP_Y, piPred, cu);

        // Use the min between SAD and SATD as the cost criterion
        // SAD is scaled by 2 to align with the scaling of HAD
        Distortion minSadHad = distParam.distFunc(distParam);

        // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
        uint64_t fracModeBits = xFracModeBitsIntraLuma( cu, mpmLst );

        //restore ctx
        m_CABACEstimator->getCtx() = SubCtx(CtxSet(Ctx::IntraLumaMpmFlag(), intra_ctx_size), ctxStartIntraCtx);

        double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;
//        DTRACE(g_trace_ctx, D_INTRA_COST, "IntraMRL: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, cu.intraDir[0]);

        int insertPos = -1;
        updateCandList( ModeInfo( false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, cu.intraDir[0] ), cost, RdModeList,  CandCostList, numModesForFullRD, &insertPos );
        updateCandList( ModeInfo( false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, cu.intraDir[0] ), (double)minSadHad, HadModeList, CandHadList,  numHadCand );
        m_SortedPelUnitBufs->insert(insertPos, (int)RdModeList.size());
      }
    }
    cu.multiRefIdx = 0;
  }

  if (testMip)
  {
    cu.mipFlag = true;
    cu.multiRefIdx = 0;

    double mipHadCost[MAX_NUM_MIP_MODE] = { MAX_DOUBLE };

    initIntraPatternChType(cu, cu.Y());
    initIntraMip( cu );

    const int transpOff    = getNumModesMip( cu.Y() );
    const int numModesFull = (transpOff << 1);
    for( uint32_t uiModeFull = 0; uiModeFull < numModesFull; uiModeFull++ )
    {
      const bool     isTransposed = (uiModeFull >= transpOff ? true : false);
      const uint32_t uiMode       = (isTransposed ? uiModeFull - transpOff : uiModeFull);

      cu.mipTransposedFlag = isTransposed;
      cu.intraDir[CH_L] = uiMode;
      distParam.cur.buf = piPred.buf = m_SortedPelUnitBufs->getTestBuf().Y().buf;
      predIntraMip(piPred, cu);

      // Use the min between SAD and HAD as the cost criterion
      // SAD is scaled by 2 to align with the scaling of HAD
      Distortion minSadHad = distParam.distFunc(distParam);

      uint64_t fracModeBits = xFracModeBitsIntraLuma( cu, mpmLst );

      //restore ctx
      m_CABACEstimator->getCtx() = SubCtx(CtxSet(Ctx::IntraLumaMpmFlag(), intra_ctx_size), ctxStartIntraCtx);

      double cost = double(minSadHad) + double(fracModeBits) * sqrtLambdaForFirstPass;
      mipHadCost[uiModeFull] = cost;
      DTRACE(g_trace_ctx, D_INTRA_COST, "IntraMIP: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, uiModeFull);

      int insertPos = -1;
      updateCandList( ModeInfo( true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, cu.intraDir[0] ), cost, RdModeList,  CandCostList, numModesForFullRD+1, &insertPos );
      updateCandList( ModeInfo( true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, cu.intraDir[0] ), 0.8*(double)minSadHad, HadModeList, CandHadList,  numHadCand );
      m_SortedPelUnitBufs->insert(insertPos, (int)RdModeList.size());
    }

    const double thresholdHadCost = 1.0 + 1.4 / sqrt((double)(cu.lwidth()*cu.lheight()));
    xReduceHadCandList(RdModeList, CandCostList, *m_SortedPelUnitBufs, numModesForFullRD, thresholdHadCost, mipHadCost, cu, fastMip);
  }

  if( m_pcEncCfg->m_bFastUDIUseMPMEnabled )
  {
    const int numMPMs = NUM_MOST_PROBABLE_MODES;
    unsigned  intraMpms[numMPMs];

    cu.multiRefIdx = 0;

    const int numCand = CU::getIntraMPMs( cu, intraMpms );
    ModeInfo mostProbableMode(false, false, 0, NOT_INTRA_SUBPARTITIONS, 0);

    for( int j = 0; j < numCand; j++ )
    {
      bool mostProbableModeIncluded = false;
      mostProbableMode.modeId = intraMpms[j];

      for( int i = 0; i < numModesForFullRD; i++ )
      {
        mostProbableModeIncluded |= ( mostProbableMode == RdModeList[i] );
      }
      if( !mostProbableModeIncluded )
      {
        numModesForFullRD++;
        RdModeList.push_back( mostProbableMode );
        CandCostList.push_back(0);
      }
    }
  }
}

bool IntraSearch::estIntraPredLumaQT(CodingUnit &cu, Partitioner &partitioner, double bestCost)
{
  CodingStructure       &cs           = *cu.cs;
  const int             width         = partitioner.currArea().lwidth();
  const int             height        = partitioner.currArea().lheight();

  //===== loop over partitions =====

  const TempCtx ctxStart           ( m_CtxCache, m_CABACEstimator->getCtx() );

  // variables for saving fast intra modes scan results across multiple LFNST passes
  double costInterCU = xFindInterCUCost( cu );

  bool validReturn = false;

  //===== determine set of modes to be tested (using prediction signal only) =====
  int numModesAvailable = NUM_LUMA_MODE; // total number of Intra modes
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> RdModeList;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> HadModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> CandCostList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> CandHadList;

  int numModesForFullRD = g_aucIntraModeNumFast_UseMPM_2D[Log2(width) - MIN_CU_LOG2][Log2(height) - MIN_CU_LOG2];

#if INTRA_FULL_SEARCH
  numModesForFullRD = numModesAvailable;
#endif
  const SPS& sps = *cu.cs->sps;
  const bool mipAllowed = sps.MIP && cu.lwidth() <= sps.getMaxTbSize() && cu.lheight() <= sps.getMaxTbSize() && ((cu.lfnstIdx == 0) || allowLfnstWithMip(cu.lumaSize()));
  const int SizeThr = 8>>std::max(0,m_pcEncCfg->m_useFastMIP-2);
  const bool testMip    = mipAllowed && (cu.lwidth() <= (SizeThr * cu.lheight()) && cu.lheight() <= (SizeThr * cu.lwidth())) && (cu.lwidth() <= MIP_MAX_WIDTH && cu.lheight() <= MIP_MAX_HEIGHT);
#if ISP_VVC
  bool testISP = sps.ISP && CU::canUseISP(width, height, cu.cs->sps->getMaxTbSize());
  if (testISP)
  {
    int numTotalPartsHor = (int)width >> floorLog2(CU::getISPSplitDim(width, height, TU_1D_VERT_SPLIT));
    int numTotalPartsVer = (int)height >> floorLog2(CU::getISPSplitDim(width, height, TU_1D_HORZ_SPLIT));
    m_ispTestedModes[0].init(numTotalPartsHor, numTotalPartsVer);
    // the total number of subpartitions is modified to take into account the cases where LFNST cannot be combined with
    // ISP due to size restrictions
    numTotalPartsHor = sps.LFNST && CU::canUseLfnstWithISP(cu.Y(), HOR_INTRA_SUBPARTITIONS) ? numTotalPartsHor : 0;
    numTotalPartsVer = sps.LFNST && CU::canUseLfnstWithISP(cu.Y(), VER_INTRA_SUBPARTITIONS) ? numTotalPartsVer : 0;
    for (int j = 1; j < NUM_LFNST_NUM_PER_SET; j++)
    {
      m_ispTestedModes[j].init(numTotalPartsHor, numTotalPartsVer);
    }
    testISP = m_ispTestedModes[0].numTotalParts[0];
  }
  else
  {
    m_ispTestedModes[0].init(0, 0);
  }
#endif

  xEstimateLumaRdModeList(numModesForFullRD, RdModeList, HadModeList, CandCostList, CandHadList, cu, testMip);

  CHECK( (size_t)numModesForFullRD != RdModeList.size(), "Inconsistent state!" );

  // after this point, don't use numModesForFullRD
  if( m_pcEncCfg->m_usePbIntraFast && !cs.slice->isIntra() && RdModeList.size() < numModesAvailable && !cs.slice->disableSATDForRd )
  {
    double pbintraRatio = PBINTRA_RATIO;
    int maxSize = -1;
    ModeInfo bestMipMode;
    int bestMipIdx = -1;
    for( int idx = 0; idx < RdModeList.size(); idx++ )
    {
      if( RdModeList[idx].mipFlg )
      {
        bestMipMode = RdModeList[idx];
        bestMipIdx = idx;
        break;
      }
    }
    const int numHadCand = 3;
    for (int k = numHadCand - 1; k >= 0; k--)
    {
      if (CandHadList.size() < (k + 1) || CandHadList[k] > cs.interHad * pbintraRatio) { maxSize = k; }
    }
    if (maxSize > 0)
    {
      RdModeList.resize(std::min<size_t>(RdModeList.size(), maxSize));
      if( bestMipIdx >= 0 )
      {
        if( RdModeList.size() <= bestMipIdx )
        {
          RdModeList.push_back(bestMipMode);
          m_SortedPelUnitBufs->swap( maxSize, bestMipIdx );
        }
      }
    }
    if (maxSize == 0)
    {
      cs.dist = MAX_DISTORTION;
      cs.interHad = 0;
      return false;
    }
  }

  //===== check modes (using r-d costs) =====
  ModeInfo       uiBestPUMode;

  CodingStructure *csTemp = m_pTempCS[Log2(cu.lwidth())][Log2(cu.lheight())];
  CodingStructure *csBest = m_pBestCS[Log2(cu.lwidth())][Log2(cu.lheight())];

  csTemp->slice   = csBest->slice   = cs.slice;
  csTemp->picture = csBest->picture = cs.picture;
  csTemp->initStructData();
  csBest->initStructData();

  int bestLfnstIdx = 0;
#if BDPCM_VVC
  bool testBDPCM = sps.BDPCM && CU::bdpcmAllowed(cu, ComponentID(partitioner.chType));
  int            bestBDPCMMode = 0;
#if DETECT_SC
  testBDPCM &= cs.picture->useSC;
#endif
#endif
#if ISP_VVC
  int bestISP = 0;
  bool mip = 0;
  int  mrl = 0;
  int EndMode = (int)RdModeList.size();
  bool useISPlfnst = testISP && sps.LFNST;
  double bestCostIsp[2] = { MAX_DOUBLE, MAX_DOUBLE };
#if TS_VVC
  bool noLFNST_ts = false;
#endif
#if BDPCM_VVC
  for (int mode_cur = 0; mode_cur < EndMode + (2 * int(testBDPCM)); mode_cur++)
#else
  for (int mode = 0; mode < EndMode; mode++)
#endif
#else
  for (int mode = 0; mode < (int)RdModeList.size(); mode++)
#endif
  {
#if BDPCM_VVC
    int mode = mode_cur;
    if (mode_cur >= EndMode)
    {
      mode = mode_cur - EndMode ? -1 : -2;
      testISP = false;
    }
#endif
    // set CU/PU to luma prediction mode
    ModeInfo testMode;
#if ISP_VVC
    int noISP = 0;
    int endISP = testISP ? 2 : 0;
#if TS_VVC
    bool noLFNST = false || noLFNST_ts;
#else
    bool noLFNST = false;
#endif
    if (mode && useISPlfnst)
    {
      if (bestCostIsp[0] > (bestCostIsp[1] * 1.4))
      {
        noLFNST = true;
      }
      if (mode > 2)
      {
        endISP = 0;
        testISP = false;
      }
    }
    if (testISP)
    {
      xSpeedISP(1, testISP, mode, noISP, endISP, cu, RdModeList, uiBestPUMode, bestISP, bestLfnstIdx);
    }
    for (int ispM = 0; ispM <= endISP; ispM++)
    {
      if (ispM && (ispM == noISP))
      {
        continue;
      }
#endif
      {

#if BDPCM_VVC
        if (mode < 0)
        {
          cu.bdpcmMode = -mode;
          testMode = ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, cu.bdpcmMode == 2 ? VER_IDX : HOR_IDX);
        }
        else
#endif
        {
          testMode = RdModeList[mode];
          cu.bdpcmMode = 0;
        }
#if ISP_VVC
        cu.ispMode = ispM;
#else
        cu.ispMode = testMode.ispMod;
#endif
        cu.mipFlag = testMode.mipFlg;
        cu.mipTransposedFlag = testMode.mipTrFlg;
        cu.multiRefIdx = testMode.mRefId;
        cu.intraDir[CH_L] = testMode.modeId;
#if ISP_VVC 
        if (cu.ispMode)
        {
          int stopFound = xSpeedISP(0, testISP, mode, noISP, endISP, cu, RdModeList, uiBestPUMode, bestISP, 0);
          if (stopFound)
          {
            continue;
          }
        }
#endif
        CHECK(cu.mipFlag && cu.multiRefIdx, "Error: combination of MIP and MRL not supported");
        CHECK(cu.multiRefIdx && (cu.intraDir[0] == PLANAR_IDX), "Error: combination of MRL and Planar mode not supported");
        CHECK(cu.ispMode && cu.mipFlag, "Error: combination of ISP and MIP not supported");
        CHECK(cu.ispMode && cu.multiRefIdx, "Error: combination of ISP and MRL not supported");
      }

      // determine residual for partition
      cs.initSubStructure(*csTemp, partitioner.chType, cs.area, true);
#if ISP_VVC
      int doISP = (((cu.ispMode == 0) && noLFNST) || (useISPlfnst && mode && cu.ispMode && (bestLfnstIdx == 0))) ?-mode :mode;
      xIntraCodingLumaQT(*csTemp, partitioner, m_SortedPelUnitBufs->getBufFromSortedList(mode), bestCost, doISP);
#else
      xIntraCodingLumaQT(*csTemp, partitioner, m_SortedPelUnitBufs->getBufFromSortedList(mode), bestCost);
#endif

      DTRACE(g_trace_ctx, D_INTRA_COST, "IntraCost T [x=%d,y=%d,w=%d,h=%d] %f (%d,%d,%d,%d,%d,%d) \n", cu.blocks[0].x,
        cu.blocks[0].y, width, height, csTemp->cost, testMode.modeId, testMode.ispMod,
        cu.multiRefIdx, cu.mipFlag, cu.lfnstIdx, cu.mtsFlag);

#if ISP_VVC
      if (cu.ispMode && !csTemp->cus[0]->firstTU->cbf[COMP_Y])
      {
        csTemp->cost = MAX_DOUBLE;
        csTemp->costDbOffset = 0;
      }
      if (useISPlfnst)
      {
        if (cu.ispMode == 0)
        {
          bestCostIsp[0] = csTemp->cost < bestCostIsp[0] ? csTemp->cost : bestCostIsp[0];
        }
        else
        {
          bestCostIsp[1] = csTemp->cost < bestCostIsp[1] ? csTemp->cost : bestCostIsp[1];
        }
      }
#endif

      // check r-d cost
      if (csTemp->cost < csBest->cost)
      {
        validReturn = true;
        std::swap(csTemp, csBest);
        uiBestPUMode = testMode;
        bestLfnstIdx = csBest->cus[0]->lfnstIdx;
#if BDPCM_VVC
        bestBDPCMMode = cu.bdpcmMode;
#endif
#if ISP_VVC
        mip = csBest->cus[0]->mipFlag;
        mrl = csBest->cus[0]->multiRefIdx;
        bestISP = csBest->cus[0]->ispMode;
#if  ISP_VVC
        m_ispTestedModes[bestLfnstIdx].bestSplitSoFar = ISPType(bestISP);
#endif
        if (csBest->cost < bestCost)
        {
          bestCost = csBest->cost;
        }
#endif
#if TS_VVC
        if (csBest->getTU(partitioner.chType)->mtsIdx[COMP_Y] == MTS_SKIP)
        {
          if ((floorLog2(csBest->getTU(partitioner.chType)->blocks[COMP_Y].width) + floorLog2(csBest->getTU(partitioner.chType)->blocks[COMP_Y].height)) >= 6)
          {
            noLFNST_ts = 1;
          }
        }
#endif
      }

      // reset context models
      m_CABACEstimator->getCtx() = ctxStart;

      csTemp->releaseIntermediateData();

      if (m_pcEncCfg->m_fastLocalDualTreeMode && CU::isConsIntra(cu) && !cu.slice->isIntra() && csBest->cost != MAX_DOUBLE && costInterCU != COST_UNKNOWN && mode >= 0)
      {
        if (m_pcEncCfg->m_fastLocalDualTreeMode == 2)
        {
          //Note: only try one intra mode, which is especially useful to reduce EncT for LDB case (around 4%)
#if ISP_VVC
          EndMode = 0;
#endif
          break;
        }
        else
        {
          if (csBest->cost > costInterCU * 1.5)
          {
#if ISP_VVC
            EndMode = 0;
#endif
            break;
          }
        }
      }
#if ISP_VVC
    }
#endif
  } // Mode loop

#if ISP_VVC
  cu.ispMode = bestISP;
#endif
  if (validReturn)
  {
    cs.useSubStructure(*csBest, partitioner.chType, TREE_D, cu.singleChan(CH_L), true);
    const ReshapeData& reshapeData = cs.picture->reshapeData;
    if (cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag())
    {
      cs.getRspRecoBuf().copyFrom(csBest->getRspRecoBuf());
    }

    //=== update PU data ====
    cu.lfnstIdx = bestLfnstIdx;
#if !ISP_VVC
    cu.ispMode = uiBestPUMode.ispMod;
#endif
    cu.mipFlag = uiBestPUMode.mipFlg;
    cu.mipTransposedFlag = uiBestPUMode.mipTrFlg;
    cu.multiRefIdx = uiBestPUMode.mRefId;
    cu.intraDir[CH_L] = uiBestPUMode.modeId;
#if ISP_VVC
    cu.mipFlag = mip;
    cu.multiRefIdx = mrl;
#endif
#if BDPCM_VVC
    cu.bdpcmMode = bestBDPCMMode;
#endif
  }
  else
  {
    THROW("fix this");
  }

  csBest->releaseIntermediateData();

  return validReturn;
}

#if ISP_VVC
void IntraSearch::estIntraPredChromaQT( CodingUnit& cu, Partitioner& partitioner, const double maxCostAllowed )
#else
void IntraSearch::estIntraPredChromaQT( CodingUnit &cu, Partitioner &partitioner )
#endif
{
  PROFILER_SCOPE_AND_STAGE_EXT( 0, g_timeProfiler, P_INTRA_CHROMA, cu.cs, CH_C );
  const ChromaFormat format   = cu.chromaFormat;
  const uint32_t    numberValidComponents = getNumberValidComponents(format);
  CodingStructure &cs = *cu.cs;
  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
#if ISP_VVC
  bool      lumaUsesISP = !CU::isSepTree(cu) && cu.ispMode;
  PartSplit ispType = lumaUsesISP ? CU::getISPType(cu, COMP_Y) : TU_NO_ISP;
#if ISP_VVC
  double bestCostSoFar = maxCostAllowed;
#endif
#endif

  uint32_t   uiBestMode = 0;
  Distortion uiBestDist = 0;
  double     dBestCost  = MAX_DOUBLE;

  //----- init mode list ----
  {
    uint32_t  uiMinMode = 0;
    uint32_t  uiMaxMode = NUM_CHROMA_MODE;

    //----- check chroma modes -----
    uint32_t chromaCandModes[ NUM_CHROMA_MODE ];
    CU::getIntraChromaCandModes( cu, chromaCandModes );

    // create a temporary CS
    CodingStructure &saveCS = *m_pSaveCS[0];
    saveCS.pcv      = cs.pcv;
    saveCS.picture  = cs.picture;
    saveCS.area.repositionTo( cs.area );
    saveCS.clearTUs();

      if( !CU::isSepTree(cu) && cu.ispMode )
      {
        saveCS.clearCUs();
      }

    if( CU::isSepTree(cu) )
    {
      if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
      {
        partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );

        do
        {
          cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType, partitioner.treeType ), partitioner.chType, &cu ).depth = partitioner.currTrDepth;
        } while( partitioner.nextPart( cs ) );

        partitioner.exitCurrSplit();
      }
      else
        cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType, partitioner.treeType ), partitioner.chType, &cu );
    }

    std::vector<TransformUnit*> orgTUs;
#if ISP_VVC
    if (lumaUsesISP)
    {
      CodingUnit& auxCU = saveCS.addCU(cu, partitioner.chType);
      auxCU.ispMode = cu.ispMode;
      saveCS.sps = cu.cs->sps;
    }
#endif

    // create a store for the TUs
    for( const auto &ptu : cs.tus )
    {
      // for split TUs in HEVC, add the TUs without Chroma parts for correct setting of Cbfs
#if ISP_VVC
      if (lumaUsesISP || cu.contains(*ptu, CH_C))
#else
      if( /*lumaUsesISP ||*/ cu.contains( *ptu, CH_C ) )
#endif
      {
        saveCS.addTU( *ptu, partitioner.chType, nullptr );
        orgTUs.push_back( ptu );
      }
    }
#if ISP_VVC
    if (lumaUsesISP)
    {
      saveCS.clearCUs();
    }
#endif

    // SATD pre-selecting.
    int     satdModeList  [NUM_CHROMA_MODE] = { 0 };
    int64_t satdSortedCost[NUM_CHROMA_MODE] = { 0 };
    bool    modeDisable[NUM_INTRA_MODE + 1] = { false }; // use intra mode idx to check whether enable

    CodingStructure& cs = *(cu.cs);
    CompArea areaCb = cu.Cb();
    CompArea areaCr = cu.Cr();
    CPelBuf orgCb  = cs.getOrgBuf (COMP_Cb);
    PelBuf predCb  = cs.getPredBuf(COMP_Cb);
    CPelBuf orgCr  = cs.getOrgBuf (COMP_Cr);
    PelBuf predCr  = cs.getPredBuf(COMP_Cr);

    DistParam distParamSadCb  = m_pcRdCost->setDistParam( orgCb, predCb, cu.cs->sps->bitDepths[ CH_C ], DF_SAD);
    DistParam distParamSatdCb = m_pcRdCost->setDistParam( orgCb, predCb, cu.cs->sps->bitDepths[ CH_C ], DF_HAD);
    DistParam distParamSadCr  = m_pcRdCost->setDistParam( orgCr, predCr, cu.cs->sps->bitDepths[ CH_C ], DF_SAD);
    DistParam distParamSatdCr = m_pcRdCost->setDistParam( orgCr, predCr, cu.cs->sps->bitDepths[ CH_C ], DF_HAD);

    cu.intraDir[1] = MDLM_L_IDX; // temporary assigned, just to indicate this is a MDLM mode. for luma down-sampling operation.

    initIntraPatternChType(cu, cu.Cb());
    initIntraPatternChType(cu, cu.Cr());
    loadLMLumaRecPels(cu, cu.Cb());

    for (int idx = uiMinMode; idx < uiMaxMode; idx++)
    {
      int mode = chromaCandModes[idx];
      satdModeList[idx] = mode;
      if (CU::isLMCMode(mode) && !CU::isLMCModeEnabled(cu, mode))
      {
        continue;
      }
      if ((mode == LM_CHROMA_IDX) || (mode == PLANAR_IDX) || (mode == DM_CHROMA_IDX)) // only pre-check regular modes and MDLM modes, not including DM ,Planar, and LM
      {
        continue;
      }

      cu.intraDir[1]    = mode; // temporary assigned, for SATD checking.

      const bool isLMCMode = CU::isLMCMode(mode);
      if( isLMCMode )
      {
        predIntraChromaLM(COMP_Cb, predCb, cu, areaCb, mode);
      }
      else
      {
        initPredIntraParams(cu, cu.Cb(), *cs.sps);
        predIntraAng(COMP_Cb, predCb, cu);
      }
      int64_t sadCb = distParamSadCb.distFunc(distParamSadCb) * 2;
      int64_t satdCb = distParamSatdCb.distFunc(distParamSatdCb);
      int64_t sad = std::min(sadCb, satdCb);

      if( isLMCMode )
      {
        predIntraChromaLM(COMP_Cr, predCr, cu, areaCr, mode);
      }
      else
      {
        initPredIntraParams(cu, cu.Cr(), *cs.sps);
        predIntraAng(COMP_Cr, predCr, cu);
      }
      int64_t sadCr = distParamSadCr.distFunc(distParamSadCr) * 2;
      int64_t satdCr = distParamSatdCr.distFunc(distParamSatdCr);
      sad += std::min(sadCr, satdCr);
      satdSortedCost[idx] = sad;
    }

    // sort the mode based on the cost from small to large.
    for (int i = uiMinMode; i <= uiMaxMode - 1; i++)
    {
      for (int j = i + 1; j <= uiMaxMode - 1; j++)
      {
        if (satdSortedCost[j] < satdSortedCost[i])
        {
          std::swap( satdModeList[i],   satdModeList[j]);
          std::swap( satdSortedCost[i], satdSortedCost[j]);
        }
      }
    }

    int reducedModeNumber = 2; // reduce the number of chroma modes
    for (int i = 0; i < reducedModeNumber; i++)
    {
      modeDisable[satdModeList[uiMaxMode - 1 - i]] = true; // disable the last reducedModeNumber modes
    }

    int bestLfnstIdx = 0;
    // save the dist
    Distortion baseDist = cs.dist;
#if BDPCM_VVC
    int32_t bestBDPCMMode = 0;
    bool testBDPCM = CU::bdpcmAllowed(cu, COMP_Cb);
#if DETECT_SC
    testBDPCM &= cs.picture->useSC;
#endif
    if (partitioner.chType != CH_C)
    {
      testBDPCM = testBDPCM && cu.ispMode == 0 /*&& cu.firstTU->mtsIdx[COMP_Y] == 0*/ && cu.lfnstIdx == 0;///???
#if BDPCM_VVC==2
      if (cu.firstTU->mtsIdx[COMP_Y] != MTS_SKIP)
      {
        testBDPCM = false;
      }
#endif
    }
    for (int mode_cur = uiMinMode; mode_cur < ((int)uiMaxMode + (2 * int(testBDPCM))); mode_cur++)
#else
    for (uint32_t uiMode = uiMinMode; uiMode < uiMaxMode; uiMode++)
#endif
    {
#if BDPCM_VVC
      int uiMode = mode_cur;
      if (mode_cur >= (int)uiMaxMode)
      {
        uiMode = mode_cur - (int)uiMaxMode ? -1 : -2;
#if BDPCM_VVC == 2
        if ((uiMode == -1) && (saveCS.tus[0]->mtsIdx[COMP_Cb] != MTS_SKIP) && (saveCS.tus[0]->mtsIdx[COMP_Cr] != MTS_SKIP))
        {
          continue;
        }
#endif
      }
      int chromaIntraMode;
      if (uiMode < 0)
      {
        cu.bdpcmModeChroma = -uiMode;
        chromaIntraMode = cu.bdpcmModeChroma == 2 ? chromaCandModes[1] : chromaCandModes[2];
      }
      else
      {
        cu.bdpcmModeChroma = 0;
        chromaIntraMode = chromaCandModes[uiMode];
#else
      const int chromaIntraMode = chromaCandModes[uiMode];
#endif
      if (CU::isLMCMode(chromaIntraMode) && !CU::isLMCModeEnabled(cu, chromaIntraMode))
      {
        continue;
      }
      if (modeDisable[chromaIntraMode] && CU::isLMCModeEnabled(cu, chromaIntraMode)) // when CCLM is disable, then MDLM is disable. not use satd checking
      {
        continue;
      }
#if BDPCM_VVC
      }
#endif
      cs.dist = baseDist;
      //----- restore context models -----
      m_CABACEstimator->getCtx() = ctxStart;

      //----- chroma coding -----
      cu.intraDir[1] = chromaIntraMode;
#if ISP_VVC 
      m_ispTestedModes[0].IspType = ispType;
      m_ispTestedModes[0].subTuCounter = -1;
#endif
      xIntraChromaCodingQT( cs, partitioner );
#if ISP_VVC
      if (lumaUsesISP && cs.dist == MAX_UINT)
      {
        continue;
      }
#endif

      if (cs.sps->transformSkip)
      {
        m_CABACEstimator->getCtx() = ctxStart;
      }
#if ISP_VVC
      m_ispTestedModes[0].IspType = ispType;
      m_ispTestedModes[0].subTuCounter = -1;
#endif
      uint64_t fracBits   = xGetIntraFracBitsQT( cs, partitioner, false );
      Distortion uiDist = cs.dist;
      double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

      //----- compare -----
      if( dCost < dBestCost )
      {
#if ISP_VVC
        if (lumaUsesISP && (dCost < bestCostSoFar))
        {
          bestCostSoFar = dCost;
        }
#endif
        for( uint32_t i = getFirstComponentOfChannel( CH_C ); i < numberValidComponents; i++ )
        {
          const CompArea& area = cu.blocks[i];
          saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
          cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf   ( area ) );
          for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
          {
            saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
          }
        }
        dBestCost    = dCost;
        uiBestDist   = uiDist;
        uiBestMode   = chromaIntraMode;
        bestLfnstIdx = cu.lfnstIdx;
#if BDPCM_VVC
        bestBDPCMMode = cu.bdpcmModeChroma;
#endif

      }
    }
    cu.lfnstIdx = bestLfnstIdx;
#if BDPCM_VVC
    cu.bdpcmModeChroma= bestBDPCMMode;
#endif

    for( uint32_t i = getFirstComponentOfChannel( CH_C ); i < numberValidComponents; i++ )
    {
      const CompArea& area = cu.blocks[i];

      cs.getRecoBuf         ( area ).copyFrom( saveCS.getRecoBuf( area ) );
      cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf    ( area ) );

      for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
      {
        orgTUs[ j ]->copyComponentFrom( *saveCS.tus[ j ], area.compID );
      }
    }
  }
  cu.intraDir[1] = uiBestMode;
  cs.dist        = uiBestDist;

  //----- restore context models -----
  m_CABACEstimator->getCtx() = ctxStart;
#if ISP_VVC
  if (lumaUsesISP && bestCostSoFar >= maxCostAllowed)
  {
    cu.ispMode = 0;
  }
#endif
}

void IntraSearch::saveCuAreaCostInSCIPU( Area area, double cost )
{
  if( m_numCuInSCIPU < NUM_INTER_CU_INFO_SAVE )
  {
    m_cuAreaInSCIPU[m_numCuInSCIPU] = area;
    m_cuCostInSCIPU[m_numCuInSCIPU] = cost;
    m_numCuInSCIPU++;
  }
}

void IntraSearch::initCuAreaCostInSCIPU()
{
  for( int i = 0; i < NUM_INTER_CU_INFO_SAVE; i++ )
  {
    m_cuAreaInSCIPU[i] = Area();
    m_cuCostInSCIPU[i] = 0;
  }
  m_numCuInSCIPU = 0;
}
// -------------------------------------------------------------------------------------------------------------------
// Intra search
// -------------------------------------------------------------------------------------------------------------------

void IntraSearch::xEncIntraHeader( CodingStructure &cs, Partitioner &partitioner, const bool luma )
{
  CodingUnit &cu = *cs.getCU( partitioner.chType, partitioner.treeType );

  if (luma)
  {
#if ISP_VVC
    bool isFirst = cu.ispMode ? m_ispTestedModes[0].subTuCounter == 0 : partitioner.currArea().lumaPos() == cs.area.lumaPos();
#else
    bool isFirst = partitioner.currArea().lumaPos() == cs.area.lumaPos();
#endif

    // CU header
    if( isFirst )
    {
      if ((!cs.slice->isIntra() || cs.slice->sps->IBC || cs.slice->sps->PLT) && cu.Y().valid())
      {
        m_CABACEstimator->pred_mode   ( cu );
      }
      m_CABACEstimator->bdpcm_mode  ( cu, ComponentID(partitioner.chType) );
    }

    // luma prediction mode
    if (isFirst)
    {
      if ( !cu.Y().valid())
      {
        m_CABACEstimator->pred_mode( cu );
      }
      m_CABACEstimator->intra_luma_pred_mode( cu );
    }
  }
  else //  if (chroma)
  {
    bool isFirst = partitioner.currArea().Cb().valid() && partitioner.currArea().chromaPos() == cs.area.chromaPos();

    if( isFirst )
    {
#if BDPCM_VVC
      m_CABACEstimator->bdpcm_mode(cu, ComponentID(CH_C));
#endif
      m_CABACEstimator->intra_chroma_pred_mode(  cu );
    }
  }
}

void IntraSearch::xEncSubdivCbfQT( CodingStructure &cs, Partitioner &partitioner, const bool luma )
{
  const UnitArea& currArea = partitioner.currArea();
#if ISP_VVC
  int subTuCounter = m_ispTestedModes[0].subTuCounter;
  TransformUnit  &currTU   = *cs.getTU(currArea.blocks[partitioner.chType], partitioner.chType, subTuCounter);
#else
  TransformUnit  &currTU   = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );
#endif
  CodingUnit     &currCU   = *currTU.cu;
  const uint32_t currDepth = partitioner.currTrDepth;
#if ISP_VVC
  const bool  subdiv = currTU.depth > currDepth;
  ComponentID compID = partitioner.chType == CH_L ? COMP_Y : COMP_Cb;

  if (!luma)
  {
    const bool chromaCbfISP = currArea.blocks[COMP_Cb].valid() && currCU.ispMode && !subdiv;
    if (!currCU.ispMode || chromaCbfISP)
    {
      const uint32_t numberValidComponents = getNumberValidComponents(currArea.chromaFormat);
      const uint32_t cbfDepth = (chromaCbfISP ? currDepth - 1 : currDepth);

      for (uint32_t ch = COMP_Cb; ch < numberValidComponents; ch++)
      {
        const ComponentID compID = ComponentID(ch);
        if (currDepth == 0 || TU::getCbfAtDepth(currTU, compID, currDepth - 1) || chromaCbfISP)
        {
          const bool prevCbf = (compID == COMP_Cr ? TU::getCbfAtDepth(currTU, COMP_Cb, currDepth) : false);
          m_CABACEstimator->cbf_comp(currCU, TU::getCbfAtDepth(currTU, compID, currDepth), currArea.blocks[compID], cbfDepth, prevCbf);
        }
      }
    }
  }

  if (subdiv)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else if (currCU.ispMode && isLuma(compID))
    {
      partitioner.splitCurrArea(m_ispTestedModes[0].IspType, cs);
    }
    else
      THROW("Cannot perform an implicit split!");

    do
    {
      xEncSubdivCbfQT(cs, partitioner, luma);   //?
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while (partitioner.nextPart(cs));

    partitioner.exitCurrSplit();
  }
  else
#endif
  {
    //===== Cbfs =====
    if (luma)
    {
      bool previousCbf = false;
      bool lastCbfIsInferred = false;
#if ISP_VVC
      if (m_ispTestedModes[0].IspType != TU_NO_ISP)
      {
        bool     rootCbfSoFar = false;
        uint32_t nTus = currCU.ispMode == HOR_INTRA_SUBPARTITIONS ? currCU.lheight() >> floorLog2(currTU.lheight())
          : currCU.lwidth() >> floorLog2(currTU.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = currCU.firstTU;
          for (int tuIdx = 0; tuIdx < nTus - 1; tuIdx++)
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMP_Y, currDepth);
            tuPointer = tuPointer->next;
          }
          if (!rootCbfSoFar)
          {
            lastCbfIsInferred = true;
          }
        }
        if (!lastCbfIsInferred)
        {
          previousCbf = TU::getPrevTuCbfAtDepth(currTU, COMP_Y, partitioner.currTrDepth);
        }
      }
#endif
      if (!lastCbfIsInferred)
      {
        m_CABACEstimator->cbf_comp(currCU, TU::getCbfAtDepth(currTU, COMP_Y, currDepth), currTU.Y(), currTU.depth, previousCbf, currCU.ispMode);
      }
    }
#if !ISP_VVC
    else  //if( chroma )
    {
      const uint32_t numberValidComponents = getNumberValidComponents(currArea.chromaFormat);
      const uint32_t cbfDepth = currDepth;

      for (uint32_t ch = COMP_Cb; ch < numberValidComponents; ch++)
      {
        const ComponentID compID = ComponentID(ch);

        if (currDepth == 0 || TU::getCbfAtDepth(currTU, compID, currDepth - 1))
        {
          const bool prevCbf = (compID == COMP_Cr ? TU::getCbfAtDepth(currTU, COMP_Cb, currDepth) : false);
          m_CABACEstimator->cbf_comp(currCU, TU::getCbfAtDepth(currTU, compID, currDepth), currArea.blocks[compID], cbfDepth, prevCbf);
        }
      }
    }
#endif
  }
}
#if ISP_VVC
void IntraSearch::xEncCoeffQT(CodingStructure& cs, Partitioner& partitioner, const ComponentID compID, CUCtx* cuCtx, const int subTuIdx, const PartSplit ispType)
#else
void IntraSearch::xEncCoeffQT( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID, CUCtx *cuCtx )
#endif
{
  const UnitArea& currArea  = partitioner.currArea();

#if ISP_VVC
  int subTuCounter          = m_ispTestedModes[0].subTuCounter;
  TransformUnit& currTU     = *cs.getTU(currArea.blocks[partitioner.chType], partitioner.chType, subTuCounter);
#else
  TransformUnit& currTU     = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );
#endif
  uint32_t   currDepth      = partitioner.currTrDepth;
  const bool subdiv         = currTU.depth > currDepth;

  if (subdiv)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
#if ISP_VVC
    else if (currTU.cu->ispMode)
    {
      partitioner.splitCurrArea(m_ispTestedModes[0].IspType, cs);
    }
#endif
    else
      THROW("Implicit TU split not available!");

    do
    {
#if ISP_VVC
      xEncCoeffQT(cs, partitioner, compID, cuCtx, subTuCounter, m_ispTestedModes[0].IspType);
      subTuCounter += subTuCounter != -1 ? 1 : 0;
#else
      xEncCoeffQT( cs, partitioner, compID );
#endif
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else

  if( currArea.blocks[compID].valid() )
  {
    if( compID == COMP_Cr )
    {
      const int cbfMask = ( TU::getCbf( currTU, COMP_Cb ) ? 2 : 0 ) + ( TU::getCbf( currTU, COMP_Cr ) ? 1 : 0 );
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    }
    if( TU::getCbf( currTU, compID ) )
    {
      if( isLuma(compID) )
      {
        m_CABACEstimator->residual_coding( currTU, compID, cuCtx );
        m_CABACEstimator->mts_idx( *currTU.cu, cuCtx );
      }
      else
        m_CABACEstimator->residual_coding( currTU, compID );
    }
  }
}

uint64_t IntraSearch::xGetIntraFracBitsQT( CodingStructure &cs, Partitioner &partitioner, const bool luma, CUCtx *cuCtx )
{
  m_CABACEstimator->resetBits();

  xEncIntraHeader( cs, partitioner, luma );
  xEncSubdivCbfQT( cs, partitioner, luma );

  if( luma )
  {
    xEncCoeffQT( cs, partitioner, COMP_Y, cuCtx );

    CodingUnit &cu = *cs.cus[0];
#if ISP_VVC
    if (cuCtx /*&& CU::isSepTree(cu)*/
      && (!cu.ispMode || (cu.lfnstIdx && m_ispTestedModes[0].subTuCounter == 0)
        || (!cu.lfnstIdx
          && m_ispTestedModes[0].subTuCounter == m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1] - 1)))
#else
    if( cuCtx )
#endif
    {
      m_CABACEstimator->residual_lfnst_mode( cu, *cuCtx );
    }
  }
  else
  {
    xEncCoeffQT( cs, partitioner, COMP_Cb );
    xEncCoeffQT( cs, partitioner, COMP_Cr );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

uint64_t IntraSearch::xGetIntraFracBitsQTChroma(const TransformUnit& currTU, const ComponentID compID, CUCtx *cuCtx)
{
  m_CABACEstimator->resetBits();

  if ( currTU.jointCbCr )
  {
    const int cbfMask = ( TU::getCbf( currTU, COMP_Cb ) ? 2 : 0 ) + ( TU::getCbf( currTU, COMP_Cr ) ? 1 : 0 );
    m_CABACEstimator->cbf_comp( *currTU.cu, cbfMask>>1, currTU.blocks[ COMP_Cb ], currTU.depth, false );
    m_CABACEstimator->cbf_comp( *currTU.cu, cbfMask &1, currTU.blocks[ COMP_Cr ], currTU.depth, cbfMask>>1 );
    if( cbfMask )
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    if (cbfMask >> 1)
      m_CABACEstimator->residual_coding( currTU, COMP_Cb, cuCtx );
    if (cbfMask & 1)
      m_CABACEstimator->residual_coding( currTU, COMP_Cr, cuCtx );
  }
  else
  {
    if ( compID == COMP_Cb )
      m_CABACEstimator->cbf_comp( *currTU.cu, TU::getCbf( currTU, compID ), currTU.blocks[ compID ], currTU.depth, false );
    else
    {
      const bool cbCbf    = TU::getCbf( currTU, COMP_Cb );
      const bool crCbf    = TU::getCbf( currTU, compID );
      const int  cbfMask  = ( cbCbf ? 2 : 0 ) + ( crCbf ? 1 : 0 );
      m_CABACEstimator->cbf_comp( *currTU.cu, crCbf, currTU.blocks[ compID ], currTU.depth, cbCbf );
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    }
  }

  if( !currTU.jointCbCr && TU::getCbf( currTU, compID ) )
  {
    m_CABACEstimator->residual_coding( currTU, compID, cuCtx );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

void IntraSearch::xIntraCodingTUBlock(TransformUnit &tu, const ComponentID compID, const bool checkCrossCPrediction, Distortion &ruiDist, uint32_t *numSig, PelUnitBuf *predBuf, const bool loadTr)
{
  if (!tu.blocks[compID].valid())
  {
    return;
  }

  CodingStructure &cs             = *tu.cs;
  const CompArea      &area       = tu.blocks[compID];
  const SPS           &sps        = *cs.sps;
  const ReshapeData&  reshapeData = cs.picture->reshapeData;

  const ChannelType    chType     = toChannelType(compID);
  const int            bitDepth   = sps.bitDepths[chType];

#if ISP_VVC   // area
  CPelBuf        piOrg            = cs.getOrgBuf    (area);
  PelBuf         piPred           = cs.getPredBuf   (area);
  PelBuf         piResi           = cs.getResiBuf   (area);
  PelBuf         piReco           = cs.getRecoBuf   (area);
#else
  CPelBuf        piOrg            = cs.getOrgBuf    (compID);
  PelBuf         piPred           = cs.getPredBuf   (compID);
  PelBuf         piResi           = cs.getResiBuf   (compID);
  PelBuf         piReco           = cs.getRecoBuf   (compID);
#endif

  const CodingUnit& cu            = *tu.cu;

  //===== init availability pattern =====
  CHECK( tu.jointCbCr && compID == COMP_Cr, "wrong combination of compID and jointCbCr" );
  bool jointCbCr = tu.jointCbCr && compID == COMP_Cb;

  if ( isLuma(compID) )
  {
    bool predRegDiffFromTB = CU::isPredRegDiffFromTB(*tu.cu );
    bool firstTBInPredReg  = false;
    CompArea areaPredReg(COMP_Y, tu.chromaFormat, area);
#if ISP_VVC
    if (tu.cu->ispMode )
    {
      firstTBInPredReg = CU::isFirstTBInPredReg(*tu.cu, area);
      if (predRegDiffFromTB)
      {
        if (firstTBInPredReg)
        {
          CU::adjustPredArea(areaPredReg);
          initIntraPatternChTypeISP(*tu.cu, areaPredReg, piReco);
        }
      }
      else
        initIntraPatternChTypeISP(*tu.cu, area, piReco);
    }
    else
#endif
    {
      initIntraPatternChType(*tu.cu, area);
    }

    //===== get prediction signal =====
    if (predRegDiffFromTB)
    {
      if (firstTBInPredReg)
      {
        PelBuf piPredReg = cs.getPredBuf(areaPredReg);
        predIntraAng(compID, piPredReg, cu);
      }
    }
    else
    {
      if( predBuf )
      {
        piPred.copyFrom( predBuf->Y() );
      }
      else if( CU::isMIP( cu, CH_L ) )
      {
        initIntraMip( cu );
        predIntraMip( piPred, cu );
      }
      else
      {
        predIntraAng(compID, piPred, cu);
      }
    }
  }
  DTRACE( g_trace_ctx, D_PRED, "@(%4d,%4d) [%2dx%2d] IMode=%d\n", tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), CU::getFinalIntraMode(cu, chType) );
  const Slice &slice = *cs.slice;
  bool flag = cs.picHeader->lmcsEnabled && (slice.isIntra() || (!slice.isIntra() && reshapeData.getCTUFlag()));

  if (isLuma(compID))
  {
    //===== get residual signal =====
    if (cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag() )
    {
#if ISP_VVC
      piResi.subtract(cs.getRspOrgBuf(area), piPred);
#else
      piResi.subtract( cs.getRspOrgBuf(), piPred);
#endif
    }
    else
    {
      piResi.subtract( piOrg, piPred );
    }
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  //--- transform and quantization           ---
  TCoeff uiAbsSum = 0;
  const QpParam cQP(tu, compID);

  m_pcTrQuant->selectLambda(compID);

  flag =flag && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
  if (flag && isChroma(compID) && cs.picHeader->lmcsChromaResidualScale )
  {
    int cResScaleInv = tu.chromaAdj;
    double cRescale = (double)(1 << CSCALE_FP_PREC) / (double)cResScaleInv;
    m_pcTrQuant->scaleLambda( 1.0/(cRescale*cRescale) );
  }

  CPelBuf         crOrg  = cs.getOrgBuf  ( COMP_Cr );
  PelBuf          crPred = cs.getPredBuf ( COMP_Cr );
  PelBuf          crResi = cs.getResiBuf ( COMP_Cr );
  PelBuf          crReco = cs.getRecoBuf ( COMP_Cr );

  if ( jointCbCr )
  {
    // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
    const int    absIct = abs( TU::getICTMode(tu) );
    const double lfact  = ( absIct == 1 || absIct == 3 ? 0.8 : 0.5 );
    m_pcTrQuant->scaleLambda( lfact );
  }
  if ( sps.jointCbCr && isChroma(compID) && (tu.cu->cs->slice->sliceQp > 18) )
  {
    m_pcTrQuant->scaleLambda( 1.3 );
  }

  if( isLuma(compID) )
  {
    m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);

    DTRACE( g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_TU_ABS_SUM ), compID, uiAbsSum );
#if ISP_VVC
    if (tu.cu->ispMode && isLuma(compID) && CU::isISPLast(*tu.cu, area, area.compID) && CU::allLumaCBFsAreZero(*tu.cu))
    {
      // ISP has to have at least one non-zero CBF
      ruiDist = MAX_INT;
      return;
    }
#endif
    //--- inverse transform ---
    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
    }
    else
    {
      piResi.fill(0);
    }
  }
  else // chroma
  {
    int         codedCbfMask  = 0;
    ComponentID codeCompId    = (tu.jointCbCr ? (tu.jointCbCr >> 1 ? COMP_Cb : COMP_Cr) : compID);
    const QpParam qpCbCr(tu, codeCompId);

    if( tu.jointCbCr )
    {
      ComponentID otherCompId = ( codeCompId==COMP_Cr ? COMP_Cb : COMP_Cr );
      tu.getCoeffs( otherCompId ).fill(0); // do we need that?
      TU::setCbfAtDepth (tu, otherCompId, tu.depth, false );
    }
    PelBuf& codeResi = ( codeCompId == COMP_Cr ? crResi : piResi );
    uiAbsSum = 0;
    m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    DTRACE( g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_TU_ABS_SUM ), codeCompId, uiAbsSum );
    if( uiAbsSum > 0 )
    {
      m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
      codedCbfMask += ( codeCompId == COMP_Cb ? 2 : 1 );
    }
    else
    {
      codeResi.fill(0);
    }

    if( tu.jointCbCr )
    {
      if( tu.jointCbCr == 3 && codedCbfMask == 2 )
      {
        codedCbfMask = 3;
        TU::setCbfAtDepth (tu, COMP_Cr, tu.depth, true );
      }
      if( tu.jointCbCr != codedCbfMask )
      {
        ruiDist = MAX_DISTORTION;
        return;
      }
      m_pcTrQuant->invTransformICT( tu, piResi, crResi );
      uiAbsSum = codedCbfMask;
    }
  }

  //===== reconstruction =====
  if ( flag && uiAbsSum > 0 && isChroma(compID) && cs.picHeader->lmcsChromaResidualScale )
  {
    piResi.scaleSignal(tu.chromaAdj, 0, slice.clpRngs[compID]);

    if( jointCbCr )
    {
      crResi.scaleSignal(tu.chromaAdj, 0, slice.clpRngs[COMP_Cr]);
    }
  }

  piReco.reconstruct(piPred, piResi, cs.slice->clpRngs[ compID ]);
  if( jointCbCr )
  {
    crReco.reconstruct(crPred, crResi, cs.slice->clpRngs[ COMP_Cr ]);
  }


  //===== update distortion =====
  if( (cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag()) || m_pcEncCfg->m_lumaLevelToDeltaQPEnabled )
  {
    const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMP_Y] );
    if( compID == COMP_Y && !m_pcEncCfg->m_lumaLevelToDeltaQPEnabled )
    {
#if ISP_VVC
      PelBuf tmpRecLuma = cs.getRspRecoBuf(area);
#else
      PelBuf tmpRecLuma = cs.getRspRecoBuf();
#endif
      tmpRecLuma.rspSignal( piReco, reshapeData.getInvLUT());
      ruiDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.bitDepths[toChannelType(compID)], compID, DF_SSE_WTD, &orgLuma);
    }
    else
    {
      ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE_WTD, &orgLuma );
      if( jointCbCr )
      {
        ruiDist += m_pcRdCost->getDistPart( crOrg, crReco, bitDepth, COMP_Cr, DF_SSE_WTD, &orgLuma );
      }
    }
  }
  else
  {
    ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE );
    if( jointCbCr )
    {
      ruiDist += m_pcRdCost->getDistPart( crOrg, crReco, bitDepth, COMP_Cr, DF_SSE );
    }
  }
}

#if ISP_VVC
void IntraSearch::xIntraCodingLumaQT(CodingStructure& cs, Partitioner& partitioner, PelUnitBuf* predBuf, const double bestCostSoFar, int numMode)
#else
void IntraSearch::xIntraCodingLumaQT( CodingStructure& cs, Partitioner& partitioner, PelUnitBuf* predBuf, const double bestCostSoFar )
#endif
{
  PROFILER_SCOPE_AND_STAGE_EXT( 0, g_timeProfiler, P_INTRA_RD_SEARCH_LUMA, &cs, partitioner.chType );
  const UnitArea& currArea  = partitioner.currArea();
  uint32_t        currDepth = partitioner.currTrDepth;
#if !ISP_VVC
  TransformUnit& tu = cs.addTU( CS::getArea( cs, currArea, partitioner.chType, partitioner.treeType ), partitioner.chType, cs.cus[0] );
  tu.depth = currDepth;

  CHECK( !tu.Y().valid(), "Invalid TU" );
#endif
  Distortion singleDistLuma = 0;
  uint32_t   numSig         = 0;
  const SPS &sps            = *cs.sps;
  CodingUnit &cu            = *cs.cus[0];
  bool mtsAllowed           = CU::isMTSAllowed(cu, COMP_Y);
  uint64_t singleFracBits   = 0;
#if  ISP_VVC               
  bool   splitCbfLumaSum    = false;
  double bestCostForISP     = bestCostSoFar;
  double dSingleCost        = MAX_DOUBLE;
  int endLfnstIdx           = (partitioner.isSepTree(cs) && partitioner.chType == CH_C && (currArea.lwidth() < 8 || currArea.lheight() < 8))
                           || (currArea.lwidth() > sps.getMaxTbSize() || currArea.lheight() > sps.getMaxTbSize()) || !sps.LFNST || (numMode < 0) ? 0 : 2;
  numMode                   = (numMode < 0) ? -numMode : numMode;
#else
  int endLfnstIdx   = (partitioner.isSepTree(cs) && partitioner.chType == CH_C && (currArea.lwidth() < 8 || currArea.lheight() < 8))
                        || (currArea.lwidth() > sps.getMaxTbSize() || currArea.lheight() > sps.getMaxTbSize()) || !sps.LFNST ? 0 : 2;
#endif

  if (cu.mipFlag && !allowLfnstWithMip(cu.lumaSize()))
  {
    endLfnstIdx = 0;
  }
  int bestMTS = 0;
  int EndMTS  = mtsAllowed ? m_pcEncCfg->m_MTSIntraMaxCand +1 : 0;
#if ISP_VVC
  if (cu.ispMode && (EndMTS || endLfnstIdx))
  {
    EndMTS = 0;
    if ((m_ispTestedModes[1].numTotalParts[cu.ispMode - 1] == 0)
     && (m_ispTestedModes[2].numTotalParts[cu.ispMode - 1] == 0))
    {
      endLfnstIdx = 0;
    }
  }
#endif
#if BDPCM_VVC
  if (cu.bdpcmMode)
  {
    endLfnstIdx = 0;
    EndMTS = 0;
  }
#endif
#if TS_VVC
  bool checkTransformSkip = sps.transformSkip;

  SizeType transformSkipMaxSize = 1 << sps.log2MaxTransformSkipBlockSize;
  //bool tsAllowed = TU::isTSAllowed(tu, COMP_Y);
  bool tsAllowed = cu.cs->sps->transformSkip && (!cu.ispMode) && (!cu.bdpcmMode) &&(!cu.sbtInfo);
  tsAllowed &= cu.blocks[COMP_Y].width <= transformSkipMaxSize && cu.blocks[COMP_Y].height <= transformSkipMaxSize;
#if DETECT_SC
  tsAllowed &= cs.picture->useSC;
#endif
#if CHANGE_SIZE
  if (tsAllowed && sps.BDPCM)
  {
    int size = sps.log2MaxTransformSkipBlockSize - DIF_SIZE;
    SizeType transformSkipMaxSize = 1 << size;
    tsAllowed &= cu.blocks[COMP_Y].width <= transformSkipMaxSize && cu.blocks[COMP_Y].height <= transformSkipMaxSize;
  }
#endif
  if (tsAllowed)
  {
    EndMTS += 1;
  }
#endif
  if (endLfnstIdx || EndMTS)
  {
#if ISP_VVC
    bool       splitCbfLuma  = false;
    const PartSplit ispType  = CU::getISPType(cu, COMP_Y);
    CUCtx cuCtx;
    cuCtx.isDQPCoded         = true;
    cuCtx.isChromaQpAdjCoded = true;
    cs.cost                  = 0.0;
#else
    double dSingleCost       = MAX_DOUBLE;
#endif
    Distortion       singleDistTmpLuma = 0;
    uint64_t         singleTmpFracBits = 0;
    double           singleCostTmp     = 0;
    const TempCtx    ctxStart          (m_CtxCache, m_CABACEstimator->getCtx());
          TempCtx    ctxBest           (m_CtxCache);
    CodingStructure &saveCS            = *m_pSaveCS[0];
    TransformUnit *  tmpTU             = nullptr;
    int              bestLfnstIdx      = 0;
    int              startLfnstIdx     = 0;
    // speedUps LFNST
    bool   rapidLFNST                  = false;
    bool   rapidDCT                    = false;
    double thresholdDCT                = 1;

    if (m_pcEncCfg->m_MTS == 2)
    {
      thresholdDCT += 1.4 / sqrt(cu.lwidth() * cu.lheight());
    }

    if (m_pcEncCfg->m_LFNST > 1)
    {
      rapidLFNST = true;

      if (m_pcEncCfg->m_LFNST > 2)
      {
        rapidDCT    = true;
        endLfnstIdx = endLfnstIdx ? 1 : 0;
      }
    }

    saveCS.pcv              = cs.pcv;
    saveCS.picture          = cs.picture;
    saveCS.area.repositionTo( cs.area);
    saveCS.clearTUs();

#if ISP_VVC
    if (cu.ispMode)
    {
      partitioner.splitCurrArea(ispType, cs);
    }

    TransformUnit& tu = cs.addTU(CS::getArea(cs, partitioner.currArea(), partitioner.chType, partitioner.treeType), partitioner.chType, cs.cus[0]);

    if (cu.ispMode)
    {
      do
      {
        saveCS.addTU(
          CS::getArea(cs, partitioner.currArea(), partitioner.chType, partitioner.treeType),
          partitioner.chType, cs.cus[0]);
      } while (partitioner.nextPart(cs));

      partitioner.exitCurrSplit();
    }
    else
    {
      tmpTU = &saveCS.addTU(currArea, partitioner.chType, cs.cus[0]);
    }
#else
    tmpTU = &saveCS.addTU( currArea, partitioner.chType, cs.cus[0] );
#endif


    std::vector<TrMode> trModes{ TrMode(0, true) };
#if TS_VVC
    if (tsAllowed)
    {
      trModes.push_back(TrMode(1, true));
    }
#endif
    double dct2Cost           = MAX_DOUBLE;
    double trGrpStopThreshold = 1.001;
    double trGrpBestCost      = MAX_DOUBLE;

    if (mtsAllowed)
    {
      if (m_pcEncCfg->m_LFNST)
      {
        uint32_t uiIntraMode = cs.cus[0]->intraDir[partitioner.chType];
        int MTScur           = (uiIntraMode < 34) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;

        trModes.push_back(TrMode(     2, true));
        trModes.push_back(TrMode(MTScur, true));

        MTScur = (uiIntraMode < 34) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;

        trModes.push_back(TrMode(MTScur,            true));
        trModes.push_back(TrMode(MTS_DST7_DST7 + 3, true));
      }
      else
      {
        for (int i = 2; i < 6; i++)
        {
          trModes.push_back(TrMode(i, true));
        }
      }
    }

#if TS_VVC
    if ((EndMTS && !m_pcEncCfg->m_LFNST) || (tsAllowed && !mtsAllowed))
#else
    if (EndMTS && !m_pcEncCfg->m_LFNST)
#endif
    {
      xPreCheckMTS(tu, &trModes, m_pcEncCfg->m_MTSIntraMaxCand, predBuf);
#if TS_VVC
      if (!mtsAllowed && !trModes[1].second)
      {
        EndMTS = 0;
      }
#endif
    }

    bool NStopMTS = true;

    for (int modeId = 0; modeId <= EndMTS && NStopMTS; modeId++)
    {
      if (modeId > 1)
      {
        trGrpBestCost = MAX_DOUBLE;
      }
      for (int lfnstIdx = startLfnstIdx; lfnstIdx <= endLfnstIdx; lfnstIdx++)
      {
        if (lfnstIdx && modeId)
        {
          continue;
        }
#if TS_VVC
        if (mtsAllowed || tsAllowed)
#else
        if (mtsAllowed)
#endif
        {
#if TS_VVC
          if (m_pcEncCfg->m_TS && bestMTS == MTS_SKIP)
          {
            break;
          }
          if (!m_pcEncCfg->m_LFNST && !trModes[modeId].second && mtsAllowed)
#else
          if (!m_pcEncCfg->m_LFNST  && !trModes[modeId].second)
#endif
          {
            continue;
          }

          tu.mtsIdx[COMP_Y] = trModes[modeId].first;
        }

#if ISP_VVC
        if (cu.ispMode && lfnstIdx)
        {
          if (m_ispTestedModes[lfnstIdx].numTotalParts[cu.ispMode - 1] == 0)
          {
            if (lfnstIdx == 2)
            {
              endLfnstIdx = 1;
            }
            continue;
          }
        }

#endif
        cu.lfnstIdx                          = lfnstIdx;
        cuCtx.lfnstLastScanPos               = false;
        cuCtx.violatesLfnstConstrained[CH_L] = false;
        cuCtx.violatesLfnstConstrained[CH_C] = false;

        if ((lfnstIdx != startLfnstIdx) || (modeId))
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }

        singleDistTmpLuma = 0;

#if ISP_VVC
        if (cu.ispMode)
        {
          splitCbfLuma = false;

          partitioner.splitCurrArea(ispType, cs);

          singleCostTmp = xTestISP(cs, partitioner, bestCostForISP, ispType, splitCbfLuma, singleTmpFracBits, singleDistTmpLuma, cuCtx);

          partitioner.exitCurrSplit();

          if (modeId && (singleCostTmp == MAX_DOUBLE))
          {
            m_ispTestedModes[lfnstIdx].numTotalParts[cu.ispMode - 1] = 0;
          }

          bool storeCost = (numMode == 1) ? true : false;

          if ((m_pcEncCfg->m_ISP >= 2) && (numMode <= 1))
          {
            storeCost = true;
          }

          if (storeCost)
          {
            m_ispTestedModes[0].bestCost[cu.ispMode - 1] = singleCostTmp;
          }
        }
        else
#endif
        {
#if TS_VVC
          bool TrLoad = (EndMTS && !m_pcEncCfg->m_LFNST) || (tsAllowed && !mtsAllowed && (lfnstIdx == 0)) ? true : false;
#else
          bool TrLoad = EndMTS && !m_pcEncCfg->m_LFNST;
#endif

          xIntraCodingTUBlock(tu, COMP_Y, false, singleDistTmpLuma, &numSig, predBuf, TrLoad);

          cuCtx.mtsLastScanPos = false;
          //----- determine rate and r-d cost -----
#if TS_VVC
        if ((sps.LFNST ? (modeId == EndMTS && modeId != 0 && checkTransformSkip) : (trModes[modeId].first != 0)) && !TU::getCbfAtDepth(tu, COMP_Y, currDepth))
        {
          singleCostTmp = MAX_DOUBLE;
        }
        else
#endif
        {
#if ISP_VVC
          m_ispTestedModes[0].IspType      = TU_NO_ISP;
          m_ispTestedModes[0].subTuCounter = -1;
#endif
          singleTmpFracBits = xGetIntraFracBitsQT(cs, partitioner, true, &cuCtx);

          if (tu.mtsIdx[COMP_Y] > MTS_SKIP)
          {
            if (!cuCtx.mtsLastScanPos)
            {
              singleCostTmp = MAX_DOUBLE;
            }
            else
            {
              singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
            }
          }
          else
          {
            singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
          }
        }

          if (((EndMTS && (m_pcEncCfg->m_MTS == 2)) || rapidLFNST) && modeId == 0 && lfnstIdx == 0)
          {
            if (singleCostTmp > bestCostSoFar * thresholdDCT)
            {
              EndMTS = 0;

              if (rapidDCT)
              {
                endLfnstIdx = 0;   // break the loop but do not cpy best
              }
            }
          }

          if (lfnstIdx && !cuCtx.lfnstLastScanPos && !cu.ispMode)
          {
            bool rootCbfL = false;

            for (uint32_t t = 0; t < getNumberValidTBlocks(*cu.cs->pcv); t++)
            {
              rootCbfL |= tu.cbf[t] != 0;
            }

            if (rapidLFNST && !rootCbfL)
            {
              endLfnstIdx = lfnstIdx; // break the loop
            }
            bool cbfAtZeroDepth = CU::isSepTree(cu)
              ? rootCbfL
              : (cs.area.chromaFormat != CHROMA_400 && std::min(cu.firstTU->blocks[1].width, cu.firstTU->blocks[1].height) < 4)
                ? TU::getCbfAtDepth(tu, COMP_Y, currDepth)
                : rootCbfL;

            if (cbfAtZeroDepth)
            {
              singleCostTmp = MAX_DOUBLE;
            }
          }
        }

        if (singleCostTmp < dSingleCost)
        {
          trGrpBestCost  = singleCostTmp;
          dSingleCost    = singleCostTmp;
          singleDistLuma = singleDistTmpLuma;
          singleFracBits = singleTmpFracBits;
          bestLfnstIdx   = lfnstIdx;
          bestMTS        = modeId;

#if ISP_VVC
          if (dSingleCost < bestCostForISP)
          {
            bestCostForISP = dSingleCost;
          }

          splitCbfLumaSum = splitCbfLuma;

          if (lfnstIdx == 0 && modeId == 0 && cu.ispMode == 0)
#else
          if ((lfnstIdx == 0) && (modeId == 0) )
#endif
          {
            dct2Cost = singleCostTmp;

            if (!TU::getCbfAtDepth(tu, COMP_Y, currDepth))
            {
              if (rapidLFNST)
              {
                 endLfnstIdx = 0;   // break the loop but do not cpy best
              }

              EndMTS = 0;
            }
          }

          if (bestLfnstIdx != endLfnstIdx || bestMTS != EndMTS)
          {
#if ISP_VVC
            if (cu.ispMode)
            {
              saveCS.getRecoBuf(currArea.Y()).copyFrom(cs.getRecoBuf(currArea.Y()));

              for (uint32_t j = 0; j < cs.tus.size(); j++)
              {
                saveCS.tus[j]->copyComponentFrom(*cs.tus[j], COMP_Y);
              }
            }
            else
#endif
            {
              saveCS.getPredBuf(tu.Y()).copyFrom(cs.getPredBuf(tu.Y()));
              saveCS.getRecoBuf(tu.Y()).copyFrom(cs.getRecoBuf(tu.Y()));

              tmpTU->copyComponentFrom(tu, COMP_Y);
            }

            ctxBest = m_CABACEstimator->getCtx();
          }
      
        }
        else
        {
          if( rapidLFNST )
          {
            endLfnstIdx = lfnstIdx; // break the loop
          }
        }
      }
      if (m_pcEncCfg->m_LFNST && m_pcEncCfg->m_MTS == 2 && modeId && modeId != EndMTS)
      {
        NStopMTS = false;

        if (bestMTS || bestLfnstIdx)
        {
          if ((modeId > 1 && bestMTS == modeId) || modeId == 1)
          {
            NStopMTS = (dct2Cost / trGrpBestCost) < trGrpStopThreshold;
          }
        }
      }
    }

    cu.lfnstIdx = bestLfnstIdx;
#if ISP_VVC
    if (dSingleCost != MAX_DOUBLE)
#endif
    {
      if (bestLfnstIdx != endLfnstIdx || bestMTS != EndMTS)
      {
#if ISP_VVC
        if (cu.ispMode)
        {
          const UnitArea& currArea = partitioner.currArea();
          cs.getRecoBuf(currArea.Y()).copyFrom(saveCS.getRecoBuf(currArea.Y()));

          if (saveCS.tus.size() != cs.tus.size())
          {
            partitioner.splitCurrArea(ispType, cs);

            do
            {
              partitioner.nextPart(cs);
              cs.addTU(CS::getArea(cs, partitioner.currArea(), partitioner.chType, partitioner.treeType),
                partitioner.chType, cs.cus[0]);
            } while (saveCS.tus.size() != cs.tus.size());

            partitioner.exitCurrSplit();
          }

          for (uint32_t j = 0; j < saveCS.tus.size(); j++)
          {
            cs.tus[j]->copyComponentFrom(*saveCS.tus[j], COMP_Y);
          }
        }
        else
#endif
        {
          cs.getRecoBuf(tu.Y()).copyFrom(saveCS.getRecoBuf(tu.Y()));

          tu.copyComponentFrom(*tmpTU, COMP_Y);
        }

        m_CABACEstimator->getCtx() = ctxBest;
      }

      // otherwise this would've happened in useSubStructure
      cs.picture->getRecoBuf(currArea.Y()).copyFrom(cs.getRecoBuf(currArea.Y()));
    }
  }
  else
  {
#if ISP_VVC
    if (cu.ispMode)
    {
      const PartSplit ispType = CU::getISPType(cu, COMP_Y);
      partitioner.splitCurrArea(ispType, cs);

      CUCtx      cuCtx;
      dSingleCost = xTestISP(cs, partitioner, bestCostForISP, ispType, splitCbfLumaSum, singleFracBits, singleDistLuma, cuCtx);
      partitioner.exitCurrSplit();
      bool storeCost = (numMode == 1) ? true : false;
      if ((m_pcEncCfg->m_ISP >= 2) && (numMode <= 1))
      {
        storeCost = true;
      }
      if (storeCost)
      {
        m_ispTestedModes[0].bestCost[cu.ispMode - 1] = dSingleCost;
      }
    }
    else
    {
      TransformUnit& tu =
        cs.addTU(CS::getArea(cs, currArea, partitioner.chType, partitioner.treeType), partitioner.chType, cs.cus[0]);
      tu.depth = currDepth;

      CHECK(!tu.Y().valid(), "Invalid TU");
      xIntraCodingTUBlock(tu, COMP_Y, false, singleDistLuma, &numSig, predBuf);
      //----- determine rate and r-d cost -----
#if ISP_VVC
      m_ispTestedModes[0].IspType = TU_NO_ISP;
      m_ispTestedModes[0].subTuCounter = -1;
#endif
      singleFracBits = xGetIntraFracBitsQT(cs, partitioner, true);
#if ISP_VVC
      dSingleCost = m_pcRdCost->calcRdCost(singleFracBits, singleDistLuma);
#endif
    }
#else
    xIntraCodingTUBlock(tu, COMP_Y, false, singleDistLuma, &numSig, predBuf);

    //----- determine rate and r-d cost -----
    singleFracBits = xGetIntraFracBitsQT(cs, partitioner, true);
#endif
  }
#if ISP_VVC

  if (cu.ispMode)
  { 
    for (auto& ptu : cs.tus)
    {
      if (currArea.Y().contains(ptu->Y()))
      {
        TU::setCbfAtDepth(*ptu, COMP_Y, currDepth, splitCbfLumaSum ? 1 : 0);
      }
    }
  }
#endif
  cs.dist     += singleDistLuma;
  cs.fracBits += singleFracBits;
#if ISP_VVC
  cs.cost      = dSingleCost;
#else
  cs.cost      = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );
#endif

  STAT_COUNT_CU_MODES( partitioner.chType == CH_L, g_cuCounters1D[CU_RD_TESTS][0][!cs.slice->isIntra() + cs.slice->depth] );
  STAT_COUNT_CU_MODES( partitioner.chType == CH_L && !cs.slice->isIntra(), g_cuCounters2D[CU_RD_TESTS][Log2( cs.area.lheight() )][Log2( cs.area.lwidth() )] );
}

#if ISP_VVC
ChromaCbfs IntraSearch::xIntraChromaCodingQT(CodingStructure& cs, Partitioner& partitioner)
#else
void IntraSearch::xIntraChromaCodingQT( CodingStructure &cs, Partitioner& partitioner )
#endif
{
  UnitArea    currArea      = partitioner.currArea();

  if( !currArea.Cb().valid() ) 
#if ISP_VVC
    return ChromaCbfs(false);
#else        
    return;
#endif

  TransformUnit& currTU     = *cs.getTU( currArea.chromaPos(), CH_C );
  const CodingUnit& cu  = *cs.getCU( currArea.chromaPos(), CH_C, TREE_D );
#if ISP_VVC
  ChromaCbfs cbfs(false);
  uint32_t   currDepth = partitioner.currTrDepth;
  if (currDepth == currTU.depth)
  {
    if (!currArea.Cb().valid() || !currArea.Cr().valid())
    {
      return cbfs;
    }
#endif

    CodingStructure& saveCS = *m_pSaveCS[1];
    saveCS.pcv = cs.pcv;
    saveCS.picture = cs.picture;
    saveCS.area.repositionTo(cs.area);

    TransformUnit& tmpTU = saveCS.tus.empty() ? saveCS.addTU(currArea, partitioner.chType, nullptr) : *saveCS.tus.front();
    tmpTU.initData();
    tmpTU.UnitArea::operator=(currArea);

    const unsigned      numTBlocks = getNumberValidTBlocks(*cs.pcv);

    CompArea& cbArea = currTU.blocks[COMP_Cb];
    CompArea& crArea = currTU.blocks[COMP_Cr];
    double     bestCostCb = MAX_DOUBLE;
    double     bestCostCr = MAX_DOUBLE;
    Distortion bestDistCb = 0;
    Distortion bestDistCr = 0;

    TempCtx ctxStartTU(m_CtxCache);
    TempCtx ctxStart(m_CtxCache);
    TempCtx ctxBest(m_CtxCache);

    ctxStartTU = m_CABACEstimator->getCtx();
#if TS_CHROMA
    ctxStart = m_CABACEstimator->getCtx();
#endif
    currTU.jointCbCr = 0;

    // Do predictions here to avoid repeating the "default0Save1Load2" stuff
#if BDPCM_VVC
    int  predMode = cu.bdpcmModeChroma ? BDPCM_IDX : CU::getFinalIntraMode(cu, CH_C);
#else
    uint32_t  predMode = CU::getFinalIntraMode(cu, CH_C);
#endif

    PelBuf piPredCb = cs.getPredBuf(COMP_Cb);
    PelBuf piPredCr = cs.getPredBuf(COMP_Cr);

    initIntraPatternChType(*currTU.cu, cbArea);
    initIntraPatternChType(*currTU.cu, crArea);

    if (CU::isLMCMode(predMode))
    {
      loadLMLumaRecPels(cu, cbArea);
      predIntraChromaLM(COMP_Cb, piPredCb, cu, cbArea, predMode);
      predIntraChromaLM(COMP_Cr, piPredCr, cu, crArea, predMode);
    }
    else
    {
      predIntraAng(COMP_Cb, piPredCb, cu);
      predIntraAng(COMP_Cr, piPredCr, cu);
    }

    // determination of chroma residuals including reshaping and cross-component prediction
    //----- get chroma residuals -----
    PelBuf resiCb = cs.getResiBuf(COMP_Cb);
    PelBuf resiCr = cs.getResiBuf(COMP_Cr);
    resiCb.subtract(cs.getOrgBuf(COMP_Cb), piPredCb);
    resiCr.subtract(cs.getOrgBuf(COMP_Cr), piPredCr);

    //----- get reshape parameter ----
    ReshapeData& reshapeData = cs.picture->reshapeData;
    bool doReshaping = (cs.picHeader->lmcsEnabled && cs.picHeader->lmcsChromaResidualScale && (cs.slice->isIntra() || reshapeData.getCTUFlag()) && (cbArea.width * cbArea.height > 4));
    if (doReshaping)
    {
      const Area area = currTU.Y().valid() ? currTU.Y() : Area(recalcPosition(currTU.chromaFormat, currTU.chType, CH_L, currTU.blocks[currTU.chType].pos()), recalcSize(currTU.chromaFormat, currTU.chType, CH_L, currTU.blocks[currTU.chType].size()));
      const CompArea& areaY = CompArea(COMP_Y, currTU.chromaFormat, area);
      currTU.chromaAdj = reshapeData.calculateChromaAdjVpduNei(currTU, areaY, currTU.cu->treeType);
    }

    //===== store original residual signals (std and crossCompPred) =====
    CompStorage  orgResiCb[5], orgResiCr[5]; // 0:std, 1-3:jointCbCr (placeholder at this stage), 4:crossComp
    for (int k = 0; k < 1; k += 4)
    {
      orgResiCb[k].create(cbArea);
      orgResiCr[k].create(crArea);
      orgResiCb[k].copyFrom(resiCb);
      orgResiCr[k].copyFrom(resiCr);

      if (doReshaping)
      {
        int cResScaleInv = currTU.chromaAdj;
        orgResiCb[k].scaleSignal(cResScaleInv, 1, cs.slice->clpRngs[COMP_Cb]);
        orgResiCr[k].scaleSignal(cResScaleInv, 1, cs.slice->clpRngs[COMP_Cr]);
      }
    }

    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    cuCtx.lfnstLastScanPos = false;

    CodingStructure& saveCScur = *m_pSaveCS[2];

    saveCScur.pcv = cs.pcv;
    saveCScur.picture = cs.picture;
    saveCScur.area.repositionTo(cs.area);

    TransformUnit& tmpTUcur = saveCScur.tus.empty() ? saveCScur.addTU(currArea, partitioner.chType, nullptr) : *saveCScur.tus.front();
    tmpTUcur.initData();
    tmpTUcur.UnitArea::operator=(currArea);

    TempCtx ctxBestTUL(m_CtxCache);

    const SPS& sps = *cs.sps;
    double     bestCostCbcur = MAX_DOUBLE;
    double     bestCostCrcur = MAX_DOUBLE;
    Distortion bestDistCbcur = 0;
    Distortion bestDistCrcur = 0;

    int  endLfnstIdx = (partitioner.isSepTree(cs) && partitioner.chType == CH_C && (partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8))
      || (partitioner.currArea().lwidth() > sps.getMaxTbSize() || partitioner.currArea().lheight() > sps.getMaxTbSize()) || !sps.LFNST ? 0 : 2;
    int  startLfnstIdx = 0;
    int  bestLfnstIdx = 0;
    bool NOTONE_LFNST = sps.LFNST ? true : false;

    // speedUps LFNST
    bool rapidLFNST = false;
    if (m_pcEncCfg->m_LFNST > 1)
    {
      rapidLFNST = true;
      if (m_pcEncCfg->m_LFNST > 2)
      {
        endLfnstIdx = endLfnstIdx ? 1 : 0;
      }
    }
#if TS_CHROMA
    int ts_used = 0;
    bool TScheck = false;
#endif
    if (partitioner.chType != CH_C)
    {
      startLfnstIdx = currTU.cu->lfnstIdx;
      endLfnstIdx = currTU.cu->lfnstIdx;
      bestLfnstIdx = currTU.cu->lfnstIdx;
      NOTONE_LFNST = false;
      rapidLFNST = false;
#if TS_CHROMA
      ts_used = currTU.mtsIdx[COMP_Y];
#endif
    }
#if BDPCM_VVC
    if (cu.bdpcmModeChroma)
    {
      endLfnstIdx = 0;
      NOTONE_LFNST = false;
    }
#endif

    double dSingleCostAll = MAX_DOUBLE;
    double singleCostTmpAll = 0;

    for (int lfnstIdx = startLfnstIdx; lfnstIdx <= endLfnstIdx; lfnstIdx++)
    {
      if (rapidLFNST && lfnstIdx)
      {
        if ((lfnstIdx == 2) && (bestLfnstIdx == 0))
        {
          continue;
        }
      }

      currTU.cu->lfnstIdx = lfnstIdx;
      if (lfnstIdx)
      {
        m_CABACEstimator->getCtx() = ctxStartTU;
      }

      cuCtx.lfnstLastScanPos = false;
      cuCtx.violatesLfnstConstrained[CH_L] = false;
      cuCtx.violatesLfnstConstrained[CH_C] = false;

      for (uint32_t c = COMP_Cb; c < numTBlocks; c++)
      {
        const ComponentID compID = ComponentID(c);
        const CompArea& area = currTU.blocks[compID];
        double     dSingleCost = MAX_DOUBLE;
        Distortion singleDistCTmp = 0;
        double     singleCostTmp = 0;
#if TS_CHROMA
#if BDPCM_VVC
        bool tsAllowed = TU::isTSAllowed(currTU, compID) && m_pcEncCfg->m_useChromaTS && !currTU.cu->lfnstIdx /*&& !cu.bdpcmModeChroma*/;
#else
        bool tsAllowed = TU::isTSAllowed(currTU, compID) && m_pcEncCfg->m_useChromaTS && !currTU.cu->lfnstIdx;
#endif
        if ((partitioner.chType == CH_L) && (!ts_used))
        {
          tsAllowed = false;
        }
#if DETECT_SC
        tsAllowed &= cs.picture->useSC;
#endif
        uint8_t nNumTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests       
        std::vector<TrMode> trModes;
        if (nNumTransformCands > 1)
        {
          trModes.push_back(TrMode(0, true));   // DCT2
          trModes.push_back(TrMode(1, true));   // TS
          TScheck = true;
        }
        bool cbfDCT2 = true;
        const bool isLastMode = NOTONE_LFNST || cs.sps->jointCbCr ||  tsAllowed ? false : true;
        int bestModeId = 0;
        ctxStart = m_CABACEstimator->getCtx();
        for (int modeId = 0; modeId < nNumTransformCands; modeId++)
        {
          if (doReshaping || lfnstIdx || modeId)
          {
            resiCb.copyFrom(orgResiCb[0]);
            resiCr.copyFrom(orgResiCr[0]);
          }
          if (modeId == 0)
          {
            if ( tsAllowed)
            {
              xPreCheckMTS(currTU, &trModes, m_pcEncCfg->m_MTSIntraMaxCand, 0, compID);
            }
          }

          currTU.mtsIdx[compID] = currTU.cu->bdpcmModeChroma ? MTS_SKIP : modeId;

          if (modeId)
          {
            if (!cbfDCT2 && trModes[modeId].first == MTS_SKIP)
            {
              break;
            }
            m_CABACEstimator->getCtx() = ctxStart;
          }
          singleDistCTmp = 0;
          if (tsAllowed)
          {
            xIntraCodingTUBlock(currTU, compID, false, singleDistCTmp, 0, 0, true);
            if ((modeId == 0) && (!trModes[modeId + 1].second))
            {
              nNumTransformCands = 1;
            }
          }
          else
#else
        const bool isLastMode = NOTONE_LFNST || cs.sps->jointCbCr ? false : true;
        if (doReshaping || lfnstIdx)
        {
          resiCb.copyFrom(orgResiCb[0]);
          resiCr.copyFrom(orgResiCr[0]);
        }
#endif
        {
          xIntraCodingTUBlock(currTU, compID, false, singleDistCTmp);
        }
#if TS_CHROMA
        if (((currTU.mtsIdx[compID] == MTS_SKIP && !currTU.cu->bdpcmModeChroma)
          && !TU::getCbf(currTU, compID)))   // In order not to code TS flag when cbf is zero, the case for TS with
                                             // cbf being zero is forbidden.
        {
          singleCostTmp = MAX_DOUBLE;
        }
        else
#endif
        {
          uint64_t fracBitsTmp = xGetIntraFracBitsQTChroma(currTU, compID, &cuCtx);
          singleCostTmp = m_pcRdCost->calcRdCost(fracBitsTmp, singleDistCTmp);
        }

        if (singleCostTmp < dSingleCost)
        {
          dSingleCost = singleCostTmp;

          if (compID == COMP_Cb)
          {
            bestCostCb = singleCostTmp;
            bestDistCb = singleDistCTmp;
          }
          else
          {
            bestCostCr = singleCostTmp;
            bestDistCr = singleDistCTmp;
          }
#if TS_CHROMA
          bestModeId = modeId;
          if (currTU.mtsIdx[compID] == MTS_DCT2_DCT2)
          {
            cbfDCT2 = TU::getCbfAtDepth(currTU, compID, currDepth);
          }
#endif
          if (!isLastMode)
          {
            saveCS.getRecoBuf(area).copyFrom(cs.getRecoBuf(area));
            tmpTU.copyComponentFrom(currTU, compID);
            ctxBest = m_CABACEstimator->getCtx();
          }
        }
#if TS_CHROMA
        }
        if ((TScheck) && ((c == COMP_Cb && bestModeId < (nNumTransformCands - 1)) ))
        {
          m_CABACEstimator->getCtx() = ctxBest;

          currTU.copyComponentFrom(tmpTU, COMP_Cb); // Cbf of Cb is needed to estimate cost for Cr Cbf
        }
#endif
      }

      singleCostTmpAll = bestCostCb + bestCostCr;

      bool rootCbfL = false;
      if (NOTONE_LFNST)
      {
        for (uint32_t t = 0; t < getNumberValidTBlocks(*cs.pcv); t++)
        {
          rootCbfL |= bool(tmpTU.cbf[t]);
        }
        if (rapidLFNST && !rootCbfL)
        {
          endLfnstIdx = lfnstIdx; // end this
        }
      }

      if (NOTONE_LFNST && lfnstIdx && !cuCtx.lfnstLastScanPos)
      {
        bool cbfAtZeroDepth = CU::isSepTree(*currTU.cu)
          ? rootCbfL : (cs.area.chromaFormat != CHROMA_400
            && std::min(tmpTU.blocks[1].width, tmpTU.blocks[1].height) < 4)
          ? TU::getCbfAtDepth(currTU, COMP_Y, currTU.depth) : rootCbfL;
        if (cbfAtZeroDepth)
        {
          singleCostTmpAll = MAX_DOUBLE;
        }
      }
#if TS_CHROMA
      if ((NOTONE_LFNST || TScheck) && (singleCostTmpAll < dSingleCostAll))
#else
      if (NOTONE_LFNST && (singleCostTmpAll < dSingleCostAll))
#endif
      {
        bestLfnstIdx = lfnstIdx;
#if TS_CHROMA
        if ((lfnstIdx != endLfnstIdx) || TScheck)
#else
        if (lfnstIdx != endLfnstIdx)
#endif
        {
          dSingleCostAll = singleCostTmpAll;

          bestCostCbcur = bestCostCb;
          bestCostCrcur = bestCostCr;
          bestDistCbcur = bestDistCb;
          bestDistCrcur = bestDistCr;

          saveCScur.getRecoBuf(cbArea).copyFrom(saveCS.getRecoBuf(cbArea));
          saveCScur.getRecoBuf(crArea).copyFrom(saveCS.getRecoBuf(crArea));

          tmpTUcur.copyComponentFrom(tmpTU, COMP_Cb);
          tmpTUcur.copyComponentFrom(tmpTU, COMP_Cr);
        }
        ctxBestTUL = m_CABACEstimator->getCtx();
      }
    }
#if TS_CHROMA
    if ((NOTONE_LFNST && (bestLfnstIdx != endLfnstIdx)) || TScheck)
#else
    if (NOTONE_LFNST && (bestLfnstIdx != endLfnstIdx))
#endif
    {
      bestCostCb = bestCostCbcur;
      bestCostCr = bestCostCrcur;
      bestDistCb = bestDistCbcur;
      bestDistCr = bestDistCrcur;
      currTU.cu->lfnstIdx = bestLfnstIdx;
      if (!cs.sps->jointCbCr)
      {
        cs.getRecoBuf(cbArea).copyFrom(saveCScur.getRecoBuf(cbArea));
        cs.getRecoBuf(crArea).copyFrom(saveCScur.getRecoBuf(crArea));

        currTU.copyComponentFrom(tmpTUcur, COMP_Cb);
        currTU.copyComponentFrom(tmpTUcur, COMP_Cr);

        m_CABACEstimator->getCtx() = ctxBestTUL;
      }
    }

    Distortion bestDistCbCr = bestDistCb + bestDistCr;

    if (cs.sps->jointCbCr)
    {
#if TS_CHROMA
      if ((NOTONE_LFNST && (bestLfnstIdx != endLfnstIdx)) || TScheck)
#else
      if (NOTONE_LFNST && (bestLfnstIdx != endLfnstIdx))
#endif
      {
        saveCS.getRecoBuf(cbArea).copyFrom(saveCScur.getRecoBuf(cbArea));
        saveCS.getRecoBuf(crArea).copyFrom(saveCScur.getRecoBuf(crArea));

        tmpTU.copyComponentFrom(tmpTUcur, COMP_Cb);
        tmpTU.copyComponentFrom(tmpTUcur, COMP_Cr);
        m_CABACEstimator->getCtx() = ctxBestTUL;
        ctxBest = m_CABACEstimator->getCtx();
      }
      // Test using joint chroma residual coding
      double     bestCostCbCr = bestCostCb + bestCostCr;
      int        bestJointCbCr = 0;
#if TS_CHROMA
      bool checkDCTOnly = m_pcEncCfg->m_useChromaTS && ((TU::getCbf(tmpTU, COMP_Cb) && tmpTU.mtsIdx[COMP_Cb] == MTS_DCT2_DCT2 && !TU::getCbf(tmpTU, COMP_Cr)) ||
        (TU::getCbf(tmpTU, COMP_Cr) && tmpTU.mtsIdx[COMP_Cr] == MTS_DCT2_DCT2 && !TU::getCbf(tmpTU, COMP_Cb)) ||
        (TU::getCbf(tmpTU, COMP_Cb) && tmpTU.mtsIdx[COMP_Cb] == MTS_DCT2_DCT2 && TU::getCbf(tmpTU, COMP_Cr) && tmpTU.mtsIdx[COMP_Cr] == MTS_DCT2_DCT2));
      bool checkTSOnly = m_pcEncCfg->m_useChromaTS && ((TU::getCbf(tmpTU, COMP_Cb) && tmpTU.mtsIdx[COMP_Cb] == MTS_SKIP && !TU::getCbf(tmpTU, COMP_Cr)) ||
        (TU::getCbf(tmpTU, COMP_Cr) && tmpTU.mtsIdx[COMP_Cr] == MTS_SKIP && !TU::getCbf(tmpTU, COMP_Cb)) ||
        (TU::getCbf(tmpTU, COMP_Cb) && tmpTU.mtsIdx[COMP_Cb] == MTS_SKIP && TU::getCbf(tmpTU, COMP_Cr) && tmpTU.mtsIdx[COMP_Cr] == MTS_SKIP));
#endif
      bool       lastIsBest = false;
      bool NOLFNST1 = false;
      if (rapidLFNST && (startLfnstIdx != endLfnstIdx))
      {
        if (bestLfnstIdx == 2)
        {
          NOLFNST1 = true;
        }
        else
        {
          endLfnstIdx = 1;
        }
      }

      for (int lfnstIdxj = startLfnstIdx; lfnstIdxj <= endLfnstIdx; lfnstIdxj++)
      {
        if (rapidLFNST && NOLFNST1 && (lfnstIdxj == 1))
        {
          continue;
        }
        currTU.cu->lfnstIdx = lfnstIdxj;
        std::vector<int> jointCbfMasksToTest;
        if (TU::getCbf(tmpTU, COMP_Cb) || TU::getCbf(tmpTU, COMP_Cr))
        {
          jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(currTU, orgResiCb, orgResiCr);
        }
#if 0//BDPCM_VVC
        if (jointCbfMasksToTest.size() && currTU.cu->bdpcmModeChroma)
        {
          //CHECK(!checkTSOnly || checkDCTOnly, "bdpcm only allows transform skip");
        }
#endif
        for (int cbfMask : jointCbfMasksToTest)
        {
#if !TS_CHROMA
          Distortion distTmp = 0;
#endif
          currTU.jointCbCr = (uint8_t)cbfMask;
#if TS_CHROMA
          ComponentID codeCompId = ((currTU.jointCbCr >> 1) ? COMP_Cb : COMP_Cr);
          ComponentID otherCompId = ((codeCompId == COMP_Cb) ? COMP_Cr : COMP_Cb);
#if BDPCM_VVC
          bool tsAllowed = TU::isTSAllowed(currTU, codeCompId) && (m_pcEncCfg->m_useChromaTS) && !currTU.cu->lfnstIdx && !cu.bdpcmModeChroma;
#else
          bool        tsAllowed = TU::isTSAllowed(currTU, codeCompId) && (m_pcEncCfg->m_useChromaTS) && !currTU.cu->lfnstIdx;
#endif
          if ((partitioner.chType == CH_L)&& tsAllowed && (currTU.mtsIdx[COMP_Y] != MTS_SKIP))
          {
            tsAllowed = false;
          }
#if DETECT_SC
          tsAllowed &= cs.picture->useSC;
#endif
          if (!tsAllowed)
          {
            checkTSOnly = false;
          }
          uint8_t     numTransformCands = 1 + (tsAllowed && !(checkDCTOnly || checkTSOnly)? 1 : 0); // DCT + TS = 2 tests
          std::vector<TrMode> trModes;
          if (numTransformCands > 1)
          {
            trModes.push_back(TrMode(0, true)); // DCT2
            trModes.push_back(TrMode(1, true));//TS
          }
          else
          {
#if DETECT_SC
            currTU.mtsIdx[codeCompId] = checkTSOnly || currTU.cu->bdpcmModeChroma ? 1 : 0;
#else
            currTU.mtsIdx[codeCompId] = checkTSOnly ? 1 : 0;
#endif
          }

          for (int modeId = 0; modeId < numTransformCands; modeId++)
          {
            Distortion distTmp = 0;
#if DETECT_SC
            currTU.mtsIdx[codeCompId] = currTU.cu->bdpcmModeChroma ? MTS_SKIP : MTS_DCT2_DCT2;
#endif
            if (numTransformCands > 1)
            {
              currTU.mtsIdx[codeCompId] = currTU.cu->bdpcmModeChroma ? MTS_SKIP : trModes[modeId].first;
            }
            currTU.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
#endif

            m_CABACEstimator->getCtx() = ctxStartTU;

            resiCb.copyFrom(orgResiCb[cbfMask]);
            resiCr.copyFrom(orgResiCr[cbfMask]);
#if TS_CHROMA
            if ((modeId == 0) && (numTransformCands > 1))
            {
              xPreCheckMTS(currTU, &trModes, m_pcEncCfg->m_MTSIntraMaxCand, 0, COMP_Cb);
              currTU.mtsIdx[codeCompId] = trModes[modeId].first;
              currTU.mtsIdx[(codeCompId == COMP_Cr) ? COMP_Cb : COMP_Cr] = MTS_DCT2_DCT2;
            }
#endif
            cuCtx.lfnstLastScanPos = false;
            cuCtx.violatesLfnstConstrained[CH_L] = false;
            cuCtx.violatesLfnstConstrained[CH_C] = false;
#if TS_CHROMA
            if (numTransformCands > 1)
            {
              xIntraCodingTUBlock(currTU, COMP_Cb, false, distTmp, 0, 0, true);
              if ((modeId == 0) && !trModes[modeId + 1].second)
              {
                numTransformCands = 1;
              }
            }
            else
#endif
            {
              xIntraCodingTUBlock(currTU, COMP_Cb, false, distTmp, 0);
            }

            double costTmp = std::numeric_limits<double>::max();
            if (distTmp < MAX_DISTORTION)
            {
              uint64_t bits = xGetIntraFracBitsQTChroma(currTU, COMP_Cb, &cuCtx);
              costTmp = m_pcRdCost->calcRdCost(bits, distTmp);
            }
#if TS_CHROMA
            else if (!currTU.mtsIdx[codeCompId])
            {
              numTransformCands = 1;
            }
#endif
            bool rootCbfL = false;
            for (uint32_t t = 0; t < getNumberValidTBlocks(*cs.pcv); t++)
            {
              rootCbfL |= bool(tmpTU.cbf[t]);
            }
            if (rapidLFNST && !rootCbfL)
            {
              endLfnstIdx = lfnstIdxj;
            }
            if (NOTONE_LFNST && currTU.cu->lfnstIdx && !cuCtx.lfnstLastScanPos)
            {
              bool cbfAtZeroDepth = CU::isSepTree(*currTU.cu) ? rootCbfL
                : (cs.area.chromaFormat != CHROMA_400 && std::min(tmpTU.blocks[1].width, tmpTU.blocks[1].height) < 4)
                ? TU::getCbfAtDepth(currTU, COMP_Y, currTU.depth) : rootCbfL;
              if (cbfAtZeroDepth)
              {
                costTmp = MAX_DOUBLE;
              }
            }
            if (costTmp < bestCostCbCr)
            {
              bestCostCbCr = costTmp;
              bestDistCbCr = distTmp;
              bestJointCbCr = currTU.jointCbCr;

              // store data
              bestLfnstIdx = lfnstIdxj;
#if TS_CHROMA
              if ((cbfMask != jointCbfMasksToTest.back() || (lfnstIdxj != endLfnstIdx)) || (modeId != (numTransformCands - 1)))
#else
              if (cbfMask != jointCbfMasksToTest.back() || (lfnstIdxj != endLfnstIdx))
#endif
              {
                saveCS.getRecoBuf(cbArea).copyFrom(cs.getRecoBuf(cbArea));
                saveCS.getRecoBuf(crArea).copyFrom(cs.getRecoBuf(crArea));

                tmpTU.copyComponentFrom(currTU, COMP_Cb);
                tmpTU.copyComponentFrom(currTU, COMP_Cr);

                ctxBest = m_CABACEstimator->getCtx();
              }
              else
              {
                lastIsBest = true;
                cs.cus[0]->lfnstIdx = bestLfnstIdx;
              }
            }
#if TS_CHROMA
          }
#endif
        }

        // Retrieve the best CU data (unless it was the very last one tested)
      }
      if (!lastIsBest)
      {
        cs.getRecoBuf(cbArea).copyFrom(saveCS.getRecoBuf(cbArea));
        cs.getRecoBuf(crArea).copyFrom(saveCS.getRecoBuf(crArea));

        cs.cus[0]->lfnstIdx = bestLfnstIdx;
        currTU.copyComponentFrom(tmpTU, COMP_Cb);
        currTU.copyComponentFrom(tmpTU, COMP_Cr);
        m_CABACEstimator->getCtx() = ctxBest;
      }
      currTU.jointCbCr = (TU::getCbf(currTU, COMP_Cb) | TU::getCbf(currTU, COMP_Cr)) ? bestJointCbCr : 0;
    } // jointCbCr

    cs.dist += bestDistCbCr;
    cuCtx.violatesLfnstConstrained[CH_L] = false;
    cuCtx.violatesLfnstConstrained[CH_C] = false;
    cuCtx.lfnstLastScanPos = false;
    cuCtx.violatesMtsCoeffConstraint = false;
    cuCtx.mtsLastScanPos = false;
#if ISP_VVC
    cbfs.cbf(COMP_Cb) = TU::getCbf(currTU, COMP_Cb);
    cbfs.cbf(COMP_Cr) = TU::getCbf(currTU, COMP_Cr);
  }
  else
  {
    unsigned   numValidTBlocks = getNumberValidTBlocks(*cs.pcv);
    ChromaCbfs SplitCbfs(false);

    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else if (currTU.cu->ispMode)
    {
      partitioner.splitCurrArea(m_ispTestedModes[0].IspType, cs);
    }
    else
      THROW("Implicit TU split not available");

    do
    {
      ChromaCbfs subCbfs = xIntraChromaCodingQT(cs, partitioner);

      for (uint32_t ch = COMP_Cb; ch < numValidTBlocks; ch++)
      {
        const ComponentID compID = ComponentID(ch);
        SplitCbfs.cbf(compID) |= subCbfs.cbf(compID);
      }
    } while (partitioner.nextPart(cs));

    partitioner.exitCurrSplit();

    /*if (lumaUsesISP && cs.dist == MAX_UINT) //ahenkel
    {
      return cbfs;
    }*/
    {
      cbfs.Cb |= SplitCbfs.Cb;
      cbfs.Cr |= SplitCbfs.Cr;

      if (1)   //(!lumaUsesISP)
      {
        for (auto& ptu : cs.tus)
        {
          if (currArea.Cb().contains(ptu->Cb()) || (!ptu->Cb().valid() && currArea.Y().contains(ptu->Y())))
          {
            TU::setCbfAtDepth(*ptu, COMP_Cb, currDepth, SplitCbfs.Cb);
            TU::setCbfAtDepth(*ptu, COMP_Cr, currDepth, SplitCbfs.Cr);
          }
        }
      }
    }
  }
  return cbfs;
#endif
}

uint64_t IntraSearch::xFracModeBitsIntraLuma(const CodingUnit& cu, const unsigned* mpmLst)
{
  m_CABACEstimator->resetBits();

  if (!cu.ciip)
  {
    m_CABACEstimator->intra_luma_pred_mode(cu, mpmLst);
  }

  return m_CABACEstimator->getEstFracBits();
}

template<typename T, size_t N, int M>
void IntraSearch::xReduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, SortedPelUnitBufs<M>& sortedPelBuffer, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const CodingUnit& cu, const bool fastMip)
{
  const int maxCandPerType = numModesForFullRD >> 1;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> tempRdModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> tempCandCostList;
  const double minCost = candCostList[0];
  bool keepOneMip = candModeList.size() > numModesForFullRD;
  const int maxNumConv = 3; 

  int numConv = 0;
  int numMip = 0;
  for (int idx = 0; idx < candModeList.size() - (keepOneMip?0:1); idx++)
  {
    bool addMode = false;
    const ModeInfo& orgMode = candModeList[idx];

    if (!orgMode.mipFlg)
    {
      addMode = (numConv < maxNumConv);
      numConv += addMode ? 1:0;
    }
    else
    {
      addMode = ( numMip < maxCandPerType || (candCostList[idx] < thresholdHadCost * minCost) || keepOneMip );
      keepOneMip = false;
      numMip += addMode ? 1:0;
    }
    if( addMode )
    {
      tempRdModeList.push_back(orgMode);
      tempCandCostList.push_back(candCostList[idx]);
    }
  }

  // sort Pel Buffer
  int i = -1;
  for( auto &m: tempRdModeList)
  {
    if( ! (m == candModeList.at( ++i )) )
    {
      for( int j = i; j < (int)candModeList.size()-1; )
      {
        if( m == candModeList.at( ++j ) )
        {
          sortedPelBuffer.swap( i, j);
          break;
        }
      }
    }
  }
  sortedPelBuffer.reduceTo( (int)tempRdModeList.size() );

  if ((cu.lwidth() > 8 && cu.lheight() > 8))
  {
    // Sort MIP candidates by Hadamard cost
    const int transpOff = getNumModesMip(cu.Y());
    static_vector<uint8_t, FAST_UDI_MAX_RDMODE_NUM> sortedMipModes(0);
    static_vector<double, FAST_UDI_MAX_RDMODE_NUM> sortedMipCost(0);
    for (uint8_t mode : { 0, 1, 2 })
    {
      uint8_t candMode = mode + uint8_t((mipHadCost[mode + transpOff] < mipHadCost[mode]) ? transpOff : 0);
      updateCandList(candMode, mipHadCost[candMode], sortedMipModes, sortedMipCost, 3);
    }

    // Append MIP mode to RD mode list
    const int modeListSize = int(tempRdModeList.size());
    for (int idx = 0; idx < 3; idx++)
    {
      const bool     isTransposed = (sortedMipModes[idx] >= transpOff ? true : false);
      const uint32_t mipIdx       = (isTransposed ? sortedMipModes[idx] - transpOff : sortedMipModes[idx]);
      const ModeInfo mipMode( true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mipIdx );
      bool alreadyIncluded = false;
      for (int modeListIdx = 0; modeListIdx < modeListSize; modeListIdx++)
      {
        if (tempRdModeList[modeListIdx] == mipMode)
        {
          alreadyIncluded = true;
          break;
        }
      }

      if (!alreadyIncluded)
      {
        tempRdModeList.push_back(mipMode);
        tempCandCostList.push_back(0);
        if( fastMip ) break;
      }
    }
  }

  candModeList = tempRdModeList;
  candCostList = tempCandCostList;
  numModesForFullRD = int(candModeList.size());
}

#if TS_CHROMA
void IntraSearch::xPreCheckMTS(TransformUnit &tu, std::vector<TrMode> *trModes, const int maxCand, PelUnitBuf *predBuf, const ComponentID& compID)
#else
void IntraSearch::xPreCheckMTS(TransformUnit &tu, std::vector<TrMode> *trModes, const int maxCand, PelUnitBuf *predBuf)
#endif
{
  CodingStructure&   cs          = *tu.cs;
#if TS_CHROMA
  const CompArea& area = tu.blocks[compID];
#else
  const CompArea&    area        = tu.blocks[COMP_Y];
#endif
  const ReshapeData& reshapeData = cs.picture->reshapeData;
#if TS_CHROMA
  PelBuf piPred = cs.getPredBuf(area);
  PelBuf piResi = cs.getResiBuf(area);
#else
  PelBuf piPred    = cs.getPredBuf(COMP_Y);
  PelBuf piResi    = cs.getResiBuf(COMP_Y);
#endif

  const CodingUnit& cu = *cs.getCU(area.pos(), CH_L,TREE_D);
#if TS_CHROMA
  if (compID == COMP_Y)
#endif
  {
    initIntraPatternChType(*tu.cu, area);
    if (predBuf)
    {
      piPred.copyFrom(predBuf->Y());
    }
    else if (CU::isMIP(cu, CH_L))
    {
      initIntraMip(cu);
      predIntraMip(piPred, cu);
    }
    else
    {
      predIntraAng(COMP_Y, piPred, cu);
    }
  }

  //===== get residual signal =====
#if TS_CHROMA
  if (isLuma(compID))
#endif
  {
    if (cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag())
    {
      piResi.subtract(cs.getRspOrgBuf(), piPred);
    }
    else
    {
      CPelBuf piOrg = cs.getOrgBuf(COMP_Y);
      piResi.subtract(piOrg, piPred);
    }
  }

#if TS_CHROMA
  if (isChroma(compID))
  {
    ComponentID codeCompId = (tu.jointCbCr ? (tu.jointCbCr >> 1 ? COMP_Cb : COMP_Cr) : compID);
    m_pcTrQuant->checktransformsNxN(tu, trModes, m_pcEncCfg->m_MTSIntraMaxCand, codeCompId);
  }
  else
    m_pcTrQuant->checktransformsNxN(tu, trModes, m_pcEncCfg->m_MTSIntraMaxCand, compID);
#else
  m_pcTrQuant->checktransformsNxN(tu, trModes, m_pcEncCfg->m_MTSIntraMaxCand);
#endif
}

#if ISP_VVC
double IntraSearch::xTestISP(CodingStructure& cs, Partitioner& subTuPartitioner, double bestCostForISP, PartSplit ispType, bool& splitcbf, uint64_t& singleFracBits, Distortion& singleDistLuma, CUCtx& cuCtx)
{
  int  subTuCounter = 0;
  bool earlySkipISP = false;
  bool splitCbfLuma = false;
  CodingUnit& cu = *cs.cus[0];

  Distortion singleDistTmpLumaSUM = 0;
  uint64_t   singleTmpFracBitsSUM = 0;
  double     singleCostTmpSUM = 0;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;

  do
  {
    Distortion singleDistTmpLuma = 0;
    uint64_t   singleTmpFracBits = 0;
    double     singleCostTmp = 0;
    TransformUnit& tmpTUcur = ((cs.tus.size() < (subTuCounter + 1)))
      ? cs.addTU(CS::getArea(cs, subTuPartitioner.currArea(), subTuPartitioner.chType,
        subTuPartitioner.treeType),
        subTuPartitioner.chType, cs.cus[0])
      : *cs.tus[subTuCounter];
    tmpTUcur.depth = subTuPartitioner.currTrDepth;

    // Encode TU
    xIntraCodingTUBlock(tmpTUcur, COMP_Y, false, singleDistTmpLuma, 0);
    cuCtx.mtsLastScanPos = false;

    if (singleDistTmpLuma == MAX_INT)   // all zero CBF skip
    {
      earlySkipISP = true;
      singleCostTmpSUM = MAX_DOUBLE;
      break;
    }

    {
      if (m_pcRdCost->calcRdCost(singleTmpFracBitsSUM, singleDistTmpLumaSUM + singleDistTmpLuma) > bestCostForISP)
      {
        earlySkipISP = true;
      }
      else
      {
        m_ispTestedModes[0].IspType = ispType;
        m_ispTestedModes[0].subTuCounter = subTuCounter;
        singleTmpFracBits = xGetIntraFracBitsQT(cs, subTuPartitioner, true, &cuCtx);
      }
      singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
    }

    singleCostTmpSUM += singleCostTmp;
    singleDistTmpLumaSUM += singleDistTmpLuma;
    singleTmpFracBitsSUM += singleTmpFracBits;

    subTuCounter++;

    splitCbfLuma |= TU::getCbfAtDepth(
      *cs.getTU(subTuPartitioner.currArea().lumaPos(), subTuPartitioner.chType, subTuCounter - 1), COMP_Y,
      subTuPartitioner.currTrDepth);
    int nSubPartitions = m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1];
    int doStop = (m_pcEncCfg->m_ISP == 1) ? (subTuCounter < nSubPartitions) ? true : false : true;
    if (doStop)
    {
      if (singleCostTmpSUM > bestCostForISP)
      {
        earlySkipISP = true;
        break;
      }
      if (subTuCounter < nSubPartitions)
      {
        double threshold = nSubPartitions == 2 ? 0.95 : subTuCounter == 1 ? 0.83 : 0.91;
        if (singleCostTmpSUM > bestCostForISP * threshold)
        {
          earlySkipISP = true;
          break;
        }
      }
    }
  } while (subTuPartitioner.nextPart(cs));
  singleDistLuma = singleDistTmpLumaSUM;
  singleFracBits = singleTmpFracBitsSUM;

  splitcbf = splitCbfLuma;
  return earlySkipISP ? MAX_DOUBLE : singleCostTmpSUM;
}

int IntraSearch::xSpeedISP(int speed, bool& testISP, int mode, int& noISP, int& endISP, CodingUnit& cu, static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>& RdModeList, ModeInfo       uiBestPUMode, int bestISP, int bestLfnstIdx)
{
  if (speed)
  {
    if (mode >= 1)
    {
      if (m_ispTestedModes[0].splitIsFinished[1] && m_ispTestedModes[0].splitIsFinished[0])
      {
        testISP = false;
        endISP = 0;
      }
      else
      {
        if (m_pcEncCfg->m_ISP >= 2)
        {
          if (mode == 1) //best Hor||Ver
          {
            int bestDir = 0;
            for (int d = 0; d < 2; d++)
            {
              int d2 = d ? 0 : 1;
              if ((m_ispTestedModes[0].bestCost[d] <= m_ispTestedModes[0].bestCost[d2])
                && (m_ispTestedModes[0].bestCost[d] != MAX_DOUBLE))
              {
                bestDir = d + 1;
                m_ispTestedModes[0].splitIsFinished[d2] = true;
              }
            }
            m_ispTestedModes[0].bestModeSoFar = bestDir;
            if (m_ispTestedModes[0].bestModeSoFar <= 0)
            {
              m_ispTestedModes[0].splitIsFinished[1] = true;
              m_ispTestedModes[0].splitIsFinished[0] = true;
              testISP = false;
              endISP = 0;
            }
          }
          if (m_ispTestedModes[0].bestModeSoFar == 2)
          {
            noISP = 1;
          }
          else
            endISP = 1;
        }
      }
    }
    if (testISP)
    {
      if (mode == 2)
      {
        for (int d = 0; d < 2; d++)
        {
          int d2 = d ? 0 : 1;
          if (m_ispTestedModes[0].bestCost[d] == MAX_DOUBLE)
          {
            m_ispTestedModes[0].splitIsFinished[d] = true;
          }
          if ((m_ispTestedModes[0].bestCost[d2] < 1.3 * m_ispTestedModes[0].bestCost[d])
            && (int(m_ispTestedModes[0].bestSplitSoFar) != (d + 1)))
          {
            if (d)
            {
              endISP = 1;
            }
            else
            {
              noISP = 1;
            }
            m_ispTestedModes[0].splitIsFinished[d] = true;
          }
        }
      }
      else
      {
        if (m_ispTestedModes[0].splitIsFinished[0])
        {
          noISP = 1;
        }
        if (m_ispTestedModes[0].splitIsFinished[1])
        {
          endISP = 1;
        }
      }
    }
    if ((noISP == 1) && (endISP == 1))
    {
      endISP = 0;
    }
  }
  else
  {
    bool stopFound = false;
    if (m_pcEncCfg->m_ISP >= 3)
    {
      if (mode)
      {
        if ((bestISP == 0) || ((uiBestPUMode.modeId != RdModeList[mode - 1].modeId)
          && (uiBestPUMode.modeId != RdModeList[mode].modeId)))
        {
          stopFound = true;
        }
      }
    }
    if (cu.mipFlag || cu.multiRefIdx)
    {
      cu.mipFlag = false;
      cu.multiRefIdx = 0;
      if (!stopFound)
      {
        for (int k = 0; k < mode; k++)
        {
          if (cu.intraDir[CH_L] == RdModeList[k].modeId)
          {
            stopFound = true;
            break;
          }
        }
      }
    }
    if (stopFound)
    {
      testISP = false;
      endISP = 0;
      return 1;
    }
    if (!stopFound && (m_pcEncCfg->m_ISP >= 2) && (cu.intraDir[CH_L] == DC_IDX))
    {
      stopFound = true;
      endISP = 0;
      return 1;
    }
  }
  return 0;
}
#endif

} // namespace vvenc

//! \}


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


/** \file     EncSearch.cpp
 *  \brief    encoder inter search class
 */

#include "InterSearch.h"
#include "EncModeCtrl.h"
#include "EncLib.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/Reshape.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"

#include <math.h>

 //! \ingroup EncoderLib
 //! \{

namespace vvenc {

static const Mv s_acMvRefineH[9] =
{
  Mv(  0,  0 ), // 0
  Mv(  0, -1 ), // 1
  Mv(  0,  1 ), // 2
  Mv( -1,  0 ), // 3
  Mv(  1,  0 ), // 4
  Mv( -1, -1 ), // 5
  Mv(  1, -1 ), // 6
  Mv( -1,  1 ), // 7
  Mv(  1,  1 )  // 8
};

static const Mv s_acMvRefineQ[9] =
{
  Mv(  0,  0 ), // 0
  Mv(  0, -1 ), // 1
  Mv(  0,  1 ), // 2
  Mv( -1, -1 ), // 5
  Mv(  1, -1 ), // 6
  Mv( -1,  0 ), // 3
  Mv(  1,  0 ), // 4
  Mv( -1,  1 ), // 7
  Mv(  1,  1 )  // 8
};

static const bool s_skipQpelPosition[ 42 ][ 9 ] =
{
  { false, true,  true,  true,  true,  true,  true,  true,  true  },
  { true,  true,  true,  true,  true,  false, true,  true,  true  },
  { true,  true,  true,  true,  true,  true,  false, true,  true  },
  { true,  false, true,  true,  true,  true,  true,  true,  true  },
  { true,  false, true,  false, true,  false, true,  true,  true  },
  { true,  false, true,  true,  false, true,  false, true,  true  },
  { true,  true,  false, true,  true,  true,  true,  true,  true  },
  { true,  true,  false, true,  true,  false, true,  false, true  },
  { true,  true,  false, true,  true,  true,  false, true,  false },
  { true,  true,  false, true,  true,  true,  true,  false, false },
  { true,  true,  true,  true,  true,  false, true,  true,  true  },
  { true,  true,  false, true,  true,  false, true,  false, true  },
  { true,  true,  true,  true,  true,  true,  false, true,  true  },
  { true,  true,  false, true,  true,  true,  false, true,  false },
  { true,  false, true,  false, false, true,  true,  true,  true  },
  { true,  true,  true,  true,  true,  false, true,  true,  true  },
  { true,  false, true,  false, true,  false, true,  true,  true  },
  { true,  true,  true,  true,  true,  true,  false, true,  true  },
  { true,  false, true,  true,  false, true,  false, true,  true  },
  { true,  true,  true,  true,  false, true,  false, true,  false },
  { true,  false, true,  true,  true,  true,  true,  true,  true  },
  { true,  false, true,  true,  false, true,  false, true,  true  },
  { true,  true,  false, true,  true,  true,  true,  true,  true  },
  { true,  true,  false, true,  true,  true,  false, true,  false },
  { true,  true,  true,  false, true,  false, true,  false, true  },
  { true,  false, true,  true,  true,  true,  true,  true,  true  },
  { true,  false, true,  false, true,  false, true,  true,  true  },
  { true,  true,  false, true,  true,  true,  true,  true,  true  },
  { true,  true,  false, true,  true,  false, true,  false, true  },
  { true,  true,  true,  true,  true,  true,  false, true,  true  },
  { true,  true,  false, true,  true,  true,  true,  true,  true  },
  { true,  true,  false, true,  true,  true,  false, true,  false },
  { true,  true,  true,  true,  true,  false, true,  true,  true  },
  { true,  true,  false, true,  true,  true,  true,  true,  true  },
  { true,  true,  false, true,  true,  false, true,  false, true  },
  { true,  true,  true,  true,  true,  true,  false, true,  true  },
  { true,  false, true,  true,  true,  true,  true,  true,  true  },
  { true,  false, true,  true,  false, true,  false, true,  true  },
  { true,  true,  true,  true,  true,  false, true,  true,  true  },
  { true,  false, true,  true,  true,  true,  true,  true,  true  },
  { true,  false, true,  false, true,  false, true,  true,  true  },
  { false, false, false, false, false, false, false, false, false },
};

//   1,0    3,0    0,1    1,1    2,1    3,1    1,2    3,2    0,3    1,3    2,3    3,3    H1,0   H3,0
static const bool s_doInterpQ[ 42 ][ 14 ] =
{
  { false, false, false, false, false, false, false, false, false, false, false, false, false, false },
  { false, false, false, false, false, false, false, false, true,  false, false, false, false, true  },
  { false, false, true,  false, false, false, false, false, false, false, false, false, true,  false },
  { false, true,  false, false, false, false, false, false, false, false, false, false, false, false },
  { false, true,  false, false, false, false, false, false, true,  false, false, true,  false, true  },
  { false, true,  true,  false, false, true,  false, false, false, false, false, false, true,  false },
  { true,  false, false, false, false, false, false, false, false, false, false, false, false, false },
  { true,  false, false, false, false, false, false, false, true,  true,  false, false, false, true  },
  { true,  false, true,  true,  false, false, false, false, false, false, false, false, true,  false },
  { false, true,  false, false, false, true,  false, false, false, false, false, true,  true,  true  },
  { false, false, false, false, false, false, false, false, false, false, true,  false, false, true  },
  { false, true,  false, false, false, false, false, false, false, false, true,  true,  false, true  },
  { false, false, false, false, true,  false, false, false, false, false, false, false, true,  false },
  { false, true,  false, false, true,  true,  false, false, false, false, false, false, true,  false },
  { true,  false, false, true,  false, false, false, false, false, true,  false, false, true,  true  },
  { false, false, false, false, false, false, false, false, false, false, true,  false, false, true  },
  { true,  false, false, false, false, false, false, false, false, true,  true,  false, false, true  },
  { false, false, false, false, true,  false, false, false, false, false, false, false, true,  false },
  { true,  false, false, true,  true,  false, false, false, false, false, false, false, true,  false },
  { false, false, false, false, false, false, false, false, true,  true,  false, true,  false, true  },
  { false, false, false, false, false, false, false, true,  false, false, false, false, false, false },
  { false, false, false, false, false, false, false, true,  true,  false, false, true,  false, true  },
  { false, false, false, false, false, false, true,  false, false, false, false, false, false, false },
  { false, false, false, false, false, false, true,  false, true,  true,  false, false, false, true  },
  { false, false, true,  true,  false, true,  false, false, false, false, false, false, true,  false },
  { false, false, false, false, false, false, false, true,  false, false, false, false, false, false },
  { false, false, true,  false, false, true,  false, true,  false, false, false, false, true,  false },
  { false, false, false, false, false, false, true,  false, false, false, false, false, false, false },
  { false, false, true,  true,  false, false, true,  false, false, false, false, false, true,  false },
  { false, false, false, false, false, false, false, false, false, false, true,  false, false, true  },
  { false, false, false, false, false, false, false, true,  false, false, false, false, false, false },
  { false, false, false, false, false, false, false, true,  false, false, true,  true,  false, true  },
  { false, false, false, false, true,  false, false, false, false, false, false, false, true,  false },
  { false, false, false, false, false, false, false, true,  false, false, false, false, false, false },
  { false, false, false, false, true,  true,  false, true,  false, false, false, false, true,  false },
  { false, false, false, false, false, false, false, false, false, false, true,  false, false, true  },
  { false, false, false, false, false, false, true,  false, false, false, false, false, false, false },
  { false, false, false, false, false, false, true,  false, false, true,  true,  false, false, true  },
  { false, false, false, false, true,  false, false, false, false, false, false, false, true,  false },
  { false, false, false, false, false, false, true,  false, false, false, false, false, false, false },
  { false, false, false, true,  true,  false, true,  false, false, false, false, false, true,  false },
  { true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true,  true  },
};


InterSearch::InterSearch()
  : m_modeCtrl                    (nullptr)
  , m_defaultCachedBvs            (nullptr)
  , m_pcEncCfg                    (nullptr)
  , m_pcTrQuant                   (nullptr)
  , m_iSearchRange                (0)
  , m_bipredSearchRange           (0)
  , m_motionEstimationSearchMethod(VVENC_MESEARCH_FULL)
  , m_motionEstimationSearchMethodSCC( 0 )
  , m_CABACEstimator              (nullptr)
  , m_CtxCache                    (nullptr)
  , m_pTempPel                    (nullptr)
{
  for (int i=0; i<MAX_NUM_REF_LIST_ADAPT_SR; i++)
  {
    memset (m_aaiAdaptSR[i], 0, MAX_IDX_ADAPT_SR * sizeof (int));
  }
  for (int i=0; i<AMVP_MAX_NUM_CANDS+1; i++)
  {
    memset (m_auiMVPIdxCost[i], 0, (AMVP_MAX_NUM_CANDS+1) * sizeof (uint32_t) );
  }
}


InterSearch::~InterSearch()
{
  destroy();
}

void InterSearch::init( const VVEncCfg& encCfg, TrQuant* pTrQuant, RdCost* pRdCost, EncModeCtrl* pModeCtrl, CodingStructure **pSaveCS )
{
  InterPrediction::init( pRdCost, encCfg.m_internChromaFormat, encCfg.m_CTUSize );
  m_numBVs                       = 0;
  m_pcEncCfg                     = &encCfg;
  m_pcTrQuant                    = pTrQuant;
  m_pcRdCost                     = pRdCost;
  m_modeCtrl                     = pModeCtrl;
  m_pSaveCS                      = pSaveCS;

  m_iSearchRange                    = encCfg.m_SearchRange;
  m_bipredSearchRange               = encCfg.m_bipredSearchRange;
  m_motionEstimationSearchMethod    = vvencMESearchMethod( encCfg.m_motionEstimationSearchMethod );
  m_motionEstimationSearchMethodSCC = encCfg.m_motionEstimationSearchMethodSCC;

  for( uint32_t iDir = 0; iDir < MAX_NUM_REF_LIST_ADAPT_SR; iDir++ )
  {
    for( uint32_t iRefIdx = 0; iRefIdx < MAX_IDX_ADAPT_SR; iRefIdx++ )
    {
      m_aaiAdaptSR[iDir][iRefIdx] = m_iSearchRange;
    }
  }

  // initialize motion cost
  for( int iNum = 0; iNum < AMVP_MAX_NUM_CANDS + 1; iNum++ )
  {
    for( int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++ )
    {
      if( iIdx < iNum )
      {
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits( iIdx, iNum );
      }
      else
      {
        m_auiMVPIdxCost[iIdx][iNum] = MAX_UINT;
      }
    }
  }

  const ChromaFormat cform = encCfg.m_internChromaFormat;
  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    m_tmpPredStorage[i].create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  }
  m_tmpStorageLCU.create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  m_pTempPel = new Pel[ encCfg.m_CTUSize * encCfg.m_CTUSize ];
  m_tmpAffiStorage.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  m_tmpAffiError = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[0] = new int[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[1] = new int[MAX_CU_SIZE * MAX_CU_SIZE];
}

void InterSearch::destroy()
{
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = nullptr;
  }

  for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    m_tmpPredStorage[i].destroy();
  }
  m_tmpStorageLCU.destroy();
  m_tmpAffiStorage.destroy();
  if (m_tmpAffiError != NULL)
  {
    delete[] m_tmpAffiError;
    m_tmpAffiError = nullptr;
  }
  if (m_tmpAffiDeri[0] != NULL)
  {
    delete[] m_tmpAffiDeri[0];
    m_tmpAffiDeri[0] = nullptr;
  }
  if (m_tmpAffiDeri[1] != NULL)
  {
    delete[] m_tmpAffiDeri[1];
    m_tmpAffiDeri[1] = nullptr;
  }

  m_pSaveCS  = nullptr;
}

void InterSearch::setCtuEncRsrc( CABACWriter* cabacEstimator, CtxCache* ctxCache, ReuseUniMv* pReuseUniMv, BlkUniMvInfoBuffer* pBlkUniMvInfoBuffer, AffineProfList* pAffineProfList, IbcBvCand* pCachedBvs )
{
  m_CABACEstimator     = cabacEstimator;
  m_CtxCache           = ctxCache;
  m_ReuseUniMv         = pReuseUniMv;
  m_BlkUniMvInfoBuffer = pBlkUniMvInfoBuffer;
  m_AffineProfList     = pAffineProfList;
  m_defaultCachedBvs   = pCachedBvs;
}

ReuseUniMv::ReuseUniMv()
{
  const int numPos     = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx = MAX_CU_SIZE_IDX-2;
  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( int y = 0; y < numPos; y++ )
      {
        for( int x = 0; x < numPos; x++ )
        {
          m_reusedUniMVs[ wIdx ][ hIdx ][ x ][ y ] = nullptr;
        }
      }
    }
  }
}

ReuseUniMv::~ReuseUniMv()
{
  resetReusedUniMvs();
}

void ReuseUniMv::resetReusedUniMvs()
{
  const int numPos     = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx = MAX_CU_SIZE_IDX-2;
  for ( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for ( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for ( int y = 0; y < numPos; y++ )
      {
        for ( int x = 0; x < numPos; x++ )
        {
          if ( m_reusedUniMVs[ wIdx ][ hIdx ][ x ][ y ] )
          {
            delete [] m_reusedUniMVs[ wIdx ][ hIdx ][ x ][ y ];
            m_reusedUniMVs[ wIdx ][ hIdx ][ x ][ y ] = nullptr;
          }
        }
      }
    }
  }
}

void InterSearch::loadGlobalUniMvs( const Area& lumaArea, const PreCalcValues& pcv)
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew(lumaArea, pcv, idx1, idx2, idx3, idx4);
  if( m_ReuseUniMv->m_reusedUniMVs[idx1][idx2][idx3][idx4])
  {
//    DTRACE( g_trace_ctx, D_TMP, "%d unimv load %d %d %d %d \n", g_trace_ctx->getChannelCounter(D_TMP), idx3,idx4,idx1,idx2 );
    m_BlkUniMvInfoBuffer->insertUniMvCands(lumaArea, m_ReuseUniMv->m_reusedUniMVs[idx1][idx2][idx3][idx4]);
  }
}

void InterSearch::getBestSbt( CodingStructure* tempCS, CodingUnit* cu, uint8_t& histBestSbt, Distortion& curPuSse, uint8_t sbtAllowed, bool doPreAnalyzeResi, bool mtsAllowed )
{
  m_estMinDistSbt[NUMBER_SBT_MODE] = MAX_DISTORTION;
  m_skipSbtAll = false;

  if( doPreAnalyzeResi )
  {
    xCalcMinDistSbt( *tempCS, *cu, sbtAllowed );
  }

  curPuSse = getEstDistSbt( NUMBER_SBT_MODE );

  if( doPreAnalyzeResi )
  {
    if( m_skipSbtAll && !mtsAllowed )
    {
      histBestSbt = 0; //try DCT2
    }
    else
    {
      int  slShift = 4 + std::min( Log2( cu->lwidth() * cu->lheight() ), 9 );
      assert( curPuSse != MAX_DISTORTION );
      histBestSbt = m_modeCtrl->findBestSbt( cu->cs->area, (uint32_t)( curPuSse >> slShift ) );
      if( m_skipSbtAll && CU::isSbtMode( histBestSbt ) ) //special case, skip SBT when loading SBT
      {
        histBestSbt = 0; //try DCT2
      }
    }
  }
}


inline void InterSearch::xTZSearchHelp( TZSearchStruct& rcStruct, const int iSearchX, const int iSearchY, const uint8_t ucPointNr, const uint32_t uiDistance )
{
  Distortion  uiSad = 0;

  const Pel* const  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iRefStride + iSearchX;

  m_cDistParam.cur.buf = piRefSrch;

  if( 1 == rcStruct.subShiftMode )
  {
    // motion cost
    Distortion uiBitCost = m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY, rcStruct.imvShift );

    // Skip search if bit cost is already larger than best SAD
    if (uiBitCost < rcStruct.uiBestSad)
    {
      Distortion uiTempSad = m_cDistParam.distFunc( m_cDistParam );

      if((uiTempSad + uiBitCost) < rcStruct.uiBestSad)
      {
        // it's not supposed that any member of DistParams is manipulated beside cur.buf
        int subShift = m_cDistParam.subShift;
        const Pel* pOrgCpy = m_cDistParam.org.buf;
        uiSad += uiTempSad >> m_cDistParam.subShift;

        while( m_cDistParam.subShift > 0 )
        {
          int isubShift           = m_cDistParam.subShift -1;
          m_cDistParam.org.buf = rcStruct.pcPatternKey->buf + (rcStruct.pcPatternKey->stride << isubShift);
          m_cDistParam.cur.buf = piRefSrch + (rcStruct.iRefStride << isubShift);
          uiTempSad            = m_cDistParam.distFunc( m_cDistParam );
          uiSad               += uiTempSad >> m_cDistParam.subShift;

          if(((uiSad << isubShift) + uiBitCost) > rcStruct.uiBestSad)
          {
            break;
          }

          m_cDistParam.subShift--;
        }

        if(m_cDistParam.subShift == 0)
        {
          uiSad += uiBitCost;

          if( uiSad < rcStruct.uiBestSad )
          {
            rcStruct.uiBestSad      = uiSad;
            rcStruct.iBestX         = iSearchX;
            rcStruct.iBestY         = iSearchY;
            rcStruct.uiBestDistance = uiDistance;
            rcStruct.uiBestRound    = 0;
            rcStruct.ucPointNr      = ucPointNr;
            m_cDistParam.maximumDistortionForEarlyExit = uiSad;
          }
        }

        // restore org ptr
        m_cDistParam.org.buf  = pOrgCpy;
        m_cDistParam.subShift = subShift;
      }
    }
  }
  else
  {
    uiSad = m_cDistParam.distFunc( m_cDistParam );

    // only add motion cost if uiSad is smaller than best. Otherwise pointless
    // to add motion cost.
    if( uiSad < rcStruct.uiBestSad )
    {
      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY, rcStruct.imvShift );

      if( uiSad < rcStruct.uiBestSad )
      {
        rcStruct.uiBestSad      = uiSad;
        rcStruct.iBestX         = iSearchX;
        rcStruct.iBestY         = iSearchY;
        rcStruct.uiBestDistance = uiDistance;
        rcStruct.uiBestRound    = 0;
        rcStruct.ucPointNr      = ucPointNr;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
  }
}



inline void InterSearch::xTZ2PointSearch( TZSearchStruct& rcStruct )
{
  const SearchRange& sr = rcStruct.searchRange;

  static const int xOffset[2][9] = { {  0, -1, -1,  0, -1, +1, -1, -1, +1 }, {  0,  0, +1, +1, -1, +1,  0, +1,  0 } };
  static const int yOffset[2][9] = { {  0,  0, -1, -1, +1, -1,  0, +1,  0 }, {  0, -1, -1,  0, -1, +1, +1, +1, +1 } };

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  const int iX1 = rcStruct.iBestX + xOffset[0][rcStruct.ucPointNr];
  const int iX2 = rcStruct.iBestX + xOffset[1][rcStruct.ucPointNr];

  const int iY1 = rcStruct.iBestY + yOffset[0][rcStruct.ucPointNr];
  const int iY2 = rcStruct.iBestY + yOffset[1][rcStruct.ucPointNr];

  if( iX1 >= sr.left && iX1 <= sr.right && iY1 >= sr.top && iY1 <= sr.bottom )
  {
    xTZSearchHelp( rcStruct, iX1, iY1, 0, 2 );
  }

  if( iX2 >= sr.left && iX2 <= sr.right && iY2 >= sr.top && iY2 <= sr.bottom )
  {
    xTZSearchHelp( rcStruct, iX2, iY2, 0, 2 );
  }
}

inline void InterSearch::xTZ4PointSquareSearch( TZSearchStruct & rcStruct, const int iStartX, const int iStartY, const int iDist )
{
  const SearchRange& sr = rcStruct.searchRange;
  CHECK( iDist == 0 || iDist > 2, "Invalid distance" );
  // 4 point search,                   //     1 2 3
  // search around the start point     //     4 0 5
  // with the required  distance       //     6 7 8
  const int iTop = iStartY - iDist;
  const int iBottom = iStartY + iDist;
  const int iLeft = iStartX - iDist;
  const int iRight = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iTop >= sr.top )
  {
    if ( iLeft >= sr.left ) // check top left
    {
      xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
    }
    if ( iRight <= sr.right ) // check top right
    {
      xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
    }
  }
  if ( iBottom <= sr.bottom )
  {
    if ( iLeft >= sr.left ) // check bottom left
    {
      xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
    }
    if ( iRight <= sr.right ) // check bottom right
    {
      xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
    }
  }
}

inline void InterSearch::xTZ8PointSquareSearch( TZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist )
{
  const SearchRange& sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK( iDist == 0 , "Invalid distance");
  const int iTop        = iStartY - iDist;
  const int iBottom     = iStartY + iDist;
  const int iLeft       = iStartX - iDist;
  const int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iTop >= sr.top ) // check top
  {
    if ( iLeft >= sr.left ) // check top left
    {
      xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );

    if ( iRight <= sr.right ) // check top right
    {
      xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= sr.left ) // check middle left
  {
    xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= sr.right ) // check middle right
  {
    xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= sr.bottom ) // check bottom
  {
    if ( iLeft >= sr.left ) // check bottom left
    {
      xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );

    if ( iRight <= sr.right ) // check bottom right
    {
      xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}

inline void InterSearch::xTZ8PointDiamondSearch( TZSearchStruct& rcStruct,
                                                 const int iStartX,
                                                 const int iStartY,
                                                 const int iDist,
                                                 const bool bCheckCornersAtDist1 )
{
  const SearchRange& sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK( iDist == 0, "Invalid distance" );
  const int iTop        = iStartY - iDist;
  const int iBottom     = iStartY + iDist;
  const int iLeft       = iStartX - iDist;
  const int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iDist == 1 )
  {
    if ( iTop >= sr.top ) // check top
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
        }
        xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
        if ( iRight <= sr.right ) // check middle right
        {
          xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
        }
      }
      else
      {
        xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
      }
    }
    if ( iLeft >= sr.left ) // check middle left
    {
      xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= sr.right ) // check middle right
    {
      xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= sr.bottom ) // check bottom
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
        }
        xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
        if ( iRight <= sr.right ) // check middle right
        {
          xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
        }
      }
      else
      {
        xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
      }
    }
  }
  else
  {
    if ( iDist <= 8 )
    {
      const int iTop_2      = iStartY - (iDist>>1);
      const int iBottom_2   = iStartY + (iDist>>1);
      const int iLeft_2     = iStartX - (iDist>>1);
      const int iRight_2    = iStartX + (iDist>>1);

      if (  iTop >= sr.top && iLeft >= sr.left &&
           iRight <= sr.right && iBottom <= sr.bottom ) // check border
      {
        xTZSearchHelp( rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        xTZSearchHelp( rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= sr.top ) // check top
        {
          xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= sr.top ) // check half top
        {
          if ( iLeft_2 >= sr.left ) // check half left
          {
            xTZSearchHelp( rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= sr.right ) // check half right
          {
            xTZSearchHelp( rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= sr.left ) // check left
        {
          xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= sr.right ) // check right
        {
          xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= sr.bottom ) // check half bottom
        {
          if ( iLeft_2 >= sr.left ) // check half left
          {
            xTZSearchHelp( rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= sr.right ) // check half right
          {
            xTZSearchHelp( rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= sr.bottom ) // check bottom
        {
          xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= sr.top && iLeft >= sr.left &&
           iRight <= sr.right && iBottom <= sr.bottom ) // check border
      {
        xTZSearchHelp( rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( rcStruct, iStartX, iBottom, 0, iDist );
        for ( int index = 1; index < 4; index++ )
        {
          const int iPosYT = iTop    + ((iDist>>2) * index);
          const int iPosYB = iBottom - ((iDist>>2) * index);
          const int iPosXL = iStartX - ((iDist>>2) * index);
          const int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= sr.top ) // check top
        {
          xTZSearchHelp( rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= sr.left ) // check left
        {
          xTZSearchHelp( rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= sr.right ) // check right
        {
          xTZSearchHelp( rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= sr.bottom ) // check bottom
        {
          xTZSearchHelp( rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( int index = 1; index < 4; index++ )
        {
          const int iPosYT = iTop    + ((iDist>>2) * index);
          const int iPosYB = iBottom - ((iDist>>2) * index);
          const int iPosXL = iStartX - ((iDist>>2) * index);
          const int iPosXR = iStartX + ((iDist>>2) * index);

          if ( iPosYT >= sr.top ) // check top
          {
            if ( iPosXL >= sr.left ) // check left
            {
              xTZSearchHelp( rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= sr.right ) // check right
            {
              xTZSearchHelp( rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= sr.bottom ) // check bottom
          {
            if ( iPosXL >= sr.left ) // check left
            {
              xTZSearchHelp( rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= sr.right ) // check right
            {
              xTZSearchHelp( rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}

Distortion InterSearch::xPatternRefinement( const CPelBuf* pcPatternKey,
                                            Mv baseRefMv,
                                            int iFrac, Mv& rcMvFrac,
                                            bool bAllowUseOfHadamard,
                                            Distortion& uiDistBest,
                                            int& patternId,
                                            CPelBuf* pattern,
                                            bool useAltHpelIf )
{
  Distortion  uiDist;
  uiDistBest = m_pcEncCfg->m_fastSubPel ? uiDistBest : MAX_DISTORTION;
  uint32_t        uiDirecBest = 0;

  Pel*  piRefPos;
  int iRefStride = pcPatternKey->width + 1;
  m_pcRdCost->setDistParam( m_cDistParam, *pcPatternKey, m_filteredBlock[0][0][0], iRefStride, m_lumaClpRng.bd, COMP_Y, 0, m_pcEncCfg->m_bUseHADME && bAllowUseOfHadamard );

  const ClpRng& clpRng = m_lumaClpRng;
  int width = pattern->width;
  int height = pattern->height;
  int srcStride = pattern->stride;

  int intStride = width + 1;
  int dstStride = width + 1;
  Pel* intPtr;
  Pel* dstPtr;
  int filterSize = NTAPS_LUMA;
  int halfFilterSize = ( filterSize >> 1 );
  const Pel* srcPtr = pattern->buf - halfFilterSize*srcStride - 1;

  const ChromaFormat chFmt = m_currChromaFormat;

  Distortion distH[ 9 ] = { uiDistBest, uiDistBest, uiDistBest, uiDistBest, uiDistBest, uiDistBest, uiDistBest, uiDistBest, uiDistBest };
  const int TH = 17, TL = 15, shift = 4;

  const Mv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  for (uint32_t i = 0; i < 9; i++)
  {
    if( m_pcEncCfg->m_fastSubPel )
    {
      if( s_skipQpelPosition[ patternId ][ i ] )
      {
        continue;
      }

      if( 2 == iFrac )
      {
        if ( ( 5 == i && 0 == uiDirecBest ) || ( 7 == i && 1 == uiDirecBest ) || ( 8 == i && ( 1 == uiDirecBest || 3 == uiDirecBest || 5 == uiDirecBest ) ) )
        {
          break;
        }

        if( 0 == i )
        {
          // split the prediction with funny widths into power-of-2 and +1 parts for the sake of SIMD speed-up
          m_if.filterHor( COMP_Y, srcPtr, srcStride, m_filteredBlockTmp[ 0 ][ 0 ], intStride, width, height + filterSize, 0 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, useAltHpelIf );
          m_if.filterHor( COMP_Y, srcPtr + width, srcStride, m_filteredBlockTmp[ 0 ][ 0 ] + width, intStride, 1, height + filterSize, 0 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, useAltHpelIf );

          // split the prediction with funny widths into power-of-2 and +1 parts for the sake of SIMD speed-up
          m_if.filterHor( COMP_Y, srcPtr, srcStride, m_filteredBlockTmp[ 2 ][ 0 ], intStride, width, height + filterSize, 2 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, useAltHpelIf );
          m_if.filterHor( COMP_Y, srcPtr + width, srcStride, m_filteredBlockTmp[ 2 ][ 0 ] + width, intStride, 1, height + filterSize, 2 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, useAltHpelIf );

          intPtr = m_filteredBlockTmp[ 0 ][ 0 ] + halfFilterSize * intStride + 1;
          dstPtr = m_filteredBlock[ 0 ][ 0 ][ 0 ];
          m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
        }
        else if( 1 == i )
        {
          intPtr = m_filteredBlockTmp[ 0 ][ 0 ] + ( halfFilterSize - 1 ) * intStride + 1;
          dstPtr = m_filteredBlock[ 2 ][ 0 ][ 0 ];
          m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
        }
        else if( 3 == i )
        {
          intPtr = m_filteredBlockTmp[ 2 ][ 0 ] + halfFilterSize * intStride;
          dstPtr = m_filteredBlock[ 0 ][ 2 ][ 0 ];
          // split the prediction with funny widths into power-of-2 and +1 parts for the sake of SIMD speed-up
          m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
          m_if.filterVer( COMP_Y, intPtr + width, intStride, dstPtr + width, dstStride, 1, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
        }
        else if( 5 == i )
        {
          intPtr = m_filteredBlockTmp[ 2 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
          dstPtr = m_filteredBlock[ 2 ][ 2 ][ 0 ];
          // split the prediction with funny widths into power-of-2 and +1 parts for the sake of SIMD speed-up
          m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
          m_if.filterVer( COMP_Y, intPtr + width, intStride, dstPtr + width, dstStride, 1, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
        }
      }
    }
    Mv cMvTest = pcMvRefine[ i ];
    cMvTest += baseRefMv;

    int horVal = cMvTest.hor * iFrac;
    int verVal = cMvTest.ver * iFrac;
    piRefPos = m_filteredBlock[verVal & 3][horVal & 3][0];

    if ( horVal == 2 && ( verVal & 1 ) == 0 )
    {
      piRefPos += 1;
    }
    if ( ( horVal & 1 ) == 0 && verVal == 2 )
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;


    m_cDistParam.cur.buf   = piRefPos;
    uiDist = m_cDistParam.distFunc( m_cDistParam );
    uiDist += m_pcRdCost->getCostOfVectorWithPredictor( cMvTest.hor, cMvTest.ver, 0 );

    distH[ i ] = uiDist;
    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
      m_cDistParam.maximumDistortionForEarlyExit = uiDist;
    }
  }

  rcMvFrac = pcMvRefine[uiDirecBest];

  if( m_pcEncCfg->m_fastSubPel && 2 == iFrac )
  {
    switch ( uiDirecBest )
    {
    case 0:
      // hor
      distH[ 3 ] <<= shift;
      patternId += ( distH[ 3 ] > TH * distH[ 4 ] ? 2 : ( distH[ 3 ] < TL * distH[ 4 ] ? 1 : 0 ) );
      // ver
      distH[ 1 ] <<= shift;
      patternId += ( distH[ 1 ] > TH * distH[ 2 ] ? 6 : ( distH[ 1 ] < TL * distH[ 2 ] ? 3 : 0 ) );
      break;
    case 1:
      // hor
      distH[ 5 ] <<= shift;
      patternId += ( distH[ 5 ] > TH * distH[ 6 ] ? 4 : ( distH[ 5 ] < TL * distH[ 6 ] ? 2 : 0 ) );
      // ver
      patternId += ( distH[ 2 ] - distH[ 0 ] > distH[ 0 ] - distH[ 1 ] ? 1 : 0 );

      patternId += ( 41 == patternId ? 0 : 8 );
      break;
    case 2:
      // hor
      distH[ 7 ] <<= shift;
      patternId += ( distH[ 7 ] > TH * distH[ 8 ] ? 4 : ( distH[ 7 ] < TL * distH[ 8 ] ? 2 : 0 ) );
      // ver
      patternId += ( distH[ 1 ] - distH[ 0 ] > distH[ 0 ] - distH[ 2 ] ? 1 : 0 );

      patternId += ( 41 == patternId ? 0 : 13 );
      break;
    case 3:
      // hor
      patternId += ( distH[ 4 ] - distH[ 0 ] > distH[ 0 ] - distH[ 3 ] ? 1 : 0 );
      // ver
      distH[ 5 ] <<= shift;
      patternId += ( distH[ 5 ] > TH * distH[ 7 ] ? 4 : ( distH[ 5 ] < TL * distH[ 7 ] ? 2 : 0 ) );

      patternId += ( 41 == patternId ? 0 : 18 );
      break;
    case 4:
      // hor
      patternId += ( distH[ 3 ] - distH[ 0 ] > distH[ 0 ] - distH[ 4 ] ? 1 : 0 );
      // ver
      distH[ 6 ] <<= shift;
      patternId += ( distH[ 6 ] > TH * distH[ 8 ] ? 4 : ( distH[ 6 ] < TL * distH[ 8 ] ? 2 : 0 ) );

      patternId += ( 41 == patternId ? 0 : 23 );
      break;
    case 5:
      // hor
      patternId += ( distH[ 6 ] - distH[ 1 ] > distH[ 1 ] - distH[ 5 ] ? 1 : 0 );
      // ver
      patternId += ( distH[ 7 ] - distH[ 3 ] > distH[ 3 ] - distH[ 5 ] ? 2 : 0 );

      patternId += ( 41 == patternId ? 0 : 28 );
      break;
    case 6:
      // hor
      patternId += ( distH[ 5 ] - distH[ 1 ] > distH[ 1 ] - distH[ 6 ] ? 1 : 0 );
      // ver
      patternId += ( distH[ 8 ] - distH[ 4 ] > distH[ 4 ] - distH[ 6 ] ? 2 : 0 );

      patternId += ( 41 == patternId ? 0 : 31 );
      break;
    case 7:
      // hor
      patternId += ( distH[ 8 ] - distH[ 2 ] > distH[ 2 ] - distH[ 7 ] ? 1 : 0 );
      // ver
      patternId += ( distH[ 5 ] - distH[ 3 ] > distH[ 3 ] - distH[ 7 ] ? 2 : 0 );

      patternId += ( 41 == patternId ? 0 : 34 );
      break;
    case 8:
      // hor
      patternId += ( distH[ 7 ] - distH[ 2 ] > distH[ 2 ] - distH[ 8 ] ? 1 : 0 );
      // ver
      patternId += ( distH[ 6 ] - distH[ 4 ] > distH[ 4 ] - distH[ 8 ] ? 2 : 0 );

      patternId += ( 41 == patternId ? 0 : 37 );
      break;
    default:
      break;
    }
  }

  return uiDistBest;
}

//! search of the best candidate for inter prediction
#if USE_COST_BEFORE
bool InterSearch::predInterSearch(CodingUnit& cu, Partitioner& partitioner, double& bestCostInter)
#else
void InterSearch::predInterSearch(CodingUnit& cu, Partitioner& partitioner)
#endif
{
  CodingStructure& cs = *cu.cs;

  AMVPInfo     amvp[2];
  Mv           cMvSrchRngLT;
  Mv           cMvSrchRngRB;
  Mv           cMvZero;
  Mv           cMv[2];
  Mv           cMvBi[2];
  Mv           cMvTemp[2][MAX_REF_PICS];
  Mv           cMvHevcTemp[2][MAX_REF_PICS];
  int          iNumPredDir = cs.slice->isInterP() ? 1 : 2;

  Mv           cMvPred[2][MAX_REF_PICS];

  Mv           cMvPredBi[2][MAX_REF_PICS];
  int          aaiMvpIdxBi[2][MAX_REF_PICS];

  int          aaiMvpIdx[2][MAX_REF_PICS];
  int          aaiMvpNum[2][MAX_REF_PICS];

  AMVPInfo     aacAMVPInfo[2][MAX_REF_PICS];

  int          iRefIdx[2]={0,0}; //If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  int          iRefIdxBi[2] = { -1, -1 };

  uint32_t     uiMbBits[3] = {1, 1, 0};

  uint32_t     uiLastMode = 0;
  int          iRefStart, iRefEnd;

  int          symMode = 0;

  int          bestBiPRefIdxL1 = 0;
  int          bestBiPMvpL1    = 0;
  Distortion   biPDistTemp     = MAX_DISTORTION;

  uint8_t      BcwIdx          = (cu.cs->slice->isInterB() ? cu.BcwIdx : BCW_DEFAULT);
  bool         enforceBcwPred = false;
  MergeCtx     mergeCtx;

  // Loop over Prediction Units
  uint32_t     puIdx = 0;
  uint32_t     uiLastModeTemp = 0;
  Distortion   uiAffineCost = MAX_DISTORTION;
  Distortion   uiHevcCost = MAX_DISTORTION;
  bool checkAffine = (cu.imv == 0);
  if (cu.cs->bestParent != nullptr && cu.cs->bestParent->getCU(CH_L,TREE_D) != nullptr && cu.cs->bestParent->getCU(CH_L,TREE_D)->affine == false)
  {
    m_skipPROF = true;
  }

  m_encOnly = true;
  {
    if (cu.cs->sps->SbtMvp)
    {
      Size bufSize = g_miScaling.scale(cu.lumaSize());
      mergeCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
    }
    CU::spanMotionInfo( cu );
    Distortion   uiCost[2] = { MAX_DISTORTION, MAX_DISTORTION };
    Distortion   uiCostBi  =   MAX_DISTORTION;
    Distortion   uiCostTemp;

    uint32_t         uiBits[3];
    uint32_t         uiBitsTemp;
    Distortion   bestBiPDist = MAX_DISTORTION;

    Distortion   uiCostTempL0[MAX_NUM_REF];
    for (int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
    {
      uiCostTempL0[iNumRef] = MAX_DISTORTION;
    }
    uint32_t         uiBitsTempL0[MAX_NUM_REF];

    Mv           mvValidList1;
    int          refIdxValidList1 = 0;
    uint32_t         bitsValidList1   = MAX_UINT;
    Distortion   costValidList1   = MAX_DISTORTION;

    CPelUnitBuf origBuf = cu.cs->getOrgBuf( cu );

    xGetBlkBits( cs.slice->isInterP(), puIdx, uiLastMode, uiMbBits );

    m_pcRdCost->selectMotionLambda();

    unsigned imvShift = cu.imv == IMV_HPEL ? 1 : (cu.imv << 1);

      //  Uni-directional prediction
      for ( int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
      {
        RefPicList  refPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
        for (int iRefIdxTemp = 0; iRefIdxTemp < cs.slice->numRefIdx[ refPicList ]; iRefIdxTemp++)
        {
          uiBitsTemp = uiMbBits[iRefList];
          if ( cs.slice->numRefIdx[ refPicList ] > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == cs.slice->numRefIdx[ refPicList ]-1 )
            {
              uiBitsTemp--;
            }
          }
          xEstimateMvPredAMVP( cu, origBuf, refPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], amvp[refPicList], biPDistTemp);

          aaiMvpIdx[iRefList][iRefIdxTemp] = cu.mvpIdx[refPicList];
          aaiMvpNum[iRefList][iRefIdxTemp] = cu.mvpNum[refPicList];

          if(cs.picHeader->mvdL1Zero && iRefList==1 && biPDistTemp < bestBiPDist)
          {
            bestBiPDist = biPDistTemp;
            bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
            bestBiPRefIdxL1 = iRefIdxTemp;
          }

          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

          if ( m_pcEncCfg->m_bFastMEForGenBLowDelayEnabled && iRefList == 1 )    // list 1
          {
            if ( cs.slice->list1IdxToList0Idx[ iRefIdxTemp ] >= 0 )
            {
              cMvTemp[1][iRefIdxTemp] = cMvTemp[0][cs.slice->list1IdxToList0Idx[iRefIdxTemp ]];
              uiCostTemp = uiCostTempL0[cs.slice->list1IdxToList0Idx[ iRefIdxTemp ]];
              /*first subtract the bit-rate part of the cost of the other list*/
              uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[cs.slice->list1IdxToList0Idx[ iRefIdxTemp ]] );
              /*correct the bit-rate part of the current ref*/
              m_pcRdCost->setPredictor  ( cMvPred[iRefList][iRefIdxTemp] );
              uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( cMvTemp[1][iRefIdxTemp].hor, cMvTemp[1][iRefIdxTemp].ver, imvShift + MV_FRACTIONAL_BITS_DIFF );
              /*calculate the correct cost*/
              uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
            }
            else
            {
              xMotionEstimation( cu, origBuf, refPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[refPicList] );
            }
          }
          else
          {
            xMotionEstimation( cu, origBuf, refPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[refPicList] );
          }
          xCopyAMVPInfo( &amvp[refPicList], &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
          xCheckBestMVP( refPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], amvp[refPicList], uiBitsTemp, uiCostTemp, cu.imv );

          if ( iRefList == 0 )
          {
            uiCostTempL0[iRefIdxTemp] = uiCostTemp;
            uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
          }
          if ( uiCostTemp < uiCost[iRefList] )
          {
            uiCost[iRefList] = uiCostTemp;
            uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

            // set motion
            cMv    [iRefList] = cMvTemp[iRefList][iRefIdxTemp];
            iRefIdx[iRefList] = iRefIdxTemp;
          }

          if ( iRefList == 1 && uiCostTemp < costValidList1 && cs.slice->list1IdxToList0Idx[ iRefIdxTemp ] < 0 )
          {
            costValidList1 = uiCostTemp;
            bitsValidList1 = uiBitsTemp;

            // set motion
            mvValidList1     = cMvTemp[iRefList][iRefIdxTemp];
            refIdxValidList1 = iRefIdxTemp;
          }
        }
      }

      ::memcpy(cMvHevcTemp, cMvTemp, sizeof(cMvTemp));
      if (cu.imv == 0 && (!cu.slice->sps->BCW || BcwIdx == BCW_DEFAULT))
      {
        m_BlkUniMvInfoBuffer->insertUniMvCands(cu.Y(), &cMvTemp[0][0]);

        unsigned idx1, idx2, idx3, idx4;
        getAreaIdxNew(cu.Y(), *cs.pcv, idx1, idx2, idx3, idx4);
        if( ! m_ReuseUniMv->m_reusedUniMVs[idx1][idx2][idx3][idx4] )
        {
          m_ReuseUniMv->m_reusedUniMVs[idx1][idx2][idx3][idx4] = new Mv[ 2 * MAX_REF_PICS ];
//          DTRACE( g_trace_ctx, D_TMP, "%d unimv first reuse %d %d %d %d \n", g_trace_ctx->getChannelCounter(D_TMP), idx3,idx4,idx1,idx2 );
        }
        ::memcpy(m_ReuseUniMv->m_reusedUniMVs[idx1][idx2][idx3][idx4], cMvTemp, 2 * MAX_REF_PICS * sizeof(Mv));
      }
#if USE_COST_BEFORE  
      if (bestCostInter != MAX_DOUBLE)
      {
        int L = (cu.slice->TLayer <= 2) ? 0 : (cu.slice->TLayer - 2);
        double besCostMerge = bestCostInter;
        bestCostInter = (uiCost[0] < uiCost[1]) ? uiCost[0] : uiCost[1];
        if ((cu.slice->TLayer > (log2(m_pcEncCfg->m_GOPSize) - (m_pcEncCfg->m_FastInferMerge & 7))) && bestCostInter > MRG_FAST_RATIOMYV[L] * besCostMerge)
        {
          m_skipPROF = false;
          m_encOnly = false;
          return true;
        }
      }
#endif
      //  Bi-predictive Motion estimation
      if( cs.slice->isInterB() && !CU::isBipredRestriction( cu ) && (cu.slice->checkLDC || BcwIdx == BCW_DEFAULT) )
      {
        bool doBiPred = true;
        cMvBi[0] = cMv[0];
        cMvBi[1] = cMv[1];
        iRefIdxBi[0] = iRefIdx[0];
        iRefIdxBi[1] = iRefIdx[1];

        ::memcpy( cMvPredBi,   cMvPred,   sizeof( cMvPred   ) );
        ::memcpy( aaiMvpIdxBi, aaiMvpIdx, sizeof( aaiMvpIdx ) );

        uint32_t uiMotBits[2];

        if(cs.picHeader->mvdL1Zero)
        {
          xCopyAMVPInfo(&aacAMVPInfo[1][bestBiPRefIdxL1], &amvp[REF_PIC_LIST_1]);
          aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;
          cMvPredBi  [1][bestBiPRefIdxL1] = amvp[REF_PIC_LIST_1].mvCand[bestBiPMvpL1];

          cMvBi    [1] = cMvPredBi[1][bestBiPRefIdxL1];
          iRefIdxBi[1] = bestBiPRefIdxL1;
          cu.mv    [REF_PIC_LIST_1][0] = cMvBi[1];
          cu.refIdx[REF_PIC_LIST_1]    = iRefIdxBi[1];
          cu.mvpIdx[REF_PIC_LIST_1]    = bestBiPMvpL1;

          PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_1].getCompactBuf( cu );
          motionCompensation( cu, predBufTmp, REF_PIC_LIST_1 );

          uiMotBits[0] = uiBits[0] - uiMbBits[0];
          uiMotBits[1] = uiMbBits[1];

          if ( cs.slice->numRefIdx[ REF_PIC_LIST_1 ] > 1 )
          {
            uiMotBits[1] += bestBiPRefIdxL1 + 1;
            if ( bestBiPRefIdxL1 == cs.slice->numRefIdx[ REF_PIC_LIST_1 ]-1 )
            {
              uiMotBits[1]--;
            }
          }

          uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];

          uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];

          cMvTemp[1][bestBiPRefIdxL1] = cMvBi[1];
        }
        else
        {
          uiMotBits[0] = uiBits[0] - uiMbBits[0];
          uiMotBits[1] = uiBits[1] - uiMbBits[1];
          uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
        }

        if( doBiPred )
        {
          // 4-times iteration (default)
          int iNumIter = 4;

          // fast encoder setting: only one iteration
          if ( m_pcEncCfg->m_fastInterSearchMode==VVENC_FASTINTERSEARCH_MODE1 || m_pcEncCfg->m_fastInterSearchMode==VVENC_FASTINTERSEARCH_MODE2 || cs.picHeader->mvdL1Zero )
          {
            iNumIter = 1;
          }

          for ( int iIter = 0; iIter < iNumIter; iIter++ )
          {
            int         iRefList    = iIter % 2;

            if ( m_pcEncCfg->m_fastInterSearchMode==VVENC_FASTINTERSEARCH_MODE1 || m_pcEncCfg->m_fastInterSearchMode==VVENC_FASTINTERSEARCH_MODE2 )
            {
              if( uiCost[0] <= uiCost[1] )
              {
                iRefList = 1;
              }
              else
              {
                iRefList = 0;
              }
            }
            else if ( iIter == 0 )
            {
              iRefList = 0;
            }
            if ( iIter == 0 && !cs.picHeader->mvdL1Zero)
            {
              cu.mv    [1 - iRefList][0] = cMv    [1 - iRefList];
              cu.refIdx[1 - iRefList]    = iRefIdx[1 - iRefList];

              PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getCompactBuf( cu );
              motionCompensation( cu, predBufTmp, RefPicList(1 - iRefList) );
            }

            RefPicList  refPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

            if(cs.picHeader->mvdL1Zero)
            {
              iRefList = 0;
              refPicList = REF_PIC_LIST_0;
            }

            bool bChanged = false;

            iRefStart = 0;
            iRefEnd   = cs.slice->numRefIdx[ refPicList ]-1;
            for (int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++)
            {
              uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
              if ( cs.slice->numRefIdx[ refPicList ] > 1 )
              {
                uiBitsTemp += iRefIdxTemp+1;
                if ( iRefIdxTemp == cs.slice->numRefIdx[ refPicList ]-1 )
                {
                  uiBitsTemp--;
                }
              }
              uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
              if ( cs.slice->biDirPred )
              {
                uiBitsTemp += 1; // add one bit for symmetrical MVD mode
              }
              // call ME
              xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], &amvp[refPicList] );
              xMotionEstimation ( cu, origBuf, refPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[refPicList], true );
              xCheckBestMVP( refPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], amvp[refPicList], uiBitsTemp, uiCostTemp, cu.imv);
              if ( uiCostTemp < uiCostBi )
              {
                bChanged = true;

                cMvBi[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
                iRefIdxBi[iRefList] = iRefIdxTemp;

                uiCostBi            = uiCostTemp;
                uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
                uiBits[2]           = uiBitsTemp;

                if(iNumIter!=1)
                {
                  //  Set motion
                  cu.mv    [refPicList][0] = cMvBi    [iRefList];
                  cu.refIdx[refPicList]    = iRefIdxBi[iRefList];

                  PelUnitBuf predBufTmp = m_tmpPredStorage[iRefList].getCompactBuf( cu );
                  motionCompensation( cu, predBufTmp, refPicList );
                }
              }
            } // for loop-iRefIdxTemp

            if( !bChanged )
            {
              if ((uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1]) || enforceBcwPred)
              {
                xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], &amvp[REF_PIC_LIST_0]);
                xCheckBestMVP( REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], amvp[REF_PIC_LIST_0], uiBits[2], uiCostBi, cu.imv);
                if(!cs.picHeader->mvdL1Zero)
                {
                  xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], &amvp[REF_PIC_LIST_1]);
                  xCheckBestMVP( REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], amvp[REF_PIC_LIST_1], uiBits[2], uiCostBi, cu.imv);
                }
              }
              break;
            }
          } // for loop-iter
        }

        // SMVD
        if( cs.slice->biDirPred )
        {
          double th1 = 1.02;
          bool testSME = true;
          int numStartCand = m_pcEncCfg->m_SMVD > 1 ? 1 : 5;
          Distortion symCost;
          Mv cMvPredSym[2];
          int mvpIdxSym[2];

          int curRefList = REF_PIC_LIST_0;
          int tarRefList = 1 - curRefList;
          RefPicList eCurRefList = (curRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
          int refIdxCur = cs.slice->symRefIdx[ curRefList ];
          int refIdxTar = cs.slice->symRefIdx[ tarRefList ];
          if( aacAMVPInfo[ curRefList ][ refIdxCur ].mvCand[ 0 ] == aacAMVPInfo[ curRefList ][ refIdxCur ].mvCand[ 1 ] )
          {
            aacAMVPInfo[ curRefList ][ refIdxCur ].numCand = 1;
          }
          if( aacAMVPInfo[ tarRefList ][ refIdxTar ].mvCand[ 0 ] == aacAMVPInfo[ tarRefList ][ refIdxTar ].mvCand[ 1 ] )
          {
            aacAMVPInfo[ tarRefList ][ refIdxTar ].numCand = 1;
          }

          MvField cCurMvField, cTarMvField;
          Distortion costStart = MAX_DISTORTION;
          for ( int i = 0; i < aacAMVPInfo[curRefList][refIdxCur].numCand; i++ )
          {
            for ( int j = 0; j < aacAMVPInfo[tarRefList][refIdxTar].numCand; j++ )
            {
              GCC_WARNING_DISABLE_array_bounds // probably a bug in gcc-10 static analyzer: It thinks the indices are -1 and therefore triggers -Werror=array-bounds
              cCurMvField.setMvField( aacAMVPInfo[curRefList][refIdxCur].mvCand[i], refIdxCur );
              cTarMvField.setMvField( aacAMVPInfo[tarRefList][refIdxTar].mvCand[j], refIdxTar );
              GCC_WARNING_RESET
              Distortion cost = xGetSymCost( cu, origBuf, eCurRefList, cCurMvField, cTarMvField, BcwIdx );
              if ( cost < costStart )
              {
                costStart = cost;
                cMvPredSym[curRefList] = aacAMVPInfo[curRefList][refIdxCur].mvCand[i];
                cMvPredSym[tarRefList] = aacAMVPInfo[tarRefList][refIdxTar].mvCand[j];
                mvpIdxSym[curRefList] = i;
                mvpIdxSym[tarRefList] = j;
              }
            }
          }
          cCurMvField.mv = cMvPredSym[curRefList];
          cTarMvField.mv = cMvPredSym[tarRefList];

          m_pcRdCost->setCostScale(0);
          Mv pred = cMvPredSym[curRefList];
          pred.changeTransPrecInternal2Amvr(cu.imv);
          m_pcRdCost->setPredictor(pred);
          Mv mv = cCurMvField.mv;
          mv.changeTransPrecInternal2Amvr(cu.imv);
          uint32_t bits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
          bits += m_auiMVPIdxCost[mvpIdxSym[curRefList]][AMVP_MAX_NUM_CANDS];
          bits += m_auiMVPIdxCost[mvpIdxSym[tarRefList]][AMVP_MAX_NUM_CANDS];
          costStart += m_pcRdCost->getCost(bits);

          std::vector<Mv> symmvdCands;
          auto smmvdCandsGen = [&](Mv mvCand, bool mvPrecAdj)
          {
            if (mvPrecAdj && cu.imv)
            {
              mvCand.roundTransPrecInternal2Amvr(cu.imv);
            }

            bool toAddMvCand = true;
            for (std::vector<Mv>::iterator pos = symmvdCands.begin(); pos != symmvdCands.end(); pos++)
            {
              if (*pos == mvCand)
              {
                toAddMvCand = false;
                break;
              }
            }

            if (toAddMvCand)
            {
              symmvdCands.push_back(mvCand);
            }
          };

          smmvdCandsGen(cMvHevcTemp[curRefList][refIdxCur], false);
          smmvdCandsGen(cMvTemp[curRefList][refIdxCur], false);
          if (iRefIdxBi[curRefList] == refIdxCur)
          {
            smmvdCandsGen(cMvBi[curRefList], false);
          }
          for (int i = 0; i < m_BlkUniMvInfoBuffer->m_uniMvListSize; i++)
          {
            if( symmvdCands.size() >= numStartCand )
            {
              break;
            }
            BlkUniMvInfo* curMvInfo = m_BlkUniMvInfoBuffer->getBlkUniMvInfo(i);
            smmvdCandsGen(curMvInfo->uniMvs[curRefList][refIdxCur], true);
          }

          for (auto mvStart : symmvdCands)
          {
            bool checked = false; //if it has been checkin in the mvPred.
            for (int i = 0; i < aacAMVPInfo[curRefList][refIdxCur].numCand && !checked; i++)
            {
              checked |= (mvStart == aacAMVPInfo[curRefList][refIdxCur].mvCand[i]);
            }
            if (checked)
            {
              continue;
            }

            Distortion bestCost = costStart;
            xSymMvdCheckBestMvp(cu, origBuf, mvStart, (RefPicList)curRefList, aacAMVPInfo, BcwIdx, cMvPredSym, mvpIdxSym, costStart, false);
            if (costStart < bestCost)
            {
              cCurMvField.setMvField(mvStart, refIdxCur);
              cTarMvField.setMvField(mvStart.getSymmvdMv(cMvPredSym[curRefList], cMvPredSym[tarRefList]), refIdxTar);
            }
          }
          Mv startPtMv = cCurMvField.mv;

          Distortion mvpCost = m_pcRdCost->getCost(m_auiMVPIdxCost[mvpIdxSym[curRefList]][AMVP_MAX_NUM_CANDS] + m_auiMVPIdxCost[mvpIdxSym[tarRefList]][AMVP_MAX_NUM_CANDS]);
          symCost = costStart - mvpCost;

          // ME
          testSME = m_pcEncCfg->m_SMVD <= 2 || ( symCost < uiCostBi * th1 && uiCostBi < uiCost[ 0 ] && uiCostBi < uiCost[ 1 ] );
          if( testSME )
          {
            xSymMotionEstimation( cu, origBuf, cMvPredSym[ curRefList ], cMvPredSym[ tarRefList ], eCurRefList, cCurMvField, cTarMvField, symCost, BcwIdx );
          }

          symCost += mvpCost;

          if (startPtMv != cCurMvField.mv)
          { // if ME change MV, run a final check for best MVP.
            xSymMvdCheckBestMvp(cu, origBuf, cCurMvField.mv, (RefPicList)curRefList, aacAMVPInfo, BcwIdx, cMvPredSym, mvpIdxSym, symCost, true);
          }

          bits = uiMbBits[2];
          bits += 1; // add one bit for #symmetrical MVD mode
          symCost += m_pcRdCost->getCost(bits);
          cTarMvField.setMvField(cCurMvField.mv.getSymmvdMv(cMvPredSym[curRefList], cMvPredSym[tarRefList]), refIdxTar);

          // save results
          if ( symCost < uiCostBi )
          {
            uiCostBi = symCost;
            symMode = 1 + curRefList;

            cMvBi[curRefList] = cCurMvField.mv;
            iRefIdxBi[curRefList] = cCurMvField.refIdx;
            aaiMvpIdxBi[curRefList][cCurMvField.refIdx] = mvpIdxSym[curRefList];
            cMvPredBi[curRefList][iRefIdxBi[curRefList]] = cMvPredSym[curRefList];

            cMvBi[tarRefList] = cTarMvField.mv;
            iRefIdxBi[tarRefList] = cTarMvField.refIdx;
            aaiMvpIdxBi[tarRefList][cTarMvField.refIdx] = mvpIdxSym[tarRefList];
            cMvPredBi[tarRefList][iRefIdxBi[tarRefList]] = cMvPredSym[tarRefList];
          }
        }
      } // if (B_SLICE)

        //  Clear Motion Field
      cu.mv [REF_PIC_LIST_0][0] = Mv();
      cu.mv [REF_PIC_LIST_1][0] = Mv();
      cu.mvd[REF_PIC_LIST_0][0] = cMvZero;
      cu.mvd[REF_PIC_LIST_1][0] = cMvZero;
      cu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
      cu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
      cu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
      cu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
      cu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
      cu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;

      // Set Motion Field
      cMv    [1] = mvValidList1;
      iRefIdx[1] = refIdxValidList1;
      uiBits [1] = bitsValidList1;
      uiCost [1] = costValidList1;
      if( enforceBcwPred )
      {
        uiCost[0] = uiCost[1] = MAX_UINT;
      }

      uiLastModeTemp = uiLastMode;
      if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
      {
#if USE_COST_BEFORE
        bestCostInter = uiCostBi;
#endif
        uiLastMode = 2;
        cu.mv [REF_PIC_LIST_0][0] = cMvBi[0];
        cu.mv [REF_PIC_LIST_1][0] = cMvBi[1];
        cu.mvd[REF_PIC_LIST_0][0] = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
        cu.mvd[REF_PIC_LIST_1][0] = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
        cu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
        cu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
        cu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
        cu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
        cu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
        cu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
        cu.interDir = 3;

        cu.smvdMode = symMode;
      }
      else if ( uiCost[0] <= uiCost[1] )
      {
#if USE_COST_BEFORE
        bestCostInter = uiCost[0];
#endif
        uiLastMode = 0;
        cu.mv [REF_PIC_LIST_0][0] = cMv[0];
        cu.mvd[REF_PIC_LIST_0][0] = cMv[0] - cMvPred[0][iRefIdx[0]];
        cu.refIdx[REF_PIC_LIST_0] = iRefIdx[0];
        cu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][iRefIdx[0]];
        cu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdx[0]];
        cu.interDir = 1;
      }
      else
      {
#if USE_COST_BEFORE
        bestCostInter = uiCost[1];
#endif
        uiLastMode = 1;
        cu.mv [REF_PIC_LIST_1][0] = cMv[1];
        cu.mvd[REF_PIC_LIST_1][0] = cMv[1] - cMvPred[1][iRefIdx[1]];
        cu.refIdx[REF_PIC_LIST_1] = iRefIdx[1];
        cu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][iRefIdx[1]];
        cu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdx[1]];
        cu.interDir = 2;
      }

      if( BcwIdx != BCW_DEFAULT )
      {
        cu.BcwIdx = BCW_DEFAULT; // Reset to default for the Non-NormalMC modes.
      }
      uiHevcCost = (uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1]) ? uiCostBi : ((uiCost[0] <= uiCost[1]) ? uiCost[0] : uiCost[1]);

    if( cu.interDir == 3 && !cu.mergeFlag )
    {
      if (BcwIdx != BCW_DEFAULT)
      {
        cu.BcwIdx = BcwIdx;
      }
    }

    if (m_pcEncCfg->m_Affine > 1)
    {
      checkAffine = m_modeCtrl->comprCUCtx->bestCU ? (checkAffine && m_modeCtrl->comprCUCtx->bestCU->affine) : checkAffine;
      if (cu.slice->TLayer > 3)
      {
        checkAffine = false;
      }
    }
    if (cu.Y().width > 8 && cu.Y().height > 8 && cu.slice->sps->Affine && checkAffine)
    {
      PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_INTER_MVD_SEARCH_AFFINE, &cs, partitioner.chType );
      m_hevcCost = uiHevcCost;
      // save normal hevc result
      uint32_t uiMRGIndex = cu.mergeIdx;
      bool bMergeFlag = cu.mergeFlag;
      uint32_t uiInterDir = cu.interDir;
      int  iSymMode = cu.smvdMode;

      Mv cMvd[2];
      uint32_t uiMvpIdx[2], uiMvpNum[2];
      uiMvpIdx[0] = cu.mvpIdx[REF_PIC_LIST_0];
      uiMvpIdx[1] = cu.mvpIdx[REF_PIC_LIST_1];
      uiMvpNum[0] = cu.mvpNum[REF_PIC_LIST_0];
      uiMvpNum[1] = cu.mvpNum[REF_PIC_LIST_1];
      cMvd[0] = cu.mvd[REF_PIC_LIST_0][0];
      cMvd[1] = cu.mvd[REF_PIC_LIST_1][0];

      MvField cHevcMvField[2];
      cHevcMvField[0].setMvField(cu.mv[REF_PIC_LIST_0][0], cu.refIdx[REF_PIC_LIST_0]);
      cHevcMvField[1].setMvField(cu.mv[REF_PIC_LIST_1][0], cu.refIdx[REF_PIC_LIST_1]);

      // do affine ME & Merge
      cu.affineType = AFFINEMODEL_4PARAM;
      Mv acMvAffine4Para[2][MAX_REF_PICS][3];
      int refIdx4Para[2] = { -1, -1 };

      xPredAffineInterSearch(cu, origBuf, puIdx, uiLastModeTemp, uiAffineCost, cMvHevcTemp, acMvAffine4Para, refIdx4Para, BcwIdx, enforceBcwPred);

      if (cu.imv == 0)
      {
        storeAffineMotion(cu.mv, cu.refIdx, AFFINEMODEL_4PARAM, BcwIdx);
      }
      if (cu.slice->sps->AffineType)
      {
        if (uiAffineCost < uiHevcCost * 1.05) ///< condition for 6 parameter affine ME
        {
          // save 4 parameter results
          Mv bestMv[2][3], bestMvd[2][3];
          int bestMvpIdx[2], bestMvpNum[2], bestRefIdx[2];
          uint8_t bestInterDir;

          bestInterDir = cu.interDir;
          bestRefIdx[0] = cu.refIdx[0];
          bestRefIdx[1] = cu.refIdx[1];
          bestMvpIdx[0] = cu.mvpIdx[0];
          bestMvpIdx[1] = cu.mvpIdx[1];
          bestMvpNum[0] = cu.mvpNum[0];
          bestMvpNum[1] = cu.mvpNum[1];

          for (int refList = 0; refList < 2; refList++)
          {
            bestMv[refList][0] = cu.mv[refList][0];
            bestMv[refList][1] = cu.mv[refList][1];
            bestMv[refList][2] = cu.mv[refList][2];
            bestMvd[refList][0] = cu.mvd[refList][0];
            bestMvd[refList][1] = cu.mvd[refList][1];
            bestMvd[refList][2] = cu.mvd[refList][2];
          }

          refIdx4Para[0] = bestRefIdx[0];
          refIdx4Para[1] = bestRefIdx[1];

          Distortion uiAffine6Cost = MAX_DISTORTION;
          cu.affineType = AFFINEMODEL_6PARAM;
          xPredAffineInterSearch(cu, origBuf, puIdx, uiLastModeTemp, uiAffine6Cost, cMvHevcTemp, acMvAffine4Para, refIdx4Para, BcwIdx, enforceBcwPred);

          if (cu.imv == 0)
          {
            storeAffineMotion(cu.mv, cu.refIdx, AFFINEMODEL_6PARAM, BcwIdx);
          }

          // reset to 4 parameter affine inter mode
          if (uiAffineCost <= uiAffine6Cost)
          {
            cu.affineType = AFFINEMODEL_4PARAM;
            cu.interDir = bestInterDir;
            cu.refIdx[0] = bestRefIdx[0];
            cu.refIdx[1] = bestRefIdx[1];
            cu.mvpIdx[0] = bestMvpIdx[0];
            cu.mvpIdx[1] = bestMvpIdx[1];
            cu.mvpNum[0] = bestMvpNum[0];
            cu.mvpNum[1] = bestMvpNum[1];

            for (int verIdx = 0; verIdx < 3; verIdx++)
            {
              cu.mvd[REF_PIC_LIST_0][verIdx] = bestMvd[0][verIdx];
              cu.mvd[REF_PIC_LIST_1][verIdx] = bestMvd[1][verIdx];
            }

            CU::setAllAffineMv(cu, bestMv[0][0], bestMv[0][1], bestMv[0][2], REF_PIC_LIST_0);
            CU::setAllAffineMv(cu, bestMv[1][0], bestMv[1][1], bestMv[1][2], REF_PIC_LIST_1);
          }
          else
          {
            uiAffineCost = uiAffine6Cost;
          }
        }

        uiAffineCost += m_pcRdCost->getCost(1); // add one bit for affine_type
      }

      if (uiHevcCost <= uiAffineCost)
      {
        // set hevc me result
        cu.affine = false;
        cu.mergeFlag = bMergeFlag;
        cu.regularMergeFlag = false;
        cu.mergeIdx = uiMRGIndex;
        cu.interDir = uiInterDir;
        cu.smvdMode = iSymMode;
        cu.mv[REF_PIC_LIST_0][0]  = cHevcMvField[0].mv;
        cu.refIdx[REF_PIC_LIST_0] = cHevcMvField[0].refIdx;
        cu.mv[REF_PIC_LIST_1][0]  = cHevcMvField[1].mv;
        cu.refIdx[REF_PIC_LIST_1] = cHevcMvField[1].refIdx;
        cu.mvpIdx[REF_PIC_LIST_0] = uiMvpIdx[0];
        cu.mvpIdx[REF_PIC_LIST_1] = uiMvpIdx[1];
        cu.mvpNum[REF_PIC_LIST_0] = uiMvpNum[0];
        cu.mvpNum[REF_PIC_LIST_1] = uiMvpNum[1];
        cu.mvd[REF_PIC_LIST_0][0] = cMvd[0];
        cu.mvd[REF_PIC_LIST_1][0] = cMvd[1];
      }
      else
      {
        cu.smvdMode = 0;
        CHECK(!cu.affine, "Wrong.");
        uiLastMode = uiLastModeTemp;
      }
    }

    
    CU::spanMotionInfo( cu, mergeCtx );

    m_skipPROF = false;
    m_encOnly  = false;
    //  MC
    PelUnitBuf predBuf = cu.cs->getPredBuf(cu);
    motionCompensation( cu, predBuf, REF_PIC_LIST_X );
    puIdx++;
#if USE_COST_BEFORE
    return false;
#endif
  }
}

// AMVP
void InterSearch::xEstimateMvPredAMVP( CodingUnit& cu, CPelUnitBuf& origBuf, RefPicList refPicList, int iRefIdx, Mv& rcMvPred, AMVPInfo& rAMVPInfo, Distortion& distBiP )
{
  Mv         cBestMv;
  int        iBestIdx   = 0;
  Distortion uiBestCost = MAX_DISTORTION;
  int        i;

  AMVPInfo*  pcAMVPInfo = &rAMVPInfo;

  // Fill the MV Candidates
  CU::fillMvpCand( cu, refPicList, iRefIdx, *pcAMVPInfo );

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->mvCand[0];

  PelUnitBuf predBuf = m_tmpStorageLCU.getCompactBuf( cu );

  //-- Check Minimum Cost.
  for( i = 0 ; i < pcAMVPInfo->numCand; i++)
  {
    Distortion uiTmpCost = xGetTemplateCost( cu, origBuf, predBuf, pcAMVPInfo->mvCand[i], i, AMVP_MAX_NUM_CANDS, refPicList, iRefIdx );
    if( uiBestCost > uiTmpCost )
    {
      uiBestCost  = uiTmpCost;
      cBestMv     = pcAMVPInfo->mvCand[i];
      iBestIdx    = i;
      distBiP     = uiTmpCost;
    }
  }

  // Setting Best MVP
  rcMvPred = cBestMv;
  cu.mvpIdx[refPicList] = iBestIdx;
  cu.mvpNum[refPicList] = pcAMVPInfo->numCand;

  return;
}

uint32_t InterSearch::xGetMvpIdxBits(int iIdx, int iNum)
{
  CHECK(iIdx < 0 || iNum < 0 || iIdx >= iNum, "Invalid parameters");

  if (iNum == 1)
  {
    return 0;
  }

  uint32_t uiLength = 1;
  int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }

  bool bCodeLast = ( iNum-1 > iTemp );

  uiLength += (iTemp-1);

  if( bCodeLast )
  {
    uiLength++;
  }

  return uiLength;
}

void InterSearch::xGetBlkBits( bool bPSlice, int iPartIdx, uint32_t uiLastMode, uint32_t uiBlkBit[3])
{
  uiBlkBit[0] = (! bPSlice) ? 3 : 1;
  uiBlkBit[1] = 3;
  uiBlkBit[2] = 5;
}

void InterSearch::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->numCand = pSrc->numCand;
  for (int i = 0; i < pSrc->numCand; i++)
  {
    pDst->mvCand[i] = pSrc->mvCand[i];
  }
}

void InterSearch::xCheckBestMVP ( RefPicList refPicList, const Mv& cMv, Mv& rcMvPred, int& riMVPIdx, AMVPInfo& amvpInfo, uint32_t& ruiBits, Distortion& ruiCost, const uint8_t imv )
{
  if ( imv > 0 && imv < 3 )
  {
    return;
  }

  AMVPInfo* pcAMVPInfo = &amvpInfo;

  CHECK(pcAMVPInfo->mvCand[riMVPIdx] != rcMvPred, "Invalid MV prediction candidate");

  if (pcAMVPInfo->numCand < 2)
  {
    return;
  }

  m_pcRdCost->setCostScale ( 0    );

  int iBestMVPIdx = riMVPIdx;

  Mv pred = rcMvPred;
  pred.changeTransPrecInternal2Amvr(imv);
  m_pcRdCost->setPredictor( pred );
  Mv mv = cMv;
  mv.changeTransPrecInternal2Amvr(imv);
  int iOrgMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
  int iBestMvBits = iOrgMvBits;

  for (int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->numCand; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }

    pred = pcAMVPInfo->mvCand[iMVPIdx];
    pred.changeTransPrecInternal2Amvr(imv);
    m_pcRdCost->setPredictor( pred );
    int iMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  //if changed
  {
    rcMvPred = pcAMVPInfo->mvCand[iBestMVPIdx];

    riMVPIdx = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}


Distortion InterSearch::xGetTemplateCost( const CodingUnit& cu,
                                          CPelUnitBuf& origBuf,
                                          PelUnitBuf&  predBuf,
                                          Mv           cMvCand,
                                          int          iMVPIdx,
                                          int          iMVPNum,
                                          RefPicList   refPicList,
                                          int          iRefIdx
)
{
  Distortion uiCost = MAX_DISTORTION;

  const Picture* picRef = cu.slice->getRefPic( refPicList, iRefIdx );
  clipMv( cMvCand, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv );

  // prediction pattern
  xPredInterBlk( COMP_Y, cu, picRef, cMvCand, predBuf, false, cu.slice->clpRngs[ COMP_Y ], false, false);

  // calc distortion

  uiCost = m_pcRdCost->getDistPart(origBuf.Y(), predBuf.Y(), cu.cs->sps->bitDepths[ CH_L ], COMP_Y, DF_SAD);
  uiCost += m_pcRdCost->getCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum] );

  return uiCost;
}

void InterSearch::xMotionEstimation(CodingUnit& cu, CPelUnitBuf& origBuf, RefPicList refPicList, Mv& rcMvPred, int iRefIdxPred, Mv& rcMv, int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost, const AMVPInfo& amvpInfo, bool bBi)
{
  Mv cMvHalf, cMvQter;

  CHECK(refPicList >= MAX_NUM_REF_LIST_ADAPT_SR || iRefIdxPred>=int(MAX_IDX_ADAPT_SR), "Invalid reference picture list");
  m_iSearchRange = m_aaiAdaptSR[refPicList][iRefIdxPred];

  int    iSrchRng   = (bBi ? m_bipredSearchRange : m_iSearchRange);
  double fWeight    = 1.0;

  CPelUnitBuf  origBufTmpCnst;
  CPelUnitBuf* pBuf      = &origBuf;

  if(bBi) // Bi-predictive ME
  {
    PelUnitBuf  origBufTmp = m_tmpStorageLCU.getCompactBuf( cu );
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)refPicList].getCompactBuf( cu );
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq( otherBuf, m_pcEncCfg->m_bClipForBiPredMeEnabled, cu.slice->clpRngs );
   
    origBufTmpCnst = m_tmpStorageLCU.getCompactBuf( cu );
    pBuf           = &origBufTmpCnst;
    fWeight        = xGetMEDistortionWeight( cu.BcwIdx, refPicList );
  }

  //  Search key pattern initialization
  CPelBuf  tmpPattern   = pBuf->Y();
  CPelBuf* pcPatternKey = &tmpPattern;

  m_lumaClpRng = cu.cs->slice->clpRngs[ COMP_Y ];

  const Picture* refPic = cu.slice->getRefPic(refPicList, iRefIdxPred);
  CPelBuf buf = cu.cs->sps->wrapAroundEnabled ? refPic->getRecoWrapBuf(cu.blocks[COMP_Y]) : refPic->getRecoBuf(cu.blocks[COMP_Y]);

  TZSearchStruct cStruct;
  cStruct.pcPatternKey  = pcPatternKey;
  cStruct.iRefStride    = buf.stride;
  cStruct.piRefY        = buf.buf;
  cStruct.imvShift      = cu.imv == IMV_HPEL ? 1 : (cu.imv << 1);
  cStruct.useAltHpelIf  = cu.imv == IMV_HPEL;
  cStruct.inCtuSearch   = false;
  cStruct.zeroMV        = false;

  CodedCUInfo &relatedCU = m_modeCtrl->getBlkInfo( cu );

  bool bQTBTMV  = false;
  bool bQTBTMV2 = false;
  Mv cIntMv;
  if( !bBi )
  {
    bool bValid = relatedCU.getMv( refPicList, iRefIdxPred, cIntMv );
    if( bValid )
    {
      bQTBTMV2 = true;
      cIntMv.changePrecision( MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    }
  }

  Mv predQuarter = rcMvPred;
  predQuarter.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
  m_pcRdCost->setPredictor( predQuarter );
  m_pcRdCost->setCostScale(2);

  //  Do integer search
  if( ( m_motionEstimationSearchMethod == VVENC_MESEARCH_FULL ) || bBi || bQTBTMV )
  {
    cStruct.subShiftMode = m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE1 || m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE3 ? 2 : 0;
    m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMP_Y, cStruct.subShiftMode);

    Mv bestInitMv = (bBi ? rcMv : rcMvPred);
    Mv cTmpMv     = bestInitMv;
    clipMv(cTmpMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
    cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;
    Distortion uiBestSad = m_cDistParam.distFunc(m_cDistParam);
    uiBestSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);

    for (int i = 0; i < m_BlkUniMvInfoBuffer->m_uniMvListSize; i++)
    {
      const BlkUniMvInfo* curMvInfo = m_BlkUniMvInfoBuffer->getBlkUniMvInfo( i );

      int j = 0;
      for (; j < i; j++)
      {
        const BlkUniMvInfo *prevMvInfo = m_BlkUniMvInfoBuffer->getBlkUniMvInfo( j );
        if (curMvInfo->uniMvs[refPicList][iRefIdxPred] == prevMvInfo->uniMvs[refPicList][iRefIdxPred])
        {
          break;
        }
      }
      if (j < i)
        continue;

      cTmpMv = curMvInfo->uniMvs[refPicList][iRefIdxPred];
      clipMv(cTmpMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
      cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

      Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
      if (uiSad < uiBestSad)
      {
        uiBestSad = uiSad;
        bestInitMv = curMvInfo->uniMvs[refPicList][iRefIdxPred];
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }

    if( !bQTBTMV )
    {
      xSetSearchRange(cu, bestInitMv, iSrchRng, cStruct.searchRange );
    }
    xPatternSearch( cStruct, rcMv, ruiCost);
  }
  else if( bQTBTMV2 )
  {
    rcMv = cIntMv;
    cStruct.subShiftMode = ( !m_pcEncCfg->m_bRestrictMESampling && m_pcEncCfg->m_motionEstimationSearchMethod == VVENC_MESEARCH_SELECTIVE ) ? 1 :
                            ( m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE1 || m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE3 ) ? 2 : 0;
    xTZSearch(cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiCost, NULL, false, true);
  }
  else
  {
    cStruct.subShiftMode = ( !m_pcEncCfg->m_bRestrictMESampling && m_pcEncCfg->m_motionEstimationSearchMethod == VVENC_MESEARCH_SELECTIVE ) ? 1 :
                            ( m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE1 || m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE3 ) ? 2 : 0;
    rcMv = rcMvPred;
    const Mv *pIntegerMv2Nx2NPred = nullptr;
    xPatternSearchFast(cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiCost, pIntegerMv2Nx2NPred);
    relatedCU.setMv( refPicList, iRefIdxPred, rcMv );
  }

  DTRACE( g_trace_ctx, D_ME, "%d %d %d :MECostFPel<L%d,%d>: %d,%d,%dx%d, %d", DTRACE_GET_COUNTER( g_trace_ctx, D_ME ), cu.slice->poc, 0, ( int ) refPicList, ( int ) bBi, cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, ruiCost );
  // sub-pel refinement for sub-pel resolution
  if ( cu.imv == 0 || cu.imv == IMV_HPEL )
  {
    xPatternSearchFracDIF(cu, refPicList, iRefIdxPred, cStruct, rcMv, cMvHalf, cMvQter, ruiCost);
    m_pcRdCost->setCostScale( 0 );
    rcMv <<= 2;
    rcMv  += ( cMvHalf <<= 1 );
    rcMv  += cMvQter;
    uint32_t uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor( rcMv.hor, rcMv.ver, cStruct.imvShift );
    ruiBits += uiMvBits;
    ruiCost = ( Distortion ) ( floor( fWeight * ( ( double ) ruiCost - ( double ) m_pcRdCost->getCost( uiMvBits ) ) ) + ( double ) m_pcRdCost->getCost( ruiBits ) );
    rcMv.changePrecision(MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL);
  }
  else // integer refinement for integer-pel and 4-pel resolution
  {
    rcMv.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    xPatternSearchIntRefine( cu, cStruct, rcMv, rcMvPred, riMVPIdx, ruiBits, ruiCost, amvpInfo, fWeight);
  }
  DTRACE(g_trace_ctx, D_ME, "   MECost<L%d,%d>: %6d (%d)  MV:%d,%d\n", (int)refPicList, (int)bBi, ruiCost, ruiBits, rcMv.hor << 2, rcMv.ver << 2);
}


void InterSearch::xSetSearchRange ( const CodingUnit& cu,
                                    const Mv& cMvPred,
                                    const int iSrchRng,
                                    SearchRange& sr )
{
  const PreCalcValues& pcv = *cu.cs->pcv;
  const int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
  Mv cFPMvPred = cMvPred;
  clipMv( cFPMvPred, cu.lumaPos(), cu.lumaSize(), pcv );

  Mv mvTL(cFPMvPred.hor - (iSrchRng << iMvShift), cFPMvPred.ver - (iSrchRng << iMvShift));
  Mv mvBR(cFPMvPred.hor + (iSrchRng << iMvShift), cFPMvPred.ver + (iSrchRng << iMvShift));

  if( pcv.wrapArround )
  {
    wrapClipMv( mvTL, cu.lumaPos(), cu.lumaSize(), *cu.cs);
    wrapClipMv( mvBR, cu.lumaPos(), cu.lumaSize(), *cu.cs);
  }
  else
  {
    clipMv( mvTL, cu.lumaPos(), cu.lumaSize(), pcv);
    clipMv( mvBR, cu.lumaPos(), cu.lumaSize(), pcv);
  }

  mvTL.divideByPowerOf2( iMvShift );
  mvBR.divideByPowerOf2( iMvShift );

  sr.left   = mvTL.hor;
  sr.top    = mvTL.ver;
  sr.right  = mvBR.hor;
  sr.bottom = mvBR.ver;
}


void InterSearch::xPatternSearch( TZSearchStruct&  cStruct,
                                  Mv&                 rcMv,
                                  Distortion&         ruiSAD )
{
  Distortion  uiSad;
  Distortion  uiSadBest = MAX_DISTORTION;
  int         iBestX = 0;
  int         iBestY = 0;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMP_Y, cStruct.subShiftMode );

  const SearchRange& sr = cStruct.searchRange;

  const Pel* piRef = cStruct.piRefY + (sr.top * cStruct.iRefStride);
  for ( int y = sr.top; y <= sr.bottom; y++ )
  {
    for ( int x = sr.left; x <= sr.right; x++ )
    {
      //  find min. distortion position
      m_cDistParam.cur.buf = piRef + x;

      uiSad = m_cDistParam.distFunc( m_cDistParam );

      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( x, y, cStruct.imvShift );

      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
    piRef += cStruct.iRefStride;
  }
  rcMv.set( iBestX, iBestY );

  cStruct.uiBestSad = uiSadBest; // th for testing
  ruiSAD = uiSadBest - m_pcRdCost->getCostOfVectorWithPredictor( iBestX, iBestY, cStruct.imvShift );
  return;
}


void InterSearch::xPatternSearchFast( const CodingUnit& cu,
                                      RefPicList            refPicList,
                                      int                   iRefIdxPred,
                                      TZSearchStruct&       cStruct,
                                      Mv&                   rcMv,
                                      Distortion&           ruiSAD,
                                      const Mv* const       pIntegerMv2Nx2NPred )
{
  if( cu.cs->picture->isSCC && m_motionEstimationSearchMethodSCC )
  {
    switch ( m_motionEstimationSearchMethodSCC )
    {
    case 3: //VVENC_MESEARCH_DIAMOND_FAST:
      xTZSearch         ( cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, true, true );
      break;
    case 2: //VVENC_MESEARCH_DIAMOND:
      xTZSearch         ( cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, true );
      break;
    case 1: //VVENC_MESEARCH_SELECTIVE:
      xTZSearchSelective( cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
      break;
    default:
      break;
    }
    return;
  }

  switch ( m_motionEstimationSearchMethod )
  {
  case VVENC_MESEARCH_DIAMOND_FAST:
    xTZSearch         ( cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, false, true );
    break;
  case VVENC_MESEARCH_DIAMOND:
    xTZSearch         ( cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, false );
    break;

  case VVENC_MESEARCH_SELECTIVE:
    xTZSearchSelective( cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
    break;

  case VVENC_MESEARCH_DIAMOND_ENHANCED:
    xTZSearch         ( cu, refPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, true );
    break;

  case VVENC_MESEARCH_FULL: // shouldn't get here.
  default:
    break;
  }
}


void InterSearch::xTZSearch( const CodingUnit& cu,
                             RefPicList            refPicList,
                             int                   iRefIdxPred,
                             TZSearchStruct&    cStruct,
                             Mv&                   rcMv,
                             Distortion&           ruiSAD,
                             const Mv* const       pIntegerMv2Nx2NPred,
                             const bool            bExtendedSettings,
                             const bool            bFastSettings)
{
  const bool bUseRasterInFastMode                    = true; //toggle this to further reduce runtime

  const bool bUseAdaptiveRaster                      = bExtendedSettings;
  const int  iRaster                                 = (bFastSettings && bUseRasterInFastMode) ? 8 : 5;
  const bool bTestZeroVector                         = true && !bFastSettings;
  const bool bTestZeroVectorStart                    = bExtendedSettings;
  const bool bTestZeroVectorStop                     = false;
  const bool bFirstSearchDiamond                     = true;  // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bFirstCornersForDiamondDist1            = bExtendedSettings;
  const bool bFirstSearchStop                        = m_pcEncCfg->m_bFastMEAssumingSmootherMVEnabled;
  const uint32_t uiFirstSearchRounds                 = bFastSettings ? (bUseRasterInFastMode?3:2) : 3;     // first search stop X rounds after best match (must be >=1)
  const bool bEnableRasterSearch                     = bFastSettings ? bUseRasterInFastMode : true;
  const bool bAlwaysRasterSearch                     = bExtendedSettings;  // true: BETTER but factor 2 slower
  const bool bRasterRefinementEnable                 = false; // enable either raster refinement or star refinement
  const bool bRasterRefinementDiamond                = false; // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bRasterRefinementCornersForDiamondDist1 = bExtendedSettings;
  const bool bStarRefinementEnable                   = true;  // enable either star refinement or raster refinement
  const bool bStarRefinementDiamond                  = true;  // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bStarRefinementCornersForDiamondDist1   = bExtendedSettings;
  const bool bStarRefinementStop                     = false || bFastSettings;
  const uint32_t uiStarRefinementRounds              = 2;  // star refinement stop X rounds after best match (must be >=1)
  const bool bNewZeroNeighbourhoodTest               = bExtendedSettings;

  int iSearchRange = m_iSearchRange;
  {
    clipMv( rcMv, cu.lumaPos(), cu.lumaSize(),*cu.cs->pcv );
  }
  rcMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
  rcMv.divideByPowerOf2(2);

  // init TZSearchStruct
  cStruct.uiBestSad = MAX_DISTORTION;

  //
  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMP_Y, cStruct.subShiftMode );

  // distortion


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( cStruct, rcMv.hor, rcMv.ver, 0, 0 );

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    if ((rcMv.hor != 0 || rcMv.ver != 0) &&
      (0 != cStruct.iBestX || 0 != cStruct.iBestY))
    {
      // only test 0-vector if not obviously previously tested.
      xTZSearchHelp( cStruct, 0, 0, 0, 0 );
    }
  }

  SearchRange& sr = cStruct.searchRange;

  if (pIntegerMv2Nx2NPred != 0)
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    {
      clipMv( integerMv2Nx2NPred, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv );
    }
    integerMv2Nx2NPred.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
    integerMv2Nx2NPred.divideByPowerOf2(2);

    if ((rcMv != integerMv2Nx2NPred) &&
      (integerMv2Nx2NPred.hor != cStruct.iBestX || integerMv2Nx2NPred.ver != cStruct.iBestY))
    {
      // only test integerMv2Nx2NPred if not obviously previously tested.
      xTZSearchHelp( cStruct, integerMv2Nx2NPred.hor, integerMv2Nx2NPred.ver, 0, 0);
    }
  }

  for (int i = 0; i < m_BlkUniMvInfoBuffer->m_uniMvListSize; i++)
  {
    const BlkUniMvInfo* curMvInfo = m_BlkUniMvInfoBuffer->getBlkUniMvInfo( i );

    int j = i;
    for (; j < i; j++)
    {
      const BlkUniMvInfo *prevMvInfo = m_BlkUniMvInfoBuffer->getBlkUniMvInfo( j );
      if (curMvInfo->uniMvs[refPicList][iRefIdxPred] == prevMvInfo->uniMvs[refPicList][iRefIdxPred])
      {
        break;
      }
    }

    Mv cTmpMv = curMvInfo->uniMvs[refPicList][iRefIdxPred];
    clipMv(cTmpMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
    cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

    Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
    uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
    if (uiSad < cStruct.uiBestSad)
    {
      cStruct.uiBestSad = uiSad;
      cStruct.iBestX = cTmpMv.hor;
      cStruct.iBestY = cTmpMv.ver;
      m_cDistParam.maximumDistortionForEarlyExit = uiSad;
    }
  }

  {
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= MV_FRACTIONAL_BITS_INTERNAL;
    xSetSearchRange(cu, currBestMv, m_iSearchRange >> (bFastSettings ? 1 : 0), sr );
  }

  // starting point after initial examination
  int  iDist = 0;
  int  iStartX = cStruct.iBestX;
  int  iStartY = cStruct.iBestY;

  // Early termination of motion search after selection of starting candidate
  if ( m_pcEncCfg->m_bIntegerET )
  {
    bool isLargeBlock = cu.lumaSize().area() > 64;
    xTZ8PointDiamondSearch( cStruct, iStartX, iStartY, 1, false ); // 4-point small diamond search
    if ( cStruct.iBestX == iStartX && cStruct.iBestY == iStartY )
    {
      if ( isLargeBlock )
      {
        xTZ4PointSquareSearch( cStruct, iStartX, iStartY, 1 );
        if ( cStruct.iBestX == iStartX && cStruct.iBestY == iStartY )
        {
          // write out best match
          rcMv.set( cStruct.iBestX, cStruct.iBestY );
          ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY, cStruct.imvShift );
          return;
        }
      }
      else
      {
        // write out best match
        rcMv.set( cStruct.iBestX, cStruct.iBestY );
        ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY, cStruct.imvShift );
        return;
      }
    }
  }

  // start search
  iDist = 0;
  iStartX = cStruct.iBestX;
  iStartY = cStruct.iBestY;

  const bool bBestCandidateZero = (cStruct.iBestX == 0) && (cStruct.iBestY == 0);

  // first search around best position up to now.
  // The following works as a "subsampled/log" window search around the best candidate
  for ( iDist = 1; iDist <= iSearchRange; iDist*=2 )
  {
    if ( bFirstSearchDiamond == 1 )
    {
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bFirstCornersForDiamondDist1 );
    }
    else
    {
      xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
    }

    if ( bFirstSearchStop && ( cStruct.uiBestRound >= uiFirstSearchRounds ) ) // stop criterion
    {
      break;
    }
  }

  if (!bNewZeroNeighbourhoodTest)
  {
    // test whether zero Mv is a better start point than Median predictor
    if ( bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)) )
    {
      xTZSearchHelp( cStruct, 0, 0, 0, 0 );
      if ( (cStruct.iBestX == 0) && (cStruct.iBestY == 0) )
      {
        // test its neighborhood
        for ( iDist = 1; iDist <= iSearchRange; iDist*=2 )
        {
          xTZ8PointDiamondSearch( cStruct, 0, 0, iDist, false );
          if ( bTestZeroVectorStop && (cStruct.uiBestRound > 0) ) // stop criterion
          {
            break;
          }
        }
      }
    }
  }
  else
  {
    // Test also zero neighbourhood but with half the range
    // It was reported that the original (above) search scheme using bTestZeroVectorStart did not
    // make sense since one would have already checked the zero candidate earlier
    // and thus the conditions for that test would have not been satisfied
    if (bTestZeroVectorStart == true && bBestCandidateZero != true)
    {
      for ( iDist = 1; iDist <= (iSearchRange >> 1); iDist*=2 )
      {
        xTZ8PointDiamondSearch( cStruct, 0, 0, iDist, false );
        if ( bTestZeroVectorStop && (cStruct.uiBestRound > 2) ) // stop criterion
        {
          break;
        }
      }
    }
  }

  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if ( cStruct.uiBestDistance == 1 )
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch( cStruct );
  }

  // raster search if distance is too big
  if (bUseAdaptiveRaster)
  {
    int iWindowSize     = iRaster;
    SearchRange localsr = sr;

    if (!(bEnableRasterSearch && ( ((int)(cStruct.uiBestDistance) >= iRaster))))
    {
      iWindowSize ++;
      localsr.left   /= 2;
      localsr.right  /= 2;
      localsr.top    /= 2;
      localsr.bottom /= 2;
    }
    cStruct.uiBestDistance = iWindowSize;
    for ( iStartY = localsr.top; iStartY <= localsr.bottom; iStartY += iWindowSize )
    {
      for ( iStartX = localsr.left; iStartX <= localsr.right; iStartX += iWindowSize )
      {
        xTZSearchHelp( cStruct, iStartX, iStartY, 0, iWindowSize );
      }
    }
  }
  else
  {
    if ( bEnableRasterSearch && ( ((int)(cStruct.uiBestDistance) >= iRaster) || bAlwaysRasterSearch ) )
    {
      cStruct.uiBestDistance = iRaster;
      for ( iStartY = sr.top; iStartY <= sr.bottom; iStartY += iRaster )
      {
        for ( iStartX = sr.left; iStartX <= sr.right; iStartX += iRaster )
        {
          xTZSearchHelp( cStruct, iStartX, iStartY, 0, iRaster );
        }
      }
    }
  }

  // raster refinement

  if ( bRasterRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if ( cStruct.uiBestDistance > 1 )
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if ( bRasterRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bRasterRefinementCornersForDiamondDist1 );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
      }

      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // star refinement
  if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < iSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bStarRefinementCornersForDiamondDist1 );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY, cStruct.imvShift );
}


void InterSearch::xTZSearchSelective( const CodingUnit& cu,
                                      RefPicList            refPicList,
                                      int                   iRefIdxPred,
                                      TZSearchStruct&    cStruct,
                                      Mv                    &rcMv,
                                      Distortion            &ruiSAD,
                                      const Mv* const       pIntegerMv2Nx2NPred )
{
  const bool bTestZeroVector          = true;
  const bool bEnableRasterSearch      = true;
  const bool bAlwaysRasterSearch      = false;  // 1: BETTER but factor 15x slower
  const bool bStarRefinementEnable    = true;   // enable either star refinement or raster refinement
  const bool bStarRefinementDiamond   = true;   // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bStarRefinementStop      = false;
  const uint32_t uiStarRefinementRounds   = 2;  // star refinement stop X rounds after best match (must be >=1)
  const int  iSearchRange             = m_iSearchRange;
  const int  iSearchRangeInitial      = m_iSearchRange >> 2;
  const int  uiSearchStep             = 4;
  const int  iMVDistThresh            = 8;

  int   iStartX                 = 0;
  int   iStartY                 = 0;
  int   iDist                   = 0;
  clipMv( rcMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv );
  rcMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
  rcMv.divideByPowerOf2(2);

  // init TZSearchStruct
  cStruct.uiBestSad = MAX_DISTORTION;
  cStruct.iBestX = 0;
  cStruct.iBestY = 0;

  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMP_Y, cStruct.subShiftMode );


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( cStruct, rcMv.hor, rcMv.ver, 0, 0 );

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( cStruct, 0, 0, 0, 0 );
  }

  SearchRange& sr = cStruct.searchRange;

  if ( pIntegerMv2Nx2NPred != 0 )
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    clipMv( integerMv2Nx2NPred, cu.lumaPos(), cu.lumaSize(),*cu.cs->pcv );
    integerMv2Nx2NPred.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
    integerMv2Nx2NPred.divideByPowerOf2(2);

    xTZSearchHelp( cStruct, integerMv2Nx2NPred.hor, integerMv2Nx2NPred.ver, 0, 0);

  }

  for (int i = 0; i < m_BlkUniMvInfoBuffer->m_uniMvListSize; i++)
  {
    const BlkUniMvInfo* curMvInfo = m_BlkUniMvInfoBuffer->getBlkUniMvInfo( i );

    int j = 0;
    for (; j < i; j++)
    {
      const BlkUniMvInfo *prevMvInfo = m_BlkUniMvInfoBuffer->getBlkUniMvInfo( j );
      if (curMvInfo->uniMvs[refPicList][iRefIdxPred] == prevMvInfo->uniMvs[refPicList][iRefIdxPred])
      {
        break;
      }
    }
    if (j < i)
      continue;

    Mv cTmpMv = curMvInfo->uniMvs[refPicList][iRefIdxPred];
    clipMv(cTmpMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
    cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

    Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
    uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
    if (uiSad < cStruct.uiBestSad)
    {
      cStruct.uiBestSad = uiSad;
      cStruct.iBestX = cTmpMv.hor;
      cStruct.iBestY = cTmpMv.ver;
      m_cDistParam.maximumDistortionForEarlyExit = uiSad;
    }
  }

  {
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( cu, currBestMv, m_iSearchRange, sr );
  }

  // Initial search
  int iBestX = cStruct.iBestX;
  int iBestY = cStruct.iBestY;
  int iFirstSrchRngHorLeft    = ((iBestX - iSearchRangeInitial) > sr.left)   ? (iBestX - iSearchRangeInitial) : sr.left;
  int iFirstSrchRngVerTop     = ((iBestY - iSearchRangeInitial) > sr.top)    ? (iBestY - iSearchRangeInitial) : sr.top;
  int iFirstSrchRngHorRight   = ((iBestX + iSearchRangeInitial) < sr.right)  ? (iBestX + iSearchRangeInitial) : sr.right;
  int iFirstSrchRngVerBottom  = ((iBestY + iSearchRangeInitial) < sr.bottom) ? (iBestY + iSearchRangeInitial) : sr.bottom;

  for ( iStartY = iFirstSrchRngVerTop; iStartY <= iFirstSrchRngVerBottom; iStartY += uiSearchStep )
  {
    for ( iStartX = iFirstSrchRngHorLeft; iStartX <= iFirstSrchRngHorRight; iStartX += uiSearchStep )
    {
      xTZSearchHelp( cStruct, iStartX, iStartY, 0, 0 );
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, 1, false );
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, 2, false );
    }
  }

  int iMaxMVDistToPred = (abs(cStruct.iBestX - iBestX) > iMVDistThresh || abs(cStruct.iBestY - iBestY) > iMVDistThresh);

  //full search with early exit if MV is distant from predictors
  if ( bEnableRasterSearch && (iMaxMVDistToPred || bAlwaysRasterSearch) )
  {
    for ( iStartY = sr.top; iStartY <= sr.bottom; iStartY += 1 )
    {
      for ( iStartX = sr.left; iStartX <= sr.right; iStartX += 1 )
      {
        xTZSearchHelp( cStruct, iStartX, iStartY, 0, 1 );
      }
    }
  }
  //Smaller MV, refine around predictor
  else if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    // start refinement
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < iSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, false );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY, cStruct.imvShift );
}

void InterSearch::xPatternSearchIntRefine(CodingUnit& cu, TZSearchStruct&  cStruct, Mv& rcMv, Mv& rcMvPred, int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost, const AMVPInfo& amvpInfo, double fWeight)
{

  CHECK( cu.imv == 0 || cu.imv == IMV_HPEL , "xPatternSearchIntRefine(): Sub-pel MV used.");
  CHECK( amvpInfo.mvCand[riMVPIdx] != rcMvPred, "xPatternSearchIntRefine(): MvPred issue.");

  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMP_Y, 0, m_pcEncCfg->m_bUseHADME && !cu.cs->slice->disableSATDForRd);

  // -> set MV scale for cost calculation to QPEL (0)
  m_pcRdCost->setCostScale ( 0 );

  Distortion  uiDist, uiSATD = 0;
  Distortion  uiBestDist  = MAX_DISTORTION;
  // subtract old MVP costs because costs for all newly tested MVPs are added in here
  ruiBits -= m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];

  Mv cBestMv = rcMv;
  Mv cBaseMvd[2];
  int iBestBits = 0;
  int iBestMVPIdx = riMVPIdx;
  Mv testPos[9] = { { 0, 0}, { -1, -1},{ -1, 0},{ -1, 1},{ 0, -1},{ 0, 1},{ 1, -1},{ 1, 0},{ 1, 1} };


  cBaseMvd[0] = (rcMv - amvpInfo.mvCand[0]);
  cBaseMvd[1] = (rcMv - amvpInfo.mvCand[1]);
  CHECK( (cBaseMvd[0].hor & 0x03) != 0 || (cBaseMvd[0].ver & 0x03) != 0 , "xPatternSearchIntRefine(): AMVP cand 0 Mvd issue.");
  CHECK( (cBaseMvd[1].hor & 0x03) != 0 || (cBaseMvd[1].ver & 0x03) != 0 , "xPatternSearchIntRefine(): AMVP cand 1 Mvd issue.");

  cBaseMvd[0].roundTransPrecInternal2Amvr(cu.imv);
  cBaseMvd[1].roundTransPrecInternal2Amvr(cu.imv);

  // test best integer position and all 8 neighboring positions
  for (int pos = 0; pos < 9; pos ++)
  {
    Mv cTestMv[2];
    // test both AMVP candidates for each position
    for (int iMVPIdx = 0; iMVPIdx < amvpInfo.numCand; iMVPIdx++)
    {
      cTestMv[iMVPIdx] = testPos[pos];
      cTestMv[iMVPIdx].changeTransPrecAmvr2Internal(cu.imv);
      cTestMv[iMVPIdx] += cBaseMvd[iMVPIdx];
      cTestMv[iMVPIdx] += amvpInfo.mvCand[iMVPIdx];

      if ( iMVPIdx == 0 || cTestMv[0] != cTestMv[1])
      {
        Mv cTempMV = cTestMv[iMVPIdx];
        {
          clipMv(cTempMV, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
        }
        m_cDistParam.cur.buf = cStruct.piRefY  + cStruct.iRefStride * (cTempMV.ver >>  MV_FRACTIONAL_BITS_INTERNAL) + (cTempMV.hor >> MV_FRACTIONAL_BITS_INTERNAL);
        uiDist = uiSATD = (Distortion) (m_cDistParam.distFunc( m_cDistParam ) * fWeight);
      }
      else
      {
        uiDist = uiSATD;
      }

      int iMvBits = m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];
      Mv pred = amvpInfo.mvCand[iMVPIdx];
      pred.changeTransPrecInternal2Amvr(cu.imv);
      m_pcRdCost->setPredictor( pred );
      Mv mv = cTestMv[iMVPIdx];
      mv.changeTransPrecInternal2Amvr(cu.imv);
      iMvBits += m_pcRdCost->getBitsOfVectorWithPredictor( mv.hor, mv.ver, 0 );
      uiDist += m_pcRdCost->getCost(iMvBits);

      if (uiDist < uiBestDist)
      {
        uiBestDist = uiDist;
        cBestMv = cTestMv[iMVPIdx];
        iBestMVPIdx = iMVPIdx;
        iBestBits = iMvBits;
      }
    }
  }
  if( uiBestDist == MAX_DISTORTION )
  {
    ruiCost = MAX_DISTORTION;
    return;
  }

  rcMv = cBestMv;
  rcMvPred = amvpInfo.mvCand[iBestMVPIdx];
  riMVPIdx = iBestMVPIdx;
  m_pcRdCost->setPredictor( rcMvPred );

  ruiBits += iBestBits;
  // taken from JEM 5.0
  // verify since it makes no sence to subtract Lamda*(Rmvd+Rmvpidx) from D+Lamda(Rmvd)
  // this would take the rate for the MVP idx out of the cost calculation
  // however this rate is always 1 so impact is small
  ruiCost = uiBestDist - m_pcRdCost->getCost(iBestBits) + m_pcRdCost->getCost(ruiBits);
  // taken from JEM 5.0
  // verify since it makes no sense to add rate for MVDs twicce

  return;
}

void InterSearch::xPatternSearchFracDIF(
  const CodingUnit& cu,
  RefPicList            refPicList,
  int                   iRefIdx,
  TZSearchStruct&    cStruct,
  const Mv&             rcMvInt,
  Mv&                   rcMvHalf,
  Mv&                   rcMvQter,
  Distortion&           ruiCost
)
{
  PROFILER_SCOPE_AND_STAGE( 0, g_timeProfiler, P_FRAC_PEL );

  //  Reference pattern initialization (integer scale)
  int         iOffset    = rcMvInt.hor + rcMvInt.ver * cStruct.iRefStride;
  CPelBuf cPatternRoi(cStruct.piRefY + iOffset, cStruct.iRefStride, *cStruct.pcPatternKey);

  //  Half-pel refinement
  m_pcRdCost->setCostScale(1);
  if( 0 == m_pcEncCfg->m_fastSubPel )
  {
    xExtDIFUpSamplingH( &cPatternRoi, cStruct.useAltHpelIf );
  }

  rcMvHalf = rcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  Mv baseRefMv(0, 0);
  Distortion  uiDistBest = MAX_DISTORTION;
  int patternId = 41;
  ruiCost = xPatternRefinement( cStruct.pcPatternKey, baseRefMv, 2, rcMvHalf, ( !cu.cs->slice->disableSATDForRd ), uiDistBest, patternId, &cPatternRoi, cStruct.useAltHpelIf );
  patternId -= ( m_pcEncCfg->m_fastSubPel ? 41 : 0 );


  //  quarter-pel refinement
  if( cStruct.imvShift == IMV_OFF && 0 != patternId )
  {
    PROFILER_SCOPE_AND_STAGE( 0, g_timeProfiler, P_QPEL );
    m_pcRdCost->setCostScale( 0 );
    xExtDIFUpSamplingQ( &cPatternRoi, rcMvHalf, patternId );
    baseRefMv = rcMvHalf;
    baseRefMv <<= 1;

    rcMvQter = rcMvInt;    rcMvQter <<= 1;    // for mv-cost
    rcMvQter += rcMvHalf;  rcMvQter <<= 1;
    ruiCost = xPatternRefinement( cStruct.pcPatternKey, baseRefMv, 1, rcMvQter, ( !cu.cs->slice->disableSATDForRd ), uiDistBest, patternId, &cPatternRoi, cStruct.useAltHpelIf );
  }

}

Distortion InterSearch::xGetSymCost( const CodingUnit& cu, CPelUnitBuf& origBuf, RefPicList eCurRefPicList, const MvField& cCurMvField, MvField& cTarMvField, int BcwIdx )
{
  Distortion cost = MAX_DISTORTION;
  RefPicList eTarRefPicList = (RefPicList)(1 - (int)eCurRefPicList);

  // get prediction of eCurRefPicList
  PelUnitBuf  predBufA  = m_tmpPredStorage[eCurRefPicList].getCompactBuf( cu );
  const Picture* picRefA = cu.slice->getRefPic( eCurRefPicList, cCurMvField.refIdx );
  Mv mvA = cCurMvField.mv;
  clipMv( mvA, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv );
  xPredInterBlk( COMP_Y, cu, picRefA, mvA, predBufA, false, cu.slice->clpRngs[ COMP_Y ], false, false );

  // get prediction of eTarRefPicList
  PelUnitBuf predBufB = m_tmpPredStorage[eTarRefPicList].getCompactBuf( cu );
  const Picture* picRefB = cu.slice->getRefPic( eTarRefPicList, cTarMvField.refIdx );
  Mv mvB = cTarMvField.mv;
  clipMv( mvB, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv );
  xPredInterBlk( COMP_Y, cu, picRefB, mvB, predBufB, false, cu.slice->clpRngs[ COMP_Y ], false, false );

  PelUnitBuf bufTmp = m_tmpStorageLCU.getCompactBuf( UnitAreaRelative( cu, cu ) );
  bufTmp.copyFrom( origBuf );
  bufTmp.removeHighFreq( predBufA, m_pcEncCfg->m_bClipForBiPredMeEnabled, cu.slice->clpRngs/*, getBcwWeight( cu.BcwIdx, eTarRefPicList )*/ );
  double fWeight = xGetMEDistortionWeight( cu.BcwIdx, eTarRefPicList );

  // calc distortion
  DFunc distFunc = ( !cu.slice->disableSATDForRd ) ? DF_HAD : DF_SAD;
  cost = ( Distortion ) floor( fWeight * ( double ) m_pcRdCost->getDistPart( bufTmp.Y(), predBufB.Y(), cu.cs->sps->bitDepths[ CH_L ], COMP_Y, distFunc ) );

  return(cost);
}

Distortion InterSearch::xSymRefineMvSearch( CodingUnit& cu, CPelUnitBuf& origBuf, Mv& rcMvCurPred, Mv& rcMvTarPred, RefPicList refPicList, MvField& rCurMvField, 
                                            MvField& rTarMvField, Distortion uiMinCost, int SearchPattern, int nSearchStepShift, uint32_t uiMaxSearchRounds, int BcwIdx )
{
  const Mv mvSearchOffsetCross[4] = { Mv( 0 , 1 ) , Mv( 1 , 0 ) , Mv( 0 , -1 ) , Mv( -1 ,  0 ) };
  const Mv mvSearchOffsetSquare[8] = { Mv( -1 , 1 ) , Mv( 0 , 1 ) , Mv( 1 ,  1 ) , Mv( 1 ,  0 ) , Mv( 1 , -1 ) , Mv( 0 , -1 ) , Mv( -1 , -1 ) , Mv( -1 , 0 ) };
  const Mv mvSearchOffsetDiamond[8] = { Mv( 0 , 2 ) , Mv( 1 , 1 ) , Mv( 2 ,  0 ) , Mv( 1 , -1 ) , Mv( 0 , -2 ) , Mv( -1 , -1 ) , Mv( -2 ,  0 ) , Mv( -1 , 1 ) };
  const Mv mvSearchOffsetHexagon[6] = { Mv( 2 , 0 ) , Mv( 1 , 2 ) , Mv( -1 ,  2 ) , Mv( -2 ,  0 ) , Mv( -1 , -2 ) , Mv( 1 , -2 ) };

  int nDirectStart = 0, nDirectEnd = 0, nDirectRounding = 0, nDirectMask = 0;
  const Mv * pSearchOffset;
  if ( SearchPattern == 0 )
  {
    nDirectEnd = 3;
    nDirectRounding = 4;
    nDirectMask = 0x03;
    pSearchOffset = mvSearchOffsetCross;
  }
  else if ( SearchPattern == 1 )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetSquare;
  }
  else if ( SearchPattern == 2 )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetDiamond;
  }
  else if ( SearchPattern == 3 )
  {
    nDirectEnd = 5;
    pSearchOffset = mvSearchOffsetHexagon;
  }
  else
  {
    THROW( "Invalid search pattern" );
  }

  int nBestDirect;
  for ( uint32_t uiRound = 0; uiRound < uiMaxSearchRounds; uiRound++ )
  {
    Distortion roundZeroBestCost = MAX_DISTORTION;
    const int positionLut[ 8 ] = { 0, 2, 4, 6, 1, 3, 5, 7 };
    nBestDirect = -1;
    MvField mvCurCenter = rCurMvField;
    for ( int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++ )
    {
      // terminate the search if none of the first four tested points hasn't provided improvement
      if( m_pcEncCfg->m_SMVD > 1 && 2 == SearchPattern && 0 == uiRound && 4 == nIdx && roundZeroBestCost > uiMinCost )
      {
        break;
      }
      int nDirect;
      if ( SearchPattern == 3 )
      {
        nDirect = nIdx < 0 ? nIdx + 6 : nIdx >= 6 ? nIdx - 6 : nIdx;
      }
      else
      {
        if( m_pcEncCfg->m_SMVD > 1 && 2 == SearchPattern && 0 == uiRound )
        {
          nDirect = positionLut[ ( nIdx + nDirectRounding ) & nDirectMask ];
        }
        else
        {
          nDirect = ( nIdx + nDirectRounding ) & nDirectMask;
        }
      }

      Mv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;
      MvField mvCand = mvCurCenter, mvPair;
      mvCand.mv += mvOffset;

      // get MVD cost
      Mv pred = rcMvCurPred;
      pred.changeTransPrecInternal2Amvr(cu.imv);
      m_pcRdCost->setPredictor( pred );
      m_pcRdCost->setCostScale( 0 );
      Mv mv = mvCand.mv;
      mv.changeTransPrecInternal2Amvr(cu.imv);
      uint32_t uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor( mv.hor, mv.ver, 0 );
      Distortion uiCost = m_pcRdCost->getCost( uiMvBits );

      // get MVD pair and set target MV
      mvPair.refIdx = rTarMvField.refIdx;
      mvPair.mv.set( rcMvTarPred.hor - (mvCand.mv.hor - rcMvCurPred.hor), rcMvTarPred.ver - (mvCand.mv.ver - rcMvCurPred.ver) );
      uiCost += xGetSymCost( cu, origBuf, refPicList, mvCand, mvPair, BcwIdx );
      if ( uiCost < uiMinCost )
      {
        uiMinCost = uiCost;
        rCurMvField = mvCand;
        rTarMvField = mvPair;
        nBestDirect = nDirect;
      }
      if ( m_pcEncCfg->m_SMVD > 1 && 2 == SearchPattern && 0 == uiRound && 4 > nIdx && uiCost < roundZeroBestCost)
      {
        roundZeroBestCost = uiCost;
      }
    }

    if ( nBestDirect == -1 )
    {
      break;
    }
    int nStep = 1;
    if( (SearchPattern == 1 || SearchPattern == 2) && m_pcEncCfg->m_SMVD <= 1 )
    {
      // test at most 3 points in fast presets
      nStep = 2 - ( nBestDirect & 0x01 );
    }
    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  return(uiMinCost);
}


void InterSearch::xSymMotionEstimation( CodingUnit& cu, CPelUnitBuf& origBuf, Mv& rcMvCurPred, Mv& rcMvTarPred, RefPicList refPicList, MvField& rCurMvField, MvField& rTarMvField, Distortion& ruiCost, int BcwIdx )
{
  // Refine Search
  int nSearchStepShift = MV_FRACTIONAL_BITS_DIFF;
  int nDiamondRound = 8;
  int nCrossRound = 1;

  nSearchStepShift += cu.imv == IMV_HPEL ? 1 : (cu.imv << 1);
  nDiamondRound >>= cu.imv;

  ruiCost = xSymRefineMvSearch( cu, origBuf, rcMvCurPred, rcMvTarPred, refPicList, rCurMvField, rTarMvField, ruiCost, 2, nSearchStepShift, nDiamondRound, BcwIdx );
  if( m_pcEncCfg->m_SMVD < 3 )
  {
    ruiCost = xSymRefineMvSearch( cu, origBuf, rcMvCurPred, rcMvTarPred, refPicList, rCurMvField, rTarMvField, ruiCost, 0, nSearchStepShift, nCrossRound, BcwIdx );
  }
}


/**
* \brief Generate half-sample interpolated block
*
* \param pattern Reference picture ROI
* \param biPred    Flag indicating whether block is for biprediction
*/
void InterSearch::xExtDIFUpSamplingH(CPelBuf* pattern, bool useAltHpelIf)
{
  PROFILER_SCOPE_AND_STAGE( 0, g_timeProfiler, P_HPEL_INTERP );
  const ClpRng& clpRng = m_lumaClpRng;
  int width      = pattern->width;
  int height     = pattern->height;
  int srcStride  = pattern->stride;

  int intStride = width + 1;
  int dstStride = width + 1;
  Pel* intPtr;
  Pel* dstPtr;
  int filterSize = NTAPS_LUMA;
  int halfFilterSize = (filterSize>>1);
  const Pel* srcPtr = pattern->buf - halfFilterSize*srcStride - 1;

  const ChromaFormat chFmt = m_currChromaFormat;

  // split the prediction with funny widths into power-of-2 and +1 parts for the sake of SIMD speed-up
  m_if.filterHor( COMP_Y, srcPtr,         srcStride, m_filteredBlockTmp[0][0]        , intStride, width, height + filterSize, 0 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, useAltHpelIf );
  m_if.filterHor( COMP_Y, srcPtr + width, srcStride, m_filteredBlockTmp[0][0] + width, intStride,     1, height + filterSize, 0 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, useAltHpelIf );

  // split the prediction with funny widths into power-of-2 and +1 parts for the sake of SIMD speed-up
  m_if.filterHor( COMP_Y, srcPtr,         srcStride, m_filteredBlockTmp[2][0],         intStride, width, height + filterSize, 2 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, useAltHpelIf );
  m_if.filterHor( COMP_Y, srcPtr + width, srcStride, m_filteredBlockTmp[2][0] + width, intStride,     1, height + filterSize, 2 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, useAltHpelIf );

  intPtr = m_filteredBlockTmp[0][0] + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0][0];
  m_if.filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf);

  intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0][0];
  m_if.filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf);

  intPtr = m_filteredBlockTmp[2][0] + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2][0];
  // split the prediction with funny widths into power-of-2 and +1 parts for the sake of SIMD speed-up
  m_if.filterVer( COMP_Y, intPtr,         intStride, dstPtr,         dstStride, width, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
  m_if.filterVer( COMP_Y, intPtr + width, intStride, dstPtr + width, dstStride,     1, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );

  intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[2][2][0];
  // split the prediction with funny widths into power-of-2 and +1 parts for the sake of SIMD speed-up
  m_if.filterVer( COMP_Y, intPtr,         intStride, dstPtr,         dstStride, width, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
  m_if.filterVer( COMP_Y, intPtr + width, intStride, dstPtr + width, dstStride,     1, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, useAltHpelIf );
}





/**
* \brief Generate quarter-sample interpolated blocks
*
* \param pattern    Reference picture ROI
* \param halfPelRef Half-pel mv
* \param biPred     Flag indicating whether block is for biprediction
*/
void InterSearch::xExtDIFUpSamplingQ( CPelBuf* pattern, Mv halfPelRef, int& patternId )
{
  PROFILER_SCOPE_AND_STAGE( 0, g_timeProfiler, P_QPEL_INTERP );
  const ClpRng& clpRng = m_lumaClpRng;
  int width      = pattern->width;
  int height     = pattern->height;
  int srcStride  = pattern->stride;

  Pel const* srcPtr;
  int intStride = width + 1;
  int dstStride = width + 1;
  Pel* intPtr;
  Pel* dstPtr;
  int filterSize = NTAPS_LUMA;

  int halfFilterSize = (filterSize>>1);

  int extHeight = (halfPelRef.ver == 0) ? height + filterSize : height + filterSize-1;

  const ChromaFormat chFmt = m_currChromaFormat;

  if( s_doInterpQ[ patternId ][ 12 ] )
  {
    // Horizontal filter 1/4
    srcPtr = pattern->buf - halfFilterSize * srcStride - 1;
    intPtr = m_filteredBlockTmp[ 1 ][ 0 ];
    if( halfPelRef.ver > 0 )
    {
      srcPtr += srcStride;
    }
    if( halfPelRef.hor >= 0 )
    {
      srcPtr += 1;
    }
    m_if.filterHor( COMP_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng );
  }

  if( s_doInterpQ[ patternId ][ 13 ] )
  {
    // Horizontal filter 3/4
    srcPtr = pattern->buf - halfFilterSize*srcStride - 1;
    intPtr = m_filteredBlockTmp[ 3 ][ 0 ];
    if( halfPelRef.ver > 0 )
    {
      srcPtr += srcStride;
    }
    if( halfPelRef.hor > 0 )
    {
      srcPtr += 1;
    }
    m_if.filterHor( COMP_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng );
  }

  if( s_doInterpQ[ patternId ][ 3 ] )
  {
    // Generate @ 1,1
    intPtr = m_filteredBlockTmp[ 1 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
    dstPtr = m_filteredBlock[ 1 ][ 1 ][ 0 ];
    if( halfPelRef.ver == 0 )
    {
      intPtr += intStride;
    }
    m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
  }

  if( s_doInterpQ[ patternId ][ 11 ] )
  {
    // Generate @ 3,3
    intPtr = m_filteredBlockTmp[ 3 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
    dstPtr = m_filteredBlock[ 3 ][ 3 ][ 0 ];
    m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
  }

  if( s_doInterpQ[ patternId ][ 5 ] )
  {
    // Generate @ 3,1
    intPtr = m_filteredBlockTmp[ 1 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
    dstPtr = m_filteredBlock[ 3 ][ 1 ][ 0 ];
    m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
  }

  if( s_doInterpQ[ patternId ][ 9 ] )
  {
    // Generate @ 1,3
    intPtr = m_filteredBlockTmp[ 3 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
    dstPtr = m_filteredBlock[ 1 ][ 3 ][ 0 ];
    if( halfPelRef.ver == 0 )
    {
      intPtr += intStride;
    }
    m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
  }

  if (halfPelRef.ver != 0)
  {
    if( s_doInterpQ[ patternId ][ 4 ] )
    {
      // Generate @ 2,1
      intPtr = m_filteredBlockTmp[ 1 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
      dstPtr = m_filteredBlock[ 2 ][ 1 ][ 0 ];
      if( halfPelRef.ver == 0 )
      {
        intPtr += intStride;
      }
      m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
    }

    if( s_doInterpQ[ patternId ][ 10 ] )
    {
      // Generate @ 2,3
      intPtr = m_filteredBlockTmp[ 3 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
      dstPtr = m_filteredBlock[ 2 ][ 3 ][ 0 ];
      if( halfPelRef.ver == 0 )
      {
        intPtr += intStride;
      }
      m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
    }
  }
  else
  {
    if( s_doInterpQ[ patternId ][ 2 ] )
    {
      // Generate @ 0,1
      intPtr = m_filteredBlockTmp[ 1 ][ 0 ] + halfFilterSize * intStride;
      dstPtr = m_filteredBlock[ 0 ][ 1 ][ 0 ];
      m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
    }

    if( s_doInterpQ[ patternId ][ 8 ] )
    {
      // Generate @ 0,3
      intPtr = m_filteredBlockTmp[ 3 ][ 0 ] + halfFilterSize * intStride;
      dstPtr = m_filteredBlock[ 0 ][ 3 ][ 0 ];
      m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
    }
  }

  if (halfPelRef.hor != 0)
  {
    if( s_doInterpQ[ patternId ][ 6 ] )
    {
      // Generate @ 1,2
      intPtr = m_filteredBlockTmp[ 2 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
      dstPtr = m_filteredBlock[ 1 ][ 2 ][ 0 ];
      if( halfPelRef.hor > 0 )
      {
        intPtr += 1;
      }
      if( halfPelRef.ver >= 0 )
      {
        intPtr += intStride;
      }
      m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
    }

    if( s_doInterpQ[ patternId ][ 7 ] )
    {
      // Generate @ 3,2
      intPtr = m_filteredBlockTmp[ 2 ][ 0 ] + ( halfFilterSize - 1 ) * intStride;
      dstPtr = m_filteredBlock[ 3 ][ 2 ][ 0 ];
      if( halfPelRef.hor > 0 )
      {
        intPtr += 1;
      }
      if( halfPelRef.ver > 0 )
      {
        intPtr += intStride;
      }
      m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
    }
  }
  else
  {
    if( s_doInterpQ[ patternId ][ 0 ] )
    {
      // Generate @ 1,0
      intPtr = m_filteredBlockTmp[ 0 ][ 0 ] + ( halfFilterSize - 1 ) * intStride + 1;
      dstPtr = m_filteredBlock[ 1 ][ 0 ][ 0 ];
      if( halfPelRef.ver >= 0 )
      {
        intPtr += intStride;
      }
      m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
    }

    if( s_doInterpQ[ patternId ][ 1 ] )
    {
      // Generate @ 3,0
      intPtr = m_filteredBlockTmp[ 0 ][ 0 ] + ( halfFilterSize - 1 ) * intStride + 1;
      dstPtr = m_filteredBlock[ 3 ][ 0 ][ 0 ];
      if( halfPelRef.ver > 0 )
      {
        intPtr += intStride;
      }
      m_if.filterVer( COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng );
    }
  }
}


void InterSearch::xEncodeInterResidualQT(CodingStructure &cs, Partitioner &partitioner, const ComponentID compID)
{
  const UnitArea& currArea    = partitioner.currArea();
  const TransformUnit& currTU = *cs.getTU(isLuma(partitioner.chType) ? currArea.lumaPos() : currArea.chromaPos(), partitioner.chType);
  const CodingUnit &cu        = *currTU.cu;
  const unsigned currDepth    = partitioner.currTrDepth;

  const bool bSubdiv          = currDepth != currTU.depth;

  if (compID == MAX_NUM_TBLOCKS)  // we are not processing a channel, instead we always recurse and code the CBFs
  {
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      CHECK( !bSubdiv, "Not performing the implicit TU split" );
    }
    else if( cu.sbtInfo && partitioner.canSplit( CU::getSbtTuSplit( cu.sbtInfo ), cs ) )
    {
      CHECK( !bSubdiv, "Not performing the implicit TU split - sbt" );
    }
    else
    {
      CHECK( bSubdiv, "transformsplit not supported" );
    }

    CHECK(CU::isIntra(cu), "Inter search provided with intra CU");

    if( cu.chromaFormat != CHROMA_400
      && (!CU::isSepTree(cu) || isChroma(partitioner.chType))
      )
    {
      {
        {
          const bool  chroma_cbf = TU::getCbfAtDepth( currTU, COMP_Cb, currDepth );
          if (!(cu.sbtInfo && (currDepth == 0 || (currDepth == 1 && currTU.noResidual))))
          m_CABACEstimator->cbf_comp( cu, chroma_cbf, currArea.blocks[COMP_Cb], currDepth );
        }
        {
          const bool  chroma_cbf = TU::getCbfAtDepth( currTU, COMP_Cr, currDepth );
          if (!(cu.sbtInfo && (currDepth == 0 || (currDepth == 1 && currTU.noResidual))))
          m_CABACEstimator->cbf_comp( cu, chroma_cbf, currArea.blocks[COMP_Cr], currDepth, TU::getCbfAtDepth( currTU, COMP_Cb, currDepth ) );
        }
      }
    }

    if( !bSubdiv && !( cu.sbtInfo && currTU.noResidual )
      && !isChroma(partitioner.chType)
      )
    {
      m_CABACEstimator->cbf_comp( cu, TU::getCbfAtDepth( currTU, COMP_Y, currDepth ), currArea.Y(), currDepth );
    }
  }

  if (!bSubdiv)
  {
    if (compID != MAX_NUM_TBLOCKS) // we have already coded the CBFs, so now we code coefficients
    {
      if( currArea.blocks[compID].valid() )
      {
        if( compID == COMP_Cr )
        {
          const int cbfMask = ( TU::getCbf( currTU, COMP_Cb ) ? 2 : 0) + ( TU::getCbf( currTU, COMP_Cr ) ? 1 : 0 );
          m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
        }
        if( TU::getCbf( currTU, compID ) )
        {
          m_CABACEstimator->residual_coding( currTU, compID );
        }
      }
    }
  }
  else
  {
    if( compID == MAX_NUM_TBLOCKS || TU::getCbfAtDepth( currTU, compID, currDepth ) )
    {
      if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
      {
        partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
      }
      else if( cu.sbtInfo && partitioner.canSplit( CU::getSbtTuSplit( cu.sbtInfo ), cs ) )
      {
        partitioner.splitCurrArea( CU::getSbtTuSplit( cu.sbtInfo ), cs );
      }
      else
        THROW( "Implicit TU split not available!" );

      do
      {
        xEncodeInterResidualQT( cs, partitioner, compID );
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
    }
  }
}

void InterSearch::xCalcMinDistSbt( CodingStructure &cs, const CodingUnit& cu, const uint8_t sbtAllowed )
{
  if( !sbtAllowed )
  {
    m_estMinDistSbt[NUMBER_SBT_MODE] = 0;
    for( int comp = 0; comp < getNumberValidTBlocks( *cs.pcv ); comp++ )
    {
      const ComponentID compID = ComponentID( comp );
      CPelBuf pred = cs.getPredBuf( compID );
      CPelBuf org  = cs.getOrgBuf( compID );
      m_estMinDistSbt[NUMBER_SBT_MODE] += m_pcRdCost->getDistPart( org, pred, cs.sps->bitDepths[ toChannelType( compID ) ], compID, DF_SSE );
    }
    return;
  }

  //SBT fast algorithm 2.1 : estimate a minimum RD cost of a SBT mode based on the luma distortion of uncoded part and coded part (assuming distorted can be reduced to 1/16);
  //                         if this cost is larger than the best cost, no need to try a specific SBT mode
  int cuWidth  = cu.lwidth();
  int cuHeight = cu.lheight();
  int numPartX = cuWidth  >= 16 ? 4 : ( cuWidth  == 4 ? 1 : 2 );
  int numPartY = cuHeight >= 16 ? 4 : ( cuHeight == 4 ? 1 : 2 );
  Distortion dist[4][4];
  memset( dist, 0, sizeof( Distortion ) * 16 );

  for( uint32_t c = 0; c < getNumberValidTBlocks( *cs.pcv ); c++ )
  {
    const ComponentID compID   = ComponentID( c );
    const CompArea&   compArea = cu.blocks[compID];
    const CPelBuf orgPel  = cs.getOrgBuf( compArea );
    const CPelBuf predPel = cs.getPredBuf( compArea );
    int lengthX = compArea.width / numPartX;
    int lengthY = compArea.height / numPartY;
    int strideOrg  = orgPel.stride;
    int stridePred = predPel.stride;
    uint32_t   uiShift = DISTORTION_PRECISION_ADJUSTMENT( ( *cs.sps.bitDepths[ toChannelType( compID ) ] - 8 ) << 1 );
    Intermediate_Int iTemp;

    //calc distY of 16 sub parts
    for( int j = 0; j < numPartY; j++ )
    {
      for( int i = 0; i < numPartX; i++ )
      {
        int posX = i * lengthX;
        int posY = j * lengthY;
        const Pel* ptrOrg  = orgPel.bufAt( posX, posY );
        const Pel* ptrPred = predPel.bufAt( posX, posY );
        Distortion uiSum = 0;
        for( int n = 0; n < lengthY; n++ )
        {
          for( int m = 0; m < lengthX; m++ )
          {
            iTemp = ptrOrg[m] - ptrPred[m];
            uiSum += Distortion( ( iTemp * iTemp ) >> uiShift );
          }
          ptrOrg += strideOrg;
          ptrPred += stridePred;
        }
        if( isChroma( compID ) )
        {
          uiSum = (Distortion)( uiSum * m_pcRdCost->getChromaWeight() );
        }
        dist[j][i] += uiSum;
      }
    }
  }

  //SSE of a CU
  m_estMinDistSbt[NUMBER_SBT_MODE] = 0;
  for( int j = 0; j < numPartY; j++ )
  {
    for( int i = 0; i < numPartX; i++ )
    {
      m_estMinDistSbt[NUMBER_SBT_MODE] += dist[j][i];
    }
  }
  //init per-mode dist
  for( int i = SBT_VER_H0; i < NUMBER_SBT_MODE; i++ )
  {
    m_estMinDistSbt[i] = MAX_DISTORTION;
  }

  //SBT fast algorithm 1: not try SBT if the residual is too small to compensate bits for encoding residual info
  uint64_t minNonZeroResiFracBits = 12 << SCALE_BITS;
  if( m_pcRdCost->calcRdCost( 0, m_estMinDistSbt[NUMBER_SBT_MODE] ) < m_pcRdCost->calcRdCost( minNonZeroResiFracBits, 0 ) )
  {
    m_skipSbtAll = true;
    return;
  }

  //derive estimated minDist of SBT = zero-residual part distortion + non-zero residual part distortion / 16
  int shift = 5;
  Distortion distResiPart = 0, distNoResiPart = 0;

  if( CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) )
  {
    int offsetResiPart = 0;
    int offsetNoResiPart = numPartX / 2;
    distResiPart = distNoResiPart = 0;
    assert( numPartX >= 2 );
    for( int j = 0; j < numPartY; j++ )
    {
      for( int i = 0; i < numPartX / 2; i++ )
      {
        distResiPart   += dist[j][i + offsetResiPart];
        distNoResiPart += dist[j][i + offsetNoResiPart];
      }
    }
    m_estMinDistSbt[SBT_VER_H0] = ( distResiPart >> shift ) + distNoResiPart;
    m_estMinDistSbt[SBT_VER_H1] = ( distNoResiPart >> shift ) + distResiPart;
  }

  if( CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed ) )
  {
    int offsetResiPart = 0;
    int offsetNoResiPart = numPartY / 2;
    assert( numPartY >= 2 );
    distResiPart = distNoResiPart = 0;
    for( int j = 0; j < numPartY / 2; j++ )
    {
      for( int i = 0; i < numPartX; i++ )
      {
        distResiPart   += dist[j + offsetResiPart][i];
        distNoResiPart += dist[j + offsetNoResiPart][i];
      }
    }
    m_estMinDistSbt[SBT_HOR_H0] = ( distResiPart >> shift ) + distNoResiPart;
    m_estMinDistSbt[SBT_HOR_H1] = ( distNoResiPart >> shift ) + distResiPart;
  }

  if( CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) )
  {
    assert( numPartX == 4 );
    m_estMinDistSbt[SBT_VER_Q0] = m_estMinDistSbt[SBT_VER_Q1] = 0;
    for( int j = 0; j < numPartY; j++ )
    {
      m_estMinDistSbt[SBT_VER_Q0] += dist[j][0] + ( ( dist[j][1] + dist[j][2] + dist[j][3] ) << shift );
      m_estMinDistSbt[SBT_VER_Q1] += dist[j][3] + ( ( dist[j][0] + dist[j][1] + dist[j][2] ) << shift );
    }
    m_estMinDistSbt[SBT_VER_Q0] = m_estMinDistSbt[SBT_VER_Q0] >> shift;
    m_estMinDistSbt[SBT_VER_Q1] = m_estMinDistSbt[SBT_VER_Q1] >> shift;
  }

  if( CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed ) )
  {
    assert( numPartY == 4 );
    m_estMinDistSbt[SBT_HOR_Q0] = m_estMinDistSbt[SBT_HOR_Q1] = 0;
    for( int i = 0; i < numPartX; i++ )
    {
      m_estMinDistSbt[SBT_HOR_Q0] += dist[0][i] + ( ( dist[1][i] + dist[2][i] + dist[3][i] ) << shift );
      m_estMinDistSbt[SBT_HOR_Q1] += dist[3][i] + ( ( dist[0][i] + dist[1][i] + dist[2][i] ) << shift );
    }
    m_estMinDistSbt[SBT_HOR_Q0] = m_estMinDistSbt[SBT_HOR_Q0] >> shift;
    m_estMinDistSbt[SBT_HOR_Q1] = m_estMinDistSbt[SBT_HOR_Q1] >> shift;
  }

  //SBT fast algorithm 5: try N SBT modes with the lowest distortion
  Distortion temp[NUMBER_SBT_MODE];
  memcpy( temp, m_estMinDistSbt, sizeof( Distortion ) * NUMBER_SBT_MODE );
  memset( m_sbtRdoOrder, 255, NUMBER_SBT_MODE );
  int startIdx = 0, numRDO;
  numRDO = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) + CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  numRDO = std::min( ( numRDO << 1 ), SBT_NUM_RDO );
  for( int i = startIdx; i < startIdx + numRDO; i++ )
  {
    Distortion minDist = MAX_DISTORTION;
    for( int n = SBT_VER_H0; n <= SBT_HOR_H1; n++ )
    {
      if( temp[n] < minDist )
      {
        minDist = temp[n];
        m_sbtRdoOrder[i] = n;
      }
    }
    temp[m_sbtRdoOrder[i]] = MAX_DISTORTION;
  }

  startIdx += numRDO;
  numRDO = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) + CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  numRDO = std::min( ( numRDO << 1 ), SBT_NUM_RDO );
  for( int i = startIdx; i < startIdx + numRDO; i++ )
  {
    Distortion minDist = MAX_DISTORTION;
    for( int n = SBT_VER_Q0; n <= SBT_HOR_Q1; n++ )
    {
      if( temp[n] < minDist )
      {
        minDist = temp[n];
        m_sbtRdoOrder[i] = n;
      }
    }
    temp[m_sbtRdoOrder[i]] = MAX_DISTORTION;
  }
}

uint8_t InterSearch::skipSbtByRDCost( int width, int height, int mtDepth, uint8_t sbtIdx, uint8_t sbtPos, double bestCost, Distortion distSbtOff, double costSbtOff, bool rootCbfSbtOff )
{
  int sbtMode = CU::getSbtMode( sbtIdx, sbtPos );

  //SBT fast algorithm 2.2 : estimate a minimum RD cost of a SBT mode based on the luma distortion of uncoded part and coded part (assuming distorted can be reduced to 1/16);
  //                         if this cost is larger than the best cost, no need to try a specific SBT mode
  if( m_pcRdCost->calcRdCost( 11 << SCALE_BITS, m_estMinDistSbt[sbtMode] ) > bestCost )
  {
    return 0; //early skip type 0
  }

  if( costSbtOff != MAX_DOUBLE )
  {
    if( !rootCbfSbtOff )
    {
      //SBT fast algorithm 3: skip SBT when the residual is too small (estCost is more accurate than fast algorithm 1, counting PU mode bits)
      uint64_t minNonZeroResiFracBits = 10 << SCALE_BITS;
      Distortion distResiPart;
      if( sbtIdx == SBT_VER_HALF || sbtIdx == SBT_HOR_HALF )
      {
        distResiPart = (Distortion)( ( ( m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[sbtMode] ) * 9 ) >> 4 );
      }
      else
      {
        distResiPart = (Distortion)( ( ( m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[sbtMode] ) * 3 ) >> 3 );
      }

      double estCost = ( costSbtOff - m_pcRdCost->calcRdCost( 0 << SCALE_BITS, distSbtOff ) ) + m_pcRdCost->calcRdCost( minNonZeroResiFracBits, m_estMinDistSbt[sbtMode] + distResiPart );
      if( estCost > costSbtOff )
      {
        return 1;
      }
      if( estCost > bestCost )
      {
        return 2;
      }
    }
    else
    {
      //SBT fast algorithm 4: skip SBT when an estimated RD cost is larger than the bestCost
      double weight = sbtMode > SBT_HOR_H1 ? 0.4 : 0.6;
      double estCost = ( ( costSbtOff - m_pcRdCost->calcRdCost( 0 << SCALE_BITS, distSbtOff ) ) * weight ) + m_pcRdCost->calcRdCost( 0 << SCALE_BITS, m_estMinDistSbt[sbtMode] );
      if( estCost > bestCost )
      {
        return 3;
      }
    }
  }
  return MAX_UCHAR;
}

void InterSearch::xEstimateInterResidualQT(CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist /*= NULL*/)
{
  const UnitArea& currArea = partitioner.currArea();
  const SPS &sps           = *cs.sps;

  const uint32_t numValidComp  = getNumberValidComponents( sps.chromaFormatIdc );
  const uint32_t numTBlocks    = getNumberValidTBlocks   ( *cs.pcv );
  CodingUnit& cu               = *cs.getCU(partitioner.chType, partitioner.treeType);
  const unsigned currDepth = partitioner.currTrDepth;
  const bool useTS = cs.picture->useScTS;

  bool bCheckFull  = !partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
  if( cu.sbtInfo && partitioner.canSplit( CU::getSbtTuSplit( cu.sbtInfo ), cs ) )
  {
    bCheckFull = false;
  }
  bool bCheckSplit = !bCheckFull;

  // get temporary data
  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;
  if (bCheckSplit)
  {
    csSplit = &cs;
  }
  else if (bCheckFull)
  {
    csFull = &cs;
  }

  Distortion uiSingleDist         = 0;
  Distortion uiSingleDistComp [3] = { 0, 0, 0 };

  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
  TempCtx       ctxBest   ( m_CtxCache );

  PelUnitBuf    orgResiBuf;
  orgResiBuf = m_tmpStorageLCU.getCompactBuf( currArea );
  orgResiBuf.copyFrom(cs.getResiBuf(currArea));

  if (bCheckFull)
  {
    ReshapeData& reshapeData = cs.picture->reshapeData;

    TransformUnit& tu = csFull->addTU(CS::getArea(cs, currArea, partitioner.chType, partitioner.treeType), partitioner.chType, &cu);
    tu.depth          = currDepth;
    tu.mtsIdx[COMP_Y] = MTS_DCT2_DCT2;
    tu.checkTuNoResidual( partitioner.currPartIdx() );
    if (cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag() && cs.picHeader->lmcsChromaResidualScale && !(CS::isDualITree(cs) && cs.slice->isIntra() && tu.cu->predMode == MODE_IBC))
    {
      tu.chromaAdj = reshapeData.calculateChromaAdjVpduNei(tu, tu.blocks[COMP_Y], tu.cu->treeType);
    }

    double minCost [MAX_NUM_TBLOCKS];

    m_CABACEstimator->resetBits();

    memset(m_pTempPel, 0, sizeof(Pel) * tu.Y().area()); // not necessary needed for inside of recursion (only at the beginning)

    for (uint32_t i = 0; i < numTBlocks; i++)
    {
      minCost[i] = MAX_DOUBLE;
    }

    CodingStructure &saveCS = *m_pSaveCS[0];
    saveCS.pcv     = cs.pcv;
    saveCS.picture = cs.picture;
    saveCS.area.repositionTo(currArea);
    saveCS.clearTUs();
    TransformUnit&  bestTU = saveCS.addTU(CS::getArea(cs, currArea, partitioner.chType, partitioner.treeType), partitioner.chType, nullptr);

    for( uint32_t c = 0; c < numTBlocks; c++ )
    {
      const ComponentID compID    = ComponentID(c);
      const CompArea&   compArea  = tu.blocks[compID];
      const int channelBitDepth   = sps.bitDepths[toChannelType(compID)];

      if( !tu.blocks[compID].valid() )
      {
        continue;
      }
      bool tsAllowed = useTS && TU::isTSAllowed(tu, compID) && (isLuma(compID) || (isChroma(compID) && m_pcEncCfg->m_useChromaTS));
      if (isChroma(compID) && tsAllowed && (tu.mtsIdx[COMP_Y] != MTS_SKIP))
      {
        tsAllowed = false;
      }
      uint8_t nNumTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
      std::vector<TrMode> trModes;

      if (nNumTransformCands > 1)
      {
        trModes.push_back(TrMode(0, true)); //DCT2
        //for a SBT-no-residual TU, the RDO process should be called once, in order to get the RD cost
        if ( !tu.noResidual )
        {
          trModes.push_back(TrMode(1, true));
        }
        else
        {
          nNumTransformCands--;
        }
      }
      bool isLast = true;
      for (int transformMode = 0; transformMode < nNumTransformCands; transformMode++)
      {
        const bool isFirstMode = transformMode == 0;

        // copy the original residual into the residual buffer
        csFull->getResiBuf(compArea).copyFrom(orgResiBuf.get(compID));


        m_CABACEstimator->getCtx() = ctxStart;
        m_CABACEstimator->resetBits();

        if (bestTU.mtsIdx[compID] == MTS_SKIP && m_pcEncCfg->m_TS)
        {
          continue;
        }
        tu.mtsIdx[compID] = transformMode ? trModes[transformMode].first : 0;

        const QpParam cQP(tu, compID);  // note: uses tu.transformSkip[compID]
        m_pcTrQuant->selectLambda(compID);

        const Slice& slice = *tu.cu->slice;
        if (slice.picHeader->lmcsEnabled && reshapeData.getCTUFlag() && isChroma(compID) && slice.picHeader->lmcsChromaResidualScale )
        {
          double cRescale = (double)(1 << CSCALE_FP_PREC) / (double)(tu.chromaAdj);
          m_pcTrQuant->scaleLambda( 1.0/(cRescale*cRescale) );
        }

        if ( sps.jointCbCr && isChroma( compID ) && ( tu.cu->cs->slice->sliceQp > 18 ) )
        {
          m_pcTrQuant->scaleLambda( 1.05 );
        }
        TCoeff     currAbsSum = 0;
        uint64_t   currCompFracBits = 0;
        Distortion currCompDist = 0;
        double     currCompCost = 0;
        uint64_t   nonCoeffFracBits = 0;
        Distortion nonCoeffDist = 0;
        double     nonCoeffCost = 0;

        if (slice.picHeader->lmcsEnabled && reshapeData.getCTUFlag() && isChroma(compID) && slice.picHeader->lmcsChromaResidualScale && tu.blocks[compID].width*tu.blocks[compID].height > 4 )
        {
          PelBuf resiBuf = csFull->getResiBuf(compArea);
          resiBuf.scaleSignal(tu.chromaAdj, 1, slice.clpRngs[compID]);
        }

        if (nNumTransformCands > 1)
        {
          if (transformMode == 0)
          {
            m_pcTrQuant->checktransformsNxN(tu, &trModes, 2, compID);
            tu.mtsIdx[compID] = trModes[0].first;
            if (!trModes[transformMode + 1].second)
            {
              nNumTransformCands = 1;
            }
          }
          m_pcTrQuant->transformNxN(tu, compID, cQP, currAbsSum, m_CABACEstimator->getCtx(), true);
        }
        else
        {
          m_pcTrQuant->transformNxN(tu, compID, cQP, currAbsSum, m_CABACEstimator->getCtx());
        }
        if (isFirstMode || (currAbsSum == 0))
        {
          const CPelBuf zeroBuf(m_pTempPel, compArea);
          const CPelBuf& orgResi = orgResiBuf.get(compID);

          nonCoeffDist = m_pcRdCost->getDistPart(zeroBuf, orgResi, channelBitDepth, compID, DF_SSE); // initialized with zero residual distortion

          if (!tu.noResidual)
          {
            const bool prevCbf = (compID == COMP_Cr ? tu.cbf[COMP_Cb] : false);
            m_CABACEstimator->cbf_comp(*tu.cu, false, compArea, currDepth, prevCbf);
          }

          nonCoeffFracBits = m_CABACEstimator->getEstFracBits();
          nonCoeffCost = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist, !m_pcEncCfg->m_lumaLevelToDeltaQPEnabled);
        }

        if ((puiZeroDist != NULL) && isFirstMode)
        {
          *puiZeroDist += nonCoeffDist; // initialized with zero residual distortion
        }

        if (currAbsSum > 0) //if non-zero coefficients are present, a residual needs to be derived for further prediction
        {
          if (isFirstMode)
          {
            m_CABACEstimator->getCtx() = ctxStart;
            m_CABACEstimator->resetBits();
          }

          const bool prevCbf = ( compID == COMP_Cr ? tu.cbf[COMP_Cb] : false );
          m_CABACEstimator->cbf_comp( *tu.cu, true, compArea, currDepth, prevCbf );
          if( compID == COMP_Cr )
          {
            const int cbfMask = ( tu.cbf[COMP_Cb] ? 2 : 0 ) + 1;
            m_CABACEstimator->joint_cb_cr( tu, cbfMask );
          }
          CUCtx cuCtx;
          cuCtx.isDQPCoded = true;
          cuCtx.isChromaQpAdjCoded = true;
          m_CABACEstimator->residual_coding(tu, compID, &cuCtx);
          m_CABACEstimator->mts_idx(cu, &cuCtx);

          currCompFracBits = m_CABACEstimator->getEstFracBits();

          PelBuf resiBuf  = csFull->getResiBuf(compArea);
          CPelBuf orgResi = orgResiBuf.get(compID);

          m_pcTrQuant->invTransformNxN(tu, compID, resiBuf, cQP);
          if (slice.picHeader->lmcsEnabled && isChroma(compID) && slice.picHeader->lmcsChromaResidualScale && tu.blocks[compID].width*tu.blocks[compID].height > 4)
          {
            resiBuf.scaleSignal(tu.chromaAdj, 0, slice.clpRngs[compID]);
          }

          currCompDist = m_pcRdCost->getDistPart(orgResi, resiBuf, channelBitDepth, compID, DF_SSE);
          currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist, false);
        }
        else if (transformMode > 0)
        {
          currCompCost = MAX_DOUBLE;
        }
        else
        {
          currCompFracBits = nonCoeffFracBits;
          currCompDist     = nonCoeffDist;
          currCompCost     = nonCoeffCost;

          tu.cbf[compID] = 0;
        }

        // evaluate
        if ((currCompCost < minCost[compID]) || (transformMode == 1 && currCompCost == minCost[compID]))
        {
          // copy component
          if (isFirstMode && ((nonCoeffCost < currCompCost) || (currAbsSum == 0))) // check for forced null
          {
            tu.getCoeffs( compID ).fill( 0 );
            csFull->getResiBuf( compArea ).fill( 0 );
            tu.cbf[compID]   = 0;

            currAbsSum       = 0;
            currCompFracBits = nonCoeffFracBits;
            currCompDist     = nonCoeffDist;
            currCompCost     = nonCoeffCost;
          }

          uiSingleDistComp[compID] = currCompDist;
          minCost[compID]          = currCompCost;
          if (transformMode != (nNumTransformCands - 1))
          {
            bestTU.copyComponentFrom(tu, compID);
            saveCS.getResiBuf(compArea).copyFrom(csFull->getResiBuf(compArea));
          }
          else
          {
            isLast = false;
          }
        }
        if( tu.noResidual )
        {
          CHECK( currCompFracBits > 0 || currAbsSum, "currCompFracBits > 0 when tu noResidual" );
        }
      }
      if (isLast)
      {
        tu.copyComponentFrom(bestTU, compID);
        csFull->getResiBuf(compArea).copyFrom(saveCS.getResiBuf(compArea));
      }
    } // component loop

    if ( tu.blocks[COMP_Cb].valid() )
    {
      const CompArea& cbArea = tu.blocks[COMP_Cb];
      const CompArea& crArea = tu.blocks[COMP_Cr];
      bool checkJointCbCr = (sps.jointCbCr) && (!tu.noResidual) && (TU::getCbf(tu, COMP_Cb) || TU::getCbf(tu, COMP_Cr));
      const int channelBitDepth = sps.bitDepths[toChannelType(COMP_Cb)];
      const Slice& slice = *tu.cu->slice;
      bool      reshape         = slice.picHeader->lmcsEnabled && reshapeData.getCTUFlag() && slice.picHeader->lmcsChromaResidualScale
                               && tu.blocks[COMP_Cb].width * tu.blocks[COMP_Cb].height > 4;
      double minCostCbCr = minCost[COMP_Cb] + minCost[COMP_Cr];
      bool   isLastBest  = false;

      bool checkDCTOnly = m_pcEncCfg->m_useChromaTS && ((TU::getCbf(tu, COMP_Cb) && tu.mtsIdx[COMP_Cb] == MTS_DCT2_DCT2 && !TU::getCbf(tu, COMP_Cr)) ||
        (TU::getCbf(tu, COMP_Cr) && tu.mtsIdx[COMP_Cr] == MTS_DCT2_DCT2 && !TU::getCbf(tu, COMP_Cb)) ||
        (TU::getCbf(tu, COMP_Cb) && tu.mtsIdx[COMP_Cb] == MTS_DCT2_DCT2 && TU::getCbf(tu, COMP_Cr) && tu.mtsIdx[COMP_Cr] == MTS_DCT2_DCT2));
      bool checkTSOnly = m_pcEncCfg->m_useChromaTS && ((TU::getCbf(tu, COMP_Cb) && tu.mtsIdx[COMP_Cb] == MTS_SKIP && !TU::getCbf(tu, COMP_Cr)) ||
        (TU::getCbf(tu, COMP_Cr) && tu.mtsIdx[COMP_Cr] == MTS_SKIP && !TU::getCbf(tu, COMP_Cb)) ||
        (TU::getCbf(tu, COMP_Cb) && tu.mtsIdx[COMP_Cb] == MTS_SKIP && TU::getCbf(tu, COMP_Cr) && tu.mtsIdx[COMP_Cr] == MTS_SKIP));

      CompStorage      orgResiCb[4], orgResiCr[4];   // 0:std, 1-3:jointCbCr
      std::vector<int> jointCbfMasksToTest;
      if ( checkJointCbCr )
      {
        orgResiCb[0].create(cbArea);
        orgResiCr[0].create(crArea);
        orgResiCb[0].copyFrom(orgResiBuf.Cb());
        orgResiCr[0].copyFrom(orgResiBuf.Cr());
        if (reshape)
        {
          orgResiCb[0].scaleSignal(tu.chromaAdj, 1, slice.clpRngs[COMP_Cb]);
          orgResiCr[0].scaleSignal(tu.chromaAdj, 1, slice.clpRngs[COMP_Cr]);
        }

        jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(tu, orgResiCb, orgResiCr);

        bestTU.copyComponentFrom(tu, COMP_Cb);
        bestTU.copyComponentFrom(tu, COMP_Cr);
        saveCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
        saveCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
      }

      for (int cbfMask: jointCbfMasksToTest)
      {
        ComponentID codeCompId = (cbfMask >> 1 ? COMP_Cb : COMP_Cr);
        ComponentID otherCompId = (codeCompId == COMP_Cr ? COMP_Cb : COMP_Cr);
        bool tsAllowed = useTS && TU::isTSAllowed(tu, codeCompId) && (m_pcEncCfg->m_useChromaTS);
        if (tsAllowed && (tu.mtsIdx[COMP_Y] != MTS_SKIP))
        {
          tsAllowed = false;
        }
        if (!tsAllowed)
        {
          checkTSOnly = false;
        }
        uint8_t     numTransformCands = 1 + (tsAllowed && (!(checkDCTOnly || checkTSOnly)) ? 1 : 0); // DCT + TS = 2 tests
        std::vector<TrMode> trModes;
        if (numTransformCands > 1)
        {
          trModes.push_back(TrMode(0, true)); // DCT2
          trModes.push_back(TrMode(1, true));//TS
        }
        else
        {
          tu.mtsIdx[codeCompId] = checkTSOnly ? 1 : 0;
        }
        for (int modeId = 0; modeId < numTransformCands; modeId++)
        {
          TCoeff     currAbsSum = 0;
          uint64_t   currCompFracBits = 0;
          Distortion currCompDistCb = 0;
          Distortion currCompDistCr = 0;
          double     currCompCost = 0;

          tu.jointCbCr = (uint8_t)cbfMask;
          if (numTransformCands > 1)
          {
            tu.mtsIdx[codeCompId] = trModes[modeId].first;
          }
          tu.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
          const QpParam cQP(tu, COMP_Cb);  // note: uses tu.transformSkip[compID]
          m_pcTrQuant->selectLambda(COMP_Cb);

          // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
          const int    absIct = abs(TU::getICTMode(tu));
          const double lfact = (absIct == 1 || absIct == 3 ? 0.8 : 0.5);
          m_pcTrQuant->scaleLambda(lfact);
          if (checkJointCbCr && (tu.cu->cs->slice->sliceQp > 18))
          {
            m_pcTrQuant->scaleLambda(1.05);
          }

          m_CABACEstimator->getCtx() = ctxStart;
          m_CABACEstimator->resetBits();

          PelBuf cbResi = csFull->getResiBuf(cbArea);
          PelBuf crResi = csFull->getResiBuf(crArea);
          cbResi.copyFrom(orgResiCb[cbfMask]);
          crResi.copyFrom(orgResiCr[cbfMask]);

          if (reshape)
          {
            double cRescale = (double)(1 << CSCALE_FP_PREC) / (double)(tu.chromaAdj);
            m_pcTrQuant->scaleLambda(1.0 / (cRescale * cRescale));
          }

          int         codedCbfMask = 0;
          ComponentID codeCompId = (tu.jointCbCr >> 1 ? COMP_Cb : COMP_Cr);
          ComponentID otherCompId = (codeCompId == COMP_Cr ? COMP_Cb : COMP_Cr);
          const QpParam qpCbCr(tu, codeCompId);

          tu.getCoeffs(otherCompId).fill(0);   // do we need that?
          TU::setCbfAtDepth(tu, otherCompId, tu.depth, false);

          PelBuf& codeResi = (codeCompId == COMP_Cr ? crResi : cbResi);
          TCoeff  compAbsSum = 0;
          if (numTransformCands > 1)
          {
            if (modeId == 0)
            {
              m_pcTrQuant->checktransformsNxN(tu, &trModes, 2, codeCompId);
              tu.mtsIdx[codeCompId] = trModes[modeId].first;
              tu.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
              if (!trModes[modeId + 1].second)
              {
                numTransformCands = 1;
              }
            }
            m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, compAbsSum, m_CABACEstimator->getCtx(), true);
          }
          else
          {
            m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, compAbsSum, m_CABACEstimator->getCtx());
          }
          if (compAbsSum > 0)
          {
            m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
            codedCbfMask += (codeCompId == COMP_Cb ? 2 : 1);
          }
          else
          {
            codeResi.fill(0);
          }

          if (tu.jointCbCr == 3 && codedCbfMask == 2)
          {
            codedCbfMask = 3;
            TU::setCbfAtDepth(tu, COMP_Cr, tu.depth, true);
          }
          if (codedCbfMask && tu.jointCbCr != codedCbfMask)
          {
            codedCbfMask = 0;
          }
          currAbsSum = codedCbfMask;
          if (!tu.mtsIdx[codeCompId])
          {
            numTransformCands = (currAbsSum <= 0) ? 1 : numTransformCands;
          }
          if (currAbsSum > 0)
          {
            m_CABACEstimator->cbf_comp(*tu.cu, codedCbfMask >> 1, cbArea, currDepth, false);
            m_CABACEstimator->cbf_comp(*tu.cu, codedCbfMask & 1, crArea, currDepth, codedCbfMask >> 1);
            m_CABACEstimator->joint_cb_cr(tu, codedCbfMask);
            if (codedCbfMask >> 1)
              m_CABACEstimator->residual_coding(tu, COMP_Cb);
            if (codedCbfMask & 1)
              m_CABACEstimator->residual_coding(tu, COMP_Cr);
            currCompFracBits = m_CABACEstimator->getEstFracBits();

            m_pcTrQuant->invTransformICT(tu, cbResi, crResi);
            if (reshape)
            {
              cbResi.scaleSignal(tu.chromaAdj, 0, slice.clpRngs[COMP_Cb]);
              crResi.scaleSignal(tu.chromaAdj, 0, slice.clpRngs[COMP_Cr]);
            }

            currCompDistCb = m_pcRdCost->getDistPart(orgResiBuf.Cb(), cbResi, channelBitDepth, COMP_Cb, DF_SSE);
            currCompDistCr = m_pcRdCost->getDistPart(orgResiBuf.Cr(), crResi, channelBitDepth, COMP_Cr, DF_SSE);
            currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDistCr + currCompDistCb, false);
          }
          else
            currCompCost = MAX_DOUBLE;

          // evaluate
          if (currCompCost < minCostCbCr)
          {
            uiSingleDistComp[COMP_Cb] = currCompDistCb;
            uiSingleDistComp[COMP_Cr] = currCompDistCr;
            minCostCbCr = currCompCost;
            isLastBest = (cbfMask == jointCbfMasksToTest.back()) && (modeId == (numTransformCands - 1));
            if (!isLastBest)
            {
              bestTU.copyComponentFrom(tu, COMP_Cb);
              bestTU.copyComponentFrom(tu, COMP_Cr);
              saveCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
              saveCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
            }
          }
        }

        if( !isLastBest )
        {
          // copy component
          tu.copyComponentFrom( bestTU, COMP_Cb );
          tu.copyComponentFrom( bestTU, COMP_Cr );
          csFull->getResiBuf( cbArea ).copyFrom( saveCS.getResiBuf( cbArea ) );
          csFull->getResiBuf( crArea ).copyFrom( saveCS.getResiBuf( crArea ) );
        }
      }
    }

    m_CABACEstimator->getCtx() = ctxStart;
    m_CABACEstimator->resetBits();
    if( !tu.noResidual )
    {
      static const ComponentID cbf_getComp[3] = { COMP_Cb, COMP_Cr, COMP_Y };
      for( unsigned c = 0; c < numTBlocks; c++)
      {
        const ComponentID compID = cbf_getComp[c];
        if( tu.blocks[compID].valid() )
        {
          const bool prevCbf = ( compID == COMP_Cr ? TU::getCbfAtDepth( tu, COMP_Cb, currDepth ) : false );
          m_CABACEstimator->cbf_comp( *tu.cu, TU::getCbfAtDepth( tu, compID, currDepth ), tu.blocks[compID], currDepth, prevCbf );
        }
      }
    }

    for (uint32_t ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID = ComponentID(ch);
      if (tu.blocks[compID].valid())
      {
        if( compID == COMP_Cr )
        {
          const int cbfMask = ( TU::getCbf( tu, COMP_Cb ) ? 2 : 0 ) + ( TU::getCbf( tu, COMP_Cr ) ? 1 : 0 );
          m_CABACEstimator->joint_cb_cr(tu, cbfMask);
        }
        if( TU::getCbf( tu, compID ) )
        {
          m_CABACEstimator->residual_coding( tu, compID );
        }
        uiSingleDist += uiSingleDistComp[compID];
      }
    }
    if( tu.noResidual )
    {
      CHECK( m_CABACEstimator->getEstFracBits() > 0, "no residual TU's bits shall be 0" );
    }

    csFull->fracBits += m_CABACEstimator->getEstFracBits();
    csFull->dist     += uiSingleDist;
    csFull->cost      = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist, !m_pcEncCfg->m_lumaLevelToDeltaQPEnabled);
  } // check full

  // code sub-blocks
  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( cu.sbtInfo && partitioner.canSplit( CU::getSbtTuSplit( cu.sbtInfo ), cs ) )
    {
      partitioner.splitCurrArea( CU::getSbtTuSplit( cu.sbtInfo ), cs );
    }
    else
      THROW( "Implicit TU split not available!" );

    do
    {
      xEstimateInterResidualQT(*csSplit, partitioner, bCheckFull ? nullptr : puiZeroDist );

      csSplit->cost = m_pcRdCost->calcRdCost( csSplit->fracBits, csSplit->dist );
    } while( partitioner.nextPart( *csSplit ) );

    partitioner.exitCurrSplit();

    unsigned        anyCbfSet   =   0;
    unsigned        compCbf[3]  = { 0, 0, 0 };

    if( !bCheckFull )
    {
      for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
      {
        for( unsigned ch = 0; ch < numTBlocks; ch++ )
        {
          compCbf[ ch ] |= ( TU::getCbfAtDepth( currTU, ComponentID(ch), currDepth + 1 ) ? 1 : 0 );
        }
      }

      for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
      {
        TU::setCbfAtDepth   ( currTU, COMP_Y,  currDepth, compCbf[ COMP_Y  ] );
        if( currArea.chromaFormat != CHROMA_400 )
        {
          TU::setCbfAtDepth ( currTU, COMP_Cb, currDepth, compCbf[ COMP_Cb ] );
          TU::setCbfAtDepth ( currTU, COMP_Cr, currDepth, compCbf[ COMP_Cr ] );
        }
      }

      anyCbfSet    = compCbf[ COMP_Y  ];
      if( currArea.chromaFormat != CHROMA_400 )
      {
        anyCbfSet |= compCbf[ COMP_Cb ];
        anyCbfSet |= compCbf[ COMP_Cr ];
      }

      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      // when compID isn't a channel, code Cbfs:
      xEncodeInterResidualQT( *csSplit, partitioner, MAX_NUM_TBLOCKS );

      for (uint32_t ch = 0; ch < numValidComp; ch++)
      {
        const ComponentID compID = ComponentID(ch);
        xEncodeInterResidualQT( *csSplit, partitioner, compID );
      }

      csSplit->fracBits = m_CABACEstimator->getEstFracBits();
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      if( bCheckFull && anyCbfSet && csSplit->cost < csFull->cost )
      {
        cs.useSubStructure( *csSplit, partitioner.chType, partitioner.treeType, currArea, false );
        cs.cost = csSplit->cost;
      }
    }

    if( csSplit && csFull )
    {
      csSplit->releaseIntermediateData();
      csFull ->releaseIntermediateData();
    }
  }
}

void InterSearch::encodeResAndCalcRdInterCU(CodingStructure &cs, Partitioner &partitioner, const bool skipResidual )
{
  CodingUnit &cu = *cs.getCU( partitioner.chType, partitioner.treeType );
  bool luma      = true;
  bool chroma    = true;
  if (cu.predMode == MODE_IBC)
  {
    luma   = cu.mcControl <= 3;
    chroma = (cu.mcControl >> 1) != 1;
  }
  if( cu.predMode == MODE_INTER )
    CHECK( CU::isSepTree(cu), "CU with Inter mode must be in single tree" );

  const ChromaFormat format      = cs.area.chromaFormat;;
  const int  numValidComponents  = getNumberValidComponents(format);
  const SPS &sps                 = *cs.sps;
  const ReshapeData& reshapeData = cs.picture->reshapeData;

  if( skipResidual ) //  No residual coding : SKIP mode
  {
    cu.skip    = true;
    cu.rootCbf = false;
    CHECK( cu.sbtInfo != 0, "sbtInfo shall be 0 if CU has no residual" );
    cs.getResiBuf().fill(0);
    cs.getRecoBuf().copyFrom(cs.getPredBuf() );
    if( cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag() && !cu.ciip && !CU::isIBC(cu))
    {
      cs.getRecoBuf().Y().rspSignal( reshapeData.getFwdLUT());
    }

    // add new "empty" TU(s) spanning the whole CU
    cs.addEmptyTUs( partitioner, &cu );
    Distortion distortion = 0;

    for (int comp = 0; comp < numValidComponents; comp++)
    {
      const ComponentID compID = ComponentID(comp);
      if (compID == COMP_Y && !luma)
        continue;
      if (compID != COMP_Y && !chroma)
        continue;
      CPelBuf reco = cs.getRecoBuf (compID);
      CPelBuf org  = cs.getOrgBuf  (compID);
      if ((cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag()) || m_pcEncCfg->m_lumaLevelToDeltaQPEnabled )
      {
        const CompArea& areaY = cu.Y();
        const CPelBuf orgLuma = cs.getOrgBuf( areaY );
        if (compID == COMP_Y && !m_pcEncCfg->m_lumaLevelToDeltaQPEnabled )
        {
          PelBuf tmpRecLuma = cs.getRspRecoBuf();
          tmpRecLuma.rspSignal(reco, reshapeData.getInvLUT());
          distortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.bitDepths[ CH_L ], compID, DF_SSE_WTD, &orgLuma);
        }
        else
          distortion += m_pcRdCost->getDistPart( org, reco, sps.bitDepths[ CH_C ], compID, DF_SSE_WTD, &orgLuma );
      }
      else
      {
        distortion  += m_pcRdCost->getDistPart( org, reco, sps.bitDepths[ toChannelType( compID ) ], compID, DF_SSE );
      }
    }

    CodingUnit& cu = *cs.getCU(partitioner.chType, TREE_D);
    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_skip_flag  ( cu );
    m_CABACEstimator->merge_data(cu);
    cs.fracBits = m_CABACEstimator->getEstFracBits();
    cs.dist     = distortion;
    cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

    return;
  }

  //  Residual coding.
  if (luma)
  {
    if (cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag())
    {
      if (!cu.ciip && !CU::isIBC(cu))
      {
        const CompArea& areaY = cu.Y();
        PelBuf tmpPred = m_tmpStorageLCU.getCompactBuf(areaY);
        tmpPred.rspSignal(cs.getPredBuf(COMP_Y), reshapeData.getFwdLUT());
        cs.getResiBuf(COMP_Y).subtract(cs.getRspOrgBuf(), tmpPred);
      }
      else
      {
        cs.getResiBuf(COMP_Y).subtract(cs.getRspOrgBuf(), cs.getPredBuf(COMP_Y));
      }
    }
    else
    {
      cs.getResiBuf(COMP_Y).subtract(cs.getOrgBuf(COMP_Y), cs.getPredBuf(COMP_Y));
    }
  }
  if (chroma)
  {
    cs.getResiBuf(COMP_Cb).subtract(cs.getOrgBuf(COMP_Cb), cs.getPredBuf(COMP_Cb));
    cs.getResiBuf(COMP_Cr).subtract(cs.getOrgBuf(COMP_Cr), cs.getPredBuf(COMP_Cr));
  }

  Distortion zeroDistortion = 0;

  const TempCtx ctxStart( m_CtxCache, m_CABACEstimator->getCtx() );

  xEstimateInterResidualQT(cs, partitioner, &zeroDistortion );
  TransformUnit& firstTU = *cs.getTU( partitioner.chType );

  cu.rootCbf = false;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->rqt_root_cbf( cu );
  const uint64_t  zeroFracBits = m_CABACEstimator->getEstFracBits();
  double zeroCost = m_pcRdCost->calcRdCost( zeroFracBits, zeroDistortion, !m_pcEncCfg->m_lumaLevelToDeltaQPEnabled );

  const int  numValidTBlocks   = getNumberValidTBlocks( *cs.pcv );
  for (uint32_t i = 0; i < numValidTBlocks; i++)
  {
    cu.rootCbf |= TU::getCbfAtDepth(firstTU, ComponentID(i), 0);
  }

  // -------------------------------------------------------
  // If a block full of 0's is efficient, then just use 0's.
  // The costs at this point do not include header bits.

  if (zeroCost < cs.cost || !cu.rootCbf)
  {
    cu.sbtInfo = 0;
    cu.rootCbf = false;

    cs.clearTUs();

    // add a new "empty" TU spanning the whole CU
    cs.addEmptyTUs( partitioner, &cu );
  }

  // all decisions now made. Fully encode the CU, including the headers:
  m_CABACEstimator->getCtx() = ctxStart;

  uint64_t finalFracBits = xGetSymbolFracBitsInter( cs, partitioner );
  // we've now encoded the CU, and so have a valid bit cost
  if (!cu.rootCbf)
  {
    if (luma)
    {
      cs.getResiBuf().bufs[0].fill(0); // Clear the residual image, if we didn't code it.
    }
    if (chroma && isChromaEnabled(cs.pcv->chrFormat))
    {
      cs.getResiBuf().bufs[1].fill(0); // Clear the residual image, if we didn't code it.
      cs.getResiBuf().bufs[2].fill(0); // Clear the residual image, if we didn't code it.
    }
  }
  if (luma)
  {
    if (cu.rootCbf && cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag())
    {
      if (!cu.ciip && !CU::isIBC(cu))
      {
        PelBuf tmpPred = m_tmpStorageLCU.getCompactBuf(cu.Y());
        tmpPred.rspSignal(cs.getPredBuf(COMP_Y), reshapeData.getFwdLUT());
        cs.getRecoBuf(COMP_Y).reconstruct(tmpPred, cs.getResiBuf(COMP_Y), cs.slice->clpRngs[COMP_Y]);
      }
      else
      {
        cs.getRecoBuf(COMP_Y).reconstruct(cs.getPredBuf(COMP_Y), cs.getResiBuf(COMP_Y), cs.slice->clpRngs[COMP_Y]);
      }
    }
    else
    {
      cs.getRecoBuf().bufs[0].reconstruct(cs.getPredBuf().bufs[0], cs.getResiBuf().bufs[0], cs.slice->clpRngs[COMP_Y]);
      if (cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag() && !cu.ciip && !CU::isIBC(cu))
      {
        cs.getRecoBuf().bufs[0].rspSignal(reshapeData.getFwdLUT());
      }
    }
  }
  if (chroma)
  {
    cs.getRecoBuf().bufs[1].reconstruct(cs.getPredBuf().bufs[1], cs.getResiBuf().bufs[1], cs.slice->clpRngs[COMP_Cb]);
    cs.getRecoBuf().bufs[2].reconstruct(cs.getPredBuf().bufs[2], cs.getResiBuf().bufs[2], cs.slice->clpRngs[COMP_Cr]);
  }
  // update with clipped distortion and cost (previously unclipped reconstruction values were used)
  Distortion finalDistortion = 0;

  for (int comp = 0; comp < numValidComponents; comp++)
  {
    const ComponentID compID = ComponentID(comp);
    if (compID == COMP_Y && !luma)
      continue;
    if (compID != COMP_Y && !chroma)
      continue;
    CPelBuf reco = cs.getRecoBuf (compID);
    CPelBuf org  = cs.getOrgBuf  (compID);

    if( (cs.picHeader->lmcsEnabled && reshapeData.getCTUFlag()) || m_pcEncCfg->m_lumaLevelToDeltaQPEnabled )
    {
      const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMP_Y] );
      if (compID == COMP_Y && !m_pcEncCfg->m_lumaLevelToDeltaQPEnabled )
      {
        PelBuf tmpRecLuma = cs.getRspRecoBuf();
        tmpRecLuma.rspSignal( reco, reshapeData.getInvLUT());
        finalDistortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.bitDepths[toChannelType(compID)], compID, DF_SSE_WTD, &orgLuma);
      }
      else
      {
        finalDistortion += m_pcRdCost->getDistPart(org, reco, sps.bitDepths[toChannelType(compID)], compID, DF_SSE_WTD, &orgLuma);
      }
    }
    else
    {
      finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.bitDepths[toChannelType(compID)], compID, DF_SSE );
    }
  }

  cs.dist     = finalDistortion;
  cs.fracBits = finalFracBits;
  cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

  CHECK(cs.tus.size() == 0, "No TUs present");
}

uint64_t InterSearch::xGetSymbolFracBitsInter(CodingStructure &cs, Partitioner &partitioner)
{
  uint64_t fracBits   = 0;
  CodingUnit &cu    = *cs.getCU( partitioner.chType, partitioner.treeType );

  m_CABACEstimator->resetBits();

  if( cu.mergeFlag && !cu.rootCbf )
  {
    cu.skip = true;

    m_CABACEstimator->cu_skip_flag  ( cu );
    if (!cu.ciip)
    {
      m_CABACEstimator->merge_data(cu);
    }
    fracBits   += m_CABACEstimator->getEstFracBits();
  }
  else
  {
    CHECK( cu.skip, "Skip flag has to be off at this point!" );

    if (cu.Y().valid())
    m_CABACEstimator->cu_skip_flag( cu );
    m_CABACEstimator->pred_mode   ( cu );
    m_CABACEstimator->cu_pred_data( cu );
    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual ( cu, partitioner, cuCtx );
    fracBits       += m_CABACEstimator->getEstFracBits();
  }

  return fracBits;
}

double InterSearch::xGetMEDistortionWeight(uint8_t BcwIdx, RefPicList refPicList)
{
  return 0.5;
}

void InterSearch::xSymMvdCheckBestMvp(
  CodingUnit& cu,
  CPelUnitBuf& origBuf,
  Mv curMv,
  RefPicList curRefList,
  AMVPInfo amvpInfo[2][MAX_REF_PICS],
  int32_t BcwIdx,
  Mv cMvPredSym[2],
  int32_t mvpIdxSym[2],
  Distortion& bestCost,
  bool skip
)
{
  RefPicList tarRefList = (RefPicList)(1 - curRefList);
  int32_t refIdxCur = cu.slice->symRefIdx[curRefList];
  int32_t refIdxTar = cu.slice->symRefIdx[tarRefList];

  MvField cCurMvField, cTarMvField;
  cCurMvField.setMvField(curMv, refIdxCur);
  AMVPInfo& amvpCur = amvpInfo[curRefList][refIdxCur];
  AMVPInfo& amvpTar = amvpInfo[tarRefList][refIdxTar];
  m_pcRdCost->setCostScale(0);

  double fWeight = 0.0;
  PelUnitBuf bufTmp;

  // get prediction of eCurRefPicList
  PelUnitBuf predBufA = m_tmpPredStorage[curRefList].getCompactBuf( cu );
  const Picture* picRefA = cu.slice->getRefPic(curRefList, cCurMvField.refIdx);
  Mv mvA = cCurMvField.mv;
  clipMv( mvA, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv );
  xPredInterBlk( COMP_Y, cu, picRefA, mvA, predBufA, false, cu.slice->clpRngs[ COMP_Y ], false, false );

  bufTmp = m_tmpStorageLCU.getBuf( UnitAreaRelative( cu, cu ) );
  bufTmp.copyFrom( origBuf );
  bufTmp.removeHighFreq( predBufA, m_pcEncCfg->m_bClipForBiPredMeEnabled, cu.slice->clpRngs/*, getBcwWeight( cu.BcwIdx, tarRefList )*/ );
  fWeight = xGetMEDistortionWeight( cu.BcwIdx, tarRefList );

  int32_t skipMvpIdx[2];
  skipMvpIdx[0] = skip ? mvpIdxSym[0] : -1;
  skipMvpIdx[1] = skip ? mvpIdxSym[1] : -1;

  for (int i = 0; i < amvpCur.numCand; i++)
  {
    for (int j = 0; j < amvpTar.numCand; j++)
    {
      if (skipMvpIdx[curRefList] == i && skipMvpIdx[tarRefList] == j)
        continue;

      Distortion cost = MAX_DISTORTION;
      cTarMvField.setMvField(curMv.getSymmvdMv(amvpCur.mvCand[i], amvpTar.mvCand[j]), refIdxTar);

      // get prediction of eTarRefPicList
      PelUnitBuf predBufB = m_tmpPredStorage[tarRefList].getCompactBuf( cu );
      const Picture* picRefB = cu.slice->getRefPic(tarRefList, cTarMvField.refIdx);
      Mv mvB = cTarMvField.mv;
      clipMv( mvB, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv );
      xPredInterBlk( COMP_Y, cu, picRefB, mvB, predBufB, false, cu.slice->clpRngs[ COMP_Y ], false, false );

        // calc distortion
      DFunc distFunc = ( !cu.slice->disableSATDForRd ) ? DF_HAD : DF_SAD;
      cost = ( Distortion ) floor( fWeight * ( double ) m_pcRdCost->getDistPart( bufTmp.Y(), predBufB.Y(), cu.cs->sps->bitDepths[ CH_L ], COMP_Y, distFunc ) );

      Mv pred = amvpCur.mvCand[i];
      pred.changeTransPrecInternal2Amvr(cu.imv);
      m_pcRdCost->setPredictor(pred);
      Mv mv = curMv;
      mv.changeTransPrecInternal2Amvr(cu.imv);
      uint32_t bits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
      bits += m_auiMVPIdxCost[i][AMVP_MAX_NUM_CANDS];
      bits += m_auiMVPIdxCost[j][AMVP_MAX_NUM_CANDS];
      cost += m_pcRdCost->getCost(bits);
      if (cost < bestCost)
      {
        bestCost = cost;
        cMvPredSym[curRefList] = amvpCur.mvCand[i];
        cMvPredSym[tarRefList] = amvpTar.mvCand[j];
        mvpIdxSym[curRefList] = i;
        mvpIdxSym[tarRefList] = j;
      }
    }
  }
}

void InterSearch::resetSavedAffineMotion()
{
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      m_affineMotion.acMvAffine4Para[i][j] = Mv(0, 0);
      m_affineMotion.acMvAffine6Para[i][j] = Mv(0, 0);
    }
    m_affineMotion.acMvAffine6Para[i][2] = Mv(0, 0);

    m_affineMotion.affine4ParaRefIdx[i] = -1;
    m_affineMotion.affine6ParaRefIdx[i] = -1;
  }
  m_affineMotion.affine4ParaAvail = false;
  m_affineMotion.affine6ParaAvail = false;
}

void InterSearch::storeAffineMotion(Mv acAffineMv[2][3], int16_t affineRefIdx[2], EAffineModel affineType, int BcwIdx)
{
  if ((BcwIdx == BCW_DEFAULT || !m_affineMotion.affine6ParaAvail) && affineType == AFFINEMODEL_6PARAM)
  {
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        m_affineMotion.acMvAffine6Para[i][j] = acAffineMv[i][j];
      }
      m_affineMotion.affine6ParaRefIdx[i] = affineRefIdx[i];
    }
    m_affineMotion.affine6ParaAvail = true;
  }

  if ((BcwIdx == BCW_DEFAULT || !m_affineMotion.affine4ParaAvail) && affineType == AFFINEMODEL_4PARAM)
  {
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        m_affineMotion.acMvAffine4Para[i][j] = acAffineMv[i][j];
      }
      m_affineMotion.affine4ParaRefIdx[i] = affineRefIdx[i];
    }
    m_affineMotion.affine4ParaAvail = true;
  }
}

void InterSearch::xPredAffineInterSearch( CodingUnit& cu,
                                          CPelUnitBuf&    origBuf,
                                          int             puIdx,
                                          uint32_t&       lastMode,
                                          Distortion&     affineCost,
                                          Mv              hevcMv[2][MAX_REF_PICS],
                                          Mv              mvAffine4Para[2][MAX_REF_PICS][3],
                                          int             refIdx4Para[2],
                                          uint8_t         BcwIdx,
                                          bool            enforceBcwPred,
                                          uint32_t        BcwIdxBits )
{
  const Slice &slice = *cu.slice;

  affineCost = MAX_DISTORTION;

  Mv        cMvZero;
  Mv        aacMv[2][3];
  Mv        cMvBi[2][3];
  AffineMVInfo tmp;

  int       iNumPredDir = slice.isInterP() ? 1 : 2;

  int mvNum = 2;
  mvNum = cu.affineType ? 3 : 2;

  // Mvp
  Mv        cMvPred[2][MAX_REF_PICS][3];
  Mv        cMvPredBi[2][MAX_REF_PICS][3];
  int       aaiMvpIdxBi[2][MAX_REF_PICS];
  int       aaiMvpIdx[2][MAX_REF_PICS];
  int       aaiMvpNum[2][MAX_REF_PICS];

  AffineAMVPInfo aacAffineAMVPInfo[2][MAX_REF_PICS];
  AffineAMVPInfo affiAMVPInfoTemp[2];

  uint32_t      uiMbBits[3] = { 1, 1, 0 };
  int           iRefIdx[2] = { 0,0 }; // If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  int           iRefIdxBi[2];
  int           iRefStart, iRefEnd;
  int           bestBiPRefIdxL1 = 0;
  int           bestBiPMvpL1 = 0;
  Distortion    biPDistTemp = MAX_DISTORTION;

  Distortion    uiCost[2] = { MAX_DISTORTION, MAX_DISTORTION };
  Distortion    uiCostBi = MAX_DISTORTION;
  Distortion    uiCostTemp;

  uint32_t      uiBits[3] = { 0 };
  uint32_t      uiBitsTemp;
  Distortion    bestBiPDist = MAX_DISTORTION;

  Distortion    uiCostTempL0[MAX_NUM_REF];
  for (int iNumRef = 0; iNumRef < MAX_NUM_REF; iNumRef++)
  {
    uiCostTempL0[iNumRef] = MAX_DISTORTION;
  }
  uint32_t      uiBitsTempL0[MAX_NUM_REF];

  Mv            mvValidList1[4];
  int           refIdxValidList1 = 0;
  uint32_t      bitsValidList1 = MAX_UINT;
  Distortion    costValidList1 = MAX_DISTORTION;
  Mv            mvHevc[3];
  const bool    affineAmvrEnabled = false;

  xGetBlkBits(slice.isInterP(), puIdx, lastMode, uiMbBits);

  cu.affine = true;
  cu.mergeFlag = false;
  cu.regularMergeFlag = false;
  if (BcwIdx != BCW_DEFAULT)
  {
    cu.BcwIdx = BcwIdx;
  }

  // Uni-directional prediction
  for (int iRefList = 0; iRefList < iNumPredDir; iRefList++)
  {
    RefPicList  refPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    cu.interDir = (iRefList ? 2 : 1);
    for (int iRefIdxTemp = 0; iRefIdxTemp < slice.numRefIdx[refPicList]; iRefIdxTemp++)
    {
      // Get RefIdx bits
      uiBitsTemp = uiMbBits[iRefList];
      if (slice.numRefIdx[refPicList] > 1)
      {
        uiBitsTemp += iRefIdxTemp + 1;
        if (iRefIdxTemp == slice.numRefIdx[refPicList] - 1)
        {
          uiBitsTemp--;
        }
      }

      // Do Affine AMVP
      xEstimateAffineAMVP(cu, affiAMVPInfoTemp[refPicList], origBuf, refPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], biPDistTemp);
      if (affineAmvrEnabled)
      {
        biPDistTemp += m_pcRdCost->getCost(xCalcAffineMVBits(cu, cMvPred[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp]));
      }
      aaiMvpIdx[iRefList][iRefIdxTemp] = cu.mvpIdx[refPicList];
      aaiMvpNum[iRefList][iRefIdxTemp] = cu.mvpNum[refPicList];;
      if (cu.affineType == AFFINEMODEL_6PARAM && refIdx4Para[iRefList] != iRefIdxTemp)
      {
        xCopyAffineAMVPInfo(affiAMVPInfoTemp[refPicList], aacAffineAMVPInfo[iRefList][iRefIdxTemp]);
        continue;
      }

      // set hevc ME result as start search position when it is best than mvp
      for (int i = 0; i<3; i++)
      {
        mvHevc[i] = hevcMv[iRefList][iRefIdxTemp];
        mvHevc[i].roundAffinePrecInternal2Amvr(cu.imv);
      }
      PelUnitBuf predBuf = m_tmpStorageLCU.getCompactBuf(cu);

      Distortion uiCandCost = xGetAffineTemplateCost(cu, origBuf, predBuf, mvHevc, aaiMvpIdx[iRefList][iRefIdxTemp],
        AMVP_MAX_NUM_CANDS, refPicList, iRefIdxTemp);

      if (affineAmvrEnabled)
      {
        uiCandCost += m_pcRdCost->getCost(xCalcAffineMVBits(cu, mvHevc, cMvPred[iRefList][iRefIdxTemp]));
      }

      //check stored affine motion
      bool affine4Para = cu.affineType == AFFINEMODEL_4PARAM;
      bool savedParaAvail = cu.imv && ((m_affineMotion.affine4ParaRefIdx[iRefList] == iRefIdxTemp && affine4Para && m_affineMotion.affine4ParaAvail) ||
        (m_affineMotion.affine6ParaRefIdx[iRefList] == iRefIdxTemp && !affine4Para && m_affineMotion.affine6ParaAvail));

      if (savedParaAvail)
      {
        Mv mvFour[3];
        for (int i = 0; i < mvNum; i++)
        {
          mvFour[i] = affine4Para ? m_affineMotion.acMvAffine4Para[iRefList][i] : m_affineMotion.acMvAffine6Para[iRefList][i];
          mvFour[i].roundAffinePrecInternal2Amvr(cu.imv);
        }

        Distortion candCostInherit = xGetAffineTemplateCost(cu, origBuf, predBuf, mvFour, aaiMvpIdx[iRefList][iRefIdxTemp], AMVP_MAX_NUM_CANDS, refPicList, iRefIdxTemp);
        candCostInherit += m_pcRdCost->getCost(xCalcAffineMVBits(cu, mvFour, cMvPred[iRefList][iRefIdxTemp]));

        if (candCostInherit < uiCandCost)
        {
          uiCandCost = candCostInherit;
          memcpy(mvHevc, mvFour, 3 * sizeof(Mv));
        }
      }

      if (cu.affineType == AFFINEMODEL_4PARAM && m_AffineProfList->m_affMVListSize)
      {
        int shift = MAX_CU_DEPTH;
        for (int i = 0; i < m_AffineProfList->m_affMVListSize; i++)
        {
          AffineMVInfo *mvInfo = m_AffineProfList->m_affMVList + ((m_AffineProfList->m_affMVListIdx - i - 1 + m_AffineProfList->m_affMVListMaxSize) % (m_AffineProfList->m_affMVListMaxSize));
          //check;
          int j = 0;
          for (; j < i; j++)
          {
            AffineMVInfo *prevMvInfo = m_AffineProfList->m_affMVList + ((m_AffineProfList->m_affMVListIdx - j - 1 + m_AffineProfList->m_affMVListMaxSize) % (m_AffineProfList->m_affMVListMaxSize));
            if ((mvInfo->affMVs[iRefList][iRefIdxTemp][0] == prevMvInfo->affMVs[iRefList][iRefIdxTemp][0]) &&
              (mvInfo->affMVs[iRefList][iRefIdxTemp][1] == prevMvInfo->affMVs[iRefList][iRefIdxTemp][1])
              && (mvInfo->x == prevMvInfo->x) && (mvInfo->y == prevMvInfo->y)
              && (mvInfo->w == prevMvInfo->w)
              )
            {
              break;
            }
          }
          if (j < i)
            continue;

          Mv mvTmp[3], *nbMv = mvInfo->affMVs[iRefList][iRefIdxTemp];
          int vx, vy;
          int dMvHorX, dMvHorY, dMvVerX, dMvVerY;
          int mvScaleHor = nbMv[0].hor << shift;
          int mvScaleVer = nbMv[0].ver << shift;
          Mv dMv = nbMv[1] - nbMv[0];
          dMvHorX = dMv.hor << (shift - Log2(mvInfo->w));
          dMvHorY = dMv.ver << (shift - Log2(mvInfo->w));
          dMvVerX = -dMvHorY;
          dMvVerY = dMvHorX;
          vx = mvScaleHor + dMvHorX * (cu.Y().x - mvInfo->x) + dMvVerX * (cu.Y().y - mvInfo->y);
          vy = mvScaleVer + dMvHorY * (cu.Y().x - mvInfo->x) + dMvVerY * (cu.Y().y - mvInfo->y);
          roundAffineMv(vx, vy, shift);
          mvTmp[0] = Mv(vx, vy);
          mvTmp[0].clipToStorageBitDepth();
          clipMv(mvTmp[0], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
          mvTmp[0].roundAffinePrecInternal2Amvr(cu.imv);
          vx = mvScaleHor + dMvHorX * (cu.Y().x + cu.Y().width - mvInfo->x) + dMvVerX * (cu.Y().y - mvInfo->y);
          vy = mvScaleVer + dMvHorY * (cu.Y().x + cu.Y().width - mvInfo->x) + dMvVerY * (cu.Y().y - mvInfo->y);
          roundAffineMv(vx, vy, shift);
          mvTmp[1] = Mv(vx, vy);
          mvTmp[1].clipToStorageBitDepth();
          clipMv(mvTmp[1], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
          mvTmp[0].roundAffinePrecInternal2Amvr(cu.imv);
          mvTmp[1].roundAffinePrecInternal2Amvr(cu.imv);
          Distortion tmpCost = xGetAffineTemplateCost(cu, origBuf, predBuf, mvTmp, aaiMvpIdx[iRefList][iRefIdxTemp], AMVP_MAX_NUM_CANDS, refPicList, iRefIdxTemp);
          if (affineAmvrEnabled)
          {
            tmpCost += m_pcRdCost->getCost(xCalcAffineMVBits(cu, mvTmp, cMvPred[iRefList][iRefIdxTemp]));
          }
          if (tmpCost < uiCandCost)
          {
            uiCandCost = tmpCost;
            std::memcpy(mvHevc, mvTmp, 3 * sizeof(Mv));
          }
        }
      }
      if (cu.affineType == AFFINEMODEL_6PARAM)
      {
        Mv mvFour[3];
        mvFour[0] = mvAffine4Para[iRefList][iRefIdxTemp][0];
        mvFour[1] = mvAffine4Para[iRefList][iRefIdxTemp][1];
        mvAffine4Para[iRefList][iRefIdxTemp][0].roundAffinePrecInternal2Amvr(cu.imv);
        mvAffine4Para[iRefList][iRefIdxTemp][1].roundAffinePrecInternal2Amvr(cu.imv);

        int shift = MAX_CU_DEPTH;
        int vx2 = (mvFour[0].hor << shift) - ((mvFour[1].ver - mvFour[0].ver) << (shift + Log2(cu.lheight()) - Log2(cu.lwidth())));
        int vy2 = (mvFour[0].ver << shift) + ((mvFour[1].hor - mvFour[0].hor) << (shift + Log2(cu.lheight()) - Log2(cu.lwidth())));
        int offset = (1 << (shift - 1));
        vx2 = (vx2 + offset - (vx2 >= 0)) >> shift;
        vy2 = (vy2 + offset - (vy2 >= 0)) >> shift;
        mvFour[2].hor = vx2;
        mvFour[2].ver = vy2;
        mvFour[2].clipToStorageBitDepth();
        mvFour[0].roundAffinePrecInternal2Amvr(cu.imv);
        mvFour[1].roundAffinePrecInternal2Amvr(cu.imv);
        mvFour[2].roundAffinePrecInternal2Amvr(cu.imv);
        Distortion uiCandCostInherit = xGetAffineTemplateCost(cu, origBuf, predBuf, mvFour, aaiMvpIdx[iRefList][iRefIdxTemp], AMVP_MAX_NUM_CANDS, refPicList, iRefIdxTemp);
        if (affineAmvrEnabled)
        {
          uiCandCostInherit += m_pcRdCost->getCost(xCalcAffineMVBits(cu, mvFour, cMvPred[iRefList][iRefIdxTemp]));
        }
        if (uiCandCostInherit < uiCandCost)
        {
          uiCandCost = uiCandCostInherit;
          for (int i = 0; i < 3; i++)
          {
            mvHevc[i] = mvFour[i];
          }
        }
      }

      if (uiCandCost < biPDistTemp)
      {
        ::memcpy(tmp.affMVs[iRefList][iRefIdxTemp], mvHevc, sizeof(Mv) * 3);
      }
      else
      {
        ::memcpy(tmp.affMVs[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], sizeof(Mv) * 3);
      }

      // GPB list 1, save the best MvpIdx, RefIdx and Cost
      if (slice.picHeader->mvdL1Zero && iRefList == 1 && biPDistTemp < bestBiPDist)
      {
        bestBiPDist = biPDistTemp;
        bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
        bestBiPRefIdxL1 = iRefIdxTemp;
      }

      // Update bits
      uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

      if (m_pcEncCfg->m_bFastMEForGenBLowDelayEnabled && iRefList == 1)   // list 1
      {
        if (slice.list1IdxToList0Idx[iRefIdxTemp] >= 0 && (cu.affineType != AFFINEMODEL_6PARAM || slice.list1IdxToList0Idx[iRefIdxTemp] == refIdx4Para[0]))
        {
          int iList1ToList0Idx = slice.list1IdxToList0Idx[iRefIdxTemp];
          ::memcpy(tmp.affMVs[1][iRefIdxTemp], tmp.affMVs[0][iList1ToList0Idx], sizeof(Mv) * 3);
          uiCostTemp = uiCostTempL0[iList1ToList0Idx];

          uiCostTemp -= m_pcRdCost->getCost(uiBitsTempL0[iList1ToList0Idx]);
          uiBitsTemp += xCalcAffineMVBits(cu, tmp.affMVs[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp]);
          /*calculate the correct cost*/
          uiCostTemp += m_pcRdCost->getCost(uiBitsTemp);
          DTRACE(g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiCostTemp);
        }
        else
        {
          xAffineMotionEstimation(cu, origBuf, refPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, tmp.affMVs[iRefList][iRefIdxTemp], 
                                  uiBitsTemp, uiCostTemp, aaiMvpIdx[iRefList][iRefIdxTemp], affiAMVPInfoTemp[refPicList]);
        }
      }
      else
      {
        xAffineMotionEstimation(cu, origBuf, refPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, tmp.affMVs[iRefList][iRefIdxTemp], 
                                uiBitsTemp, uiCostTemp, aaiMvpIdx[iRefList][iRefIdxTemp], affiAMVPInfoTemp[refPicList]);
      }
      // Set best AMVP Index
      xCopyAffineAMVPInfo(affiAMVPInfoTemp[refPicList], aacAffineAMVPInfo[iRefList][iRefIdxTemp]);
      if (cu.imv != 2)//|| !m_pcEncCfg->getUseAffineAmvrEncOpt())
        xCheckBestAffineMVP(cu, affiAMVPInfoTemp[refPicList], refPicList, tmp.affMVs[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);

      if (iRefList == 0)
      {
        uiCostTempL0[iRefIdxTemp] = uiCostTemp;
        uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
      }
      DTRACE(g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d, uiCost[iRefList]=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiCostTemp, uiCost[iRefList]);
      if (uiCostTemp < uiCost[iRefList])
      {
        uiCost[iRefList] = uiCostTemp;
        uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

                                       // set best motion
        ::memcpy(aacMv[iRefList], tmp.affMVs[iRefList][iRefIdxTemp], sizeof(Mv) * 3);
        iRefIdx[iRefList] = iRefIdxTemp;
      }

      if (iRefList == 1 && uiCostTemp < costValidList1 && slice.list1IdxToList0Idx[iRefIdxTemp] < 0)
      {
        costValidList1 = uiCostTemp;
        bitsValidList1 = uiBitsTemp;

        // set motion
        memcpy(mvValidList1, tmp.affMVs[iRefList][iRefIdxTemp], sizeof(Mv) * 3);
        refIdxValidList1 = iRefIdxTemp;
      }
    } // End refIdx loop
  } // end Uni-prediction

  if (cu.affineType == AFFINEMODEL_4PARAM)
  {
    ::memcpy(mvAffine4Para, tmp.affMVs, sizeof(tmp.affMVs));
    if (cu.imv == 0)
    {
      m_AffineProfList->insert( tmp, cu.Y());
    }
  }

  // Bi-directional prediction
  if (slice.isInterB() && !CU::isBipredRestriction(cu))
  {
    cu.interDir = 3;
    m_isBi = true;

    // Set as best list0 and list1
    iRefIdxBi[0] = iRefIdx[0];
    iRefIdxBi[1] = iRefIdx[1];

    ::memcpy(cMvBi, aacMv, sizeof(aacMv));
    ::memcpy(cMvPredBi, cMvPred, sizeof(cMvPred));
    ::memcpy(aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx));

    uint32_t uiMotBits[2];
    bool doBiPred = true;

    if (slice.picHeader->mvdL1Zero) // GPB, list 1 only use Mvp
    {
      xCopyAffineAMVPInfo(aacAffineAMVPInfo[1][bestBiPRefIdxL1], affiAMVPInfoTemp[REF_PIC_LIST_1]);
      cu.mvpIdx[REF_PIC_LIST_1] = bestBiPMvpL1;
      aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;

      // Set Mv for list1
      Mv pcMvTemp[3] = { affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandLT[bestBiPMvpL1],
                         affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandRT[bestBiPMvpL1],
                         affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandLB[bestBiPMvpL1] };
      ::memcpy(cMvPredBi[1][bestBiPRefIdxL1], pcMvTemp, sizeof(Mv) * 3);
      ::memcpy(cMvBi[1], pcMvTemp, sizeof(Mv) * 3);
      ::memcpy(tmp.affMVs[1][bestBiPRefIdxL1], pcMvTemp, sizeof(Mv) * 3);
      iRefIdxBi[1] = bestBiPRefIdxL1;

      // Get list1 prediction block
      CU::setAllAffineMv(cu, cMvBi[1][0], cMvBi[1][1], cMvBi[1][2], REF_PIC_LIST_1);
      cu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];

      PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_1].getCompactBuf( cu );
      motionCompensation(cu, predBufTmp, REF_PIC_LIST_1);

      // Update bits
      uiMotBits[0] = uiBits[0] - uiMbBits[0];
      uiMotBits[1] = uiMbBits[1];

      if (slice.numRefIdx[REF_PIC_LIST_1] > 1)
      {
        uiMotBits[1] += bestBiPRefIdxL1 + 1;
        if (bestBiPRefIdxL1 == slice.numRefIdx[REF_PIC_LIST_1] - 1)
        {
          uiMotBits[1]--;
        }
      }
      uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];
      uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
    }
    else
    {
      uiMotBits[0] = uiBits[0] - uiMbBits[0];
      uiMotBits[1] = uiBits[1] - uiMbBits[1];
      uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
    }

    if (doBiPred)
    {
      // 4-times iteration (default)
      int iNumIter = 4;
      // fast encoder setting or GPB: only one iteration
      if (m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE1 || m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE2 || slice.picHeader->mvdL1Zero)
      {
        iNumIter = 1;
      }

      for (int iIter = 0; iIter < iNumIter; iIter++)
      {
        // Set RefList
        int iRefList = iIter % 2;
        if (m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE1 || m_pcEncCfg->m_fastInterSearchMode == VVENC_FASTINTERSEARCH_MODE2)
        {
          if (uiCost[0] <= uiCost[1])
          {
            iRefList = 1;
          }
          else
          {
            iRefList = 0;
          }
        }
        else if (iIter == 0)
        {
          iRefList = 0;
        }

        // First iterate, get prediction block of opposite direction
        if (iIter == 0 && !slice.picHeader->mvdL1Zero)
        {
          CU::setAllAffineMv(cu, aacMv[1 - iRefList][0], aacMv[1 - iRefList][1], aacMv[1 - iRefList][2], RefPicList(1 - iRefList));
          cu.refIdx[1 - iRefList] = iRefIdx[1 - iRefList];

          PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getCompactBuf( cu );
          motionCompensation(cu, predBufTmp, RefPicList(1 - iRefList));
        }

        RefPicList refPicList = (iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

        if (slice.picHeader->mvdL1Zero) // GPB, fix List 1, search List 0
        {
          iRefList = 0;
          refPicList = REF_PIC_LIST_0;
        }

        bool bChanged = false;

        iRefStart = 0;
        iRefEnd = slice.numRefIdx[refPicList] - 1;
        for (int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++)
        {
          if (cu.affineType == AFFINEMODEL_6PARAM && refIdx4Para[iRefList] != iRefIdxTemp)
          {
            continue;
          }
          // update bits
          uiBitsTemp = uiMbBits[2] + uiMotBits[1 - iRefList];
          //  uiBitsTemp += ((cu.slice->getSPS()->BCW == true) ? BcwIdxBits : 0);
          if (slice.numRefIdx[refPicList] > 1)
          {
            uiBitsTemp += iRefIdxTemp + 1;
            if (iRefIdxTemp == slice.numRefIdx[refPicList] - 1)
            {
              uiBitsTemp--;
            }
          }
          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

          // call Affine ME
          xAffineMotionEstimation(cu, origBuf, refPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, tmp.affMVs[iRefList][iRefIdxTemp], 
                                  uiBitsTemp, uiCostTemp, aaiMvpIdxBi[iRefList][iRefIdxTemp], aacAffineAMVPInfo[iRefList][iRefIdxTemp], true);
          xCopyAffineAMVPInfo(aacAffineAMVPInfo[iRefList][iRefIdxTemp], affiAMVPInfoTemp[refPicList]);
          if (cu.imv != 2)
          {
            xCheckBestAffineMVP(cu, affiAMVPInfoTemp[refPicList], refPicList, tmp.affMVs[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp);
          }

          if (uiCostTemp < uiCostBi)
          {
            bChanged = true;
            ::memcpy(cMvBi[iRefList], tmp.affMVs[iRefList][iRefIdxTemp], sizeof(Mv) * 3);
            iRefIdxBi[iRefList] = iRefIdxTemp;

            uiCostBi = uiCostTemp;
            uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1 - iRefList];
            uiBits[2] = uiBitsTemp;

            if (iNumIter != 1) // MC for next iter
            {
              //  Set motion
              CU::setAllAffineMv(cu, cMvBi[iRefList][0], cMvBi[iRefList][1], cMvBi[iRefList][2], refPicList);
              cu.refIdx[refPicList] = iRefIdxBi[refPicList];
              PelUnitBuf predBufTmp = m_tmpPredStorage[iRefList].getCompactBuf( cu );
              motionCompensation(cu, predBufTmp, refPicList);
            }
          }
        } // for loop-iRefIdxTemp

        if (!bChanged)
        {
          if ((uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1]) || enforceBcwPred)
          {
            xCopyAffineAMVPInfo(aacAffineAMVPInfo[0][iRefIdxBi[0]], affiAMVPInfoTemp[REF_PIC_LIST_0]);
            xCheckBestAffineMVP(cu, affiAMVPInfoTemp[REF_PIC_LIST_0], REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], uiBits[2], uiCostBi);

            if (!slice.picHeader->mvdL1Zero)
            {
              xCopyAffineAMVPInfo(aacAffineAMVPInfo[1][iRefIdxBi[1]], affiAMVPInfoTemp[REF_PIC_LIST_1]);
              xCheckBestAffineMVP(cu, affiAMVPInfoTemp[REF_PIC_LIST_1], REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], uiBits[2], uiCostBi);
            }
          }
          break;
        }
      } // for loop-iter
    }
    m_isBi = false;
  } // if (B_SLICE)

  cu.mv [REF_PIC_LIST_0][0] = Mv();
  cu.mv [REF_PIC_LIST_1][0] = Mv();
  cu.mvd[REF_PIC_LIST_0][0] = cMvZero;
  cu.mvd[REF_PIC_LIST_1][0] = cMvZero;
  cu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
  cu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
  cu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  cu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  cu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  cu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;

  for (int verIdx = 0; verIdx < 3; verIdx++)
  {
    cu.mvd[REF_PIC_LIST_0][verIdx] = cMvZero;
    cu.mvd[REF_PIC_LIST_1][verIdx] = cMvZero;
  }

  // Set Motion Field
  memcpy(aacMv[1], mvValidList1, sizeof(Mv) * 3);
  iRefIdx[1] = refIdxValidList1;
  uiBits[1] = bitsValidList1;
  uiCost[1] = costValidList1;

  if (enforceBcwPred)
  {
    uiCost[0] = uiCost[1] = MAX_UINT;
  }

  // Affine ME result set
  if (uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1]) // Bi
  {
    lastMode = 2;
    affineCost = uiCostBi;
    cu.interDir = 3;
    CU::setAllAffineMv(cu, cMvBi[0][0], cMvBi[0][1], cMvBi[0][2], REF_PIC_LIST_0);
    CU::setAllAffineMv(cu, cMvBi[1][0], cMvBi[1][1], cMvBi[1][2], REF_PIC_LIST_1);
    cu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
    cu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];

    for (int verIdx = 0; verIdx < mvNum; verIdx++)
    {
      cu.mvd[REF_PIC_LIST_0][verIdx] = cMvBi[0][verIdx] - cMvPredBi[0][iRefIdxBi[0]][verIdx];
      cu.mvd[REF_PIC_LIST_1][verIdx] = cMvBi[1][verIdx] - cMvPredBi[1][iRefIdxBi[1]][verIdx];
      if (verIdx != 0)
      {
        cu.mvd[0][verIdx] = cu.mvd[0][verIdx] - cu.mvd[0][0];
        cu.mvd[1][verIdx] = cu.mvd[1][verIdx] - cu.mvd[1][0];
      }
    }


    cu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
    cu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
    cu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
    cu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
  }
  else if (uiCost[0] <= uiCost[1]) // List 0
  {
    lastMode = 0;
    affineCost = uiCost[0];
    cu.interDir = 1;
    CU::setAllAffineMv(cu, aacMv[0][0], aacMv[0][1], aacMv[0][2], REF_PIC_LIST_0);
    cu.refIdx[REF_PIC_LIST_0] = iRefIdx[0];

    for (int verIdx = 0; verIdx < mvNum; verIdx++)
    {
      cu.mvd[REF_PIC_LIST_0][verIdx] = aacMv[0][verIdx] - cMvPred[0][iRefIdx[0]][verIdx];
      if (verIdx != 0)
      {
        cu.mvd[0][verIdx] = cu.mvd[0][verIdx] - cu.mvd[0][0];
      }
    }

    cu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][iRefIdx[0]];
    cu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdx[0]];
  }
  else
  {
    lastMode = 1;
    affineCost = uiCost[1];
    cu.interDir = 2;
    CU::setAllAffineMv(cu, aacMv[1][0], aacMv[1][1], aacMv[1][2], REF_PIC_LIST_1);
    cu.refIdx[REF_PIC_LIST_1] = iRefIdx[1];

    for (int verIdx = 0; verIdx < mvNum; verIdx++)
    {
      cu.mvd[REF_PIC_LIST_1][verIdx] = aacMv[1][verIdx] - cMvPred[1][iRefIdx[1]][verIdx];
      if (verIdx != 0)
      {
        cu.mvd[1][verIdx] = cu.mvd[1][verIdx] - cu.mvd[1][0];
      }
    }

    cu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][iRefIdx[1]];
    cu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdx[1]];
  }
  if (BcwIdx != BCW_DEFAULT)
  {
    cu.BcwIdx = BCW_DEFAULT;
  }
}

Distortion InterSearch::xGetAffineTemplateCost(CodingUnit& cu, CPelUnitBuf& origBuf, PelUnitBuf& predBuf, Mv acMvCand[3], int iMVPIdx, int iMVPNum, RefPicList refPicList, int iRefIdx)
{
  Distortion uiCost = MAX_DISTORTION;

  const Picture* picRef = cu.slice->getRefPic(refPicList, iRefIdx);

  // prediction pattern
  Mv mv[3];
  memcpy(mv, acMvCand, sizeof(mv));
  xPredAffineBlk(COMP_Y, cu, picRef, mv, predBuf, false, cu.slice->clpRngs[COMP_Y], refPicList);

  // calc distortion
  enum DFunc distFunc = (cu.cs->slice->disableSATDForRd) ? DF_SAD : DF_HAD;
  uiCost = m_pcRdCost->getDistPart(origBuf.Y(), predBuf.Y(), cu.cs->sps->bitDepths[CH_L], COMP_Y
    , distFunc
  );
  uiCost += m_pcRdCost->getCost(m_auiMVPIdxCost[iMVPIdx][iMVPNum]);

  DTRACE(g_trace_ctx, D_COMMON, " (%d) affineTemplateCost=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiCost);
  return uiCost;
}

void solveEqual(double** dEqualCoeff, int iOrder, double* dAffinePara)
{
  for (int k = 0; k < iOrder; k++)
  {
    dAffinePara[k] = 0.;
  }

  // row echelon
  for (int i = 1; i < iOrder; i++)
  {
    // find column max
    double temp = fabs(dEqualCoeff[i][i - 1]);
    int tempIdx = i;
    for (int j = i + 1; j < iOrder + 1; j++)
    {
      if (fabs(dEqualCoeff[j][i - 1]) > temp)
      {
        temp = fabs(dEqualCoeff[j][i - 1]);
        tempIdx = j;
      }
    }

    // swap line
    if (tempIdx != i)
    {
      for (int j = 0; j < iOrder + 1; j++)
      {
        dEqualCoeff[0][j] = dEqualCoeff[i][j];
        dEqualCoeff[i][j] = dEqualCoeff[tempIdx][j];
        dEqualCoeff[tempIdx][j] = dEqualCoeff[0][j];
      }
    }

    // elimination first column
    if (dEqualCoeff[i][i - 1] == 0.)
    {
      return;
    }
    for (int j = i + 1; j < iOrder + 1; j++)
    {
      for (int k = i; k < iOrder + 1; k++)
      {
        dEqualCoeff[j][k] = dEqualCoeff[j][k] - dEqualCoeff[i][k] * dEqualCoeff[j][i - 1] / dEqualCoeff[i][i - 1];
      }
    }
  }

  if (dEqualCoeff[iOrder][iOrder - 1] == 0.)
  {
    return;
  }
  dAffinePara[iOrder - 1] = dEqualCoeff[iOrder][iOrder] / dEqualCoeff[iOrder][iOrder - 1];
  for (int i = iOrder - 2; i >= 0; i--)
  {
    if (dEqualCoeff[i + 1][i] == 0.)
    {
      for (int k = 0; k < iOrder; k++)
      {
        dAffinePara[k] = 0.;
      }
      return;
    }
    double temp = 0;
    for (int j = i + 1; j < iOrder; j++)
    {
      temp += dEqualCoeff[i + 1][j] * dAffinePara[j];
    }
    dAffinePara[i] = (dEqualCoeff[i + 1][iOrder] - temp) / dEqualCoeff[i + 1][i];
  }
}

void InterSearch::xCheckBestAffineMVP(CodingUnit& cu, AffineAMVPInfo &affineAMVPInfo, RefPicList refPicList, Mv acMv[3], Mv acMvPred[3], int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost)
{
  if (affineAMVPInfo.numCand < 2)
  {
    return;
  }

  int mvNum = cu.affineType ? 3 : 2;

  m_pcRdCost->selectMotionLambda();
  m_pcRdCost->setCostScale(0);

  int iBestMVPIdx = riMVPIdx;

  // Get origin MV bits
  Mv tmpPredMv[3];
  int iOrgMvBits = xCalcAffineMVBits(cu, acMv, acMvPred);
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];

  int iBestMvBits = iOrgMvBits;
  for (int iMVPIdx = 0; iMVPIdx < affineAMVPInfo.numCand; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }
    tmpPredMv[0] = affineAMVPInfo.mvCandLT[iMVPIdx];
    tmpPredMv[1] = affineAMVPInfo.mvCandRT[iMVPIdx];
    if (mvNum == 3)
    {
      tmpPredMv[2] = affineAMVPInfo.mvCandLB[iMVPIdx];
    }
    int iMvBits = xCalcAffineMVBits(cu, acMv, tmpPredMv);
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  // if changed
  {
    acMvPred[0] = affineAMVPInfo.mvCandLT[iBestMVPIdx];
    acMvPred[1] = affineAMVPInfo.mvCandRT[iBestMVPIdx];
    acMvPred[2] = affineAMVPInfo.mvCandLB[iBestMVPIdx];
    riMVPIdx = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost(uiOrgBits)) + m_pcRdCost->getCost(ruiBits);
  }
}

void InterSearch::xAffineMotionEstimation(CodingUnit& cu,
  CPelUnitBuf&    origBuf,
  RefPicList      refPicList,
  Mv              acMvPred[3],
  int             iRefIdxPred,
  Mv              acMv[3],
  uint32_t&       ruiBits,
  Distortion&     ruiCost,
  int&            mvpIdx,
  const AffineAMVPInfo& aamvpi,
  bool            bBi)
{
  int bestMvpIdx = mvpIdx;
  const int width = cu.Y().width;
  const int height = cu.Y().height;

  const Picture* refPic = cu.slice->getRefPic(refPicList, iRefIdxPred);

  // Set Origin YUV: pcYuv
  CPelUnitBuf*   pBuf = &origBuf;
  double        fWeight = 1.0;

  CPelUnitBuf  origBufTmpCnst;
  enum DFunc distFunc = (cu.cs->slice->disableSATDForRd) ? DF_SAD : DF_HAD;

  // if Bi, set to ( 2 * Org - ListX )
  if (bBi)
  {
    PelUnitBuf  origBufTmp = m_tmpStorageLCU.getCompactBuf(cu);
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)refPicList].getCompactBuf( cu );
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq(otherBuf, m_pcEncCfg->m_bClipForBiPredMeEnabled, cu.slice->clpRngs);

    origBufTmpCnst = m_tmpStorageLCU.getCompactBuf(cu);
    pBuf           = &origBufTmpCnst;
    fWeight        = xGetMEDistortionWeight(cu.BcwIdx, refPicList);
  }

  // pred YUV
  PelUnitBuf  predBuf = m_tmpAffiStorage.getCompactBuf(cu);

  // Set start Mv position, use input mv as started search mv
  Mv acMvTemp[3];
  ::memcpy(acMvTemp, acMv, sizeof(Mv) * 3);
  // Set delta mv
  // malloc buffer
  int iParaNum = cu.affineType ? 7 : 5;
  int affineParaNum = iParaNum - 1;
  int mvNum = cu.affineType ? 3 : 2;
  double **pdEqualCoeff;
  pdEqualCoeff = new double *[iParaNum];
  for (int i = 0; i < iParaNum; i++)
  {
    pdEqualCoeff[i] = new double[iParaNum];
  }

  int64_t  i64EqualCoeff[7][7];
  Pel    *piError = m_tmpAffiError;
  int    *pdDerivate[2];
  pdDerivate[0] = m_tmpAffiDeri[0];
  pdDerivate[1] = m_tmpAffiDeri[1];

  Distortion uiCostBest = MAX_DISTORTION;
  uint32_t uiBitsBest = 0;

  // do motion compensation with origin mv

  clipMv(acMvTemp[0], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
  clipMv(acMvTemp[1], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
  if (cu.affineType == AFFINEMODEL_6PARAM)
  {
    clipMv(acMvTemp[2], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
  }

  acMvTemp[0].roundAffinePrecInternal2Amvr(cu.imv);
  acMvTemp[1].roundAffinePrecInternal2Amvr(cu.imv);
  if (cu.affineType == AFFINEMODEL_6PARAM)
  {
    acMvTemp[2].roundAffinePrecInternal2Amvr(cu.imv);
  }
  xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.cs->slice->clpRngs[COMP_Y], refPicList);

  // get error
  uiCostBest = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->bitDepths[CH_L], COMP_Y, distFunc);

  // get cost with mv
  m_pcRdCost->setCostScale(0);
  uiBitsBest = ruiBits;
  DTRACE(g_trace_ctx, D_COMMON, " (%d) xx uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest);
  uiBitsBest += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
  DTRACE(g_trace_ctx, D_COMMON, " (%d) yy uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest);
  uiCostBest = (Distortion)(floor(fWeight * (double)uiCostBest) + (double)m_pcRdCost->getCost(uiBitsBest));

  DTRACE(g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest, uiCostBest);

  ::memcpy(acMv, acMvTemp, sizeof(Mv) * 3);

  const int bufStride = pBuf->Y().stride;
  const int predBufStride = predBuf.Y().stride;
  Mv prevIterMv[7][3];
  int iIterTime;
  if (cu.affineType == AFFINEMODEL_6PARAM)
  {
    iIterTime = bBi ? 3 : 4;
  }
  else
  {
    iIterTime = bBi ? 3 : 5;
  }

  if (!cu.cs->sps->AffineType)// getUseAffineType())
  {
    iIterTime = bBi ? 5 : 7;
  }

  for (int iter = 0; iter<iIterTime; iter++)    // iterate loop
  {
    memcpy(prevIterMv[iter], acMvTemp, sizeof(Mv) * 3);
    /*********************************************************************************
    *                         use gradient to update mv
    *********************************************************************************/
    // get Error Matrix
    const Pel* pOrg = pBuf->Y().buf;
    Pel* pPred = predBuf.Y().buf;
    for (int j = 0; j< height; j++)
    {
      for (int i = 0; i< width; i++)
      {
        piError[i + j * width] = pOrg[i] - pPred[i];
      }
      pOrg += bufStride;
      pPred += predBufStride;
    }

    // sobel x direction
    // -1 0 1
    // -2 0 2
    // -1 0 1
    pPred = predBuf.Y().buf;
    m_HorizontalSobelFilter(pPred, predBufStride, pdDerivate[0], width, width, height);

    // sobel y direction
    // -1 -2 -1
    //  0  0  0
    //  1  2  1
    m_VerticalSobelFilter(pPred, predBufStride, pdDerivate[1], width, width, height);

    // solve delta x and y
    for (int row = 0; row < iParaNum; row++)
    {
      memset(&i64EqualCoeff[row][0], 0, iParaNum * sizeof(int64_t));
    }

    m_EqualCoeffComputer(piError, width, pdDerivate, width, i64EqualCoeff, width, height
      , (cu.affineType == AFFINEMODEL_6PARAM)
    );

    for (int row = 0; row < iParaNum; row++)
    {
      for (int i = 0; i < iParaNum; i++)
      {
        pdEqualCoeff[row][i] = (double)i64EqualCoeff[row][i];
      }
    }

    double dAffinePara[6];
    double dDeltaMv[6];
    Mv acDeltaMv[3];

    solveEqual(pdEqualCoeff, affineParaNum, dAffinePara);

    // convert to delta mv
    dDeltaMv[0] = dAffinePara[0];
    dDeltaMv[2] = dAffinePara[2];
    const bool extParams = cu.affineType == AFFINEMODEL_6PARAM;
    if (extParams)
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = dAffinePara[3] * width + dAffinePara[2];
      dDeltaMv[4] = dAffinePara[4] * height + dAffinePara[0];
      dDeltaMv[5] = dAffinePara[5] * height + dAffinePara[2];
    }
    else
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = -dAffinePara[3] * width + dAffinePara[2];
    }

    const int normShiftTab[3] = { MV_PRECISION_QUARTER - MV_PRECISION_INT, MV_PRECISION_SIXTEENTH - MV_PRECISION_INT, MV_PRECISION_QUARTER - MV_PRECISION_INT };
    const int stepShiftTab[3] = { MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL - MV_PRECISION_SIXTEENTH, MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER };
    const int multiShift = 1 << normShiftTab[cu.imv];
    const int mvShift = stepShiftTab[cu.imv];

    acDeltaMv[0] = Mv((int)(dDeltaMv[0] * multiShift + SIGN(dDeltaMv[0]) * 0.5) << mvShift, (int)(dDeltaMv[2] * multiShift + SIGN(dDeltaMv[2]) * 0.5) << mvShift);
    acDeltaMv[1] = Mv((int)(dDeltaMv[1] * multiShift + SIGN(dDeltaMv[1]) * 0.5) << mvShift, (int)(dDeltaMv[3] * multiShift + SIGN(dDeltaMv[3]) * 0.5) << mvShift);
    if (extParams)
    {
      acDeltaMv[2] = Mv((int)(dDeltaMv[4] * multiShift + SIGN(dDeltaMv[4]) * 0.5) << mvShift, (int)(dDeltaMv[5] * multiShift + SIGN(dDeltaMv[5]) * 0.5) << mvShift);
    }
    bool bAllZero = false;
    for (int i = 0; i < mvNum; i++)
    {
      Mv deltaMv = acDeltaMv[i];
      if (cu.imv == 2)
      {
        deltaMv.roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_HALF);
      }
      if (deltaMv.hor != 0 || deltaMv.ver != 0)
      {
        bAllZero = false;
        break;
      }
      bAllZero = true;
    }

    if (bAllZero)
      break;

    // do motion compensation with updated mv
    for (int i = 0; i < mvNum; i++)
    {
      acMvTemp[i] += acDeltaMv[i];
      acMvTemp[i].hor = Clip3(MV_MIN, MV_MAX, acMvTemp[i].hor);
      acMvTemp[i].ver = Clip3(MV_MIN, MV_MAX, acMvTemp[i].ver);
      acMvTemp[i].roundAffinePrecInternal2Amvr(cu.imv);

      clipMv(acMvTemp[i], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
    }

    xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.slice->clpRngs[COMP_Y], refPicList);

    // get error
    Distortion uiCostTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->bitDepths[CH_L], COMP_Y, distFunc);
    DTRACE(g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiCostTemp);

    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t uiBitsTemp = ruiBits;
    uiBitsTemp += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
    uiCostTemp = (Distortion)(floor(fWeight * (double)uiCostTemp) + (double)m_pcRdCost->getCost(uiBitsTemp));

    // store best cost and mv
    if (uiCostTemp < uiCostBest)
    {
      uiCostBest = uiCostTemp;
      uiBitsBest = uiBitsTemp;
      memcpy(acMv, acMvTemp, sizeof(Mv) * 3);
      mvpIdx = bestMvpIdx;
    }
    else if(m_pcEncCfg->m_Affine > 1)
    {
      break;
    }
  }

  auto checkCPMVRdCost = [&](Mv ctrlPtMv[3])
  {
    xPredAffineBlk(COMP_Y, cu, refPic, ctrlPtMv, predBuf, false, cu.slice->clpRngs[COMP_Y], refPicList);
    // get error
    Distortion costTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->bitDepths[CH_L], COMP_Y, distFunc);
    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t bitsTemp = ruiBits;
    bitsTemp += xCalcAffineMVBits(cu, ctrlPtMv, acMvPred);
    costTemp = (Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));
    // store best cost and mv
    if (costTemp < uiCostBest)
    {
      uiCostBest = costTemp;
      uiBitsBest = bitsTemp;
      ::memcpy(acMv, ctrlPtMv, sizeof(Mv) * 3);
    }
  };

  const uint32_t mvShiftTable[3] = { MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL - MV_PRECISION_INTERNAL, MV_PRECISION_INTERNAL - MV_PRECISION_INT };
  const uint32_t mvShift = mvShiftTable[cu.imv];
  if (uiCostBest <= AFFINE_ME_LIST_MVP_TH*m_hevcCost)
  {
    Mv mvPredTmp[3] = { acMvPred[0], acMvPred[1], acMvPred[2] };
    Mv mvME[3];
    ::memcpy(mvME, acMv, sizeof(Mv) * 3);
    Mv dMv = mvME[0] - mvPredTmp[0];

    for (int j = 0; j < mvNum; j++)
    {
      if ((!j && mvME[j] != mvPredTmp[j]) || (j && mvME[j] != (mvPredTmp[j] + dMv)))
      {
        ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);
        acMvTemp[j] = mvPredTmp[j];

        if (j)
          acMvTemp[j] += dMv;

        checkCPMVRdCost(acMvTemp);
      }
    }

    //keep the rotation/zoom;
    if (mvME[0] != mvPredTmp[0])
    {
      ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);
      for (int i = 1; i < mvNum; i++)
      {
        acMvTemp[i] -= dMv;
      }
      acMvTemp[0] = mvPredTmp[0];

      checkCPMVRdCost(acMvTemp);
    }

    //keep the translation;
    if (cu.affineType == AFFINEMODEL_6PARAM && mvME[1] != (mvPredTmp[1] + dMv) && mvME[2] != (mvPredTmp[2] + dMv))
    {
      ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);

      acMvTemp[1] = mvPredTmp[1] + dMv;
      acMvTemp[2] = mvPredTmp[2] + dMv;

      checkCPMVRdCost(acMvTemp);
    }

    // 8 nearest neighbor search
    int testPos[8][2] = { { -1, 0 },{ 0, -1 },{ 0, 1 },{ 1, 0 },{ -1, -1 },{ -1, 1 },{ 1, 1 },{ 1, -1 } };
    const int maxSearchRound = 3;

    for (int rnd = 0; rnd < maxSearchRound; rnd++)
    {
      bool modelChange = false;
      //search the model parameters with finear granularity;
      for (int j = 0; j < mvNum; j++)
      {
        bool loopChange = false;
        for (int iter = 0; iter < 2; iter++)
        {
          if (iter == 1 && !loopChange)
          {
            break;
          }
          Mv centerMv[3];
          memcpy(centerMv, acMv, sizeof(Mv) * 3);
          memcpy(acMvTemp, acMv, sizeof(Mv) * 3);

          for (int i = ((iter == 0) ? 0 : 4); i < ((iter == 0) ? 4 : 8); i++)
          {
            acMvTemp[j].set(centerMv[j].hor + (testPos[i][0] << mvShift), centerMv[j].ver + (testPos[i][1] << mvShift));
            clipMv(acMvTemp[j], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
            xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.slice->clpRngs[COMP_Y], refPicList);

            Distortion costTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->bitDepths[CH_L], COMP_Y, distFunc);
            uint32_t bitsTemp = ruiBits;
            bitsTemp += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
            costTemp = (Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));

            if (costTemp < uiCostBest)
            {
              uiCostBest = costTemp;
              uiBitsBest = bitsTemp;
              ::memcpy(acMv, acMvTemp, sizeof(Mv) * 3);
              modelChange = true;
              loopChange = true;
            }
          }
        }
      }

      if (!modelChange)
      {
        break;
      }
    }
  }
  acMvPred[0] = aamvpi.mvCandLT[mvpIdx];
  acMvPred[1] = aamvpi.mvCandRT[mvpIdx];
  acMvPred[2] = aamvpi.mvCandLB[mvpIdx];

  // free buffer
  for (int i = 0; i<iParaNum; i++)
    delete[]pdEqualCoeff[i];
  delete[]pdEqualCoeff;

  ruiBits = uiBitsBest;
  ruiCost = uiCostBest;
  DTRACE(g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest, uiCostBest);
}

void InterSearch::xEstimateAffineAMVP(CodingUnit& cu, AffineAMVPInfo& affineAMVPInfo, CPelUnitBuf& origBuf, RefPicList refPicList, int iRefIdx, Mv acMvPred[3], Distortion& distBiP)
{
  Mv         bestMvLT, bestMvRT, bestMvLB;
  int        iBestIdx = 0;
  Distortion uiBestCost = MAX_DISTORTION;

  // Fill the MV Candidates
  CU::fillAffineMvpCand(cu, refPicList, iRefIdx, affineAMVPInfo);
  CHECK(affineAMVPInfo.numCand == 0, "Assertion failed.");

  PelUnitBuf predBuf = m_tmpStorageLCU.getCompactBuf( cu );

  bool stop_check = false;
  if (affineAMVPInfo.mvCandLT[0] == affineAMVPInfo.mvCandLT[1])
  {
    if ((affineAMVPInfo.mvCandRT[0] == affineAMVPInfo.mvCandRT[1]) && (affineAMVPInfo.mvCandLB[0] == affineAMVPInfo.mvCandLB[1]))
    {
      stop_check = true;
    }
  }

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  for (int i = 0; i < affineAMVPInfo.numCand; i++)
  {
    if (i && stop_check)
    {
      continue;
    }
    Mv mv[3] = { affineAMVPInfo.mvCandLT[i], affineAMVPInfo.mvCandRT[i], affineAMVPInfo.mvCandLB[i] };

    Distortion uiTmpCost = xGetAffineTemplateCost(cu, origBuf, predBuf, mv, i, AMVP_MAX_NUM_CANDS, refPicList, iRefIdx);

    if (uiBestCost > uiTmpCost)
    {
      uiBestCost = uiTmpCost;
      bestMvLT = affineAMVPInfo.mvCandLT[i];
      bestMvRT = affineAMVPInfo.mvCandRT[i];
      bestMvLB = affineAMVPInfo.mvCandLB[i];
      iBestIdx = i;
      distBiP  = uiTmpCost;
    }
  }

  // Setting Best MVP
  acMvPred[0] = bestMvLT;
  acMvPred[1] = bestMvRT;
  acMvPred[2] = bestMvLB;

  cu.mvpIdx[refPicList] = iBestIdx;
  cu.mvpNum[refPicList] = affineAMVPInfo.numCand;
  DTRACE(g_trace_ctx, D_COMMON, "#estAffi=%d \n", affineAMVPInfo.numCand);
}

void InterSearch::xCopyAffineAMVPInfo(AffineAMVPInfo& src, AffineAMVPInfo& dst)
{
  dst.numCand = src.numCand;
  DTRACE(g_trace_ctx, D_COMMON, " (%d) #copyAffi=%d \n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), src.numCand);
  ::memcpy(dst.mvCandLT, src.mvCandLT, sizeof(Mv)*src.numCand);
  ::memcpy(dst.mvCandRT, src.mvCandRT, sizeof(Mv)*src.numCand);
  ::memcpy(dst.mvCandLB, src.mvCandLB, sizeof(Mv)*src.numCand);
}

uint32_t InterSearch::xCalcAffineMVBits(CodingUnit& cu, Mv acMvTemp[3], Mv acMvPred[3])
{
  int mvNum = cu.affineType ? 3 : 2;
  m_pcRdCost->setCostScale(0);
  uint32_t bitsTemp = 0;

  for (int verIdx = 0; verIdx < mvNum; verIdx++)
  {
    Mv pred = verIdx == 0 ? acMvPred[verIdx] : acMvPred[verIdx] + acMvTemp[0] - acMvPred[0];
    pred.changeAffinePrecInternal2Amvr(cu.imv);
    m_pcRdCost->setPredictor(pred);
    Mv mv = acMvTemp[verIdx];
    mv.changeAffinePrecInternal2Amvr(cu.imv);

    bitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
  }

  return bitsTemp;
}


//! set adaptive search range based on poc difference
void InterSearch::setSearchRange( const Slice* slice, const VVEncCfg& encCfg )
{
  if( !encCfg.m_bUseASR || slice->isIRAP() )
  {
    return;
  }

  int iCurrPOC = slice->poc;
  int iRefPOC;
  int iGOPSize = encCfg.m_GOPSize;
  int iOffset = (iGOPSize >> 1);
  int iMaxSR = encCfg.m_SearchRange;
  int iNumPredDir = slice->isInterP() ? 1 : 2;

  for (int iDir = 0; iDir < iNumPredDir; iDir++)
  {
    RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for (int iRefIdx = 0; iRefIdx < slice->numRefIdx[e]; iRefIdx++)
    {
      iRefPOC = slice->getRefPic(e, iRefIdx)->getPOC();
      int newSearchRange = Clip3(encCfg.m_minSearchWindow, iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
      m_aaiAdaptSR[iDir][iRefIdx] = newSearchRange;
    }
  }
}

void InterSearch::xIBCSearchMVCandUpdate(Distortion  sad, int x, int y, Distortion* sadBestCand, Mv* cMVCand)
{
  int j = CHROMA_REFINEMENT_CANDIDATES - 1;

  if (sad < sadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
  {
    for (int t = CHROMA_REFINEMENT_CANDIDATES - 1; t >= 0; t--)
    {
      if (sad < sadBestCand[t])
        j = t;
    }

    for (int k = CHROMA_REFINEMENT_CANDIDATES - 1; k > j; k--)
    {
      sadBestCand[k] = sadBestCand[k - 1];

      cMVCand[k].set(cMVCand[k - 1].hor, cMVCand[k - 1].ver);
    }
    sadBestCand[j] = sad;
    cMVCand[j].set(x, y);
  }
}

int InterSearch::xIBCSearchMVChromaRefine(CodingUnit& cu,
  int         roiWidth,
  int         roiHeight,
  int         cuPelX,
  int         cuPelY,
  Distortion* sadBestCand,
  Mv* cMVCand

)
{
  if ((!isChromaEnabled(cu.chromaFormat)) || (!cu.Cb().valid()))
  {
    return 0;
  }

  int bestCandIdx = 0;
  Distortion  sadBest = std::numeric_limits<Distortion>::max();
  Distortion  tempSad;

  Pel* pRef;
  Pel* pOrg;
  int refStride, orgStride;
  int width, height;

  int picWidth = cu.cs->slice->pps->picWidthInLumaSamples;
  int picHeight = cu.cs->slice->pps->picHeightInLumaSamples;

  UnitArea allCompBlocks(cu.chromaFormat, (Area)cu.block(COMP_Y));
  for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    if (sadBestCand[cand] == std::numeric_limits<Distortion>::max())
    {
      continue;
    }

    if ((!cMVCand[cand].hor) && (!cMVCand[cand].ver))
      continue;

    if (((int)(cuPelY + cMVCand[cand].ver + roiHeight) >= picHeight) || ((cuPelY + cMVCand[cand].ver) < 0))
      continue;

    if (((int)(cuPelX + cMVCand[cand].hor + roiWidth) >= picWidth) || ((cuPelX + cMVCand[cand].hor) < 0))
      continue;

    tempSad = sadBestCand[cand];

    cu.mv[0][0] = cMVCand[cand];
    cu.mv[0][0].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    cu.interDir = 1;
    cu.refIdx[0] = cu.cs->slice->numRefIdx[REF_PIC_LIST_0]; // last idx in the list

    PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_0].getBuf(UnitAreaRelative(cu, cu));
    motionCompensation(cu, predBufTmp, REF_PIC_LIST_0);

    for (unsigned int ch = COMP_Cb; ch < getNumberValidComponents(cu.cs->sps->chromaFormatIdc); ch++)
    {
      width = roiWidth >> getComponentScaleX(ComponentID(ch), cu.chromaFormat);
      height = roiHeight >> getComponentScaleY(ComponentID(ch), cu.chromaFormat);

      PelUnitBuf origBuf = cu.cs->getOrgBuf(allCompBlocks);
      PelUnitBuf* pBuf = &origBuf;
      CPelBuf  tmpPattern = pBuf->get(ComponentID(ch));
      pOrg = (Pel*)tmpPattern.buf;

      Picture* refPic = cu.slice->pic;
      const CPelBuf refBuf = refPic->getRecoBuf(allCompBlocks.blocks[ComponentID(ch)]);
      pRef = (Pel*)refBuf.buf;

      refStride = refBuf.stride;
      orgStride = tmpPattern.stride;

      //ComponentID compID = (ComponentID)ch;
      PelUnitBuf* pBufRef = &predBufTmp;
      CPelBuf  tmpPatternRef = pBufRef->get(ComponentID(ch));
      pRef = (Pel*)tmpPatternRef.buf;
      refStride = tmpPatternRef.stride;


      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          tempSad += ((abs(pRef[col] - pOrg[col])) >> (cu.cs->sps->bitDepths[CH_C] - 8));
        }
        pRef += refStride;
        pOrg += orgStride;
      }
    }

    if (tempSad < sadBest)
    {
      sadBest = tempSad;
      bestCandIdx = cand;
    }
  }

  return bestCandIdx;
}
static unsigned int xMergeCandLists(Mv* dst, unsigned int dn, unsigned int dstTotalLength, Mv* src, unsigned int sn)
{
  for (unsigned int cand = 0; cand < sn && dn < dstTotalLength; cand++)
  {
    if (src[cand] == Mv())
    {
      continue;
    }
    bool found = false;
    for (int j = 0; j < dn; j++)
    {
      if (src[cand] == dst[j])
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      dst[dn] = src[cand];
      dn++;
    }
  }

  return dn;
}
void InterSearch::xIntraPatternSearchIBC(CodingUnit& cu, TZSearchStruct& cStruct, Mv& rcMv, Distortion& ruiCost, Mv* pcMvSrchRngLT, Mv* pcMvSrchRngRB, Mv* pcMvPred)
{
  const int   srchRngHorLeft = pcMvSrchRngLT->hor;
  const int   srchRngHorRight = pcMvSrchRngRB->hor;
  const int   srchRngVerTop = pcMvSrchRngLT->ver;
  const int   srchRngVerBottom = pcMvSrchRngRB->ver;

  const unsigned int  lcuWidth = cu.cs->slice->sps->CTUSize;
  const int   puPelOffsetX = 0;
  const int   puPelOffsetY = 0;
  const int   cuPelX = cu.Y().x;
  const int   cuPelY = cu.Y().y;

  int          roiWidth = cu.lwidth();
  int          roiHeight = cu.lheight();

  Distortion  sad;
  Distortion  sadBest = std::numeric_limits<Distortion>::max();
  int         bestX = 0;
  int         bestY = 0;

  const Pel* piRefSrch = cStruct.piRefY; 

  int         bestCandIdx = 0;

  Distortion  sadBestCand[CHROMA_REFINEMENT_CANDIDATES];
  Mv          cMVCand[CHROMA_REFINEMENT_CANDIDATES];

  const bool  useAmvr = cu.cs->sps->AMVR;


  for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    sadBestCand[cand] = std::numeric_limits<Distortion>::max();
    cMVCand[cand].set(0, 0);
  }

  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMP_Y, cStruct.subShiftMode);

  const int picWidth = cu.cs->slice->pps->picWidthInLumaSamples;
  const int picHeight = cu.cs->slice->pps->picHeightInLumaSamples;


  {
    m_cDistParam.subShift = 0;
    Distortion tempSadBest = 0;

    int srLeft = srchRngHorLeft, srRight = srchRngHorRight, srTop = srchRngVerTop, srBottom = srchRngVerBottom;
    m_numBVs = 0;
    m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), m_defaultCachedBvs->m_bvCands, m_defaultCachedBvs->currCnt);

    Mv cMvPredEncOnly[IBC_NUM_CANDIDATES];
    int nbPreds = 0;
    CU::getIbcMVPsEncOnly(cu, cMvPredEncOnly, nbPreds);
    m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), cMvPredEncOnly, nbPreds);

    for (unsigned int cand = 0; cand < m_numBVs; cand++)
    {
      int xPred = m_acBVs[cand].hor;
      int yPred = m_acBVs[cand].ver;

      if (!(xPred == 0 && yPred == 0)
        && !((yPred < srTop) || (yPred > srBottom))
        && !((xPred < srLeft) || (xPred > srRight)))
      {
        bool validCand = searchBvIBC(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth);

        if (validCand)
        {
          sad = m_pcRdCost->getBvCostMultiplePredsIBC(xPred, yPred, useAmvr);
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * yPred + xPred;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, xPred, yPred, sadBestCand, cMVCand);
        }
      }
    }

    bestX = cMVCand[0].hor;
    bestY = cMVCand[0].ver;
    rcMv.set(bestX, bestY);
    sadBest = sadBestCand[0];

    const int boundY = (0 - roiHeight - puPelOffsetY);
    for (int y = std::max(srchRngVerTop, 0 - cuPelY); y <= boundY; ++y)
    {
      if (!searchBvIBC(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, y, lcuWidth))
      {
        continue;
      }

      sad = m_pcRdCost->getBvCostMultiplePredsIBC(0, y, useAmvr);
      m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y;
      sad += m_cDistParam.distFunc(m_cDistParam);

      xIBCSearchMVCandUpdate(sad, 0, y, sadBestCand, cMVCand);
      tempSadBest = sadBestCand[0];
      if (sadBestCand[0] <= 3)
      {
        bestX = cMVCand[0].hor;
        bestY = cMVCand[0].ver;
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }
    }

    const int boundX = std::max(srchRngHorLeft, -cuPelX);
    for (int x = 0 - roiWidth - puPelOffsetX; x >= boundX; --x)
    {
      if (!searchBvIBC(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, 0, lcuWidth))
      {
        continue;
      }

      sad = m_pcRdCost->getBvCostMultiplePredsIBC(x, 0, useAmvr);
      m_cDistParam.cur.buf = piRefSrch + x;
      sad += m_cDistParam.distFunc(m_cDistParam);


      xIBCSearchMVCandUpdate(sad, x, 0, sadBestCand, cMVCand);
      tempSadBest = sadBestCand[0];
      if (sadBestCand[0] <= 3)
      {
        bestX = cMVCand[0].hor;
        bestY = cMVCand[0].ver;
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }
    }

    bestX = cMVCand[0].hor;
    bestY = cMVCand[0].ver;
    sadBest = sadBestCand[0];
    if ((!bestX && !bestY) || (sadBest - m_pcRdCost->getBvCostMultiplePredsIBC(bestX, bestY, useAmvr) <= 32))
    {
      //chroma refine
      bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
      bestX = cMVCand[bestCandIdx].hor;
      bestY = cMVCand[bestCandIdx].ver;
      sadBest = sadBestCand[bestCandIdx];
      rcMv.set(bestX, bestY);
      ruiCost = sadBest;
      goto end;
    }

    if (cu.lwidth() < 16 && cu.lheight() < 16)
    {
      int stepS = 2;
      if (m_pcEncCfg->m_IBCFastMethod > 2)
      {
        if (m_pcEncCfg->m_IBCFastMethod == 5)
        {
          stepS = 8;
        }
        else if ((cu.lwidth() > 4) || (cu.lheight() > 4))
        {
          stepS = 4;
        }
      }

      const int minCuLog2 = m_pcEncCfg->m_log2MinCodingBlockSize;
      const int minCuMask = (1 << minCuLog2) - 1;
      bool lastDec = false;

      for (int searchStep = 0; searchStep < 3; searchStep++)
      {
        int delaySy = searchStep ? 1 : 0;
        int delaySx = searchStep > 1 ? 1 : 0;
        int startY = (std::max(srchRngVerTop, -cuPelY) + delaySy);
        int startX = (std::max(srchRngHorLeft, -cuPelX) + delaySx);
        int endY = srchRngVerBottom;
        int endX = srchRngHorRight;

        if (m_pcEncCfg->m_IBCFastMethod > 5)
        {
          startY = bestY - 4;
          endY = bestY + 4;
          startX = bestX - 4;
          endX = bestX + 4;
          stepS = 1;
          if (searchStep)
          {
            break;
          }
        }

        for (int y = startY; y <= endY; y += stepS)
        {
          if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
            continue;
          bool firstX = true;
          int stepSx = searchStep ? stepS : 1;
          for (int x = startX; x <= endX; firstX = false, x += stepSx)
          {
            if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
              continue;

            bool isSameAsLast = !firstX && ((cuPelX + x) & minCuMask) > 1;
            if (searchStep || (m_pcEncCfg->m_IBCFastMethod > 5))
            {
              if (!searchBvIBC(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
              {
                continue;
              }
            }
            else if ((isSameAsLast && !lastDec) || (!isSameAsLast && !searchBvIBC(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth)))
            {
              lastDec = false;
              continue;
            }
            lastDec = true;

            sad = m_pcRdCost->getBvCostMultiplePredsIBC(x, y, useAmvr);
            m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
            sad += m_cDistParam.distFunc(m_cDistParam);

            xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);


            if (searchStep && sadBestCand[0] <= 5)
            {
              //chroma refine & return
              bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
              bestX = cMVCand[bestCandIdx].hor;
              bestY = cMVCand[bestCandIdx].ver;
              sadBest = sadBestCand[bestCandIdx];
              rcMv.set(bestX, bestY);
              ruiCost = sadBest;
              goto end;
            }
          }
        }

        if ((searchStep < 2) && (m_pcEncCfg->m_IBCFastMethod < 6))
        {
          if ((m_pcEncCfg->m_IBCFastMethod > 2) && (m_pcEncCfg->m_IBCFastMethod < 5))
          {
            if ((bestX == cMVCand[0].hor) && (bestY == cMVCand[0].ver))
            {
              sadBest = sadBestCand[bestCandIdx];
              rcMv.set(bestX, bestY);
              ruiCost = sadBest;
              goto end;
            }
          }
          bestX = cMVCand[0].hor;
          bestY = cMVCand[0].ver;
          sadBest = sadBestCand[0];

          int StopSearch = searchStep ? 32 : 16;
          if ((searchStep && (sadBest >= tempSadBest)) || (sadBest - m_pcRdCost->getBvCostMultiplePredsIBC(bestX, bestY, useAmvr) <= StopSearch))
          {
            //chroma refine
            bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);

            bestX = cMVCand[bestCandIdx].hor;
            bestY = cMVCand[bestCandIdx].ver;
            sadBest = sadBestCand[bestCandIdx];
            rcMv.set(bestX, bestY);
            ruiCost = sadBest;
            goto end;
          }
        }
      }
    }
  }

  bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);

  bestX = cMVCand[bestCandIdx].hor;
  bestY = cMVCand[bestCandIdx].ver;
  sadBest = sadBestCand[bestCandIdx];
  rcMv.set(bestX, bestY);
  ruiCost = sadBest;

end:
  m_numBVs = 0;
  m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), m_defaultCachedBvs->m_bvCands, m_defaultCachedBvs->currCnt);

  m_defaultCachedBvs->currCnt = 0;
  m_defaultCachedBvs->currCnt = xMergeCandLists(m_defaultCachedBvs->m_bvCands, m_defaultCachedBvs->currCnt, IBC_NUM_CANDIDATES, cMVCand, CHROMA_REFINEMENT_CANDIDATES);
  m_defaultCachedBvs->currCnt = xMergeCandLists(m_defaultCachedBvs->m_bvCands, m_defaultCachedBvs->currCnt, IBC_NUM_CANDIDATES, m_acBVs, m_numBVs);

  for (unsigned int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    if (cMVCand[cand].hor == 0 && cMVCand[cand].ver == 0)
    {
      continue;
    }
    m_ctuRecord[cu.lumaPos()][cu.lumaSize()].bvRecord[cMVCand[cand]] = sadBestCand[cand];
  }

  return;
}



// based on xMotionEstimation
void InterSearch::xIBCEstimation(CodingUnit& cu, PelUnitBuf& origBuf, Mv* pcMvPred, Mv& rcMv, Distortion& ruiCost )
{
  const int iPicWidth = cu.cs->slice->pps->picWidthInLumaSamples;
  const int iPicHeight = cu.cs->slice->pps->picHeightInLumaSamples;
  const unsigned int  lcuWidth = cu.cs->slice->sps->CTUSize;
  const int           cuPelX = cu.Y().x;
  const int           cuPelY = cu.Y().y;
  int                 iRoiWidth = cu.lwidth();
  int                 iRoiHeight = cu.lheight();

  PelUnitBuf* pBuf = &origBuf;

  //  Search key pattern initialization
  CPelBuf  tmpPattern = pBuf->Y();
  CPelBuf* pcPatternKey = &tmpPattern;
  PelBuf tmpOrgLuma;
  ReshapeData& reshapeData = cu.cs->picture->reshapeData;
  if ((cu.cs->slice->lmcsEnabled && reshapeData.getCTUFlag()))
  {
    tmpOrgLuma = m_tmpStorageLCU.getCompactBuf(cu.Y());
    tmpOrgLuma.rspSignal(tmpPattern, reshapeData.getInvLUT());
    pcPatternKey = (CPelBuf*)&tmpOrgLuma;
  }
  m_lumaClpRng = cu.cs->slice->clpRngs[COMP_Y];
  Picture* refPic = cu.slice->pic;
  const CPelBuf refBuf = refPic->getRecoBuf(cu.blocks[COMP_Y]);

  TZSearchStruct cStruct; 
  cStruct.pcPatternKey = pcPatternKey;
  cStruct.iRefStride = refBuf.stride;
  cStruct.piRefY = refBuf.buf;
  CHECK(cu.imv == IMV_HPEL, "IF_IBC");
  cStruct.imvShift = cu.imv << 1;
  cStruct.subShiftMode = 0; 

  m_pcRdCost->getMotionCostIBC(0);
  m_pcRdCost->setPredictorsIBC(pcMvPred);
  m_pcRdCost->setCostScale(0);

  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMP_Y, cStruct.subShiftMode);
  bool buffered = false;
  if (m_pcEncCfg->m_IBCFastMethod)// IBC_FAST_METHOD_BUFFERBV
  {
    ruiCost = MAX_UINT;
    std::unordered_map<Mv, Distortion>& history = m_ctuRecord[cu.lumaPos()][cu.lumaSize()].bvRecord;
    for (std::unordered_map<Mv, Distortion>::iterator p = history.begin(); p != history.end(); p++)
    {
      const Mv& bv = p->first;

      int xBv = bv.hor;
      int yBv = bv.ver;
      if (searchBvIBC(cu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xBv, yBv, lcuWidth))
      {
        buffered = true;
        Distortion sad = m_pcRdCost->getBvCostMultiplePredsIBC(xBv, yBv, cu.cs->sps->AMVR);
        m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yBv + xBv;
        sad += m_cDistParam.distFunc(m_cDistParam);
        if (sad < ruiCost)
        {
          rcMv = bv;
          ruiCost = sad;
        }
        else if (sad == ruiCost)
        {
          // stabilise the search through the unordered list
          if (bv.hor < rcMv.hor
            || (bv.hor == rcMv.hor && bv.ver < rcMv.ver))
          {
            // update the vector.
            rcMv = bv;
          }
        }
      }
    }

    if (buffered)
    {
      Mv cMvPredEncOnly[IBC_NUM_CANDIDATES];
      int nbPreds = 0;
      CU::getIbcMVPsEncOnly(cu, cMvPredEncOnly, nbPreds);

      for (unsigned int cand = 0; cand < nbPreds; cand++)
      {
        int xPred = cMvPredEncOnly[cand].hor;
        int yPred = cMvPredEncOnly[cand].ver;

        if (searchBvIBC(cu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xPred, yPred, lcuWidth))
        {
          Distortion sad = m_pcRdCost->getBvCostMultiplePredsIBC(xPred, yPred, cu.cs->sps->AMVR);
          m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yPred + xPred;
          sad += m_cDistParam.distFunc(m_cDistParam);
          if (sad < ruiCost)
          {
            rcMv.set(xPred, yPred);
            ruiCost = sad;
          }
          else if (sad == ruiCost)
          {
            // stabilise the search through the unordered list
            if (xPred < rcMv.hor
              || (xPred == rcMv.hor && yPred < rcMv.ver))
            {
              // update the vector.
              rcMv.set(xPred, yPred);
            }
          }
          m_ctuRecord[cu.lumaPos()][cu.lumaSize()].bvRecord[Mv(xPred, yPred)] = sad;
        }
      }
    }
  }

  if (!buffered)
  {
    Mv        cMvSrchRngLT;
    Mv        cMvSrchRngRB;

    // assume that intra BV is integer-pel precision
    xSetIntraSearchRangeIBC(cu, cu.lwidth(), cu.lheight(), cMvSrchRngLT, cMvSrchRngRB);

    //  Do integer search
    xIntraPatternSearchIBC(cu, cStruct, rcMv, ruiCost, &cMvSrchRngLT, &cMvSrchRngRB, pcMvPred);
  }
}
// based on xSetSearchRange
void InterSearch::xSetIntraSearchRangeIBC(CodingUnit& cu, int iRoiWidth, int iRoiHeight, Mv& rcMvSrchRngLT, Mv& rcMvSrchRngRB)
{
 // const SPS& sps = *cu.cs->sps;

  int srLeft, srRight, srTop, srBottom;

  const int cuPelX = cu.Y().x;
  const int cuPelY = cu.Y().y;

  const int lcuWidth = cu.cs->slice->sps->CTUSize;
  const int ctuSizeLog2 = floorLog2(lcuWidth);
  int numLeftCTUs = (1 << ((7 - ctuSizeLog2) << 1)) - ((ctuSizeLog2 < 7) ? 1 : 0);

  srLeft = -(numLeftCTUs * lcuWidth + (cuPelX % lcuWidth));
  srTop = -(cuPelY % lcuWidth);

  srRight = lcuWidth - (cuPelX % lcuWidth) - iRoiWidth;
  srBottom = lcuWidth - (cuPelY % lcuWidth) - iRoiHeight;

  rcMvSrchRngLT.hor=srLeft;
  rcMvSrchRngLT.ver=srTop;
  rcMvSrchRngRB.hor=srRight;
  rcMvSrchRngRB.ver=srBottom;

  rcMvSrchRngLT <<= 2;
  rcMvSrchRngRB <<= 2;
  bool temp = m_clipMvInSubPic;
  m_clipMvInSubPic = true;
  clipMv(rcMvSrchRngLT,cu.lumaPos(),cu.lumaSize(), *cu.cs->pcv, *cu.cs->pps, m_clipMvInSubPic);
  clipMv(rcMvSrchRngRB, cu.lumaPos(),cu.lumaSize(), *cu.cs->pcv, * cu.cs->pps, m_clipMvInSubPic);
  m_clipMvInSubPic = temp;
  rcMvSrchRngLT >>= 2;
  rcMvSrchRngRB >>= 2;
}

bool InterSearch::predIBCSearch(CodingUnit& cu, Partitioner& partitioner)
{
  Mv           cMvSrchRngLT;
  Mv           cMvSrchRngRB;
  cu.imv = 2;
  AMVPInfo amvpInfo4Pel;
  CU::fillIBCMvpCand(cu, amvpInfo4Pel);

  cu.imv = 0;// (Int)cu.cs->sps->getUseIMV(); // set as IMV=0 initially
  Mv    cMv, cMvPred[2];
  AMVPInfo amvpInfo;
  CU::fillIBCMvpCand(cu, amvpInfo);
  // store in full pel accuracy, shift before use in search
  cMvPred[0] = amvpInfo.mvCand[0];
  cMvPred[0].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  cMvPred[1] = amvpInfo.mvCand[1];
  cMvPred[1].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);

  int iBvpNum = 2;
  int bvpIdxBest = 0;
  cMv.setZero();
  Distortion cost = 0;
  if (cu.cs->sps->maxNumIBCMergeCand == 1)
  {
    iBvpNum = 1;
    cMvPred[1] = cMvPred[0];
  }

  if (cMv.hor == 0 && cMv.ver == 0)
  {
    // if hash search does not work or is not enabled
    PelUnitBuf origBuf = cu.cs->getOrgBuf(cu);
    xIBCEstimation(cu, origBuf, cMvPred, cMv, cost );
  }

  if (cMv.hor == 0 && cMv.ver == 0)
  {
    return false;
  }
  /// ibc search
  /////////////////////////////////////////////////////////
  unsigned int bitsBVPBest, bitsBVPTemp;
  bitsBVPBest = MAX_INT;
  m_pcRdCost->setCostScale(0);

  for (int bvpIdxTemp = 0; bvpIdxTemp < iBvpNum; bvpIdxTemp++)
  {
    m_pcRdCost->setPredictor(cMvPred[bvpIdxTemp]);

    bitsBVPTemp = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.hor, cMv.ver, 0);

    if (bitsBVPTemp < bitsBVPBest)
    {
      bitsBVPBest = bitsBVPTemp;
      bvpIdxBest = bvpIdxTemp;

      if (cu.cs->sps->AMVR && cMv != cMvPred[bvpIdxTemp])
        cu.imv = 1; // set as full-pel
      else
        cu.imv = 0; // set as fractional-pel

    }

    unsigned int bitsBVPQP = MAX_UINT;


    Mv mvPredQuadPel;
    if ((cMv.hor % 4 == 0) && (cMv.ver % 4 == 0) && (cu.cs->sps->AMVR))
    {
      mvPredQuadPel = amvpInfo4Pel.mvCand[bvpIdxTemp];// cMvPred[bvpIdxTemp];

      mvPredQuadPel.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_4PEL);

      m_pcRdCost->setPredictor(mvPredQuadPel);

      bitsBVPQP = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.hor >> 2, cMv.ver >> 2, 0);

    }
    mvPredQuadPel.changePrecision(MV_PRECISION_4PEL, MV_PRECISION_INT);
    if (bitsBVPQP < bitsBVPBest && cMv != mvPredQuadPel)
    {
      bitsBVPBest = bitsBVPQP;
      bvpIdxBest = bvpIdxTemp;

      if (cu.cs->sps->AMVR)
        cu.imv = 2; // set as quad-pel
    }

  }

  cu.bv = cMv; // bv is always at integer accuracy
  cMv.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
  cu.mv[REF_PIC_LIST_0][0] = cMv; // store in fractional pel accuracy

  cu.mvpIdx[REF_PIC_LIST_0] = bvpIdxBest;

  if (cu.imv == 2 && cMv != amvpInfo4Pel.mvCand[bvpIdxBest])
    cu.mvd[REF_PIC_LIST_0][0] = cMv - amvpInfo4Pel.mvCand[bvpIdxBest];
  else
    cu.mvd[REF_PIC_LIST_0][0] = cMv - amvpInfo.mvCand[bvpIdxBest];

  if (cu.mvd[REF_PIC_LIST_0][0] == Mv(0, 0))
    cu.imv = 0;
  if (cu.imv == 2)
    assert((cMv.hor % 16 == 0) && (cMv.ver % 16 == 0));
  if (cu.cs->sps->AMVR)
    assert(cu.imv > 0 || cu.mvd[REF_PIC_LIST_0][0] == Mv());

  cu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;

  return true;
}


static inline bool isYPartBefore( SplitSeries series, const int ctuSizeLog2, const Position& refPos, const Position& pos )
{
#ifndef NDEBUG
  const int refCtuX = refPos.x >> ctuSizeLog2;
  const int refCtuY = refPos.y >> ctuSizeLog2;
  const int posCtuX = pos.x >> ctuSizeLog2;
  const int posCtuY = pos.y >> ctuSizeLog2;

  CHECK( refCtuX != posCtuX || refCtuY != posCtuY, "This method can only be applied for positions within the same CTU" );

#endif
  const int ctuMask = ( 1 << ctuSizeLog2 ) - 1;

  const int refX = refPos.x & ctuMask;
  const int refY = refPos.y & ctuMask;
  const int posX = pos.x & ctuMask;
  const int posY = pos.y & ctuMask;

  int x = 0, y = 0, w = 1 << ctuSizeLog2, h = 1 << ctuSizeLog2;
  
  while( true )
  {
    PartSplit split = PartSplit( series & SPLIT_MASK );

    switch( split )
    {
    case CU_QUAD_SPLIT:
      w >>= 1;
      if( posX >= x + w ) x += w;
    case CU_HORZ_SPLIT:
      h >>= 1;
      if( posY >= y + h ) y += h;
      break;

    case CU_VERT_SPLIT:
      w >>= 1;
      if( posX >= x + w ) x += w;
      goto checkXonly;

    case CU_TRIH_SPLIT:
      h >>= 2;
      if( posY >= y + h ) { y += h; h <<= 1; }
      if( posY >= y + h ) { y += h; h >>= 1; }
      break;

    case CU_TRIV_SPLIT:
      w >>= 2;
      if( posX >= x + w ) { x += w; w <<= 1; }
      if( posX >= x + w ) { x += w; w >>= 1; }
      goto checkXonly;

    default:
      return false;
    }

    if( refY >= y + h ) return true;
    else if( refY < y ) return false;

checkXonly:
    if( refX >= x + w ) return true;
    else if( refX < x ) return false;

    series >>= SPLIT_DMULT; continue;
  }

  return false;
}

bool InterSearch::searchBvIBC(const CodingUnit& cu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xBv, int yBv, int ctuSize) const
{
  const int ctuSizeLog2 = Log2(ctuSize);

  int refRightX  = xPos + xBv + width  - 1;
  int refBottomY = yPos + yBv + height - 1;

  int refLeftX = xPos + xBv;
  int refTopY  = yPos + yBv;

  if ((xPos + xBv) < 0)
  {
    return false;
  }
  if (refRightX >= picWidth)
  {
    return false;
  }

  if ((yPos + yBv) < 0)
  {
    return false;
  }
  if (refBottomY >= picHeight)
  {
    return false;
  }
  if ((xBv + width) > 0 && (yBv + height) > 0)
  {
    return false;
  }

  // Don't search the above CTU row
  if (refTopY >> ctuSizeLog2 < yPos >> ctuSizeLog2)
    return false;

  // Don't search the below CTU row
  if (refBottomY >> ctuSizeLog2 > yPos >> ctuSizeLog2)
  {
    return false;
  }

  unsigned curTileIdx = cu.cs->pps->getTileIdx(cu.lumaPos());
  unsigned refTileIdx = cu.cs->pps->getTileIdx(Position(refLeftX, refTopY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = cu.cs->pps->getTileIdx(Position(refLeftX, refBottomY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = cu.cs->pps->getTileIdx(Position(refRightX, refTopY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = cu.cs->pps->getTileIdx(Position(refRightX, refBottomY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }

  const Position cuPos{ xPos, yPos };

  //int numLeftCTUs = (1 << ((7 - ctuSizeLog2) << 1)) - ((ctuSizeLog2 < 7) ? 1 : 0);
  static const int numLeftCTUsLUT[3] = { 15, 3, 1 };

  // in the same CTU line
  const int numLeftCTUs = numLeftCTUsLUT[ctuSizeLog2 - 5];

  if( ( refRightX >> ctuSizeLog2 <= xPos >> ctuSizeLog2 ) && ( refLeftX >> ctuSizeLog2 >= ( xPos >> ctuSizeLog2 ) - numLeftCTUs ) )
  {
    // in the same CTU, or left CTU
    // if part of ref block is in the left CTU, some area can be referred from the not-yet updated local CTU buffer
    if( ( ctuSizeLog2 == 7 ) && ( ( refLeftX >> ctuSizeLog2 ) == ( ( xPos >> ctuSizeLog2 ) - 1 ) ) )
    {
      // ref block's collocated block in current CTU
      const Position refPosCol64x64{ ( refLeftX + ctuSize ) & ~63, refTopY & ~63 };
      if( refPosCol64x64 == Position{ xPos & ~63, yPos & ~63 } )
        return false;

      //CodingUnit* curef = cu.cs->getCU(refPosCol64x64, CH_L, cu.treeType);
      //bool isDecomp = curef && ((cu.cs != curef->cs) || cu.idx < curef->idx);
      bool isDecomp = isYPartBefore( cu.splitSeries, ctuSizeLog2, cuPos, refPosCol64x64 );
      if( isDecomp )
      {
        return false;
      }
    }
  }
  else
    return false;

  // in the same CTU, or valid area from left CTU. Check if the reference block is already coded
  const Position refPosBR{ refRightX, refBottomY };
  //CodingUnit* curef = cu.cs->getCU(refPosBR, CH_L, cu.treeType);
  //bool isDecomp = curef && ((cu.cs != curef->cs) || cu.idx < curef->idx);
  bool isDecomp = ( ( refPosBR.x >> ctuSizeLog2 ) < ( cuPos.x >> ctuSizeLog2 ) ) || ( refRightX < xPos && refBottomY < yPos ) || isYPartBefore( cu.splitSeries, ctuSizeLog2, cuPos, refPosBR );

  return isDecomp;
}

} // namespace vvenc

//! \}


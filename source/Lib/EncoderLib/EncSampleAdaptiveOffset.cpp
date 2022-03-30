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


/**
 \file     EncSampleAdaptiveOffset.cpp
 \brief       estimation part of sample adaptive offset class
 */

#include "EncSampleAdaptiveOffset.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/CodingStructure.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "vvenc/vvencCfg.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {


#define SAOCtx(c) SubCtx( Ctx::Sao, c )


//! rounding with IBDI
inline double xRoundIbdi2(int bitDepth, double x)
{
#if FULL_NBIT
  return ((x) >= 0 ? ((int)((x) + 0.5)) : ((int)((x) -0.5)));
#else
  if (DISTORTION_PRECISION_ADJUSTMENT(bitDepth) == 0)
    return ((x) >= 0 ? ((int)((x) + 0.5)) : ((int)((x) -0.5)));
  else
    return ((x) > 0) ? (int)(((int)(x) + (1 << (DISTORTION_PRECISION_ADJUSTMENT(bitDepth) - 1)))
                             / (1 << DISTORTION_PRECISION_ADJUSTMENT(bitDepth)))
                     : ((int)(((int)(x) - (1 << (DISTORTION_PRECISION_ADJUSTMENT(bitDepth) - 1)))
                              / (1 << DISTORTION_PRECISION_ADJUSTMENT(bitDepth))));
#endif
}

inline double xRoundIbdi(int bitDepth, double x)
{
  return (bitDepth > 8 ? xRoundIbdi2(bitDepth, (x)) : ((x)>=0 ? ((int)((x)+0.5)) : ((int)((x)-0.5)))) ;
}


EncSampleAdaptiveOffset::EncSampleAdaptiveOffset()
  : m_CABACEstimator( nullptr )
  , m_CtxCache      ( nullptr )
{
}

EncSampleAdaptiveOffset::~EncSampleAdaptiveOffset()
{
}

void EncSampleAdaptiveOffset::init( const VVEncCfg& encCfg )
{
  m_EncCfg = &encCfg;

  if ( encCfg.m_bUseSAO )
  {
    SampleAdaptiveOffset::init( encCfg.m_internChromaFormat, encCfg.m_CTUSize, encCfg.m_CTUSize, encCfg.m_log2SaoOffsetScale[CH_L], encCfg.m_log2SaoOffsetScale[CH_C] );
  }
}

void EncSampleAdaptiveOffset::initSlice( const Slice* slice )
{
  memcpy( m_lambda, slice->getLambdas(), sizeof( m_lambda ) );
}

void EncSampleAdaptiveOffset::setCtuEncRsrc( CABACWriter* cabacEstimator, CtxCache* ctxCache )
{
  m_CABACEstimator = cabacEstimator;
  m_CtxCache       = ctxCache;
}

void EncSampleAdaptiveOffset::disabledRate( CodingStructure& cs, double saoDisabledRate[ MAX_NUM_COMP ][ VVENC_MAX_TLAYER ], SAOBlkParam* reconParams, const double saoEncodingRate, const double saoEncodingRateChroma, const ChromaFormat& chromaFormat )
{
  if ( saoEncodingRate > 0.0 )
  {
    const PreCalcValues& pcv     = *cs.pcv;
    const int numberOfComponents = getNumberValidComponents( chromaFormat );
    const int picTempLayer       = cs.slice->depth;
    int numCtusForSAOOff[MAX_NUM_COMP];

    for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      numCtusForSAOOff[compIdx] = 0;
      for( int ctuRsAddr=0; ctuRsAddr< pcv.sizeInCtus; ctuRsAddr++)
      {
        if( reconParams[ctuRsAddr][compIdx].modeIdc == SAO_MODE_OFF)
        {
          numCtusForSAOOff[compIdx]++;
        }
      }
    }
    if (saoEncodingRateChroma > 0.0)
    {
      for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        saoDisabledRate[compIdx][picTempLayer] = (double)numCtusForSAOOff[compIdx]/(double)pcv.sizeInCtus;
      }
    }
    else if (picTempLayer == 0)
    {
      saoDisabledRate[COMP_Y][0] = (double)(numCtusForSAOOff[COMP_Y]+numCtusForSAOOff[COMP_Cb]+numCtusForSAOOff[COMP_Cr])/(double)(pcv.sizeInCtus *3);
    }
  }
}

void EncSampleAdaptiveOffset::decidePicParams( const CodingStructure& cs, double saoDisabledRate[ MAX_NUM_COMP ][ VVENC_MAX_TLAYER ], bool saoEnabled[ MAX_NUM_COMP ], const double saoEncodingRate, const double saoEncodingRateChroma, const ChromaFormat& chromaFormat )
{
  const Slice& slice           = *cs.slice;
  const int numberOfComponents = getNumberValidComponents( chromaFormat );

  // reset
  if( slice.pendingRasInit )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
    {
      for( int tempLayer = 1; tempLayer < VVENC_MAX_TLAYER; tempLayer++ )
      {
        saoDisabledRate[ compIdx ][ tempLayer ] = 0.0;
      }
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    saoEnabled[ compIdx ] = false;
  }

  const int picTempLayer = slice.depth;
  for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
  {
    // enable per default
    saoEnabled[ compIdx ] = true;

    if( saoEncodingRate > 0.0 )
    {
      if( saoEncodingRateChroma > 0.0 )
      {
        // decide slice-level on/off based on previous results
        if( ( picTempLayer > 0 )
          && ( saoDisabledRate[ compIdx ][ picTempLayer - 1 ] > ( ( compIdx == COMP_Y ) ? saoEncodingRate : saoEncodingRateChroma ) ) )
        {
          saoEnabled[ compIdx ] = false;
        }
      }
      else
      {
        // decide slice-level on/off based on previous results
        if( ( picTempLayer > 0 )
          && ( saoDisabledRate[ COMP_Y ][ 0 ] > saoEncodingRate ) )
        {
          saoEnabled[ compIdx ] = false;
        }
      }
    }
  }
}

void EncSampleAdaptiveOffset::storeCtuReco( CodingStructure& cs, const UnitArea& ctuArea )
{
  const int STORE_CTU_INCREASE = 8;
  const PreCalcValues& pcv = *cs.pcv;
  Position lPos( ctuArea.lx() + STORE_CTU_INCREASE, ctuArea.ly() + STORE_CTU_INCREASE );
  Size     lSize( std::min( ctuArea.lwidth(), pcv.lumaWidth - lPos.x ), std::min( ctuArea.lheight(), pcv.lumaHeight - lPos.y ) );
  if ( ctuArea.lx() == 0 )
  {
    lPos.x       = 0;
    lSize.width += STORE_CTU_INCREASE;
  }
  if ( ctuArea.ly() == 0 )
  {
    lPos.y        = 0;
    lSize.height += STORE_CTU_INCREASE;
  }
  const UnitArea relocArea( ctuArea.chromaFormat, Area( lPos, lSize ) );
  Picture& pic       = *cs.picture;
  PelUnitBuf recoYuv = pic.getRecoBuf().subBuf( relocArea );
  PelUnitBuf tempYuv = pic.getSaoBuf().subBuf( relocArea );
  tempYuv.copyFrom( recoYuv );
}

void EncSampleAdaptiveOffset::getCtuStatistics( CodingStructure& cs, std::vector<SAOStatData**>& saoStatistics, const UnitArea& ctuArea, const int ctuRsAddr )
{
  const PreCalcValues& pcv     = *cs.pcv;
  const int numberOfComponents = getNumberValidComponents( pcv.chrFormat );
  bool isLeftAvail             = false;
  bool isRightAvail            = false;
  bool isAboveAvail            = false;
  bool isBelowAvail            = false;
  bool isAboveLeftAvail        = false;
  bool isAboveRightAvail       = false;

  deriveLoopFilterBoundaryAvailibility( cs, ctuArea.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail );

  // NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
  // For simplicity, here only picture boundaries are considered.

  isRightAvail      = ( ctuArea.Y().x + pcv.maxCUSize < pcv.lumaWidth  );
  isBelowAvail      = ( ctuArea.Y().y + pcv.maxCUSize < pcv.lumaHeight );
  isAboveRightAvail = ( ( ctuArea.Y().y > 0 ) && ( isRightAvail ) );

  //VirtualBoundaries vb;
  //bool isCtuCrossedByVirtualBoundaries = vb.isCrossedByVirtualBoundaries(xPos, yPos, width, height, cs.slice->pps);

  for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    const CompArea& compArea = ctuArea.block( compID );

    PelBuf srcBuf = cs.picture->getSaoBuf().get( compID );
    PelBuf orgBuf = cs.picture->getOrigBuf().get( compID );

    getBlkStats( compID,
                 cs.sps->bitDepths[ toChannelType( compID ) ],
                 saoStatistics[ ctuRsAddr ][ compID ],
                 srcBuf.bufAt( compArea ),
                 orgBuf.bufAt( compArea ),
                 srcBuf.stride,
                 orgBuf.stride,
                 compArea.width,
                 compArea.height,
                 isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail
               );
  }
}

void EncSampleAdaptiveOffset::getStatistics(std::vector<SAOStatData**>& blkStats, PelUnitBuf& orgYuv, PelUnitBuf& srcYuv, CodingStructure& cs )
{
  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail;

  const PreCalcValues& pcv = *cs.pcv;
  const int numberOfComponents = getNumberValidComponents(pcv.chrFormat);

  size_t lineBufferSize = pcv.maxCUSize + 1;
  if (m_signLineBuf1.size() != lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }

  int ctuRsAddr = 0;
  for( uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUSize )
  {
    for( uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUSize )
    {
      const uint32_t width  = (xPos + pcv.maxCUSize  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUSize;
      const uint32_t height = (yPos + pcv.maxCUSize > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUSize;
      const UnitArea area( cs.area.chromaFormat, Area(xPos , yPos, width, height) );

      deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail );

      //NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
      //For simplicity, here only picture boundaries are considered.

      isRightAvail      = (xPos + pcv.maxCUSize  < pcv.lumaWidth );
      isBelowAvail      = (yPos + pcv.maxCUSize < pcv.lumaHeight);
      isAboveRightAvail = ((yPos > 0) && (isRightAvail));

      for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        const ComponentID compID = ComponentID(compIdx);
        const CompArea& compArea = area.block( compID );

        int  srcStride  = srcYuv.get(compID).stride;
        Pel* srcBlk     = srcYuv.get(compID).bufAt( compArea );

        int  orgStride  = orgYuv.get(compID).stride;
        Pel* orgBlk     = orgYuv.get(compID).bufAt( compArea );

        getBlkStats(compID, cs.sps->bitDepths[toChannelType(compID)], blkStats[ctuRsAddr][compID]
                  , srcBlk, orgBlk, srcStride, orgStride, compArea.width, compArea.height
                  , isLeftAvail,  isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail );
      }
      ctuRsAddr++;
    }
  }
}

void EncSampleAdaptiveOffset::decideCtuParams( CodingStructure& cs, const std::vector<SAOStatData**>& saoStatistics, const bool saoEnabled[ MAX_NUM_COMP ], const bool allBlksDisabled, const UnitArea& ctuArea, const int ctuRsAddr, SAOBlkParam* reconParams, SAOBlkParam* codedParams )
{
  const PreCalcValues& pcv = *cs.pcv;
  const Slice& slice       = *cs.slice;
  const int  ctuPosX       = ctuRsAddr % pcv.widthInCtus;
  const int  ctuPosY       = ctuRsAddr / pcv.widthInCtus;

  // reset CABAC estimator
  if( m_EncCfg->m_ensureWppBitEqual
      && m_EncCfg->m_numThreads < 1
      && ctuPosX == 0
      && ctuPosY > 0 )
  {
    m_CABACEstimator->initCtxModels( slice );
  }

  // check disabled
  if( allBlksDisabled )
  {
    codedParams[ ctuRsAddr ].reset();
    return;
  }

  // get merge list
  SAOBlkParam* mergeList[ NUM_SAO_MERGE_TYPES ] = { NULL };
  getMergeList( cs, ctuRsAddr, reconParams, mergeList );

  const TempCtx ctxStart( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  TempCtx       ctxBest ( m_CtxCache );

  SAOBlkParam modeParam;
  double minCost  = MAX_DOUBLE;
  double modeCost = MAX_DOUBLE;
  for( int mode = 1; mode < NUM_SAO_MODES; mode++ )
  {
    if( mode > 1 )
    {
      m_CABACEstimator->getCtx() = SAOCtx( ctxStart );
    }
    switch( mode )
    {
    case SAO_MODE_NEW:
      {
        deriveModeNewRDO( cs.sps->bitDepths, ctuRsAddr, mergeList, saoEnabled, saoStatistics, modeParam, modeCost );
      }
      break;
    case SAO_MODE_MERGE:
      {
        deriveModeMergeRDO( cs.sps->bitDepths, ctuRsAddr, mergeList, saoEnabled, saoStatistics, modeParam, modeCost );
      }
      break;
    default:
      {
        THROW( "Not a supported SAO mode." );
      }
    }

    if( modeCost < minCost )
    {
      minCost                  = modeCost;
      codedParams[ ctuRsAddr ] = modeParam;
      ctxBest                  = SAOCtx( m_CABACEstimator->getCtx() );
    }
  }

  // apply reconstructed offsets
  m_CABACEstimator->getCtx() = SAOCtx( ctxBest );
  reconParams[ ctuRsAddr ] = codedParams[ ctuRsAddr ];

  reconstructBlkSAOParam( reconParams[ ctuRsAddr ], mergeList );

  Picture& pic = *cs.picture;
  offsetCTU( ctuArea, pic.getSaoBuf(), cs.getRecoBuf(), reconParams[ ctuRsAddr ], cs );
}

int64_t EncSampleAdaptiveOffset::getDistortion(const int channelBitDepth, int typeIdc, int typeAuxInfo, int* invQuantOffset, SAOStatData& statData)
{
  int64_t dist        = 0;
  int shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth);

  switch(typeIdc)
  {
  case SAO_TYPE_EO_0:
  case SAO_TYPE_EO_90:
  case SAO_TYPE_EO_135:
  case SAO_TYPE_EO_45:
    {
      for (int offsetIdx=0; offsetIdx<NUM_SAO_EO_CLASSES; offsetIdx++)
      {
        dist += estSaoDist( statData.count[offsetIdx], invQuantOffset[offsetIdx], statData.diff[offsetIdx], shift);
      }
    }
    break;
  case SAO_TYPE_BO:
    {
      for (int offsetIdx=typeAuxInfo; offsetIdx<typeAuxInfo+4; offsetIdx++)
      {
        int bandIdx = offsetIdx % NUM_SAO_BO_CLASSES ;
        dist += estSaoDist( statData.count[bandIdx], invQuantOffset[bandIdx], statData.diff[bandIdx], shift);
      }
    }
    break;
  default:
    {
      THROW("Not a supported type");
    }
  }

  return dist;
}

inline int64_t EncSampleAdaptiveOffset::estSaoDist(int64_t count, int64_t offset, int64_t diffSum, int shift)
{
  return (( count*offset*offset-diffSum*offset*2 ) >> shift);
}


inline int EncSampleAdaptiveOffset::estIterOffset(int typeIdx, double lambda, int offsetInput, int64_t count, int64_t diffSum, int shift, int bitIncrease, int64_t& bestDist, double& bestCost, int offsetTh )
{
  int iterOffset, tempOffset;
  int64_t tempDist, tempRate;
  double tempCost, tempMinCost;
  int offsetOutput = 0;
  iterOffset = offsetInput;
  // Assuming sending quantized value 0 results in zero offset and sending the value zero needs 1 bit. entropy coder can be used to measure the exact rate here.
  tempMinCost = lambda;
  while (iterOffset != 0)
  {
    // Calculate the bits required for signaling the offset
    tempRate = (typeIdx == SAO_TYPE_BO) ? (abs((int)iterOffset)+2) : (abs((int)iterOffset)+1);
    if (abs((int)iterOffset)==offsetTh) //inclusive
    {
      tempRate --;
    }
    // Do the dequantization before distortion calculation
    tempOffset  = iterOffset * (1<< bitIncrease);
    tempDist    = estSaoDist( count, tempOffset, diffSum, shift);
    tempCost    = ((double)tempDist + lambda * (double) tempRate);
    if(tempCost < tempMinCost)
    {
      tempMinCost = tempCost;
      offsetOutput = iterOffset;
      bestDist = tempDist;
      bestCost = tempCost;
    }
    iterOffset = (iterOffset > 0) ? (iterOffset-1):(iterOffset+1);
  }
  return offsetOutput;
}

void EncSampleAdaptiveOffset::deriveOffsets(ComponentID compIdx, const int channelBitDepth, int typeIdc, SAOStatData& statData, int* quantOffsets, int& typeAuxInfo)
{
  int bitDepth = channelBitDepth;
  int shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(bitDepth);
  int offsetTh = SampleAdaptiveOffset::getMaxOffsetQVal(channelBitDepth);  //inclusive

  ::memset(quantOffsets, 0, sizeof(int)*MAX_NUM_SAO_CLASSES);

  //derive initial offsets
  int numClasses = (typeIdc == SAO_TYPE_BO)?((int)NUM_SAO_BO_CLASSES):((int)NUM_SAO_EO_CLASSES);
  for(int classIdx=0; classIdx< numClasses; classIdx++)
  {
    if( (typeIdc != SAO_TYPE_BO) && (classIdx==SAO_CLASS_EO_PLAIN)  )
    {
      continue; //offset will be zero
    }

    if(statData.count[classIdx] == 0)
    {
      continue; //offset will be zero
    }
#if (  DISTORTION_PRECISION_ADJUSTMENT(x)  == 0 )
    quantOffsets[classIdx] =
       (int) xRoundIbdi(bitDepth, (double)(statData.diff[classIdx] ) / (double)(statData.count[classIdx] << m_offsetStepLog2[compIdx]));
     quantOffsets[classIdx] = Clip3(-offsetTh, offsetTh, quantOffsets[classIdx]);
#else
      quantOffsets[classIdx] =
        (int) xRoundIbdi(bitDepth, (double)(statData.diff[classIdx] << DISTORTION_PRECISION_ADJUSTMENT(bitDepth))
                                     / (double)(statData.count[classIdx] << m_offsetStepLog2[compIdx]));
      quantOffsets[classIdx] = Clip3(-offsetTh, offsetTh, quantOffsets[classIdx]);
#endif
  }

  // adjust offsets
  switch(typeIdc)
  {
  case SAO_TYPE_EO_0:
  case SAO_TYPE_EO_90:
  case SAO_TYPE_EO_135:
  case SAO_TYPE_EO_45:
    {
      int64_t classDist;
      double classCost;
      for(int classIdx=0; classIdx<NUM_SAO_EO_CLASSES; classIdx++)
      {
        if(classIdx==SAO_CLASS_EO_FULL_VALLEY && quantOffsets[classIdx] < 0)
        {
          quantOffsets[classIdx] =0;
        }
        if(classIdx==SAO_CLASS_EO_HALF_VALLEY && quantOffsets[classIdx] < 0)
        {
          quantOffsets[classIdx] =0;
        }
        if(classIdx==SAO_CLASS_EO_HALF_PEAK   && quantOffsets[classIdx] > 0)
        {
          quantOffsets[classIdx] =0;
        }
        if(classIdx==SAO_CLASS_EO_FULL_PEAK   && quantOffsets[classIdx] > 0)
        {
          quantOffsets[classIdx] =0;
        }

        if( quantOffsets[classIdx] != 0 ) //iterative adjustment only when derived offset is not zero
        {
          quantOffsets[classIdx] = estIterOffset( typeIdc, m_lambda[compIdx], quantOffsets[classIdx], statData.count[classIdx], statData.diff[classIdx], shift, m_offsetStepLog2[compIdx], classDist , classCost , offsetTh );
        }
      }

      typeAuxInfo =0;
    }
    break;
  case SAO_TYPE_BO:
    {
      int64_t  distBOClasses[NUM_SAO_BO_CLASSES];
      double costBOClasses[NUM_SAO_BO_CLASSES];
      ::memset(distBOClasses, 0, sizeof(int64_t)*NUM_SAO_BO_CLASSES);
      for(int classIdx=0; classIdx< NUM_SAO_BO_CLASSES; classIdx++)
      {
        costBOClasses[classIdx]= m_lambda[compIdx];
        if( quantOffsets[classIdx] != 0 ) //iterative adjustment only when derived offset is not zero
        {
          quantOffsets[classIdx] = estIterOffset( typeIdc, m_lambda[compIdx], quantOffsets[classIdx], statData.count[classIdx], statData.diff[classIdx], shift, m_offsetStepLog2[compIdx], distBOClasses[classIdx], costBOClasses[classIdx], offsetTh );
        }
      }

      //decide the starting band index
      double minCost = MAX_DOUBLE, cost;
      for(int band=0; band< NUM_SAO_BO_CLASSES- 4+ 1; band++)
      {
        cost  = costBOClasses[band  ];
        cost += costBOClasses[band+1];
        cost += costBOClasses[band+2];
        cost += costBOClasses[band+3];

        if(cost < minCost)
        {
          minCost = cost;
          typeAuxInfo = band;
        }
      }
      //clear those unused classes
      int clearQuantOffset[NUM_SAO_BO_CLASSES];
      ::memset(clearQuantOffset, 0, sizeof(int)*NUM_SAO_BO_CLASSES);
      for(int i=0; i< 4; i++)
      {
        int band = (typeAuxInfo+i)%NUM_SAO_BO_CLASSES;
        clearQuantOffset[band] = quantOffsets[band];
      }
      ::memcpy(quantOffsets, clearQuantOffset, sizeof(int)*NUM_SAO_BO_CLASSES);
    }
    break;
  default:
    {
      THROW("Not a supported type");
    }
  }
}

void EncSampleAdaptiveOffset::deriveModeNewRDO(const BitDepths &bitDepths, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], const bool* sliceEnabled, const std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, double& modeNormCost )
{
  double minCost, cost;
  uint64_t previousFracBits;
  const int numberOfComponents = m_numberOfComponents;

  int64_t dist[MAX_NUM_COMP], modeDist[MAX_NUM_COMP];
  SAOOffset testOffset[MAX_NUM_COMP];
  int invQuantOffset[MAX_NUM_SAO_CLASSES];
  for(int comp=0; comp < MAX_NUM_COMP; comp++)
  {
    modeDist[comp] = 0;
  }

  //pre-encode merge flags
  modeParam[COMP_Y].modeIdc = SAO_MODE_OFF;
  const TempCtx ctxStartBlk   ( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  m_CABACEstimator->sao_block_pars( modeParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), true );
  const TempCtx ctxStartLuma  ( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  TempCtx       ctxBestLuma   ( m_CtxCache );

    //------ luma --------//
  {
    const ComponentID compIdx = COMP_Y;
    //"off" case as initial cost
    modeParam[compIdx].modeIdc = SAO_MODE_OFF;
    m_CABACEstimator->resetBits();
    m_CABACEstimator->sao_offset_pars( modeParam[compIdx], compIdx, sliceEnabled[compIdx], bitDepths[CH_L] );
    modeDist[compIdx] = 0;
    minCost           = m_lambda[compIdx] * (FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits());
    ctxBestLuma = SAOCtx( m_CABACEstimator->getCtx() );
    if(sliceEnabled[compIdx])
    {
      for(int typeIdc=0; typeIdc< NUM_SAO_NEW_TYPES; typeIdc++)
      {
        testOffset[compIdx].modeIdc = SAO_MODE_NEW;
        testOffset[compIdx].typeIdc = typeIdc;

        //derive coded offset
        deriveOffsets(compIdx, bitDepths[CH_L], typeIdc, blkStats[ctuRsAddr][compIdx][typeIdc], testOffset[compIdx].offset, testOffset[compIdx].typeAuxInfo);

        //inversed quantized offsets
        invertQuantOffsets(compIdx, typeIdc, testOffset[compIdx].typeAuxInfo, invQuantOffset, testOffset[compIdx].offset);

        //get distortion
        dist[compIdx] = getDistortion(bitDepths[CH_L], testOffset[compIdx].typeIdc, testOffset[compIdx].typeAuxInfo, invQuantOffset, blkStats[ctuRsAddr][compIdx][typeIdc]);

        //get rate
        m_CABACEstimator->getCtx() = SAOCtx( ctxStartLuma );
        m_CABACEstimator->resetBits();
        m_CABACEstimator->sao_offset_pars( testOffset[compIdx], compIdx, sliceEnabled[compIdx], bitDepths[CH_L] );
        double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
        cost = (double)dist[compIdx] + m_lambda[compIdx]*rate;
        if(cost < minCost)
        {
          minCost = cost;
          modeDist[compIdx] = dist[compIdx];
          modeParam[compIdx]= testOffset[compIdx];
          ctxBestLuma = SAOCtx( m_CABACEstimator->getCtx() );
        }
      }
    }
    m_CABACEstimator->getCtx() = SAOCtx( ctxBestLuma );
  }

  //------ chroma --------//
//"off" case as initial cost
  cost = 0;
  previousFracBits = 0;
  m_CABACEstimator->resetBits();
  for(uint32_t componentIndex = COMP_Cb; componentIndex < numberOfComponents; componentIndex++)
  {
    const ComponentID component = ComponentID(componentIndex);

    modeParam[component].modeIdc = SAO_MODE_OFF;
    modeDist [component]         = 0;
    m_CABACEstimator->sao_offset_pars( modeParam[component], component, sliceEnabled[component], bitDepths[CH_C] );
    const uint64_t currentFracBits = m_CABACEstimator->getEstFracBits();
    cost += m_lambda[component] * FRAC_BITS_SCALE * (currentFracBits - previousFracBits);
    previousFracBits = currentFracBits;
  }

  minCost = cost;

  //doesn't need to store cabac status here since the whole CTU parameters will be re-encoded at the end of this function

  for(int typeIdc=0; typeIdc< NUM_SAO_NEW_TYPES; typeIdc++)
  {
    m_CABACEstimator->getCtx() = SAOCtx( ctxBestLuma );
    m_CABACEstimator->resetBits();
    previousFracBits = 0;
    cost = 0;

    for(uint32_t componentIndex = COMP_Cb; componentIndex < numberOfComponents; componentIndex++)
    {
      const ComponentID component = ComponentID(componentIndex);
      if(!sliceEnabled[component])
      {
        testOffset[component].modeIdc = SAO_MODE_OFF;
        dist[component]= 0;
        continue;
      }
      testOffset[component].modeIdc = SAO_MODE_NEW;
      testOffset[component].typeIdc = typeIdc;

      //derive offset & get distortion
      deriveOffsets(component, bitDepths[CH_C], typeIdc, blkStats[ctuRsAddr][component][typeIdc], testOffset[component].offset, testOffset[component].typeAuxInfo);
      invertQuantOffsets(component, typeIdc, testOffset[component].typeAuxInfo, invQuantOffset, testOffset[component].offset);
      dist[component] = getDistortion(bitDepths[CH_C], typeIdc, testOffset[component].typeAuxInfo, invQuantOffset, blkStats[ctuRsAddr][component][typeIdc]);
      m_CABACEstimator->sao_offset_pars( testOffset[component], component, sliceEnabled[component], bitDepths[CH_C] );
      const uint64_t currentFracBits = m_CABACEstimator->getEstFracBits();
      cost += dist[component] + (m_lambda[component] * FRAC_BITS_SCALE * (currentFracBits - previousFracBits));
      previousFracBits = currentFracBits;
    }

    if(cost < minCost)
    {
      minCost = cost;
      for(uint32_t componentIndex = COMP_Cb; componentIndex < numberOfComponents; componentIndex++)
      {
        modeDist[componentIndex]  = dist[componentIndex];
        modeParam[componentIndex] = testOffset[componentIndex];
      }
    }

  } // SAO_TYPE loop

  //----- re-gen rate & normalized cost----//
  modeNormCost = 0;
  for(uint32_t componentIndex = COMP_Y; componentIndex < numberOfComponents; componentIndex++)
  {
    modeNormCost += (double)modeDist[componentIndex] / m_lambda[componentIndex];
  }

  m_CABACEstimator->getCtx() = SAOCtx( ctxStartBlk );
  m_CABACEstimator->resetBits();
  m_CABACEstimator->sao_block_pars( modeParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), false );
  modeNormCost += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
}

void EncSampleAdaptiveOffset::deriveModeMergeRDO(const BitDepths &bitDepths, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], const bool* sliceEnabled, const std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, double& modeNormCost )
{
  modeNormCost = MAX_DOUBLE;

  double cost;
  SAOBlkParam testBlkParam;
  const int numberOfComponents = m_numberOfComponents;

  const TempCtx ctxStart  ( m_CtxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  TempCtx       ctxBest   ( m_CtxCache );

  for(int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)
  {
    if(mergeList[mergeType] == NULL)
    {
      continue;
    }

    testBlkParam = *(mergeList[mergeType]);
    //normalized distortion
    double normDist=0;
    for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      testBlkParam[compIdx].modeIdc = SAO_MODE_MERGE;
      testBlkParam[compIdx].typeIdc = mergeType;

      SAOOffset& mergedOffsetParam = (*(mergeList[mergeType]))[compIdx];

      if( mergedOffsetParam.modeIdc != SAO_MODE_OFF)
      {
        //offsets have been reconstructed. Don't call inversed quantization function.
        normDist += (((double)getDistortion(bitDepths[toChannelType(ComponentID(compIdx))], mergedOffsetParam.typeIdc, mergedOffsetParam.typeAuxInfo, mergedOffsetParam.offset, blkStats[ctuRsAddr][compIdx][mergedOffsetParam.typeIdc]))
                       /m_lambda[compIdx] );
      }
    }

    //rate
    m_CABACEstimator->getCtx() = SAOCtx( ctxStart );
    m_CABACEstimator->resetBits();
    m_CABACEstimator->sao_block_pars( testBlkParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), false );
    double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
    cost = normDist+rate;

    if(cost < modeNormCost)
    {
      modeNormCost = cost;
      modeParam    = testBlkParam;
      ctxBest      = SAOCtx( m_CABACEstimator->getCtx() );
    }
  }
  if( modeNormCost < MAX_DOUBLE )
  {
    m_CABACEstimator->getCtx() = SAOCtx( ctxBest );
  }
}

void EncSampleAdaptiveOffset::getBlkStats(const ComponentID compIdx, const int channelBitDepth, SAOStatData* statsDataTypes
                        , Pel* srcBlk, Pel* orgBlk, int srcStride, int orgStride, int width, int height
                        , bool isLeftAvail,  bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail )
{
  int x,y, startX, startY, endX, endY, edgeType, firstLineStartX, firstLineEndX;
  int8_t signLeft, signRight, signDown;
  int64_t *diff, *count;
  Pel* srcLine, *orgLine;
  const int skipLinesR = compIdx == COMP_Y ? 5 : 3;
  const int skipLinesB = compIdx == COMP_Y ? 4 : 2;

  for(int typeIdx=0; typeIdx< NUM_SAO_NEW_TYPES; typeIdx++)
  {
    SAOStatData& statsData= statsDataTypes[typeIdx];
    statsData.reset();

    srcLine = srcBlk;
    orgLine = orgBlk;
    diff    = statsData.diff;
    count   = statsData.count;
    switch(typeIdx)
    {
    case SAO_TYPE_EO_0:
      {
        diff +=2;
        count+=2;
        endY   =  isBelowAvail ? (height - skipLinesB) : height;
        startX = (isLeftAvail  ? 0 : 1);
        endX   = (isRightAvail ? (width - skipLinesR) : (width - 1));

        for (y=0; y<endY; y++)
        {
          signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX-1]);
          for (x=startX; x<endX; x++)
          {
            signRight =  (int8_t)sgn(srcLine[x] - srcLine[x+1]);
            edgeType  =  signRight + signLeft;
            signLeft  = -signRight;

            diff [edgeType] += (orgLine[x] - srcLine[x]);
            count[edgeType] ++;
          }
          srcLine  += srcStride;
          orgLine  += orgStride;
        }
      }
      break;
    case SAO_TYPE_EO_90:
      {
        diff +=2;
        count+=2;
        int8_t *signUpLine = &m_signLineBuf1[0];

        startX = 0;
        startY = isAboveAvail ? 0 : 1;
        endX   = (isRightAvail ? (width - skipLinesR) : width);
        endY   = isBelowAvail ? (height - skipLinesB) : (height - 1);
        if (!isAboveAvail)
        {
          srcLine += srcStride;
          orgLine += orgStride;
        }

        Pel* srcLineAbove = srcLine - srcStride;
        for (x=startX; x<endX; x++)
        {
          signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
        }

        Pel* srcLineBelow;
        for (y=startY; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x=startX; x<endX; x++)
          {
            signDown  = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
            edgeType  = signDown + signUpLine[x];
            signUpLine[x]= -signDown;

            diff [edgeType] += (orgLine[x] - srcLine[x]);
            count[edgeType] ++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
        }
      }
      break;
    case SAO_TYPE_EO_135:
      {
        diff +=2;
        count+=2;
        int8_t *signUpLine, *signDownLine, *signTmpLine;

        signUpLine  = &m_signLineBuf1[0];
        signDownLine= &m_signLineBuf2[0];

        startX = isLeftAvail  ? 0 : 1;

        endX   = isRightAvail ? (width - skipLinesR): (width - 1);
        endY   = isBelowAvail ? (height - skipLinesB) : (height - 1);

        //prepare 2nd line's upper sign
        Pel* srcLineBelow = srcLine + srcStride;
        for (x=startX; x<endX+1; x++)
        {
          signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x-1]);
        }

        //1st line
        Pel* srcLineAbove = srcLine - srcStride;
        firstLineStartX = isAboveLeftAvail ? 0    : 1;
        firstLineEndX   = isAboveAvail     ? endX : 1;
        for(x=firstLineStartX; x<firstLineEndX; x++)
        {
          edgeType = sgn(srcLine[x] - srcLineAbove[x-1]) - signUpLine[x+1];
          diff [edgeType] += (orgLine[x] - srcLine[x]);
          count[edgeType] ++;
        }
        srcLine  += srcStride;
        orgLine  += orgStride;


        //middle lines
        for (y=1; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x=startX; x<endX; x++)
          {
            signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x+1]);
            edgeType = signDown + signUpLine[x];
            diff [edgeType] += (orgLine[x] - srcLine[x]);
            count[edgeType] ++;

            signDownLine[x+1] = -signDown;
          }
          signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX-1]);

          signTmpLine  = signUpLine;
          signUpLine   = signDownLine;
          signDownLine = signTmpLine;

          srcLine += srcStride;
          orgLine += orgStride;
        }
      }
      break;
    case SAO_TYPE_EO_45:
      {
        diff +=2;
        count+=2;
        int8_t *signUpLine = &m_signLineBuf1[1];

        startX = isLeftAvail  ? 0 : 1;
        endX   = isRightAvail ? (width - skipLinesR) : (width - 1);
        endY   = isBelowAvail ? (height - skipLinesB) : (height - 1);

        //prepare 2nd line upper sign
        Pel* srcLineBelow = srcLine + srcStride;
        for (x=startX-1; x<endX; x++)
        {
          signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x+1]);
        }


        //first line
        Pel* srcLineAbove = srcLine - srcStride;
        firstLineStartX = isAboveAvail ? startX : endX;
        firstLineEndX   = (!isRightAvail && isAboveRightAvail) ? width : endX;
        for(x=firstLineStartX; x<firstLineEndX; x++)
        {
          edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) - signUpLine[x-1];
          diff [edgeType] += (orgLine[x] - srcLine[x]);
          count[edgeType] ++;
        }

        srcLine += srcStride;
        orgLine += orgStride;

        //middle lines
        for (y=1; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for(x=startX; x<endX; x++)
          {
            signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x-1]);
            edgeType = signDown + signUpLine[x];

            diff [edgeType] += (orgLine[x] - srcLine[x]);
            count[edgeType] ++;

            signUpLine[x-1] = -signDown;
          }
          signUpLine[endX-1] = (int8_t)sgn(srcLineBelow[endX-1] - srcLine[endX]);
          srcLine  += srcStride;
          orgLine  += orgStride;
        }
      }
      break;
    case SAO_TYPE_BO:
      {
        startX = 0;
        endX   = isRightAvail ? (width - skipLinesR) : width;
        endY   = isBelowAvail ? (height- skipLinesB) : height;
        int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
        for (y=0; y< endY; y++)
        {
          for (x=startX; x< endX; x++)
          {

            int bandIdx= srcLine[x] >> shiftBits;
            diff [bandIdx] += (orgLine[x] - srcLine[x]);
            count[bandIdx] ++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
        }
      }
      break;
    default:
      {
        THROW("Not a supported SAO type");
      }
    }
  }
}

void EncSampleAdaptiveOffset::deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position& pos, bool& isLeftAvail, bool& isAboveAvail, bool& isAboveLeftAvail) const
{
  const bool isLoopFiltAcrossSlicePPS = cs.pps->loopFilterAcrossSlicesEnabled;
  const bool isLoopFiltAcrossTilePPS = cs.pps->loopFilterAcrossTilesEnabled;

  const int width = cs.pcv->maxCUSize;
  const int height = cs.pcv->maxCUSize;
  const CodingUnit* cuCurr = cs.getCU(pos, CH_L, TREE_D);
  const CodingUnit* cuLeft = cs.getCU(pos.offset(-width, 0), CH_L, TREE_D);
  const CodingUnit* cuAbove = cs.getCU(pos.offset(0, -height), CH_L, TREE_D);
  const CodingUnit* cuAboveLeft = cs.getCU(pos.offset(-width, -height), CH_L, TREE_D);

  if (!isLoopFiltAcrossSlicePPS)
  {
    isLeftAvail      = (cuLeft == NULL)      ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail     = (cuAbove == NULL)     ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isAboveLeftAvail = (cuAboveLeft == NULL) ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
  }
  else
  {
    isLeftAvail      = (cuLeft != NULL);
    isAboveAvail     = (cuAbove != NULL);
    isAboveLeftAvail = (cuAboveLeft != NULL);
  }

  if (!isLoopFiltAcrossTilePPS)
  {
    isLeftAvail      = (!isLeftAvail)      ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail     = (!isAboveAvail)     ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isAboveLeftAvail = (!isAboveLeftAvail) ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
  }


  SubPic curSubPic = cs.pps->getSubPicFromCU(*cuCurr);
  if (!curSubPic.loopFilterAcrossSubPicEnabled )
  {
    isLeftAvail      = (!isLeftAvail)      ? false : CU::isSameSubPic(*cuCurr, *cuLeft);
    isAboveAvail     = (!isAboveAvail)     ? false : CU::isSameSubPic(*cuCurr, *cuAbove);
    isAboveLeftAvail = (!isAboveLeftAvail) ? false : CU::isSameSubPic(*cuCurr, *cuAboveLeft);
  }

}

} // namespace vvenc

//! \}


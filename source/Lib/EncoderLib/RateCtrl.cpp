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

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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


/** \file     RateCtrl.cpp
    \brief    Rate control manager class
*/
#ifdef VVENC_ENABLE_THIRDPARTY_JSON
#include "nlohmann/json.hpp"
#endif

#include "vvenc/version.h"
#include "RateCtrl.h"
#include "CommonLib/Picture.h"

#include <cmath>

namespace vvenc {

static const int RC_MAX_PIC_LIST_SIZE = 64;

//sequence level
EncRCSeq::EncRCSeq()
{
  twoPass             = false;
  isLookAhead         = false;
  framesCoded         = 0;
  targetRate          = 0;
  frameRate           = 0;
  gopSize             = 0;
  intraPeriod         = 0;
  bitsUsed            = 0;
  bitsUsedIn1stPass   = 0;
  estimatedBitUsage   = 0;
  std::memset (qpCorrection, 0, sizeof (qpCorrection));
  std::memset (actualBitCnt, 0, sizeof (actualBitCnt));
  std::memset (currFrameCnt, 0, sizeof (currFrameCnt));
  std::memset (targetBitCnt, 0, sizeof (targetBitCnt));
  lastIntraQP         = 0;
  bitDepth            = 0;
}

EncRCSeq::~EncRCSeq()
{
  destroy();
}

void EncRCSeq::create( bool twoPassRC, bool lookAhead, int targetBitrate, int frRate, int intraPer, int GOPSize, int bitDpth, std::list<TRCPassStats> &firstPassStats )
{
  destroy();
  twoPass             = twoPassRC;
  isLookAhead         = lookAhead;
  targetRate          = targetBitrate;
  frameRate           = frRate;
  intraPeriod         = intraPer;
  gopSize             = GOPSize;
  firstPassData       = firstPassStats;
  bitDepth            = bitDpth;

  int bitdepthLumaScale = 2 * ( bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( bitDepth ) );
  minEstLambda = 0.1;
  maxEstLambda = 10000.0 * pow( 2.0, bitdepthLumaScale );

  framesCoded = 0;
  bitsUsed = 0;
  bitsUsedIn1stPass = 0;
  estimatedBitUsage = 0;
  std::memset (qpCorrection, 0, sizeof (qpCorrection));
  std::memset (actualBitCnt, 0, sizeof (actualBitCnt));
  std::memset (currFrameCnt, 0, sizeof (currFrameCnt));
  std::memset (targetBitCnt, 0, sizeof (targetBitCnt));
}

void EncRCSeq::destroy()
{
  return;
}

void EncRCSeq::updateAfterPic ( int bits, int tgtBits )
{
  framesCoded++;

  if (isLookAhead)
  {
    const uint64_t* const tlBits  = actualBitCnt; // recently updated in EncRCPic::updateAfterPicture()
    const unsigned* const tlCount = currFrameCnt;

    estimatedBitUsage = int64_t (0.5 + ((double) targetRate * framesCoded) / (double) frameRate);

    if (framesCoded < intraPeriod) // apply crossfade between I-period estimate and simple bit-counting
    {
      const int gopsInIp = intraPeriod / gopSize;
      uint64_t totalBitsSecondPass = (tlBits[0] + (tlCount[0] >> 1)) / std::max (1u, tlCount[0]) +
                    ((gopsInIp - 1) * tlBits[1] + (tlCount[1] >> 1)) / std::max (1u, tlCount[1]);
      for (int frameLevel = 2; frameLevel <= 7; frameLevel++)
      {
        totalBitsSecondPass += ((gopsInIp << (frameLevel - 2)) * tlBits[frameLevel] + (tlCount[frameLevel] >> 1)) / std::max (1u, tlCount[frameLevel]);
      }
      bitsUsed = int64_t (0.5 + ((double) totalBitsSecondPass * framesCoded) / (double) intraPeriod);
      totalBitsSecondPass = tlBits[0] + tlBits[1] + tlBits[2] + tlBits[3] + tlBits[4] + tlBits[5] + tlBits[6] + tlBits[7];
      bitsUsed = (bitsUsed * (intraPeriod - framesCoded) + totalBitsSecondPass * framesCoded + (intraPeriod >> 1)) / intraPeriod;
    }
    else
    {
      bitsUsed = tlBits[0] + tlBits[1] + tlBits[2] + tlBits[3] + tlBits[4] + tlBits[5] + tlBits[6] + tlBits[7];
    }
  }
  else
  {
    estimatedBitUsage += tgtBits;
    bitsUsed += bits;
  }
}

void EncRCSeq::getTargetBitsFromFirstPass (const int poc, int &targetBits, double &frameVsGopRatio, bool &isNewScene, bool &refreshParameters)
{
  TRCPassStats* stats = nullptr;
  for( auto& it : firstPassData )
  {
    if( poc == it.poc )
    {
      stats = &it;
    }
  }
  if( ! stats )
  {
    THROW( "miss entry for poc " << poc << " in first pass rate control statistics" );
  }
  targetBits        = stats->targetBits;
  frameVsGopRatio   = stats->frameInGopRatio;
  isNewScene        = stats->isNewScene;
  refreshParameters = stats->refreshParameters;
}

//picture level
EncRCPic::EncRCPic()
{
  encRCSeq            = NULL;
  frameLevel          = 0;
  targetBits          = 0;
  tmpTargetBits       = 0;
  picQPOffsetQPA      = 0;
  picLambdaOffsetQPA  = 0.0;
  picActualBits       = 0;
  picQP               = 0;
  isNewScene          = false;
  refreshParams       = false;
}

EncRCPic::~EncRCPic()
{
  destroy();
}

int EncRCPic::xEstPicTargetBits( EncRCSeq* encRcSeq, int frameLevel )
{
  int targetBits    = 0;

  // bit allocation for 2-pass RC
  if ( encRcSeq->twoPass || encRcSeq->isLookAhead )
  {
    double frameVsGopRatio = 1.0;

    encRcSeq->getTargetBitsFromFirstPass (poc, tmpTargetBits, frameVsGopRatio, isNewScene, refreshParams);

    // calculate the difference of under/overspent bits and adjust the current target bits based on the GOP and frame ratio for every frame
    targetBits = int (0.5 + tmpTargetBits + (encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed) * 0.5 * frameVsGopRatio);
  }

  return targetBits;
}

void EncRCPic::addToPictureList( std::list<EncRCPic*>& listPreviousPictures )
{
  if ( listPreviousPictures.size() > RC_MAX_PIC_LIST_SIZE )
  {
    EncRCPic* p = listPreviousPictures.front();
    listPreviousPictures.pop_front();
    p->destroy();
    delete p;
  }

  listPreviousPictures.push_back( this );
}

void EncRCPic::create( EncRCSeq* encRcSeq, int frameLvl, int framePoc )
{
  destroy();
  encRCSeq   = encRcSeq;
  poc        = framePoc;

  int tgtBits    = xEstPicTargetBits( encRcSeq, frameLvl );

  if ( encRcSeq->twoPass || encRcSeq->isLookAhead )
  {
    tgtBits = std::max (1, tgtBits);
  }

  frameLevel       = frameLvl;
  targetBits       = tgtBits;

  picActualBits       = 0;
  picQP               = 0;
  picQPOffsetQPA      = 0;
  picLambdaOffsetQPA  = 0.0;
}

void EncRCPic::destroy()
{
  encRCSeq = NULL;
}

void EncRCPic::updateCtuMSE( const unsigned int ctuAddress, const double distortion )
{
  //THROW( "Not supported in 2-pass rate control" );
}

void EncRCPic::clipTargetQP (std::list<EncRCPic*>& listPreviousPictures, int &qp)
{
  int lastCurrTLQP = -1;
  int lastPrevTLQP = -1;
  std::list<EncRCPic*>::iterator it;

  for (it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++)
  {
    if ((*it)->frameLevel == frameLevel && (*it)->picQP >= 0)  // current temporal level
    {
      lastCurrTLQP = (*it)->picQP;
    }
    if ((*it)->frameLevel == frameLevel - 1 && (*it)->picQP >= 0) // last temporal level
    {
      lastPrevTLQP = (*it)->picQP;
    }
  }

  qp = Clip3 (0, MAX_QP, qp);

  if (lastCurrTLQP >= 0) // limit QP changes among prev. frames from same temporal level
  {
    const int clipRange = (refreshParams ? 5 + encRCSeq->intraPeriod / encRCSeq->gopSize : std::max(3, 6 - (frameLevel >> 1)));

    qp = Clip3 (lastCurrTLQP - clipRange, std::min (MAX_QP, lastCurrTLQP + clipRange), qp);
  }
  if (lastPrevTLQP >= 0) // prevent QP from being lower than QPs at lower temporal level
  {
    qp = Clip3 (lastPrevTLQP + 1, MAX_QP, qp);
  }
  if (encRCSeq->lastIntraQP >= 0 && (frameLevel == 1 || frameLevel == 2))
  {
    qp = Clip3 (encRCSeq->lastIntraQP, MAX_QP, qp);
  }
}

void EncRCPic::updateAfterPicture( int actualTotalBits, double averageQP, bool isIRAP)
{
  picActualBits       = actualTotalBits;
  picQP               = int (averageQP + 0.5);

  if ((frameLevel <= 7) && (picActualBits > 0) && (targetBits > 0)) // update qpCorrection for EncGOP::picInitRateControl()
  {
    const bool refreshed = (encRCSeq->actualBitCnt[frameLevel] == 0) && (encRCSeq->targetBitCnt[frameLevel] == 0);

    encRCSeq->actualBitCnt[frameLevel] += (uint64_t) picActualBits;
    encRCSeq->targetBitCnt[frameLevel] += (uint64_t) targetBits;

    if (encRCSeq->isLookAhead) encRCSeq->currFrameCnt[frameLevel]++;

    encRCSeq->qpCorrection[frameLevel] = (refreshed ? 1.0 : 6.0) * log ((double) encRCSeq->actualBitCnt[frameLevel] / (double) encRCSeq->targetBitCnt[frameLevel]) / log (2.0);
    encRCSeq->qpCorrection[frameLevel] = Clip3 (-12.0, 12.0, encRCSeq->qpCorrection[frameLevel]);
  }
}

RateCtrl::RateCtrl(MsgLog& logger)
: msg ( logger )
{
  m_pcEncCfg    = nullptr;
  encRCSeq      = NULL;
  encRCPic      = NULL;
  flushPOC      = -1;
  rcPass        = 0;
  rcIsFinalPass = true;
#ifdef VVENC_ENABLE_THIRDPARTY_JSON
  m_pqpaStatsWritten = 0;
#endif
}

RateCtrl::~RateCtrl()
{
  destroy();
}

void RateCtrl::destroy()
{
  if ( encRCSeq != NULL )
  {
    delete encRCSeq;
    encRCSeq = NULL;
  }
  while ( m_listRCPictures.size() > 0 )
  {
    EncRCPic* p = m_listRCPictures.front();
    m_listRCPictures.pop_front();
    delete p;
  }

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
  if ( m_rcStatsFHandle.is_open() )
  {
    m_rcStatsFHandle.close();
  }
  m_pqpaStatsWritten = 0;
#endif
}

void RateCtrl::init( const VVEncCfg& encCfg )
{
  destroy();

  m_pcEncCfg = &encCfg;

  encRCSeq = new EncRCSeq;
  encRCSeq->create( m_pcEncCfg->m_RCNumPasses == 2, m_pcEncCfg->m_LookAhead, m_pcEncCfg->m_RCTargetBitrate, (int)((double)(m_pcEncCfg->m_FrameRate / m_pcEncCfg->m_FrameScale) / m_pcEncCfg->m_temporalSubsampleRatio + 0.5), m_pcEncCfg->m_IntraPeriod, m_pcEncCfg->m_GOPSize, m_pcEncCfg->m_internalBitDepth[CH_L], getFirstPassStats() );
}

int RateCtrl::getBaseQP()
{
  // estimate near-optimal base QP for PPS in second RC pass
  double d = (3840.0 * 2160.0) / double (m_pcEncCfg->m_SourceWidth * m_pcEncCfg->m_SourceHeight);
  const int firstPassBaseQP = std::max (17, MAX_QP_PERCEPT_QPA - 2 - int (0.5 + sqrt ((d * m_pcEncCfg->m_RCTargetBitrate) / 500000.0)));
  const unsigned fps = m_pcEncCfg->m_FrameRate / m_pcEncCfg->m_FrameScale;
  std::list<TRCPassStats>& firstPassData = m_listRCFirstPassStats;
  int baseQP = MAX_QP;

  if (firstPassData.size() > 0 && fps > 0)
  {
    const int log2HeightMinus7 = int (0.5 + log ((double) std::max (128, m_pcEncCfg->m_SourceHeight)) / log (2.0)) - 7;
    uint64_t sumFrBits = 0, sumVisAct = 0; // first-pass data

    for (auto& stats : firstPassData)
    {
      sumFrBits += stats.numBits;
      sumVisAct += stats.visActY;
    }
    d = (double) m_pcEncCfg->m_RCTargetBitrate * (double) firstPassData.size() / double (fps * sumFrBits);
    d = firstPassBaseQP - (105.0 / 128.0) * sqrt ((double) std::max (1, firstPassBaseQP)) * log (d) / log (2.0);
    baseQP = int (0.5 + d + 0.125 * log2HeightMinus7 * std::max (0.0, 24.0 + 0.001/*log2HeightMinus7*/ * (log ((double) sumVisAct / firstPassData.size()) / log (2.0) - 0.5 * m_pcEncCfg->m_internalBitDepth[CH_L] - 3.0) - d));
  }
  else if (m_pcEncCfg->m_LookAhead)
  {
    baseQP = firstPassBaseQP - (std::max (0, MAX_QP_PERCEPT_QPA - firstPassBaseQP) >> 1);
  }

  return Clip3 (17, MAX_QP, baseQP);
}

void RateCtrl::setRCPass(const VVEncCfg& encCfg, const int pass, const char* statsFName)
{
  m_pcEncCfg    = &encCfg;
  rcPass        = pass;
  rcIsFinalPass = (pass >= m_pcEncCfg->m_RCNumPasses - 1);

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
  if( m_rcStatsFHandle.is_open() ) m_rcStatsFHandle.close();

  const std::string name = statsFName != nullptr ? statsFName : "";
  if( name.length() )
  {
    openStatsFile( name );
    if( rcIsFinalPass )
    {
      readStatsFile();
    }
  }
#else
  CHECK( statsFName != nullptr && strlen( statsFName ) > 0, "reading/writing rate control statistics file not supported, please compile with json enabled" );
#endif
}

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
void RateCtrl::openStatsFile(const std::string& name)
{
  if( rcIsFinalPass )
  {
    m_rcStatsFHandle.open( name, std::ios::in );
    CHECK( m_rcStatsFHandle.fail(), "unable to open rate control statistics file for reading" );
    readStatsHeader();
  }
  else
  {
    m_rcStatsFHandle.open( name, std::ios::trunc | std::ios::out );
    CHECK( m_rcStatsFHandle.fail(), "unable to open rate control statistics file for writing" );
    writeStatsHeader();
  }
}
#endif

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
void RateCtrl::writeStatsHeader()
{
  nlohmann::json header = {
    { "version",      VVENC_VERSION },
    { "SourceWidth",  m_pcEncCfg->m_SourceWidth },
    { "SourceHeight", m_pcEncCfg->m_SourceHeight },
    { "CTUSize",      m_pcEncCfg->m_CTUSize },
    { "GOPSize",      m_pcEncCfg->m_GOPSize },
    { "IntraPeriod",  m_pcEncCfg->m_IntraPeriod },
    { "PQPA",         m_pcEncCfg->m_usePerceptQPA },
    { "QP",           m_pcEncCfg->m_QP },
    { "RCInitialQP",  m_pcEncCfg->m_RCInitialQP }
  };
  m_rcStatsFHandle << header << std::endl;
}
#endif

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
void RateCtrl::readStatsHeader()
{
  std::string line;
  if( ! std::getline( m_rcStatsFHandle, line ) )
  {
    THROW( "unable to read header from rate control statistics file" );
  }
  nlohmann::json header = nlohmann::json::parse( line );
  if( header.find( "version" )         == header.end() || ! header[ "version" ].is_string()
      || header.find( "SourceWidth" )  == header.end() || ! header[ "SourceWidth" ].is_number()
      || header.find( "SourceHeight" ) == header.end() || ! header[ "SourceHeight" ].is_number()
      || header.find( "CTUSize" )      == header.end() || ! header[ "CTUSize" ].is_number()
      || header.find( "GOPSize" )      == header.end() || ! header[ "GOPSize" ].is_number()
      || header.find( "IntraPeriod" )  == header.end() || ! header[ "IntraPeriod" ].is_number()
      || header.find( "PQPA" )         == header.end() || ! header[ "PQPA" ].is_boolean()
      || header.find( "QP" )           == header.end() || ! header[ "QP" ].is_number()
      || header.find( "RCInitialQP" )  == header.end() || ! header[ "RCInitialQP" ].is_number()
    )
  {
    THROW( "header line in rate control statistics file not recognized" );
  }
  if( header[ "version" ]      != VVENC_VERSION )              msg.log( VVENC_WARNING, "WARNING: wrong version in rate control statistics file\n" );
  if( header[ "SourceWidth" ]  != m_pcEncCfg->m_SourceWidth )  msg.log( VVENC_WARNING, "WARNING: wrong frame width in rate control statistics file\n" );
  if( header[ "SourceHeight" ] != m_pcEncCfg->m_SourceHeight ) msg.log( VVENC_WARNING, "WARNING: wrong frame height in rate control statistics file\n" );
  if( header[ "CTUSize" ]      != m_pcEncCfg->m_CTUSize )      msg.log( VVENC_WARNING, "WARNING: wrong CTU size in rate control statistics file\n" );
  if( header[ "GOPSize" ]      != m_pcEncCfg->m_GOPSize )      msg.log( VVENC_WARNING, "WARNING: wrong GOP size in rate control statistics file\n" );
  if( header[ "IntraPeriod" ]  != m_pcEncCfg->m_IntraPeriod )  msg.log( VVENC_WARNING, "WARNING: wrong intra period in rate control statistics file\n" );
}
#endif

void RateCtrl::storeStatsData( const TRCPassStats& statsData )
{
#ifdef VVENC_ENABLE_THIRDPARTY_JSON
  nlohmann::json data = {
    { "poc",       statsData.poc },
    { "qp",        statsData.qp },
    { "lambda",    statsData.lambda },
    { "visActY",   statsData.visActY },
    { "numBits",   statsData.numBits },
    { "psnrY",     statsData.psnrY },
    { "isIntra",   statsData.isIntra },
    { "tempLayer", statsData.tempLayer }
  };

  if( m_rcStatsFHandle.is_open() )
  {
    CHECK( ! m_rcStatsFHandle.good(), "unable to write to rate control statistics file" );
    if( m_listRCIntraPQPAStats.size() > m_pqpaStatsWritten )
    {
      std::vector<uint8_t> pqpaTemp;
      while( m_pqpaStatsWritten < (int)m_listRCIntraPQPAStats.size() )
      {
        pqpaTemp.push_back( m_listRCIntraPQPAStats[ m_pqpaStatsWritten ] );
        m_pqpaStatsWritten++;
      }
      data[ "pqpaStats" ] = pqpaTemp;
    }
    m_rcStatsFHandle << data << std::endl;
  }
  else
  {
    // ensure same precision for internal and written data by serializing internal data as well
    std::stringstream iss;
    iss << data;
    data = nlohmann::json::parse( iss.str() );
    m_listRCFirstPassStats.push_back( TRCPassStats( data[ "poc" ],
                                                    data[ "qp" ],
                                                    data[ "lambda" ],
                                                    data[ "visActY" ],
                                                    data[ "numBits" ],
                                                    data[ "psnrY" ],
                                                    data[ "isIntra" ],
                                                    data[ "tempLayer" ]
                                                    ) );
    if( m_pcEncCfg->m_LookAhead && (int) m_listRCFirstPassStats.size() > m_pcEncCfg->m_IntraPeriod + m_pcEncCfg->m_GOPSize + 1 )
    {
      m_listRCFirstPassStats.pop_front();
    }
  }
#else
  m_listRCFirstPassStats.push_back( statsData );

  if( m_pcEncCfg->m_LookAhead && (int) m_listRCFirstPassStats.size() > m_pcEncCfg->m_IntraPeriod + m_pcEncCfg->m_GOPSize + 1 )
  {
    m_listRCFirstPassStats.pop_front();
  }
#endif
}

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
void RateCtrl::readStatsFile()
{
  CHECK( ! m_rcStatsFHandle.good(), "unable to read from rate control statistics file" );

  int lineNum = 2;
  std::string line;
  while( std::getline( m_rcStatsFHandle, line ) )
  {
    nlohmann::json data = nlohmann::json::parse( line );
    if( data.find( "poc" )          == data.end() || ! data[ "poc" ].is_number()
        || data.find( "qp" )        == data.end() || ! data[ "qp" ].is_number()
        || data.find( "lambda" )    == data.end() || ! data[ "lambda" ].is_number()
        || data.find( "visActY" )   == data.end() || ! data[ "visActY" ].is_number()
        || data.find( "numBits" )   == data.end() || ! data[ "numBits" ].is_number()
        || data.find( "psnrY" )     == data.end() || ! data[ "psnrY" ].is_number()
        || data.find( "isIntra" )   == data.end() || ! data[ "isIntra" ].is_boolean()
        || data.find( "tempLayer" ) == data.end() || ! data[ "tempLayer" ].is_number() )
    {
      THROW( "syntax of rate control statistics file in line " << lineNum << " not recognized: (" << line << ")" );
    }
    m_listRCFirstPassStats.push_back( TRCPassStats( data[ "poc" ],
                                                    data[ "qp" ],
                                                    data[ "lambda" ],
                                                    data[ "visActY" ],
                                                    data[ "numBits" ],
                                                    data[ "psnrY" ],
                                                    data[ "isIntra" ],
                                                    data[ "tempLayer" ]
                                                    ) );
    if( data.find( "pqpaStats" ) != data.end() )
    {
      CHECK( ! data[ "pqpaStats" ].is_array(), "pqpa array data in rate control statistics file not recognized" );
      std::vector<uint8_t> pqpaTemp = data[ "pqpaStats" ];
      for( auto el : pqpaTemp )
      {
        m_listRCIntraPQPAStats.push_back( el );
      }
    }
    lineNum++;
  }
}
#endif

void RateCtrl::processFirstPassData (const bool flush)
{
  CHECK( m_listRCFirstPassStats.size() == 0, "No data available from the first pass!" );

  m_listRCFirstPassStats.sort( []( const TRCPassStats& a, const TRCPassStats& b ) { return a.poc < b.poc; } );

  if ( flush || !m_pcEncCfg->m_LookAhead )
  {
    // store start POC of last chunk of pictures
    flushPOC = m_listRCFirstPassStats.back().poc - std::max( 32, m_pcEncCfg->m_GOPSize );
  }

  // run a simple scene change detection
  detectNewScene();

  // process and scale GOP and frame bits using the data from the first pass to account for different target bitrates
  processGops();

  // loop though the first-pass data and update RC parameters when new scenes are detected
  adaptToSceneChanges();

  encRCSeq->firstPassData = m_listRCFirstPassStats;
}

double RateCtrl::getAverageBitsFromFirstPass()
{
  uint64_t totalBitsFirstPass = 0;
  std::list<TRCPassStats>::iterator it;

  if (encRCSeq->intraPeriod > 1 && encRCSeq->gopSize > 1 && m_pcEncCfg->m_LookAhead)
  {
    const int  gopsInIp = encRCSeq->intraPeriod / encRCSeq->gopSize;
    uint64_t tlBits [8] = { 0 };
    unsigned tlCount[8] = { 0 };
    double bitsUsed;

    for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // sum per level
    {
      tlBits [it->tempLayer] += it->numBits;
      tlCount[it->tempLayer]++;
    }

    totalBitsFirstPass = (tlBits[0] + (tlCount[0] >> 1)) / std::max (1u, tlCount[0]) +
        ((gopsInIp - 1) * tlBits[1] + (tlCount[1] >> 1)) / std::max (1u, tlCount[1]);
    for (int frameLevel = 2; frameLevel <= 7; frameLevel++)
    {
      totalBitsFirstPass += ((gopsInIp << (frameLevel - 2)) * tlBits[frameLevel] + (tlCount[frameLevel] >> 1)) / std::max (1u, tlCount[frameLevel]);
    }
    bitsUsed = totalBitsFirstPass / (double) encRCSeq->intraPeriod;

    if (m_listRCFirstPassStats.size() < encRCSeq->gopSize) // apply crossfade, similar to updateAfterPic
    {
      totalBitsFirstPass = tlBits[0] + tlBits[1] + tlBits[2] + tlBits[3] + tlBits[4] + tlBits[5] + tlBits[6] + tlBits[7];
      bitsUsed = ((double) totalBitsFirstPass * (encRCSeq->gopSize - m_listRCFirstPassStats.size()) / (double) m_listRCFirstPassStats.size() + bitsUsed * m_listRCFirstPassStats.size()) / (double) encRCSeq->gopSize;
    }

    return bitsUsed;
  }

  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // for two-pass RC
  {
    totalBitsFirstPass += it->numBits;
  }

  return totalBitsFirstPass / (double) m_listRCFirstPassStats.size();
}

void RateCtrl::detectNewScene()
{
  uint16_t visActPrev = 0;
  double   psnrPrev = 0.0;
  std::list<TRCPassStats>::iterator it;

  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++)
  {
    it->isNewScene = ((it->visActY * 64 > visActPrev * 181) || (it->tempLayer <= 1 && it->visActY <= (1u << (encRCSeq->bitDepth - 6))) || (it->tempLayer <= 1 && it->visActY + 3 > visActPrev && std::abs(it->psnrY - psnrPrev) > 4.5));
    visActPrev = it->visActY;
    if (it->tempLayer <= 1) psnrPrev = it->psnrY;
  }
}

void RateCtrl::processGops()
{
  const unsigned fps = encRCSeq->frameRate;
  const int gopShift = int (0.5 + log ((double) encRCSeq->gopSize) / log (2.0));
  const int itOffset = m_listRCFirstPassStats.begin()->poc + 1; // NOTE: in two-pass RC, this value is 1
  const int qpOffset = Clip3 (0, 6, ((m_pcEncCfg->m_QP + 1) >> 1) - 9);
  const double bp1pf = getAverageBitsFromFirstPass();  // first-pass bits/frame
  const double ratio = (double) encRCSeq->targetRate / (fps * bp1pf);  // ratio of second and first pass
  const double rp[6] = { pow (ratio, 0.5), pow (ratio, 0.75), pow (ratio, 0.875), pow (ratio, 0.9375), pow (ratio, 0.96875), pow (ratio, 0.984375) };
  std::list<TRCPassStats>::iterator it;
  std::vector<uint32_t> gopBits (2 + (m_listRCFirstPassStats.size() - 1) / encRCSeq->gopSize); // +2 for the first I frame (GOP) and a potential last incomplete GOP
  std::vector<uint32_t> tgtBits (2 + (m_listRCFirstPassStats.size() - 1) / encRCSeq->gopSize);

  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // scaling, part 1
  {
    const int vecIdx = 1 + ((it->poc - itOffset) >> gopShift);

    it->targetBits = std::max (0, int (0.5 + it->numBits * (it->tempLayer + qpOffset < 6 ? rp[it->tempLayer + qpOffset] : ratio)));
    gopBits[vecIdx] += (uint32_t) it->targetBits; // similar to g in VCIP paper
    tgtBits[vecIdx] += (uint32_t) (0.5 + it->numBits * ratio);
  }
  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // scaling, part 2
  {
    const int vecIdx = 1 + ((it->poc - itOffset) >> gopShift);

    it->frameInGopRatio = (double) it->targetBits / gopBits[vecIdx];
    it->targetBits = std::max (1, int (0.5 + it->frameInGopRatio * tgtBits[vecIdx]));
  }
}

void RateCtrl::adaptToSceneChanges()
{
  const int numOfLevels = int ( log( encRCSeq->gopSize ) / log( 2.0 ) + 0.5 ) + 2;

  std::list<TRCPassStats>::iterator it;
  int prevSceneChange = -100;
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++ )
  {
    if ( it->isNewScene ) // filter out scene cuts which happen too close to the previous detected scene cut
    {
      if ( it->poc >= prevSceneChange + (encRCSeq->gopSize >> 1) )
      {
        prevSceneChange = it->poc;
      }
      else
      {
        it->isNewScene = false;
      }
    }
  }

  // assumption: scene cut detection data cleaned and RC parameters updated; first-pass data sorted by POC
  // mark frames where parameter refresh is needed after a scene cut
  bool* refreshNeeded = new bool[ numOfLevels ];
  for ( int i = 0; i < numOfLevels; i++ )
  {
    refreshNeeded[ i ] = false;
  }
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++ )
  {
    if ( it->isNewScene )
    {
      for ( int i = 0; i < numOfLevels; i++ )
      {
        refreshNeeded[ i ] = true;
      }
    }
    it->refreshParameters = refreshNeeded[ it->tempLayer ];
    refreshNeeded[ it->tempLayer ] = false;
  }

  delete[] refreshNeeded;
}

void RateCtrl::addRCPassStats (const int poc, const int qp, const double lambda, const uint16_t visActY,
                               const uint32_t numBits, const double psnrY, const bool isIntra, const int tempLayer)
{
  storeStatsData( TRCPassStats( poc, qp, lambda, visActY, numBits, psnrY, isIntra, tempLayer + int( !isIntra ) ) );
}

void RateCtrl::xUpdateAfterPicRC( const Picture* pic )
{
  EncRCPic* encRCPic = pic->encRCPic;

  encRCPic->updateAfterPicture( pic->actualTotalBits, pic->slices[ 0 ]->sliceQp, pic->slices[ 0 ]->isIRAP() );
  encRCPic->addToPictureList( getPicList() );
  encRCSeq->updateAfterPic( pic->actualTotalBits, encRCPic->tmpTargetBits );
}

void RateCtrl::xUpdateAfterCtuRC( const Slice* slice, const int numberOfWrittenBits, const int ctuRsAddr, std::mutex* m_rcMutex, const double lambda )
{
  //THROW( "Not supported in 2-pass rate control" );
}

void RateCtrl::initRateControlPic( Picture& pic, Slice* slice, int& qp, double& finalLambda )
{
  EncRCPic* encRcPic = new EncRCPic;
  encRcPic->create( encRCSeq, pic.slices[ 0 ]->isIntra() ? 0 : pic.slices[ 0 ]->TLayer + 1, pic.slices[ 0 ]->poc );
  pic.encRCPic = encRcPic;
  encRCPic = encRcPic;

  const int frameLevel = ( slice->isIntra() ? 0 : slice->TLayer + 1 );
  double lambda = encRCSeq->maxEstLambda;
  int   sliceQP = MAX_QP;

  if ( frameLevel <= 7 )
  {
    if ( m_pcEncCfg->m_RCNumPasses == 2 || m_pcEncCfg->m_LookAhead )
    {
      EncRCSeq* encRcSeq = encRCSeq;
      std::list<TRCPassStats>::iterator it;

      for ( it = encRcSeq->firstPassData.begin(); it != encRcSeq->firstPassData.end(); it++ )
      {
        if ( ( it->poc == slice->poc ) && ( encRcPic->targetBits > 0 ) && ( it->numBits > 0 ) )
        {
          double d = (double)encRcPic->targetBits;
          const int firstPassSliceQP = it->qp;
          const int log2HeightMinus7 = int( 0.5 + log( (double)std::max( 128, m_pcEncCfg->m_SourceHeight ) ) / log( 2.0 ) ) - 7;
          uint16_t visAct = it->visActY;

          if ( it->isNewScene ) // spatiotemporal visual activity is transient at camera/scene change, find next steady-state activity
          {
            std::list<TRCPassStats>::iterator itNext = it;

            itNext++;
            while ( itNext != encRcSeq->firstPassData.end() && !itNext->isIntra )
            {
              if ( itNext->poc == it->poc + 2 )
              {
                visAct = itNext->visActY;
                break;
              }
              itNext++;
            }
          }
          if ( it->refreshParameters )
          {
            encRCSeq->qpCorrection[ frameLevel ] = ( ( it->poc == 0 ) && ( d < it->numBits ) ? std::max( -1.0 * visAct / double( 1 << ( encRCSeq->bitDepth - 3 ) ), 1.0 - it->numBits / d ) : 0.0 );
            if ( !m_pcEncCfg->m_LookAhead )
            {
              encRCSeq->actualBitCnt[ frameLevel ] = encRCSeq->targetBitCnt[ frameLevel ] = 0;
            }
          }
          CHECK( slice->TLayer >= 7, "analyzed RC frame must have TLayer < 7" );

          // try to reach target rate less aggressively in first coded frames, prevents temporary very low quality during second GOP
          if ( it->poc == m_pcEncCfg->m_GOPSize )
          {
            d = std::max( 1.0, d - ( encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed ) * 0.25 * it->frameInGopRatio );
            encRcPic->targetBits = int( d + 0.5 ); // update the member to be on the safe side
          }
          // try to hit target rate more aggressively in last coded frames, lambda/QP clipping below will ensure smooth value change
          if ( it->poc >= flushPOC && flushPOC >= 0 )
          {
            if ( m_pcEncCfg->m_LookAhead && encRcSeq->bitsUsedIn1stPass > 0 )
            {
              const double bp1pf = (double)encRCSeq->bitsUsedIn1stPass / (double)std::max( 1, encRCSeq->framesCoded ); // first pass
              const double ratio = (double)encRCSeq->targetRate / ( encRCSeq->frameRate * bp1pf ); // targeted 2nd-to-1st pass ratio
              d = std::max( 0.0, d - ( encRCSeq->estimatedBitUsage - encRCSeq->bitsUsed ) * 0.5 * it->frameInGopRatio );
              d = std::max( 1.0, d + ( encRCSeq->bitsUsedIn1stPass * ratio - encRCSeq->bitsUsed ) * it->frameInGopRatio );
            }
            else
            {
              d = std::max( 1.0, d + ( encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed ) * 0.5 * it->frameInGopRatio );
              encRcPic->targetBits = int( d + 0.5 ); // update the member to be on the safe side
            }
          }
          d /= (double)it->numBits;
          d = firstPassSliceQP - ( 105.0 / 128.0 ) * sqrt( (double)std::max( 1, firstPassSliceQP ) ) * log( d ) / log( 2.0 );
          sliceQP = int( 0.5 + d + 0.125 * log2HeightMinus7 * std::max( 0.0, 24.0 + 0.001/*log2HeightMinus7*/ * ( log( (double)visAct ) / log( 2.0 ) - 0.5 * encRcSeq->bitDepth - 3.0 ) - d ) + encRCSeq->qpCorrection[ frameLevel ] );
          if ( it->poc == 0 || it->refreshParameters ) // avoid overcoding after some scene cuts
          {
            const int clipQP = ( ( ( ( 1 << encRCSeq->bitDepth ) + ( it->poc == 0 ? 1 << ( encRCSeq->bitDepth - 1 ) : visAct ) ) * m_pcEncCfg->m_QP ) >> ( encRCSeq->bitDepth + 1 ) ) + ( it->isIntra ? m_pcEncCfg->m_intraQPOffset : 0 );

            if ( sliceQP < clipQP )
            {
              encRcPic->targetBits = int( 0.5 + encRcPic->targetBits * pow( 2.0, (sliceQP - clipQP) / 5.0 ) ); // VCIP paper, Tab. 1
              sliceQP = clipQP;
            }
          }
          encRcPic->clipTargetQP( getPicList(), sliceQP );
          lambda = it->lambda * pow( 2.0, double( sliceQP - firstPassSliceQP ) / 3.0 );
          lambda = Clip3( encRcSeq->minEstLambda, encRcSeq->maxEstLambda, lambda );

          if ( m_pcEncCfg->m_LookAhead )
          {
            encRCSeq->bitsUsedIn1stPass += it->numBits;

            if ( (sliceQP > 0) && slice->pps->sliceChromaQpFlag && slice->isIntra() && !pic.cs->pcv->ISingleTree && !m_pcEncCfg->m_usePerceptQPA && (m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity == 0) )
            {
              sliceQP--; // balance BD-rate performance across all YCbCr components; see also code in EncSlice::xInitSliceLambdaQP()
              lambda *= 0.7937; // * pow (2, -1/3)
            }
          }
          if ( it->isIntra ) // update history, for parameter clipping in subsequent key frames
          {
            encRcSeq->lastIntraQP = sliceQP;
          }

          break;
        }
      }
    }
  }

  qp = sliceQP;
  finalLambda = lambda;
}

void RateCtrl::setFinalLambda( const double lambda )
{
  //THROW( "Not supported in 2-pass rate control" );
}

void RateCtrl::initRCGOP( const int numberOfPictures )
{

  //THROW("Not supported in 2-pass rate control");
}

void RateCtrl::destroyRCGOP()
{
  //THROW( "Not supported in 2-pass rate control" );
}

}

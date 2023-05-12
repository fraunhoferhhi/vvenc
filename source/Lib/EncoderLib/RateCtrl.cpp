/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

//sequence level
EncRCSeq::EncRCSeq()
{
  twoPass             = false;
  isLookAhead         = false;
  framesCoded         = 0;
  targetRate          = 0;
  frameRate           = 0.0;
  gopSize             = 0;
  intraPeriod         = 0;
  bitsUsed            = 0;
  bitsUsedIn1stPass   = 0;
  bitsUsedQPLimDiff   = 0;
  estimatedBitUsage   = 0;
  std::memset (qpCorrection, 0, sizeof (qpCorrection));
  std::memset (actualBitCnt, 0, sizeof (actualBitCnt));
  std::memset (currFrameCnt, 0, sizeof (currFrameCnt));
  std::memset (targetBitCnt, 0, sizeof (targetBitCnt));
  lastIntraQP         = 0;
  lastIntraBitsSaved  = false;
  bitDepth            = 0;
}

EncRCSeq::~EncRCSeq()
{
  destroy();
}

void EncRCSeq::create( bool twoPassRC, bool lookAhead, int targetBitrate, double frRate, int intraPer, int GOPSize, int bitDpth, std::list<TRCPassStats> &firstPassStats )
{
  destroy();
  twoPass             = twoPassRC;
  isLookAhead         = lookAhead;
  targetRate          = targetBitrate;
  frameRate           = frRate;
  intraPeriod         = Clip3<unsigned>( GOPSize, 4 * VVENC_MAX_GOP, intraPer );
  gopSize             = GOPSize;
  firstPassData       = firstPassStats;
  bitDepth            = bitDpth;

  int bitdepthLumaScale = 2 * ( bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( bitDepth ) );
  minEstLambda = 0.1;
  maxEstLambda = 65535.9375 * pow( 2.0, bitdepthLumaScale );

  framesCoded = 0;
  bitsUsed = 0;
  bitsUsedIn1stPass = 0;
  bitsUsedQPLimDiff = 0;
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

void EncRCSeq::updateAfterPic (const int actBits, const int tgtBits)
{
  framesCoded++;

  if (isLookAhead)
  {
    const uint64_t* const tlBits  = actualBitCnt; // recently updated in EncRCPic::updateAfterPicture()
    const unsigned* const tlCount = currFrameCnt;

    estimatedBitUsage = int64_t (0.5 + ((double) targetRate * framesCoded) / frameRate);

    if (framesCoded < intraPeriod) // apply crossfade between I-period estimate and simple bit-counting
    {
      const int gopsInIp = intraPeriod / gopSize;
      uint64_t totalBitsSecondPass = (tlBits[0] + (tlCount[0] >> 1)) / std::max (1u, tlCount[0]) +
                    ((gopsInIp - 1) * tlBits[1] + (tlCount[1] >> 1)) / std::max (1u, tlCount[1]);
      for (int l = 2; l <= 7; l++)
      {
        totalBitsSecondPass += ((gopsInIp << (l - 2)) * tlBits[l] + (tlCount[l] >> 1)) / std::max (1u, tlCount[l]);
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
    bitsUsed += actBits;
  }
}

//picture level
EncRCPic::EncRCPic()
{
  encRCSeq            = NULL;
  frameLevel          = 0;
  targetBits          = 0;
  tmpTargetBits       = 0;
  picQP               = 0;
  poc                 = 0;
  refreshParams       = false;
  visActSteady        = 0;
}

EncRCPic::~EncRCPic()
{
  destroy();
}

void EncRCPic::addToPictureList( std::list<EncRCPic*>& listPreviousPictures )
{
  if ( listPreviousPictures.size() > std::min( VVENC_MAX_GOP, 2 * encRCSeq->gopSize ) )
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
  frameLevel = frameLvl;
}

void EncRCPic::destroy()
{
  encRCSeq = NULL;
}

void EncRCPic::clipTargetQP (std::list<EncRCPic*>& listPreviousPictures, const int baseQP, int &qp)
{
  int lastCurrTLQP = -1;
  int lastPrevTLQP = -1;
  int halvedAvgQP  = -1;
  std::list<EncRCPic*>::iterator it;

  for (it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++)
  {
    if ((*it)->frameLevel == frameLevel && (*it)->picQP >= 0)  // current temporal level
    {
      lastCurrTLQP = (*it)->picQP;
    }
    if ((*it)->frameLevel == frameLevel - 1 && (*it)->picQP >= 0) // last temporal level
    {
      lastPrevTLQP = (*it)->picQP >> (frameLevel == 1 ? 1 : 0);
    }
    if ((*it)->frameLevel == 1 && frameLevel == 0 && refreshParams && lastCurrTLQP < 0)
    {
      lastCurrTLQP = (*it)->picQP;
    }
    halvedAvgQP += (*it)->picQP;
  }
  if (listPreviousPictures.size() >= 1)
  {
    if (baseQP >= 32)
      halvedAvgQP = int (((frameLevel + 1) * halvedAvgQP + 1 + listPreviousPictures.size()) / ((frameLevel + 2) * listPreviousPictures.size()));
    else
      halvedAvgQP = int ((halvedAvgQP + 1 + listPreviousPictures.size()) / (2 * listPreviousPictures.size()));
  }
  if (frameLevel <= 1 && lastPrevTLQP < halvedAvgQP) lastPrevTLQP = halvedAvgQP; // TL0I
  if (frameLevel == 1 && lastCurrTLQP < 0) lastCurrTLQP = encRCSeq->lastIntraQP; // TL0B

  qp = Clip3 (frameLevel + std::max (0, baseQP >> 1), MAX_QP, qp);

  if (lastCurrTLQP >= 0) // limit QP changes among prev. frames from same temporal level
  {
    const int clipRange = (refreshParams ? 5 + (encRCSeq->intraPeriod + (encRCSeq->gopSize >> 1)) / encRCSeq->gopSize : std::max (3, 6 - (frameLevel >> 1)));

    qp = Clip3 (lastCurrTLQP - clipRange, std::min (MAX_QP, lastCurrTLQP + clipRange), qp);
  }
  if (lastPrevTLQP >= 0) // prevent QP from being lower than QPs at lower temporal level
  {
    qp = Clip3 (std::min (MAX_QP, lastPrevTLQP + 1), MAX_QP, qp);
  }
  else if (encRCSeq->lastIntraQP >= -1 && (frameLevel == 1 || frameLevel == 2))
  {
    qp = Clip3 ((encRCSeq->lastIntraQP >> 1) + 1, MAX_QP, qp);
  }
}

void EncRCPic::updateAfterPicture (const int picActualBits, const int averageQP)
{
  picQP = averageQP;

  if ((frameLevel <= 7) && (picActualBits > 0) && (targetBits > 0)) // update, for initRateControlPic()
  {
    const uint16_t vaMin = 1u << (encRCSeq->bitDepth - 6);
    const double clipVal = (visActSteady > 0 && visActSteady < vaMin + 48 ? 0.25 * (visActSteady - vaMin) : 12.0);

    encRCSeq->actualBitCnt[frameLevel] += (uint64_t) picActualBits;
    encRCSeq->targetBitCnt[frameLevel] += (uint64_t) targetBits;

    if (encRCSeq->isLookAhead) encRCSeq->currFrameCnt[frameLevel]++;

    encRCSeq->qpCorrection[frameLevel] = (refreshParams ? 1.0 : 5.0) * log ((double) encRCSeq->actualBitCnt[frameLevel] / (double) encRCSeq->targetBitCnt[frameLevel]) / log (2.0); // 5.0 as in VCIP paper, Tab. 1
    encRCSeq->qpCorrection[frameLevel] = Clip3 (-clipVal, clipVal, encRCSeq->qpCorrection[frameLevel]);

    if (frameLevel > std::max (1, int (log ((double) encRCSeq->gopSize) / log (2.0))))
    {
      double highTlQpCorr = 0.0;

      for (int l = 2; l <= 7; l++) // stabilization when corrections differ between low and high levels
      {
        highTlQpCorr += encRCSeq->qpCorrection[l];
      }
      if (highTlQpCorr > 1.0) // attenuate low-level QP correction towards 0 when bits need to be saved
      {
        if (encRCSeq->qpCorrection[0] < -1.0e-9) encRCSeq->qpCorrection[0] /= highTlQpCorr;
        if (encRCSeq->qpCorrection[1] < -1.0e-9) encRCSeq->qpCorrection[1] /= highTlQpCorr;
      }
    }
  }
}

RateCtrl::RateCtrl(MsgLog& logger)
: msg ( logger )
{
  m_pcEncCfg           = nullptr;
  encRCSeq             = NULL;
  encRCPic             = NULL;
  flushPOC             = -1;
  rcPass               = 0;
  rcIsFinalPass        = true;
#ifdef VVENC_ENABLE_THIRDPARTY_JSON
  m_pqpaStatsWritten   = 0;
#endif
  m_numPicStatsTotal   = 0;
  m_numPicAddedToList  = 0;
  m_updateNoisePoc     = -1;
  m_resetNoise         = true;
  std::fill_n( m_minNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u );
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
  encRCSeq->create( m_pcEncCfg->m_RCNumPasses == 2, m_pcEncCfg->m_LookAhead == 1, m_pcEncCfg->m_RCTargetBitrate, (double)m_pcEncCfg->m_FrameRate / m_pcEncCfg->m_FrameScale, m_pcEncCfg->m_IntraPeriod, m_pcEncCfg->m_GOPSize, m_pcEncCfg->m_internalBitDepth[CH_L], getFirstPassStats() );
}

int RateCtrl::getBaseQP()
{
  // estimate near-optimal base QP for PPS in second RC pass
  double d = (3840.0 * 2160.0) / double (m_pcEncCfg->m_SourceWidth * m_pcEncCfg->m_SourceHeight);
  const double firstQPOffset = sqrt ((d * m_pcEncCfg->m_RCTargetBitrate) / 500000.0);
  std::list<TRCPassStats>& firstPassData = m_listRCFirstPassStats;
  int baseQP = MAX_QP;

  if (firstPassData.size() > 0 && encRCSeq->frameRate > 0.0)
  {
    const int firstPassBaseQP = (m_pcEncCfg->m_RCInitialQP > 0 ? Clip3 (17, MAX_QP, m_pcEncCfg->m_RCInitialQP) : std::max (17, MAX_QP_PERCEPT_QPA - 2 - int (0.5 + firstQPOffset)));
    uint64_t sumFrBits = 0;  // sum of first-pass frame bits

    for (auto& stats : firstPassData)
    {
      sumFrBits += stats.numBits;
    }
    baseQP = int (24.5 - log (d) / log (2.0)); // QPstart, equivalent to round (24 + 2*log2 (resRatio))
    d = (double) m_pcEncCfg->m_RCTargetBitrate * (double) firstPassData.size() / (encRCSeq->frameRate * sumFrBits);
    d = firstPassBaseQP - (105.0 / 128.0) * sqrt ((double) std::max (1, firstPassBaseQP)) * log (d) / log (2.0);
    baseQP = int (0.5 + d + 0.5 * std::max (0.0, baseQP - d));
  }
  else if (m_pcEncCfg->m_LookAhead)
  {
    baseQP = int (24.5 - log (d) / log (2.0)); // QPstart, equivalent to round (24 + 2*log2 (resRatio))
    d = MAX_QP_PERCEPT_QPA - 2.0 - 1.5 * firstQPOffset - 0.5 * log (double (encRCSeq->intraPeriod / encRCSeq->gopSize)) / log (2.0);
    baseQP = int (0.5 + d + 0.5 * std::max (0.0, baseQP - d));
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
#endif // VVENC_ENABLE_THIRDPARTY_JSON

void RateCtrl::storeStatsData( const TRCPassStats& statsData )
{
#ifdef VVENC_ENABLE_THIRDPARTY_JSON
  nlohmann::json data = {
    { "poc",            statsData.poc },
    { "qp",             statsData.qp },
    { "lambda",         statsData.lambda },
    { "visActY",        statsData.visActY },
    { "numBits",        statsData.numBits },
    { "psnrY",          statsData.psnrY },
    { "isIntra",        statsData.isIntra },
    { "tempLayer",      statsData.tempLayer },
    { "isStartOfIntra", statsData.isStartOfIntra },
    { "isStartOfGop",   statsData.isStartOfGop },
    { "gopNum",         statsData.gopNum },
    { "scType",         statsData.scType },
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
    std::list<TRCPassStats>& listRCFirstPassStats = m_pcEncCfg->m_LookAhead ? m_firstPassCache : m_listRCFirstPassStats;
    listRCFirstPassStats.push_back( TRCPassStats( data[ "poc" ],
                                                    data[ "qp" ],
                                                    data[ "lambda" ],
                                                    data[ "visActY" ],
                                                    data[ "numBits" ],
                                                    data[ "psnrY" ],
                                                    data[ "isIntra" ],
                                                    data[ "tempLayer" ],
                                                    data[ "isStartOfIntra" ],
                                                    data[ "isStartOfGop" ],
                                                    data[ "gopNum" ],
                                                    data[ "scType" ],
                                                    statsData.minNoiseLevels
                                                    ) );
  }
  m_numPicStatsTotal++;
#else
  m_listRCFirstPassStats.push_back( statsData );

  if( m_pcEncCfg->m_LookAhead && (int) m_listRCFirstPassStats.size() > encRCSeq->intraPeriod + encRCSeq->gopSize + 1 )
  {
    m_listRCFirstPassStats.pop_front();
  }
#endif
}

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
void RateCtrl::readStatsFile()
{
  CHECK( ! m_rcStatsFHandle.good(), "unable to read from rate control statistics file" );

  uint8_t minNoiseLevels[ QPA_MAX_NOISE_LEVELS ];
  std::fill_n( minNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u );

  int lineNum = 2;
  std::string line;
  while( std::getline( m_rcStatsFHandle, line ) )
  {
    nlohmann::json data = nlohmann::json::parse( line );
    if( data.find( "poc" )               == data.end() || ! data[ "poc" ].is_number()
        || data.find( "qp" )             == data.end() || ! data[ "qp" ].is_number()
        || data.find( "lambda" )         == data.end() || ! data[ "lambda" ].is_number()
        || data.find( "visActY" )        == data.end() || ! data[ "visActY" ].is_number()
        || data.find( "numBits" )        == data.end() || ! data[ "numBits" ].is_number()
        || data.find( "psnrY" )          == data.end() || ! data[ "psnrY" ].is_number()
        || data.find( "isIntra" )        == data.end() || ! data[ "isIntra" ].is_boolean()
        || data.find( "tempLayer" )      == data.end() || ! data[ "tempLayer" ].is_number()
        || data.find( "isStartOfIntra" ) == data.end() || ! data[ "isStartOfIntra" ].is_boolean()
        || data.find( "isStartOfGop" )   == data.end() || ! data[ "isStartOfGop" ].is_boolean()
        || data.find( "gopNum" )         == data.end() || ! data[ "gopNum" ].is_number()
        || data.find( "scType" )         == data.end() || ! data[ "scType" ].is_number() )
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
                                                    data[ "tempLayer" ],
                                                    data[ "isStartOfIntra" ],
                                                    data[ "isStartOfGop" ],
                                                    data[ "gopNum" ],
                                                    data[ "scType" ],
                                                    minNoiseLevels
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

void RateCtrl::processFirstPassData( const bool flush, const int poc /*= -1*/ )
{
  if( m_pcEncCfg->m_RCNumPasses > 1 )
  {
    // two pass rc
    CHECK( m_pcEncCfg->m_LookAhead, "two pass rc does not support look-ahead mode" );

    xProcessFirstPassData( flush, poc );
  }
  else
  {
    // single pass rc
    CHECK( !m_pcEncCfg->m_LookAhead,     "single pass rc should be only used in look-ahead mode" );
    CHECK( m_firstPassCache.size() == 0, "no data available from the first pass" );
    CHECK( poc < 0,                      "no valid poc given" );

    // fetch RC data for the next look-ahead chunk
    // the next look-ahead chunk starts with a given POC, so find a pic for a given POC in cache
    // NOTE!!!: pictures in cache are in coding order

    auto picCacheItr = find_if( m_firstPassCache.begin(), m_firstPassCache.end(), [poc]( auto& picStat ) { return picStat.poc == poc; } );

    for( int count = 0; picCacheItr != m_firstPassCache.end(); ++picCacheItr )
    {
      auto& picStat = *picCacheItr;
      count++;
      if( !picStat.addedToList )
      {
        picStat.addedToList = true;
        m_numPicAddedToList++;
        m_listRCFirstPassStats.push_back( picStat );
        if( m_pcEncCfg->m_LookAhead && (int) m_listRCFirstPassStats.size() > encRCSeq->intraPeriod + encRCSeq->gopSize + 1 )
        {
          m_listRCFirstPassStats.pop_front();
          m_firstPassCache.pop_front();
        }

        // the chunk is considered either to contain a particular number of pictures or up to next TID0 picture (including it)
        // in flush-mode, ensure the deterministic definition of last chunk
        if( ( count >= m_pcEncCfg->m_GOPSize + 1 || ( picStat.tempLayer == 0 && m_listRCFirstPassStats.size() > 2 ) ) && !( flush && m_numPicAddedToList > m_numPicStatsTotal - m_pcEncCfg->m_GOPSize ) )
          break;
      }
    }
    // enable flush only in last chunk (provides correct calculation of flushPOC)
    xProcessFirstPassData( flush && ( m_numPicAddedToList == m_numPicStatsTotal ), poc );
  }
}

void RateCtrl::xProcessFirstPassData( const bool flush, const int poc )
{
  CHECK( m_listRCFirstPassStats.size() == 0, "No data available from the first pass!" );

  m_listRCFirstPassStats.sort( []( const TRCPassStats& a, const TRCPassStats& b ) { return a.poc < b.poc; } );

  if ( flush || !m_pcEncCfg->m_LookAhead )
  {
    // store start POC of last chunk of pictures
    flushPOC = m_listRCFirstPassStats.back().poc - std::max( 32, m_pcEncCfg->m_GOPSize );
  }

  // perform a simple scene change detection on first-pass data and update RC parameters when new scenes are detected
  detectSceneCuts();

  // process and scale GOP and frame bits using the data from the first pass to account for different target bitrates
  processGops();

  if( m_pcEncCfg->m_GOPSize > 8
      && m_pcEncCfg->m_IntraPeriod >= m_pcEncCfg->m_GOPSize
      && m_pcEncCfg->m_usePerceptQPA
      && m_pcEncCfg->m_RCNumPasses == 1 )
  {
    updateMinNoiseLevelsGop( flush, poc );
  }

  encRCSeq->firstPassData = m_listRCFirstPassStats;
}

double RateCtrl::getAverageBitsFromFirstPass()
{
  uint64_t totalBitsFirstPass = 0;
  std::list<TRCPassStats>::iterator it;

  if (encRCSeq->intraPeriod > 1 && encRCSeq->gopSize > 1 && m_pcEncCfg->m_LookAhead)
  {
    const int gopsInIp  = encRCSeq->intraPeriod / encRCSeq->gopSize;
    int l = 1;
    uint64_t tlBits [8] = { 0 };
    unsigned tlCount[8] = { 0 };
    double bitsUsed;

    for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // sum per level
    {
      if (it->tempLayer <= m_pcEncCfg->m_maxTLayer && it->refreshParameters)
      {
        tlBits[it->tempLayer] = tlCount[it->tempLayer] = 0; // exclude ref-frame stats of previous scene
      }
      tlBits[it->tempLayer] += it->numBits;
      tlCount[it->tempLayer]++;
    }
    if (tlBits[0] == 0)
    {
      l = 0; // no I-frame in the analysis range
    }

    totalBitsFirstPass = (tlBits[0] + (tlCount[0] >> 1)) / std::max (1u, tlCount[0]) +
        ((gopsInIp - l) * tlBits[1] + (tlCount[1] >> 1)) / std::max (1u, tlCount[1]);
    for (l = 2; l <= 7; l++)
    {
      totalBitsFirstPass += ((gopsInIp << (l - 2)) * tlBits[l] + (tlCount[l] >> 1)) / std::max (1u, tlCount[l]);
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

void RateCtrl::detectSceneCuts()
{
  const int minPocDif = (encRCSeq->gopSize + 1) >> 1;
  double psnrTL01Prev = 0.0;
  int sceneCutPocPrev = -minPocDif;
  uint16_t visActPrev = 0;
  bool needRefresh[8] = { false };
  std::list<TRCPassStats>::iterator it = m_listRCFirstPassStats.begin();

  visActPrev = it->visActY;
  if (it->tempLayer <= 1) psnrTL01Prev = it->psnrY;
  it->refreshParameters = (it->poc == 0);

  for (it++; it != m_listRCFirstPassStats.end(); it++)
  {
    const int tmpLevel = it->tempLayer;
    const bool  isTL01 = (tmpLevel <= 1);

    it->isNewScene = ((it->visActY * 64 > visActPrev * 181) || (isTL01 && it->visActY <= (1u << (encRCSeq->bitDepth - 6))) || (isTL01 && it->visActY + 3 > visActPrev && psnrTL01Prev > 0.0 && std::abs (it->psnrY - psnrTL01Prev) > 4.5));

    if (it->isNewScene) // filter out scene cuts which happen too closely to the last detected scene cut
    {
      if (it->poc >= sceneCutPocPrev + minPocDif)
      {
        for (int frameLevel = 0; frameLevel <= m_pcEncCfg->m_maxTLayer + 1; frameLevel++)
        {
          needRefresh[frameLevel] = true;
        }
        sceneCutPocPrev = it->poc;
      }
      else
      {
        it->isNewScene = false;
      }
    }
    if (it->scType == SCT_TL0_SCENE_CUT && !needRefresh[0]) // assume scene cuts at all adapted I-frames
    {
      it->isNewScene = needRefresh[0] = needRefresh[1] = true;
    }

    it->refreshParameters = needRefresh[tmpLevel];
    needRefresh[tmpLevel] = false;

    visActPrev = it->visActY;
    if (isTL01) psnrTL01Prev = it->psnrY;
  }
}

void RateCtrl::processGops()
{
  const double bp1pf = getAverageBitsFromFirstPass();  // first-pass bits/frame
  const double ratio = (double) encRCSeq->targetRate / (encRCSeq->frameRate * bp1pf); // 2nd-to-1st pass
  double fac;
  int vecIdx;
  int gopNum;
  std::list<TRCPassStats>::iterator it;
  std::vector<uint32_t> gopBits (2 + (m_listRCFirstPassStats.back().gopNum - m_listRCFirstPassStats.front().gopNum)); // +2 for the first I frame (GOP) and a potential last incomplete GOP

  vecIdx = 0;
  gopNum = m_listRCFirstPassStats.front().gopNum;
  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // scaling, part 1
  {
    if (it->gopNum > gopNum)
    {
      vecIdx++;
      gopNum = it->gopNum;
    }
    CHECK (vecIdx >= (int) gopBits.size(), "array idx out of bounds");
    it->targetBits = (int) std::max (1.0, 0.5 + it->numBits * ratio);
    gopBits[vecIdx] += (uint32_t) it->numBits; // summed to gf in VCIP'21 paper

    if (it->poc == 0 && it->isIntra) // put the first I-frame into separate GOP
    {
      vecIdx++;
    }
  }
  vecIdx = 0;
  fac = 1.0 / gopBits[vecIdx];
  gopNum = m_listRCFirstPassStats.front().gopNum;
  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // scaling, part 2
  {
    if (it->gopNum > gopNum)
    {
      vecIdx++;
      fac = 1.0 / gopBits[vecIdx];
      gopNum = it->gopNum;
    }
    it->frameInGopRatio = (double) it->numBits * fac; // rf/gf in VCIP'21 paper

    if (it->poc == 0 && it->isIntra) // put the first I-frame into separate GOP
    {
      vecIdx++;
      fac = 1.0 / gopBits[vecIdx];
    }
  }
}

void RateCtrl::updateMinNoiseLevelsGop( const bool flush, const int poc )
{
  CHECK( poc <= m_updateNoisePoc, "given TL0 poc before last TL0 poc" );

  const bool bIncomplete = ( poc - m_updateNoisePoc ) < m_pcEncCfg->m_GOPSize;

  // reset only if full gop pics available or previous gop ends with intra frame
  if( ! bIncomplete || m_resetNoise )
  {
    std::fill_n( m_minNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u );
  }
  m_resetNoise = true;

  // currently disabled for last incomplete gop (TODO: check)
  if( bIncomplete && flush )
  {
    std::fill_n( m_minNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u );
    return;
  }

  // continue with stats after last used poc
  const int startPoc = m_updateNoisePoc + 1;
  auto itr           = find_if( m_listRCFirstPassStats.begin(), m_listRCFirstPassStats.end(),  [ startPoc ]( const auto& stat ) { return stat.poc == startPoc; } );
  if( itr == m_listRCFirstPassStats.end() )
  {
    itr = m_listRCFirstPassStats.begin();
  }

  for( ; itr != m_listRCFirstPassStats.end(); itr++ )
  {
    const auto& stat = *itr;
    if( stat.poc > poc )
    {
      m_resetNoise = stat.isIntra; // in case last update poc is intra, we cannot reuse the old noise levels for the next gop
      break;
    }
    for( int i = 0; i < QPA_MAX_NOISE_LEVELS; i++ )
    {
      if( stat.minNoiseLevels[ i ] < m_minNoiseLevels[ i ] )
      {
        m_minNoiseLevels[ i ] = stat.minNoiseLevels[ i ];
      }
    }
  }

  // store highest poc used for current update
  m_updateNoisePoc = poc;
}

double RateCtrl::updateQPstartModelVal()
{
  unsigned num = 0, sum = 0;

  for (int avgIndex = 0; avgIndex < QPA_MAX_NOISE_LEVELS; avgIndex++) // go through all ranges
  {
    if (m_minNoiseLevels[avgIndex] < 255)
    {
      num++;
      sum += m_minNoiseLevels[avgIndex];
    }
  }

  if (num == 0) return 24.0; // default if no data - else compressed noise level equivalent QP

  return 24.0 + 0.5 * (6.0 * log ((double) sum / (double) num) / log (2.0) - 1.0 - 24.0);
}

void RateCtrl::addRCPassStats( const int poc,
                               const int qp,
                               const double lambda,
                               const uint16_t visActY,
                               const uint32_t numBits,
                               const double psnrY,
                               const bool isIntra,
                               const uint32_t tempLayer,
                               const bool isStartOfIntra,
                               const bool isStartOfGop,
                               const int gopNum,
                               const SceneType scType,
                               const uint8_t minNoiseLevels[ QPA_MAX_NOISE_LEVELS ] )
{
  storeStatsData (TRCPassStats (poc, qp, lambda, visActY, numBits, psnrY, isIntra, tempLayer + int (!isIntra), isStartOfIntra, isStartOfGop, gopNum, scType, minNoiseLevels));
}

void RateCtrl::xUpdateAfterPicRC( const Picture* pic )
{
  const int clipBits = std::max( encRCPic->targetBits, pic->actualTotalBits );
  EncRCPic* encRCPic = pic->encRCPic;

  encRCPic->updateAfterPicture( pic->isMeanQPLimited ? clipBits : pic->actualTotalBits, pic->slices[ 0 ]->sliceQp );
  encRCPic->addToPictureList( getPicList() );
  encRCSeq->updateAfterPic( pic->actualTotalBits, encRCPic->tmpTargetBits );

  if ( encRCSeq->isLookAhead )
  {
    if ( pic->isMeanQPLimited ) encRCSeq->bitsUsedQPLimDiff += pic->actualTotalBits - clipBits;
    if ( encRCSeq->framesCoded >= encRCSeq->intraPeriod ) encRCSeq->bitsUsed += encRCSeq->bitsUsedQPLimDiff; // actual, not ideal bits
  }
}

void RateCtrl::initRateControlPic( Picture& pic, Slice* slice, int& qp, double& finalLambda )
{
  const int frameLevel = ( slice->isIntra() ? 0 : slice->TLayer + 1 );
  EncRCPic*   encRcPic = new EncRCPic;

  encRcPic->create( encRCSeq, frameLevel, slice->poc );
  pic.encRCPic = encRcPic;
  encRCPic = encRcPic;

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
        if ( it->poc == slice->poc && it->numBits > 0 )
        {
          const double dLimit = std::max ( 2.0, 6.0 - double( frameLevel >> 1 ) );
          const double sqrOfResRatio = double( m_pcEncCfg->m_SourceWidth * m_pcEncCfg->m_SourceHeight ) / ( 3840.0 * 2160.0 );
          const int firstPassSliceQP = it->qp;
          const int secondPassBaseQP = ( m_pcEncCfg->m_LookAhead ? ( m_pcEncCfg->m_QP + getBaseQP() ) >> 1 : m_pcEncCfg->m_QP );
          double d = (double)it->targetBits, baseQP;
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
          encRcPic->visActSteady  = visAct;
          encRcPic->tmpTargetBits = it->targetBits;

          if ( it->refreshParameters )
          {
            encRCSeq->qpCorrection[ frameLevel ] = ( ( it->poc == 0 ) && ( d < it->numBits ) ? std::max( -1.0 * visAct / double( 1 << ( encRCSeq->bitDepth - 3 ) ), 1.0 - it->numBits / d ) : ( it->poc <= m_pcEncCfg->m_GOPSize && ( secondPassBaseQP < 32 || !m_pcEncCfg->m_LookAhead ) ? 0.0625 * ( 32 - secondPassBaseQP ) : 0.0 ) );
            if ( !m_pcEncCfg->m_LookAhead )
            {
              encRCSeq->actualBitCnt[ frameLevel ] = encRCSeq->targetBitCnt[ frameLevel ] = 0;
            }
            else if ( encRCSeq->framesCoded >= encRCSeq->intraPeriod )
            {
              if ( frameLevel == 0 ) encRCSeq->lastIntraBitsSaved = ( encRcSeq->estimatedBitUsage > encRcSeq->bitsUsed ); // cut GOP
              if ( frameLevel == 1 && encRCSeq->lastIntraBitsSaved && encRcSeq->estimatedBitUsage < encRcSeq->bitsUsed ) // next GOP
              {
                encRCSeq->bitsUsedQPLimDiff += ( encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed ) >> 1;  // relax rate constraints
              }
            }
            encRcPic->refreshParams = true;
          }
          CHECK( slice->TLayer >= 7, "analyzed RC frame must have TLayer < 7" );

          // calculate the difference of under or overspent bits and adjust the current target bits based on the GOP and frame ratio
          d = std::max( 1.0, encRcPic->tmpTargetBits + ( encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed ) * 0.5 * it->frameInGopRatio );

          // try to reach target rate less aggressively in first coded frames, prevents temporary very low quality during second GOP
          if ( it->isStartOfGop && ( it->poc == m_pcEncCfg->m_GOPSize || ( it->isIntra && m_pcEncCfg->m_IntraPeriod >= 2 * m_pcEncCfg->m_GOPSize && ( it->poc < flushPOC || flushPOC < 0 ) ) ) )
          {
            const double fac = ( it->poc > 2 * m_pcEncCfg->m_GOPSize ? -0.25 : ( it->poc == 2 * m_pcEncCfg->m_GOPSize ? 0.0 : 0.25 ) );

            d = std::max( 1.0, d - ( encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed ) * fac * it->frameInGopRatio );
          }
          // try to hit target rate more aggressively in last coded frames, lambda/QP clipping below will ensure smooth value change
          if ( it->poc >= flushPOC && flushPOC >= 0 )
          {
            if ( m_pcEncCfg->m_LookAhead && encRcSeq->bitsUsedIn1stPass > 0 )
            {
              const double bp1pf = (double)encRCSeq->bitsUsedIn1stPass / std::max( 1u, encRCSeq->framesCoded ); // first coding pass
              const double ratio = (double)encRCSeq->targetRate / ( encRCSeq->frameRate * bp1pf ); // targeted 2nd-to-1st pass ratio
              d = std::max( 0.0, d - ( encRCSeq->estimatedBitUsage - encRCSeq->bitsUsed ) * 0.5 * it->frameInGopRatio );
              d = std::max( 1.0, d + ( encRCSeq->bitsUsedIn1stPass * ratio - encRCSeq->bitsUsed ) * it->frameInGopRatio );
            }
            else
            {
              d = std::max( 1.0, d + ( encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed ) * 0.5 * it->frameInGopRatio );
            }
          }
          else if ( d > dLimit * encRcPic->tmpTargetBits )
          {
            d = encRcPic->tmpTargetBits * dLimit; // prevent large spendings after easy scenes
          }
          else if ( d * dLimit < encRcPic->tmpTargetBits )
          {
            d = encRcPic->tmpTargetBits / dLimit; // prevent small spendings after hard scenes
          }
          encRcPic->targetBits = int( d + 0.5 );

          baseQP = updateQPstartModelVal() + log (sqrOfResRatio) / log (2.0); // GOP's QPstart
          d /= (double)it->numBits;
          d = firstPassSliceQP - ( 105.0 / 128.0 ) * sqrt( (double)std::max( 1, firstPassSliceQP ) ) * log( d ) / log( 2.0 );
          sliceQP = int( 0.5 + d + 0.5 * std::max( 0.0, baseQP - d ) + encRCSeq->qpCorrection[ frameLevel ] );

          encRcPic->clipTargetQP( getPicList(), ( m_pcEncCfg->m_LookAhead ? getBaseQP() : secondPassBaseQP + ( it->isIntra ? m_pcEncCfg->m_intraQPOffset : 0 ) ), sliceQP );
          lambda = it->lambda * pow( 2.0, double( sliceQP - firstPassSliceQP ) / 3.0 );
          lambda = Clip3( encRcSeq->minEstLambda, encRcSeq->maxEstLambda, lambda );

          if ( m_pcEncCfg->m_LookAhead )
          {
            encRCSeq->bitsUsedIn1stPass += it->numBits;
            if ( frameLevel == 1 ) encRCSeq->lastIntraBitsSaved = false;

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

}

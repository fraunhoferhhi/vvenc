/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     RateCtrl.h
    \brief    Rate control manager class
*/

#ifndef __ENCRATECTRL__
#define __ENCRATECTRL__

#pragma once

#include "CommonLib/CommonDef.h"
#include "Utilities/MsgLog.h"

#include "vvenc/vvencCfg.h"

#include <vector>
#include <algorithm>
#include <list>
#include <fstream>

namespace vvenc {
  struct Picture;

  struct TRCPassStats
  {
    TRCPassStats( const int _poc, const int _qp, const double _lambda, const uint16_t _visActY,
                  const uint32_t _numBits, const double _psnrY, const bool _isIntra, const int _tempLayer,
                  const bool _isStartOfIntra, const bool _isStartOfGop, const int _gopNum, const SceneType _scType,
                  int _spVisAct, const uint16_t _motionEstError, const uint8_t _minNoiseLevels[ QPA_MAX_NOISE_LEVELS ] ) :
                  poc( _poc ), qp( _qp ), lambda( _lambda ), visActY( _visActY ),
                  numBits( _numBits ), psnrY( _psnrY ), isIntra( _isIntra ), tempLayer( _tempLayer ),
                  isStartOfIntra( _isStartOfIntra ), isStartOfGop( _isStartOfGop ), gopNum( _gopNum ), scType( _scType ),
                  spVisAct(_spVisAct), motionEstError( _motionEstError ),
                  isNewScene( false ), refreshParameters( false ), frameInGopRatio( -1.0 ), targetBits( 0 ), addedToList( false )
                  {
                    std::memcpy( minNoiseLevels, _minNoiseLevels, sizeof( minNoiseLevels ) );
                  }
    TRCPassStats() {};
    int       poc                                     { 0 };
    int       qp                                      { 0 };
    double    lambda                                  { 0.0 };
    uint16_t  visActY                                 { 0 };
    uint32_t  numBits                                 { 0 };
    double    psnrY                                   { 0.0 };
    bool      isIntra                                 { false };
    int       tempLayer                               { 0 };
    bool      isStartOfIntra                          { false };
    bool      isStartOfGop                            { false };
    int       gopNum                                  { 0 };
    SceneType scType                                  { SCT_NONE };
    int       spVisAct                                { 0 };
    uint16_t  motionEstError                          { 0 };
    uint8_t   minNoiseLevels[ QPA_MAX_NOISE_LEVELS ]  {};
    bool      isNewScene                              { false };
    bool      refreshParameters                       { false };
    double    frameInGopRatio                         { 0.0 };
    int       targetBits                              { 0 };
    bool      addedToList                             { false };
  };

  class EncRCSeq
  {
  public:
    EncRCSeq();
    ~EncRCSeq();

    void create( bool twoPassRC, bool lookAhead, int targetBitrate, int maxBitrate, double frRate, int intraPer, int GOPSize, int bitDpth, std::list<TRCPassStats> &firstPassStats );
    void destroy();
    void updateAfterPic (const int actBits, const int tgtBits);

    bool            twoPass;
    bool            isLookAhead;
    bool            isIntraGOP;
    bool            isRateSavingMode;
    double          frameRate;
    int             targetRate;
    int             maxGopRate;
    int             gopSize;
    unsigned        intraPeriod;
    bool            scRelax;
    int             bitDepth;
    int64_t         bitsUsed;
    int64_t         bitsUsedQPLimDiff;
    int64_t         estimatedBitUsage;
    double          rateBoostFac;
    double          qpCorrection[8];
    uint64_t        actualBitCnt[8];
    uint64_t        targetBitCnt[8];
    int             lastAverageQP;
    int             lastIntraQP;
    double          lastIntraSM; // temporal stationarity measure; 1: highly stationary (static), 0: highly nonstationary (irregular)
    std::list<TRCPassStats> firstPassData;
    double          minEstLambda;
    double          maxEstLambda;
  };

  class EncRCPic
  {
  public:
    EncRCPic();
    ~EncRCPic();

    void   create( EncRCSeq* encRCSeq, int frameLevel, int framePoc );
    void   destroy();
    void   clipTargetQP (std::list<EncRCPic*>& listPreviousPictures, const int baseQP, const int refrIncrFac, const int maxTL, const double resRatio, int &qp, int* qpAvg);
    void   updateAfterPicture (const int picActualBits, const int averageQP);
    void   addToPictureList( std::list<EncRCPic*>& listPreviousPictures );

    int     targetBits;
    int     tmpTargetBits;
    int     poc;
    bool    refreshParams;
    uint16_t visActSteady;

  protected:
    EncRCSeq* encRCSeq;
    int       frameLevel;    // non-I: TLayer + 1
    int16_t   picQP;         // in integer form
    uint16_t  picBits;       // after 2nd pass
  };

  class RateCtrl
  {
  public:
    RateCtrl(MsgLog& logger);
    ~RateCtrl();

    void init( const VVEncCfg& encCfg );
    void destroy();
    int  getBaseQP();
    void setRCPass (const VVEncCfg& encCfg, const int pass, const char* statsFName);
    void addRCPassStats( const int poc, const int qp, const double lambda, const uint16_t visActY,
                         const uint32_t numBits, const double psnrY, const bool isIntra, const uint32_t tempLayer,
                         const bool isStartOfIntra, const bool isStartOfGop, const int gopNum, const SceneType scType,
                         int spVisAct, const uint16_t motEstError, const uint8_t minNoiseLevels[QPA_MAX_NOISE_LEVELS] );
    void setRCRateSavingState( const int maxRate );
    void processFirstPassData( const bool flush, const int poc = -1 );
    void updateAfterPicEncRC( const Picture* pic );
    void initRateControlPic( Picture& pic, Slice* slice, int& qp, double& finalLambda );

    std::list<EncRCPic*>&    getPicList()        { return m_listRCPictures; }
    std::list<TRCPassStats>& getFirstPassStats() { return m_listRCFirstPassStats; }
    std::vector<uint8_t>*    getIntraPQPAStats() { return &m_listRCIntraPQPAStats; }
    const uint8_t*           getMinNoiseLevels() { return m_minNoiseLevels; }
    int                      lastPOCInCache()    { CHECK(m_firstPassCache.empty(), "Accessing empty cache"); return m_firstPassCache.back().poc; }

    std::list<EncRCPic*>    m_listRCPictures;
    EncRCSeq*               encRCSeq;
    EncRCPic*               encRCPic;
    int                     flushPOC;
    int                     rcPass;
    bool                    rcIsFinalPass;
    const VVEncCfg*         m_pcEncCfg;

  protected:
    MsgLog&                 msg;

    void xProcessFirstPassData( const bool flush, const int poc );
    void storeStatsData( TRCPassStats statsData );
    double getAverageBitsFromFirstPass();
    void detectSceneCuts();
    void processGops();
    void updateMotionErrStatsGop( const bool flush, const int poc );
    double getLookAheadBoostFac ( const int thresholdDivisor );
    double updateQPstartModelVal();
#ifdef VVENC_ENABLE_THIRDPARTY_JSON
    void openStatsFile( const std::string& name );
    void writeStatsHeader();
    void readStatsHeader();
    void readStatsFile();
#endif
    void adjustStatsDownsample();

  private:
    std::list<TRCPassStats> m_listRCFirstPassStats;
    std::list<TRCPassStats> m_firstPassCache;
    std::vector<uint8_t>    m_listRCIntraPQPAStats;
#ifdef VVENC_ENABLE_THIRDPARTY_JSON
    std::fstream            m_rcStatsFHandle;
    int                     m_pqpaStatsWritten;
#endif
    int                     m_numPicStatsTotal;
    int                     m_numPicAddedToList;
    int                     m_updateNoisePoc;
    bool                    m_resetNoise;
    uint8_t                 m_gopMEErrorCBufIdx;
    uint16_t                m_maxPicMotionError;
    uint16_t                m_gopMEErrorCBuf[ QPA_MAX_NOISE_LEVELS ];
    uint8_t                 m_minNoiseLevels[ QPA_MAX_NOISE_LEVELS ];
    TRCPassStats            m_tempDownSamplStats[ VVENC_MAX_TLAYER + 1 ];
  };

}
#endif

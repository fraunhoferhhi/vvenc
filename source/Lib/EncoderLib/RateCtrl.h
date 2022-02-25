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
                  uint32_t _numBits, double _psnrY, bool _isIntra, int _tempLayer ) :
                  poc( _poc ), qp( _qp ), lambda( _lambda ), visActY( _visActY ),
                  numBits( _numBits ), psnrY( _psnrY ), isIntra( _isIntra ),
                  tempLayer( _tempLayer ),
                  isNewScene( false ), refreshParameters( false ), frameInGopRatio( -1.0 ), targetBits( 0 ), addedToList( false )
                  {}
    int       poc;
    int       qp;
    double    lambda;
    uint16_t  visActY;
    uint32_t  numBits;
    double    psnrY;
    bool      isIntra;
    int       tempLayer;
    bool      isNewScene;
    bool      refreshParameters;
    double    frameInGopRatio;
    int       targetBits;
    bool      addedToList;
  };

  class EncRCSeq
  {
  public:
    EncRCSeq();
    ~EncRCSeq();

    void create( bool twoPass, bool lookAhead, int targetBitrate, int frameRate, int intraPeriod, int GOPSize, int bitDepth, std::list<TRCPassStats> &firstPassData );
    void destroy();
    void updateAfterPic (const int actBits, const int tgtBits);
    void getTargetBitsFromFirstPass (const int poc, int &targetBits, double &frameVsGopRatio, bool &isNewScene, bool &refreshParameters);

    bool            twoPass;
    bool            isLookAhead;
    int             framesCoded;
    int             targetRate;
    int             frameRate;
    int             gopSize;
    int             intraPeriod;
    int             bitDepth;
    int64_t         bitsUsed;
    int64_t         bitsUsedIn1stPass;
    int64_t         estimatedBitUsage;
    double          qpCorrection[8];
    uint64_t        actualBitCnt[8];
    unsigned        currFrameCnt[8];
    uint64_t        targetBitCnt[8];
    int             lastIntraQP;
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
    void   clipTargetQP (std::list<EncRCPic*>& listPreviousPictures, int &qp);
    void   updateAfterPicture (const int actualTotalBits, const int averageQP);
    void   addToPictureList( std::list<EncRCPic*>& listPreviousPictures );

    int     targetBits;
    int     tmpTargetBits;
    int     picQPOffsetQPA;
    int     poc;
    double  picLambdaOffsetQPA;

  protected:
    int xEstPicTargetBits( EncRCSeq* encRCSeq, int frameLevel );

    EncRCSeq* encRCSeq;
    int     frameLevel;
    int     picActualBits;   // the whole picture, including header
    int     picQP;           // in integer form
    bool    isNewScene;
    bool    refreshParams;
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
    void addRCPassStats (const int poc, const int qp, const double lambda, const uint16_t visActY,
                         const uint32_t numBits, const double psnrY, const bool isIntra, const int tempLayer);
    void processFirstPassData (const bool flush);
    void processFirstPassData (const bool flush, int poc);
    void processGops();
    double getAverageBitsFromFirstPass();
    void detectSceneCuts();
    void xUpdateAfterPicRC( const Picture* pic );
    void initRateControlPic( Picture& pic, Slice* slice, int& qp, double& finalLambda );

    std::list<EncRCPic*>& getPicList() { return m_listRCPictures; }
    std::list<TRCPassStats>& getFirstPassStats() { return m_listRCFirstPassStats; }
    std::vector<uint8_t>* getIntraPQPAStats() { return &m_listRCIntraPQPAStats; }

    std::list<EncRCPic*>    m_listRCPictures;
    EncRCSeq*   encRCSeq;
    EncRCPic*   encRCPic;
    std::mutex  rcMutex;
    int         flushPOC;
    int         rcPass;
    bool        rcIsFinalPass;
    const VVEncCfg*         m_pcEncCfg;

  protected:
    MsgLog&                 msg;
    void storeStatsData( const TRCPassStats& statsData );
#ifdef VVENC_ENABLE_THIRDPARTY_JSON
    void openStatsFile( const std::string& name );
    void writeStatsHeader();
    void readStatsHeader();
    void readStatsFile();
#endif

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
  };

}
#endif

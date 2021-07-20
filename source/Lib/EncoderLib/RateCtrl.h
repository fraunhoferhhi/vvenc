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
/** \file     RateCtrl.h
    \brief    Rate control manager class
*/

#ifndef __ENCRATECTRL__
#define __ENCRATECTRL__

#pragma once

#include "CommonLib/CommonDef.h"

#include "vvenc/vvencCfg.h"

#include <vector>
#include <algorithm>
#include <list>

namespace vvenc {
  struct Picture;

  struct TRCLCU
  {
    double  lambda;
    double  actualSSE;
    double  actualMSE;
    int     numberOfPixel;
  };

  struct TRCParameter
  {
    double  alpha;
    double  beta;
    double  skipRatio;
    int     validPix;
  };

  struct TRCPassStats
  {
    TRCPassStats( const int _poc, const int _qp, const double _lambda, const uint16_t _visActY,
                  uint32_t _numBits, double _psnrY, bool _isIntra, int _tempLayer ) :
                  poc( _poc ), qp( _qp ), lambda( _lambda ), visActY( _visActY ),
                  numBits( _numBits ), psnrY( _psnrY ), isIntra( _isIntra ), tempLayer( _tempLayer + int( !_isIntra ) ),
                  isNewScene( false ), refreshParameters( false ), frameInGopRatio( -1.0 ), targetBits( 0 )
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
  };

  class EncRCSeq
  {
  public:
    EncRCSeq();
    ~EncRCSeq();

    void create( bool twoPass, int totFrames, int targetBitrate, int frameRate, int intraPeriod, int GOPSize, int picWidth, int picHeight, int LCUWidth, int LCUHeight, int numberOfLevel, int adaptiveBit, std::list<TRCPassStats> &firstPassData );
    void destroy();
    void initBitsRatio( int bitsRatio[] );
    void initGOPID2Level( int GOPID2Level[] );
    void initPicPara( TRCParameter* picPara = NULL );    // NULL to initial with default value
    void updateAfterPic( int bits, int tgtBits );
    void setAllBitRatio( double basicLambda, double* equaCoeffA, double* equaCoeffB );
    int  getLeftAverageBits() { CHECK( !( framesLeft > 0 ), "No frames left" ); return (int)( bitsLeft / framesLeft ); }
    void getTargetBitsFromFirstPass (const int poc, int &targetBits, double &frameVsGopRatio, bool &isNewScene, bool &refreshParameters);
    void clipRcAlpha( double& alpha );

  public:
    bool            twoPass;
    int             fppParFrames;
    int             totalFrames;
    int             targetRate;
    int             frameRate;
    int             gopSize;
    int             picWidth;
    int             picHeight;
    int             lcuWidth;
    int             lcuHeight;
    int             averageBits;
    int             intraPeriod;
    int             numberOfPixel;
    int             numberOfLCU;
    int             framesCoded;
    int             framesLeft;
    int             adaptiveBits;
    int             bitDepth;
    int64_t         bitsUsed;
    int64_t         estimatedBitUsage;
    double          lastLambda;
    double          qpCorrection[8];
    uint64_t        actualBitCnt[8];
    uint64_t        targetBitCnt[8];
    double          lastIntraLambda;
    int             lastIntraQP;
    TRCParameter*   picParam;
    int*            bitsRatio;
    int*            gopID2Level;
    std::list<TRCPassStats> firstPassData;

  private:
    int             numberOfLevel;
    int64_t         targetBits;
    int64_t         bitsLeft;
  };

  class EncRCGOP
  {
  public:
    EncRCGOP();
    ~EncRCGOP();

    void create( EncRCSeq* encRCSeq, int numPic );
    void destroy();
    void updateAfterPicture( int bitsCost );

  private:
    int    xEstGOPTargetBits( EncRCSeq* encRCSeq, int GOPSize );
    void   xCalEquaCoeff( EncRCSeq* encRCSeq, double* lambdaRatio, double* equaCoeffA, double* equaCoeffB, int GOPSize );
    double xSolveEqua( EncRCSeq* encRCSeq, double targetBpp, double* equaCoeffA, double* equaCoeffB, int GOPSize );

  public:
    int     numPics;
    int     targetBits;
    int     picsLeft;
    int     bitsLeft;
    int*    picTargetBitInGOP;
    double  minEstLambda;
    double  maxEstLambda;

  private:
    EncRCSeq* encRcSeq;
  };

  class EncRCPic
  {
  public:
    EncRCPic();
    ~EncRCPic();

    void create( EncRCSeq* encRCSeq, EncRCGOP* encRCGOP, int frameLevel, int framePoc, int frameRcIdxInGop, std::list<EncRCPic*>& listPreviousPictures );
    void destroy();

    void   calCostSliceI( Picture* pic );
    int    estimatePicQP( double lambda, std::list<EncRCPic*>& listPreviousPictures );
    void   clipQpFrameRc( std::list<EncRCPic*>& listPreviousPictures, int &QP );
    void   clipTargetQP (std::list<EncRCPic*>& listPreviousPictures, int &qp);
    int    getRefineBitsForIntra( int orgBits );
    double calculateLambdaIntra( double alpha, double beta, double MADPerPixel, double bitsPerPixel );
    double estimatePicLambda( std::list<EncRCPic*>& listPreviousPictures, bool isIRAP );
    void   clipLambdaFrameRc( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale );
    void   updateAlphaBetaIntra( double& alpha, double& beta );
    void   updateAfterCTU( int LCUIdx, int bits, double lambda );
    void   updateAfterPicture( int actualHeaderBits, int actualTotalBits, double averageQP, double averageLambda, bool isIRAP );
    void   clipRcBeta( double& beta );
    void   addToPictureList( std::list<EncRCPic*>& listPreviousPictures );
    void   calPicMSE();

  private:
    int xEstPicTargetBits( EncRCSeq* encRCSeq, EncRCGOP* encRCGOP, int frameLevel );
    int xEstPicHeaderBits( std::list<EncRCPic*>& listPreviousPictures, int frameLevel );

  public:
    TRCLCU* lcu;
    int     targetBits;
    int     tmpTargetBits;
    int     bitsLeft;
    int     numberOfLCU;
    int     picQPOffsetQPA;
    int     poc;
    int     rcIdxInGop;
    double  picLambdaOffsetQPA;
    double  finalLambda;
    int     estimatedBits;
  
  private:
    EncRCSeq* encRCSeq;
    EncRCGOP* encRCGOP;

    int     frameLevel;
    int     numberOfPixel;
    int     estHeaderBits;
    int     picActualHeaderBits;
    int     picActualBits;          // the whole picture, including header
    int     picQP;                  // in integer form
    int     validPixelsInPic;
    double  totalCostIntra;
    double  picLambda;
    double  picMSE;
    bool    isNewScene;
    bool    refreshParams;
  };

  class RateCtrl
  {
  public:
    RateCtrl();
    ~RateCtrl();

    void init( int totFrames, int targetBitrate, int frameRate, int intraPeriod, int GOPSize, int picWidth, int picHeight,
               int LCUWidth, int LCUHeight, int bitDepth, const vvencGOPEntry GOPList[ VVENC_MAX_GOP ], int maxParallelFrames );
    void destroy();
    void initRCGOP (const int numberOfPictures);
    void destroyRCGOP();

    void setRCPass (const int pass, const int maxPass);
    void addRCPassStats (const int poc, const int qp, const double lambda, const uint16_t visActY,
                         const uint32_t numBits, const double psnrY, const bool isIntra, const int tempLayer);
    void processFirstPassData (const int secondPassBaseQP);
    void processGops (const int secondPassBaseQP);
    uint64_t getTotalBitsInFirstPass();
    void detectNewScene();
    void adaptToSceneChanges();

    std::list<EncRCPic*>& getPicList() { return m_listRCPictures; }
    std::list<TRCPassStats>& getFirstPassStats() { return m_listRCFirstPassStats; }
    std::vector<uint8_t>* getIntraPQPAStats() { return &m_listRCIntraPQPAStats; }

  public:
    std::list<EncRCPic*>    m_listRCPictures;
    EncRCSeq*   encRCSeq;
    EncRCGOP*   encRCGOP;
    EncRCPic*   encRCPic;
    std::mutex  rcMutex;
    int         flushPOC;
    int         rcPass;
    int         rcMaxPass;
    bool        rcIsFinalPass;

  private:
    std::list<TRCPassStats> m_listRCFirstPassStats;
    std::vector<uint8_t>    m_listRCIntraPQPAStats;
  };
}
#endif

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

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
    double  bitWeight;
    double  costIntra;
    double  actualSSE;
    double  actualMSE;
    int     QP;     // QP of skip mode is set to RC_INVALID_QP_VALUE
    int     actualBits;
    int     targetBits;
    int     numberOfPixel;
    int     targetBitsLeft;
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
    TRCPassStats( int _poc, int _qp, uint32_t _numBits, double _yPsnr, double _uPsnr, double _vPsnr, bool _isIntra, int _tempLayer ) : poc( _poc ), qp( _qp ), numBits( _numBits ), yPsnr( _yPsnr ), uPsnr( _uPsnr ), vPsnr( _vPsnr ), isIntra( _isIntra ), tempLayer( _tempLayer ), isNewScene( false ), frameInGopRatio( -1.0 ), gopBitsVsBitrate( -1.0 ), scaledBits( double( numBits ) ), targetBits( 0 ), estAlpha() {}
    int       poc;
    int       qp;
    uint32_t  numBits;
    double    yPsnr;
    double    uPsnr;
    double    vPsnr;
    bool      isIntra;
    int       tempLayer;

    bool      isNewScene;
    double    frameInGopRatio;
    double    gopBitsVsBitrate;
    double    scaledBits;
    int       targetBits;
    double    estAlpha[ 7 ];
  };

  class EncRCSeq
  {
  public:
    EncRCSeq();
    ~EncRCSeq();

    void create( int RCMode, bool twoPass, int totFrames, int targetBitrate, int frameRate, int intraPeriod, int GOPSize, int picWidth, int picHeight, int LCUWidth, int LCUHeight, int numberOfLevel, bool useLCUSeparateModel, int adaptiveBit, std::list<TRCPassStats> &firstPassData );
    void destroy();
    void initBitsRatio( int bitsRatio[] );
    void initGOPID2Level( int GOPID2Level[] );
    void initPicPara( TRCParameter* picPara = NULL );    // NULL to initial with default value
    void initLCUPara( TRCParameter** LCUPara = NULL );    // NULL to initial with default value
    void updateAfterPic( int bits, int tgtBits );
    void setAllBitRatio( double basicLambda, double* equaCoeffA, double* equaCoeffB );
    void setQpInGOP( int gopId, int gopQp, int &qp );
    bool isQpResetRequired( int gopId );
    int  getLeftAverageBits() { CHECK( !( framesLeft > 0 ), "No frames left" ); return (int)( bitsLeft / framesLeft ); }
    void getTargetBitsFromFirstPass( int poc, int &targetBits, double &gopVsBitrateRatio, double &frameVsGopRatio, bool &isNewScene, double alpha[] );

  public:
    int             rcMode;
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
    double          bitUsageRatio;
    double          lastLambda;
    bool            useLCUSeparateModel;
    TRCParameter*   picParam;
    TRCParameter**  lcuParam;
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
    int     gopQP;
    int     idealTargetGOPBits;
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
    void   clipQpConventional( std::list<EncRCPic*>& listPreviousPictures, int &QP );
    void   clipQpFrameRc( std::list<EncRCPic*>& listPreviousPictures, int &QP );
    void   clipQpGopRc( std::list<EncRCPic*>& listPreviousPictures, int &QP );
    void   clipQpTwoPass( std::list<EncRCPic*>& listPreviousPictures, int &QP );
    int    getRefineBitsForIntra( int orgBits );
    double calculateLambdaIntra( double alpha, double beta, double MADPerPixel, double bitsPerPixel );
    double estimatePicLambda( std::list<EncRCPic*>& listPreviousPictures, bool isIRAP );
    void   clipLambdaConventional( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale );
    void   clipLambdaFrameRc( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale );
    void   clipLambdaGopRc( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale );
    void   clipLambdaTwoPass( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale );
    void   updateAlphaBetaIntra( double *alpha, double *beta );
    double getLCUTargetBpp( bool isIRAP, const int ctuRsAddr );
    double getLCUEstLambdaAndQP( double bpp, int clipPicQP, int *estQP, const int ctuRsAddr );
    double getLCUEstLambda( double bpp, const int ctuRsAddr );
    int    getLCUEstQP( double lambda, int clipPicQP, const int ctuRsAddr );
    void   updateAfterCTU( int LCUIdx, int bits, int QP, double lambda, double skipRatio, bool updateLCUParameter = true );
    void   updateAfterPicture( int actualHeaderBits, int actualTotalBits, double averageQP, double averageLambda, bool isIRAP );
    void   getLCUInitTargetBits();
    double clipRcAlpha( const int bitdepth, const double alpha );
    double clipRcBeta( const double beta );
    void   addToPictureList( std::list<EncRCPic*>& listPreviousPictures );
    double calAverageQP();
    double calAverageLambda();

  private:
    int xEstPicTargetBits( EncRCSeq* encRCSeq, EncRCGOP* encRCGOP );
    int xEstPicHeaderBits( std::list<EncRCPic*>& listPreviousPictures, int frameLevel );

  public:
    TRCLCU* lcu;
    int     targetBits;
    int     tmpTargetBits;
    int     bitsLeft;
    int     numberOfLCU;
    int     lcuLeft;
    int     picQPOffsetQPA;
    int     poc;
    int     rcIdxInGop;
    double  picLambdaOffsetQPA;
    double  picEstLambda;
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
    double  remainingCostIntra;
    double  picLambda;
    double  picMSE;
    bool    isNewScene;
  };

  class RateCtrl
  {
  public:
    RateCtrl();
    ~RateCtrl();

    void init( int RCMode, int totFrames, int targetBitrate, int frameRate, int intraPeriod, int GOPSize, int picWidth, int picHeight, int LCUWidth, int LCUHeight, int bitDepth, int keepHierBits, bool useLCUSeparateModel, const GOPEntry GOPList[ MAX_GOP ], int maxParallelFrames );
    void destroy();
    void initRCPic( int frameLevel, int framePoc, int frameRcIdxInGop );
    void initRCGOP( int numberOfPictures );
    void destroyRCGOP();

    void setRCPass( int pass, int maxPass );
    void addRCPassStats( int poc, int qp, uint32_t numBits, double yPsnr, double uPsnr, double vPsnr, bool isIntra, int tempLayer );
    void processFirstPassData( const unsigned sizeInCtus );
    void estimateAlphaFirstPass( int numOfLevels, int startPoc, int pocRange, double *alphaEstimate );
    void processGops();
    void scaleGops( std::vector<double> &scaledBits, std::vector<int> &gopBits, double &actualBitrateAfterScaling );
    int64_t getTotalBitsInFirstPass();
    void detectNewScene();

    std::list<EncRCPic*>& getPicList() { return m_listRCPictures; }
    std::list<TRCPassStats>& getFirstPassStats() { return m_listRCFirstPassStats; }
    std::vector<uint8_t>* getIntraPQPAStats() { return &m_listRCIntraPQPAStats; }

  public:
    std::list<EncRCPic*>    m_listRCPictures;
    EncRCSeq*   encRCSeq;
    EncRCGOP*   encRCGOP;
    EncRCPic*   encRCPic;
    std::mutex  rcMutex;
    int         rcQP;
    int         rcPQPAOffset;
    int         rcPass;
    int         rcMaxPass;
    bool        rcIsFinalPass;

  private:
    std::list<TRCPassStats> m_listRCFirstPassStats;
    std::vector<uint8_t>    m_listRCIntraPQPAStats;
  };
}
#endif

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
/** \file     LegacyRateCtrl.h
\brief    Legacy Rate control manager class
*/

#pragma once

#include "CommonLib/CommonDef.h"

#include "vvenc/vvencCfg.h"
#include "RateCtrl.h"

#include <vector>
#include <algorithm>
#include <list>

namespace vvenc {

  struct LegacyTRCLCU
  {
    double  lambda;
    double  actualSSE;
    double  actualMSE;
    int     numberOfPixel;
  };

  struct LegacyTRCParam
  {
    double  alpha;
    double  beta;
    double  skipRatio;
    int     validPix;
  };

  class LegacyEncRCSeq : public EncRCSeq
  {
  public:
    LegacyEncRCSeq();
    ~LegacyEncRCSeq();

    void create( bool twoPass, int totFrames, int targetBitrate, int frameRate, int intraPeriod, int GOPSize, int picWidth, int picHeight, int LCUWidth, int LCUHeight, int numberOfLevel, int adaptiveBit, int bitDepth, std::list<TRCPassStats> &firstPassData );
    void destroy();
    void initBitsRatio( int bitsRatio[] );
    void initGOPID2Level( int GOPID2Level[] );
    void initPicPara( LegacyTRCParam* picPara = NULL );    // NULL to initial with default value
    void setAllBitRatio( double basicLambda, double* equaCoeffA, double* equaCoeffB );
    void updateAfterPic( int bits, int tgtBits );
    int  getLeftAverageBits() { CHECK( !( framesLeft > 0 ), "No frames left" ); return (int)( bitsLeft / framesLeft ); }
    void clipRcAlpha( double& alpha );

    int             totalFrames;
    int             fppParFrames;
    int             picWidth;
    int             picHeight;
    int             lcuWidth;
    int             lcuHeight;
    int             numberOfPixel;
    int             numberOfLCU;
    int             averageBits;
    int             framesLeft;
    int             adaptiveBits;
    double          lastLambda;
    LegacyTRCParam* picParam;
    int*            bitsRatio;
    int*            gopID2Level;

  private:
    int             numberOfLevel;
    int64_t         targetBits;
    int64_t         bitsLeft;
  };

  class LegacyEncRCGOP
  {
  public:
    LegacyEncRCGOP();
    ~LegacyEncRCGOP();

    void create( LegacyEncRCSeq* encRCSeq, int numPic );
    void destroy();
    void updateAfterPicture( int bitsCost );

    int     numPics;
    int     targetBits;
    int     picsLeft;
    int     bitsLeft;
    int*    picTargetBitInGOP;
    double  minEstLambda;
    double  maxEstLambda;

  private:
    int    xEstGOPTargetBits( LegacyEncRCSeq* encRCSeq, int GOPSize );
    void   xCalEquaCoeff( LegacyEncRCSeq* encRCSeq, double* lambdaRatio, double* equaCoeffA, double* equaCoeffB, int GOPSize );
    double xSolveEqua( LegacyEncRCSeq* encRCSeq, double targetBpp, double* equaCoeffA, double* equaCoeffB, int GOPSize );

    LegacyEncRCSeq* encRcSeq;
  };

  class LegacyEncRCPic : public EncRCPic
  {
  public:
    LegacyEncRCPic();
    ~LegacyEncRCPic();

    void   create( LegacyEncRCSeq* encRCSeq, LegacyEncRCGOP* encRCGOP, int frameLevel, int framePoc, int frameRcIdxInGop, std::list<EncRCPic*>& listPreviousPictures );
    void   destroy();
    void   calCostSliceI( Picture* pic );
    int    estimatePicQP( double lambda, std::list<EncRCPic*>& listPreviousPictures );
    void   clipQpFrameRc( std::list<EncRCPic*>& listPreviousPictures, int &QP );
    int    getRefineBitsForIntra( int orgBits );
    double calculateLambdaIntra( double alpha, double beta, double MADPerPixel, double bitsPerPixel );
    double estimatePicLambda( std::list<EncRCPic*>& listPreviousPictures, bool isIRAP );
    void   clipLambdaFrameRc( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale );
    void   updateAlphaBetaIntra( double& alpha, double& beta );
    void   updateAfterCTU( int LCUIdx, int bits, double lambda );
    void   updateAfterPicture( int actualHeaderBits, int actualTotalBits, double averageQP, double averageLambda, bool isIRAP );
    void   clipRcBeta( double& beta );
    void   calPicMSE();
    void   updateCtuMSE( const unsigned int ctuAddress, const double distortion );

    LegacyTRCLCU* lcu;
    int     bitsLeft;
    int     numberOfLCU;
    int     rcIdxInGop;
    double  finalLambda;
    int     estimatedBits;

  protected:
    int xEstPicTargetBits( LegacyEncRCSeq* encRCSeq, LegacyEncRCGOP* encRCGOP, int frameLevel );
    int xEstPicHeaderBits( std::list<EncRCPic*>& listPreviousPictures, int frameLevel );

    LegacyEncRCSeq* encRCSeq;
    LegacyEncRCGOP* encRCGOP;
    int     numberOfPixel;
    int     estHeaderBits;
    int     picActualHeaderBits;
    int     validPixelsInPic;
    double  totalCostIntra;
    double  picLambda;
    double  picMSE;
  };

  class LegacyRateCtrl : public RateCtrl
  {
  public:
    LegacyRateCtrl();
    ~LegacyRateCtrl();

    void init( const VVEncCfg& encCfg );
    void destroy();
    void initRCGOP( const int numberOfPictures );
    void destroyRCGOP();
    void xUpdateAfterPicRC( const Picture* pic );
    void xUpdateAfterCtuRC( const Slice* slice, const int numberOfWrittenBits, const int ctuRsAddr, std::mutex* mutex, const double lambda );
    void initRateControlPic( Picture& pic, Slice* slice, int& qp, double& finalLambda );
    void setFinalLambda( const double lambda );

    LegacyEncRCSeq*   encRCSeq;
    LegacyEncRCGOP*   encRCGOP;
    LegacyEncRCPic*   encRCPic;
  };
}

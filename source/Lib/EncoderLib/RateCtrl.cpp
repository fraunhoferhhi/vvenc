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


/** \file     RateCtrl.cpp
    \brief    Rate control manager class
*/
#include "RateCtrl.h"
#include "CommonLib/Picture.h"

#include <cmath>

namespace vvenc {

static const int    RC_SMOOTH_WINDOW_SIZE =                           40;
static const int    RC_MAX_PIC_LIST_SIZE =                            64;
static const int    RC_ITERATION_NUM =                                20;
static const int    RC_LAMBDA_PREC =                             1000000;
static const int    RC_GOP_ID_QP_OFFSET[ 6 ] =      { 0, 0, 0, 3, 1, 1 };
static const int    RC_GOP_ID_QP_OFFSET_GOP32[ 7 ] = { 0, 0, 0, 0, 3, 1, 1 };
static const double RC_WEIGHT_PIC_TARGET_BIT_IN_GOP =                0.9;
static const double RC_WEIGHT_PIC_TARGET_BIT_IN_BUFFER =             1.0 - RC_WEIGHT_PIC_TARGET_BIT_IN_GOP;
static const double RC_WEIGHT_HISTORY_LAMBDA =                       0.5;
static const double RC_ALPHA_MIN_VALUE =                            0.05;
static const double RC_ALPHA_MAX_VALUE =                           500.0;
static const double RC_BETA_MIN_VALUE =                             -3.0;
static const double RC_BETA_MAX_VALUE =                             -0.1;
static const double RC_ALPHA =                                    6.7542;
static const double RC_BETA1 =                                    1.2517;
static const double RC_BETA2 =                                    1.7860;

//sequence level
EncRCSeq::EncRCSeq()
{
  twoPass             = false;
  fppParFrames        = 0;
  totalFrames         = 0;
  targetRate          = 0;
  frameRate           = 0;
  targetBits          = 0;
  gopSize             = 0;
  picWidth            = 0;
  picHeight           = 0;
  lcuWidth            = 0;
  lcuHeight           = 0;
  numberOfLevel       = 0;
  intraPeriod         = 0;
  numberOfLCU         = 0;
  averageBits         = 0;
  bitsRatio           = NULL;
  gopID2Level         = NULL;
  picParam            = NULL;
  numberOfPixel       = 0;
  framesCoded         = 0;
  bitsUsed            = 0;
  framesLeft          = 0;
  bitsLeft            = 0;
  estimatedBitUsage   = 0;
  adaptiveBits        = 0;
  lastLambda          = 0.0;
  std::memset (qpCorrection, 0, sizeof (qpCorrection));
  std::memset (actualBitCnt, 0, sizeof (actualBitCnt));
  std::memset (targetBitCnt, 0, sizeof (targetBitCnt));
  lastIntraLambda     = 0.0;
  lastIntraQP         = 0;
  bitDepth            = 0;
}

EncRCSeq::~EncRCSeq()
{
  destroy();
}

void EncRCSeq::create( bool twoPassRC, int totFrames, int targetBitrate, int frRate, int intraPer, int GOPSize, int pictureWidth, int pictureHeight, int LCUWidth, int LCUHeight, int numOfLevel, int adaptiveBit, std::list<TRCPassStats> &firstPassStats )
{
  destroy();
  twoPass             = twoPassRC;
  totalFrames         = totFrames;
  targetRate          = targetBitrate;
  frameRate           = frRate;
  intraPeriod         = intraPer;
  gopSize             = GOPSize;
  picWidth            = pictureWidth;
  picHeight           = pictureHeight;
  lcuWidth            = LCUWidth;
  lcuHeight           = LCUHeight;
  numberOfLevel       = numOfLevel;
  firstPassData       = firstPassStats;

  numberOfPixel = picWidth * picHeight;
  targetBits = (int64_t)totalFrames * (int64_t)targetRate / (int64_t)frameRate;

  averageBits     = (int)( targetRate / frameRate );
  int picWidthInBU  = ( picWidth  % lcuWidth  ) == 0 ? picWidth  / lcuWidth  : picWidth  / lcuWidth  + 1;
  int picHeightInBU = ( picHeight % lcuHeight ) == 0 ? picHeight / lcuHeight : picHeight / lcuHeight + 1;
  numberOfLCU     = picWidthInBU * picHeightInBU;

  bitsRatio = new int[ gopSize ];
  gopID2Level = new int[ gopSize ];
  for ( int i = 0; i < gopSize; i++ )
  {
    bitsRatio[ i ] = 1;
    gopID2Level[ i ] = 1;
  }

  picParam = new TRCParameter[ numberOfLevel ];
  for ( int i = 0; i < numberOfLevel; i++ )
  {
    picParam[ i ].alpha = 0.0;
    picParam[ i ].beta = 0.0;
    picParam[ i ].validPix = -1;
    picParam[ i ].skipRatio = 0.0;
  }

  framesCoded = 0;
  bitsUsed = 0;
  framesLeft = totalFrames;
  bitsLeft   = targetBits;
  estimatedBitUsage = 0;
  lastLambda = 0.0;
  std::memset (qpCorrection, 0, sizeof (qpCorrection));
  std::memset (actualBitCnt, 0, sizeof (actualBitCnt));
  std::memset (targetBitCnt, 0, sizeof (targetBitCnt));
  adaptiveBits = (twoPass ? 0 : adaptiveBit);
}

void EncRCSeq::destroy()
{
  if (bitsRatio != NULL)
  {
    delete[] bitsRatio;
    bitsRatio = NULL;
  }
  if ( gopID2Level != NULL )
  {
    delete[] gopID2Level;
    gopID2Level = NULL;
  }

  if ( picParam != NULL )
  {
    delete[] picParam;
    picParam = NULL;
  }

}

void EncRCSeq::initBitsRatio( int bRatio[] )
{
  for ( int i = 0; i < gopSize; i++ )
  {
    bitsRatio[ i ] = bRatio[ i ];
  }
}

void EncRCSeq::initGOPID2Level( int GOPID2Level[] )
{
  for ( int i = 0; i < gopSize; i++ )
  {
    gopID2Level[ i ] = GOPID2Level[ i ];
  }
}

void EncRCSeq::initPicPara( TRCParameter* picPara )
{
  CHECK( picParam == NULL, "Object does not exist" );

  if ( picPara == NULL )
  {
    for ( int i = 0; i < numberOfLevel; i++ )
    {
      int bitdepthLumaScale = 2 * ( bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( bitDepth ) );
      if ( i > 0 )
      {
        picParam[ i ].alpha = 3.2003 * pow( 2.0, bitdepthLumaScale );
        picParam[ i ].beta = -1.367;
      }
      else
      {
        picParam[ i ].alpha = pow( 2.0, bitdepthLumaScale ) * RC_ALPHA;
        picParam[ i ].beta = RC_BETA2;
      }
    }
  }
  else
  {
    for ( int i = 0; i < numberOfLevel; i++ )
    {
      picParam[ i ] = picPara[ i ];
    }
  }
}

void EncRCSeq::updateAfterPic ( int bits, int tgtBits )
{
  estimatedBitUsage += tgtBits;
  bitsUsed += bits;
  framesCoded++;
  bitsLeft -= bits;
  framesLeft--;
}

void EncRCSeq::getTargetBitsFromFirstPass (const int poc, int &targetBits, double &frameVsGopRatio, bool &isNewScene, bool &refreshParameters)
{
  std::list<TRCPassStats>::iterator it;
  for ( it = firstPassData.begin(); it != firstPassData.end(); it++ )
  {
    if ( poc == it->poc )
    {
      targetBits = it->targetBits;
      frameVsGopRatio = it->frameInGopRatio;
      isNewScene = it->isNewScene;
      refreshParameters = it->refreshParameters;
      break;
    }
  }
}

void EncRCSeq::setAllBitRatio( double basicLambda, double* equaCoeffA, double* equaCoeffB )
{
  int* bitsRatio = new int[ gopSize ];
  for ( int i = 0; i < gopSize; i++ )
  {
    bitsRatio[ i ] = (int)( equaCoeffA[ i ] * pow( basicLambda, equaCoeffB[ i ] ) * (double)picParam[ gopID2Level[ i ] ].validPix );
  }
  initBitsRatio( bitsRatio );
  delete[] bitsRatio;
}

void EncRCSeq::clipRcAlpha( double& alpha )
{
  const int bitdepthLumaScale = 2 * ( bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( bitDepth ) );
  alpha = Clip3( RC_ALPHA_MIN_VALUE, RC_ALPHA_MAX_VALUE * pow( 2.0, bitdepthLumaScale ), alpha );
}

//GOP level
EncRCGOP::EncRCGOP()
{
  encRcSeq           = NULL;
  picTargetBitInGOP  = NULL;
  numPics            = 0;
  targetBits         = 0;
  picsLeft           = 0;
  bitsLeft           = 0;
  minEstLambda       = 0.0;
  maxEstLambda       = 0.0;
}

EncRCGOP::~EncRCGOP()
{
  destroy();
}

void EncRCGOP::create( EncRCSeq* encRCSeq, int numPic )
{
  destroy();
  int tgtBits = xEstGOPTargetBits( encRCSeq, numPic );
  int bitdepthLumaScale = 2 * ( encRCSeq->bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( encRCSeq->bitDepth ) );
  minEstLambda = 0.1;
  maxEstLambda = 10000.0 * pow( 2.0, bitdepthLumaScale );

  if ( encRCSeq->adaptiveBits > 0 && encRCSeq->lastLambda > 0.1 )
  {
    double targetBpp = (double)tgtBits / encRCSeq->numberOfPixel;
    double basicLambda = 0.0;
    double* lambdaRatio = new double[ encRCSeq->gopSize ];
    double* equaCoeffA = new double[ encRCSeq->gopSize ];
    double* equaCoeffB = new double[ encRCSeq->gopSize ];

    if ( encRCSeq->adaptiveBits == 1 && encRCSeq->gopSize == 4 )   // for GOP size = 4, low delay case
    {
      if ( encRCSeq->lastLambda < 120.0 )
      {
        lambdaRatio[ 1 ] = 0.725 * log( encRCSeq->lastLambda ) + 0.5793;
        lambdaRatio[ 0 ] = 1.3 * lambdaRatio[ 1 ];
        lambdaRatio[ 2 ] = 1.3 * lambdaRatio[ 1 ];
        lambdaRatio[ 3 ] = 1.0;
      }
      else
      {
        lambdaRatio[ 0 ] = 5.0;
        lambdaRatio[ 1 ] = 4.0;
        lambdaRatio[ 2 ] = 5.0;
        lambdaRatio[ 3 ] = 1.0;
      }
    }
    else if ( encRCSeq->adaptiveBits == 2 && encRCSeq->gopSize == 8 )  // for GOP size = 8, random access case
    {
      if ( encRCSeq->lastLambda < 90.0 )
      {
        lambdaRatio[ 0 ] = 1.0;
        lambdaRatio[ 1 ] = 0.725 * log( encRCSeq->lastLambda ) + 0.7963;
        lambdaRatio[ 2 ] = 1.3 * lambdaRatio[ 1 ];
        lambdaRatio[ 3 ] = 3.25 * lambdaRatio[ 1 ];
        lambdaRatio[ 4 ] = 3.25 * lambdaRatio[ 1 ];
        lambdaRatio[ 5 ] = 1.3  * lambdaRatio[ 1 ];
        lambdaRatio[ 6 ] = 3.25 * lambdaRatio[ 1 ];
        lambdaRatio[ 7 ] = 3.25 * lambdaRatio[ 1 ];
      }
      else
      {
        lambdaRatio[ 0 ] = 1.0;
        lambdaRatio[ 1 ] = 4.0;
        lambdaRatio[ 2 ] = 5.0;
        lambdaRatio[ 3 ] = 12.3;
        lambdaRatio[ 4 ] = 12.3;
        lambdaRatio[ 5 ] = 5.0;
        lambdaRatio[ 6 ] = 12.3;
        lambdaRatio[ 7 ] = 12.3;
      }
    }
    else if ( encRCSeq->adaptiveBits == 2 && encRCSeq->gopSize == 16 )  // for GOP size = 16, random access case
    {
      const double qdfParaLev2A = 0.5847;
      const double qdfParaLev2B = -0.0782;
      const double qdfParaLev3A = 0.5468;
      const double qdfParaLev3B = -0.1364;
      const double qdfParaLev4A = 0.6539;
      const double qdfParaLev4B = -0.203;
      const double qdfParaLev5A = 0.8623;
      const double qdfParaLev5B = -0.4676;
      double qdfLev1Lev2 = Clip3( 0.12, 0.9, qdfParaLev2A * encRCSeq->picParam[ 2 ].skipRatio + qdfParaLev2B );
      double qdfLev1Lev3 = Clip3( 0.13, 0.9, qdfParaLev3A * encRCSeq->picParam[ 3 ].skipRatio + qdfParaLev3B );
      double qdfLev1Lev4 = Clip3( 0.15, 0.9, qdfParaLev4A * encRCSeq->picParam[ 4 ].skipRatio + qdfParaLev4B );
      double qdfLev1Lev5 = Clip3( 0.20, 0.9, qdfParaLev5A * encRCSeq->picParam[ 5 ].skipRatio + qdfParaLev5B );
      double qdfLev2Lev3 = Clip3( 0.09, 0.9, qdfLev1Lev3 * ( 1 - qdfLev1Lev2 ) );
      double qdfLev2Lev4 = Clip3( 0.12, 0.9, qdfLev1Lev4 * ( 1 - qdfLev1Lev2 ) );
      double qdfLev2Lev5 = Clip3( 0.14, 0.9, qdfLev1Lev5 * ( 1 - qdfLev1Lev2 ) );
      double qdfLev3Lev4 = Clip3( 0.06, 0.9, qdfLev1Lev4 * ( 1 - qdfLev1Lev3 ) );
      double qdfLev3Lev5 = Clip3( 0.09, 0.9, qdfLev1Lev5 * ( 1 - qdfLev1Lev3 ) );
      double qdfLev4Lev5 = Clip3( 0.10, 0.9, qdfLev1Lev5 * ( 1 - qdfLev1Lev4 ) );

      double lambdaLev1 = 1 / ( 1 + 2 * ( qdfLev1Lev2 + 2 * qdfLev1Lev3 + 4 * qdfLev1Lev4 + 8 * qdfLev1Lev5 ) );
      double lambdaLev2 = 1 / ( 1 + ( 3 * qdfLev2Lev3 + 5 * qdfLev2Lev4 + 8 * qdfLev2Lev5 ) );
      double lambdaLev3 = 1 / ( 1 + 2 * qdfLev3Lev4 + 4 * qdfLev3Lev5 );
      double lambdaLev4 = 1 / ( 1 + 2 * qdfLev4Lev5 );
      double lambdaLev5 = 1 / ( 1.0 );

      lambdaRatio[ 0 ] = 1.0;
      lambdaRatio[ 1 ] = lambdaLev2 / lambdaLev1;
      lambdaRatio[ 2 ] = lambdaLev3 / lambdaLev1;
      lambdaRatio[ 3 ] = lambdaLev4 / lambdaLev1;
      lambdaRatio[ 4 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 5 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 6 ] = lambdaLev4 / lambdaLev1;
      lambdaRatio[ 7 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 8 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 9 ] = lambdaLev3 / lambdaLev1;
      lambdaRatio[ 10 ] = lambdaLev4 / lambdaLev1;
      lambdaRatio[ 11 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 12 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 13 ] = lambdaLev4 / lambdaLev1;
      lambdaRatio[ 14 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 15 ] = lambdaLev5 / lambdaLev1;
    }
    else if ( encRCSeq->adaptiveBits == 2 && encRCSeq->gopSize == 32 ) // for GOP size = 32, random access case
    {
      const double qdfParaLev2A = 0.7534;
      const double qdfParaLev2B = -0.0303;
      const double qdfParaLev3A = 0.7044;
      const double qdfParaLev3B = -0.0445;
      const double qdfParaLev4A = 0.7084;
      const double qdfParaLev4B = -0.1401;
      const double qdfParaLev5A = 0.8844;
      const double qdfParaLev5B = -0.3676;
      const double qdfParaLev6A = 1.2336;
      const double qdfParaLev6B = -0.7511;

      double qdfLev1Lev2 = Clip3( 0.12, 0.9, qdfParaLev2A * encRCSeq->picParam[ 2 ].skipRatio + qdfParaLev2B );
      double qdfLev1Lev3 = Clip3( 0.13, 0.9, qdfParaLev3A * encRCSeq->picParam[ 3 ].skipRatio + qdfParaLev3B );
      double qdfLev1Lev4 = Clip3( 0.15, 0.9, qdfParaLev4A * encRCSeq->picParam[ 4 ].skipRatio + qdfParaLev4B );
      double qdfLev1Lev5 = Clip3( 0.20, 0.9, qdfParaLev5A * encRCSeq->picParam[ 5 ].skipRatio + qdfParaLev5B );
      double qdfLev1Lev6 = Clip3( 0.25, 0.9, qdfParaLev6A * encRCSeq->picParam[ 6 ].skipRatio + qdfParaLev6B );

      double qdfLev2Lev3 = Clip3( 0.09, 0.9, qdfLev1Lev3 * ( 1 - qdfLev1Lev2 ) );
      double qdfLev2Lev4 = Clip3( 0.12, 0.9, qdfLev1Lev4 * ( 1 - qdfLev1Lev2 ) );
      double qdfLev2Lev5 = Clip3( 0.14, 0.9, qdfLev1Lev5 * ( 1 - qdfLev1Lev2 ) );
      double qdfLev2Lev6 = Clip3( 0.16, 0.9, qdfLev1Lev6 * ( 1 - qdfLev1Lev2 ) );
      double qdfLev3Lev4 = Clip3( 0.06, 0.9, qdfLev1Lev4 * ( 1 - qdfLev1Lev3 ) );
      double qdfLev3Lev5 = Clip3( 0.09, 0.9, qdfLev1Lev5 * ( 1 - qdfLev1Lev3 ) );
      double qdfLev3Lev6 = Clip3( 0.10, 0.9, qdfLev1Lev6 * ( 1 - qdfLev1Lev3 ) );
      double qdfLev4Lev5 = Clip3( 0.10, 0.9, qdfLev1Lev5 * ( 1 - qdfLev1Lev4 ) );
      double qdfLev4Lev6 = Clip3( 0.10, 0.9, qdfLev1Lev6 * ( 1 - qdfLev1Lev4 ) );
      double qdfLev5Lev6 = Clip3( 0.12, 0.9, qdfLev1Lev6 * ( 1 - qdfLev1Lev5 ) );

      double lambdaLev1 = 1 / ( 1 + 2 * qdfLev1Lev2 + 4 * qdfLev1Lev3 + 6 * qdfLev1Lev4 + 8 * qdfLev1Lev5 + 10 * qdfLev1Lev6 );
      double lambdaLev2 = 1 / ( 1 + 3 * qdfLev2Lev3 + 5 * qdfLev2Lev4 + 8 * qdfLev2Lev5 + 9 * qdfLev2Lev6 );
      double lambdaLev3 = 1 / ( 1 + 2 * qdfLev3Lev4 + 4 * qdfLev3Lev5 + 6 * qdfLev3Lev6 );
      double lambdaLev4 = 1 / ( 1 + 2 * qdfLev4Lev5 + 4 * qdfLev4Lev6 );
      double lambdaLev5 = 1 / ( 1 + 2 * qdfLev5Lev6 );
      double lambdaLev6 = 1 / ( 1.0 );

      lambdaRatio[ 0 ] = 1.0;
      lambdaRatio[ 1 ] = lambdaLev2 / lambdaLev1;
      lambdaRatio[ 2 ] = lambdaLev3 / lambdaLev1;
      lambdaRatio[ 3 ] = lambdaLev4 / lambdaLev1;
      lambdaRatio[ 4 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 5 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 6 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 7 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 8 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 9 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 10 ] = lambdaLev4 / lambdaLev1;
      lambdaRatio[ 11 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 12 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 13 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 14 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 15 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 16 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 17 ] = lambdaLev3 / lambdaLev1;
      lambdaRatio[ 18 ] = lambdaLev4 / lambdaLev1;
      lambdaRatio[ 19 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 20 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 21 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 22 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 23 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 24 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 25 ] = lambdaLev4 / lambdaLev1;
      lambdaRatio[ 26 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 27 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 28 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 29 ] = lambdaLev5 / lambdaLev1;
      lambdaRatio[ 30 ] = lambdaLev6 / lambdaLev1;
      lambdaRatio[ 31 ] = lambdaLev6 / lambdaLev1;
    }

    xCalEquaCoeff( encRCSeq, lambdaRatio, equaCoeffA, equaCoeffB, encRCSeq->gopSize );
    basicLambda = xSolveEqua( encRCSeq, targetBpp, equaCoeffA, equaCoeffB, encRCSeq->gopSize );
    encRCSeq->setAllBitRatio( basicLambda, equaCoeffA, equaCoeffB );

    delete[]lambdaRatio;
    delete[]equaCoeffA;
    delete[]equaCoeffB;
  }

  picTargetBitInGOP = new int[ numPic ];
  int i;
  int totalPicRatio = 0;
  int currPicRatio = 0;
  for ( i = 0; i < numPic; i++ )
  {
    totalPicRatio += encRCSeq->bitsRatio[ i ];
  }
  for ( i = 0; i < numPic; i++ )
  {
    currPicRatio = encRCSeq->bitsRatio[ i ];
    picTargetBitInGOP[ i ] = (int)( ( (double)tgtBits ) * currPicRatio / totalPicRatio );
  }

  encRcSeq     = encRCSeq;
  numPics      = numPic;
  targetBits   = tgtBits;
  picsLeft     = numPics;
  bitsLeft     = targetBits;
}

void EncRCGOP::destroy()
{
  encRcSeq = NULL;
  if ( picTargetBitInGOP != NULL )
  {
    delete[] picTargetBitInGOP;
    picTargetBitInGOP = NULL;
  }
}

void EncRCGOP::xCalEquaCoeff( EncRCSeq* encRCSeq, double* lambdaRatio, double* equaCoeffA, double* equaCoeffB, int GOPSize )
{
  for ( int i = 0; i<GOPSize; i++ )
  {
    int frameLevel = encRCSeq->gopID2Level[ i ];
    double alpha = encRCSeq->picParam[ frameLevel ].alpha;
    double beta = encRCSeq->picParam[ frameLevel ].beta;
    equaCoeffA[ i ] = pow( 1.0 / alpha, 1.0 / beta ) * pow( lambdaRatio[ i ], 1.0 / beta );
    equaCoeffB[ i ] = 1.0 / beta;
  }
}

double EncRCGOP::xSolveEqua( EncRCSeq* encRCSeq, double targetBpp, double* equaCoeffA, double* equaCoeffB, int GOPSize )
{
  double solution = 100.0;
  double minNumber = minEstLambda;
  double maxNumber = maxEstLambda;
  for ( int i = 0; i < RC_ITERATION_NUM; i++ )
  {
    double fx = 0.0;
    for ( int j = 0; j < GOPSize; j++ )
    {
      double tmpBpp = equaCoeffA[ j ] * pow( solution, equaCoeffB[ j ] );
      double actualBpp = tmpBpp * (double)encRCSeq->picParam[ encRCSeq->gopID2Level[ j ] ].validPix / (double)encRCSeq->numberOfPixel;
      fx += actualBpp;
    }

    if ( fabs( fx - targetBpp ) < 0.000001 )
    {
      break;
    }

    if ( fx > targetBpp )
    {
      minNumber = solution;
      solution = ( solution + maxNumber ) / 2.0;
    }
    else
    {
      maxNumber = solution;
      solution = ( solution + minNumber ) / 2.0;
    }
  }

  solution = Clip3( minEstLambda, maxEstLambda, solution );
  return solution;
}

void EncRCGOP::updateAfterPicture( int bitsCost )
{
  bitsLeft -= bitsCost;
  picsLeft--;
}

int EncRCGOP::xEstGOPTargetBits( EncRCSeq* encRCSeq, int GOPSize )
{
  int realInfluencePicture = encRCSeq->totalFrames > 0 ? std::min( 8 < encRCSeq->intraPeriod ? encRCSeq->intraPeriod + encRCSeq->gopSize : RC_SMOOTH_WINDOW_SIZE, encRCSeq->framesLeft ) :
                                                             ( 8 < encRCSeq->intraPeriod ? encRCSeq->intraPeriod + encRCSeq->gopSize : RC_SMOOTH_WINDOW_SIZE );
  double averageTargetBitsPerPic = ( (double)( encRCSeq->targetRate ) / encRCSeq->frameRate );
  int currentTargetBitsPerPic = (int)( ( averageTargetBitsPerPic * ( encRCSeq->framesCoded + realInfluencePicture ) - encRCSeq->bitsUsed ) / realInfluencePicture );
  int targetBits = currentTargetBitsPerPic * GOPSize;

  if ( targetBits < 200 )
  {
    targetBits = 200;   // at least allocate 200 bits for one GOP
  }

  return targetBits;
}

//picture level
EncRCPic::EncRCPic()
{
  encRCSeq            = NULL;
  encRCGOP            = NULL;

  frameLevel          = 0;
  numberOfPixel       = 0;
  numberOfLCU         = 0;
  targetBits          = 0;
  tmpTargetBits       = 0;
  estHeaderBits       = 0;
  picQPOffsetQPA      = 0;
  picLambdaOffsetQPA  = 0.0;
  bitsLeft            = 0;
  lcu                 = NULL;
  picActualHeaderBits = 0;
  picActualBits       = 0;
  picQP               = 0;
  picLambda           = 0.0;
  picMSE              = 0.0;
  totalCostIntra      = 0.0;
  validPixelsInPic    = 0;
  isNewScene          = false;
  refreshParams       = false;
  finalLambda         = 0.0;
  estimatedBits       = 0;
}

EncRCPic::~EncRCPic()
{
  destroy();
}

int EncRCPic::xEstPicTargetBits( EncRCSeq* encRcSeq, EncRCGOP* encRcGOP, int frameLevel )
{
  int targetBits    = 0;

  // bit allocation for 1-pass RC
  if (!encRcSeq->twoPass)
  {
    int GOPbitsLeft   = encRcGOP->bitsLeft;
    int currPicRatio  = encRcSeq->bitsRatio[ rcIdxInGop ];
    int totalPicRatio = 0;

    for ( int i = rcIdxInGop; i < encRcGOP->numPics; i++ )
    {
      totalPicRatio += encRcSeq->bitsRatio[ i ];
    }

    targetBits  = int( ((double)GOPbitsLeft) * currPicRatio / totalPicRatio );

    if ( targetBits < 100 )
    {
      targetBits = 100;   // at least allocate 100 bits for one picture
    }

    if ( encRcSeq->framesLeft > encRcSeq->gopSize || encRcSeq->totalFrames < 1 )
    {
      targetBits = int( RC_WEIGHT_PIC_TARGET_BIT_IN_BUFFER * targetBits + RC_WEIGHT_PIC_TARGET_BIT_IN_GOP * encRCGOP->picTargetBitInGOP[ rcIdxInGop ] );
    }
  } // !twoPass

  // bit allocation for 2-pass RC
  if (encRcSeq->twoPass)
  {
    double frameVsGopRatio = 1.0;

    encRcSeq->getTargetBitsFromFirstPass (poc, tmpTargetBits, frameVsGopRatio, isNewScene, refreshParams);

    // calculate the difference of under/overspent bits and adjust the current target bits based on the GOP and frame ratio for every frame
    targetBits = int (0.5 + tmpTargetBits + (encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed) * 0.5 * frameVsGopRatio);
  }

  return targetBits;
}

int EncRCPic::xEstPicHeaderBits( std::list<EncRCPic*>& listPreviousPictures, int frameLevel )
{
  int numPreviousPics   = 0;
  int totalPreviousBits = 0;

  std::list<EncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( (*it)->frameLevel == frameLevel )
    {
      totalPreviousBits += (*it)->picActualHeaderBits;
      numPreviousPics++;
    }
  }

  int estHeadBits = 0;
  if ( numPreviousPics > 0 )
  {
    estHeadBits = totalPreviousBits / numPreviousPics;
  }

  return estHeadBits;
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

void EncRCPic::create( EncRCSeq* encRcSeq, EncRCGOP* encRcGOP, int frameLvl, int framePoc, int frameRcIdxInGop, std::list<EncRCPic*>& listPreviousPictures )
{
  destroy();
  encRCSeq   = encRcSeq;
  encRCGOP   = encRcGOP;
  poc        = framePoc;
  rcIdxInGop = frameRcIdxInGop;

  int tgtBits    = xEstPicTargetBits( encRcSeq, encRcGOP, frameLvl );
  int estHeadBits = xEstPicHeaderBits( listPreviousPictures, frameLvl );

  if (encRcSeq->twoPass)
  {
    tgtBits = std::max (1, tgtBits);
  }
  else if ( tgtBits < estHeadBits + 100 )
  {
    tgtBits = estHeadBits + 100;   // at least allocate 100 bits for picture data
  }

  frameLevel       = frameLvl;
  numberOfPixel    = encRcSeq->numberOfPixel;
  numberOfLCU      = encRcSeq->numberOfLCU;
  targetBits       = tgtBits;
  estHeaderBits    = estHeadBits;
  bitsLeft         = targetBits;
  estimatedBits    = targetBits;
  int picWidth       = encRcSeq->picWidth;
  int picHeight      = encRcSeq->picHeight;
  int LCUWidth       = encRcSeq->lcuWidth;
  int LCUHeight      = encRcSeq->lcuHeight;
  int picWidthInLCU  = ( picWidth  % LCUWidth  ) == 0 ? picWidth  / LCUWidth  : picWidth  / LCUWidth  + 1;
  int picHeightInLCU = ( picHeight % LCUHeight ) == 0 ? picHeight / LCUHeight : picHeight / LCUHeight + 1;

  bitsLeft       -= estHeaderBits;

  lcu = new TRCLCU[ numberOfLCU ];
  int LCUIdx;
  for ( int i = 0; i < picWidthInLCU; i++ )
  {
    for ( int j = 0; j < picHeightInLCU; j++ )
    {
      LCUIdx = j * picWidthInLCU + i;
      lcu[ LCUIdx ].actualSSE = 0.0;
      lcu[ LCUIdx ].actualMSE = 0.0;
      lcu[ LCUIdx ].lambda = 0.0;
      int currWidth = ( ( i == picWidthInLCU - 1 ) ? picWidth - LCUWidth * ( picWidthInLCU - 1 ) : LCUWidth );
      int currHeight = ( ( j == picHeightInLCU - 1 ) ? picHeight - LCUHeight * ( picHeightInLCU - 1 ) : LCUHeight );
      lcu[ LCUIdx ].numberOfPixel = currWidth * currHeight;
    }
  }
  picActualHeaderBits = 0;
  picActualBits       = 0;
  picQP               = 0;
  picQPOffsetQPA      = 0;
  validPixelsInPic    = 0;
  totalCostIntra      = 0.0;
  picLambda           = 0.0;
  picLambdaOffsetQPA  = 0.0;
  picMSE              = 0.0;
}

void EncRCPic::destroy()
{
  if( lcu != NULL )
  {
    delete[] lcu;
    lcu = NULL;
  }
  encRCSeq = NULL;
  encRCGOP = NULL;
}


double EncRCPic::estimatePicLambda( std::list<EncRCPic*>& listPreviousPictures, bool isIRAP )
{
  double alpha = encRCSeq->picParam[ frameLevel ].alpha;
  double beta = encRCSeq->picParam[ frameLevel ].beta;
  double bpp = (double)targetBits / (double)numberOfPixel;

  int bitdepthLumaScale = 2 * ( encRCSeq->bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( encRCSeq->bitDepth ) );

  int lastPicValPix = 0;
  if ( listPreviousPictures.size() > 0 )
  {
    lastPicValPix = encRCSeq->picParam[ frameLevel ].validPix;
  }
  if ( lastPicValPix > 0 )
  {
    bpp = (double)targetBits / (double)lastPicValPix;
  }

  double estLambda;
  if ( isIRAP )
  {
    estLambda = calculateLambdaIntra( alpha, beta, pow( totalCostIntra / (double)numberOfPixel, RC_BETA1 ), bpp );
  }
  else
  {
    estLambda = alpha * pow( bpp, beta );
  }

  CHECK (encRCSeq->twoPass, "rate control configuration error");
  clipLambdaFrameRc( listPreviousPictures, estLambda, bitdepthLumaScale );

  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = double( int64_t( estLambda * (double)RC_LAMBDA_PREC + 0.5 ) ) / (double)RC_LAMBDA_PREC;

  return estLambda;
}

void EncRCPic::clipLambdaFrameRc( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale )
{
  bool setLastLevelLambda = false;
  double lastPrevTLLambda = -1.0;
  double lastLevelLambda = -1.0;
  double lastPicLambda = -1.0;
  double lastValidLambda = -1.0;
  std::list<EncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( ( *it )->frameLevel == frameLevel - 1 && ( *it )->picLambda > 0.0 )
    {
      lastPrevTLLambda = ( *it )->picLambda;
    }
    if ( ( *it )->frameLevel == frameLevel && ( *it )->picLambda > 0.0 )
    {
      lastLevelLambda = ( *it )->picLambda;
      setLastLevelLambda = true;
    }
    else if ( encRCSeq->framesCoded < encRCSeq->gopSize && ( *it )->frameLevel < frameLevel && ( *it )->picLambda > 0.0 && !setLastLevelLambda )
    {
      lastLevelLambda = ( *it )->picLambda;
    }
    lastPicLambda = ( *it )->picLambda;

    if ( lastPicLambda > 0.0 )
    {
      lastValidLambda = lastPicLambda;
    }
  }

  if ( lastLevelLambda > 0.0 )
  {
    lastLevelLambda = Clip3( encRCGOP->minEstLambda, encRCGOP->maxEstLambda, lastLevelLambda );
    lambda = Clip3( lastLevelLambda * pow( 2.0, -( 5.0 + encRCSeq->fppParFrames ) / 3.0 ), lastLevelLambda * pow( 2.0, ( 5.0 + encRCSeq->fppParFrames ) / 3.0 ), lambda );
  }

  if ( frameLevel > 2 )
  {
    int tlQpOffset = encRCSeq->gopSize == 32 ? RC_GOP_ID_QP_OFFSET_GOP32[ frameLevel ] : RC_GOP_ID_QP_OFFSET[ frameLevel ];
    lambda = Clip3( lastPrevTLLambda * pow( 2.0, (double)( tlQpOffset ) / 3.0 ), encRCGOP->maxEstLambda, lambda );
  }

  if ( lastPicLambda > 0.0 )
  {
    lastPicLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastPicLambda );
    double clipRange = frameLevel > 1 ? 10.0 : 12.0;
    lambda = Clip3( lastPicLambda * pow( 2.0, -clipRange / 3.0 ), lastPicLambda * pow( 2.0, clipRange / 3.0 ), lambda );
  }
  else if ( lastValidLambda > 0.0 )
  {
    lastValidLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastValidLambda );
    double clipRange = frameLevel > 1 ? 10.0 : 12.0;
    lambda = Clip3( lastValidLambda * pow( 2.0, -clipRange / 3.0 ), lastValidLambda * pow( 2.0, clipRange / 3.0 ), lambda );
  }
  else
  {
    lambda = Clip3( encRCGOP->minEstLambda, encRCGOP->maxEstLambda, lambda );
  }

  if ( lambda < encRCGOP->minEstLambda )
  {
    lambda = encRCGOP->minEstLambda;
  }
}

int EncRCPic::estimatePicQP( double lambda, std::list<EncRCPic*>& listPreviousPictures )
{
  int bitdepthLumaScale = 2 * ( encRCSeq->bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( encRCSeq->bitDepth ) );

  int QP = int( 4.2005 * log( lambda / pow( 2.0, bitdepthLumaScale ) ) + 13.7122 + 0.5 );

  CHECK (encRCSeq->twoPass, "rate control configuration error");
  clipQpFrameRc( listPreviousPictures, QP );

  return QP;
}

void EncRCPic::clipQpFrameRc( std::list<EncRCPic*>& listPreviousPictures, int &QP )
{
  bool setLastLevelQP = false;
  int lastPrevTLQP = RC_INVALID_QP_VALUE;
  int lastLevelQP = RC_INVALID_QP_VALUE;
  int lastPicQP = RC_INVALID_QP_VALUE;
  int lastValidQP = RC_INVALID_QP_VALUE;
  std::list<EncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( ( *it )->frameLevel == frameLevel - 1 && ( *it )->picQP > RC_INVALID_QP_VALUE )
    {
      lastPrevTLQP = ( *it )->picQP;
    }
    if ( ( *it )->frameLevel == frameLevel && ( *it )->picQP > RC_INVALID_QP_VALUE ) // use the QP value from the last available frame of that level
    {
      lastLevelQP = ( *it )->picQP;
      setLastLevelQP = true;
    }
    else if ( encRCSeq->framesCoded < encRCSeq->gopSize && ( *it )->frameLevel < frameLevel && ( *it )->picQP > RC_INVALID_QP_VALUE && !setLastLevelQP )
    {
      lastLevelQP = ( *it )->picQP;
    }
    lastPicQP = ( *it )->picQP;
    if ( lastPicQP > RC_INVALID_QP_VALUE )
    {
      lastValidQP = lastPicQP;
    }
  }

  if ( lastLevelQP > RC_INVALID_QP_VALUE )
  {
    QP = Clip3( lastLevelQP - ( 5 + encRCSeq->fppParFrames ), lastLevelQP + ( 5 + encRCSeq->fppParFrames ), QP );
  }

  if ( frameLevel > 2 )
  {
    int tlQpOffset = encRCSeq->gopSize == 32 ? RC_GOP_ID_QP_OFFSET_GOP32[ frameLevel ] : RC_GOP_ID_QP_OFFSET[ frameLevel ];
    QP = Clip3( lastPrevTLQP + tlQpOffset, MAX_QP, QP );
  }

  if ( lastPicQP > RC_INVALID_QP_VALUE )
  {
    int clipRange = frameLevel > 1 ? 10 : 12;
    QP = Clip3( lastPicQP - clipRange, lastPicQP + clipRange, QP );
  }
  else if ( lastValidQP > RC_INVALID_QP_VALUE )
  {
    int clipRange = frameLevel > 1 ? 10 : 12;
    QP = Clip3( lastValidQP - clipRange, lastValidQP + clipRange, QP );
  }
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

void EncRCPic::updateAfterCTU( int LCUIdx, int bits, double lambda )
{
  if (encRCSeq->twoPass) return;

  lcu[ LCUIdx ].lambda = lambda;
  lcu[ LCUIdx ].actualSSE = lcu[ LCUIdx ].actualMSE * lcu[ LCUIdx ].numberOfPixel;

  bitsLeft -= bits;
}

void EncRCPic::calPicMSE()
{
  double totalSSE = 0.0;
  int totalPixels = 0;
  for ( int i = 0; i < numberOfLCU; i++ )
  {
    if ( lcu[ i ].lambda > 0.1 && encRCSeq->adaptiveBits > 0 )
    {
      validPixelsInPic += lcu[ i ].numberOfPixel;
      totalSSE += lcu[ i ].actualSSE;
      totalPixels += lcu[ i ].numberOfPixel;
    }
  }

  picMSE = totalPixels > 0 ? totalSSE / (double)totalPixels : 1.0;
}

void EncRCPic::updateAfterPicture( int actualHeaderBits, int actualTotalBits, double averageQP, double averageLambda, bool isIRAP)
{
  picActualHeaderBits = actualHeaderBits;
  picActualBits       = actualTotalBits;
  picLambda           = averageLambda;
  picQP               = int (averageQP + 0.5);

  if (!encRCSeq->twoPass)
  {
    if ( averageQP > 0.0 )
    {
      picQP           = int( averageQP + 0.5 );
    }
    else
    {
      picQP           = RC_INVALID_QP_VALUE;
    }

    double alpha = encRCSeq->picParam[ frameLevel ].alpha;
    double beta  = encRCSeq->picParam[ frameLevel ].beta;
    double skipRatio = 0;
    int numOfSkipPixel = 0;
    for (int LCUIdx = 0; LCUIdx < numberOfLCU; LCUIdx++)
    {
      numOfSkipPixel += int( encRCSeq->picParam[ frameLevel ].skipRatio * lcu[ LCUIdx ].numberOfPixel );
    }
    skipRatio = (double)numOfSkipPixel / (double)numberOfPixel;

    if (isIRAP)
    {
      updateAlphaBetaIntra(alpha, beta);
      TRCParameter rcPara;
      rcPara.skipRatio = skipRatio;
      rcPara.validPix = validPixelsInPic;
      rcPara.alpha = alpha;
      rcPara.beta = beta;
      encRCSeq->picParam[ frameLevel ] = rcPara;
    }
    else
    {
      // update parameters
      double pictureActualBits = (double)picActualBits;
      double pictureActualBpp = validPixelsInPic > 0 ? pictureActualBits / (double)validPixelsInPic : 0.001;
      TRCParameter rcPara;
      rcPara.skipRatio = skipRatio;
      rcPara.validPix = validPixelsInPic;
      double avgMSE = picMSE;
      double updatedK = pictureActualBpp * averageLambda / avgMSE;
      double updatedC = avgMSE / pow( pictureActualBpp, -updatedK );
      rcPara.alpha = updatedC * updatedK;
      rcPara.beta = -updatedK - 1.0;
      if ( validPixelsInPic > 0 )
      {
        encRCSeq->clipRcAlpha( rcPara.alpha );
        clipRcBeta( rcPara.beta );
        encRCSeq->picParam[ frameLevel ] = rcPara;
      }
      if ( frameLevel == 1 )
      {
        double currLambda = Clip3( encRCGOP->minEstLambda, encRCGOP->maxEstLambda, picLambda );
        encRCSeq->lastLambda = RC_WEIGHT_HISTORY_LAMBDA * encRCSeq->lastLambda + ( 1.0 - RC_WEIGHT_HISTORY_LAMBDA ) * currLambda;
      }
    }
  } // if !twoPass

  if ((frameLevel <= 7) && (picActualBits > 0) && (targetBits > 0)) // update qpCorrection for EncGOP::picInitRateControl()
  {
    const bool refreshed = (encRCSeq->actualBitCnt[frameLevel] == 0) && (encRCSeq->targetBitCnt[frameLevel] == 0);

    encRCSeq->actualBitCnt[frameLevel] += (uint64_t) picActualBits;
    encRCSeq->targetBitCnt[frameLevel] += (uint64_t) targetBits;

    encRCSeq->qpCorrection[frameLevel] = (refreshed ? 1.0 : 6.0) * log ((double) encRCSeq->actualBitCnt[frameLevel] / (double) encRCSeq->targetBitCnt[frameLevel]) / log (2.0);
    encRCSeq->qpCorrection[frameLevel] = Clip3 (-12.0, 12.0, encRCSeq->qpCorrection[frameLevel]);
  }
}

void EncRCPic::clipRcBeta( double& beta)
{
  beta =  Clip3( RC_BETA_MIN_VALUE, RC_BETA_MAX_VALUE, beta );
}

int EncRCPic::getRefineBitsForIntra( int orgBits )
{
  double alpha = 0.25, beta = 0.5582;
  int iIntraBits;

  alpha = orgBits * 40 < numberOfPixel ? 0.25 : 0.3 ;

  iIntraBits = (int)( alpha * pow( totalCostIntra * 4.0 / (double)orgBits, beta ) * (double)orgBits + 0.5 );

  return iIntraBits;
}

double EncRCPic::calculateLambdaIntra(double alpha, double beta, double MADPerPixel, double bitsPerPixel)
{
  return ( (alpha / 256.0) * pow( MADPerPixel / bitsPerPixel, beta ) );
}

void EncRCPic::updateAlphaBetaIntra(double& alpha, double& beta)
{
  double lnbpp = log( pow( totalCostIntra / (double)numberOfPixel, RC_BETA1 ) ) - log( (double)picActualBits / (double)numberOfPixel );
  double diffLambda = beta * ( log( (double)picActualBits ) - log( (double)targetBits ) );

  diffLambda = Clip3(-0.125, 0.125, 0.25*diffLambda);
  alpha    =  alpha * exp(diffLambda);
  beta     =  beta + diffLambda / lnbpp;
}

RateCtrl::RateCtrl()
{
  encRCSeq      = NULL;
  encRCGOP      = NULL;
  encRCPic      = NULL;
  flushPOC      = -1;
  rcPass        = 0;
  rcMaxPass     = 0;
  rcIsFinalPass = true;
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
  if ( encRCGOP != NULL )
  {
    delete encRCGOP;
    encRCGOP = NULL;
  }
  while ( m_listRCPictures.size() > 0 )
  {
    EncRCPic* p = m_listRCPictures.front();
    m_listRCPictures.pop_front();
    delete p;
  }
}

void RateCtrl::init( int totalFrames, int targetBitrate, int frameRate, int intraPeriod, int GOPSize,
                     int picWidth, int picHeight, int LCUWidth, int LCUHeight, int bitDepth,
                     const vvencGOPEntry GOPList[ VVENC_MAX_GOP ], int maxParallelFrames )
{
  destroy();

  bool isLowdelay = true;
  for ( int i = 0; i < GOPSize - 1; i++ )
  {
    if ( GOPList[ i ].m_POC > GOPList[ i + 1 ].m_POC )
    {
      isLowdelay = false;
      break;
    }
  }

  int numberOfLevel = 1;
  int adaptiveBit = 0;
  if ( !isLowdelay && ( GOPSize == 32 || GOPSize == 16 || GOPSize == 8 ) )
  {
    numberOfLevel = int( log( (double)GOPSize ) / log( 2.0 ) + 0.5 ) + 1;
  }
  numberOfLevel++;    // intra picture
  numberOfLevel++;    // non-reference picture


  int* bitsRatio;
  bitsRatio = new int[ GOPSize ];
  for ( int i = 0; i < GOPSize; i++ )
  {
    bitsRatio[ i ] = 10;
    if ( !GOPList[ i ].m_refPic )
    {
      bitsRatio[ i ] = 2;
    }
  }

  double bpp = (double)( targetBitrate / (double)( frameRate * picWidth * picHeight ) );
  if ( GOPSize == 4 && isLowdelay )
  {
    if ( bpp > 0.2 )
    {
      static const int init[] = { 2, 3, 2, 6 };
      std::copy_n( init, 4, bitsRatio );
    }
    else if ( bpp > 0.1 )
    {
      static const int init[] = { 2, 3, 2, 10 };
      std::copy_n( init, 4, bitsRatio );
    }
    else if ( bpp > 0.05 )
    {
      static const int init[] = { 2, 3, 2, 12 };
      std::copy_n( init, 4, bitsRatio );
    }
    else
    {
      static const int init[] = { 2, 3, 2, 14 };
      std::copy_n( init, 4, bitsRatio );
    }

    adaptiveBit = 1;
  }
  else if ( GOPSize == 8 && !isLowdelay )
  {
    if ( bpp > 0.2 )
    {
      static const int init[] = { 15, 5, 4, 1, 1, 4, 1, 1 };
      std::copy_n( init, 8, bitsRatio );
    }
    else if ( bpp > 0.1 )
    {
      static const int init[] = { 20, 6, 4, 1, 1, 4, 1, 1 };
      std::copy_n( init, 8, bitsRatio );
    }
    else if ( bpp > 0.05 )
    {
      static const int init[] = { 25, 7, 4, 1, 1, 4, 1, 1 };
      std::copy_n( init, 8, bitsRatio );
    }
    else
    {
      static const int init[] = { 30, 8, 4, 1, 1, 4, 1, 1 };
      std::copy_n( init, 8, bitsRatio );
    }

    adaptiveBit = 2;
  }
  else if ( GOPSize == 16 && !isLowdelay )
  {
    bitsRatio[  0 ] = (int)( ( -0.5691 * bpp + 0.3577 ) * 1000 + 0.5 );
    bitsRatio[  1 ] = (int)( ( -0.0332 * bpp + 0.1782 ) * 1000 + 0.5 );
    bitsRatio[  2 ] = (int)( (  0.0595 * bpp + 0.0810 ) * 1000 + 0.5 );
    bitsRatio[  3 ] = (int)( (  0.0710 * bpp + 0.0392 ) * 1000 + 0.5 );
    bitsRatio[  4 ] = (int)( (  0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
    bitsRatio[  5 ] = (int)( (  0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
    bitsRatio[  6 ] = (int)( (  0.0710 * bpp + 0.0392 ) * 1000 + 0.5 );
    bitsRatio[  7 ] = (int)( (  0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
    bitsRatio[  8 ] = (int)( (  0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
    bitsRatio[  9 ] = (int)( (  0.0595 * bpp + 0.0810 ) * 1000 + 0.5 );
    bitsRatio[ 10 ] = (int)( (  0.0710 * bpp + 0.0392 ) * 1000 + 0.5 );
    bitsRatio[ 11 ] = (int)( (  0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
    bitsRatio[ 12 ] = (int)( (  0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
    bitsRatio[ 13 ] = (int)( (  0.0710 * bpp + 0.0392 ) * 1000 + 0.5 );
    bitsRatio[ 14 ] = (int)( (  0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
    bitsRatio[ 15 ] = (int)( (  0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );

    adaptiveBit = 2;
  }
  else if ( GOPSize == 32 && !isLowdelay )
  {
    static const int bitsRatioInit[ 4 ][ 6 ] = {
      { 16, 10, 8, 4, 2, 1 },
      { 16, 10, 8, 4, 2, 1 },
      { 16, 10, 8, 4, 2, 1 },
      { 10,  8, 6, 4, 2, 1 } };
    int cls;
    if ( bpp > 0.2 )
    {
      cls = 0;
    }
    else if ( bpp > 0.1 )
    {
      cls = 1;
    }
    else if ( bpp > 0.05 )
    {
      cls = 2;
    }
    else
    {
      cls = 3;
    }
    static const int index[ 32 ] = { 0, 1, 2, 3, 4, 5, 5, 4, 5, 5, 3, 4, 5, 5, 4, 5, 5, 2, 3, 4, 5, 5, 4, 5, 5, 3, 4, 5, 5, 4, 5, 5 };

    for ( int i = 0; i < 32; i++ )
    {
      bitsRatio[ i ] = bitsRatioInit[ cls ][ index[ i ] ];
    }
    adaptiveBit = 2;
  }
  else
  {
    msg( VVENC_WARNING, "\n hierarchical bit allocation is not currently supported for the specified coding structure.\n" );
  }

  int* GOPID2Level = new int[ GOPSize ];
  for ( int i = 0; i<GOPSize; i++ )
  {
    GOPID2Level[ i ] = 1;
    if ( !GOPList[ i ].m_refPic )
    {
      GOPID2Level[ i ] = 2;
    }
  }

  if ( GOPSize == 4 && isLowdelay )
  {
    static const int init[] = { 3, 2, 3, 1 };
    std::copy_n( init, 4, GOPID2Level );
  }
  else if ( GOPSize == 8 && !isLowdelay )
  {
    static const int init[] = { 1, 2, 3, 4, 4, 3, 4, 4 };
    std::copy_n( init, 8, GOPID2Level );
  }
  else if ( GOPSize == 16 && !isLowdelay )
  {
    static const int init[] = { 1, 2, 3, 4, 5, 5, 4, 5, 5, 3, 4, 5, 5, 4, 5, 5 };
    std::copy_n( init, 16, GOPID2Level );
  }
  else if ( GOPSize == 32 && !isLowdelay )
  {
    static const int init[] = { 1, 2, 3, 4,  5, 6, 6, 6,  6, 6, 4, 5,  6, 6, 5, 6,  6, 3, 4, 5,  6, 6, 5, 6,  6, 4, 5, 6,  6, 5, 6, 6 };
    std::copy_n( init, 32, GOPID2Level );
  }

  encRCSeq = new EncRCSeq;
  encRCSeq->create( rcMaxPass == 1, totalFrames, targetBitrate, frameRate, intraPeriod, GOPSize, picWidth, picHeight, LCUWidth, LCUHeight, numberOfLevel, adaptiveBit, getFirstPassStats() );
  encRCSeq->initBitsRatio( bitsRatio );
  encRCSeq->initGOPID2Level( GOPID2Level );
  encRCSeq->bitDepth = bitDepth;
  if (rcMaxPass <= 0) encRCSeq->initPicPara();
  encRCSeq->fppParFrames = maxParallelFrames;

  delete[] bitsRatio;
  delete[] GOPID2Level;
}

void RateCtrl::initRCGOP (const int numberOfPictures)
{
  encRCGOP = new EncRCGOP;
  encRCGOP->create (encRCSeq, numberOfPictures);
}

void RateCtrl::destroyRCGOP()
{
  delete encRCGOP;
  encRCGOP = NULL;
}

void RateCtrl::setRCPass (const int pass, const int maxPass)
{
  rcPass        = pass;
  rcMaxPass     = maxPass;
  rcIsFinalPass = (pass >= maxPass);
}

void RateCtrl::processFirstPassData (const int secondPassBaseQP)
{
  CHECK( m_listRCFirstPassStats.size() == 0, "No data available from the first pass!" );

  m_listRCFirstPassStats.sort( []( const TRCPassStats& a, const TRCPassStats& b ) { return a.poc < b.poc; } );

  // run a simple scene change detection
  detectNewScene();

  // process and scale GOP and frame bits using the data from the first pass to account for different target bitrates
  processGops (secondPassBaseQP);

  // loop though the first pass data and update RC parameters when new scenes are detected
  adaptToSceneChanges();
}

uint64_t RateCtrl::getTotalBitsInFirstPass()
{
  uint64_t totalBitsFirstPass = 0;
  std::list<TRCPassStats>::iterator it;

  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++)
  {
    totalBitsFirstPass += it->numBits;
  }

  return totalBitsFirstPass;
}

void RateCtrl::detectNewScene()
{
  uint16_t visActPrev = 0;
  double   psnrPrev = 0.0;
  std::list<TRCPassStats>::iterator it;

  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++)
  {
    it->isNewScene = ((it->visActY * 64 > visActPrev * 181) || (it->tempLayer <= 1 && it->visActY <= (1u << (encRCSeq->bitDepth - 6))) || (it->isIntra && it->visActY > visActPrev && std::abs(it->psnrY - psnrPrev) > 4.5));
    visActPrev = it->visActY;
    if (it->isIntra) psnrPrev = it->psnrY;
  }
}

void RateCtrl::processGops (const int secondPassBaseQP)
{
  const unsigned fps = encRCSeq->frameRate;
  const int gopShift = int (0.5 + log ((double) encRCSeq->gopSize) / log (2.0));
  const int qpOffset = Clip3 (0, 6, ((secondPassBaseQP + 1) >> 1) - 9);
  const double bp1pf = getTotalBitsInFirstPass() / (double) m_listRCFirstPassStats.size(); // first pass
  const double ratio = (double) encRCSeq->targetRate / (fps * bp1pf);  // ratio of second and first pass
  const double rp[6] = { pow (ratio, 0.5), pow (ratio, 0.75), pow (ratio, 0.875), pow (ratio, 0.9375), pow (ratio, 0.96875), pow (ratio, 0.984375) };
  std::list<TRCPassStats>::iterator it;
  std::vector<uint32_t> gopBits (2 + (m_listRCFirstPassStats.size() - 1) / encRCSeq->gopSize); // +2 for the first I frame (GOP) and a potential last incomplete GOP
  std::vector<uint32_t> tgtBits (2 + (m_listRCFirstPassStats.size() - 1) / encRCSeq->gopSize);

  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // scaling, part 1
  {
    const int vecIdx = 1 + ((it->poc - 1) >> gopShift);

    it->targetBits = std::max (0, int (0.5 + it->numBits * (it->tempLayer + qpOffset < 6 ? rp[it->tempLayer + qpOffset] : ratio)));
    gopBits[vecIdx] += (uint32_t) it->targetBits; // similar to g in VCIP paper
    tgtBits[vecIdx] += (uint32_t) (0.5 + it->numBits * ratio);
  }
  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++) // scaling, part 2
  {
    const int vecIdx = 1 + ((it->poc - 1) >> gopShift);

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
  if (rcPass < rcMaxPass)
  {
    m_listRCFirstPassStats.push_back( TRCPassStats( poc, qp, lambda, visActY,
                                                    numBits, psnrY, isIntra, tempLayer ) );
  }
}

static int xCalcHADs8x8_ISlice( const Pel *piOrg, const int iStrideOrg )
{
  int k, i, j, jj;
  int diff[ 64 ], m1[ 8 ][ 8 ], m2[ 8 ][ 8 ], m3[ 8 ][ 8 ], iSumHad = 0;

  for ( k = 0; k < 64; k += 8 )
  {
    diff[ k + 0 ] = piOrg[ 0 ];
    diff[ k + 1 ] = piOrg[ 1 ];
    diff[ k + 2 ] = piOrg[ 2 ];
    diff[ k + 3 ] = piOrg[ 3 ];
    diff[ k + 4 ] = piOrg[ 4 ];
    diff[ k + 5 ] = piOrg[ 5 ];
    diff[ k + 6 ] = piOrg[ 6 ];
    diff[ k + 7 ] = piOrg[ 7 ];

    piOrg += iStrideOrg;
  }

  //horizontal
  for ( j = 0; j < 8; j++ )
  {
    jj = j << 3;
    m2[ j ][ 0 ] = diff[ jj ] + diff[ jj + 4 ];
    m2[ j ][ 1 ] = diff[ jj + 1 ] + diff[ jj + 5 ];
    m2[ j ][ 2 ] = diff[ jj + 2 ] + diff[ jj + 6 ];
    m2[ j ][ 3 ] = diff[ jj + 3 ] + diff[ jj + 7 ];
    m2[ j ][ 4 ] = diff[ jj ] - diff[ jj + 4 ];
    m2[ j ][ 5 ] = diff[ jj + 1 ] - diff[ jj + 5 ];
    m2[ j ][ 6 ] = diff[ jj + 2 ] - diff[ jj + 6 ];
    m2[ j ][ 7 ] = diff[ jj + 3 ] - diff[ jj + 7 ];

    m1[ j ][ 0 ] = m2[ j ][ 0 ] + m2[ j ][ 2 ];
    m1[ j ][ 1 ] = m2[ j ][ 1 ] + m2[ j ][ 3 ];
    m1[ j ][ 2 ] = m2[ j ][ 0 ] - m2[ j ][ 2 ];
    m1[ j ][ 3 ] = m2[ j ][ 1 ] - m2[ j ][ 3 ];
    m1[ j ][ 4 ] = m2[ j ][ 4 ] + m2[ j ][ 6 ];
    m1[ j ][ 5 ] = m2[ j ][ 5 ] + m2[ j ][ 7 ];
    m1[ j ][ 6 ] = m2[ j ][ 4 ] - m2[ j ][ 6 ];
    m1[ j ][ 7 ] = m2[ j ][ 5 ] - m2[ j ][ 7 ];

    m2[ j ][ 0 ] = m1[ j ][ 0 ] + m1[ j ][ 1 ];
    m2[ j ][ 1 ] = m1[ j ][ 0 ] - m1[ j ][ 1 ];
    m2[ j ][ 2 ] = m1[ j ][ 2 ] + m1[ j ][ 3 ];
    m2[ j ][ 3 ] = m1[ j ][ 2 ] - m1[ j ][ 3 ];
    m2[ j ][ 4 ] = m1[ j ][ 4 ] + m1[ j ][ 5 ];
    m2[ j ][ 5 ] = m1[ j ][ 4 ] - m1[ j ][ 5 ];
    m2[ j ][ 6 ] = m1[ j ][ 6 ] + m1[ j ][ 7 ];
    m2[ j ][ 7 ] = m1[ j ][ 6 ] - m1[ j ][ 7 ];
  }

  //vertical
  for ( i = 0; i < 8; i++ )
  {
    m3[ 0 ][ i ] = m2[ 0 ][ i ] + m2[ 4 ][ i ];
    m3[ 1 ][ i ] = m2[ 1 ][ i ] + m2[ 5 ][ i ];
    m3[ 2 ][ i ] = m2[ 2 ][ i ] + m2[ 6 ][ i ];
    m3[ 3 ][ i ] = m2[ 3 ][ i ] + m2[ 7 ][ i ];
    m3[ 4 ][ i ] = m2[ 0 ][ i ] - m2[ 4 ][ i ];
    m3[ 5 ][ i ] = m2[ 1 ][ i ] - m2[ 5 ][ i ];
    m3[ 6 ][ i ] = m2[ 2 ][ i ] - m2[ 6 ][ i ];
    m3[ 7 ][ i ] = m2[ 3 ][ i ] - m2[ 7 ][ i ];

    m1[ 0 ][ i ] = m3[ 0 ][ i ] + m3[ 2 ][ i ];
    m1[ 1 ][ i ] = m3[ 1 ][ i ] + m3[ 3 ][ i ];
    m1[ 2 ][ i ] = m3[ 0 ][ i ] - m3[ 2 ][ i ];
    m1[ 3 ][ i ] = m3[ 1 ][ i ] - m3[ 3 ][ i ];
    m1[ 4 ][ i ] = m3[ 4 ][ i ] + m3[ 6 ][ i ];
    m1[ 5 ][ i ] = m3[ 5 ][ i ] + m3[ 7 ][ i ];
    m1[ 6 ][ i ] = m3[ 4 ][ i ] - m3[ 6 ][ i ];
    m1[ 7 ][ i ] = m3[ 5 ][ i ] - m3[ 7 ][ i ];

    m2[ 0 ][ i ] = m1[ 0 ][ i ] + m1[ 1 ][ i ];
    m2[ 1 ][ i ] = m1[ 0 ][ i ] - m1[ 1 ][ i ];
    m2[ 2 ][ i ] = m1[ 2 ][ i ] + m1[ 3 ][ i ];
    m2[ 3 ][ i ] = m1[ 2 ][ i ] - m1[ 3 ][ i ];
    m2[ 4 ][ i ] = m1[ 4 ][ i ] + m1[ 5 ][ i ];
    m2[ 5 ][ i ] = m1[ 4 ][ i ] - m1[ 5 ][ i ];
    m2[ 6 ][ i ] = m1[ 6 ][ i ] + m1[ 7 ][ i ];
    m2[ 7 ][ i ] = m1[ 6 ][ i ] - m1[ 7 ][ i ];
  }

  for ( i = 0; i < 8; i++ )
  {
    for ( j = 0; j < 8; j++ )
    {
      iSumHad += std::abs( m2[ i ][ j ] );
    }
  }
  iSumHad -= std::abs( m2[ 0 ][ 0 ] );
  iSumHad = ( iSumHad + 2 ) >> 2;
  return( iSumHad );
}

static int updateCtuDataISlice( const CPelBuf buf )
{
  int  xBl, yBl;
  const int iBlkSize = 8;
  const Pel* pOrgInit = buf.buf;
  int  iStrideOrig = buf.stride;

  int iSumHad = 0;
  for ( yBl = 0; ( yBl + iBlkSize ) <= buf.height; yBl += iBlkSize )
  {
    for ( xBl = 0; ( xBl + iBlkSize ) <= buf.width; xBl += iBlkSize )
    {
      const Pel* pOrg = pOrgInit + iStrideOrig*yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice( pOrg, iStrideOrig );
    }
  }
  return( iSumHad );
}

void EncRCPic::calCostSliceI( Picture* pic ) // TODO: this only analyses the first slice segment. What about the others?
{
  double      iSumHadSlice = 0;
  const PreCalcValues& pcv = *pic->cs->pcv;
  const int          shift = pic->cs->sps->bitDepths[ CH_L ] - 8;
  const int         offset = ( shift>0 ) ? ( 1 << ( shift - 1 ) ) : 0;

  for ( uint32_t ctuIdx = 0; ctuIdx < pic->cs->pcv->sizeInCtus; ctuIdx++ )
  {
    uint32_t ctuRsAddr = ctuIdx;
    Position pos( ( ctuRsAddr % pcv.widthInCtus ) * pcv.maxCUSize, ( ctuRsAddr / pcv.widthInCtus ) * pcv.maxCUSize );

    const int height = std::min( pcv.maxCUSize, pcv.lumaHeight - pos.y );
    const int width = std::min( pcv.maxCUSize, pcv.lumaWidth - pos.x );
    const CompArea blk( COMP_Y, pcv.chrFormat, pos, Size( width, height ) );
    int iSumHad = updateCtuDataISlice( pic->getOrigBuf( blk ) );

    int cost = ( iSumHad + offset ) >> shift;
    iSumHadSlice += cost;
  }

  totalCostIntra = iSumHadSlice;
}


}

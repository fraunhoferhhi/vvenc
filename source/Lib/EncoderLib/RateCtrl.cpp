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
static const int    RC_LCU_SMOOTH_WINDOW_SIZE =                        4;
static const int    RC_LAMBDA_PREC =                             1000000;
static const int    RC_GOP_ID_QP_OFFSET[ 6 ] =           { 0, 0, 0, 3, 1, 1 };
static const int    RC_GOP_ID_QP_OFFSET_GOP32[ 7 ] =     { 0, 0, 0, 0, 3, 1, 1 };
static const int    RC_GOP_ID_QP_OFFSET_GRC[ 6 ] =       { 0, 3, 0, 3, 1, 1 };
static const int    RC_GOP_ID_QP_OFFSET_GRC_GOP32[ 7 ] = { 0, 0, 3, 0, 3, 1, 1 };
static const double RC_WEIGHT_PIC_TARGET_BIT_IN_GOP =                0.9;
static const double RC_WEIGHT_PIC_TARGET_BIT_IN_BUFFER =             1.0 - RC_WEIGHT_PIC_TARGET_BIT_IN_GOP;
static const double RC_WEIGHT_HISTORY_LAMBDA =                       0.5;
static const double RC_WEIGHT_CURRENT_LAMBDA =                       1.0 - RC_WEIGHT_HISTORY_LAMBDA;
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
  rcMode              = 0;
  twoPass             = false;
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
  lcuParam            = NULL;
  numberOfPixel       = 0;
  framesCoded         = 0;
  bitsUsed            = 0;
  framesLeft          = 0;
  bitsLeft            = 0;
  estimatedBitUsage   = 0;
  useLCUSeparateModel = false;
  adaptiveBits        = 0;
  bitUsageRatio       = 0.0;
  lastLambda          = 0.0;
  bitDepth            = 0;
}

EncRCSeq::~EncRCSeq()
{
  destroy();
}

void EncRCSeq::create( int RCMode, bool twoPassRC, int totFrames, int targetBitrate, int frRate, int intraPer, int GOPSize, int pictureWidth, int pictureHeight, int LCUWidth, int LCUHeight, int numOfLevel, bool useLCUSepModel, int adaptiveBit, std::list<TRCPassStats> &firstPassStats )
{
  destroy();
  rcMode              = RCMode;
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
  useLCUSeparateModel = useLCUSepModel;
  firstPassData       = firstPassStats;

  numberOfPixel = picWidth * picHeight;
  targetBits = (int64_t)totalFrames * (int64_t)targetRate / (int64_t)frameRate;

  averageBits     = (int)( targetRate / frameRate );
  int picWidthInBU  = ( picWidth  % lcuWidth  ) == 0 ? picWidth  / lcuWidth  : picWidth  / lcuWidth  + 1;
  int picHeightInBU = ( picHeight % lcuHeight ) == 0 ? picHeight / lcuHeight : picHeight / lcuHeight + 1;
  numberOfLCU     = picWidthInBU * picHeightInBU;

  bitsRatio = new int[ gopSize ];
  for ( int i = 0; i < gopSize; i++ )
  {
    bitsRatio[ i ] = 1;
  }

  gopID2Level = new int[ gopSize ];
  for ( int i = 0; i < gopSize; i++ )
  {
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

  if ( useLCUSeparateModel )
  {
    lcuParam = new TRCParameter*[ numberOfLevel ];
    for ( int i = 0; i < numberOfLevel; i++ )
    {
      lcuParam[ i ] = new TRCParameter[ numberOfLCU ];
      for ( int j = 0; j < numberOfLCU; j++ )
      {
        lcuParam[ i ][ j ].alpha = 0.0;
        lcuParam[ i ][ j ].beta = 0.0;
        lcuParam[ i ][ j ].validPix = -1;
        lcuParam[ i ][ j ].skipRatio = 0.0;
      }
    }
  }

  framesCoded = 0;
  bitsUsed = 0;
  framesLeft = totalFrames;
  bitsLeft   = targetBits;
  estimatedBitUsage = 0;
  bitUsageRatio = 0.0;
  adaptiveBits = adaptiveBit;
  lastLambda = 0.0;
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

  if ( lcuParam != NULL )
  {
    for ( int i = 0; i < numberOfLevel; i++ )
    {
      delete[] lcuParam[ i ];
    }
    delete[] lcuParam;
    lcuParam = NULL;
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
      if (i>0)
      {
        int bitdepthLumaScale = 2 * ( bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( bitDepth ) );
        picParam[ i ].alpha = 3.2003 * pow( 2.0, bitdepthLumaScale );
        picParam[ i ].beta = -1.367;
      }
      else
      {
        int bitdepthLumaScale = 2 * ( bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( bitDepth ) );
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

void EncRCSeq::initLCUPara( TRCParameter** LCUPara )
{
  if ( lcuParam == NULL )
  {
    return;
  }
  if ( LCUPara == NULL )
  {
    for ( int i = 0; i < numberOfLevel; i++ )
    {
      for ( int j = 0; j < numberOfLCU; j++ )
      {
        lcuParam[ i ][ j ].alpha = picParam[ i ].alpha;
        lcuParam[ i ][ j ].beta = picParam[ i ].beta;
      }
    }
  }
  else
  {
    for ( int i = 0; i < numberOfLevel; i++ )
    {
      for ( int j = 0; j < numberOfLCU; j++ )
      {
        lcuParam[ i ][ j ] = LCUPara[ i ][ j ];
      }
    }
  }
}

void EncRCSeq::updateAfterPic ( int bits )
{
  bitsUsed += bits;
  framesCoded++;
  bitsLeft -= bits;
  framesLeft--;
}

void EncRCSeq::getTargetBitsFromFirstPass( int numPicCoded, int &targetBits, double &gopVsBitrateRatio, double &frameVsGopRatio, bool &isNewScene, double alpha[] )
{
  int picCounter = 0;
  int numOfLevels = int( log( gopSize ) / log( 2 ) + 0.5 ) + 2;

  std::list<TRCPassStats>::iterator it;
  for ( it = firstPassData.begin(); it != firstPassData.end(); it++, picCounter++ )
  {
    if ( numPicCoded == picCounter )
    {
      targetBits = it->targetBits;
      gopVsBitrateRatio = it->gopBitsVsBitrate;
      isNewScene = it->isNewScene;
      for ( int i = 0; i < numOfLevels; i++ )
      {
        alpha[ i ] = it->estAlpha[ i ];
      }
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
void EncRCSeq::setQpInGOP( int gopId, int gopQp, int &qp )
{
  bool isFirstPicInGOP = 0 == gopId;
  bool isLastIncompleteGOP = totalFrames > 0 && framesLeft == ( totalFrames - 1 ) % gopSize;

  if ( !isFirstPicInGOP && !isLastIncompleteGOP )
  {
    qp = gopQp;
  }
}

bool EncRCSeq::isQpResetRequired( int gopId )
{
  bool isFirstPicInGOP = 0 == gopId || ( totalFrames > 0 && framesLeft == ( totalFrames - 1 ) % gopSize );
  bool resetRequired = rcMode < 3 || ( rcMode == 3 && isFirstPicInGOP );

  return resetRequired;
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
  gopQP              = 0;
  idealTargetGOPBits = -1;
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

    delete []lambdaRatio;
    delete []equaCoeffA;
    delete []equaCoeffB;
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

void EncRCGOP::xCalEquaCoeff( EncRCSeq* encRCSeq, double* lambdaRatio, double* equaCoeffA, double* equaCoeffB, int GOPSize )
{
  for ( int i=0; i<GOPSize; i++ )
  {
    int frameLevel = encRCSeq->gopID2Level[ i ];
    double alpha = encRCSeq->picParam[ frameLevel ].alpha;
    double beta = encRCSeq->picParam[ frameLevel ].beta;
    equaCoeffA[ i ] = pow( 1.0 / alpha, 1.0 / beta ) * pow( lambdaRatio[ i ], 1.0 / beta );
    equaCoeffB[ i ] = 1.0 / beta;
  }
}

double EncRCGOP::xSolveEqua(EncRCSeq* encRCSeq, double targetBpp, double* equaCoeffA, double* equaCoeffB, int GOPSize)
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

  solution = Clip3(minEstLambda, maxEstLambda, solution);
  return solution;
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

  idealTargetGOPBits = (int)( GOPSize * encRCSeq->targetRate / (double)encRCSeq->frameRate );

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
  estHeaderBits       = 0;
  picQPOffsetQPA      = 0;
  picLambdaOffsetQPA  = 0.0;
  picEstLambda        = 0.0;
  lcuLeft             = 0;
  bitsLeft            = 0;
  lcu                 = NULL;
  picActualHeaderBits = 0;
  picActualBits       = 0;
  picQP               = 0;
  picLambda           = 0.0;
  picMSE              = 0.0;
  validPixelsInPic    = 0;
  isNewScene          = false;
}

EncRCPic::~EncRCPic()
{
  destroy();
}

int EncRCPic::xEstPicTargetBits( EncRCSeq* encRcSeq, EncRCGOP* encRcGOP )
{
  int targetBits        = 0;
  int GOPbitsLeft       = encRcGOP->bitsLeft;

  int currPicPosition = encRcGOP->numPics - encRcGOP->picsLeft;
  int currPicRatio    = encRcSeq->bitsRatio[ currPicPosition ];
  int totalPicRatio   = 0;
  for ( int i = currPicPosition; i < encRcGOP->numPics; i++ )
  {
    totalPicRatio += encRcSeq->bitsRatio[ i ];
  }

  targetBits  = int( ((double)GOPbitsLeft) * currPicRatio / totalPicRatio );

  if ( targetBits < 100 )
  {
    targetBits = 100;   // at least allocate 100 bits for one picture
  }

  if ( encRCSeq->framesLeft > encRCSeq->gopSize || encRCSeq->totalFrames < 1 )
  {
    targetBits = int( RC_WEIGHT_PIC_TARGET_BIT_IN_BUFFER * targetBits + RC_WEIGHT_PIC_TARGET_BIT_IN_GOP * encRCGOP->picTargetBitInGOP[ currPicPosition ] );
  }

  // bit allocation for 2-pass RC
  if ( encRcSeq->twoPass )
  {
    double gopVsBitrateRatio = 1.0;
    double frameVsGopRatio = 1.0;
    int tmpTargetBits = 0;
    double alpha[ 7 ] = { 0.0 };
    encRcSeq->getTargetBitsFromFirstPass( encRcSeq->framesCoded, tmpTargetBits, gopVsBitrateRatio, frameVsGopRatio, isNewScene, alpha );
    targetBits = int( ( encRcSeq->estimatedBitUsage - encRcSeq->bitsUsed ) * gopVsBitrateRatio * frameVsGopRatio + tmpTargetBits ); // calculate the difference of under/overspent bits and adjust the current target bits based on the gop and frame ratio for every frame

    if ( encRcSeq->bitsUsed > 0 )
    {
      encRcSeq->bitUsageRatio = double( encRcSeq->estimatedBitUsage ) / encRcSeq->bitsUsed;
    }
    encRcSeq->estimatedBitUsage += int64_t( tmpTargetBits );
    if ( isNewScene )
    {
      int bitdepthLumaScale = 2 * ( encRCSeq->bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( encRCSeq->bitDepth ) );
      int numOfLevels = int( log( encRcSeq->gopSize ) / log( 2 ) + 0.5 ) + 2;
      for ( int i = 1; i < numOfLevels; i++ )
      {
        encRCSeq->picParam[ i ].alpha = alpha[ i ] * pow( 2.0, bitdepthLumaScale );
        encRcSeq->picParam[ i ].beta = -1.367;
      }
    }
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

void EncRCPic::create( EncRCSeq* encRcSeq, EncRCGOP* encRcGOP, int frameLvl, std::list<EncRCPic*>& listPreviousPictures )
{
  destroy();
  encRCSeq = encRcSeq;
  encRCGOP = encRcGOP;

  int tgtBits    = xEstPicTargetBits( encRcSeq, encRcGOP );
  int estHeadBits = xEstPicHeaderBits( listPreviousPictures, frameLvl );

  if ( tgtBits < estHeadBits + 100 )
  {
    tgtBits = estHeadBits + 100;   // at least allocate 100 bits for picture data
  }

  frameLevel       = frameLvl;
  numberOfPixel    = encRcSeq->numberOfPixel;
  numberOfLCU      = encRcSeq->numberOfLCU;
  picEstLambda     = 100.0;
  targetBits       = tgtBits;
  estHeaderBits    = estHeadBits;
  bitsLeft         = targetBits;
  int picWidth       = encRcSeq->picWidth;
  int picHeight      = encRcSeq->picHeight;
  int LCUWidth       = encRcSeq->lcuWidth;
  int LCUHeight      = encRcSeq->lcuHeight;
  int picWidthInLCU  = ( picWidth  % LCUWidth  ) == 0 ? picWidth  / LCUWidth  : picWidth  / LCUWidth  + 1;
  int picHeightInLCU = ( picHeight % LCUHeight ) == 0 ? picHeight / LCUHeight : picHeight / LCUHeight + 1;

  lcuLeft         = numberOfLCU;
  bitsLeft       -= estHeaderBits;

  lcu = new TRCLCU[ numberOfLCU ];
  int i, j;
  int LCUIdx;
  for ( i=0; i<picWidthInLCU; i++ )
  {
    for ( j=0; j<picHeightInLCU; j++ )
    {
      LCUIdx = j*picWidthInLCU + i;
      lcu[ LCUIdx ].actualBits = 0;
      lcu[ LCUIdx ].actualSSE = 0.0;
      lcu[ LCUIdx ].actualMSE = 0.0;
      lcu[ LCUIdx ].QP = 0;
      lcu[ LCUIdx ].lambda = 0.0;
      lcu[ LCUIdx ].targetBits = 0;
      lcu[ LCUIdx ].bitWeight = 1.0;
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

  switch ( encRCSeq->rcMode )
  {
  case 1:
    clipLambdaConventional( listPreviousPictures, estLambda, bitdepthLumaScale );
    break;
  case 2:
    if ( encRCSeq->twoPass )
    {
      clipLambdaTwoPass( listPreviousPictures, estLambda, bitdepthLumaScale );
    }
    else
    {
      clipLambdaFrameRc( listPreviousPictures, estLambda, bitdepthLumaScale );
    }
    break;
  case 3:
    clipLambdaGopRc( listPreviousPictures, estLambda, bitdepthLumaScale );
    break;
  default:
    clipLambdaConventional( listPreviousPictures, estLambda, bitdepthLumaScale );
    break;
  }

  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = double( int64_t( estLambda * (double)RC_LAMBDA_PREC + 0.5 ) ) / (double)RC_LAMBDA_PREC;
  picEstLambda = estLambda;

  double totalWeight = 0.0;
  // initial BU bit allocation weight
  for ( int i = 0; i < numberOfLCU; i++ )
  {
    double alphaLCU, betaLCU;
    if ( encRCSeq->useLCUSeparateModel )
    {
      alphaLCU = encRCSeq->lcuParam[ frameLevel][ i ].alpha;
      betaLCU  = encRCSeq->lcuParam[ frameLevel][ i ].beta;
    }
    else
    {
      alphaLCU = encRCSeq->picParam[ frameLevel ].alpha;
      betaLCU  = encRCSeq->picParam[ frameLevel ].beta;
    }

    lcu[ i ].bitWeight = lcu[ i ].numberOfPixel * pow( estLambda / alphaLCU, 1.0 / betaLCU );

    if ( lcu[ i ].bitWeight < 0.01 )
    {
      lcu[ i ].bitWeight = 0.01;
    }
    totalWeight += lcu[ i ].bitWeight;
  }
  for ( int i = 0; i < numberOfLCU; i++ )
  {
    double BUTargetBits = targetBits * lcu[ i ].bitWeight / totalWeight;
    lcu[ i ].bitWeight = BUTargetBits;
  }

  return estLambda;
}

void EncRCPic::clipLambdaConventional( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale )
{
  double lastLevelLambda = -1.0;
  double lastPicLambda = -1.0;
  double lastValidLambda = -1.0;
  std::list<EncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( ( *it )->frameLevel == frameLevel )
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
    lambda = Clip3( lastLevelLambda * pow( 2.0, -3.0 / 3.0 ), lastLevelLambda * pow( 2.0, 3.0 / 3.0 ), lambda );
  }

  if ( lastPicLambda > 0.0 )
  {
    lastPicLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastPicLambda );
    lambda = Clip3( lastPicLambda * pow( 2.0, -10.0 / 3.0 ), lastPicLambda * pow( 2.0, 10.0 / 3.0 ), lambda );
  }
  else if ( lastValidLambda > 0.0 )
  {
    lastValidLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastValidLambda );
    lambda = Clip3( lastValidLambda * pow( 2.0, -10.0 / 3.0 ), lastValidLambda * pow( 2.0, 10.0 / 3.0 ), lambda );
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
    lambda = Clip3( lastLevelLambda * pow( 2.0, -5.0 / 3.0 ), lastLevelLambda * pow( 2.0, 5.0 / 3.0 ), lambda );
  }

  if ( frameLevel > 2 )
  {
    int tlQpOffset = encRCSeq->gopSize == 32 ? RC_GOP_ID_QP_OFFSET_GOP32[ frameLevel ] : RC_GOP_ID_QP_OFFSET[ frameLevel ];
    lambda = Clip3( lastPrevTLLambda * pow( 2.0, (double)( tlQpOffset ) / 3.0 ), encRCGOP->maxEstLambda, lambda );
  }

  if ( lastPicLambda > 0.0 )
  {
    lastPicLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastPicLambda );
    if ( frameLevel > 1 )
    {
      lambda = Clip3( lastPicLambda * pow( 2.0, -10.0 / 3.0 ), lastPicLambda * pow( 2.0, 10.0 / 3.0 ), lambda );
    }
    else
    {
      lambda = Clip3( lastPicLambda * pow( 2.0, -12.0 / 3.0 ), lastPicLambda * pow( 2.0, 12.0 / 3.0 ), lambda );
    }
  }
  else if ( lastValidLambda > 0.0 )
  {
    lastValidLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastValidLambda );
    if ( frameLevel > 1 )
    {
      lambda = Clip3( lastValidLambda * pow( 2.0, -10.0 / 3.0 ), lastValidLambda * pow( 2.0, 10.0 / 3.0 ), lambda );
    }
    else
    {
      lambda = Clip3( lastValidLambda * pow( 2.0, -12.0 / 3.0 ), lastValidLambda * pow( 2.0, 12.0 / 3.0 ), lambda );
    }
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

void EncRCPic::clipLambdaGopRc( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale )
{
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
    if ( frameLevel < 1 )
    {
      double lowerClippingBoundary = Clip3( -5.0, 5.0, round( -10 * ( encRCGOP->targetBits / (double)encRCGOP->idealTargetGOPBits ) + 5 ) );
      double upperClippingBoundary = Clip3( -5.0, 5.0, round( -10 * ( encRCGOP->targetBits / (double)encRCGOP->idealTargetGOPBits ) + 11 ) );
      lambda = Clip3( lastLevelLambda * pow( 2.0, lowerClippingBoundary / 3.0 ), lastLevelLambda * pow( 2.0, upperClippingBoundary / 3.0 ), lambda );
    }
    else
    {
      lambda = Clip3( lastLevelLambda * pow( 2.0, -5.0 / 3.0 ), lastLevelLambda * pow( 2.0, 5.0 / 3.0 ), lambda );
    }
  }

  if ( ( encRCSeq->framesCoded < encRCSeq->intraPeriod && frameLevel > 1 ) || ( encRCSeq->framesCoded >= encRCSeq->intraPeriod && frameLevel > 0 ) )
  {
    int tlQpOffset = encRCSeq->gopSize == 32 ? RC_GOP_ID_QP_OFFSET_GRC_GOP32[ frameLevel ] : RC_GOP_ID_QP_OFFSET_GRC[ frameLevel ];
    lambda = Clip3( lastPrevTLLambda * pow( 2.0, (double)( tlQpOffset ) / 3.0 ), encRCGOP->maxEstLambda, lambda );
  }

  if ( lastPicLambda > 0.0 )
  {
    lastPicLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastPicLambda );
    if ( frameLevel > 1 )
    {
      lambda = Clip3( lastPicLambda * pow( 2.0, -10.0 / 3.0 ), lastPicLambda * pow( 2.0, 10.0 / 3.0 ), lambda );
    }
    else
    {
      double limitTH[ 3 ] = { pow( 2.0, bitdepthLumaScale ) * exp( ( 26 - 13.7122 ) / 4.2005 ), pow( 2.0, bitdepthLumaScale ) * exp( ( 30 - 13.7122 ) / 4.2005 ), pow( 2.0, bitdepthLumaScale ) * exp( ( 35 - 13.7122 ) / 4.2005 ) };
      if ( frameLevel == 0 )
      {
        double intraLimit = lastPicLambda < limitTH[ 0 ] ? 11.0 : ( lastPicLambda < limitTH[ 1 ] ? 12.0 : ( lastPicLambda < limitTH[ 2 ] ? 13.0 : 14.0 ) );
        lambda = Clip3( lastPicLambda * pow( 2.0, -intraLimit / 3.0 ), lastPicLambda * pow( 2.0, intraLimit / 3.0 ), lambda );
      }
      else
      {
        double intraLimit = lastPicLambda < limitTH[ 0 ] ? 8.0 : ( lastPicLambda < limitTH[ 1 ] ? 9.0 : ( lastPicLambda < limitTH[ 2 ] ? 10.0 : 11.0 ) );
        lambda = Clip3( lastPicLambda * pow( 2.0, -intraLimit / 3.0 ), lastPicLambda * pow( 2.0, intraLimit / 3.0 ), lambda );
      }
    }
  }
  else if ( lastValidLambda > 0.0 )
  {
    lastValidLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastValidLambda );
    if ( frameLevel > 1 )
    {
      lambda = Clip3( lastValidLambda * pow( 2.0, -10.0 / 3.0 ), lastValidLambda * pow( 2.0, 10.0 / 3.0 ), lambda );
    }
    else
    {
      lambda = Clip3( lastValidLambda * pow( 2.0, -12.0 / 3.0 ), lastValidLambda * pow( 2.0, 12.0 / 3.0 ), lambda );
    }
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

void EncRCPic::clipLambdaTwoPass( std::list<EncRCPic*>& listPreviousPictures, double &lambda, int bitdepthLumaScale )
{
  bool setLastLevelLambda = false;
  double lastPrevTLLambda = -1.0;
  double lastLevelLambda = -1.0;
  double lastPicLambda = -1.0;
  double lastValidLambda = -1.0;
  std::list<EncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    // collect lambda from the immediately lower TL
    if ( ( *it )->frameLevel == frameLevel - 1 && ( *it )->picLambda > 0.0 )
    {
      lastPrevTLLambda = ( *it )->picLambda;
    }

    if ( ( *it )->frameLevel == frameLevel && ( *it )->picLambda > 0.0 )
    {
      lastLevelLambda = ( *it )->picLambda;
      setLastLevelLambda = true;
    }
    else if ( encRCSeq->framesCoded < encRCSeq->gopSize && ( *it )->frameLevel < frameLevel && ( *it )->picLambda > 0.0 && !setLastLevelLambda ) // at the beginning, treat frames from the lower TLs as if they are in the same TL
    {
      lastLevelLambda = ( *it )->picLambda;
    }
    lastPicLambda = ( *it )->picLambda;

    if ( lastPicLambda > 0.0 )
    {
      lastValidLambda = lastPicLambda;
    }
  }

  // limit lambda among the frames from the same TL
  if ( lastLevelLambda > 0.0 )
  {
    lastLevelLambda = Clip3( encRCGOP->minEstLambda, encRCGOP->maxEstLambda, lastLevelLambda );
    if( isNewScene )
    {
      lambda = Clip3( lastLevelLambda * pow( 2.0, -6.0 / 3.0 ), lastLevelLambda * pow( 2.0, 6.0 / 3.0 ), lambda );
    }
    else
    {
      lambda = Clip3( lastLevelLambda * pow( 2.0, -3.0 / 3.0 ), lastLevelLambda * pow( 2.0, 3.0 / 3.0 ), lambda );
    }
  }

  // prevent frames from higher TLs to have lower lambda values than frames at lower TLs
  if ( frameLevel > 2 )
  {
    int tlQpOffset = encRCSeq->gopSize == 32 ? RC_GOP_ID_QP_OFFSET_GOP32[ frameLevel ] : RC_GOP_ID_QP_OFFSET[ frameLevel ];
    if ( encRCSeq->bitUsageRatio > 1.0 )
    {
      lambda = Clip3( lastPrevTLLambda * pow( 2.0, (double)( tlQpOffset ) / 3.0 ), lastPrevTLLambda * pow( 2.0, (double)( tlQpOffset ) / 3.0 ), lambda );
    }
    else
    {
      lambda = Clip3( lastPrevTLLambda * pow( 2.0, (double)( tlQpOffset ) / 3.0 ), encRCGOP->maxEstLambda, lambda );
    }
  }

  // clip lambda based on the previously encoded picture
  if ( lastPicLambda > 0.0 )
  {
    lastPicLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastPicLambda );
    if ( frameLevel > 1 )
    {
      lambda = Clip3( lastPicLambda * pow( 2.0, -6.0 / 3.0 ), lastPicLambda * pow( 2.0, 6.0 / 3.0 ), lambda );
    }
    else // frames at the lowest TLs should have larger clipping interval
    {
      int lowClipBoundary = Clip3( 12, 15, int( 4.0 * encRCSeq->bitUsageRatio + 8.5 + 0.5 ) );
      lambda = Clip3( lastPicLambda * pow( 2.0, -lowClipBoundary / 3.0 ), lastPicLambda * pow( 2.0, 12.0 / 3.0 ), lambda );
    }
  }
  else if ( lastValidLambda > 0.0 )
  {
    lastValidLambda = Clip3( encRCGOP->minEstLambda, 2000.0 * pow( 2.0, bitdepthLumaScale ), lastValidLambda );
    if ( frameLevel > 1 )
    {
      lambda = Clip3( lastValidLambda * pow( 2.0, -6.0 / 3.0 ), lastValidLambda * pow( 2.0, 6.0 / 3.0 ), lambda );
    }
    else
    {
      int lowClipBoundary = Clip3( 12, 15, int( 4.0 * encRCSeq->bitUsageRatio + 8.5 + 0.5 ) );
      lambda = Clip3( lastValidLambda * pow( 2.0, -lowClipBoundary / 3.0 ), lastValidLambda * pow( 2.0, 12.0 / 3.0 ), lambda );
    }
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

  switch ( encRCSeq->rcMode )
  {
  case 1:
    clipQpConventional( listPreviousPictures, QP );
    break;
  case 2:
    if ( encRCSeq->twoPass )
    {
      clipQpTwoPass( listPreviousPictures, QP );
    }
    else
    {
      clipQpFrameRc( listPreviousPictures, QP );
    }
    break;
  case 3:
    clipQpGopRc( listPreviousPictures, QP );
    break;
  default:
    clipQpConventional( listPreviousPictures, QP );
    break;
  }

  return QP;
}

void EncRCPic::clipQpConventional( std::list<EncRCPic*>& listPreviousPictures, int & QP )
{
  int lastLevelQP = RC_INVALID_QP_VALUE;
  int lastPicQP = RC_INVALID_QP_VALUE;
  int lastValidQP = RC_INVALID_QP_VALUE;
  std::list<EncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( ( *it )->frameLevel == frameLevel )
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
    QP = Clip3( lastLevelQP - 3, lastLevelQP + 3, QP );
  }

  if ( lastPicQP > RC_INVALID_QP_VALUE )
  {
    QP = Clip3( lastPicQP - 10, lastPicQP + 10, QP );
  }
  else if ( lastValidQP > RC_INVALID_QP_VALUE )
  {
    QP = Clip3( lastValidQP - 10, lastValidQP + 10, QP );
  }
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
    QP = Clip3( lastLevelQP - 5, lastLevelQP + 5, QP );
  }

  if ( frameLevel > 2 )
  {
    int tlQpOffset = encRCSeq->gopSize == 32 ? RC_GOP_ID_QP_OFFSET_GOP32[ frameLevel ] : RC_GOP_ID_QP_OFFSET[ frameLevel ];
    QP = Clip3( lastPrevTLQP + tlQpOffset, MAX_QP, QP );
  }

  if ( lastPicQP > RC_INVALID_QP_VALUE )
  {
    if ( frameLevel > 1 )
    {
      QP = Clip3( lastPicQP - 10, lastPicQP + 10, QP );
    }
    else
    {
      QP = Clip3( lastPicQP - 12, lastPicQP + 12, QP );
    }
  }
  else if ( lastValidQP > RC_INVALID_QP_VALUE )
  {
    if ( frameLevel > 1 )
    {
      QP = Clip3( lastValidQP - 10, lastValidQP + 10, QP );
    }
    else
    {
      QP = Clip3( lastValidQP - 12, lastValidQP + 12, QP );
    }
  }
}

void EncRCPic::clipQpGopRc( std::list<EncRCPic*>& listPreviousPictures, int &QP )
{
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
    }
    lastPicQP = ( *it )->picQP;
    if ( lastPicQP > RC_INVALID_QP_VALUE )
    {
      lastValidQP = lastPicQP;
    }
  }

  if ( lastLevelQP > RC_INVALID_QP_VALUE )
  {
    if ( frameLevel < 1 )
    {
      int lowerClippingBoundary = Clip3( -5, 5, (int)round( -10 * ( encRCGOP->targetBits / (double)encRCGOP->idealTargetGOPBits ) + 5 ) );
      int upperClippingBoundary = Clip3( -5, 5, (int)round( -10 * ( encRCGOP->targetBits / (double)encRCGOP->idealTargetGOPBits ) + 11 ) );
      QP = Clip3( lastLevelQP + lowerClippingBoundary, lastLevelQP + upperClippingBoundary, QP );
    }
    else
    {
      QP = Clip3( lastLevelQP - 5, lastLevelQP + 5, QP );
    }
  }

  if ( ( encRCSeq->framesCoded < encRCSeq->intraPeriod && frameLevel > 1 ) || ( encRCSeq->framesCoded >= encRCSeq->intraPeriod && frameLevel > 0 ) )
  {
    int tlQpOffset = encRCSeq->gopSize == 32 ? RC_GOP_ID_QP_OFFSET_GRC_GOP32[ frameLevel ] : RC_GOP_ID_QP_OFFSET_GRC[ frameLevel ];
    QP = Clip3( lastPrevTLQP + tlQpOffset, MAX_QP, QP );
  }

  if ( lastPicQP > RC_INVALID_QP_VALUE )
  {
    if ( frameLevel > 1 )
    {
      QP = Clip3( lastPicQP - 10, lastPicQP + 10, QP );
    }
    else
    {
      if ( frameLevel == 0 )
      {
        int intraLimit = lastPicQP < 26 ? 11 : ( lastPicQP < 30 ? 12 : ( lastPicQP < 35 ? 13 : 14 ) );
        QP = Clip3( lastPicQP - intraLimit, lastPicQP + intraLimit, QP );
      }
      else
      {
        int intraLimit = lastPicQP < 26 ? 8 : ( lastPicQP < 30 ? 9 : ( lastPicQP < 35 ? 10 : 11 ) );
        QP = Clip3( lastPicQP - intraLimit, lastPicQP + intraLimit, QP );
      }
    }
  }
  else if ( lastValidQP > RC_INVALID_QP_VALUE )
  {
    if ( frameLevel > 1 )
    {
      QP = Clip3( lastValidQP - 10, lastValidQP + 10, QP );
    }
    else
    {
      QP = Clip3( lastValidQP - 12, lastValidQP + 12, QP );
    }
  }
}

void EncRCPic::clipQpTwoPass( std::list<EncRCPic*>& listPreviousPictures, int &QP )
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
    if ( isNewScene )
    {
      QP = Clip3( lastLevelQP - 6, lastLevelQP + 6, QP );
    }
    else
    {
      QP = Clip3( lastLevelQP - 3, lastLevelQP + 3, QP );
    }
  }

  if ( frameLevel > 2 ) // in any case frame level has to be GREATER than 1
  {
    int tlQpOffset = encRCSeq->gopSize == 32 ? RC_GOP_ID_QP_OFFSET_GOP32[ frameLevel ] : RC_GOP_ID_QP_OFFSET[ frameLevel ];
    if ( encRCSeq->bitUsageRatio > 1.0 )
    {
      QP = Clip3( lastPrevTLQP + tlQpOffset, lastPrevTLQP + tlQpOffset, QP );
    }
    else
    {
      QP = Clip3( lastPrevTLQP + tlQpOffset, MAX_QP, QP );
    }
  }

  if ( lastPicQP > RC_INVALID_QP_VALUE )
  {
    if ( frameLevel > 1 )
    {
      QP = Clip3( lastPicQP - 6, lastPicQP + 6, QP );
    }
    else
    {
      int lowClipBoundary = Clip3( 12, 15, int( 4.0 * encRCSeq->bitUsageRatio + 8.5 + 0.5 ) );
      QP = Clip3( lastPicQP - lowClipBoundary, lastPicQP + 12, QP );
    }
  }
  else if ( lastValidQP > RC_INVALID_QP_VALUE )
  {
    if ( frameLevel > 1 )
    {
      QP = Clip3( lastValidQP - 6, lastValidQP + 6, QP );
    }
    else
    {
      int lowClipBoundary = Clip3( 12, 15, int( 4.0 * encRCSeq->bitUsageRatio + 8.5 + 0.5 ) );
      QP = Clip3( lastValidQP - lowClipBoundary, lastValidQP + 12, QP );
    }
  }
}

double EncRCPic::getLCUTargetBpp(bool isIRAP, const int ctuRsAddr )
{
  int   LCUIdx    = ctuRsAddr;
  double bpp      = -1.0;
  int avgBits     = 0;

  if (isIRAP)
  {
    int bitrateWindow = std::min( 4, lcuLeft );
    double MAD = lcu[ LCUIdx ].costIntra;

    if ( remainingCostIntra > 0.1 )
    {
      double weightedBitsLeft = ( bitsLeft * bitrateWindow + ( bitsLeft - lcu[ LCUIdx ].targetBitsLeft ) * lcuLeft ) / (double)bitrateWindow;
      avgBits = int( MAD * weightedBitsLeft / remainingCostIntra );
    }
    else
    {
      avgBits = int( bitsLeft / lcuLeft );
    }
    remainingCostIntra -= MAD;
  }
  else
  {
    double totalWeight = 0;
    for ( int i = LCUIdx; i < numberOfLCU; i++ )
    {
      totalWeight += lcu[ i ].bitWeight;
    }
    int realInfluenceLCU = std::min( RC_LCU_SMOOTH_WINDOW_SIZE, lcuLeft );
    avgBits = (int)( lcu[ LCUIdx ].bitWeight - ( totalWeight - bitsLeft ) / realInfluenceLCU + 0.5 );
  }

  if ( avgBits < 1 )
  {
    avgBits = 1;
  }

  bpp = (double)avgBits / (double)lcu[ LCUIdx ].numberOfPixel;
  lcu[ LCUIdx ].targetBits = avgBits;

  return bpp;
}

double EncRCPic::getLCUEstLambda( double bpp, const int ctuRsAddr )
{
  int   LCUIdx = ctuRsAddr;
  double alpha;
  double beta;
  if ( encRCSeq->useLCUSeparateModel )
  {
    alpha = encRCSeq->lcuParam[ frameLevel][ LCUIdx ].alpha;
    beta  = encRCSeq->lcuParam[ frameLevel][ LCUIdx ].beta;
  }
  else
  {
    alpha = encRCSeq->picParam[ frameLevel ].alpha;
    beta  = encRCSeq->picParam[ frameLevel ].beta;
  }

  double estLambda = alpha * pow( bpp, beta );
  //for Lambda clip, picture level clip
  double clipPicLambda = picEstLambda;

  //for Lambda clip, LCU level clip
  double clipNeighbourLambda = -1.0;
  for ( int i = LCUIdx - 1; i >= 0; i-- )
  {
    if ( lcu[ i ].lambda > 0 )
    {
      clipNeighbourLambda = lcu[ i ].lambda;
      break;
    }
  }

  if ( clipNeighbourLambda > 0.0 )
  {
    estLambda = Clip3( clipNeighbourLambda * pow( 2.0, -1.0/3.0 ), clipNeighbourLambda * pow( 2.0, 1.0/3.0 ), estLambda );
  }

  if ( clipPicLambda > 0.0 )
  {
    estLambda = Clip3( clipPicLambda * pow( 2.0, -2.0/3.0 ), clipPicLambda * pow( 2.0, 2.0/3.0 ), estLambda );
  }
  else
  {
    int bitdepthLumaScale = 2 * ( encRCSeq->bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( encRCSeq->bitDepth ) );
    estLambda = Clip3(10.0 * pow(2.0, bitdepthLumaScale ), 1000.0 * pow(2.0, bitdepthLumaScale ), estLambda);
  }

  if ( estLambda < 0.1 )
  {
    estLambda = 0.1;
  }

  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = double(int64_t(estLambda * (double)RC_LAMBDA_PREC + 0.5)) / (double)RC_LAMBDA_PREC;
  return estLambda;
}

int EncRCPic::getLCUEstQP( double lambda, int clipPicQP, const int ctuRsAddr )
{
  int LCUIdx = ctuRsAddr;
  int bitdepthLumaScale = 2 * ( encRCSeq->bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( encRCSeq->bitDepth ) );

  int estQP = int( 4.2005 * log( lambda / pow( 2.0, bitdepthLumaScale ) ) + 13.7122 + 0.5 );

  //for Lambda clip, LCU level clip
  int clipNeighbourQP = RC_INVALID_QP_VALUE;
  for ( int i = LCUIdx - 1; i >= 0; i-- )
  {
    if ( lcu[ i ].QP > RC_INVALID_QP_VALUE )
    {
      clipNeighbourQP = lcu[ i ].QP;
      break;
    }
  }

  if ( clipNeighbourQP > RC_INVALID_QP_VALUE )
  {
    estQP = Clip3( clipNeighbourQP - 1, clipNeighbourQP + 1, estQP );
  }

  estQP = Clip3( clipPicQP - 2, clipPicQP + 2, estQP );

  return estQP;
}

void EncRCPic::updateAfterCTU(int LCUIdx, int bits, int QP, double lambda, double skipRatio, bool updateLCUParameter)
{
  lcu[ LCUIdx ].actualBits = bits;
  lcu[ LCUIdx ].QP = QP;
  lcu[ LCUIdx ].lambda = lambda;
  lcu[ LCUIdx ].actualSSE = lcu[ LCUIdx ].actualMSE * lcu[ LCUIdx ].numberOfPixel;

  lcuLeft--;
  bitsLeft -= bits;

  if ( !updateLCUParameter )
  {
    return;
  }

  if ( !encRCSeq->useLCUSeparateModel )
  {
    return;
  }


  int LCUActualBits = lcu[ LCUIdx ].actualBits;
  int LCUTotalPixels = lcu[ LCUIdx ].numberOfPixel;
  double bpp = (double)LCUActualBits / (double)LCUTotalPixels;
  double inputLambda = lcu[ LCUIdx ].lambda;


  TRCParameter rcPara;
  rcPara.skipRatio = skipRatio;
  if ( QP == RC_INVALID_QP_VALUE && encRCSeq->adaptiveBits == 1 )
  {
    rcPara.validPix = 0;
  }
  else
  {
    rcPara.validPix = LCUTotalPixels;
  }

  double MSE = lcu[ LCUIdx ].actualMSE;
  double updatedK = MSE > 0 ? bpp * inputLambda / MSE : 0.0;
  double updatedC = MSE / pow(bpp, -updatedK);
  rcPara.alpha = updatedC * updatedK;
  rcPara.beta = -updatedK - 1.0;

  if (MSE > 0)
  {
    rcPara.alpha = clipRcAlpha( encRCSeq->bitDepth, rcPara.alpha );
    rcPara.beta = clipRcBeta( rcPara.beta );
    encRCSeq->lcuParam[ frameLevel ][ LCUIdx ] = rcPara;
  }
}

double EncRCPic::calAverageQP()
{
  int totalQPs = 0;
  int numTotalLCUs = 0;

  int i;
  for ( i = 0; i < numberOfLCU; i++ )
  {
    if ( lcu[ i ].QP > 0 )
    {
      totalQPs += lcu[ i ].QP;
      numTotalLCUs++;
    }
  }

  double avgQP = 0.0;
  if ( numTotalLCUs == 0 )
  {
    avgQP = RC_INVALID_QP_VALUE;
  }
  else
  {
    avgQP = ( (double)totalQPs ) / ( (double)numTotalLCUs );
  }
  return avgQP;
}

double EncRCPic::calAverageLambda()
{
  double totalLambdas = 0.0;
  int numTotalLCUs = 0;

  double totalSSE = 0.0;
  int totalPixels = 0;
  int i;
  for ( i = 0; i < numberOfLCU; i++ )
  {
    if ( lcu[ i ].lambda > 0.01 )
    {
      if ( lcu[ i ].QP > 0 || encRCSeq->adaptiveBits != 1 )
      {
        validPixelsInPic += lcu[ i ].numberOfPixel;

        totalLambdas += log( lcu[ i ].lambda );
        numTotalLCUs++;
      }

      if ( lcu[ i ].QP > 0 || encRCSeq->adaptiveBits != 1 )
      {
        totalSSE += lcu[ i ].actualSSE;
        totalPixels += lcu[ i ].numberOfPixel;
       }
    }
  }

  picMSE = totalPixels > 0 ? totalSSE / (double)totalPixels : 1.0; //1.0 is useless in the following process, just to make sure the divisor not be 0
  double avgLambda;
  if( numTotalLCUs == 0 )
  {
    avgLambda = -1.0;
  }
  else
  {
    avgLambda = pow( 2.7183, totalLambdas / numTotalLCUs );
  }
  return avgLambda;
}


void EncRCPic::updateAfterPicture( int actualHeaderBits, int actualTotalBits, double averageQP, double averageLambda, bool isIRAP)
{
  picActualHeaderBits = actualHeaderBits;
  picActualBits       = actualTotalBits;
  if ( averageQP > 0.0 )
  {
    picQP             = int( averageQP + 0.5 );
  }
  else
  {
    picQP             = RC_INVALID_QP_VALUE;
  }
  picLambda           = averageLambda;
  double alpha = encRCSeq->picParam[ frameLevel ].alpha;
  double beta  = encRCSeq->picParam[ frameLevel ].beta;
  double skipRatio = 0;
  int numOfSkipPixel = 0;
  for (int LCUIdx = 0; LCUIdx < numberOfLCU; LCUIdx++)
  {
    if ( encRCSeq->useLCUSeparateModel )
    {
      numOfSkipPixel += int( encRCSeq->lcuParam[ frameLevel ][ LCUIdx ].skipRatio * lcu[ LCUIdx ].numberOfPixel );
    }
    else
    {
      numOfSkipPixel += int( encRCSeq->picParam[ frameLevel ].skipRatio * lcu[ LCUIdx ].numberOfPixel );
    }
  }
  skipRatio = (double)numOfSkipPixel / (double)numberOfPixel;

  if (isIRAP)
  {
    updateAlphaBetaIntra(&alpha, &beta);
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
      rcPara.alpha = clipRcAlpha( encRCSeq->bitDepth, rcPara.alpha );
      rcPara.beta = clipRcBeta( rcPara.beta );
      encRCSeq->picParam[ frameLevel ] = rcPara;
    }
    if ( frameLevel == 1 )
    {
      double currLambda = Clip3( encRCGOP->minEstLambda, encRCGOP->maxEstLambda, picLambda );
      double updateLastLambda = RC_WEIGHT_HISTORY_LAMBDA * encRCSeq->lastLambda + RC_WEIGHT_CURRENT_LAMBDA * currLambda;
      encRCSeq->lastLambda = updateLastLambda;
    }
  }
}

double EncRCPic::clipRcAlpha(const int bitdepth, const double alpha)
{
  int bitdepthLumaScale = 2 * ( bitdepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( encRCSeq->bitDepth ) );
  return Clip3( RC_ALPHA_MIN_VALUE, RC_ALPHA_MAX_VALUE * pow( 2.0, bitdepthLumaScale ), alpha );
}

double EncRCPic::clipRcBeta(const double beta)
{
  return Clip3( RC_BETA_MIN_VALUE, RC_BETA_MAX_VALUE, beta );
}

int EncRCPic::getRefineBitsForIntra( int orgBits )
{
  double alpha=0.25, beta=0.5582;
  int iIntraBits;

  if (orgBits * 40 < numberOfPixel)
  {
    alpha=0.25;
  }
  else
  {
    alpha=0.30;
  }

  iIntraBits = (int)( alpha * pow( totalCostIntra * 4.0 / (double)orgBits, beta ) * (double)orgBits + 0.5 );

  return iIntraBits;
}

double EncRCPic::calculateLambdaIntra(double alpha, double beta, double MADPerPixel, double bitsPerPixel)
{
  return ( (alpha/256.0) * pow( MADPerPixel/bitsPerPixel, beta ) );
}

void EncRCPic::updateAlphaBetaIntra(double *alpha, double *beta)
{
  double lnbpp = log( pow( totalCostIntra / (double)numberOfPixel, RC_BETA1 ) ) - log( (double)picActualBits / (double)numberOfPixel );
  double diffLambda = ( *beta ) * ( log( (double)picActualBits ) - log( (double)targetBits ) );

  diffLambda = Clip3(-0.125, 0.125, 0.25*diffLambda);
  *alpha    =  (*alpha) * exp(diffLambda);
  *beta     =  (*beta) + diffLambda / lnbpp;
}


void EncRCPic::getLCUInitTargetBits()
{
  int iAvgBits     = 0;

  remainingCostIntra = totalCostIntra;
  for ( int i = numberOfLCU - 1; i >= 0; i-- )
  {
    iAvgBits += int( targetBits * lcu[ i ].costIntra / totalCostIntra );
    lcu[ i ].targetBitsLeft = iAvgBits;
  }
}


double EncRCPic::getLCUEstLambdaAndQP(double bpp, int clipPicQP, int *estQP, const int ctuRsAddr )
{
  int   LCUIdx = ctuRsAddr;

  double   alpha = encRCSeq->picParam[ frameLevel ].alpha;
  double   beta  = encRCSeq->picParam[ frameLevel ].beta;

  double costPerPixel = lcu[ LCUIdx ].costIntra / (double)lcu[ LCUIdx ].numberOfPixel;
  costPerPixel = pow( costPerPixel, RC_BETA1 );
  double estLambda = calculateLambdaIntra( alpha, beta, costPerPixel, bpp );

  int clipNeighbourQP = RC_INVALID_QP_VALUE;
  for ( int i = LCUIdx - 1; i >= 0; i-- )
  {
    if ( lcu[ i ].QP > RC_INVALID_QP_VALUE )
    {
      clipNeighbourQP = lcu[ i ].QP;
      break;
    }
  }

  int minQP = clipPicQP - 2;
  int maxQP = clipPicQP + 2;

  if ( clipNeighbourQP > RC_INVALID_QP_VALUE )
  {
    maxQP = std::min( clipNeighbourQP + 1, maxQP );
    minQP = std::max( clipNeighbourQP - 1, minQP );
  }

  int bitdepthLumaScale = 2 * ( encRCSeq->bitDepth - 8 - DISTORTION_PRECISION_ADJUSTMENT( encRCSeq->bitDepth ) );

  double maxLambda = exp( ( (double)( maxQP + 0.49 ) - 13.7122 ) / 4.2005 ) * pow( 2.0, bitdepthLumaScale );
  double minLambda = exp( ( (double)( minQP - 0.49 ) - 13.7122 ) / 4.2005 ) * pow( 2.0, bitdepthLumaScale );

  estLambda = Clip3(minLambda, maxLambda, estLambda);

  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = double( int64_t( estLambda * (double)RC_LAMBDA_PREC + 0.5 ) ) / (double)RC_LAMBDA_PREC;
  *estQP = int( 4.2005 * log( estLambda / pow( 2.0, bitdepthLumaScale ) ) + 13.7122 + 0.5 );
  *estQP = Clip3( minQP, maxQP, *estQP );

  return estLambda;
}

RateCtrl::RateCtrl()
{
  encRCSeq      = NULL;
  encRCGOP      = NULL;
  encRCPic      = NULL;
  rcQP          = 0;
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

void RateCtrl::init( int RCMode, int totalFrames, int targetBitrate, int frameRate, int intraPeriod, int GOPSize, int picWidth, int picHeight, int LCUWidth, int LCUHeight, int bitDepth, int keepHierBits, bool useLCUSeparateModel, const GOPEntry  GOPList[ MAX_GOP ] )
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
  if ( keepHierBits > 0 || ( !isLowdelay && ( GOPSize == 32 || GOPSize == 16 || GOPSize == 8 ) ) )
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

  if ( keepHierBits > 0 )
  {
    double bpp = (double)( targetBitrate / (double)( frameRate * picWidth * picHeight ) );
    if ( GOPSize == 4 && isLowdelay )
    {
      if ( bpp > 0.2 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 6;
      }
      else if( bpp > 0.1 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 10;
      }
      else if ( bpp > 0.05 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 12;
      }
      else
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 14;
      }

      if ( keepHierBits == 2 )
      {
        adaptiveBit = 1;
      }
    }
    else if ( GOPSize == 8 && !isLowdelay )
    {
      if ( bpp > 0.2 )
      {
        bitsRatio[0] = 15;
        bitsRatio[1] = 5;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else if ( bpp > 0.1 )
      {
        bitsRatio[0] = 20;
        bitsRatio[1] = 6;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else if ( bpp > 0.05 )
      {
        bitsRatio[0] = 25;
        bitsRatio[1] = 7;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else
      {
        bitsRatio[0] = 30;
        bitsRatio[1] = 8;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }

      if ( keepHierBits == 2 )
      {
        adaptiveBit = 2;
      }
    }
    else if ( GOPSize == 16 && !isLowdelay )
    {
      bitsRatio[ 0 ] = (int)( ( -0.5691 * bpp + 0.3577 ) * 1000 + 0.5 );
      bitsRatio[ 1 ] = (int)( ( -0.0332 * bpp + 0.1782 ) * 1000 + 0.5 );
      bitsRatio[ 2 ] = (int)( ( 0.0595 * bpp + 0.0810 ) * 1000 + 0.5 );
      bitsRatio[ 3 ] = (int)( ( 0.0710 * bpp + 0.0392 ) * 1000 + 0.5 );
      bitsRatio[ 4 ] = (int)( ( 0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
      bitsRatio[ 5 ] = (int)( ( 0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
      bitsRatio[ 6 ] = (int)( ( 0.0710 * bpp + 0.0392 ) * 1000 + 0.5 );
      bitsRatio[ 7 ] = (int)( ( 0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
      bitsRatio[ 8 ] = (int)( ( 0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
      bitsRatio[ 9 ] = (int)( ( 0.0595 * bpp + 0.0810 ) * 1000 + 0.5 );
      bitsRatio[ 10 ] = (int)( ( 0.0710 * bpp + 0.0392 ) * 1000 + 0.5 );
      bitsRatio[ 11 ] = (int)( ( 0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
      bitsRatio[ 12 ] = (int)( ( 0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
      bitsRatio[ 13 ] = (int)( ( 0.0710 * bpp + 0.0392 ) * 1000 + 0.5 );
      bitsRatio[ 14 ] = (int)( ( 0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );
      bitsRatio[ 15 ] = (int)( ( 0.0249 * bpp + 0.0181 ) * 1000 + 0.5 );

      if (keepHierBits == 2)
      {
        adaptiveBit = 2;
      }
    }
    else if ( GOPSize == 32 && !isLowdelay )
    {
      int bitsRatioInit[ 4 ][ 6 ] = {
        { 16, 10, 8, 4, 2, 1 },
        { 16, 10, 8, 4, 2, 1 },
        { 16, 10, 8, 4, 2, 1 },
        { 10, 8, 6, 4, 2, 1 }};
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
      int index[ 32 ] = { 0, 1, 2, 3, 4, 5, 5, 4, 5, 5, 3, 4, 5, 5, 4, 5, 5, 2, 3, 4, 5, 5, 4, 5, 5, 3, 4, 5, 5, 4, 5, 5 };

      for ( int i = 0; i < 32; i++ )
      {
        bitsRatio[ i ] = bitsRatioInit[ cls ][ index[ i ] ];
      }
      if ( keepHierBits == 2 )
      {
        adaptiveBit = 2;
      }
    }
    else
    {
      msg( WARNING, "\n hierarchical bit allocation is not support for the specified coding structure currently.\n" );
    }
  }

  int* GOPID2Level = new int[ GOPSize ];
  for ( int i=0; i<GOPSize; i++ )
  {
    GOPID2Level[i] = 1;
    if ( !GOPList[i].m_refPic )
    {
      GOPID2Level[i] = 2;
    }
  }

  if ( keepHierBits > 0 )
  {
    if ( GOPSize == 4 && isLowdelay )
    {
      GOPID2Level[0] = 3;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 1;
    }
    else if ( GOPSize == 8 && !isLowdelay )
    {
      GOPID2Level[0] = 1;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 4;
      GOPID2Level[4] = 4;
      GOPID2Level[5] = 3;
      GOPID2Level[6] = 4;
      GOPID2Level[7] = 4;
    }
    else if (GOPSize == 16 && !isLowdelay)
    {
      GOPID2Level[0] = 1;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 4;
      GOPID2Level[4] = 5;
      GOPID2Level[5] = 5;
      GOPID2Level[6] = 4;
      GOPID2Level[7] = 5;
      GOPID2Level[8] = 5;
      GOPID2Level[9] = 3;
      GOPID2Level[10] = 4;
      GOPID2Level[11] = 5;
      GOPID2Level[12] = 5;
      GOPID2Level[13] = 4;
      GOPID2Level[14] = 5;
      GOPID2Level[15] = 5;
    }
  }

  if ( !isLowdelay && GOPSize == 8 )
  {
    GOPID2Level[0] = 1;
    GOPID2Level[1] = 2;
    GOPID2Level[2] = 3;
    GOPID2Level[3] = 4;
    GOPID2Level[4] = 4;
    GOPID2Level[5] = 3;
    GOPID2Level[6] = 4;
    GOPID2Level[7] = 4;
  }
  else if (GOPSize == 16 && !isLowdelay)
  {
    GOPID2Level[0] = 1;
    GOPID2Level[1] = 2;
    GOPID2Level[2] = 3;
    GOPID2Level[3] = 4;
    GOPID2Level[4] = 5;
    GOPID2Level[5] = 5;
    GOPID2Level[6] = 4;
    GOPID2Level[7] = 5;
    GOPID2Level[8] = 5;
    GOPID2Level[9] = 3;
    GOPID2Level[10] = 4;
    GOPID2Level[11] = 5;
    GOPID2Level[12] = 5;
    GOPID2Level[13] = 4;
    GOPID2Level[14] = 5;
    GOPID2Level[15] = 5;
  }
  else if ( GOPSize == 32 && !isLowdelay )
  {
    GOPID2Level[ 0 ] = 1;
    GOPID2Level[ 1 ] = 2;
    GOPID2Level[ 2 ] = 3;
    GOPID2Level[ 3 ] = 4;
    GOPID2Level[ 4 ] = 5;
    GOPID2Level[ 5 ] = 6;
    GOPID2Level[ 6 ] = 6;
    GOPID2Level[ 7 ] = 5;
    GOPID2Level[ 8 ] = 6;
    GOPID2Level[ 9 ] = 6;
    GOPID2Level[ 10 ] = 4;
    GOPID2Level[ 11 ] = 5;
    GOPID2Level[ 12 ] = 6;
    GOPID2Level[ 13 ] = 6;
    GOPID2Level[ 14 ] = 5;
    GOPID2Level[ 15 ] = 6;
    GOPID2Level[ 16 ] = 6;
    GOPID2Level[ 17 ] = 3;
    GOPID2Level[ 18 ] = 4;
    GOPID2Level[ 19 ] = 5;
    GOPID2Level[ 20 ] = 6;
    GOPID2Level[ 21 ] = 6;
    GOPID2Level[ 22 ] = 5;
    GOPID2Level[ 23 ] = 6;
    GOPID2Level[ 24 ] = 6;
    GOPID2Level[ 25 ] = 4;
    GOPID2Level[ 26 ] = 5;
    GOPID2Level[ 27 ] = 6;
    GOPID2Level[ 28 ] = 6;
    GOPID2Level[ 29 ] = 5;
    GOPID2Level[ 30 ] = 6;
    GOPID2Level[ 31 ] = 6;
  }


  encRCSeq = new EncRCSeq;
  encRCSeq->create( RCMode, rcMaxPass == 1, totalFrames, targetBitrate, frameRate, intraPeriod, GOPSize, picWidth, picHeight, LCUWidth, LCUHeight, numberOfLevel, useLCUSeparateModel, adaptiveBit, getFirstPassStats() );
  encRCSeq->initBitsRatio( bitsRatio );
  encRCSeq->initGOPID2Level( GOPID2Level );
  encRCSeq->bitDepth = bitDepth;
  encRCSeq->initPicPara();
  if ( useLCUSeparateModel )
  {
    encRCSeq->initLCUPara();
  }

  delete[] bitsRatio;
  delete[] GOPID2Level;
}

void RateCtrl::initRCPic( int frameLevel )
{
  encRCPic = new EncRCPic;
  encRCPic->create( encRCSeq, encRCGOP, frameLevel, m_listRCPictures );
}

void RateCtrl::initRCGOP( int numberOfPictures )
{
  encRCGOP = new EncRCGOP;
  encRCGOP->create( encRCSeq, numberOfPictures );
}

void RateCtrl::destroyRCGOP()
{
  delete encRCGOP;
  encRCGOP = NULL;
}

void RateCtrl::setRCPass( int pass, int maxPass )
{
  rcPass        = pass;
  rcMaxPass     = maxPass;
  rcIsFinalPass = ( pass >= maxPass );
}

void RateCtrl::processFirstPassData()
{
  CHECK( m_listRCFirstPassStats.size() == 0, "No data available from the first pass!" );

  int numOfLevels = int( log( encRCSeq->gopSize ) / log( 2 ) + 0.5 ) + 2;

  // run a simple scene change detection
  detectNewScene();

  // process and scale GOP and frame bits using the data from the first pass to account for different target bitrates
  processGops();

  // loop though the first pass data and update alpha parameters when new scenes are detected
  std::list<TRCPassStats>::iterator it;
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++ )
  {
    if ( it->poc == 0 ) // force a new scene at the beginning
    {
      it->isNewScene = true;
      estimateAlphaFirstPass( numOfLevels, it->poc, encRCSeq->intraPeriod + 1, it->estAlpha );
    }
    else if ( it->isNewScene ) // update model parameters at every new scene
    {
      estimateAlphaFirstPass( numOfLevels, it->poc, encRCSeq->intraPeriod, it->estAlpha );
    }
  }
}

int64_t RateCtrl::getTotalBitsInFirstPass()
{
  int64_t totalBitsFirstPass = 0;

  std::list<TRCPassStats>::iterator it;
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++ )
  {
    totalBitsFirstPass += it->numBits;
  }

  return totalBitsFirstPass;
}

void RateCtrl::detectNewScene()
{
  double meanFeatureValueInter = 0.0;
  double meanFeatureValueIntra = 0.0;
  double newSceneDetectionTH = 0.1;
  double newSceneDetectionTHIntra = 0.075;
  int pocOfLastSceneChange[ 2 ] = { 0, 0 };
  int counter[ 2 ] = { 0, 0 };
  int pocOfLastCompleteGop = int( floor( ( m_listRCFirstPassStats.size() - 1 ) / encRCSeq->gopSize ) * encRCSeq->gopSize );
  int pocOfLastCompleteIp = int( floor( ( m_listRCFirstPassStats.size() - 1 ) / encRCSeq->intraPeriod ) * encRCSeq->intraPeriod );
  std::vector<double> gopFeature( 2 + pocOfLastCompleteGop / encRCSeq->gopSize );
  std::vector<double> gopFeatureIntra( 1 + pocOfLastCompleteIp / encRCSeq->intraPeriod );

  // collect the GOP features which will be used to detect scene changes
  std::list<TRCPassStats>::iterator it;
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++ )
  {
    if ( it->tempLayer == 0 )
    {
      if ( !it->isIntra )
      {
        gopFeature[ counter[ 0 ] ] = it->yPsnr / log( it->numBits );
        meanFeatureValueInter += gopFeature[ counter[ 0 ] ];
        counter[ 0 ]++;
      }
      else
      {
        gopFeatureIntra[ counter[ 1 ] ] = it->yPsnr / log( it->numBits );
        meanFeatureValueIntra += gopFeatureIntra[ counter[ 1 ] ];
        counter[ 1 ]++;
      }
    }
  }
  meanFeatureValueInter /= counter[ 0 ];
  meanFeatureValueIntra /= counter[ 1 ];

  counter[ 0 ] = 0;
  counter[ 1 ] = 0;
  // iterate through the first pass frame data to detect scene changes
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++ )
  {
    if ( it->tempLayer == 0 )
    {
      if ( !it->isIntra )
      {
        gopFeature[ counter[ 0 ] ] /= meanFeatureValueInter; // normalize GOP feature values
        if ( counter[ 0 ] > 0 )
        {
          if ( abs( gopFeature[ counter[ 0 ] ] - gopFeature[ counter[ 0 ] - 1 ] ) > newSceneDetectionTH && it->poc - encRCSeq->gopSize > pocOfLastSceneChange[ 1 ] ) // detect scene cut
          {
            it->isNewScene = true;
            pocOfLastSceneChange[ 0 ] = it->poc;
          }
        }
        counter[ 0 ]++;
      }
      else
      {
        gopFeatureIntra[ counter[ 1 ] ] /= meanFeatureValueIntra; // normalize GOP feature values
        if ( counter[ 1 ] > 0 )
        {
          if ( abs( gopFeatureIntra[ counter[ 1 ] ] - gopFeatureIntra[ counter[ 1 ] - 1 ] ) > newSceneDetectionTHIntra && it->poc - encRCSeq->intraPeriod > pocOfLastSceneChange[ 0 ] ) // detect scene cut
          {
            it->isNewScene = true;
            pocOfLastSceneChange[ 1 ] = it->poc;
          }
        }
        counter[ 1 ]++;
      }
    }
  }
}

void RateCtrl::processGops()
{
  int iterationCounter = 0;
  double actualBitrateAfterScaling = -1.0;
  int pocOfLastCompleteGop = int( floor( ( m_listRCFirstPassStats.size() - 1 ) / encRCSeq->gopSize ) * encRCSeq->gopSize );
  std::vector<int> gopBits( 2 + pocOfLastCompleteGop / encRCSeq->gopSize ); // +2 for the first I frame (GOP) and a potential last incomplete GOP
  std::vector<double> scaledBits( int( m_listRCFirstPassStats.size() ) );

  // count total bits in every GOP
  std::list<TRCPassStats>::iterator it;
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++, iterationCounter++ )
  {
    if ( it->poc == 0 )
    {
      gopBits[ 0 ] = it->numBits;
    }
    else
    {
      gopBits[ 1 + ( it->poc - 1 ) / encRCSeq->gopSize ] += it->numBits;
    }
    scaledBits[ iterationCounter ] = double( it->numBits );
  }

  // scale GOP and frame bits to account for different target bitrates besed on the first pass data
  scaleGops( scaledBits, gopBits, actualBitrateAfterScaling );

  // calculate frame and GOP ratios; calculate target bits
  iterationCounter = 0;
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++, iterationCounter++ )
  {
    it->scaledBits = scaledBits[ iterationCounter ];
    if ( it->poc == 0 ) // distinguish between the first frame and the rest of the sequence
    {
      it->frameInGopRatio = it->scaledBits / gopBits[ 0 ];
      it->gopBitsVsBitrate = double( gopBits[ 0 ] ) / actualBitrateAfterScaling;
    }
    else
    {
      it->frameInGopRatio = it->scaledBits / gopBits[ 1 + ( it->poc - 1 ) / encRCSeq->gopSize ];
      it->gopBitsVsBitrate = double( gopBits[ 1 + ( it->poc - 1 ) / encRCSeq->gopSize ] ) / actualBitrateAfterScaling;
    }
    it->targetBits = int( it->frameInGopRatio * it->gopBitsVsBitrate * encRCSeq->targetRate + 0.5 );
  }
}

void RateCtrl::scaleGops( std::vector<double> &scaledBits, std::vector<int> &gopBits, double &actualBitrateAfterScaling )
{
  int64_t totalBitsInFirstPass = getTotalBitsInFirstPass();
  double averageBitrateFirstPass = double( totalBitsInFirstPass ) / m_listRCFirstPassStats.size() * encRCSeq->frameRate;
  double scalingParameter = 0.175; // tuning parameter
  double gopScalingFactor = 1.0 - scalingParameter * log( encRCSeq->targetRate / averageBitrateFirstPass ) / log( 2 );
  double frameScalingFactor = 1.0;
  int pocOfLastCompleteGop = int( floor( ( m_listRCFirstPassStats.size() - 1 ) / encRCSeq->gopSize ) * encRCSeq->gopSize );
  int pocOfLastCompleteIp = int( floor( ( m_listRCFirstPassStats.size() - 1 ) / encRCSeq->intraPeriod ) * encRCSeq->intraPeriod );
  int gopsInIntraPeriod = encRCSeq->intraPeriod / encRCSeq->gopSize;

  // scale the first frame in the sequence
  scaledBits[ 0 ] *= gopScalingFactor;

  // scale GOP and frame bits to account for different target rates
  for ( int i = 0; i < pocOfLastCompleteIp / encRCSeq->gopSize; i += gopsInIntraPeriod )
  {
    double meanGopBits = 0.0;
    // iterate through GOPs inside an IP to calculate an average GOP bits within one IP
    for ( int j = 0; j < gopsInIntraPeriod; j++ )
    {
      meanGopBits += double( gopBits[ 1 + i + j ] );
    }
    meanGopBits /= gopsInIntraPeriod;

    // scale GOP bits in an IP
    for ( int j = 0; j < gopsInIntraPeriod; j++ )
    {
      double tmpGopScaledBits = std::max( 2000.0, ( gopBits[ 1 + i + j ] - meanGopBits ) * gopScalingFactor + meanGopBits );
      frameScalingFactor = tmpGopScaledBits / gopBits[ 1 + i + j ];
      gopBits[ 1 + i + j ] = tmpGopScaledBits;
      for ( int k = 0; k < encRCSeq->gopSize; k++ ) // scale frame bits inside the scaled GOP
      {
        scaledBits[ 1 + ( i + j ) * encRCSeq->gopSize + k ] *= frameScalingFactor;
      }
    }
  }

  // scale the potentially incomplete last IP (but not the last incomplete GOP!)
  if ( pocOfLastCompleteGop != pocOfLastCompleteIp )
  {
    double meanGopBits = 0.0;
    for ( int i = pocOfLastCompleteIp / encRCSeq->gopSize; i < pocOfLastCompleteGop / encRCSeq->gopSize; i++ )
    {
      meanGopBits += double( gopBits[ 1 + i ] );
    }
    meanGopBits /= ( ( pocOfLastCompleteGop - pocOfLastCompleteIp ) / encRCSeq->gopSize );

    // scale bits for the last incomplete IP
    for ( int i = pocOfLastCompleteIp / encRCSeq->gopSize; i < pocOfLastCompleteGop / encRCSeq->gopSize; i++ )
    {
      double tmpGopScaledBits = std::max( 2000.0, ( gopBits[ 1 + i ] - meanGopBits ) * gopScalingFactor + meanGopBits );
      frameScalingFactor = tmpGopScaledBits / gopBits[ 1 + i ];
      gopBits[ 1 + i ] = tmpGopScaledBits;
      for ( int j = 0; j < encRCSeq->gopSize; j++ ) //scale frame bits inside the scaled GOP
      {
        scaledBits[ 1 + i * encRCSeq->gopSize + j ] *= frameScalingFactor;
      }
    }
  }

  // make sure that the total scaled frame bits match the average bitrate
  int64_t totalScaledBits = 0;
  for ( int i = 0; i < m_listRCFirstPassStats.size(); i++ )
  {
    totalScaledBits += int64_t( scaledBits[ i ] );
  }
  actualBitrateAfterScaling = double( totalScaledBits ) / m_listRCFirstPassStats.size() * encRCSeq->frameRate;
}

void RateCtrl::estimateAlphaFirstPass( int numTempLevels, int startPoc, int pocRange, double *alphaEstimate )
{
  std::vector<int> bitsData( numTempLevels );
  std::vector<int> qpData( numTempLevels );
  std::vector<int> counter( numTempLevels );
  int iterationCounter = 0;

  // collect the first pass TL data for the specified POC range
  std::list<TRCPassStats>::iterator it;
  for ( it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++ )
  {
    if ( it->poc == startPoc )
    {
      if ( it->isIntra )
      {
        bitsData[ it->tempLayer ] += it->numBits;
        qpData[ it->tempLayer ] = it->qp;
        counter[ it->tempLayer ]++;
      }
      else
      {
        bitsData[ it->tempLayer + 1 ] += it->numBits;
        qpData[ it->tempLayer + 1 ] = it->qp;
        counter[ it->tempLayer + 1 ]++;
      }
      iterationCounter++;
    }
    else if ( iterationCounter > 0 && iterationCounter < pocRange )
    {
      if ( it->isIntra )
      {
        bitsData[ it->tempLayer ] += it->numBits;
        qpData[ it->tempLayer ] = it->qp;
        counter[ it->tempLayer ]++;
      }
      else
      {
        bitsData[ it->tempLayer + 1 ] += it->numBits;
        qpData[ it->tempLayer + 1 ] = it->qp;
        counter[ it->tempLayer + 1 ]++;
      }
      iterationCounter++;
    }
    else if ( iterationCounter >= pocRange )
    {
      break;
    }
  }

  // calculate alpha parameter based on the collected first pass data
  for ( int i = 0; i < numTempLevels; i++ )
  {
    if ( counter[ i ] > 0 )
    {
      double bpp = ( double( bitsData[ i ] ) / counter[ i ] ) / ( encRCSeq->picWidth * encRCSeq->picHeight );
      alphaEstimate[ i ] = exp( ( qpData[ i ] - 13.7122 ) / 4.2005 ) / pow( bpp, -1.367 );
    }
    else
    {
      alphaEstimate[ i ] = 0.0;
    }
  }
}

void RateCtrl::addRCPassStats( int poc, int qp, uint32_t numBits, double yPsnr, double uPsnr, double vPsnr, bool isIntra, int tempLayer )
{
  if( rcPass < rcMaxPass )
  {
    m_listRCFirstPassStats.push_back( TRCPassStats( poc, qp, numBits, yPsnr, uPsnr, vPsnr, isIntra, tempLayer ) );
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
      iSumHad += abs( m2[ i ][ j ] );
    }
  }
  iSumHad -= abs( m2[ 0 ][ 0 ] );
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
    lcu[ ctuRsAddr ].costIntra = cost;
    iSumHadSlice += cost;
  }

  totalCostIntra = iSumHadSlice;
}


}

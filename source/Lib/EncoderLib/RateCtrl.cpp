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
  picParam            = NULL;
  numberOfPixel       = 0;
  framesCoded         = 0;
  bitsUsed            = 0;
  framesLeft          = 0;
  bitsLeft            = 0;
  estimatedBitUsage   = 0;
  adaptiveBits        = 0;
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
  for ( int i = 0; i < gopSize; i++ )
  {
    bitsRatio[ i ] = 1;
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
    const int clipRange = (refreshParams ? 6 : std::max(3, 6 - (frameLevel >> 1)));

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
  
  if( m_cHandle.is_open() )
  {
    m_cHandle.close();
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
      bitsRatio[ 0 ] = 2;
      bitsRatio[ 1 ] = 3;
      bitsRatio[ 2 ] = 2;
      bitsRatio[ 3 ] = 6;
    }
    else if ( bpp > 0.1 )
    {
      bitsRatio[ 0 ] = 2;
      bitsRatio[ 1 ] = 3;
      bitsRatio[ 2 ] = 2;
      bitsRatio[ 3 ] = 10;
    }
    else if ( bpp > 0.05 )
    {
      bitsRatio[ 0 ] = 2;
      bitsRatio[ 1 ] = 3;
      bitsRatio[ 2 ] = 2;
      bitsRatio[ 3 ] = 12;
    }
    else
    {
      bitsRatio[ 0 ] = 2;
      bitsRatio[ 1 ] = 3;
      bitsRatio[ 2 ] = 2;
      bitsRatio[ 3 ] = 14;
    }

    adaptiveBit = 1;
  }
  else if ( GOPSize == 8 && !isLowdelay )
  {
    if ( bpp > 0.2 )
    {
      bitsRatio[ 0 ] = 15;
      bitsRatio[ 1 ] = 5;
      bitsRatio[ 2 ] = 4;
      bitsRatio[ 3 ] = 1;
      bitsRatio[ 4 ] = 1;
      bitsRatio[ 5 ] = 4;
      bitsRatio[ 6 ] = 1;
      bitsRatio[ 7 ] = 1;
    }
    else if ( bpp > 0.1 )
    {
      bitsRatio[ 0 ] = 20;
      bitsRatio[ 1 ] = 6;
      bitsRatio[ 2 ] = 4;
      bitsRatio[ 3 ] = 1;
      bitsRatio[ 4 ] = 1;
      bitsRatio[ 5 ] = 4;
      bitsRatio[ 6 ] = 1;
      bitsRatio[ 7 ] = 1;
    }
    else if ( bpp > 0.05 )
    {
      bitsRatio[ 0 ] = 25;
      bitsRatio[ 1 ] = 7;
      bitsRatio[ 2 ] = 4;
      bitsRatio[ 3 ] = 1;
      bitsRatio[ 4 ] = 1;
      bitsRatio[ 5 ] = 4;
      bitsRatio[ 6 ] = 1;
      bitsRatio[ 7 ] = 1;
    }
    else
    {
      bitsRatio[ 0 ] = 30;
      bitsRatio[ 1 ] = 8;
      bitsRatio[ 2 ] = 4;
      bitsRatio[ 3 ] = 1;
      bitsRatio[ 4 ] = 1;
      bitsRatio[ 5 ] = 4;
      bitsRatio[ 6 ] = 1;
      bitsRatio[ 7 ] = 1;
    }

    adaptiveBit = 2;
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

    adaptiveBit = 2;
  }
  else if ( GOPSize == 32 && !isLowdelay )
  {
    int bitsRatioInit[ 4 ][ 6 ] = {
      { 16, 10, 8, 4, 2, 1 },
      { 16, 10, 8, 4, 2, 1 },
      { 16, 10, 8, 4, 2, 1 },
      { 10, 8, 6, 4, 2, 1 } };
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
    adaptiveBit = 2;
  }
  else
  {
    msg( VVENC_WARNING, "\n hierarchical bit allocation is not currently supported for the specified coding structure.\n" );
  }

  encRCSeq = new EncRCSeq;
  encRCSeq->create( rcMaxPass == 1, totalFrames, targetBitrate, frameRate, intraPeriod, GOPSize, picWidth, picHeight, LCUWidth, LCUHeight, numberOfLevel, adaptiveBit, getFirstPassStats() );
  encRCSeq->initBitsRatio( bitsRatio );
  encRCSeq->bitDepth = bitDepth;
  if (rcMaxPass <= 0) encRCSeq->initPicPara();
  encRCSeq->fppParFrames = maxParallelFrames;

  delete[] bitsRatio;
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

void RateCtrl::printFirstpassStats()
{
  printf("\n\n");

  std::list<TRCPassStats>::iterator it;
  for (it = m_listRCFirstPassStats.begin(); it != m_listRCFirstPassStats.end(); it++)
  {
    printf("\npoc: %d  qp: %d  tempLayer: %d  refreshParameters: %d  targetBits: %d  lamda: %lf  isNewScene: %d  frameInGopRatio: %lf", it->poc, it->qp, it->tempLayer, it->refreshParameters, it->targetBits, it->lambda, it->isNewScene, it->frameInGopRatio );
  }

  printf("\n\n");
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
    it->isNewScene = ((it->visActY * 64 > visActPrev * 181) || (it->isIntra && it->visActY > visActPrev && std::abs(it->psnrY - psnrPrev) > 4.5));
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
    if( m_cHandle.is_open() )
    {
      writeToStatFile( poc, qp, lambda, visActY, numBits, psnrY, isIntra, tempLayer );
    }
    else
    {
      m_listRCFirstPassStats.push_back( TRCPassStats( poc, qp, lambda, visActY,
                                                      numBits, psnrY, isIntra, tempLayer ) );
    }
  }
}

void RateCtrl::writeToStatFile( const int poc, const int qp, const double lambda, const uint16_t visActY,
                                uint32_t numBits, double yPsnr, bool isIntra, int tempLayer )
{
  if( rcPass < rcMaxPass && m_cHandle.is_open() )
  {
    m_cHandle.write( (char*) &poc,       sizeof(int) );
    m_cHandle.write( (char*) &qp,        sizeof(int) );
    m_cHandle.write( (char*) &lambda,    sizeof(double) );
    m_cHandle.write( (char*) &visActY,   sizeof(uint16_t) );
    m_cHandle.write( (char*) &numBits,   sizeof(uint32_t) );
    m_cHandle.write( (char*) &yPsnr,     sizeof(double) );
    m_cHandle.write( (char*) &isIntra,   sizeof(bool) );
    m_cHandle.write( (char*) &tempLayer, sizeof(int) );
  }
}

void RateCtrl::readFirstPassDataFromFile( const std::string &fileName )
{
  m_cHandle.open( fileName.c_str(), std::ios::binary | std::ios::in );

  CHECK( !rcIsFinalPass || m_cHandle.fail(), "something went wrong reading the first pass RC data!" );

  m_listRCFirstPassStats.clear();
  
  bool eof = false;
  while( !eof )
  {
    eof = xReadData( eof );
  }
  
  m_cHandle.close();
}

bool RateCtrl::xReadData( bool &isEof )
{
  int poc, qp, tempLayer;
  double lambda, yPsnr;
  bool isIntra;
  uint16_t visActY;
  uint32_t numBits;

  m_cHandle.read( (char*) &poc,       sizeof(int) );
  m_cHandle.read( (char*) &qp,        sizeof(int) );
  m_cHandle.read( (char*) &lambda,    sizeof(double) );
  m_cHandle.read( (char*) &visActY,   sizeof(uint16_t) );
  m_cHandle.read( (char*) &numBits,   sizeof(uint32_t) );
  m_cHandle.read( (char*) &yPsnr,     sizeof(double) );
  m_cHandle.read( (char*) &isIntra,   sizeof(bool) );
  m_cHandle.read( (char*) &tempLayer, sizeof(int) );

  if( !m_cHandle.eof() )
  {
    m_listRCFirstPassStats.push_back( TRCPassStats( poc, qp, lambda, visActY,
                                                    numBits, yPsnr, isIntra, tempLayer ) );
  }
  
  if( m_cHandle.eof() || m_cHandle.fail() )
  {
    isEof = true;
  }
  
  return isEof;
}
  
void RateCtrl::openRCstatFile( const std::string &fileName )
{
  CHECK( rcIsFinalPass, "trying to write 1st pass RC data in final pass!" );
  m_cHandle.open( fileName.c_str(), std::ios::binary | std::ios::out );
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

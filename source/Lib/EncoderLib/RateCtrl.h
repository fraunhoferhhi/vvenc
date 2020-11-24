/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */
/** \file     RateCtrl.h
    \brief    Rate control manager class
*/

#ifndef __ENCRATECTRL__
#define __ENCRATECTRL__

#pragma once

#include "CommonLib/CommonDef.h"

#include "../../../include/vvenc/EncCfg.h"

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
    void updateAfterPic( int bits );
    void setAllBitRatio( double basicLambda, double* equaCoeffA, double* equaCoeffB );
    void setQpInGOP( int gopId, int gopQp, int &qp );
    bool isQpResetRequired( int gopId );
    int  getLeftAverageBits() { CHECK( !( framesLeft > 0 ), "No frames left" ); return (int)( bitsLeft / framesLeft ); }
    void getTargetBitsFromFirstPass( int poc, int &targetBits, double &gopVsBitrateRatio, bool &isNewScene, double alpha[] );

  public:
    int             rcMode;
    bool            twoPass;
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

    void create( EncRCSeq* encRCSeq, EncRCGOP* encRCGOP, int frameLevel, std::list<EncRCPic*>& listPreviousPictures );
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
    int     bitsLeft;
    int     numberOfLCU;
    int     lcuLeft;
    int     picQPOffsetQPA;
    double  picLambdaOffsetQPA;
    double  picEstLambda;
  
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

    void init( int RCMode, int totFrames, int targetBitrate, int frameRate, int intraPeriod, int GOPSize, int picWidth, int picHeight, int LCUWidth, int LCUHeight, int bitDepth, int keepHierBits, bool useLCUSeparateModel, const GOPEntry GOPList[ MAX_GOP ] );
    void destroy();
    void initRCPic( int frameLevel );
    void initRCGOP( int numberOfPictures );
    void destroyRCGOP();

    void setRCPass( int pass, int maxPass );
    void addRCPassStats( int poc, int qp, uint32_t numBits, double yPsnr, double uPsnr, double vPsnr, bool isIntra, int tempLayer );
    void processFirstPassData();
    void estimateAlphaFirstPass( int numOfLevels, int startPoc, int pocRange, double *alphaEstimate );
    void processGops();
    void scaleGops( std::vector<double> scaledBits, std::vector<int> gopBits, double &actualBitrateAfterScaling );
    int64_t getTotalBitsInFirstPass();
    void detectNewScene();

    std::list<EncRCPic*>& getPicList() { return m_listRCPictures; }
    std::list<TRCPassStats>& getFirstPassStats() { return m_listRCFirstPassStats; }

  public:
    EncRCSeq*   encRCSeq;
    EncRCGOP*   encRCGOP;
    EncRCPic*   encRCPic;
    std::mutex  rcMutex;
    int         rcQP;
    int         rcPass;
    int         rcMaxPass;
    bool        rcIsFinalPass;

  private:
    std::list<EncRCPic*>    m_listRCPictures;
    std::list<TRCPassStats> m_listRCFirstPassStats;
  };
}
#endif

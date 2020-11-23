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
/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#pragma once

#include "vvenc/EncCfg.h"
#include "CABACWriter.h"
#include "CommonLib/AdaptiveLoopFilter.h"

//! \ingroup EncoderLib
//! \{

#define ALF_CTU_PAR_TRACING ( 0 && ENABLE_TRACING )

namespace vvenc {

class EncCfg;
class NoMallocThreadPool;

struct AlfCovariance
{
  static constexpr int MaxAlfNumClippingValues = AdaptiveLoopFilter::MaxAlfNumClippingValues;
  using TE = double[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = double[MAX_NUM_ALF_LUMA_COEFF];
  using TKE = TE[AdaptiveLoopFilter::MaxAlfNumClippingValues][AdaptiveLoopFilter::MaxAlfNumClippingValues];
  using TKy = Ty[AdaptiveLoopFilter::MaxAlfNumClippingValues];

  int numCoeff;
  int numBins;
  TKy y;
  TKE E;
  double pixAcc;

  AlfCovariance() {}
  ~AlfCovariance() {}

  void create( int size, int num_bins )
  {
    numCoeff = size;
    numBins = num_bins;
    std::memset( y, 0, sizeof( y ) );
    std::memset( E, 0, sizeof( E ) );
  }

  void destroy()
  {
  }

  void reset( int num_bins = -1 )
  {
    if ( num_bins > 0 )
      numBins = num_bins;
    pixAcc = 0;
    std::memset( y, 0, sizeof( y ) );
    std::memset( E, 0, sizeof( E ) );
  }

  const AlfCovariance& operator=( const AlfCovariance& src )
  {
    numCoeff = src.numCoeff;
    numBins = src.numBins;
    std::memcpy( E, src.E, sizeof( E ) );
    std::memcpy( y, src.y, sizeof( y ) );
    pixAcc = src.pixAcc;

    return *this;
  }
#if ENABLE_TRACING
  void trace()
  {
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        for( int j = 0; j < numCoeff; j++ )
        {
          for( int i = 0; i < numCoeff; i++ )
          {
            DTRACE( g_trace_ctx, D_ALF, "%0.2f ", E[b0][b1][j][i] );
          }
          DTRACE( g_trace_ctx, D_ALF, "\n" );
        }
        DTRACE( g_trace_ctx, D_ALF, "\n" );
      }
      DTRACE( g_trace_ctx, D_ALF, "\n" );
    }
    DTRACE( g_trace_ctx, D_ALF, "\n" );
    for( int b = 0; b < numBins; b++ )
    {
      for( int j = 0; j < numCoeff; j++ )
      {
        DTRACE( g_trace_ctx, D_ALF, "%0.2f ", y[b][j] );
      }
      DTRACE( g_trace_ctx, D_ALF, "\n" );
    }
    DTRACE( g_trace_ctx, D_ALF, "PixAcc=%f\n", pixAcc );
  }
#endif

  void add( const AlfCovariance& lhs, const AlfCovariance& rhs )
  {
    numCoeff = lhs.numCoeff;
    numBins = lhs.numBins;
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        for( int j = 0; j < numCoeff; j++ )
        {
          for( int i = 0; i < numCoeff; i++ )
          {
            E[b0][b1][j][i] = lhs.E[b0][b1][j][i] + rhs.E[b0][b1][j][i];
          }
        }
      }
    }
    for( int b = 0; b < numBins; b++ )
    {
      for( int j = 0; j < numCoeff; j++ )
      {
        y[b][j] = lhs.y[b][j] + rhs.y[b][j];
      }
    }
    pixAcc = lhs.pixAcc + rhs.pixAcc;
  }

  const AlfCovariance& operator+= ( const AlfCovariance& src )
  {
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        for( int j = 0; j < numCoeff; j++ )
        {
          for( int i = 0; i < numCoeff; i++ )
          {
            E[b0][b1][j][i] += src.E[b0][b1][j][i];
          }
        }
      }
    }
    for( int b = 0; b < numBins; b++ )
    {
      for( int j = 0; j < numCoeff; j++ )
      {
        y[b][j] += src.y[b][j];
      }
    }
    pixAcc += src.pixAcc;

    return *this;
  }

  const AlfCovariance& operator-= ( const AlfCovariance& src )
  {
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        for( int j = 0; j < numCoeff; j++ )
        {
          for( int i = 0; i < numCoeff; i++ )
          {
            E[b0][b1][j][i] -= src.E[b0][b1][j][i];
          }
        }
      }
    }
    for( int b = 0; b < numBins; b++ )
    {
      for( int j = 0; j < numCoeff; j++ )
      {
        y[b][j] -= src.y[b][j];
      }
    }
    pixAcc -= src.pixAcc;

    return *this;
  }

  void setEyFromClip(const int* clip, TE _E, Ty _y, int size) const
  {
    for (int k=0; k<size; k++)
    {
      _y[k] = y[clip[k]][k];
      for (int l=0; l<size; l++)
      {
        _E[k][l] = E[clip[k]][clip[l]][k][l];
      }
    }
  }

  double optimizeFilter( const int* clip, double *f, int size ) const
  {
    gnsSolveByChol( clip, f, size );
    return calculateError( clip, f );
  }

  double optimizeFilter    ( const AlfFilterShape& alfShape, int* clip, double *f, bool optimize_clip) const;
  double optimizeFilterClip( const AlfFilterShape& alfShape, int* clip) const
  {
    Ty f;
    return optimizeFilter( alfShape, clip, f, true );
  }

  double calculateError    ( const int *clip ) const;
  double calculateError    ( const int *clip, const double *coeff ) const { return calculateError(clip, coeff, numCoeff); }
  double calculateError    ( const int *clip, const double *coeff, const int numCoeff ) const;

  template<bool doClip>
  double calcDiffErrorForCoeffs( const int *clip, const int *coeff, const int numCoeff, const int numCoeffBefore, const int numCoeffAfter, const int coeffPos, const double invFactor ) const;
  template<bool doClip>
  double calcErrorForCoeffs( const int *clip, const int *coeff, const int numCoeff, const double invFactor ) const;
  double calcErrorForCcAlfCoeffs(const int16_t* coeff, const int numCoeff, const double invFactor) const;

  void getClipMax          ( const AlfFilterShape& alfShape, int *clip_max) const;
  void reduceClipCost      ( const AlfFilterShape& alfShape, int *clip) const;
  int  gnsSolveByChol              ( TE LHS, double* rhs, double *x, int numEq ) const;

private:
  // Cholesky decomposition

  int  gnsSolveByChol              ( const int *clip, double *x, int numEq ) const;
  void gnsBacksubstitution         ( TE R, double* z, int size, double* A ) const;
  void gnsTransposeBacksubstitution( TE U, double* rhs, double* x, int order ) const;
  int  gnsCholeskyDec              ( TE inpMatr, TE outMatr, int numEq ) const;
};

class EncAdaptiveLoopFilter : public AdaptiveLoopFilter
{
public:
  inline void           setAlfWSSD(int alfWSSD) { m_alfWSSD = alfWSSD; }
  static std::vector<double>  m_lumaLevelToWeightPLUT;

private:
  int m_alfWSSD;
  const EncCfg*          m_encCfg;
  AlfCovariance***       m_alfCovariance[MAX_NUM_COMP];          // [compIdx][shapeIdx][ctbAddr][classIdx]
  AlfCovariance**        m_alfCovarianceFrame[MAX_NUM_CH];   // [CHANNEL][shapeIdx][lumaClassIdx/chromaAltIdx]
  uint8_t*               m_ctuEnableFlagTmp[MAX_NUM_COMP];
  uint8_t*               m_ctuEnableFlagTmp2[MAX_NUM_COMP];
  uint8_t*               m_ctuAlternativeTmp[MAX_NUM_COMP];
  AlfCovariance***       m_alfCovarianceCcAlf[2];           // [compIdx-1][shapeIdx][ctbAddr][filterIdx]
  AlfCovariance**        m_alfCovarianceFrameCcAlf[2];      // [compIdx-1][shapeIdx][filterIdx]

  //for RDO
  AlfParam               m_alfParamTemp;
  ParameterSetMap<APS>*  m_apsMap;
  AlfCovariance          m_alfCovarianceMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES + 2];
  int                    m_alfClipMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_CtxCache;
  double                 m_lambda[MAX_NUM_COMP];

  int**                  m_filterCoeffSet; // [lumaClassIdx/chromaAltIdx][coeffIdx]
  int**                  m_filterClippSet; // [lumaClassIdx/chromaAltIdx][coeffIdx]
  int**                  m_diffFilterCoeff;
  short                  m_filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];
  unsigned               m_bitsNewFilter[MAX_NUM_CH];
  int                    m_apsIdStart;
  double                 *m_ctbDistortionFixedFilter;
  double                 *m_ctbDistortionUnfilter[MAX_NUM_COMP];
  std::vector<short>     m_alfCtbFilterSetIndexTmp;
  AlfParam               m_alfParamTempNL;
  int                    m_clipDefaultEnc[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_filterTmp[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_clipTmp[MAX_NUM_ALF_LUMA_COEFF];

  int m_apsIdCcAlfStart[2];

  short                  m_bestFilterCoeffSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
  bool                   m_bestFilterIdxEnabled[MAX_NUM_CC_ALF_FILTERS];
  uint8_t                m_bestFilterCount;
  uint8_t*               m_trainingCovControl;
  Pel*                   m_bufOrigin;
  PelBuf*                m_buf;
  uint64_t*              m_trainingDistortion[MAX_NUM_CC_ALF_FILTERS];    // for current block size
  uint64_t*              m_lumaSwingGreaterThanThresholdCount;
  uint64_t*              m_chromaSampleCountNearMidPoint;
  uint8_t*               m_filterControl;         // current iterations filter control
  uint8_t*               m_bestFilterControl;     // best saved filter control
  int                    m_reuseApsId[2];
  bool                   m_limitCcAlf;
  NoMallocThreadPool*    m_threadpool;
#if ALF_CTU_PAR_TRACING
  std::stringstream*     m_traceStreams;
#endif
public:
  EncAdaptiveLoopFilter();
  virtual ~EncAdaptiveLoopFilter() { destroy(); }
  void init                         ( const EncCfg& encCfg, CABACWriter& cabacEstimator, CtxCache& ctxCache, NoMallocThreadPool* threadpool );
  void destroy                      ();
  void initDistortion               ();
  std::vector<int> getAvaiApsIdsLuma( CodingStructure& cs, int& newApsId );
  void alfEncoderCtb                ( CodingStructure& cs, AlfParam& alfParamNewFilters, const double lambdaChromaWeight );
  void initCABACEstimator           ( Slice* pcSlice, ParameterSetMap<APS>* apsMap );
  void setApsIdStart                ( int i ) { m_apsIdStart = i; }
  int  getApsIdStart                () { return m_apsIdStart; }
  void getStatisticsCTU             ( Picture& pic, CodingStructure& cs, PelUnitBuf& recYuv, const int ctuRsAddr );

  void performCCALF                 ( Picture& pic, CodingStructure& cs );
  void deriveFilter                 ( Picture& pic, CodingStructure& cs, const double* lambdas );
  void reconstructCTU_MT            ( Picture& pic, CodingStructure& cs, int ctuRsAddr );
  void reconstructCTU               ( Picture& pic, CodingStructure& cs, const CPelUnitBuf& recBuf, int ctuRsAddr );
  void resetFrameStats              ();

private:
  void   alfEncoder              ( CodingStructure& cs, AlfParam& alfParam, const ChannelType channel, const double lambdaChromaWeight );

  void   copyAlfParam            ( AlfParam& alfParamDst, AlfParam& alfParamSrc, ChannelType channel );
  double mergeFiltersAndCost     ( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits );

  void   getFrameStats           ( ChannelType channel, int iShapeIdx );
  void   getFrameStat            ( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx );
  void   getPreBlkStats          ( AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier* classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos);
  void   calcCovariance          ( int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel* rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance);
  template < bool clipToBdry >
  void   calcLinCovariance       ( int* ELocal, const Pel* rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, int clipTopRow, int clipBotRow );
  void   deriveStatsForCcAlfFiltering(const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv, const int compIdx,
                                      const int maskStride, const uint8_t filterIdc, CodingStructure &cs);
  void   getBlkStatsCcAlf        ( AlfCovariance &alfCovariance, const AlfFilterShape &shape, const PelUnitBuf &orgYuv,
                                   const PelUnitBuf &recYuv, const UnitArea &areaDst, const UnitArea &area,
                                   const ComponentID compID, const int yPos);
  void   calcCovarianceCcAlf     ( int ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel* rec, const int stride, const AlfFilterShape& shape, int vbDistance);
  void   mergeClasses            ( const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES]);


  double getFilterCoeffAndCost   ( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, bool onlyFilterCost = false );
  double deriveFilterCoeffs      ( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam);
  int    deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters );
  double deriveCoeffQuant        ( int *filterClipp, int *filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip );
  double deriveCtbAlfEnableFlags ( CodingStructure& cs, const int iShapeIdx, ChannelType channel, const double chromaWeight,
                                   const int numClasses, const int numCoeff, double& distUnfilter );
  void   roundFiltCoeff          ( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor );
  void   roundFiltCoeffCCALF     ( int16_t *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor );

  double getDistCoeffForce0      ( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, int zeroBitsVarBin, const int numFilters);
  int    lengthUvlc              ( int uiCode );
  int    getNonFilterCoeffRate   ( AlfParam& alfParam );

  int    getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins );
  int    getCostFilterCoeff      ( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    getCostFilterClipp      ( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    lengthFilterCoeffs      ( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff );
  double getDistForce0           ( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins );
  int    getChromaCoeffRate      ( AlfParam& alfParam, int altIdx );

  double getUnfilteredDistortion ( AlfCovariance* cov, ChannelType channel );
  double getUnfilteredDistortion ( AlfCovariance* cov, const int numClasses );
  template<bool doClip>
  double getFilteredDistortion   ( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff );

  void setEnableFlag             ( AlfParam& alfSlicePara, ChannelType channel, bool val );
  void setEnableFlag             ( AlfParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags );
  void setCtuEnableFlag          ( uint8_t** ctuFlags, ChannelType channel, uint8_t val );
  void copyCtuEnableFlag         ( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel );
  void initCtuAlternativeChroma  ( uint8_t* ctuAlts[MAX_NUM_COMP] );
  void setCtuAlternativeChroma   ( uint8_t* ctuAlts[MAX_NUM_COMP], uint8_t val );
  void copyCtuAlternativeChroma  ( uint8_t* ctuAltsDst[MAX_NUM_COMP], uint8_t* ctuAltsSrc[MAX_NUM_COMP] );
  int  getMaxNumAlternativesChroma( );
  int  getCoeffRateCcAlf         ( short chromaCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], bool filterEnabled[MAX_NUM_CC_ALF_FILTERS], uint8_t filterCount, ComponentID compID);
  void deriveCcAlfFilterCoeff    ( ComponentID compID, const PelUnitBuf& recYuv, const PelUnitBuf& recYuvExt, short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx );
  void determineControlIdcValues ( CodingStructure &cs, const ComponentID compID, const PelBuf *buf, const int ctuWidthC,
                                   const int ctuHeightC, const int picWidthC, const int picHeightC,
                                   double **unfilteredDistortion, uint64_t *trainingDistortion[MAX_NUM_CC_ALF_FILTERS],
                                   uint64_t *lumaSwingGreaterThanThresholdCount,
                                   uint64_t *chromaSampleCountNearMidPoint,
                                   bool reuseFilterCoeff, uint8_t *trainingCovControl, uint8_t *filterControl,
                                   uint64_t &curTotalDistortion, double &curTotalRate,
                                   bool     filterEnabled[MAX_NUM_CC_ALF_FILTERS],
                                   uint8_t  mapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS + 1],
                                   uint8_t &ccAlfFilterCount);
  void deriveCcAlfFilter         ( CodingStructure& cs, ComponentID compID, const PelUnitBuf& orgYuv, const PelUnitBuf& tempDecYuvBuf, const PelUnitBuf& dstYuv );
  void xSetupCcAlfAPS            ( CodingStructure& cs );
  std::vector<int> getAvailableCcAlfApsIds(CodingStructure& cs, ComponentID compID);
  void countLumaSwingGreaterThanThreshold(const Pel* luma, int lumaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* lumaSwingGreaterThanThresholdCount, int lumaCountStride);
  void countChromaSampleValueNearMidPoint(const Pel* chroma, int chromaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* chromaSampleCountNearMidPoint, int chromaSampleCountNearMidPointStride);
  void getFrameStatsCcalf        ( ComponentID compIdx, int filterIdc);
  void initDistortionCcalf       ();
};

} // namespace vvenc

//! \}

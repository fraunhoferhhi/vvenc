/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#pragma once

#include "vvenc/vvencCfg.h"
#include "CABACWriter.h"
#include "CommonLib/AdaptiveLoopFilter.h"

//! \ingroup EncoderLib
//! \{

#define ALF_CTU_PAR_TRACING ( 0 && ENABLE_TRACING )
#define ALF_SINGLE_PREC_FLOAT 1

namespace vvenc {

class NoMallocThreadPool;

#if ALF_SINGLE_PREC_FLOAT
typedef float alf_float_t; 
#else
typedef double alf_float_t;
#endif

struct AlfCovariance
{
  static constexpr int MaxAlfNumClippingValues = AdaptiveLoopFilter::MaxAlfNumClippingValues;
  using TE = alf_float_t[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = alf_float_t[MAX_NUM_ALF_LUMA_COEFF];
  using TKE = TE**;
  using TKy = Ty*;

  int numCoeff;
  int numBins;
private:
  int _numBinsAlloc;
public:

  TKy y;
  TKE E;
  alf_float_t pixAcc;
  bool all0;

  AlfCovariance() : numBins( -1 ), _numBinsAlloc( -1 ), y( nullptr ), E( nullptr ), all0( true ) {}
  ~AlfCovariance() { }

  void create( int size, int num_bins )
  {
    if( y ) destroy();

    numCoeff = size;
    numBins  = _numBinsAlloc = num_bins;

    y = new Ty[_numBinsAlloc];
    E = new TE*[_numBinsAlloc];

    for( int i = 0; i < _numBinsAlloc; i++ )
    {
      E[i] = new TE[_numBinsAlloc];
    }

    // will be done be reset either way
    //std::memset( y, 0, sizeof( y ) );
    //std::memset( E, 0, sizeof( E ) );
  }

  void destroy()
  {
    delete[] y;
    y = nullptr;

    if( E )
    {
      for( int i = 0; i < _numBinsAlloc; i++ )
      {
        delete[] E[i];
        E[i] = nullptr;
      }

      delete[] E;
      E = nullptr;
    }
  }

  void reset()
  {
    pixAcc = 0;
    all0   = true;

    for( int i = 0; i < _numBinsAlloc; i++ )
    {
      for( int j = 0; j < _numBinsAlloc; j++ )
      {
        std::memset( E[i][j], 0, sizeof( TE ) );
      }

      std::memset( y[i], 0, sizeof( Ty ) );
    }
  }

  const AlfCovariance& operator=( const AlfCovariance& src )
  {
    if( _numBinsAlloc < src.numBins )
    {
      destroy();
      create( src.numCoeff, src.numBins );
    }

    numCoeff = src.numCoeff;
    numBins  = src.numBins;

    for( int i = 0; i < numBins; i++ )
    {
      for( int j = 0; j < numBins; j++ )
      {
        std::memcpy( E[i][j], src.E[i][j], sizeof( TE ) );
      }

      std::memcpy( y[i], src.y[i], sizeof( Ty ) );
    }

    pixAcc = src.pixAcc;
    all0   = src.all0;

    return *this;
  }

#if ENABLE_TRACING
  void trace()
  {
    DTRACE( g_trace_ctx, D_ALF, "E\n");
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
    DTRACE( g_trace_ctx, D_ALF, "y\n");
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
    if( _numBinsAlloc < lhs.numBins )
    {
      destroy();
      create( lhs.numCoeff, lhs.numBins );
    }

    numCoeff = lhs.numCoeff;
    numBins  = lhs.numBins;
    all0     = lhs.all0 & rhs.all0;

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
    all0   = lhs.all0 && rhs.all0;
  }

  const AlfCovariance& operator+= ( const AlfCovariance& src );

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
    all0   &= src.all0;

    return *this;
  }

  void setEyFromClip(const int* clip, TE _E, Ty _y, int size) const
  {
    for( int k = 0; k < size; k++ )
    {
      _y[k] = y[clip[k]][k];
      for( int l = 0; l < size; l++ )
      {
        _E[k][l] = E[clip[k]][clip[l]][k][l];
      }
    }
  }

  alf_float_t optimizeFilter( const int* clip, alf_float_t*f, int size ) const
  {
    gnsSolveByChol( clip, f, size );
    return calculateError( clip, f );
  }

  alf_float_t optimizeFilter    ( const AlfFilterShape& alfShape, int* clip, alf_float_t*f, bool optimize_clip) const;
  alf_float_t optimizeFilterClip( const AlfFilterShape& alfShape, int* clip) const
  {
    Ty f;
    return optimizeFilter( alfShape, clip, f, true );
  }

  alf_float_t calculateError    ( const int *clip ) const;
  alf_float_t calculateError    ( const int *clip, const alf_float_t*coeff ) const { return calculateError(clip, coeff, numCoeff); }
  alf_float_t calculateError    ( const int *clip, const alf_float_t*coeff, const int numCoeff ) const;

  template<bool doClip>
  alf_float_t calcDiffErrorForCoeffs( const int *clip, const int *coeff, const int numCoeff, const int numCoeffBefore, const int numCoeffAfter, const int coeffPos, const alf_float_t invFactor ) const;
  template<bool doClip>
  alf_float_t calcErrorForCoeffs( const int *clip, const int *coeff, const int numCoeff, const alf_float_t invFactor ) const;
  alf_float_t calcErrorForCcAlfCoeffs(const int16_t* coeff, const int numCoeff, const alf_float_t invFactor) const;

  void getClipMax          ( const AlfFilterShape& alfShape, int *clip_max) const;
  void reduceClipCost      ( const AlfFilterShape& alfShape, int *clip) const;
  int  gnsSolveByChol              ( TE LHS, alf_float_t* rhs, alf_float_t*x, int numEq ) const;

private:
  // Cholesky decomposition

  int  gnsSolveByChol              ( const int *clip, alf_float_t*x, int numEq ) const;
  void gnsBacksubstitution         ( TE R, alf_float_t* z, int size, alf_float_t* A ) const;
  void gnsTransposeBacksubstitution( TE U, alf_float_t* rhs, alf_float_t* x, int order ) const;
  int  gnsCholeskyDec              ( TE inpMatr, TE outMatr, int numEq ) const;
};

class EncAdaptiveLoopFilter : public AdaptiveLoopFilter
{
public:
  inline void            setAlfWSSD(int alfWSSD) { m_alfWSSD = alfWSSD; }
  inline std::vector<double>&
                         getLumaLevelWeightTable() { return m_lumaLevelToWeightPLUT; }

private:
  std::vector<double>    m_lumaLevelToWeightPLUT;
  int                    m_alfWSSD;
  const VVEncCfg*        m_encCfg;
  AlfCovariance**        m_alfCovariance[MAX_NUM_COMP];          // [compIdx][ctbAddr][classIdx]
  AlfCovariance*         m_alfCovarianceFrame[MAX_NUM_CH];       // [CHANNEL][lumaClassIdx/chromaAltIdx]
  uint8_t*               m_ctuEnableFlagTmp[MAX_NUM_COMP];
  uint8_t*               m_ctuEnableFlagTmp2[MAX_NUM_COMP];
  uint8_t*               m_ctuAlternativeTmp[MAX_NUM_COMP];
  AlfCovariance**        m_alfCovarianceCcAlf[2];           // [compIdx-1][ctbAddr][filterIdx]
  AlfCovariance*         m_alfCovarianceFrameCcAlf[2];      // [compIdx-1][filterIdx]

  //for RDO
  AlfParam               m_alfParamTemp;
  ParameterSetMap<APS>*  m_apsMap;
  AlfCovariance          m_alfCovarianceMerged[MAX_NUM_ALF_CLASSES + 2];
  int                    m_alfClipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_CtxCache;
  double                 m_lambda[MAX_NUM_COMP];

  int**                  m_filterCoeffSet; // [lumaClassIdx/chromaAltIdx][coeffIdx]
  int**                  m_filterClippSet; // [lumaClassIdx/chromaAltIdx][coeffIdx]
  int**                  m_diffFilterCoeff;
  short                  m_filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];
  unsigned               m_bitsNewFilter[MAX_NUM_CH];
  int                    m_apsIdStart;
  double                *m_ctbDistortionFixedFilter;
  double                *m_ctbDistortionUnfilter[MAX_NUM_COMP];
  std::vector<short>     m_alfCtbFilterSetIndexTmp;
  AlfParam               m_alfParamTempNL;
  int                    m_clipDefaultEnc[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_filterTmp[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_clipTmp[MAX_NUM_ALF_LUMA_COEFF];
  std::vector<uint8_t>   m_numCtusInAsu;

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
  int                    m_maxAsuWidth;
  int                    m_maxAsuHeight;
  int                    m_numAsusInWidth;
  int                    m_numAsusInHeight;
  int                    m_numAsusInPic;
  int                    m_numCtusInAsuWidth;
  int                    m_numCtusInAsuHeight;
  bool                   m_accumStatCTUWise;

public:
  EncAdaptiveLoopFilter();
  virtual ~EncAdaptiveLoopFilter() { destroy(); }
  void init                         ( const VVEncCfg& encCfg, const PPS& pps, CABACWriter& cabacEstimator, CtxCache& ctxCache, NoMallocThreadPool* threadpool );
  void destroy                      ();
  void initDistortion               ();
  std::vector<int> getAvaiApsIdsLuma( CodingStructure& cs, int& newApsId );
  void alfEncoderCtb                ( CodingStructure& cs, AlfParam& alfParamNewFilters, const double lambdaChromaWeight );
  void initCABACEstimator           ( Slice* pcSlice, ParameterSetMap<APS>* apsMap );
  void setApsIdStart                ( int i ) { m_apsIdStart = i; }
  int  getApsIdStart                () { return m_apsIdStart; }
  void getStatisticsCTU             ( Picture& pic, CodingStructure& cs, PelUnitBuf& recYuv, const int ctuRsAddr, PelStorage& alfTempCtuBuf );
  void getStatisticsASU             ( Picture& pic, CodingStructure& cs, PelUnitBuf& recYuv, int xA, int yA, int xC, int yC, PelStorage& alfTempCtuBuf );
  void copyCTUforALF                ( const CodingStructure& cs, int ctuPosX, int ctuPosY );
  void deriveStatsForCcAlfFilteringCTU( CodingStructure& cs, const int compIdx, const int ctuRsAddr, PelStorage& alfTempCtuBuf );
  void deriveCcAlfFilter            ( Picture& pic, CodingStructure& cs );
  void applyCcAlfFilterCTU          ( CodingStructure& cs, ComponentID compID, const int ctuRsAddr, PelStorage& alfTempCtuBuf );
  void deriveFilter                 ( Picture& pic, CodingStructure& cs, const double* lambdas );
  void reconstructCTU_MT            ( Picture& pic, CodingStructure& cs, const int ctuRsAddr, PelStorage& alfTempCtuBuf );
  void reconstructCTU               ( Picture& pic, CodingStructure& cs, const CPelUnitBuf& recBuf, const int ctuRsAddr, PelStorage& alfTempCtuBuf );
//  void alfReconstructor             ( CodingStructure& cs );
  void resetFrameStats              ( bool ccAlfEnabled );
  bool isSkipAlfForFrame            ( const Picture& pic ) const;
private:
  void   xStoreAlfAsuEnabledFlag    ( CodingStructure& cs, int ctuX, int ctuY, int ctuIdx, const int compIdx, bool flag );
  void   xStoreAlfAsuAlternative    ( CodingStructure& cs, int ctuX, int ctuY, int ctuIdx, const int compIdx, const uint8_t alt );
  void   xStoreAlfAsuFilterIdx      ( CodingStructure& cs, int ctuX, int ctuY, int ctuIdx, const short fltIdx, short* alfCtbFilterSetIndex );
  double xCodeAlfAsuEnabledFlag     ( CodingStructure& cs, int ctuIdx, const int compIdx, AlfParam* alfParam, const double ctuLambda );
  double xCodeAlfAsuAlternative     ( CodingStructure& cs, int asuIdx, int ctuIdx, const int compIdx, AlfParam* alfParam, const double ctuLambda );
  double xCodeAlfAsuLumaFilterIdx   ( CodingStructure& cs, int asuIdx, int ctuIdx, AlfParam* alfParam, const double ctuLambda );
  void   xGetStatisticsCTU          ( Picture& pic, CodingStructure& cs, PelUnitBuf& recYuv, const int xPos, const int yPos, const int asuRsAddr, PelStorage& alfTempCtuBuf );
  void   alfEncoder              ( CodingStructure& cs, AlfParam& alfParam, const ChannelType channel, const double lambdaChromaWeight );

  void   copyAlfParam            ( AlfParam& alfParamDst, AlfParam& alfParamSrc, ChannelType channel );
  double mergeFiltersAndCost     ( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits );

  void   getFrameStats           ( ChannelType channel );
  void   getFrameStat            ( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx );
  void   getPreBlkStats          ( AlfCovariance *alfCovariace, const AlfFilterShape &shape, AlfClassifier *classifier, Pel *org, const int orgStride, Pel *rec, const int recStride,
                                   const CompArea &areaDst, const ChannelType channel, int vbCTUHeight, int vbPos );
  template<bool clipToBdry, bool simd>
  void   calcCovariance4         ( Pel* ELocal, const Pel* rec, const int stride, const int halfFilterLength, const int transposeIdx, ChannelType channel, int clipTopRow, int clipBotRow );
  template<bool clipToBdry, bool simd>
  void   calcLinCovariance4      ( Pel* ELocal, const Pel* rec, const int stride, const int halfFilterLength, const int transposeIdx,                      int clipTopRow, int clipBotRow );
  void   getBlkStatsCcAlf        ( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const PelUnitBuf& orgYuv,
                                   const PelUnitBuf &recYuv, const UnitArea &areaDst, const UnitArea &area,
                                   const ComponentID compID, const int yPos);
  void   calcCovarianceCcAlf     ( Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF], const Pel* rec, const int stride, const AlfFilterShape& shape, int vbDistance);
  void   calcCovariance4CcAlf    ( Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][16], const int N, const Pel* rec, const int stride, const AlfFilterShape& shape, int vbDistance);
  void   mergeClasses            ( const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES]);


  double getFilterCoeffAndCost   ( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int& uiCoeffBits, bool onlyFilterCost = false );
  double deriveFilterCoeffs      ( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, alf_float_t errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam);
  int    deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters );
  double deriveCoeffQuant        ( int *filterClipp, int *filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip );
  double deriveCtbAlfEnableFlags ( CodingStructure& cs, ChannelType channel, const double chromaWeight,
                                   const int numClasses, const int numCoeff, double& distUnfilter );
  void   roundFiltCoeff          ( int *filterCoeffQuant, alf_float_t*filterCoeff, const int numCoeff, const int factor );
  void   roundFiltCoeffCCALF     ( int16_t *filterCoeffQuant, alf_float_t*filterCoeff, const int numCoeff, const int factor );

  alf_float_t getDistCoeffForce0  ( bool* codedVarBins, alf_float_t errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, int zeroBitsVarBin, const int numFilters);
  int    lengthUvlc              ( int uiCode );
  int    getNonFilterCoeffRate   ( AlfParam& alfParam );

  int    getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins );
  int    getCostFilterCoeff      ( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    getCostFilterClipp      ( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    lengthFilterCoeffs      ( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff );
  alf_float_t getDistForce0       ( AlfFilterShape& alfShape, const int numFilters, alf_float_t errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins );
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
  void deriveCcAlfFilterCoeff    ( ComponentID compID, short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx );
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
  void deriveCcAlfFilter         ( CodingStructure& cs, ComponentID compID, const PelUnitBuf& orgYuv, const PelUnitBuf& dstYuv );
  void xSetupCcAlfAPS            ( CodingStructure& cs );
  std::vector<int> getAvailableCcAlfApsIds(CodingStructure& cs, ComponentID compID);
  void countLumaSwingGreaterThanThreshold(const Pel* luma, int lumaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* lumaSwingGreaterThanThresholdCount, int lumaCountStride);
  void countChromaSampleValueNearMidPoint(const Pel* chroma, int chromaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* chromaSampleCountNearMidPoint, int chromaSampleCountNearMidPointStride);
  void getFrameStatsCcalf        ( ComponentID compIdx, int filterIdc);
  void initDistortionCcalf       ();
  inline int getAsuMaxCtuX( int ctuX )
  {
    return std::min( ctuX + m_numCtusInAsuWidth, (int)m_numCTUsInWidth );
  }

  inline int getAsuMaxCtuY( int ctuY )
  {
    return std::min( ctuY + m_numCtusInAsuHeight, (int)m_numCTUsInHeight );
  }

  inline void getAsuCtuXY( int asuIdx, int& ctuX, int& ctuY )
  {
    int asuY = ( asuIdx / m_numAsusInWidth        );
    int asuX = ( asuIdx - asuY * m_numAsusInWidth );
    ctuX = asuX * m_numCtusInAsuWidth;
    ctuY = asuY * m_numCtusInAsuHeight;
  }
};

} // namespace vvenc

//! \}

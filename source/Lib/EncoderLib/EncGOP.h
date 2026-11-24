/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     EncGOP.h
    \brief    GOP encoder class (header)
*/

#pragma once

#include "EncSampleAdaptiveOffset.h"
#include "VLCWriter.h"
#include "SEIwrite.h"
#include "SEIEncoder.h"
#include "Analyze.h"
#include "EncPicture.h"
#include "CommonLib/Picture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Nal.h"
#include "EncHRD.h"

#include <map>
#include <set>
#include <tuple>
#include "EncStage.h"
#include "SEIFilmGrainAnalyzer.h"

#include <vector>
#include <list>
#include <stdlib.h>
#include <atomic>


#include "Utilities/NoMallocThreadPool.h"
#include <mutex>
#include <condition_variable>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================

class InputByteStream;
class DecLib;
class EncHRD;
class MsgLog;
class GOPCfg;

// ====================================================================================================================

class EncGOP;
struct FinishTaskParam {
  EncGOP*     gopEncoder;
  EncPicture* picEncoder;
  Picture*    pic;
  unsigned    setupNum;   ///< the picture's place in the order of the setup tasks (when its setup runs as a task)
  FinishTaskParam()                                          : gopEncoder( nullptr ), picEncoder( nullptr ), pic( nullptr ), setupNum( 0 ) {}
  FinishTaskParam( EncGOP* _g, EncPicture* _e, Picture* _p ) : gopEncoder( _g ),      picEncoder( _e ),      pic( _p ),      setupNum( 0 ) {}
};

// ====================================================================================================================
struct RateCapParam {
  unsigned accumActualBits = 0;
  unsigned accumTargetBits = 0;
  unsigned accumGopCounter = 0;
  double   nonRateCapEstim = 0.0;
  int      gopAdaptedQPAdj = 0;

  void reset()
  {
    accumActualBits = 0;
    accumTargetBits = 0;
    accumGopCounter = 0;
  }
};

// ====================================================================================================================
// film grain parameters estimated by the film grain analysis, as signalled in the FGC SEI
struct FilmGrainParam {
  int               log2ScaleFactor = 0;
  SeiFgc::CompModel compModel[ MAX_NUM_COMP ];

  FilmGrainParam() = default;
  // The parameters of the analysis, with the grain strength set to strength percent of the analysed grain.
  FilmGrainParam( const FGAnalyzer& fgAnalyzer, double strength )
  {
    fgAnalyzer.getSignalledParameters( strength, log2ScaleFactor, compModel );
    for( auto& cm : compModel )
    {
      // only the signalled intervals and model values, as the SEI writer expects
      cm.intensityValues.resize( cm.numIntensityIntervals );
      for( auto& iv : cm.intensityValues )
      {
        iv.compModelValue.resize( cm.numModelValues );
      }
    }
  }
};

class EncGOP : public EncStage
{
private:
  MsgLog&                   msg;

  Analyze                   m_AnalyzeAll;
  Analyze                   m_AnalyzeI;
  Analyze                   m_AnalyzeP;
  Analyze                   m_AnalyzeB;

  std::function<void( void*, vvencYUVBuffer* )> m_recYuvBufFunc;
  void*                                         m_recYuvBufCtx;

  NoMallocThreadPool*       m_threadPool;
  std::mutex                m_gopEncMutex;
  std::condition_variable   m_gopEncCond;
  // pictures finished so far and the count waitForFreeEncoders last returned on (both guarded by m_gopEncMutex)
  uint64_t                  m_numPicsFinished       = 0;
  uint64_t                  m_numPicsFinishedWaited = 0;
  XUCache                   m_shrdUnitCache;
  std::mutex                m_unitCacheMutex;

  const VVEncCfg*           m_pcEncCfg;
  const GOPCfg*             m_gopCfg;
  RateCtrl*                 m_pcRateCtrl;
  HLSWriter                 m_HLSWriter;
  SEIWriter                 m_seiWriter;
  SEIEncoder                m_seiEncoder;
  BlkStat                   m_BlkStat;

  ParameterSetMap<SPS>      m_spsMap;
  ParameterSetMap<PPS>      m_ppsMap;
  EncHRD                    m_EncHRD;
  VPS                       m_VPS;
  DCI                       m_DCI;

  bool                      m_isPreAnalysis;
  bool                      m_bFirstWrite;
  bool                      m_bRefreshPending;
  int                       m_lastCodingNum;
  int                       m_numPicsCoded;
  int                       m_numPicsInMissing;
  int                       m_numPicsOutOffset;
  uint64_t                  m_lastCts;
  int                       m_pocRecOut;
  int                       m_ticksPerFrameMul4;
  int                       m_lastIDR;
  int                       m_lastRasPoc;
  int                       m_pocCRA;
  int                       m_associatedIRAPPOC;
  vvencNalUnitType          m_associatedIRAPType;
  RateCapParam              m_rcap;

  ThreadRsrcPool            m_threadRsrc;          ///< CU encoders of the worker threads, shared by all picture encoders
  bool                      m_picSetupTasks;       ///< the pictures' setups run as thread pool tasks, see xEncodePicture
  unsigned                  m_numSetupsQueued;     ///< setup tasks queued so far (main thread only)
  std::atomic<unsigned>     m_numSetupsDone;       ///< setup tasks finished so far, which lets the next one in order run
  int64_t                   m_numPicsStarted;      ///< pictures that got a picture encoder so far (main thread only)
  std::list<EncPicture*>    m_freePicEncoderList;
  std::list<Picture*>       m_gopEncListInput;
  std::list<Picture*>       m_gopEncListOutput;
  std::list<Picture*>       m_procList;
  std::list<Picture*>       m_rcUpdateList;
  std::list<Picture*>       m_rcInputReorderList;  // used in RC in IFP lines synchro mode
  std::deque<PicApsGlobal*> m_globalApsList;
  std::vector<int>          m_globalCtuQpVector;
  bool                      m_forceSCC;

  FGAnalyzer              m_fgAnalyzer;                 ///< film grain analysis (--fga)

  // An analysis, run as a thread pool task. The tasks run one after the other in POC order (each waits for the
  // previous one), because every analysis adds to the statistics of the previous ones.
  struct FgaTask
  {
    EncGOP*        gop       = nullptr;
    Picture*       pic       = nullptr;
    int            codingNum = 0;   ///< the picture with whose access unit the result is taken
    int            startPoc  = std::numeric_limits<int>::min();   ///< the first picture in output order its result is for; written pictures keep theirs
    bool           sceneCut  = false;   ///< a scene cut lies between the previously analysed picture and this one
    FilmGrainParam result;
    WaitCounter    counter;
  };
  std::deque<std::unique_ptr<FgaTask>> m_fgaTasks;      ///< analyses whose access unit is not written yet
  std::unique_ptr<FgaTask>             m_fgaConsumed;   ///< the last consumed task, kept until the next one has run
  FgaTask*                             m_fgaLast = nullptr;   ///< the last submitted task
  int                                  m_fgaPrevAnalysedPoc = std::numeric_limits<int>::min();   ///< POC of the last analysed picture
  // The film grain parameter sets by the POC from which they apply in output order: an analysis applies from the
  // picture after the previously analysed one, or from a scene cut between them, but never before the picture after
  // the last written one. The latest few are kept.
  size_t                    m_fgcParamSets  = 8;   ///< the most kept: two GOPs of analyses and two more, at least 8
  std::map<int, FilmGrainParam> m_fgcByStartPoc;
  bool                      m_fgcCompSignalled[ MAX_NUM_COMP ] = { false, false, false };   ///< components present in a parameter set so far
  // The POCs of the latest access units written in the current coded layer video sequence (more than two GOPs), and
  // whether a RASL picture carried a film grain SEI since the last other picture; to decide where the SEI is needed.
  static constexpr size_t   FGC_SEI_HISTORY = 160;
  std::set<int>             m_fgcWrittenPocs;
  bool                      m_fgcRaslSeiPending = false;
  // The film grain SEI fitted to the grain the coding lost (xScaleToLostGrain): the lost-grain spectra of the latest
  // coded pictures (LostGrainPicture); per component, intensity interval and temporal layer, the last cut-offs and
  // scaling factor written; per component, the cut-offs (LostGrainCutoffs).
  // Started over at every random access point, except the costs the cut-offs are chosen by (LostGrainStates; --fga 2
  // and 3).
  struct LostGrainPicture
  {
    int      poc   = 0;
    int      scene = 0;   ///< POC of the scene cut its scene starts with (PreProcess::xDetectSceneCut)
    unsigned layer = 0;   ///< temporal layer
    std::vector<LostGrainSpectrum> spectra[ MAX_NUM_COMP ];   ///< per 8-bit tile mean (FGAnalyzer::lostGrainSpectra)
  };
  // An intensity interval is fitted to the lost grain of the LOST_GRAIN_POOL coded pictures of the picture's scene that
  // are nearest to it in output order, itself among them, whatever their temporal layers (xLostGrainPool). What a single
  // picture lost is not what its neighbours lost: Top Gun (4K HDR, a real encode at QP 31) comes from an HEVC encode
  // with two B pictures between P pictures, and every third picture has more fine grain; at periods of 6.25-3.125
  // samples the coding took 2.6-2.9 (squared 10-bit sample values) from those and 2.2-2.3 from the others, in every
  // temporal layer alike (the layers' means: 2.32-2.41), while periods of 12.5-6.25 samples differ by 4 % from picture
  // to picture. With a pool per temporal layer (the layer's last 4 pictures at most 6 back: one picture in layers 0 to
  // 3, two in layer 4) the pools held between none and only such pictures, and where that band's allowance ends the fit
  // (cut-offs 5 / 5 under a tolerance of 1.05) the scaling factor went 56-64 from one picture to the next, and all the
  // grain with it: the decoded grain's standard deviation changed by 3.3 % between neighbouring pictures in the median
  // and by more than 8.6 % between one pair in 20. The intervals of a new analysis are summed from the kept spectra,
  // so a pool does not start over with them.
  // The frequency model's scaling factor (--fga 2) is the lowest of the fits to single pictures of the pool
  // (lowestPictureFit; which ones: LOST_GRAIN_POOL_SPAN): the picture is one of them, so it never gets more than the
  // tolerance above what it lost itself, and the pictures around it, whose pools hold mostly the same pictures, get
  // the same. Fitted to the pool's sum alone, the two of three Top Gun pictures that lost less had 1.10-1.13 of their
  // lost amplitude at periods of 6.25-3.125 samples, and a picture of the lowest temporal layers, whose nearest coded
  // pictures are those of the group of pictures before it, 30 and more pictures away, got what those had lost: on
  // motion96 (Top Gun, high motion; replayed on a real encode's spectra) a group's first picture had lost 2.4 at
  // periods of 50-25 samples and its pool 5.7, and it got a scaling factor of 119 between neighbours at 75-84.
  // A single picture's fit is to the most its tiles say it lost, their mean plus two standard errors
  // (FGAnalyzer::lostGrainPowerAtMost): it lowers the fit only by what the error of its own measurement does not
  // explain. Fitted to the tiles' means, the lowest of 8 fits was also the lowest of 8 errors. In real encodes at
  // QP 31 (pictures 120 to 239 of four 10 s cuts: gangster_chase, 1080p; game_of_death, 3840x1632; motion96 and
  // hangar, Top Gun) the single fits of a pool differ by 9.4 / 8.6 / 5.1 / 3.6 % (standard deviation, in amplitude)
  // and one standard error is 4.9 / 2.9 / 2.1 / 0.9 % of a fit, so most of the difference is what the pictures lost.
  // The lowest fit was 0.83 / 0.87 / 0.90 / 0.96 of the fit to the pool's sum and is 0.91 / 0.92 / 0.93 / 0.98 of
  // it; the decoded grain's standard deviation changed by more than 21.6 / 31.4 / 11.4 / 1.9 % between one pair of
  // neighbouring pictures in 20 and does by 17.5 / 30.0 / 10.1 / 2.1 %.
  static constexpr size_t LOST_GRAIN_POOL = 8;
  // The pool's near pictures are those at most this many pictures away (the picture itself is one).
  // --fga 2 fits only the near pictures alone (xLostGrainFittedAlone): those are the pictures whose pools hold this
  // picture too. A far one counts in the pool's sum only: fitted alone, it lowered pictures far from it and not the
  // ones between. On gangster_chase a layer-1 picture 16 pictures after a scene cut had lost less than the pictures
  // before it (fitted alone: 114, the others 154-204) and took the scene's pictures 1 to 4, 6 and 8 to a scaling
  // factor of 57 between pictures at 65-74.
  // A picture without a near one (the first pictures coded of a group of pictures, of the lowest temporal layers) is
  // coded before all the pictures around it, which are of higher layers than its own; what such pictures lose, only
  // the far pictures of higher layers than its own tell, so those are fitted alone for it. With none fitted alone
  // such a picture stood out above its neighbours where every third picture loses less (hangar, replayed: the
  // scaling factor times the pattern's standard deviation changed by more than 3.5 % between one pair of
  // neighbouring pictures in 20, against 2.1 %). A far picture of its own layer or of a lower one is coded with
  // another quantizer and says nothing of them: on game_of_death the layer-0 picture of the group before (QP 20,
  // fitted alone: 74, the pictures before it 88-118) took the next group's layer-0 and layer-1 pictures, which
  // fitted alone had 112 and 96, to 74, and their decoded grain had a standard deviation of 2.18 and 2.09 between
  // neighbours at 3.44-3.50 and 2.74-2.75 (now 2.93 and 2.78 between 3.63-3.68 and 2.89). With both rules the
  // decoded grain's standard deviation changes by more than 15.8 / 17.4 / 10.3 / 2.1 % between one pair of
  // neighbouring pictures in 20 on the four cuts above.
  // The auto-regressive model's scaling factor (--fga 3) is the fit to the pool's sum, never above the fit to the sum
  // of the pool's near pictures (that keeps the far pool of a picture of the lowest layers from giving it more than
  // the near pictures lost). Not the lowest fit of single pictures: this model's scaling factors are small (21-26 on
  // Top Gun at QP 31, --fga 2's are 57-63 there), so one step is 4 % of the grain, and the lowest fit moved by a step
  // or two whenever a picture with a low fit entered or left the pool (the random access point that starts a scene
  // had 22 where the pictures after it had 24, and its six nearest pictures and the next three of the lowest layers
  // got 22 between neighbours at 23-24): the decoded grain's standard deviation changed by more than 9.1 % between
  // one pair of neighbouring pictures in 20, against 3.8 % with the fit to the sums, whose grain was within 1.04 of
  // the lost grain's amplitude at periods of 25 to 3.125 samples in the two pictures measured.
  static constexpr int LOST_GRAIN_POOL_SPAN = 6;
  struct LostGrainFitted
  {
    int h, w, scale;
  };
  using LostGrainKey = std::tuple<int, int, int, unsigned>;   ///< component, interval lower and upper bound, temporal layer
  // With the auto-regressive model (--fga 3): per component, the grain of the filter last signalled (FGAnalyzer::arGrain
  // is computed once per filter, the fit runs for every picture).
  struct ArGrainOfFilter
  {
    std::vector<int>    filter;   ///< log2 scale factor, then the model values after the scaling factor
    FGAnalyzer::ArGrain grain;
  };
  // What a component's cut-offs are chosen by (xFitLostGrain): per pair of cut-offs the two costs of its fits
  // (FGAnalyzer::LostGrainFit), and the two costs of no grain, each summed over the intervals' tiles.
  struct LostGrainCosts
  {
    std::vector<double> seen, allBands;   ///< per pair of cut-offs, in xFitLostGrain's order
    double              seenOfNone = 0.0, allBandsOfNone = 0.0;
    explicit LostGrainCosts( size_t pairs = 0 ) : seen( pairs, 0.0 ), allBands( pairs, 0.0 ) {}
    // counts the sums so far memory times, then adds a picture's costs
    void fadeAndAdd( const LostGrainCosts& picture, double memory )
    {
      seen.resize( picture.seen.size(), 0.0 );
      allBands.resize( picture.allBands.size(), 0.0 );
      for( size_t k = 0; k < seen.size(); k++ )
      {
        seen    [ k ] = memory * seen    [ k ] + picture.seen    [ k ];
        allBands[ k ] = memory * allBands[ k ] + picture.allBands[ k ];
      }
      seenOfNone     = memory * seenOfNone     + picture.seenOfNone;
      allBandsOfNone = memory * allBandsOfNone + picture.allBandsOfNone;
    }
  };
  // The cut-offs of a component's grain.
  struct LostGrainCutoffs
  {
    int h = 0, w = 0;
  };
  struct LostGrainState
  {
    std::deque<LostGrainPicture>            pictures;   ///< the latest coded ones, in coding order
    std::map<LostGrainKey, LostGrainFitted> fitted;
    std::map<int, LostGrainCutoffs>         cutoffs;    ///< per component
    ArGrainOfFilter                         arGrain[ MAX_NUM_COMP ];
  };
  // A random access point starts the fit over, but the pictures before it in output order that are coded after it
  // (its leading pictures) go on with the state from before it (xLostGrainStateOf). The costs the cut-offs are chosen
  // by are not started over: every picture adds its own in coding order, the older ones fading (xFitLostGrain).
  struct LostGrainStates
  {
    LostGrainState current;
    LostGrainState leading;                                  ///< of the pictures before the last random access point
    int            irapPoc = std::numeric_limits<int>::min();   ///< POC of the last random access point coded
    std::map<int, LostGrainCosts> cutoffCosts;               ///< per component
  };
  LostGrainStates           m_lostGrain;   ///< the fit of the film grain SEIs to the grain the coding lost (xScaleToLostGrain)

public:
  EncGOP( MsgLog& msglog );
  virtual ~EncGOP();

  void setRecYUVBufferCallback( void* ctx, std::function<void( void*, vvencYUVBuffer* )> func );

  void init               ( const VVEncCfg& encCfg, const GOPCfg* gopCfg, RateCtrl& rateCtrl, NoMallocThreadPool* threadPool, bool isPreAnalysis );
  void printOutSummary    ( const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr );
  void getParameterSets   ( AccessUnitList& accessUnit );

protected:
  virtual void initPicture    ( Picture* pic );
  virtual void processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList );
  virtual bool hasEnoughPictures( const PicList& picList ) const;
  virtual bool waitForFreeEncoders();
  virtual void startReadyPictures();

private:
  void xUpdateRasInit                 ( Slice* slice );
  void xProcessPictures               ( AccessUnitList& auList, PicList& doneList );
  bool xIsLockStep                    () const;
  bool xFetchOnEveryPass              ( bool lockStepMode ) const;
  void xStartPictures                 ( bool lockStepMode, bool fetchOnEveryPass, int maxStarts );   // maxStarts 0: no limit
  void xEncodePicture                 ( Picture* pic, EncPicture* picEncoder );
  void xOutputRecYuv                  ( const PicList& picList );
  void xReleasePictures               ( const PicList& picList, PicList& freeList );
  bool xFgaReadsSource                ( const Picture& pic ) const;

  void xInitVPS                       ( VPS &vps ) const;
  void xInitDCI                       ( DCI &dci, const SPS &sps, const int dciId ) const;
  void xInitConstraintInfo            ( ConstraintInfo &ci ) const;
  void xInitSPS                       ( SPS &sps ) const;
  void xInitPPS                       ( PPS &pps, const SPS &sps ) const;
  void xInitPPSforTiles               ( PPS &pps, const SPS &sps ) const;
  void xInitRPL                       ( SPS &sps ) const;
  void xInitHrdParameters             ( SPS &sps );

  vvencNalUnitType xGetNalUnitType    ( const GOPEntry* _gopEntry ) const;
  bool xIsSliceTemporalSwitchingPoint ( const Slice* slice, const PicList& picList ) const;

  void xSetupPicAps                   ( Picture* pic );
  void xAnalyzeFilmGrain              ( const Picture& current, const PicList& picList );
  void xIssueFgaTask                  ( Picture& pic, int codingNum );
  void xRunFgaTask                  ( FgaTask& task );
  void xWaitFgaTasks                ();
  void xInitPicsInCodingOrder         ( const PicList& picList );
  void xGetProcessingLists            ( std::list<Picture*>& procList, std::list<Picture*>& rcUpdateList, const bool lockStepMode );
  void xUpdateVAStartOfLastGop        ( Picture& keyPic ) const;
  void xInitGopQpCascade              ( Picture& keyPic, PicList::const_iterator picItr, const PicList& picList );
  void xInitFirstSlice                ( Picture& pic, const PicList& picList, bool isEncodeLtRef );
  void xInitSliceTMVPFlag             ( PicHeader* picHeader, const Slice* slice );
  void xUpdateRPRtmvp                 ( PicHeader* picHeader, Slice* slice );
  void xInitSliceMvdL1Zero            ( PicHeader* picHeader, const Slice* slice );
  void xSelectReferencePictureList    ( Slice* slice ) const;
  void xSyncAlfAps                    ( Picture& pic );

  void xUpdateRcIfp                   ();
  void xWritePicture                  ( Picture& pic, AccessUnitList& au, bool isEncodeLtRef );
  int  xWriteParameterSets            ( Picture& pic, AccessUnitList& accessUnit, HLSWriter& hlsWriter );
  int  xWritePictureSlices            ( Picture& pic, AccessUnitList& accessUnit, HLSWriter& hlsWriter );
  void xAddFilmGrainParameters       ( const FgaTask& task );
  const FilmGrainParam& xFilmGrainParametersAt( int poc ) const;
  SeiFgc xFilmGrainSei                ( const FilmGrainParam& param, ChromaFormat chromaFormat );
  bool xFilmGrainSeiNeeded           ( const Slice& slice, const SeiFgc& sei, ChromaFormat chromaFormat );
  bool xFitsLostGrain                ( ) const;
  void xStoreLostGrainSpectra        ( Picture& pic ) const;
  LostGrainState& xLostGrainStateOf  ( const Picture& pic, const Slice& slice );
  static std::vector<const LostGrainPicture*> xLostGrainPool( const std::deque<LostGrainPicture>& pictures );
  static std::vector<char> xLostGrainFittedAlone   ( const std::vector<const LostGrainPicture*>& pool );
  void xScaleToLostGrain             ( Picture& pic, const Slice& slice, SeiFgc& sei );
  void xFitLostGrainAr               ( LostGrainState& state, int c, unsigned layer, int bitDepth, int log2ScaleFactor,
                                       const std::vector<FGAnalyzer::LostGrainPower>& power,
                                       const std::vector<FGAnalyzer::LostGrainPower>& nearPower,
                                       const std::vector<FGAnalyzer::LostGrainBands>& bands, SeiFgc::CompModel& cm ) const;
  static int xBestLostGrainCutoffs   ( const LostGrainCosts& sums, int current );
  void xFitLostGrain                 ( LostGrainState& state, LostGrainCosts& costs, int c, unsigned layer, int bitDepth, bool chroma420,
                                       int log2ScaleFactor, const std::vector<FGAnalyzer::LostGrainPower>& power,
                                       const std::vector<std::vector<FGAnalyzer::LostGrainPower>>& picturePower,
                                       const std::vector<FGAnalyzer::LostGrainBands>& bands, SeiFgc::CompModel& cm ) const;
  void xWriteLeadingSEIs              ( Picture& pic, AccessUnitList& accessUnit );
  void xWriteTrailingSEIs             ( const Picture& pic, AccessUnitList& accessUnit, std::string& digestStr );
  int  xWriteVPS                      ( AccessUnitList &accessUnit, const VPS *vps, HLSWriter& hlsWriter );
  int  xWriteDCI                      ( AccessUnitList &accessUnit, const DCI *dci, HLSWriter& hlsWriter );
  int  xWriteSPS                      ( AccessUnitList &accessUnit, const SPS *sps, HLSWriter& hlsWriter );
  int  xWritePPS                      ( AccessUnitList &accessUnit, const PPS *pps, const SPS *sps, HLSWriter& hlsWriter );
  int  xWriteAPS                      ( AccessUnitList &accessUnit, const APS *aps, HLSWriter& hlsWriter, vvencNalUnitType eNalUnitType );
  void xWriteAccessUnitDelimiter      ( AccessUnitList &accessUnit, Slice* slice, bool IrapOrGdr, HLSWriter& hlsWriter );
  void xWriteSEI                      ( vvencNalUnitType naluType, SEIMessages& seiMessages, AccessUnitList &accessUnit, AccessUnitList::iterator &auPos, int temporalId, const SPS *sps );
  void xWriteSEISeparately            ( vvencNalUnitType naluType, SEIMessages& seiMessages, AccessUnitList &accessUnit, AccessUnitList::iterator &auPos, int temporalId, const SPS *sps );
  void xAttachSliceDataToNalUnit      ( OutputNALUnit& rNalu, const OutputBitstream* pcBitstreamRedirect );
  void xCabacZeroWordPadding          ( const Picture& pic, const Slice* slice, uint32_t binCountsInNalUnits, uint32_t numBytesInVclNalUnits, std::ostringstream &nalUnitData );

  void xUpdateRateCap();
  void xUpdateRateCapBits             ( const Picture* pic, const uint32_t uibits );
  void xAddPSNRStats( const Picture* pic, CPelUnitBuf cPicD, AccessUnitList&, bool printFrameMSE, double* PSNR_Y, bool isEncodeLtRef );
  uint64_t xFindDistortionPlane       ( const CPelBuf& pic0, const CPelBuf& pic1, uint32_t rshift ) const;
  void xPrintPictureInfo              ( const Picture& pic, AccessUnitList& accessUnit, const std::string& digestStr, bool printFrameMSE, bool isEncodeLtRef );
  inline bool xEncodersFinished       () { return ( int ) m_freePicEncoderList.size() >= std::max(1, m_pcEncCfg->m_maxParallelFrames); }
  inline bool xLockStepPicsFinished   ()
  {
    std::lock_guard<std::mutex> lock( m_gopEncMutex );
    return ( int ) m_freePicEncoderList.size() >= std::max(1, m_pcEncCfg->m_maxParallelFrames); 
  }
  void xForceScc                      ( Picture& pic );

};// END CLASS DEFINITION EncGOP

} // namespace vvenc

//! \}


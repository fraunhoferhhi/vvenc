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
/** \file     EncCfgExpert.h
    \brief    encoder configuration class (header)
*/

#pragma once

#include <cstring>
#include <vector>
#include <string>
#include <sstream>
#include "vvenc/vvencDecl.h"
#include "vvenc/Basics.h"

//! \ingroup Interface
//! \{

namespace vvenc {

// ====================================================================================================================

struct VVENC_DECL GOPEntry
{
  int    m_POC;
  int    m_QPOffset;
  double m_QPOffsetModelOffset;
  double m_QPOffsetModelScale;
  int    m_CbQPoffset;
  int    m_CrQPoffset;
  double m_QPFactor;
  int    m_tcOffsetDiv2;
  int    m_betaOffsetDiv2;
  int    m_CbTcOffsetDiv2;
  int    m_CbBetaOffsetDiv2;
  int    m_CrTcOffsetDiv2;
  int    m_CrBetaOffsetDiv2;
  int    m_temporalId;
  bool   m_refPic;
  int8_t m_sliceType;
  int    m_numRefPicsActive[ 2 ];
  int    m_numRefPics[ 2 ];
  int    m_deltaRefPics[ 2 ][ MAX_NUM_REF_PICS ];
  bool   m_isEncoded;
  bool   m_ltrp_in_slice_header_flag;
GOPEntry()
    : m_POC                   ( -1 )
      , m_QPOffset            ( 0 )
      , m_QPOffsetModelOffset ( 0 )
      , m_QPOffsetModelScale  ( 0 )
      , m_CbQPoffset          ( 0 )
      , m_CrQPoffset          ( 0 )
      , m_QPFactor            ( 0 )
      , m_tcOffsetDiv2        ( 0 )
      , m_betaOffsetDiv2      ( 0 )
      , m_CbTcOffsetDiv2      ( 0 )
      , m_CbBetaOffsetDiv2    ( 0 )
      , m_CrTcOffsetDiv2      ( 0 )
      , m_CrBetaOffsetDiv2    ( 0 )
      , m_temporalId          ( 0 )
      , m_refPic              ( false )
      , m_sliceType           ( 'P' )
      , m_numRefPicsActive    { 0 }
      , m_numRefPics          { 0 }
      , m_isEncoded(false)
      , m_ltrp_in_slice_header_flag(false)
      {
        std::memset( m_deltaRefPics, 0, sizeof( m_deltaRefPics ) );
      }
};

inline std::ostream& operator<< ( std::ostream& os, const GOPEntry& entry )
{
  os << entry.m_sliceType;
  os << entry.m_POC;
  os << entry.m_QPOffset;
  os << entry.m_QPOffsetModelOffset;
  os << entry.m_QPOffsetModelScale;
  os << entry.m_CbQPoffset;
  os << entry.m_CrQPoffset;
  os << entry.m_QPFactor;
  os << entry.m_tcOffsetDiv2;
  os << entry.m_betaOffsetDiv2;
  os << entry.m_CbTcOffsetDiv2;
  os << entry.m_CbBetaOffsetDiv2;
  os << entry.m_CrTcOffsetDiv2;
  os << entry.m_CrBetaOffsetDiv2;
  os << entry.m_temporalId;

  for( int l = 0; l < 2; l++)
  {
    os <<  entry.m_numRefPicsActive[l];
    os <<  entry.m_numRefPics[l];
    for ( int i = 0; i < entry.m_numRefPics[l]; i++ )
    {
      os <<  entry.m_deltaRefPics[l][i];
    }
  }

  return os;
}

inline std::istream& operator>> ( std::istream& in, GOPEntry& entry )
{
  in >> entry.m_sliceType;
  in >> entry.m_POC;
  in >> entry.m_QPOffset;
  in >> entry.m_QPOffsetModelOffset;
  in >> entry.m_QPOffsetModelScale;
  in >> entry.m_CbQPoffset;
  in >> entry.m_CrQPoffset;
  in >> entry.m_QPFactor;
  in >> entry.m_tcOffsetDiv2;
  in >> entry.m_betaOffsetDiv2;
  in >> entry.m_CbTcOffsetDiv2;
  in >> entry.m_CbBetaOffsetDiv2;
  in >> entry.m_CrTcOffsetDiv2;
  in >> entry.m_CrBetaOffsetDiv2;
  in >> entry.m_temporalId;

  for( int l = 0; l < 2; l++)
  {
    in >> entry.m_numRefPicsActive[l];
    in >> entry.m_numRefPics[l];
    for ( int i = 0; i < entry.m_numRefPics[l]; i++ )
    {
      in >> entry.m_deltaRefPics[l][i];
    }
  }

  return in;
}

// ====================================================================================================================

struct VVENC_DECL RPLEntry
{
  int    m_POC;
  int    m_temporalId;
  bool   m_refPic;
  bool   m_ltrp_in_slice_header_flag;
  int    m_numRefPicsActive;
  int8_t m_sliceType;
  int    m_numRefPics;
  int    m_deltaRefPics[ MAX_NUM_REF_PICS ];

  RPLEntry()
    : m_POC                ( -1 )
      , m_temporalId       ( 0 )
      , m_refPic           ( false )
      , m_ltrp_in_slice_header_flag(false)
      , m_numRefPicsActive ( 0 )
      , m_sliceType        ( 'P' )
      , m_numRefPics       ( 0 )
  {
    ::memset( m_deltaRefPics, 0, sizeof( m_deltaRefPics ) );
  }
};

struct VVENC_DECL WCGChromaQPControl
{
  WCGChromaQPControl()
  : enabled         (false)
  , chromaCbQpScale (1.0)  , chromaCrQpScale (1.0)
  , chromaQpScale   (0.0)  , chromaQpOffset  (0.0)
  {}

  bool   enabled;         ///< Enabled flag (0:default)
  double chromaCbQpScale; ///< Chroma Cb QP Scale (1.0:default)
  double chromaCrQpScale; ///< Chroma Cr QP Scale (1.0:default)
  double chromaQpScale;   ///< Chroma QP Scale (0.0:default)
  double chromaQpOffset;  ///< Chroma QP Offset (0.0:default)
};

// ====================================================================================================================

/// encoder configuration class
class VVENC_DECL EncCfgExpert
{
public:

  bool                m_listTracingChannels;
  std::string         m_traceRule;
  std::string         m_traceFile;

  int                 m_conformanceWindowMode;
  int                 m_confWinLeft;
  int                 m_confWinRight;
  int                 m_confWinTop;
  int                 m_confWinBottom;

  unsigned            m_temporalSubsampleRatio;                         ///< temporal subsample ratio, 2 means code every two frames

  int                 m_aiPad[ 2 ];                                     ///< number of padded pixels for width and height
  bool                m_enablePictureHeaderInSliceHeader;
  bool                m_AccessUnitDelimiter;                            ///< add Access Unit Delimiter NAL units

  bool                m_printMSEBasedSequencePSNR;
  bool                m_printHexPsnr;
  bool                m_printFrameMSE;
  bool                m_printSequenceMSE;
  bool                m_cabacZeroWordPaddingEnabled;

  unsigned            m_subProfile;

  unsigned            m_bitDepthConstraintValue;
  bool                m_intraOnlyConstraintFlag;

  int                 m_InputQueueSize;                                 ///< Size of frame input queue
  bool                m_rewriteParamSets;                               ///< Flag to enable rewriting of parameter sets at random access points
  bool                m_idrRefParamList;                                ///< indicates if reference picture list syntax elements are present in slice headers of IDR pictures
  RPLEntry            m_RPLList0[ MAX_GOP ];                            ///< the RPL entries from the config file
  RPLEntry            m_RPLList1[ MAX_GOP ];                            ///< the RPL entries from the config file
  GOPEntry            m_GOPList [ MAX_GOP ];                            ///< the coding structure entries from the config file
  int                 m_maxDecPicBuffering[ MAX_TLAYER ];               ///< total number of pictures in the decoded picture buffer
  int                 m_maxNumReorderPics [ MAX_TLAYER ];               ///< total number of reorder pictures
  int                 m_maxTempLayer;                                   ///< Max temporal layer
  int                 m_numRPLList0;
  int                 m_numRPLList1;

  bool                m_useSameChromaQPTables;
  ChromaQpMappingTableParams m_chromaQpMappingTableParams;
  int                 m_intraQPOffset;                                  ///< QP offset for intra slice (integer)
  bool                m_lambdaFromQPEnable;                             ///< enable flag for QP:lambda fix
  double              m_adLambdaModifier[ MAX_TLAYER ];                 ///< Lambda modifier array for each temporal layer
  std::vector<double> m_adIntraLambdaModifier;                          ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  double              m_dIntraQpFactor;                                 ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))
  std::vector<int>    m_qpInValsCb;                                     ///< qp input values used to derive the chroma QP mapping table
  std::vector<int>    m_qpInValsCr;                                     ///< qp input values used to derive the chroma QP mapping table
  std::vector<int>    m_qpInValsCbCr;                                   ///< qp input values used to derive the chroma QP mapping table
  std::vector<int>    m_qpOutValsCb;                                    ///< qp output values used to derive the chroma QP mapping table
  std::vector<int>    m_qpOutValsCr;                                    ///< qp output values used to derive the chroma QP mapping table
  std::vector<int>    m_qpOutValsCbCr;                                  ///< qp output values used to derive the chroma QP mapping table
  int                 m_cuQpDeltaSubdiv;                                ///< Maximum subdiv for CU luma Qp adjustment (0:default)
  int                 m_cuChromaQpOffsetSubdiv;                         ///< If negative, then do not apply chroma qp offsets.
  int                 m_chromaCbQpOffset;                               ///< Chroma Cb QP Offset (0:default)
  int                 m_chromaCrQpOffset;                               ///< Chroma Cr QP Offset (0:default)
  int                 m_chromaCbQpOffsetDualTree;                       ///< Chroma Cb QP Offset for dual tree (overwrite m_chromaCbQpOffset for dual tree)
  int                 m_chromaCrQpOffsetDualTree;                       ///< Chroma Cr QP Offset for dual tree (overwrite m_chromaCrQpOffset for dual tree)
  int                 m_chromaCbCrQpOffset;                             ///< QP Offset for joint Cb-Cr mode
  int                 m_chromaCbCrQpOffsetDualTree;                     ///< QP Offset for joint Cb-Cr mode (overwrite m_chromaCbCrQpOffset for dual tree)
  unsigned            m_sliceChromaQpOffsetPeriodicity;                 ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  int                 m_sliceChromaQpOffsetIntraOrPeriodic[ 2 ];        ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.
  bool                m_lumaLevelToDeltaQPEnabled;
  WCGChromaQPControl  m_wcgChromaQpControl;
  bool                m_sdr;
  bool                m_calculateHdrMetrics;
  int                 m_cropOffsetLeft;
  int                 m_cropOffsetTop;
  int                 m_cropOffsetRight;
  int                 m_cropOffsetBottom;


  ChromaFormat        m_internChromaFormat;
  bool                m_useIdentityTableForNon420Chroma;
  int                 m_outputBitDepth[ MAX_NUM_CH ];                   ///< bit-depth of output file
  int                 m_MSBExtendedBitDepth[ MAX_NUM_CH ];              ///< bit-depth of input samples after MSB extension
  CostMode            m_costMode;                                       ///< Cost mode to use
  HashType            m_decodedPictureHashSEIType;                      ///< Checksum mode for decoded picture hash SEI message
  bool                m_bufferingPeriodSEIEnabled;
  bool                m_pictureTimingSEIEnabled;
  bool                m_decodingUnitInfoSEIEnabled;
  bool                m_entropyCodingSyncEnabled;
  bool                m_entryPointsPresent;
  bool                m_signalledSliceIdFlag;
  int                 m_signalledSliceIdLengthMinus1;
  std::vector<int>    m_rectSliceBoundary;
  std::vector<int>    m_signalledSliceId;
  std::vector<int>    m_sliceId;

  unsigned            m_CTUSize;
  unsigned            m_MinQT[ 3 ];                                     ///< 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned            m_maxMTTDepth;
  unsigned            m_maxMTTDepthI;
  unsigned            m_maxMTTDepthIChroma;
  unsigned            m_maxBT[3];
  unsigned            m_maxTT[3];
  bool                m_dualITree;
  unsigned            m_MaxCodingDepth;                                 ///< max. total CU depth - includes depth of transform-block structure
  unsigned            m_log2DiffMaxMinCodingBlockSize;                  ///< difference between largest and smallest CU depth
  int                 m_log2MaxTbSize;

  bool                m_bUseASR;                                        ///< flag for using adaptive motion search range
  bool                m_bUseHADME;                                      ///< flag for using HAD in sub-pel ME
  int                 m_RDOQ;                                           ///< flag for using RD optimized quantization
  bool                m_useRDOQTS;                                      ///< flag for using RD optimized quantization for transform skip
  bool                m_useSelectiveRDOQ;                               ///< flag for using selective RDOQ

  bool                m_JointCbCrMode;
  bool                m_cabacInitPresent;
  bool                m_useFastLCTU;
  bool                m_usePbIntraFast;
  int                 m_useFastMrg;
  bool                m_useAMaxBT;
  bool                m_fastQtBtEnc;
  bool                m_contentBasedFastQtbt;
  int                 m_fastInterSearchMode;                            ///< Parameter that controls fast encoder settings
  bool                m_bUseEarlyCU;                                    ///< flag for using Early CU setting
  bool                m_useFastDecisionForMerge;                        ///< flag for using Fast Decision Merge RD-Cost
  bool                m_useEarlySkipDetection;                          ///< flag for using Early SKIP Detection

  bool                m_bDisableIntraCUsInInterSlices;                  ///< Flag for disabling intra predicted CUs in inter slices.
  bool                m_bUseConstrainedIntraPred;                       ///< flag for using constrained intra prediction
  bool                m_bFastUDIUseMPMEnabled;
  bool                m_bFastMEForGenBLowDelayEnabled;

  bool                m_MTSImplicit;
  int                 m_TMVPModeId;
  bool                m_DepQuantEnabled;
  bool                m_SignDataHidingEnabled;
  bool                m_MIP;
  int                 m_useFastMIP;

  unsigned            m_maxNumMergeCand;                                ///< Max number of merge candidates
  unsigned            m_maxNumAffineMergeCand;                          ///< Max number of affine merge candidates
//  unsigned            m_maxNumIBCMergeCand;                             ///< Max number of IBC merge candidates
  int                 m_Geo;
  unsigned            m_maxNumGeoCand;
  int                 m_RCRateControlMode;
  int                 m_RCNumPasses;
  int                 m_RCTargetBitrate;
  int                 m_RCKeepHierarchicalBit;
  bool                m_RCUseLCUSeparateModel;
  int                 m_RCInitialQP;
  bool                m_RCForceIntraQP;
  int                 m_motionEstimationSearchMethod;
  bool                m_bRestrictMESampling;                            ///< Restrict sampling for the Selective ME
  int                 m_SearchRange;                                    ///< ME search range
  int                 m_bipredSearchRange;                              ///< ME search range for bipred refinement
  int                 m_minSearchWindow;                                ///< ME minimum search window size for the Adaptive Window ME
  bool                m_bClipForBiPredMeEnabled;                        ///< Enables clipping for Bi-Pred ME.
  bool                m_bFastMEAssumingSmootherMVEnabled;               ///< Enables fast ME assuming a smoother MV.
  int                 m_fastSubPel;
  int                 m_SMVD;
  int                 m_AMVRspeed;
  bool                m_LMChroma;
  bool                m_horCollocatedChromaFlag;
  bool                m_verCollocatedChromaFlag;
  bool                m_MRL;
  bool                m_BDOF;
  bool                m_DMVR;
  int                 m_EDO;
  bool                m_lumaReshapeEnable;
  int                 m_reshapeSignalType;
  int                 m_updateCtrl;
  int                 m_adpOption;
  int                 m_initialCW;
  int                 m_LMCSOffset;
  ReshapeCW           m_reshapeCW;
  int                 m_Affine;
  bool                m_PROF;
  bool                m_AffineType;
  int                 m_MMVD;
  int                 m_MmvdDisNum;
  bool                m_allowDisFracMMVD;
  int                 m_CIIP;
  bool                m_SbTMVP;
  int                 m_SBT;                                            ///< Sub-Block Transform for inter blocks
  int                 m_LFNST;
  int                 m_MTS;
  int                 m_MTSIntraMaxCand;
  int                 m_ISP;
  int                 m_TS;
  int                 m_TSsize;
  int                 m_useChromaTS;
  int                 m_useBDPCM;

  bool                m_bLoopFilterDisable;                             ///< flag for using deblocking filter
  bool                m_loopFilterOffsetInPPS;                          ///< offset for deblocking filter in 0 = slice header, 1 = PPS
  int                 m_loopFilterBetaOffsetDiv2[3];                    ///< beta offset for deblocking filter
  int                 m_loopFilterTcOffsetDiv2[3];                      ///< tc offset for deblocking filter
  int                 m_deblockingFilterMetric;

  bool                m_bLFCrossTileBoundaryFlag;
  bool                m_bLFCrossSliceBoundaryFlag;                      ///< 1: filter across slice boundaries 0: do not filter across slice boundaries
  bool                m_loopFilterAcrossSlicesEnabled;

  bool                m_bUseSAO;
  double              m_saoEncodingRate;                                ///< When >0 SAO early picture termination is enabled for luma and chroma
  double              m_saoEncodingRateChroma;                          ///< The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  unsigned            m_log2SaoOffsetScale[ MAX_NUM_CH ];               ///< number of bits for the upward bit shift operation on the decoded SAO offsets
  int                 m_saoOffsetBitShift[ MAX_NUM_CH ];

  bool                m_decodingParameterSetEnabled;                    ///< enable decoding parameter set
  bool                m_vuiParametersPresent;                           ///< enable generation of VUI parameters
  bool                m_hrdParametersPresent;                           ///< enable generation or HRD parameters 
  bool                m_aspectRatioInfoPresent;                         ///< Signals whether aspect_ratio_idc is present
  int                 m_aspectRatioIdc;                                 ///< aspect_ratio_idc
  int                 m_sarWidth;                                       ///< horizontal size of the sample aspect ratio
  int                 m_sarHeight;                                      ///< vertical size of the sample aspect ratio
  bool                m_colourDescriptionPresent;                       ///< Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  int                 m_colourPrimaries;                                ///< Indicates chromaticity coordinates of the source primaries
  int                 m_transferCharacteristics;                        ///< Indicates the opto-electronic transfer characteristics of the source
  int                 m_matrixCoefficients;                             ///< Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool                m_chromaLocInfoPresent;                           ///< Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  int                 m_chromaSampleLocTypeTopField;                    ///< Specifies the location of chroma samples for top field
  int                 m_chromaSampleLocTypeBottomField;                 ///< Specifies the location of chroma samples for bottom field
  int                 m_chromaSampleLocType;                            ///< Specifies the location of chroma samples for progressive content
  bool                m_overscanInfoPresent;                            ///< Signals whether overscan_appropriate_flag is present
  bool                m_overscanAppropriateFlag;                        ///< Indicates whether conformant decoded pictures are suitable for display using overscan
  bool                m_videoSignalTypePresent;                         ///< Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present
  bool                m_videoFullRangeFlag;                             ///< Indicates the black level and range of luma and chroma signals

  std::string         m_summaryOutFilename;                             ///< filename to use for producing summary output file.
  std::string         m_summaryPicFilenameBase;                         ///< Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  unsigned            m_summaryVerboseness;                             ///< Specifies the level of the verboseness of the text output.

  std::string         m_decodeBitstreams[ 2 ];                          ///< filename for decode bitstreams.
  int                 m_switchPOC;                                      ///< dbg poc.
  int                 m_switchDQP;                                      ///< switch DQP.
  int                 m_fastForwardToPOC;                               ///< get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC
  bool                m_stopAfterFFtoPOC;
  bool                m_bs2ModPOCAndType;
  bool                m_forceDecodeBitstream1;

  bool                m_alf;                                            ///> Adaptive Loop Filter
  bool                m_useNonLinearAlfLuma;
  bool                m_useNonLinearAlfChroma;
  unsigned            m_maxNumAlfAlternativesChroma;
  bool                m_ccalf;
  int                 m_ccalfQpThreshold;
  int                 m_MCTF;
  bool                m_MCTFFutureReference;
  int                 m_MCTFNumLeadFrames;
  int                 m_MCTFNumTrailFrames;
  std::vector<int>    m_MCTFFrames;
  std::vector<double> m_MCTFStrengths;

  int                 m_dqThresholdVal;
  bool                m_qtbttSpeedUp;

  int                 m_fastLocalDualTreeMode;
  bool                m_frameParallel;
  int                 m_numFppThreads;
  bool                m_ensureFppBitEqual;
  bool                m_picPartitionFlag;
public:

  EncCfgExpert()

    :   m_listTracingChannels                         ( false )
      , m_traceRule                                   ( "" )
      , m_traceFile                                   ( "" )

      , m_conformanceWindowMode                       ( 1 )
      , m_confWinLeft                                 ( 0 )
      , m_confWinRight                                ( 0 )
      , m_confWinTop                                  ( 0 )
      , m_confWinBottom                               ( 0 )

      , m_temporalSubsampleRatio                      ( 1 )

      , m_aiPad                                       { 0, 0 }
      , m_enablePictureHeaderInSliceHeader            ( true )
      , m_AccessUnitDelimiter                         ( false )

      , m_printMSEBasedSequencePSNR                   ( false )
      , m_printHexPsnr                                ( false )
      , m_printFrameMSE                               ( false )
      , m_printSequenceMSE                            ( false )
      , m_cabacZeroWordPaddingEnabled                 ( true )

      , m_subProfile                                  ( 0 )
      , m_bitDepthConstraintValue                     ( 10 )
      , m_intraOnlyConstraintFlag                     ( false )

      , m_InputQueueSize                              ( 0 )
      , m_rewriteParamSets                            ( false )
      , m_idrRefParamList                             ( false )
      , m_maxDecPicBuffering                          { 0, 0, 0, 0, 0, 0, 0 }           // not set -> derived
      , m_maxNumReorderPics                           { 0, 0, 0, 0, 0, 0, 0 }           // not set -> derived
      , m_maxTempLayer                                ( 0 )                             // not set -> derived
      , m_numRPLList0                                 ( 0 )                             // not set -> derived
      , m_numRPLList1                                 ( 0 )                             // not set -> derived

      , m_useSameChromaQPTables                       ( true )
      , m_chromaQpMappingTableParams                  ()
      , m_intraQPOffset                               ( 0 )
      , m_lambdaFromQPEnable                          ( false )
      , m_adLambdaModifier                            { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }
      , m_adIntraLambdaModifier                       ()
      , m_dIntraQpFactor                              ( -1.0 )
      , m_qpInValsCb                                  { 25, 33, 43 }
      , m_qpInValsCr                                  { 0 }
      , m_qpInValsCbCr                                { 0 }
      , m_qpOutValsCb                                 { 25, 32, 37 }
      , m_qpOutValsCr                                 { 0 }
      , m_qpOutValsCbCr                               { 0 }
      , m_cuQpDeltaSubdiv                             ( 0 )
      , m_cuChromaQpOffsetSubdiv                      ( -1 )
      , m_chromaCbQpOffset                            ( 0 )
      , m_chromaCrQpOffset                            ( 0 )
      , m_chromaCbQpOffsetDualTree                    ( 0 )
      , m_chromaCrQpOffsetDualTree                    ( 0 )
      , m_chromaCbCrQpOffset                          ( -1 )
      , m_chromaCbCrQpOffsetDualTree                  ( 0 )
      , m_sliceChromaQpOffsetPeriodicity              ( 0 )
      , m_sliceChromaQpOffsetIntraOrPeriodic          { 0 , 0 }

      , m_lumaLevelToDeltaQPEnabled                   (false)
      , m_wcgChromaQpControl                          ()
      , m_sdr                                         (false)
      , m_calculateHdrMetrics                         (false)
      , m_cropOffsetLeft                              (0)
      , m_cropOffsetTop                               (0)
      , m_cropOffsetRight                             (0)
      , m_cropOffsetBottom                            (0)

      , m_internChromaFormat                          ( NUM_CHROMA_FORMAT )
      , m_useIdentityTableForNon420Chroma             ( true )
      , m_outputBitDepth                              { 0, 0 }
      , m_MSBExtendedBitDepth                         { 0, 0 }
      , m_costMode                                    ( COST_STANDARD_LOSSY )
      , m_decodedPictureHashSEIType                   ( HASHTYPE_NONE )
      , m_bufferingPeriodSEIEnabled                   ( false )
      , m_pictureTimingSEIEnabled                     ( false )
      , m_decodingUnitInfoSEIEnabled                  ( false )
      , m_entropyCodingSyncEnabled                    ( false )
      , m_entryPointsPresent                          ( true )
      , m_signalledSliceIdFlag                        ( false )
      , m_signalledSliceIdLengthMinus1                ( 0 )
      , m_rectSliceBoundary                           ()
      , m_signalledSliceId                            ()
      , m_sliceId                                     ()                                // not set -> derived

      , m_CTUSize                                     ( 128 )
      , m_MinQT                                       { 8, 8, 4 }
      , m_maxMTTDepth                                  ( 3 )
      , m_maxMTTDepthI                                 ( 3 )
      , m_maxMTTDepthIChroma                           ( 3 )
      , m_maxBT                                       {32, 128, 64}
      , m_maxTT                                       {32, 64, 32}
      , m_dualITree                                   ( true )
      , m_MaxCodingDepth                              ( 0 )                             // not set -> derived
      , m_log2DiffMaxMinCodingBlockSize               ( 0 )                             // not set -> derived
      , m_log2MaxTbSize                               ( 6 )

      , m_bUseASR                                     ( false )
      , m_bUseHADME                                   ( true )
      , m_RDOQ                                        ( 1 )
      , m_useRDOQTS                                   ( true )
      , m_useSelectiveRDOQ                            ( false)

      , m_JointCbCrMode                               ( false )
      , m_cabacInitPresent                            ( true )
      , m_useFastLCTU                                 ( false )
      , m_usePbIntraFast                              ( false )
      , m_useFastMrg                                  ( 0 )
      , m_useAMaxBT                                   ( false )
      , m_fastQtBtEnc                                 ( true )
      , m_contentBasedFastQtbt                        ( false )
      , m_fastInterSearchMode                         ( FASTINTERSEARCH_DISABLED )
      , m_bUseEarlyCU                                 ( false )
      , m_useFastDecisionForMerge                     ( true )
      , m_useEarlySkipDetection                       ( false )

      , m_bDisableIntraCUsInInterSlices               ( false )
      , m_bUseConstrainedIntraPred                    ( false )
      , m_bFastUDIUseMPMEnabled                       ( true )
      , m_bFastMEForGenBLowDelayEnabled               ( true )

      , m_MTSImplicit                                 ( false )
      , m_TMVPModeId                                  ( 1 )
      , m_DepQuantEnabled                             ( true )
      , m_SignDataHidingEnabled                       ( false )

      , m_MIP                                         ( false )
      , m_useFastMIP                                  ( 0 )
      , m_maxNumMergeCand                             ( 5 )
      , m_maxNumAffineMergeCand                       ( 5 )
//    , m_maxNumIBCMergeCand                          ( 6 )
      , m_Geo                                         ( 0 )
      , m_maxNumGeoCand                               ( 5 )
      , m_RCRateControlMode                           ( 0 )
      , m_RCNumPasses                                 ( 1 )
      , m_RCTargetBitrate                             ( 0 )
      , m_RCKeepHierarchicalBit                       ( 0 )
      , m_RCUseLCUSeparateModel                       ( false )
      , m_RCInitialQP                                 ( 0 )
      , m_RCForceIntraQP                              ( false )
      , m_motionEstimationSearchMethod                ( 1 )
      , m_bRestrictMESampling                         ( false )
      , m_SearchRange                                 ( 96 )
      , m_bipredSearchRange                           ( 4 )
      , m_minSearchWindow                             ( 8 )
      , m_bClipForBiPredMeEnabled                     ( false )
      , m_bFastMEAssumingSmootherMVEnabled            ( true )
      , m_fastSubPel                                  ( 0 )
      , m_SMVD                                        ( 0 )
      , m_AMVRspeed                                   ( 0 )
      , m_LMChroma                                    ( false )
      , m_horCollocatedChromaFlag                     ( true )
      , m_verCollocatedChromaFlag                     ( false )
      , m_MRL                                         ( true )
      , m_BDOF                                        ( false )
      , m_DMVR                                        ( false )
      , m_EDO                                         ( 0 )
      , m_lumaReshapeEnable                           ( false )
      , m_reshapeSignalType                           ( 0 )
      , m_updateCtrl                                  ( 0 )
      , m_adpOption                                   ( 0 )
      , m_initialCW                                   ( 0 )
      , m_LMCSOffset                                  ( 0 )
      , m_reshapeCW                                   ( )
      , m_Affine                                      ( 0 )
      , m_PROF                                        ( false )
      , m_AffineType                                  ( true )
      , m_MMVD                                        ( 0 )
      , m_MmvdDisNum                                  ( 6 )
      , m_allowDisFracMMVD                            ( false )
      , m_CIIP                                        ( 0 )
      , m_SbTMVP                                      ( false )
      , m_SBT                                         ( 0 )
      , m_LFNST                                       ( 0 )
      , m_MTS                                         ( 0 )
      , m_MTSIntraMaxCand                             ( 3 ) 
      , m_ISP(0)
      , m_TS                                          ( 0 )
      , m_TSsize                                      ( 3 )
      , m_useChromaTS                                 ( 0 )
      , m_useBDPCM                                    ( 0 )

      , m_bLoopFilterDisable                          ( false )
      , m_loopFilterOffsetInPPS                       ( true )
      , m_loopFilterBetaOffsetDiv2                    { 0 }
      , m_loopFilterTcOffsetDiv2                      { 0 }
      , m_deblockingFilterMetric                      ( 0 )

      , m_bLFCrossTileBoundaryFlag                    ( true )
      , m_bLFCrossSliceBoundaryFlag                   ( true )
      , m_loopFilterAcrossSlicesEnabled               ( false )

      , m_bUseSAO                                     ( true )
      , m_saoEncodingRate                             ( 0.75 )
      , m_saoEncodingRateChroma                       ( 0.5 )
      , m_log2SaoOffsetScale                          { 0, 0 }                          // not set -> derived
      , m_saoOffsetBitShift                           { 0, 0 }

      , m_decodingParameterSetEnabled                 ( false )
      , m_vuiParametersPresent                        ( false )
      , m_hrdParametersPresent                        ( false )
      , m_aspectRatioInfoPresent                      ( false )
      , m_aspectRatioIdc                              ( 0 )
      , m_sarWidth                                    ( 0 )
      , m_sarHeight                                   ( 0 )
      , m_colourDescriptionPresent                    ( false )
      , m_colourPrimaries                             ( 2 )
      , m_transferCharacteristics                     ( 2 )
      , m_matrixCoefficients                          ( 2 )
      , m_chromaLocInfoPresent                        ( false )
      , m_chromaSampleLocTypeTopField                 ( 0 )
      , m_chromaSampleLocTypeBottomField              ( 0 )
      , m_chromaSampleLocType                         ( 0 )
      , m_overscanInfoPresent                         ( false )
      , m_overscanAppropriateFlag                     ( false )
      , m_videoSignalTypePresent                      ( false )
      , m_videoFullRangeFlag                          ( false )

      , m_summaryOutFilename                          ( "" )
      , m_summaryPicFilenameBase                      ( "" )
      , m_summaryVerboseness                          ( 0 )

      , m_decodeBitstreams                            { "", "" }
      , m_switchPOC                                   ( -1 )
      , m_switchDQP                                   ( 0 )
      , m_fastForwardToPOC                            ( -1 )
      , m_stopAfterFFtoPOC                            ( false )
      , m_bs2ModPOCAndType                            ( false )
      , m_forceDecodeBitstream1                       ( false )

      , m_alf                                         ( false )
      , m_useNonLinearAlfLuma                         ( true )
      , m_useNonLinearAlfChroma                       ( true )
      , m_maxNumAlfAlternativesChroma                 ( MAX_NUM_ALF_ALTERNATIVES_CHROMA )
      , m_ccalf                                       ( false )
      , m_ccalfQpThreshold                            ( 37 )

      , m_MCTF                                        ( 0 )
      , m_MCTFFutureReference                         ( true )
      , m_MCTFNumLeadFrames                           ( 0 )
      , m_MCTFNumTrailFrames                          ( 0 )

      , m_dqThresholdVal                              ( 8 )
      , m_qtbttSpeedUp                                ( false )

      , m_fastLocalDualTreeMode                       ( 0 )
      , m_frameParallel                               ( false )
      , m_numFppThreads                               ( -1 )
      , m_ensureFppBitEqual                           ( false )
      , m_picPartitionFlag                            ( false )
  {
  }

  virtual ~EncCfgExpert()
  {
  }
};


} // namespace vvenc

//! \}


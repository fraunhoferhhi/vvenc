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
/** \file     EncCfgExpert.h
    \brief    encoder configuration class (header)
*/

#pragma once

#include <cstring>
#include <vector>
#include <string>
#include <sstream>
#include "vvenc/vvencDecl.h"

//! \ingroup Interface
//! \{

namespace vvenc {


// ====================================================================================================================

static const int MAX_GOP                         = 64; ///< max. value of hierarchical GOP size
static const int MAX_NUM_REF_PICS                = 29; ///< max. number of pictures used for reference
static const int MAX_TLAYER                      =  7; ///< Explicit temporal layer QP offset - max number of temporal layer
static const int MAX_NUM_CQP_MAPPING_TABLES      =  3; ///< Maximum number of chroma QP mapping tables (Cb, Cr and joint Cb-Cr)
static const int MAX_NUM_ALF_ALTERNATIVES_CHROMA =  8;
static const int MCTF_RANGE                      =  2; ///< max number of frames used for MCTF filtering in forward / backward direction

// ====================================================================================================================


enum ChannelType
{
  CH_L = 0,
  CH_C = 1,
  MAX_NUM_CH = 2
};

enum ComponentID
{
  COMP_Y          = 0,
  COMP_Cb         = 1,
  COMP_Cr         = 2,
  MAX_NUM_COMP    = 3,
  COMP_JOINT_CbCr = MAX_NUM_COMP,
  MAX_NUM_TBLOCKS = MAX_NUM_COMP
};

enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum RateControlMode
{
  RCM_AUTO          = -1,
  RCM_OFF           = 0,
  RCM_CTU_LEVEL     = 1,
  RCM_PICTURE_LEVEL = 2,
  RCM_GOP_LEVEL     = 3
};

enum FastInterSearchMode
{
  FASTINTERSEARCH_AUTO     = 0,
  FASTINTERSEARCH_MODE1    = 1,
  FASTINTERSEARCH_MODE2    = 2,
  FASTINTERSEARCH_MODE3    = 3
};

/// supported ME search methods
enum MESearchMethod
{
  MESEARCH_FULL              = 0,
  MESEARCH_DIAMOND           = 1,
  MESEARCH_SELECTIVE         = 2,
  MESEARCH_DIAMOND_ENHANCED  = 3,
  MESEARCH_DIAMOND_FAST      = 4,
  MESEARCH_NUMBER_OF_METHODS = 5
};

enum CostMode
{
  COST_STANDARD_LOSSY              = 0,
  COST_SEQUENCE_LEVEL_LOSSLESS     = 1,
  COST_LOSSLESS_CODING             = 2,
  COST_MIXED_LOSSLESS_LOSSY_CODING = 3
};

enum HashType
{
  HASHTYPE_MD5        = 0,
  HASHTYPE_CRC        = 1,
  HASHTYPE_CHECKSUM   = 2,
  HASHTYPE_NONE       = 3,
  NUMBER_OF_HASHTYPES = 4
};

// ====================================================================================================================

struct VVENC_DECL GOPEntry
{
  int    m_POC                       = -1;
  int    m_QPOffset                  = 0;
  double m_QPOffsetModelOffset       = 0.0;
  double m_QPOffsetModelScale        = 0.0;
  int    m_CbQPoffset                = 0;
  int    m_CrQPoffset                = 0;
  double m_QPFactor                  = 0.0;
  int    m_tcOffsetDiv2              = 0;
  int    m_betaOffsetDiv2            = 0;
  int    m_CbTcOffsetDiv2            = 0;
  int    m_CbBetaOffsetDiv2          = 0;
  int    m_CrTcOffsetDiv2            = 0;
  int    m_CrBetaOffsetDiv2          = 0;
  int    m_temporalId                = 0;
  bool   m_refPic                    = false;
  int8_t m_sliceType                 = 'P';
  int    m_numRefPicsActive[ 2 ]     = { 0 };
  int    m_numRefPics[ 2 ]           = { 0 };
  int    m_deltaRefPics[ 2 ][ MAX_NUM_REF_PICS ];
  bool   m_isEncoded                 = false;
  bool   m_ltrp_in_slice_header_flag = false;

  GOPEntry()
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
  int    m_POC                              = -1;
  int    m_temporalId                       = 0;
  bool   m_refPic                           = false;
  bool   m_ltrp_in_slice_header_flag        = false;
  int    m_numRefPicsActive                 = 0;
  int8_t m_sliceType                        ='P';
  int    m_numRefPics                       = 0;
  int    m_deltaRefPics[ MAX_NUM_REF_PICS ] = { 0 };
};

struct VVENC_DECL WCGChromaQPControl
{
  bool   enabled         = false;  ///< Enabled flag (0:default)
  double chromaCbQpScale = 1.0;    ///< Chroma Cb QP Scale (1.0:default)
  double chromaCrQpScale = 1.0;    ///< Chroma Cr QP Scale (1.0:default)
  double chromaQpScale   = 0.0;    ///< Chroma QP Scale (0.0:default)
  double chromaQpOffset  = 0-0;    ///< Chroma QP Offset (0.0:default)
};

struct VVENC_DECL ChromaQpMappingTableParams
{
  int               m_numQpTables                                          = 0;
  int               m_qpBdOffset                                           = 12;
  bool              m_sameCQPTableForAllChromaFlag                         = true;
  int               m_qpTableStartMinus26[MAX_NUM_CQP_MAPPING_TABLES]      = { 0 };
  int               m_numPtsInCQPTableMinus1[ MAX_NUM_CQP_MAPPING_TABLES ] = { 0 };
  std::vector<int>  m_deltaQpInValMinus1[ MAX_NUM_CQP_MAPPING_TABLES ];
  std::vector<int>  m_deltaQpOutVal[ MAX_NUM_CQP_MAPPING_TABLES ];
};

struct VVENC_DECL ReshapeCW
{
  std::vector<uint32_t> binCW;
  int       updateCtrl;
  int       adpOption;
  uint32_t  initialCW;
  int       rspPicSize;
  int       rspFps;
  int       rspBaseQP;
  int       rspTid;
  int       rspFpsToIp;
};

// ====================================================================================================================

/// encoder configuration class
class VVENC_DECL VVEncCfgExpert
{
public:

  bool                m_listTracingChannels                     = false;
  std::string         m_traceRule                               = "";
  std::string         m_traceFile                               = "";

  int                 m_conformanceWindowMode                   = 1;
  int                 m_confWinLeft                             = 0;
  int                 m_confWinRight                            = 0;
  int                 m_confWinTop                              = 0;
  int                 m_confWinBottom                           = 0;

  unsigned            m_temporalSubsampleRatio                  = 1;                                     ///< temporal subsample ratio, 2 means code every two frames

  int                 m_PadSourceWidth                          = 0;                                     ///< source width in pixel
  int                 m_PadSourceHeight                         = 0;                                     ///< source height in pixel (when interlaced = field height)

  int                 m_aiPad[ 2 ]                              = { 0, 0 };                              ///< number of padded pixels for width and height
  bool                m_enablePictureHeaderInSliceHeader        = true;
  int                 m_AccessUnitDelimiter                     = -1;                                    ///< add Access Unit Delimiter NAL units, default: auto (only enable if needed by dependent options)

  bool                m_printMSEBasedSequencePSNR               = false;
  bool                m_printHexPsnr                            = false;
  bool                m_printFrameMSE                           = false;
  bool                m_printSequenceMSE                        = false;
  bool                m_cabacZeroWordPaddingEnabled             = true;

  unsigned            m_subProfile                              = 0;
  unsigned            m_bitDepthConstraintValue                 = 10;
  bool                m_intraOnlyConstraintFlag                 = false;

  int                 m_InputQueueSize                          = 0;                                     ///< Size of frame input queue
  bool                m_rewriteParamSets                        = true;                                  ///< Flag to enable rewriting of parameter sets at random access points
  bool                m_idrRefParamList                         = false;                                 ///< indicates if reference picture list syntax elements are present in slice headers of IDR pictures
  RPLEntry            m_RPLList0[ MAX_GOP ];                                                             ///< the RPL entries from the config file
  RPLEntry            m_RPLList1[ MAX_GOP ];                                                             ///< the RPL entries from the config file
  GOPEntry            m_GOPList [ MAX_GOP ];                                                             ///< the coding structure entries from the config file
  int                 m_maxDecPicBuffering[ MAX_TLAYER ]        = { 0, 0, 0, 0, 0, 0, 0 };               ///< total number of pictures in the decoded picture buffer
  int                 m_maxNumReorderPics [ MAX_TLAYER ]        = { 0, 0, 0, 0, 0, 0, 0 };               ///< total number of reorder pictures
  int                 m_maxTempLayer                            = 0;                                     ///< Max temporal layer
  int                 m_numRPLList0                             = 0;
  int                 m_numRPLList1                             = 0;

  bool                m_useSameChromaQPTables                   = true;
  ChromaQpMappingTableParams m_chromaQpMappingTableParams       = ChromaQpMappingTableParams();
  int                 m_intraQPOffset                           = 0;                                     ///< QP offset for intra slice (integer)
  bool                m_lambdaFromQPEnable                      = false;                                 ///< enable flag for QP:lambda fix
  double              m_adLambdaModifier[ MAX_TLAYER ]          = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 }; ///< Lambda modifier array for each temporal layer
  std::vector<double> m_adIntraLambdaModifier;                                                           ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  double              m_dIntraQpFactor                          = -1.0;                                  ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))
  std::vector<int>    m_qpInValsCb                              = { 17, 22, 34, 42 };                    ///< qp input values used to derive the chroma QP mapping table
  std::vector<int>    m_qpInValsCr                              = { 0 };                                 ///< qp input values used to derive the chroma QP mapping table
  std::vector<int>    m_qpInValsCbCr                            = { 0 };                                 ///< qp input values used to derive the chroma QP mapping table
  std::vector<int>    m_qpOutValsCb                             = { 17, 23, 35, 39 };                    ///< qp output values used to derive the chroma QP mapping table
  std::vector<int>    m_qpOutValsCr                             = { 0 };                                 ///< qp output values used to derive the chroma QP mapping table
  std::vector<int>    m_qpOutValsCbCr                           = { 0 };                                 ///< qp output values used to derive the chroma QP mapping table
  int                 m_cuQpDeltaSubdiv                         = -1;                                    ///< Maximum subdiv for CU luma Qp adjustment (0:default)
  int                 m_cuChromaQpOffsetSubdiv                  = -1;                                    ///< If negative, then do not apply chroma qp offsets.
  int                 m_chromaCbQpOffset                        = 0;                                     ///< Chroma Cb QP Offset (0:default)
  int                 m_chromaCrQpOffset                        = 0;                                     ///< Chroma Cr QP Offset (0:default)
  int                 m_chromaCbQpOffsetDualTree                = 0;                                     ///< Chroma Cb QP Offset for dual tree (overwrite m_chromaCbQpOffset for dual tree)
  int                 m_chromaCrQpOffsetDualTree                = 0;                                     ///< Chroma Cr QP Offset for dual tree (overwrite m_chromaCrQpOffset for dual tree)
  int                 m_chromaCbCrQpOffset                      = -1;                                    ///< QP Offset for joint Cb-Cr mode
  int                 m_chromaCbCrQpOffsetDualTree              = 0;                                     ///< QP Offset for joint Cb-Cr mode (overwrite m_chromaCbCrQpOffset for dual tree)
  int                 m_sliceChromaQpOffsetPeriodicity          = -1;                                    ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  int                 m_sliceChromaQpOffsetIntraOrPeriodic[ 2 ] = { 0, 0};                               ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.

  int                 m_usePerceptQPATempFiltISlice             = -1;                                    ///< Flag indicating if temporal high-pass filtering in visual activity calculation in QPA should (true) or shouldn't (false) be applied for I-slices

  bool                m_lumaLevelToDeltaQPEnabled               = false;
  WCGChromaQPControl  m_wcgChromaQpControl                      = WCGChromaQPControl();
  bool                m_sdr                                     = false;                  // unused, only for conmpatiblity
  bool                m_calculateHdrMetrics                     = false;                  // unused, only for conmpatiblity

  int                 m_cropOffsetLeft                          = 0;                      // unused, only for conmpatiblity
  int                 m_cropOffsetTop                           = 0;                      // unused, only for conmpatiblity
  int                 m_cropOffsetRight                         = 0;                      // unused, only for conmpatiblity
  int                 m_cropOffsetBottom                        = 0;                      // unused, only for conmpatiblity

  ChromaFormat        m_internChromaFormat                      = NUM_CHROMA_FORMAT;
  bool                m_useIdentityTableForNon420Chroma         = true;
  int                 m_outputBitDepth[ MAX_NUM_CH ]            = { 0, 0 };                              ///< bit-depth of output file
  int                 m_MSBExtendedBitDepth[ MAX_NUM_CH ]       = { 0, 0 };                              ///< bit-depth of input samples after MSB extension
  CostMode            m_costMode                                = COST_STANDARD_LOSSY;                   ///< Cost mode to use

  HashType            m_decodedPictureHashSEIType               = HASHTYPE_NONE;                         ///< Checksum mode for decoded picture hash SEI message
  bool                m_bufferingPeriodSEIEnabled               = false;
  bool                m_pictureTimingSEIEnabled                 = false;
  bool                m_decodingUnitInfoSEIEnabled              = false;

  bool                m_entropyCodingSyncEnabled                = false;
  bool                m_entryPointsPresent                      = true;

  bool                m_signalledSliceIdFlag                    = false;         // unused, only for conmpatiblity
  int                 m_signalledSliceIdLengthMinus1            = 0;             // unused, only for conmpatiblity
  std::vector<int>    m_rectSliceBoundary;                                       // unused, only for conmpatiblity
  std::vector<int>    m_signalledSliceId;                                        // unused, only for conmpatiblity
  std::vector<int>    m_sliceId;                                                 // unused, only for conmpatiblity

  unsigned            m_CTUSize                                 = 128;
  unsigned            m_MinQT[ 3 ]                              = { 8, 8, 4 };                           ///< 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned            m_maxMTTDepth                             = 3;
  unsigned            m_maxMTTDepthI                            = 3;
  unsigned            m_maxMTTDepthIChroma                      = 3;
  unsigned            m_maxBT[3]                                = {32, 128, 64};
  unsigned            m_maxTT[3]                                = {32, 64, 32};
  bool                m_dualITree                               = true;
  unsigned            m_MaxCodingDepth                          = 0;                                     ///< max. total CU depth - includes depth of transform-block structure
  unsigned            m_log2DiffMaxMinCodingBlockSize           = 0;                                     ///< difference between largest and smallest CU depth
  int                 m_log2MaxTbSize                           = 6;
  int                 m_log2MinCodingBlockSize                  = 2;

  bool                m_bUseASR                                 = false;                                 ///< flag for using adaptive motion search range
  bool                m_bUseHADME                               = true;                                  ///< flag for using HAD in sub-pel ME
  int                 m_RDOQ                                    = 1;                                     ///< flag for using RD optimized quantization
  bool                m_useRDOQTS                               = true;                                  ///< flag for using RD optimized quantization for transform skip
  bool                m_useSelectiveRDOQ                        = false;                                 ///< flag for using selective RDOQ

  bool                m_JointCbCrMode                           = false;
  int                 m_cabacInitPresent                        = -1;
  bool                m_useFastLCTU                             = false;
  bool                m_usePbIntraFast                          = false;
  int                 m_useFastMrg                              = 0;
  int                 m_useAMaxBT                               = -1;
  bool                m_fastQtBtEnc                             = true;
  bool                m_contentBasedFastQtbt                    = false;
  int                 m_fastInterSearchMode                     = FASTINTERSEARCH_AUTO;              ///< Parameter that controls fast encoder settings
  bool                m_bUseEarlyCU                             = false;                                 ///< flag for using Early CU setting
  bool                m_useFastDecisionForMerge                 = true;                                  ///< flag for using Fast Decision Merge RD-Cost
  bool                m_useEarlySkipDetection                   = false;                                 ///< flag for using Early SKIP Detection

  bool                m_bDisableIntraCUsInInterSlices           = false;                                 ///< Flag for disabling intra predicted CUs in inter slices.
  bool                m_bUseConstrainedIntraPred                = false;                                 ///< flag for using constrained intra prediction
  bool                m_bFastUDIUseMPMEnabled                   = true;
  bool                m_bFastMEForGenBLowDelayEnabled           = true;

  bool                m_MTSImplicit                             = false;
  int                 m_TMVPModeId                              = 1;
  bool                m_DepQuantEnabled                         = true;
  bool                m_SignDataHidingEnabled                   = false;

  bool                m_MIP                                     = false;
  int                 m_useFastMIP                              = 0;

  unsigned            m_maxNumMergeCand                         = 5;                                     ///< Max number of merge candidates
  unsigned            m_maxNumAffineMergeCand                   = 5;                                     ///< Max number of affine merge candidates
  int                 m_Geo                                     = 0;
  unsigned            m_maxNumGeoCand                           = 5;
  int                 m_FastIntraTools                          = 0;

  int                 m_RCInitialQP                             = 0;
  bool                m_RCForceIntraQP                          = false;

  int                 m_motionEstimationSearchMethod            = MESEARCH_DIAMOND;
  bool                m_bRestrictMESampling                     = false;                                 ///< Restrict sampling for the Selective ME
  int                 m_SearchRange                             = 96;                                    ///< ME search range
  int                 m_bipredSearchRange                       = 4;                                     ///< ME search range for bipred refinement
  int                 m_minSearchWindow                         = 8;                                     ///< ME minimum search window size for the Adaptive Window ME
  bool                m_bClipForBiPredMeEnabled                 = false;                                 ///< Enables clipping for Bi-Pred ME.
  bool                m_bFastMEAssumingSmootherMVEnabled        = true;                                  ///< Enables fast ME assuming a smoother MV.
  int                 m_fastSubPel                              = 0;
  int                 m_SMVD                                    = 0;
  int                 m_AMVRspeed                               = 0;
  bool                m_LMChroma                                = false;
  bool                m_horCollocatedChromaFlag                 = true;
  bool                m_verCollocatedChromaFlag                 = false;
  bool                m_MRL                                     = true;
  bool                m_BDOF                                    = false;
  bool                m_DMVR                                    = false;
  int                 m_EDO                                     = 0;
  bool                m_lumaReshapeEnable                       = false;
  int                 m_reshapeSignalType                       = 0;
  int                 m_updateCtrl                              = 0;
  int                 m_adpOption                               = 0;
  int                 m_initialCW                               = 0;
  int                 m_LMCSOffset                              = 0;
  ReshapeCW           m_reshapeCW;
  int                 m_Affine                                  = 0;
  bool                m_PROF                                    = false;
  bool                m_AffineType                              = true;
  int                 m_MMVD                                    = 0;
  int                 m_MmvdDisNum                              = 6;
  bool                m_allowDisFracMMVD                        = false;
  int                 m_CIIP                                    = 0;
  bool                m_SbTMVP                                  = false;
  int                 m_SBT                                     = 0;                                     ///< Sub-Block Transform for inter blocks
  int                 m_LFNST                                   = 0;
  int                 m_MTS                                     = 0;
  int                 m_MTSIntraMaxCand                         = 3;
  int                 m_ISP                                     = 0;
  int                 m_TS                                      = 0;
  int                 m_TSsize                                  = 3;
  int                 m_useChromaTS                             = 0;
  int                 m_useBDPCM                                = 0;

  int                 m_rprEnabledFlag                          = 1;
  bool                m_resChangeInClvsEnabled                  = false;
  bool                m_craAPSreset                             = false;
  bool                m_rprRASLtoolSwitch                       = false;

#if 1 // IBC_VTM
  int                 m_IBCMode                                 = 0;
  int                 m_IBCFastMethod                           = 1;
#endif

  bool                m_bLoopFilterDisable                      = false;                                 ///< flag for using deblocking filter
  bool                m_loopFilterOffsetInPPS                   = true;                                  ///< offset for deblocking filter in 0 = slice header, 1 = PPS
  int                 m_loopFilterBetaOffsetDiv2[3]             = { 0 };                                 ///< beta offset for deblocking filter
  int                 m_loopFilterTcOffsetDiv2[3]               = { 0 };                                 ///< tc offset for deblocking filter
  int                 m_deblockingFilterMetric                  = 0;

  bool                m_bLFCrossTileBoundaryFlag                = true;
  bool                m_bLFCrossSliceBoundaryFlag               = true;                                  ///< 1: filter across slice boundaries 0: do not filter across slice boundaries
  bool                m_loopFilterAcrossSlicesEnabled           = false;

  bool                m_bUseSAO                                 = true;
  double              m_saoEncodingRate                         = -1.0;                                  ///< When >0 SAO early picture termination is enabled for luma and chroma
  double              m_saoEncodingRateChroma                   = -1.0;                                  ///< The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  unsigned            m_log2SaoOffsetScale[ MAX_NUM_CH ]        = { 0, 0 };                              ///< n umber of bits for the upward bit shift operation on the decoded SAO offsets
  int                 m_saoOffsetBitShift[ MAX_NUM_CH ]         = { 0, 0 };

  bool                m_decodingParameterSetEnabled             = false;                                 ///< enable decoding parameter set
  int                 m_vuiParametersPresent                    = -1;                                    ///< enable generation of VUI parameters; -1 auto enable, 0: off 1: enable
  int                 m_hrdParametersPresent                    = -1;                                    ///< enable generation or HRD parameters; -1 auto enable, 0: off 1: enable
  bool                m_aspectRatioInfoPresent                  = false;                                 ///< Signals whether aspect_ratio_idc is present
  int                 m_aspectRatioIdc                          = 0;                                     ///< aspect_ratio_idc
  int                 m_sarWidth                                = 0;                                     ///< horizontal size of the sample aspect ratio
  int                 m_sarHeight                               = 0;                                     ///< vertical size of the sample aspect ratio
  bool                m_colourDescriptionPresent                = false;                                 ///< Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  int                 m_colourPrimaries                         = 2;                                     ///< Indicates chromaticity coordinates of the source primaries
  int                 m_transferCharacteristics                 = 2;                                     ///< Indicates the opto-electronic transfer characteristics of the source
  int                 m_matrixCoefficients                      = 2;                                     ///< Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool                m_chromaLocInfoPresent                    = false;                                 ///< Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  int                 m_chromaSampleLocTypeTopField             = 0;                                     ///< Specifies the location of chroma samples for top field
  int                 m_chromaSampleLocTypeBottomField          = 0;                                     ///< Specifies the location of chroma samples for bottom field
  int                 m_chromaSampleLocType                     = 0;                                     ///< Specifies the location of chroma samples for progressive content
  bool                m_overscanInfoPresent                     = false;                                 ///< Signals whether overscan_appropriate_flag is present
  bool                m_overscanAppropriateFlag                 = false;                                 ///< Indicates whether conformant decoded pictures are suitable for display using overscan
  bool                m_videoSignalTypePresent                  = false;                                 ///< Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present
  bool                m_videoFullRangeFlag                      = false;                                 ///< Indicates the black level and range of luma and chroma signals

  std::vector<uint32_t> m_masteringDisplay;                                                              ///< mastering display colour volume, vector of size 10, format: G(x,y)B(x,y)R(x,y)WP(x,y)L(max,min), 0 <= GBR,WP <= 50000, 0 <= L <= uint (SEI)
                                                                                                         ///< GBR xy coordinates in increments of 1/50000 (in the ranges 0 to 50000) (e.g. 0.333 = 16667)
                                                                                                         ///< min/max luminance value in units of 1/10000 candela per square metre
  std::vector<uint32_t> m_contentLightLevel;                                                             ///< upper bound on the max light level and max avg light level among all individual samples in a 4:4:4 representation. in units of candelas per square metre (SEI)
  int                 m_preferredTransferCharacteristics        = -1;                                    ///< Alternative transfer characteristics SEI which will override the corresponding entry in the VUI, if < 0 SEI is not written")

  std::string         m_summaryOutFilename                      = "";                                    ///< filename to use for producing summary output file.
  std::string         m_summaryPicFilenameBase                  = "";                                    ///< Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  unsigned            m_summaryVerboseness                      = 0;                                     ///< Specifies the level of the verboseness of the text output.

  std::string         m_decodeBitstreams[ 2 ]                   = { "", "" };                            ///< filename for decode bitstreams.
  int                 m_switchPOC                               = -1;                                    ///< dbg poc.
  int                 m_switchDQP                               = 0;                                     ///< switch DQP.
  int                 m_fastForwardToPOC                        = -1;                                    ///< get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC
  bool                m_stopAfterFFtoPOC                        = false;
  bool                m_bs2ModPOCAndType                        = false;
  bool                m_forceDecodeBitstream1                   = false;

  bool                m_alf                                     = false;                                 ///> Adaptive Loop Filter
  bool                m_useNonLinearAlfLuma                     = true;
  bool                m_useNonLinearAlfChroma                   = true;
  unsigned            m_maxNumAlfAlternativesChroma             = MAX_NUM_ALF_ALTERNATIVES_CHROMA;
  bool                m_ccalf                                   = false;
  int                 m_ccalfQpThreshold                        = 37;
  int                 m_alfTempPred                             = -1;                                    ///> Indicates using of temporal filter data prediction through APS

  int                 m_MCTF                                    = 0;
  bool                m_MCTFFutureReference                     = true;
  int                 m_MCTFNumLeadFrames                       = 0;
  int                 m_MCTFNumTrailFrames                      = 0;
  std::vector<int>    m_MCTFFrames;
  std::vector<double> m_MCTFStrengths;

  int                 m_dqThresholdVal                          = 8;
  int                 m_qtbttSpeedUp                            = 0;

  int                 m_fastLocalDualTreeMode                   = 0;

  int                 m_maxParallelFrames                       = -1;
  int                 m_ensureWppBitEqual                       = -1;            ///< Flag indicating bit equalitiy for single thread runs respecting multithread restrictions

  bool                m_picPartitionFlag                        = false;
public:

  VVEncCfgExpert()
  {
  }

  virtual ~VVEncCfgExpert()
  {
  }
};


static inline ChannelType toChannelType             (const ComponentID id)                         { return ((int)id==(int)COMP_Y)? CH_L : CH_C;                     }
static inline uint32_t    getChannelTypeScaleX      (const ChannelType id, const ChromaFormat fmt) { return ((int)id==(int)COMP_Y || (fmt==CHROMA_444)) ? 0 : 1;     }
static inline uint32_t    getChannelTypeScaleY      (const ChannelType id, const ChromaFormat fmt) { return ((int)id==(int)COMP_Y || (fmt!=CHROMA_420)) ? 0 : 1;     }
static inline uint32_t    getComponentScaleX        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleX(toChannelType(id), fmt);  }
static inline uint32_t    getComponentScaleY        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleY(toChannelType(id), fmt);  }
static inline uint32_t    getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMP;          }
static inline uint32_t    getNumberValidChannels    (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CH;            }
///< checks if library has tracing supported enabled (see ENABLE_TRACING).
bool   VVENC_DECL  isTracingEnabled();
///< creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
std::string VVENC_DECL getCompileInfoString();
void   VVENC_DECL  decodeBitstream( const std::string& FileName);
int    VVENC_DECL  getWidthOfComponent( const ChromaFormat& chFmt, const int frameWidth, const int compId );
int    VVENC_DECL  getHeightOfComponent( const ChromaFormat& chFmt, const int frameHeight, const int compId );

} // namespace vvenc

//! \}


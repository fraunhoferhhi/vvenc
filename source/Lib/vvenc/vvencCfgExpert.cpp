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


/** \file     vvenCfgExpert.cpp
    \brief    encoder configuration class
*/

#include "vvenc/vvencCfgExpert.h"

#ifdef __cplusplus
extern "C" {
#endif

VVENC_NAMESPACE_BEGIN



VVENC_DECL void vvenc_GOPEntry_default(vvencGOPEntry *GOPEntry )
{
  GOPEntry->m_POC                       = -1;
  GOPEntry->m_QPOffset                  = 0;
  GOPEntry->m_QPOffsetModelOffset       = 0.0;
  GOPEntry->m_QPOffsetModelScale        = 0.0;
  GOPEntry->m_CbQPoffset                = 0;
  GOPEntry->m_CrQPoffset                = 0;
  GOPEntry->m_QPFactor                  = 0.0;
  GOPEntry->m_tcOffsetDiv2              = 0;
  GOPEntry->m_betaOffsetDiv2            = 0;
  GOPEntry->m_CbTcOffsetDiv2            = 0;
  GOPEntry->m_CbBetaOffsetDiv2          = 0;
  GOPEntry->m_CrTcOffsetDiv2            = 0;
  GOPEntry->m_CrBetaOffsetDiv2          = 0;
  GOPEntry->m_temporalId                = 0;
  GOPEntry->m_refPic                    = false;
  GOPEntry->m_sliceType                 = 'P';
  memset( GOPEntry->m_numRefPicsActive, 0, sizeof( GOPEntry->m_numRefPicsActive ) );
  memset( GOPEntry->m_numRefPics, 0, sizeof( GOPEntry->m_numRefPics ) );
  memset( GOPEntry->m_deltaRefPics, 0, sizeof( GOPEntry->m_deltaRefPics ) );
  GOPEntry->m_isEncoded                 = false;
  GOPEntry->m_ltrp_in_slice_header_flag = false;
}

VVENC_DECL void vvenc_WCGChromaQPControl_default(vvencWCGChromaQPControl *WCGChromaQPControl )
{
  WCGChromaQPControl->enabled         = false;  ///< Enabled flag (0:default)
  WCGChromaQPControl->chromaCbQpScale = 1.0;    ///< Chroma Cb QP Scale (1.0:default)
  WCGChromaQPControl->chromaCrQpScale = 1.0;    ///< Chroma Cr QP Scale (1.0:default)
  WCGChromaQPControl->chromaQpScale   = 0.0;    ///< Chroma QP Scale (0.0:default)
  WCGChromaQPControl->chromaQpOffset  = 0-0;    ///< Chroma QP Offset (0.0:default)
}

VVENC_DECL void vvenc_ChromaQpMappingTableParams_default(vvencChromaQpMappingTableParams *p )
{
  p->m_numQpTables                                          = 0;
  p->m_qpBdOffset                                           = 12;
  p->m_sameCQPTableForAllChromaFlag                         = true;

  memset( p->m_qpTableStartMinus26, 0, sizeof( p->m_qpTableStartMinus26 ) );
  memset( p->m_numPtsInCQPTableMinus1, 0, sizeof( p->m_numPtsInCQPTableMinus1 ) );
  memset( p->m_deltaQpInValMinus1, 16, sizeof( p->m_deltaQpInValMinus1 ) );
  memset( p->m_deltaQpOutVal, 16, sizeof( p->m_deltaQpOutVal ) );
}

VVENC_DECL void vvenc_RPLEntry_default(vvencRPLEntry *RPLEntry )
{
  RPLEntry->m_POC                              = -1;
  RPLEntry->m_temporalId                       = 0;
  RPLEntry->m_refPic                           = false;
  RPLEntry->m_ltrp_in_slice_header_flag        = false;
  RPLEntry->m_numRefPicsActive                 = 0;
  RPLEntry->m_sliceType                        ='P';
  RPLEntry->m_numRefPics                       = 0;

  memset(&RPLEntry->m_deltaRefPics,0, sizeof(RPLEntry->m_deltaRefPics));
}

VVENC_DECL void vvenc_ReshapeCW_default(vvencReshapeCW *reshapeCW )
{
  memset( reshapeCW->binCW, 0, sizeof( reshapeCW->binCW ) );
  reshapeCW->updateCtrl = 0;
  reshapeCW->adpOption  = 0;
  reshapeCW->initialCW  = 0;
  reshapeCW->rspPicSize = 0;
  reshapeCW->rspFps     = 0;
  reshapeCW->rspBaseQP  = 0;
  reshapeCW->rspTid     = 0;
  reshapeCW->rspFpsToIp = 0;
}


VVENC_DECL void vvenc_cfgExpert_default(vvencCfgExpert *cfgExpert )
{
  int i = 0;
  cfgExpert->m_listTracingChannels                     = false;
  cfgExpert->m_traceRule                               = NULL;
  cfgExpert->m_traceFile                               = NULL;

  cfgExpert->m_conformanceWindowMode                   = 1;
  cfgExpert->m_confWinLeft                             = 0;
  cfgExpert->m_confWinRight                            = 0;
  cfgExpert->m_confWinTop                              = 0;
  cfgExpert->m_confWinBottom                           = 0;

  cfgExpert->m_temporalSubsampleRatio                  = 1;                                     ///< temporal subsample ratio, 2 means code every two frames

  cfgExpert->m_PadSourceWidth                          = 0;                                     ///< source width in pixel
  cfgExpert->m_PadSourceHeight                         = 0;                                     ///< source height in pixel (when interlaced = field height)

  memset(&cfgExpert->m_aiPad,0, sizeof(cfgExpert->m_aiPad));                                    ///< number of padded pixels for width and height
  cfgExpert->m_enablePictureHeaderInSliceHeader        = true;
  cfgExpert->m_AccessUnitDelimiter                     = -1;                                    ///< add Access Unit Delimiter NAL units, default: auto (only enable if needed by dependent options)

  cfgExpert->m_printMSEBasedSequencePSNR               = false;
  cfgExpert->m_printHexPsnr                            = false;
  cfgExpert->m_printFrameMSE                           = false;
  cfgExpert->m_printSequenceMSE                        = false;
  cfgExpert->m_cabacZeroWordPaddingEnabled             = true;

  cfgExpert->m_subProfile                              = 0;
  cfgExpert->m_bitDepthConstraintValue                 = 10;
  cfgExpert->m_intraOnlyConstraintFlag                 = false;

  cfgExpert->m_InputQueueSize                          = 0;                                     ///< Size of frame input queue
  cfgExpert->m_rewriteParamSets                        = false;                                 ///< Flag to enable rewriting of parameter sets at random access points
  cfgExpert->m_idrRefParamList                         = false;                                 ///< indicates if reference picture list syntax elements are present in slice headers of IDR pictures
  for( i = 0; i < VVENC_MAX_GOP; i++ )
  {
    vvenc_RPLEntry_default( &cfgExpert->m_RPLList0[i]);                                         ///< the RPL entries from the config file
    vvenc_RPLEntry_default( &cfgExpert->m_RPLList1[i]);                                         ///< the RPL entries from the config file
    vvenc_GOPEntry_default( &cfgExpert->m_GOPList[i]);                                          ///< the coding structure entries from the config file
  }
  memset(&cfgExpert->m_maxDecPicBuffering,0, sizeof(cfgExpert->m_maxDecPicBuffering));                ///< total number of pictures in the decoded picture buffer
  memset(&cfgExpert->m_maxNumReorderPics,0, sizeof(cfgExpert->m_maxNumReorderPics));                  ///< total number of reorder pictures
  cfgExpert->m_maxTempLayer                            = 0;                                     ///< Max temporal layer
  cfgExpert->m_numRPLList0                             = 0;
  cfgExpert->m_numRPLList1                             = 0;

  cfgExpert->m_useSameChromaQPTables                   = true;

  vvenc_ChromaQpMappingTableParams_default( &cfgExpert->m_chromaQpMappingTableParams );
  cfgExpert->m_intraQPOffset                           = 0;                                     ///< QP offset for intra slice (integer)
  cfgExpert->m_lambdaFromQPEnable                      = false;                                 ///< enable flag for QP:lambda fix

  memset(&cfgExpert->m_adLambdaModifier,1.0, sizeof(cfgExpert->m_adLambdaModifier));                ///< Lambda modifier array for each temporal layer
  memset(&cfgExpert->m_adIntraLambdaModifier,0.0, sizeof(cfgExpert->m_adIntraLambdaModifier));       ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.

  cfgExpert->m_dIntraQpFactor                          = -1.0;                                  ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))

  memset(&cfgExpert->m_qpInValsCb    ,0, sizeof(cfgExpert->m_qpInValsCb));                      ///< qp input values used to derive the chroma QP mapping table
  memset(&cfgExpert->m_qpInValsCr    ,0, sizeof(cfgExpert->m_qpInValsCr));                      ///< qp input values used to derive the chroma QP mapping table
  memset(&cfgExpert->m_qpInValsCbCr  ,0, sizeof(cfgExpert->m_qpInValsCbCr));                      ///< qp input values used to derive the chroma QP mapping table
  memset(&cfgExpert->m_qpOutValsCb   ,0, sizeof(cfgExpert->m_qpOutValsCb));                      ///< qp output values used to derive the chroma QP mapping table
  memset(&cfgExpert->m_qpOutValsCr   ,0, sizeof(cfgExpert->m_qpOutValsCr));                      ///< qp output values used to derive the chroma QP mapping table
  memset(&cfgExpert->m_qpOutValsCbCr ,0, sizeof(cfgExpert->m_qpOutValsCbCr));                      ///< qp output values used to derive the chroma QP mapping table

  cfgExpert->m_qpInValsCb[0] = 17;  cfgExpert->m_qpInValsCb[1] = 22;  cfgExpert->m_qpInValsCb[2] = 34;  cfgExpert->m_qpInValsCb[3] = 42;
  cfgExpert->m_qpOutValsCb[0] = 17; cfgExpert->m_qpOutValsCb[1] = 23; cfgExpert->m_qpOutValsCb[2] = 35; cfgExpert->m_qpOutValsCb[3] = 39;

  cfgExpert->m_cuQpDeltaSubdiv                         = -1;                                    ///< Maximum subdiv for CU luma Qp adjustment (0:default)
  cfgExpert->m_cuChromaQpOffsetSubdiv                  = -1;                                    ///< If negative, then do not apply chroma qp offsets.
  cfgExpert->m_chromaCbQpOffset                        = 0;                                     ///< Chroma Cb QP Offset (0:default)
  cfgExpert->m_chromaCrQpOffset                        = 0;                                     ///< Chroma Cr QP Offset (0:default)
  cfgExpert->m_chromaCbQpOffsetDualTree                = 0;                                     ///< Chroma Cb QP Offset for dual tree (overwrite m_chromaCbQpOffset for dual tree)
  cfgExpert->m_chromaCrQpOffsetDualTree                = 0;                                     ///< Chroma Cr QP Offset for dual tree (overwrite m_chromaCrQpOffset for dual tree)
  cfgExpert->m_chromaCbCrQpOffset                      = -1;                                    ///< QP Offset for joint Cb-Cr mode
  cfgExpert->m_chromaCbCrQpOffsetDualTree              = 0;                                     ///< QP Offset for joint Cb-Cr mode (overwrite m_chromaCbCrQpOffset for dual tree)
  cfgExpert->m_sliceChromaQpOffsetPeriodicity          = -1;                                    ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.

  memset(&cfgExpert->m_sliceChromaQpOffsetIntraOrPeriodic,0, sizeof(cfgExpert->m_sliceChromaQpOffsetIntraOrPeriodic));            ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.

  cfgExpert->m_usePerceptQPATempFiltISlice             = -1;                                    ///< Flag indicating if temporal high-pass filtering in visual activity calculation in QPA should (true) or shouldn't (false) be applied for I-slices

  cfgExpert->m_lumaLevelToDeltaQPEnabled               = false;
  vvenc_WCGChromaQPControl_default(&cfgExpert->m_wcgChromaQpControl );

  cfgExpert->m_internChromaFormat                      = VVENC_NUM_CHROMA_FORMAT;
  cfgExpert->m_useIdentityTableForNon420Chroma         = true;
  cfgExpert->m_outputBitDepth[0] = cfgExpert->m_outputBitDepth[1] = 0;                              ///< bit-depth of output file
  cfgExpert->m_MSBExtendedBitDepth[0] = cfgExpert->m_MSBExtendedBitDepth[1] = 0;                    ///< bit-depth of input samples after MSB extension
  cfgExpert->m_costMode                                = VVENC_COST_STANDARD_LOSSY;                   ///< Cost mode to use

  cfgExpert->m_decodedPictureHashSEIType               = VVENC_HASHTYPE_NONE;                         ///< Checksum mode for decoded picture hash SEI message
  cfgExpert->m_bufferingPeriodSEIEnabled               = false;
  cfgExpert->m_pictureTimingSEIEnabled                 = false;
  cfgExpert->m_decodingUnitInfoSEIEnabled              = false;

  cfgExpert->m_entropyCodingSyncEnabled                = false;
  cfgExpert->m_entryPointsPresent                      = true;

  cfgExpert->m_CTUSize                                 = 128;
  cfgExpert->m_MinQT[0] = cfgExpert->m_MinQT[1] = 8;                                            ///< 0: I slice luma; 1: P/B slice; 2: I slice chroma
  cfgExpert->m_MinQT[2] = 4;
  cfgExpert->m_maxMTTDepth                             = 3;
  cfgExpert->m_maxMTTDepthI                            = 3;
  cfgExpert->m_maxMTTDepthIChroma                      = 3;

  cfgExpert->m_maxBT[0]=32;  cfgExpert->m_maxBT[1]=128;  cfgExpert->m_maxBT[2]=64;
  cfgExpert->m_maxTT[0]=32;  cfgExpert->m_maxTT[1]=64;  cfgExpert->m_maxTT[2]=32;
  cfgExpert->m_dualITree                               = true;
  cfgExpert->m_MaxCodingDepth                          = 0;                                     ///< max. total CU depth - includes depth of transform-block structure
  cfgExpert->m_log2DiffMaxMinCodingBlockSize           = 0;                                     ///< difference between largest and smallest CU depth
  cfgExpert->m_log2MaxTbSize                           = 6;

  cfgExpert->m_bUseASR                                 = false;                                 ///< flag for using adaptive motion search range
  cfgExpert->m_bUseHADME                               = true;                                  ///< flag for using HAD in sub-pel ME
  cfgExpert->m_RDOQ                                    = 1;                                     ///< flag for using RD optimized quantization
  cfgExpert->m_useRDOQTS                               = true;                                  ///< flag for using RD optimized quantization for transform skip
  cfgExpert->m_useSelectiveRDOQ                        = false;                                 ///< flag for using selective RDOQ

  cfgExpert->m_JointCbCrMode                           = false;
  cfgExpert->m_cabacInitPresent                        = -1;
  cfgExpert->m_useFastLCTU                             = false;
  cfgExpert->m_usePbIntraFast                          = false;
  cfgExpert->m_useFastMrg                              = 0;
  cfgExpert->m_useAMaxBT                               = -1;
  cfgExpert->m_fastQtBtEnc                             = true;
  cfgExpert->m_contentBasedFastQtbt                    = false;
  cfgExpert->m_fastInterSearchMode                     = VVENC_FASTINTERSEARCH_AUTO;              ///< Parameter that controls fast encoder settings
  cfgExpert->m_bUseEarlyCU                             = false;                                 ///< flag for using Early CU setting
  cfgExpert->m_useFastDecisionForMerge                 = true;                                  ///< flag for using Fast Decision Merge RD-Cost
  cfgExpert->m_useEarlySkipDetection                   = false;                                 ///< flag for using Early SKIP Detection

  cfgExpert->m_bDisableIntraCUsInInterSlices           = false;                                 ///< Flag for disabling intra predicted CUs in inter slices.
  cfgExpert->m_bUseConstrainedIntraPred                = false;                                 ///< flag for using constrained intra prediction
  cfgExpert->m_bFastUDIUseMPMEnabled                   = true;
  cfgExpert->m_bFastMEForGenBLowDelayEnabled           = true;

  cfgExpert->m_MTSImplicit                             = false;
  cfgExpert->m_TMVPModeId                              = 1;
  cfgExpert->m_DepQuantEnabled                         = true;
  cfgExpert->m_SignDataHidingEnabled                   = false;

  cfgExpert->m_MIP                                     = false;
  cfgExpert->m_useFastMIP                              = 0;

  cfgExpert->m_maxNumMergeCand                         = 5;                                     ///< Max number of merge candidates
  cfgExpert->m_maxNumAffineMergeCand                   = 5;                                     ///< Max number of affine merge candidates
//  cfgExpert->m_maxNumIBCMergeCand                    = 6;                                     ///< Max number of IBC merge candidates
  cfgExpert->m_Geo                                     = 0;
  cfgExpert->m_maxNumGeoCand                           = 5;
  cfgExpert->m_FastIntraTools                          = 0;

  cfgExpert->m_RCInitialQP                             = 0;
  cfgExpert->m_RCForceIntraQP                          = false;

  cfgExpert->m_motionEstimationSearchMethod            = VVENC_MESEARCH_DIAMOND;
  cfgExpert->m_bRestrictMESampling                     = false;                                 ///< Restrict sampling for the Selective ME
  cfgExpert->m_SearchRange                             = 96;                                    ///< ME search range
  cfgExpert->m_bipredSearchRange                       = 4;                                     ///< ME search range for bipred refinement
  cfgExpert->m_minSearchWindow                         = 8;                                     ///< ME minimum search window size for the Adaptive Window ME
  cfgExpert->m_bClipForBiPredMeEnabled                 = false;                                 ///< Enables clipping for Bi-Pred ME.
  cfgExpert->m_bFastMEAssumingSmootherMVEnabled        = true;                                  ///< Enables fast ME assuming a smoother MV.
  cfgExpert->m_fastSubPel                              = 0;
  cfgExpert->m_SMVD                                    = 0;
  cfgExpert->m_AMVRspeed                               = 0;
  cfgExpert->m_LMChroma                                = false;
  cfgExpert->m_horCollocatedChromaFlag                 = true;
  cfgExpert->m_verCollocatedChromaFlag                 = false;
  cfgExpert->m_MRL                                     = true;
  cfgExpert->m_BDOF                                    = false;
  cfgExpert->m_DMVR                                    = false;
  cfgExpert->m_EDO                                     = 0;
  cfgExpert->m_lumaReshapeEnable                       = false;
  cfgExpert->m_reshapeSignalType                       = 0;
  cfgExpert->m_updateCtrl                              = 0;
  cfgExpert->m_adpOption                               = 0;
  cfgExpert->m_initialCW                               = 0;
  cfgExpert->m_LMCSOffset                              = 0;
  vvenc_ReshapeCW_default( &cfgExpert->m_reshapeCW );
  cfgExpert->m_Affine                                  = 0;
  cfgExpert->m_PROF                                    = false;
  cfgExpert->m_AffineType                              = true;
  cfgExpert->m_MMVD                                    = 0;
  cfgExpert->m_MmvdDisNum                              = 6;
  cfgExpert->m_allowDisFracMMVD                        = false;
  cfgExpert->m_CIIP                                    = 0;
  cfgExpert->m_SbTMVP                                  = false;
  cfgExpert->m_SBT                                     = 0;                                     ///< Sub-Block Transform for inter blocks
  cfgExpert->m_LFNST                                   = 0;
  cfgExpert->m_MTS                                     = 0;
  cfgExpert->m_MTSIntraMaxCand                         = 3;
  cfgExpert->m_ISP                                     = 0;
  cfgExpert->m_TS                                      = 0;
  cfgExpert->m_TSsize                                  = 3;
  cfgExpert->m_useChromaTS                             = 0;
  cfgExpert->m_useBDPCM                                = 0;

  cfgExpert->m_rprEnabledFlag                          = 1;
  cfgExpert->m_resChangeInClvsEnabled                  = false;
  cfgExpert->m_craAPSreset                             = false;
  cfgExpert->m_rprRASLtoolSwitch                       = false;

  cfgExpert->m_bLoopFilterDisable                      = false;                                 ///< flag for using deblocking filter
  cfgExpert->m_loopFilterOffsetInPPS                   = true;                                  ///< offset for deblocking filter in 0 = slice header, 1 = PPS

  memset(&cfgExpert->m_loopFilterBetaOffsetDiv2,0, sizeof(cfgExpert->m_loopFilterBetaOffsetDiv2)); ///< beta offset for deblocking filter
  memset(&cfgExpert->m_loopFilterTcOffsetDiv2,0, sizeof(cfgExpert->m_loopFilterTcOffsetDiv2));     ///< tc offset for deblocking filter

  cfgExpert->m_deblockingFilterMetric                  = 0;

  cfgExpert->m_bLFCrossTileBoundaryFlag                = true;
  cfgExpert->m_bLFCrossSliceBoundaryFlag               = true;                                  ///< 1: filter across slice boundaries 0: do not filter across slice boundaries
  cfgExpert->m_loopFilterAcrossSlicesEnabled           = false;

  cfgExpert->m_bUseSAO                                 = true;
  cfgExpert->m_saoEncodingRate                         = -1.0;                                  ///< When >0 SAO early picture termination is enabled for luma and chroma
  cfgExpert->m_saoEncodingRateChroma                   = -1.0;                                  ///< The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  cfgExpert->m_log2SaoOffsetScale[0]=cfgExpert->m_log2SaoOffsetScale[1] = 0;                              ///< n umber of bits for the upward bit shift operation on the decoded SAO offsets
  cfgExpert->m_saoOffsetBitShift[0]=cfgExpert->m_saoOffsetBitShift[1] = 0;

  cfgExpert->m_decodingParameterSetEnabled             = false;                                 ///< enable decoding parameter set
  cfgExpert->m_vuiParametersPresent                    = -1;                                    ///< enable generation of VUI parameters; -1 auto enable, 0: off 1: enable
  cfgExpert->m_hrdParametersPresent                    = -1;                                    ///< enable generation or HRD parameters; -1 auto enable, 0: off 1: enable
  cfgExpert->m_aspectRatioInfoPresent                  = false;                                 ///< Signals whether aspect_ratio_idc is present
  cfgExpert->m_aspectRatioIdc                          = 0;                                     ///< aspect_ratio_idc
  cfgExpert->m_sarWidth                                = 0;                                     ///< horizontal size of the sample aspect ratio
  cfgExpert->m_sarHeight                               = 0;                                     ///< vertical size of the sample aspect ratio
  cfgExpert->m_colourDescriptionPresent                = false;                                 ///< Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  cfgExpert->m_colourPrimaries                         = 2;                                     ///< Indicates chromaticity coordinates of the source primaries
  cfgExpert->m_transferCharacteristics                 = 2;                                     ///< Indicates the opto-electronic transfer characteristics of the source
  cfgExpert->m_matrixCoefficients                      = 2;                                     ///< Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  cfgExpert->m_chromaLocInfoPresent                    = false;                                 ///< Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  cfgExpert->m_chromaSampleLocTypeTopField             = 0;                                     ///< Specifies the location of chroma samples for top field
  cfgExpert->m_chromaSampleLocTypeBottomField          = 0;                                     ///< Specifies the location of chroma samples for bottom field
  cfgExpert->m_chromaSampleLocType                     = 0;                                     ///< Specifies the location of chroma samples for progressive content
  cfgExpert->m_overscanInfoPresent                     = false;                                 ///< Signals whether overscan_appropriate_flag is present
  cfgExpert->m_overscanAppropriateFlag                 = false;                                 ///< Indicates whether conformant decoded pictures are suitable for display using overscan
  cfgExpert->m_videoSignalTypePresent                  = false;                                 ///< Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present
  cfgExpert->m_videoFullRangeFlag                      = false;                                 ///< Indicates the black level and range of luma and chroma signals

  memset(&cfgExpert->m_masteringDisplay,0, sizeof(cfgExpert->m_masteringDisplay));              ///< mastering display colour volume, vector of size 10, format: G(x,y)B(x,y)R(x,y)WP(x,y)L(max,min), 0 <= GBR,WP <= 50000, 0 <= L <= uint (SEI)
                                                                                                ///< GBR xy coordinates in increments of 1/50000 (in the ranges 0 to 50000) (e.g. 0.333 = 16667)
                                                                                                ///< min/max luminance value in units of 1/10000 candela per square metre
  memset(&cfgExpert->m_contentLightLevel,0, sizeof(cfgExpert->m_contentLightLevel));            ///< upper bound on the max light level and max avg light level among all individual samples in a 4:4:4 representation. in units of candelas per square metre (SEI)
  cfgExpert->m_preferredTransferCharacteristics        = -1;                                    ///< Alternative transfer characteristics SEI which will override the corresponding entry in the VUI, if < 0 SEI is not written")

  cfgExpert->m_summaryOutFilename                      = NULL;                                    ///< filename to use for producing summary output file.
  cfgExpert->m_summaryPicFilenameBase                  = NULL;                                    ///< Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  cfgExpert->m_summaryVerboseness                      = 0;                                     ///< Specifies the level of the verboseness of the text output.

  cfgExpert->m_decodeBitstreams[0]=cfgExpert->m_decodeBitstreams[1] = NULL;                            ///< filename for decode bitstreams.
  cfgExpert->m_switchPOC                               = -1;                                    ///< dbg poc.
  cfgExpert->m_switchDQP                               = 0;                                     ///< switch DQP.
  cfgExpert->m_fastForwardToPOC                        = -1;                                    ///< get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC
  cfgExpert->m_stopAfterFFtoPOC                        = false;
  cfgExpert->m_bs2ModPOCAndType                        = false;
  cfgExpert->m_forceDecodeBitstream1                   = false;

  cfgExpert->m_alf                                     = false;                                 ///> Adaptive Loop Filter
  cfgExpert->m_useNonLinearAlfLuma                     = true;
  cfgExpert->m_useNonLinearAlfChroma                   = true;
  cfgExpert->m_maxNumAlfAlternativesChroma             = VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA;
  cfgExpert->m_ccalf                                   = false;
  cfgExpert->m_ccalfQpThreshold                        = 37;
  cfgExpert->m_alfTempPred                             = -1;                                    ///> Indicates using of temporal filter data prediction through APS

  cfgExpert->m_MCTF                                    = 0;
  cfgExpert->m_MCTFFutureReference                     = true;
  cfgExpert->m_MCTFNumLeadFrames                       = 0;
  cfgExpert->m_MCTFNumTrailFrames                      = 0;
  memset(&cfgExpert->m_MCTFFrames,0, sizeof(cfgExpert->m_MCTFFrames));
  memset(&cfgExpert->m_MCTFStrengths,0, sizeof(cfgExpert->m_MCTFStrengths));

  cfgExpert->m_dqThresholdVal                          = 8;
  cfgExpert->m_qtbttSpeedUp                            = 0;

  cfgExpert->m_fastLocalDualTreeMode                   = 0;

  cfgExpert->m_maxParallelFrames                       = -1;
  cfgExpert->m_ensureWppBitEqual                       = -1;            ///< Flag indicating bit equalitiy for single thread runs respecting multithread restrictions

  cfgExpert->m_picPartitionFlag                        = false;
}



#ifdef __cplusplus
};
#endif

VVENC_NAMESPACE_END

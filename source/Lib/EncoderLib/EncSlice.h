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
/** \file     EncSlice.h
    \brief    slice encoder class (header)
*/

#pragma once

#include "EncCu.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "InterSearch.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class EncCfg;
struct SAOStatData;
class EncSampleAdaptiveOffset;
class EncAdaptiveLoopFilter;
class EncPicture;
class NoMallocThreadPool;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct LineEncRsrc;
struct CtuTaskRsrc;
struct CtuEncParam;

enum ProcessCtuState {
  CTU_ENCODE     = 0,
  RESHAPE_LF_VER,
  LF_HOR,
  SAO_FILTER,
  ALF_GET_STATISTICS,
  ALF_DERIVE_FILTER,
  ALF_RECONSTRUCT,
  PROCESS_DONE
};

/// slice encoder class
class EncSlice
{
private:
  // encoder configuration
  const EncCfg*                m_pcEncCfg;                           ///< encoder configuration class

  std::vector<CtuTaskRsrc*>    m_CtuTaskRsrc;
  std::vector<LineEncRsrc*>    m_LineEncRsrc;
  NoMallocThreadPool*          m_threadPool;
  std::vector<ProcessCtuState> m_processStates;

  LoopFilter*                  m_pLoopFilter;
  EncAdaptiveLoopFilter*       m_pALF;
  RateCtrl*                    m_pcRateCtrl;
  BinEncoder                   m_BinEncoder;
  CABACWriter                  m_CABACWriter;

  Ctx                          m_entropyCodingSyncContextState;      ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row used for writing
  std::vector<Ctx>             m_syncPicCtx;                         ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row used for estimation
  SliceType                    m_encCABACTableIdx;
  int                          m_appliedSwitchDQQ;

  double                       m_saoDisabledRate[ MAX_NUM_COMP ][ MAX_TLAYER ];
  bool                         m_saoEnabled[ MAX_NUM_COMP ];
  bool                         m_saoAllDisabled;
  std::vector<SAOBlkParam>     m_saoReconParams;
  std::vector<SAOStatData**>   m_saoStatData;

public:
  EncSlice();
  virtual ~EncSlice();

  void    init                ( const EncCfg& encCfg,
                                const SPS& sps,
                                const PPS& pps,
                                std::vector<int>* const globalCtuQpVector,
                                LoopFilter& loopFilter,
                                EncAdaptiveLoopFilter& alf,
                                RateCtrl& rateCtrl,
                                NoMallocThreadPool* threadPool );

  void    initPic             ( Picture* pic, int gopId );

  // compress and encode slice
  void    compressSlice       ( Picture* pic );      ///< analysis stage of slice                     s
  void    encodeSliceData     ( Picture* pic );
  void    saoDisabledRate     ( CodingStructure& cs, SAOBlkParam* reconParams );

  void    resetQP              ( Picture* pic, int sliceQP, double lambda );

private:
  void    xInitSliceLambdaQP   ( Slice* slice, int gopId );
  double  xCalculateLambda     ( const Slice* slice, const int GOPid, const int depth, const double refQP, const double dQP, int& iQP );
  void    xProcessCtus         ( Picture* pic, const unsigned startCtuTsAddr, const unsigned boundingCtuTsAddr );

  template<bool checkReadyState=false>
  static bool xProcessCtuTask  ( int taskIdx, CtuEncParam* ctuEncParam );

  int     xGetQPForPicture     ( const Slice* slice, unsigned gopId );
};

} // namespace vvenc

//! \}


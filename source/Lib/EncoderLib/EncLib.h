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
/** \file     EncLib.h
    \brief    encoder class (header)
*/

#pragma once

#include "EncGOP.h"
#include "EncHRD.h"
#include "CommonLib/MCTF.h"
#include <mutex>
#include "../../../include/vvenc/EncCfg.h"
#include "../../../include/vvenc/Nal.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class YUVWriterIf;
class NoMallocThreadPool;

/// encoder class
class EncLib
{
private:
  int                       m_numPicsRcvd;
  int                       m_numPicsInQueue;
  int                       m_numPicsCoded;
  int                       m_pocEncode;
  int                       m_pocRecOut;

  const EncCfg              m_cEncCfg;
  EncGOP                    m_cGOPEncoder;
  EncHRD                    m_cEncHRD;
  MCTF                      m_MCTF;
  PicList                   m_cListPic;
  YUVWriterIf*              m_yuvWriterIf;
  NoMallocThreadPool*       m_threadPool;
  RateCtrl                  m_cRateCtrl;                          ///< Rate control class

  VPS                       m_cVPS;
  DCI                       m_cDCI;
  ParameterSetMap<SPS>      m_spsMap;
  ParameterSetMap<PPS>      m_ppsMap;

  XUCache                   m_shrdUnitCache;
  std::mutex                m_unitCacheMutex;

  std::vector<int>          m_pocToGopId;
  std::vector<int>          m_nextPocOffset;

  int                       m_GOPSizeLog2;
  int                       m_TicksPerFrameMul4;

public:
  EncLib();
  virtual ~EncLib();

  void     init                ( const EncCfg& encCfg, YUVWriterIf* yuvWriterIf );
  void     destroy             ();
  void     encodePicture       ( bool flush, const YUVBuffer& yuvInBuf, AccessUnit& au, bool& isQueueEmpty );
  void     printSummary        ();

private:
  int      xGetGopIdFromPoc    ( int poc ) const { return m_pocToGopId[ poc % m_cEncCfg.m_GOPSize ]; }
  int      xGetNextPocICO      ( int poc, bool flush, int max ) const;
  void     xCreateCodingOrder  ( int start, int max, int numInQueue, bool flush, std::vector<Picture*>& encList );
  void     xInitPicture        ( Picture& pic, int picNum, const PPS& pps, const SPS& sps, const VPS& vps, const DCI& dci );
  void     xDeletePicBuffer    ();
  Picture* xGetNewPicBuffer    ( const PPS& pps, const SPS& sps );            ///< get picture buffer which will be processed. If ppsId<0, then the ppsMap will be queried for the first match.
  Picture* xGetPictureBuffer   ( int poc );

  void     xInitVPS            ( VPS &vps )                                  const; ///< initialize VPS from encoder options
  void     xInitDCI            ( DCI &dci, const SPS &sps, const int dciId ) const; ///< initialize DCI from encoder options
  void     xInitConstraintInfo ( ConstraintInfo &ci )                        const;  ///< initialize SPS from encoder options
  void     xInitSPS            ( SPS &sps )                                  const; ///< initialize SPS from encoder options
  void     xInitPPS            ( PPS &pps, const SPS &sps )                  const;  ///< initialize PPS from encoder options
  void     xInitPPSforTiles    ( PPS &pps ) const;
  void     xInitRPL            ( SPS &sps ) const;
  void     xInitHrdParameters  ( SPS &sps );
  void     xOutputRecYuv       ();
  void     xDetectScreenC      ( Picture& pic , PelUnitBuf yuvOrgBuf, int useTS);
};

} // namespace vvenc

//! \}


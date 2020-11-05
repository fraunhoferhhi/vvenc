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
/** \file     EncCfg.h
    \brief    encoder configuration class (header)
*/

#pragma once

#include <cstring>
#include <vector>
#include <string>
#include <sstream>
#include "vvenc/vvencDecl.h"
#include "vvenc/Basics.h"
#include "vvenc/EncCfgExpert.h"

//! \ingroup Interface
//! \{

namespace vvenc {

class VVENC_DECL EncCfg : public EncCfgExpert
{
public:
  bool                m_confirmFailed;                                  ///< state variable 
  int                 m_verbosity;                                      ///< encoder verbosity
  int                 m_framesToBeEncoded;                              ///< number of encoded frames
  int                 m_FrameRate;                                      ///< source frame-rates (Hz)
  int                 m_FrameSkip;                                      ///< number of skipped frames from the beginning
  int                 m_SourceWidth;                                    ///< source width in pixel
  int                 m_SourceHeight;                                   ///< source height in pixel (when interlaced = field height)
  int                 m_TicksPerSecond;                                 ///< ticks per second e.g. 90000 for dts generation         (no default || 1..27000000)
  bool                m_AccessUnitDelimiter;                            ///< add Access Unit Delimiter NAL units

  Profile::Name       m_profile;
  Level::Tier         m_levelTier;
  Level::Name         m_level;

  int                 m_IntraPeriod;                                    ///< period of I-slice (random access period)
  int                 m_DecodingRefreshType;                            ///< random access type
  int                 m_GOPSize;                                        ///< GOP size of hierarchical structure

  int                 m_QP;                                             ///< QP value of key-picture (integer)
  unsigned            m_usePerceptQPA;                                  ///< Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA). (0 = off, 1 = on for SDR, 2 = on for HDR)
  bool                m_usePerceptQPATempFiltISlice;                    ///< Flag indicating if temporal high-pass filtering in visual activity calculation in QPA should (true) or shouldn't (false) be applied for I-slices

  int                 m_inputBitDepth   [ MAX_NUM_CH ];                 ///< bit-depth of input file
  int                 m_internalBitDepth[ MAX_NUM_CH ];                 ///< bit-depth codec operates at (input/output files will be converted)

  int                 m_numWppThreads;                                  ///< number of wpp threads
  int                 m_ensureWppBitEqual;                              ///< Flag indicating bit equalitiy for single thread runs respecting multithread restrictions  
public:

  EncCfg()
    : m_confirmFailed                               ( false )
    , m_verbosity                                   ( VERBOSE )
    , m_framesToBeEncoded                           ( 0 )
    , m_FrameRate                                   ( 0 )
    , m_FrameSkip                                   ( 0 )
    , m_SourceWidth                                 ( 0 )
    , m_SourceHeight                                ( 0 )
    , m_TicksPerSecond                              ( 90000 )
    , m_AccessUnitDelimiter                         ( false )

    , m_profile                                     ( Profile::MAIN_10 )
    , m_levelTier                                   ( Level::MAIN )
    , m_level                                       ( Level::LEVEL4_1 )

    , m_IntraPeriod                                 ( 32 )
    , m_DecodingRefreshType                         ( 1 )
    , m_GOPSize                                     ( 16 )

    , m_QP                                          ( 32 )
    , m_usePerceptQPA                               ( 0 )
    , m_usePerceptQPATempFiltISlice                 ( false )

    , m_inputBitDepth                               { 8, 0 }
    , m_internalBitDepth                            { 10, 0 }

    , m_numWppThreads                               ( 0 )
    , m_ensureWppBitEqual                           ( 0 )
  {
  }
  virtual ~EncCfg()
  {
  }

  bool confirmParameter( bool bflag, const char* message );
  bool initCfgParameter();
  void setCfgParameter( const EncCfg& encCfg );
  int initPreset( int iQuality );
};




} // namespace vvenc

//! \}


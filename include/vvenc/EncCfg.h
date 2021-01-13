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
/** \file     EncCfg.h
    \brief    encoder configuration class (header)
*/

#pragma once

#include <cstring>
#include <vector>
#include <string>
#include <sstream>
#include "vvenc/vvencDecl.h"
#include "vvenc/EncCfgExpert.h"

//! \ingroup Interface
//! \{

namespace vvenc {

class VVENC_DECL EncCfg : public EncCfgExpert
{
public:
  bool                m_confirmFailed                  = false;         ///< state variable
  int                 m_verbosity                      = VERBOSE;       ///< encoder verbosity
  int                 m_framesToBeEncoded              = 0;             ///< number of encoded frames
  int                 m_FrameRate                      = 0;             ///< source frame-rates (Hz)
  int                 m_FrameSkip                      = 0;             ///< number of skipped frames from the beginning
  int                 m_SourceWidth                    = 0;             ///< source width in pixel
  int                 m_SourceHeight                   = 0;             ///< source height in pixel (when interlaced = field height)
  int                 m_TicksPerSecond                 = 90000;         ///< ticks per second e.g. 90000 for dts generation         (no default || 1..27000000)
  bool                m_AccessUnitDelimiter            = false;         ///< add Access Unit Delimiter NAL units

  Profile             m_profile                        = Profile::MAIN_10;
  Tier                m_levelTier                      = Tier::TIER_MAIN ;
  Level               m_level                          = Level::LEVEL4_1;

  int                 m_IntraPeriod                    = 32;            ///< period of I-slice (random access period)
  int                 m_IntraPeriodSec                 = 1;             ///< period of I-slice in seconds (random access period)
  int                 m_DecodingRefreshType            = 1;             ///< random access type
  int                 m_GOPSize                        = 32;            ///< GOP size of hierarchical structure

  int                 m_QP                             = 32;            ///< QP value of key-picture (integer)
  unsigned            m_usePerceptQPA                  = 0;             ///< Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA). (0 = off, 1 = on for SDR, 2 = on for HDR)
  bool                m_usePerceptQPATempFiltISlice    = false;         ///< Flag indicating if temporal high-pass filtering in visual activity calculation in QPA should (true) or shouldn't (false) be applied for I-slices

  int                 m_inputBitDepth   [ MAX_NUM_CH ] = { 8, 0};       ///< bit-depth of input file
  int                 m_internalBitDepth[ MAX_NUM_CH ] = { 10, 0};      ///< bit-depth codec operates at (input/output files will be converted)

  int                 m_numWppThreads                  = 0;             ///< number of wpp threads
  int                 m_ensureWppBitEqual              = 0;             ///< Flag indicating bit equalitiy for single thread runs respecting multithread restrictions
public:

  EncCfg()
  {
  }
  virtual ~EncCfg()
  {
  }

  bool checkExperimental( bool bflag, const char* message );
  bool confirmParameter ( bool bflag, const char* message );
  bool initCfgParameter();
  void setCfgParameter( const EncCfg& encCfg );
  int  initPreset( PresetMode preset );
  virtual void printCfg() const;
};


} // namespace vvenc

//! \}


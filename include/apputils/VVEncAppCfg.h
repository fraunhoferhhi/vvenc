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
/** \file     VVEncAppCfg.h
    \brief    Handle encoder configuration parameters (header)
*/

#pragma once
#include "apputils/apputilsDecl.h"
#include "vvenc/vvencCfg.h"
#include <string>
#include <vector>
#include <tuple>
#include <functional>

namespace apputils {

//! \ingroup apputils
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// application params and parser class for app+lib options
class APPUTILS_DECL VVEncAppCfg
{
public:
  typedef std::function<void( vvenc_config*, vvencPresetMode ) > presetChangeCallback;

public:
  std::string  m_inputFileName;                                ///< source file name
  std::string  m_bitstreamFileName;                            ///< output bitstream file
  std::string  m_reconFileName;                                ///< output reconstruction file
  std::string  m_RCStatsFileName;                              ///< rate control statistics file
  vvencChromaFormat m_inputFileChromaFormat    = VVENC_CHROMA_420;
  int          m_FrameSkip                    = 0;             ///< number of skipped frames from the beginning
  bool         m_bClipInputVideoToRec709Range  = false;
  bool         m_bClipOutputVideoToRec709Range = false;
  bool         m_packedYUVInput                = false;        ///< If true, packed 10-bit YUV ( 4 samples packed into 5-bytes consecutively )
  bool         m_packedYUVOutput               = false;        ///< If true, output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data
  bool         m_decode                        = false;
  bool         m_showVersion                   = false;
  bool         m_showHelp                      = false;

  std::string  m_additionalSettings;                           ///< set additional settings (always parsed and set after other params are set)
                                                               ///< options must be defined as tuple key=value, entries must be separated by space' ' or colon ':'
                                                               ///< values that are not arbitrary must be quoted "\"values\""
                                                               ///< e.b. "bitrate=1000000 passes=1 QpInValCb=\"17 22 34 42\"" 
private:
  const bool   m_easyMode                      = false;        ///< internal state flag, if expert or easy mode

  presetChangeCallback  m_changePresetCallback = nullptr;
public:

  VVEncAppCfg( bool easyMode = false )
  : m_easyMode (easyMode )
  {

  }
  virtual ~VVEncAppCfg(){}

  void setPresetChangeCallback( presetChangeCallback callback )
  {
    m_changePresetCallback = callback;
  }

  presetChangeCallback getPresetChangeCallback( )
  {
    return m_changePresetCallback ;
  }

public:
  int parse( int argc, char* argv[], vvenc_config* c, std::stringstream& rcOstr );

  bool checkCfg( vvenc_config* c, std::stringstream& rcOstr );
  virtual std::string getAppConfigAsString( vvencMsgLevel eMsgLevel ) const;

  std::vector <std::tuple<std::string, std::string>> getAdditionalSettingList();

private:
  bool xCheckCfg( vvenc_config* c, std::stringstream& rcOstr );

  std::vector<std::string> tokenize(std::string str, char delimiter );
};

} // namespace

//! \}


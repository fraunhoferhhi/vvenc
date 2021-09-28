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

namespace apputils {

//! \ingroup apputils
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder configuration class
class APPUTILS_DECL VVEncAppCfg : public vvenc_config
{
public:
  //vvenc_config conf;

  std::string  m_inputFileName;                                ///< source file name
  std::string  m_bitstreamFileName;                            ///< output bitstream file
  std::string  m_reconFileName;                                ///< output reconstruction file
  vvencChromaFormat m_inputFileChromaFormat    = VVENC_CHROMA_420;
  bool         m_bClipInputVideoToRec709Range  = false;
  bool         m_bClipOutputVideoToRec709Range = false;
  bool         m_packedYUVInput                = false;        ///< If true, hhi-packed 10-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data
  bool         m_packedYUVOutput               = false;        ///< If true, output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data
  bool         m_decode                        = false;
  bool         m_showVersion                   = false;

public:

  VVEncAppCfg()
  {
    vvenc_config_default( this );
  }

  virtual ~VVEncAppCfg();

public:
  bool parseCfg( int argc, char* argv[] );                     ///< parse configuration fill member variables (simple app)
  bool parseCfgFF( int argc, char* argv[] );                   ///< parse configuration fill member variables for FullFeature set (expert app)

  virtual std::string getConfigAsString( vvencMsgLevel eMsgLevel ) const;
};

} // namespace

//! \}


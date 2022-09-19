/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     YuvFileIO.h
    \brief    yuv file I/O class (header)
*/

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <algorithm>

#include "vvenc/version.h"
#include "vvenc/vvenc.h"

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
#include "../../../thirdparty/nlohmann_json/single_include/nlohmann/json.hpp"
#endif


//! \ingroup Interface
//! \{

struct vvencYUVBuffer;

namespace apputils {


// ====================================================================================================================



class LogoOverlay
{
public:

  std::string          m_cVersion     = VVENC_VERSION;

  class InputOptions
  {
    public:
    std::string          m_cLogoFilename;
    int                  m_SourceWidth  = 0;
    int                  m_SourceHeight = 0;    
    int                  m_Bitdepth     = 10;
    int                  m_BgColorMin   = -1;
    int                  m_BgColorMax   = -1;    
  };

  class RenderOptions
  {
    public:
    int           m_TopLeftX        = 0;
    int           m_TopLeftY        = 0;
    int           m_ScaledWidth     = 0;
    int           m_ScaledHeight    = 0;
    bool          m_KeepAspectRatio = true;
    int           m_Opacity         = 0;
  };

InputOptions   m_InputOpts;
RenderOptions  m_RenderOpts;
};


class LogoOverlayRenderer
{
public:

  LogoOverlayRenderer()
  {

  }

  ~LogoOverlayRenderer()
  {
    if( m_bInitialized ){ uninit(); }
  }
  int init( const std::string &fileName, std::ostream& rcOstr )
  {
    if( m_bInitialized )
    { 
      rcOstr << "error: logo overlay already initialized." << std::endl;
      return -1;
    }
    #ifndef VVENC_ENABLE_THIRDPARTY_JSON
      rcOstr << "error: logo overlay is not supported. please compile with json enabled" << std::endl;
      return -1;
    #endif

    m_rcLogoFHandle.open( fileName, std::ios::in );
    if ( m_rcLogoFHandle.fail() )
    {
      rcOstr << "error: cannot open logo overlay file '" << fileName << "'." << std::endl;
      return -1;
    }

    
    writeLogoFile();
    
    return 0;
  }

  int uninit()
  {
    if( m_bInitialized )
    { 
      return -1;
    }

    if( isOpen() )
      m_rcLogoFHandle.close();

    return 0;
  }

  bool  isInitialized()  { return m_bInitialized; }
  bool  isOpen()  { return m_rcLogoFHandle.is_open(); }
  bool  isEof()   { return m_rcLogoFHandle.eof();     }
  bool  isFail()  { return m_rcLogoFHandle.fail();    }


void writeLogoFile()
{
 //#ifdef VVENC_ENABLE_THIRDPARTY_JSON
     // create a JSON object
    nlohmann::json j =
    {
        {"version", VVENC_VERSION},
        {
            "input_opts", {
                {"//input_comment1", "set logo input options. if BgColorRange >0 px value range of background color (Y), that is removed"},
                {"LogoFilename",     m_cLogoOverlay.m_InputOpts.m_cLogoFilename},
                {"SourceWidth",  m_cLogoOverlay.m_InputOpts.m_SourceWidth},
                {"SourceHeight", m_cLogoOverlay.m_InputOpts.m_SourceHeight},
                {"Bitdepth",     m_cLogoOverlay.m_InputOpts.m_Bitdepth},
                {"//BGColor_comment", "if BgColorMin & BgColorMax >=0 px value range defines background range(Y) that is removed"},
                {"BgColorMin",   m_cLogoOverlay.m_InputOpts.m_BgColorMin},
                {"BgColorMax",   m_cLogoOverlay.m_InputOpts.m_BgColorMax},
            }
        },
        {
            "render_opts", {
                {"//render_comment",    "set logo render options."},
                {"TopLeftX",        m_cLogoOverlay.m_RenderOpts.m_TopLeftX},
                {"TopLeftY",        m_cLogoOverlay.m_RenderOpts.m_TopLeftY},
                {"//Scaling_comment",    "if ScaledWidth >0 or ScaledHeight >0, the logo is resized"},
                {"ScaledWidth",     m_cLogoOverlay.m_RenderOpts.m_ScaledWidth},
                {"ScaledHeight",    m_cLogoOverlay.m_RenderOpts.m_ScaledHeight},
                {"KeepAspectRatio", m_cLogoOverlay.m_RenderOpts.m_KeepAspectRatio},
                {"Opacity",         m_cLogoOverlay.m_RenderOpts.m_Opacity},
            }
        }
    };
 
    m_rcLogoFHandleOut.open( "sample.json", std::ios::out );
    if ( m_rcLogoFHandleOut.fail() )
    {
      return;
    }
  m_rcLogoFHandleOut << std::setw(4) << j << std::endl;
//#endif
}

void readLogoFile()
{
// #ifdef VVENC_ENABLE_THIRDPARTY_JSON
//   std::string line;
//   if( ! std::getline( m_rcStatsFHandle, line ) )
//   {
//     THROW( "unable to read header from rate control statistics file" );
//   }
//   nlohmann::json header = nlohmann::json::parse( line );
//   if( header.find( "version" )         == header.end() || ! header[ "version" ].is_string()
//       || header.find( "SourceWidth" )  == header.end() || ! header[ "SourceWidth" ].is_number()
//       || header.find( "SourceHeight" ) == header.end() || ! header[ "SourceHeight" ].is_number()
//       || header.find( "CTUSize" )      == header.end() || ! header[ "CTUSize" ].is_number()
//       || header.find( "GOPSize" )      == header.end() || ! header[ "GOPSize" ].is_number()
//       || header.find( "IntraPeriod" )  == header.end() || ! header[ "IntraPeriod" ].is_number()
//       || header.find( "PQPA" )         == header.end() || ! header[ "PQPA" ].is_boolean()
//       || header.find( "QP" )           == header.end() || ! header[ "QP" ].is_number()
//       || header.find( "RCInitialQP" )  == header.end() || ! header[ "RCInitialQP" ].is_number()
//     )
//   {
//     THROW( "header line in rate control statistics file not recognized" );
//   }
//   if( header[ "version" ]      != VVENC_VERSION )              msg.log( VVENC_WARNING, "WARNING: wrong version in rate control statistics file\n" );
//   if( header[ "SourceWidth" ]  != m_pcEncCfg->m_SourceWidth )  msg.log( VVENC_WARNING, "WARNING: wrong frame width in rate control statistics file\n" );
//   if( header[ "SourceHeight" ] != m_pcEncCfg->m_SourceHeight ) msg.log( VVENC_WARNING, "WARNING: wrong frame height in rate control statistics file\n" );
//   if( header[ "CTUSize" ]      != m_pcEncCfg->m_CTUSize )      msg.log( VVENC_WARNING, "WARNING: wrong CTU size in rate control statistics file\n" );
//   if( header[ "GOPSize" ]      != m_pcEncCfg->m_GOPSize )      msg.log( VVENC_WARNING, "WARNING: wrong GOP size in rate control statistics file\n" );
//   if( header[ "IntraPeriod" ]  != m_pcEncCfg->m_IntraPeriod )  msg.log( VVENC_WARNING, "WARNING: wrong intra period in rate control statistics file\n" );
// #endif
}


  private:
  bool            m_bInitialized = false;
  std::fstream    m_rcLogoFHandle;
  std::fstream    m_rcLogoFHandleOut;

  LogoOverlay     m_cLogoOverlay;

};

} // namespace apputils

//! \}


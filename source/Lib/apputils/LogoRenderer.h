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
/** \file     LogoRenderer.h
    \brief    yuv logo renderer class (header)
*/

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <optional>
#include <regex>

#include "vvenc/version.h"
#include "vvenc/vvenc.h"

#ifdef VVENC_ENABLE_THIRDPARTY_JSON
#include "../../../thirdparty/nlohmann_json/single_include/nlohmann/json.hpp"
using nlohmann::json;
#endif


//! \ingroup Interface
//! \{

namespace apputils {

// ====================================================================================================================

struct LogoInputOptions
{
  std::string logoFilename;
  int         sourceWidth  = 0;
  int         sourceHeight = 0; 
  int         bitdepth     = 10;
  int         bgColorMin   = -1;
  int         bgColorMax   = -1;
};

struct LogoRenderOptions
{
  int  offsetHor  = 0; // horizontal offset ( >= 0 offset from left, < 0 offset from right)
  int  offsetVer  = 0; // vertical offset   ( >= 0 offset from top, < 0 offset from bottom)
  int  opacity    = 0;
};

struct LogoOverlay
{
  std::string        version = VVENC_VERSION;
  LogoInputOptions   inputOpts;
  LogoRenderOptions  renderOpts;
};

inline void to_json( json& j, const LogoInputOptions& l)
{
  j = json{
    { "LogoFilename",  l.logoFilename },
    { "SourceWidth",   l.sourceWidth },
    { "SourceHeight",  l.sourceHeight },
    { "InputBitDepth", l.bitdepth },
    { "//BgColorMinMax",  "defines background color (min-max range inclusive). color range is removed when >= 0" },
    { "BgColorMin",    l.bgColorMin },
    { "BgColorMax",    l.bgColorMax }
  };
}

inline void to_json( json& j, const LogoRenderOptions& r)
{
  j = json{
    { "//OffsetHor",     "defines logo horizontal offset(x) in px. (clipped to pic borders), if >= 0 offset from left, < 0 offset from right+1" },
    { "//OffsetVer",     "defines logo   vertical offset(y) in px. (clipped to pic borders), if >= 0 offset from top, < 0 offset from bottom+1" },
    { "OffsetHor",        r.offsetHor },
    { "OffsetVer",        r.offsetVer },
    { "//Opacity",       "defines opacity level in range 0-100% (0: opaque - 100: transparent)" },
    { "Opacity",         r.opacity }
  };
}

inline void to_json( json& j, const LogoOverlay& l)
{
  j = json{
    { "version",      l.version     },
    { "input_opts",   l.inputOpts   },
    { "render_opts",  l.renderOpts  },
  };
}

inline void from_json(const json& j, LogoInputOptions& l)
{  
  j.at("LogoFilename").get_to(l.logoFilename);
  j.at("SourceWidth").get_to(l.sourceWidth);  
  j.at("SourceHeight").get_to(l.sourceHeight);
  j.at("InputBitDepth").get_to(l.bitdepth);
  j.at("BgColorMin").get_to(l.bgColorMin);
  j.at("BgColorMax").get_to(l.bgColorMax);
}

inline void from_json(const json& j, LogoRenderOptions& l )
{
  j.at("OffsetHor").get_to(l.offsetHor);
  j.at("OffsetVer").get_to(l.offsetVer);
  j.at("Opacity").get_to(l.opacity);
}

inline void from_json(const json& j, LogoOverlay& l )
{
  j.at("version").get_to(l.version);
  j.at("input_opts").get_to(l.inputOpts);
  j.at("render_opts").get_to(l.renderOpts);
}

class LogoRenderer
{
public:
  LogoRenderer()
  {
  }

  ~LogoRenderer()
  {
    if( m_bInitialized ){ uninit(); }
  }
  
  int init( const std::string &fileName, vvencChromaFormat chromaFormat, std::ostream& rcOstr )
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
   
    m_chromaFormat = chromaFormat;
    if( readLogoFile( fileName, rcOstr ) )
    {
      rcOstr << "sample json file for logo description:" << std::endl;     
      dumpOutput( rcOstr );
      return -1;
    }
    
    std::string ext;
    if(m_cLogo.inputOpts.logoFilename.find_last_of(".") != std::string::npos)
    {
      ext = m_cLogo.inputOpts.logoFilename.substr(m_cLogo.inputOpts.logoFilename.find_last_of(".")+1);
      std::transform( ext.begin(), ext.end(), ext.begin(), ::tolower );
    }
    if( "y4m" == ext )
    {
      if ( 0 != parseY4M ( m_cLogo.inputOpts.logoFilename ) )
      {
        rcOstr << "cannot parse y4m information in file " << m_cLogo.inputOpts.logoFilename << std::endl;     
        return -1;
      }
    }    
       
    vvenc_YUVBuffer_default( &m_cYuvBufLogo );
    vvenc_YUVBuffer_alloc_buffer( &m_cYuvBufLogo, chromaFormat, m_cLogo.inputOpts.sourceWidth, m_cLogo.inputOpts.sourceHeight );
    
    if( m_cLogo.renderOpts.opacity >= 100 )
    {
      m_bBypass = true;
    }
    else
    {
      m_bBypass = false;
      if( (m_cLogo.inputOpts.bgColorMin >= 0 && m_cLogo.inputOpts.bgColorMax >= 0 ) || 
           m_cLogo.renderOpts.opacity > 0 )
      {
        m_bAlphaNeeded = true;
        vvenc_YUVBuffer_default( &m_cYuvBufAlpha );
        vvenc_YUVBuffer_alloc_buffer( &m_cYuvBufAlpha, VVENC_CHROMA_400, m_cLogo.inputOpts.sourceWidth, m_cLogo.inputOpts.sourceHeight );
      }
    }
    
    m_bLogoReady   = false;
    m_bInitialized = true; 
    
    return 0;
  }

  int uninit()
  {
    if( m_bInitialized )
    { 
      return -1;
    }
    
    vvenc_YUVBuffer_free_buffer( &m_cYuvBufLogo );
    if ( m_bAlphaNeeded )
    {
      vvenc_YUVBuffer_free_buffer( &m_cYuvBufAlpha );
    }
    
    m_bInitialized = false;
    return 0;
  }

  bool isInitialized()  const { return m_bInitialized; }
  
  LogoInputOptions getLogoInputOptions() { return m_cLogo.inputOpts; }
  vvencYUVBuffer* getLogoYuvBuffer()     { return &m_cYuvBufLogo; }
   
  void dumpOutput( std::ostream& rcOstr )
  {
   #ifdef VVENC_ENABLE_THIRDPARTY_JSON  
    const json j { m_cLogo }; 
    rcOstr << j.dump(2) << std::endl;  
   #endif
  }
  
  int writeLogoFile( std::string fileName, std::ostream& rcOstr )
  {
  #ifdef VVENC_ENABLE_THIRDPARTY_JSON
    std::fstream    logoFHandle;   
    logoFHandle.open( fileName, std::ios::out );
    if ( logoFHandle.fail() )
    {
      rcOstr << "error: cannot open logo overlay file '" << fileName << "'." << std::endl;
      return -1;
    }
    
    const json j { m_cLogo }; 
    logoFHandle << std::setw(4) << j << std::endl;
    
    if(  logoFHandle.is_open() )
      logoFHandle.close();
    return 0;
  #else
    return -1;
  #endif
  }
  
  int readLogoFile( std::string fileName, std::ostream& rcOstr )
  {
  #ifdef VVENC_ENABLE_THIRDPARTY_JSON
    std::fstream    logoFHandle;   
    logoFHandle.open( fileName, std::ios::in );
    if ( logoFHandle.fail() )
    {
      rcOstr << "error: cannot open logo overlay file '" << fileName << "'." << std::endl;
      return -1;
    }
       
    try
    {
      json j;
      logoFHandle >> j;
      std::vector<LogoOverlay> logoInput = j.get<std::vector<LogoOverlay>>();
      if( !logoInput.empty())
        m_cLogo = logoInput.at(0);
      else
      {
        rcOstr << "error: logo json parsing error in file '" << fileName << "'." << std::endl;
        return -1;
      }
    }
    catch (std::exception& e)
    {
      rcOstr << "logo json parsing error: " << e.what() << "\n";
      return -1;
    }
    
    if( m_cLogo.inputOpts.bgColorMin >= 0 && m_cLogo.inputOpts.bgColorMax < 0 )
    {
      rcOstr << "logo must define range of BgColorMin/BgColorMax. set BgColorMin but missing BgColorMax (min/max " << 
                 m_cLogo.inputOpts.bgColorMin << "/" << m_cLogo.inputOpts.bgColorMax << ")\n";
      return -1;
    }
    if( m_cLogo.inputOpts.bgColorMax >= 0 && m_cLogo.inputOpts.bgColorMin < 0 )
    {
      rcOstr << "logo must define range of BgColorMin/BgColorMax. set BgColorMax but missing BgColorMin (min/max " << 
                 m_cLogo.inputOpts.bgColorMin << "/" << m_cLogo.inputOpts.bgColorMax << ")\n";
      return -1;
    }
    if( m_cLogo.inputOpts.bgColorMin >= 0 && m_cLogo.inputOpts.bgColorMax < m_cLogo.inputOpts.bgColorMin )
    {
      rcOstr << "logo must define range of BgColorMin/BgColorMax. BgColorMax must be >= BgColorMin (min/max " << 
                 m_cLogo.inputOpts.bgColorMin << "/" << m_cLogo.inputOpts.bgColorMax << ")\n";
      return -1;
    }
        
    // limit opacity in range 0-100 %
    m_cLogo.renderOpts.opacity = std::min( m_cLogo.renderOpts.opacity, 100 );
    m_cLogo.renderOpts.opacity = std::max( m_cLogo.renderOpts.opacity, 0 );
      
    if(  logoFHandle.is_open() )
      logoFHandle.close();

    return 0;
  #else
    return -1;
  #endif    
  }
  
  void setLogoReady()
  {
    if( !m_bInitialized ) return;
    m_bLogoReady = true;
    if( !m_bAlphaNeeded || m_bBypass ) return;
    
    // init alpha mask
    bool bgColorSet  = (m_cLogo.inputOpts.bgColorMin >= 0 && m_cLogo.inputOpts.bgColorMax >= 0 ) ? true : false;
    bool opacitySet  = (m_cLogo.renderOpts.opacity > 0) ? true : false;
    int transp = 100 - m_cLogo.renderOpts.opacity; // transparency level

    vvencYUVPlane yuvSrc = m_cYuvBufLogo.planes[0];
    vvencYUVPlane yuvDes = m_cYuvBufAlpha.planes[0];
    
    const int16_t* src = yuvSrc.ptr;
    int16_t*       dst = yuvDes.ptr;
    
    for( int y = 0; y < yuvSrc.height; y++ )
    {
      for( int x = 0; x < yuvSrc.width; x++ )
      {
        if( bgColorSet && ( src[x] >= m_cLogo.inputOpts.bgColorMin && src[x] <= m_cLogo.inputOpts.bgColorMax ))
        {
          dst[x] = 0; // ignore background ( transparent )
        }
        else if( opacitySet )
        {
          dst[x] = transp;
        }
        else
        {
          dst[x] = 100;  // opaque
        }
      }
      src += yuvSrc.stride;
      dst += yuvDes.stride;
    }
  } 
  
  int renderLogo ( const vvencYUVBuffer& yuvDestBuf )
  {   
    if( !m_bInitialized || !m_bLogoReady )
    {
      return -1;
    }

    if( m_bBypass ){ return 0; }
    
    if ( m_cYuvBufLogo.planes[0].width > yuvDestBuf.planes[0].width || m_cYuvBufLogo.planes[0].height > yuvDestBuf.planes[0].height )
    {    
      return -1;
    }
    
    int logoPosX = 0;
    int logoPosY = 0;
    if( m_cLogo.renderOpts.offsetHor != 0 )
    {
      const int maxX = yuvDestBuf.planes[0].width - m_cYuvBufLogo.planes[0].width;      
      logoPosX = (m_cLogo.renderOpts.offsetHor >= 0) ? 
                  std::min( maxX, m_cLogo.renderOpts.offsetHor ) :
                  std::max( 0, maxX + m_cLogo.renderOpts.offsetHor+1);
      
    } 
    
    if( m_cLogo.renderOpts.offsetVer != 0 )
    {
      const int maxY = yuvDestBuf.planes[0].height - m_cYuvBufLogo.planes[0].height; 
      logoPosY = (m_cLogo.renderOpts.offsetVer >= 0) ? 
                  std::min( maxY, m_cLogo.renderOpts.offsetVer ) :
                  std::max( 0, maxY + m_cLogo.renderOpts.offsetVer+1);
    }
       
    const int numComp = (m_chromaFormat==VVENC_CHROMA_400) ? 1 : 3;   
    for( int comp = 0; comp < numComp; comp++ )
    {
      vvencYUVPlane yuvDes = yuvDestBuf.planes[ comp ];
      vvencYUVPlane yuvLogo = m_cYuvBufLogo.planes[ comp ];
      
      const int csx = ( (comp == 0) || (m_chromaFormat==VVENC_CHROMA_444) ) ? 0 : 1;
      const int csy = ( (comp == 0) || (m_chromaFormat!=VVENC_CHROMA_420) ) ? 0 : 1;
      const int16_t* src = yuvLogo.ptr;
      int16_t* dst = yuvDes.ptr + ( (logoPosY >> csy) * yuvDes.stride ) + (logoPosX >> csx);

      if( m_bAlphaNeeded && m_cYuvBufAlpha.planes[0].ptr )
      {
        vvencYUVPlane yuvAlpha = m_cYuvBufAlpha.planes[0];
        const int16_t* alpha = yuvAlpha.ptr;
        
        for( int y = 0; y < yuvLogo.height; y++ )
        {
          int xA = 0;
          for( int x = 0; x < yuvLogo.width; x++, xA += (1 << csx) )
          {         
            if( alpha[xA] >= 100 )
            {
              dst[x] = src[x];                                        
            }
            else if( alpha[xA] > 0 )
            {
              dst[x] = (( src[x] * alpha[xA] ) + ( dst[x] * (100-alpha[xA]) )) / 100 ;   
            }
          }
          src   += yuvLogo.stride;
          dst   += yuvDes.stride;
          alpha += ( (1 << csy) * yuvAlpha.stride);
        }
      }
      else
      {
        for( int y = 0; y < yuvLogo.height; y++ )
        {
          for( int x = 0; x < yuvLogo.width; x++ )
          {         
            dst[x] = src[x];            
          }
          src += yuvLogo.stride;
          dst += yuvDes.stride;
        }
      }
    }
    
    return 0;
  }
  
private:
  
  int parseY4M( std::string fileName )
  {
    std::fstream cfHandle;
    cfHandle.open( fileName, std::ios::binary | std::ios::in );
    if( cfHandle.fail() )
    {
      return -1;
    }
      
    std::string headerline;
    getline(cfHandle, headerline);
    if( headerline.empty() ){ return -1; }
    std::transform( headerline.begin(), headerline.end(), headerline.begin(), ::toupper );
  
    std::regex reg("\\s+"); // tokenize at spaces
    std::sregex_token_iterator iter(headerline.begin(), headerline.end(), reg, -1);
    std::sregex_token_iterator end;
    std::vector<std::string> vec(iter, end);
  
    bool valid=false;
    for (auto &p : vec)
    {
      if( p == "YUV4MPEG2" ) // read file signature
      { valid = true; }
      else if( p[0] == 'W' ) // width
        m_cLogo.inputOpts.sourceWidth = atoi( p.substr( 1 ).c_str());
      else if( p[0] == 'H' ) // height
        m_cLogo.inputOpts.sourceHeight = atoi( p.substr( 1 ).c_str());
      else if( p[0] == 'F' )  // framerate,scale
      {
        size_t sep = p.find(":");
        if( sep == std::string::npos ) return -1;
      }
      else if( p[0] == 'A' ) // aspcet ration
      {
        size_t sep = p.find(":");
        if( sep == std::string::npos ) return -1;
      }
      else if( p[0] == 'C' ) // colorspace ( e.g. C420p10)
      {
        std::vector<std::string> ignores = {"JPEG", "MPEG2", "PALVD" }; // ignore some special cases
        for( auto &i : ignores )
        {
          auto n = p.find( i );
          if (n != std::string::npos) p.erase(n, i.length()); // remove from param string (e.g. 420PALVD)
        }
  
        size_t sep = p.find("P");
        std::string chromatype;
        if( sep != std::string::npos )
        {
          chromatype = ( p.substr( 1, sep-1 ).c_str());
          m_cLogo.inputOpts.bitdepth = atoi( p.substr( sep+1 ).c_str());
        }
        else
        {
          sep = p.find("MONO");
          if( sep != std::string::npos )
          {
            chromatype = "400";
            if( p == "MONO") m_cLogo.inputOpts.bitdepth = 8;
            else m_cLogo.inputOpts.bitdepth = atoi( p.substr( sep+5 ).c_str()); // e.g. mono10
          }
          else
          {
            chromatype = ( p.substr( 1 ).c_str());
            m_cLogo.inputOpts.bitdepth = 8;
          }
        }
  
        if( chromatype == "400" )      { m_chromaFormat =  VVENC_CHROMA_400; }
        else if( chromatype == "420" ) { m_chromaFormat =  VVENC_CHROMA_420; }
        else if( chromatype == "422" ) { m_chromaFormat =  VVENC_CHROMA_422; }
        else if( chromatype == "444" ) { m_chromaFormat =  VVENC_CHROMA_444; }
        else { return -1; } // unsupported chroma foramt}
      }
      else if( p[0] == 'I' ) // interlaced format (ignore it, because we cannot set it in any params
      {}
      else if( p[0] == 'X' ) // ignore comments
      {}
    }
  
    cfHandle.close();
    if( !valid ) return -1;
    
    return 0;
  }
   

private:
  bool              m_bInitialized = false;
  bool              m_bLogoReady   = false;
  vvencChromaFormat m_chromaFormat = VVENC_NUM_CHROMA_FORMAT;
  LogoOverlay       m_cLogo;
  vvencYUVBuffer    m_cYuvBufLogo;
  vvencYUVBuffer    m_cYuvBufAlpha;
  bool              m_bAlphaNeeded = false;
  bool              m_bBypass      = false;
};

} // namespace apputils

//! \}


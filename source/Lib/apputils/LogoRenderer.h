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
  int  topLeftX        = 0;
  int  topLeftY        = 0;
  int  marginRight     = 0;
  int  marginBottom    = 0;
  int  scaledWidth     = 0;
  int  scaledHeight    = 0;
  bool keepAspectRatio = true;
  int  opacity         = 0;
};

struct LogoOverlay
{
  std::string       version = VVENC_VERSION;
  LogoInputOptions  inputOpts;
  LogoRenderOptions renderOpts;
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
    { "//TopLeftXY",     "defines logo top left(x,y) position in px. position is clipped to max width and height (right and bottom picture border)" },
    { "TopLeftX",        r.topLeftX },
    { "TopLeftY",        r.topLeftY },
    { "//MarginRight",   "defines logo margin to right border in px. position is clipped left picture border" },
    { "MarginRight",     r.marginRight },
    { "//MarginBottom",  "defines logo margin to bottom border in px. position is clipped top picture border" },
    { "MarginBottom",    r.marginBottom },
    { "//ScaledWidthHeight","defines logo scaling in px. Not implemented yet!" },
    { "ScaledWidth",     r.scaledWidth },
    { "ScaledHeight",    r.scaledHeight },
    { "KeepAspectRatio", r.keepAspectRatio },
    { "//Opacity",       "defines opacity level in range 0-100% (0: opaque - 100: transparent)" },
    { "Opacity",         r.opacity }
  };
}

inline void to_json( json& j, const LogoOverlay& l)
{
  j = json{
    { "version",     l.version    },
    { "input_opts",  l.inputOpts  },
    { "render_opts", l.renderOpts }
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
  j.at("TopLeftX").get_to(l.topLeftX);
  j.at("TopLeftY").get_to(l.topLeftY);
  j.at("MarginRight").get_to(l.marginRight);
  j.at("MarginBottom").get_to(l.marginBottom);
  j.at("ScaledWidth").get_to(l.scaledWidth);
  j.at("ScaledHeight").get_to(l.scaledHeight);
  j.at("KeepAspectRatio").get_to(l.keepAspectRatio);
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
      rcOstr << "write sample logo file 'sample.json' as reference." << std::endl;      
      writeLogoFile( "sample.json", rcOstr );
      return -1;
    }
       
    vvenc_YUVBuffer_default( &m_cYuvBufLogo );
    vvenc_YUVBuffer_alloc_buffer( &m_cYuvBufLogo, chromaFormat, m_cLogoOverlay.inputOpts.sourceWidth, m_cLogoOverlay.inputOpts.sourceHeight );
    
    if( (m_cLogoOverlay.inputOpts.bgColorMin >= 0 && m_cLogoOverlay.inputOpts.bgColorMax >= 0 ) || m_cLogoOverlay.renderOpts.opacity > 0 )
    {
      m_bAlphaNeeded = true;
      vvenc_YUVBuffer_default( &m_cYuvBufAlpha );
      vvenc_YUVBuffer_alloc_buffer( &m_cYuvBufAlpha, VVENC_CHROMA_400, m_cLogoOverlay.inputOpts.sourceWidth, m_cLogoOverlay.inputOpts.sourceHeight );
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
  
  LogoInputOptions getLogoInputOptions() { return m_cLogoOverlay.inputOpts; }
  vvencYUVBuffer* getLogoYuvBuffer()     { return &m_cYuvBufLogo; }
   
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
    
    const json j { m_cLogoOverlay }; 
    logoFHandle << std::setw(4) << j << std::endl;
    
    if(  logoFHandle.is_open() )
      logoFHandle.close();
    return 0;
  #endif
    return -1;
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
        m_cLogoOverlay = logoInput.at(0);
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
    
    if( m_cLogoOverlay.inputOpts.bgColorMin >= 0 && m_cLogoOverlay.inputOpts.bgColorMax < 0 )
    {
      rcOstr << "logo must define range of BgColorMin/BgColorMax. set BgColorMin but missing BgColorMax (min/max " << 
                 m_cLogoOverlay.inputOpts.bgColorMin << "/" << m_cLogoOverlay.inputOpts.bgColorMax << ")\n";
      return -1;
    }
    if( m_cLogoOverlay.inputOpts.bgColorMax >= 0 && m_cLogoOverlay.inputOpts.bgColorMin < 0 )
    {
      rcOstr << "logo must define range of BgColorMin/BgColorMax. set BgColorMax but missing BgColorMin (min/max " << 
                 m_cLogoOverlay.inputOpts.bgColorMin << "/" << m_cLogoOverlay.inputOpts.bgColorMax << ")\n";
      return -1;
    }
    if( m_cLogoOverlay.inputOpts.bgColorMin >= 0 && m_cLogoOverlay.inputOpts.bgColorMax <= m_cLogoOverlay.inputOpts.bgColorMin )
    {
      rcOstr << "logo must define range of BgColorMin/BgColorMax. BgColorMax must be > BgColorMin (min/max " << 
                 m_cLogoOverlay.inputOpts.bgColorMin << "/" << m_cLogoOverlay.inputOpts.bgColorMax << ")\n";
      return -1;
    }
        
    // limit opacity in range 0-100 %
    m_cLogoOverlay.renderOpts.opacity = std::min( m_cLogoOverlay.renderOpts.opacity, 100 );
    m_cLogoOverlay.renderOpts.opacity = std::max( m_cLogoOverlay.renderOpts.opacity, 0 );
      
    if(  logoFHandle.is_open() )
      logoFHandle.close();
  #endif
      
    return 0;
  }
  
  void setLogoReady()
  {
    if( !m_bInitialized ) return;
    m_bLogoReady = true;
    if( !m_bAlphaNeeded ) return;
    
    bool bgColorSet  = (m_cLogoOverlay.inputOpts.bgColorMin >= 0 && m_cLogoOverlay.inputOpts.bgColorMax >= 0 ) ? true : false;
    bool opacitySet  = (m_cLogoOverlay.renderOpts.opacity > 0) ? true : false;
    int transp = 100 - m_cLogoOverlay.renderOpts.opacity; // transparency level

    vvencYUVPlane yuvSrc = m_cYuvBufLogo.planes[ 0 ];
    vvencYUVPlane yuvDes  = m_cYuvBufAlpha.planes[ 0 ];
    
    const int16_t* src = yuvSrc.ptr;
    int16_t* dst = yuvDes.ptr;
    
    for( int y = 0; y < yuvSrc.height; y++ )
    {
      for( int x = 0; x < yuvSrc.width; x++ )
      {
        if( bgColorSet && ( src[x] >= m_cLogoOverlay.inputOpts.bgColorMin && src[x] <= m_cLogoOverlay.inputOpts.bgColorMax ))
        {
          dst[x] = 0; // ignore background
        }
        else if( opacitySet )
        {
          dst[x] = transp;
        }
        else
        {
          dst[x] = 100;
        }
      }
      src += yuvSrc.stride;
      dst += yuvDes.stride;
    }
  } 
  
  int renderLogo ( const vvencYUVBuffer& yuvDestBuf, std::ostream& rcOstr )
  {   
    if( !m_bInitialized )
    {
      rcOstr << "LogoRenderer not initialized" << std::endl;     
      return -1;
    }
    if( !m_bLogoReady )
    {
      rcOstr << "Logo not ready - setup not completed." << std::endl;     
      return -1;
    }
    if ( m_cYuvBufLogo.planes[0].width > yuvDestBuf.planes[0].width && m_cYuvBufLogo.planes[0].height > yuvDestBuf.planes[0].height )
    {
      rcOstr << "input picture size (" << yuvDestBuf.planes[0].width << "x" << yuvDestBuf.planes[0].height << ") < logo size (" << 
                 m_cYuvBufLogo.planes[0].width << "x" << m_cYuvBufLogo.planes[0].height << ") cannot render logo" << std::endl;     
      return -1;
    }
    if ( m_cYuvBufLogo.planes[0].width > yuvDestBuf.planes[0].width )
    {
      rcOstr << "logo rendering error: input picture width (" << yuvDestBuf.planes[0].width <<  ") < logo width (" << 
                 m_cYuvBufLogo.planes[0].width << ") cannot render logo" << std::endl;     
      return -1;
    }
    if ( m_cYuvBufLogo.planes[0].height > yuvDestBuf.planes[0].height )
    {
      rcOstr << "logo rendering error: input picture height (" << yuvDestBuf.planes[0].width <<  ") < logo height (" << 
                 m_cYuvBufLogo.planes[0].height << ") cannot render logo" << std::endl;     
      return -1;
    }
    
    int logoPosX = 0;
    int logoPosY = 0;
    if( m_cLogoOverlay.renderOpts.topLeftX > 0 )
    {
      int maxX = yuvDestBuf.planes[0].width - m_cYuvBufLogo.planes[0].width; 
      logoPosX = m_cLogoOverlay.renderOpts.topLeftX > maxX ? maxX : m_cLogoOverlay.renderOpts.topLeftX;
      if( m_cLogoOverlay.renderOpts.marginRight )
      {
        logoPosX = std::max( 0, logoPosX - m_cLogoOverlay.renderOpts.marginRight);
      }
    }
    
    if( m_cLogoOverlay.renderOpts.topLeftY > 0 )
    {
      int maxY = yuvDestBuf.planes[0].height - m_cYuvBufLogo.planes[0].height; 
      logoPosY = m_cLogoOverlay.renderOpts.topLeftY > maxY ? maxY : m_cLogoOverlay.renderOpts.topLeftY;
      if( m_cLogoOverlay.renderOpts.marginBottom )
      {
        logoPosY = std::max( 0, logoPosY - m_cLogoOverlay.renderOpts.marginBottom);
      }
    }
       
    const int numComp = (m_chromaFormat==VVENC_CHROMA_400) ? 1 : 3;   
    for( int comp = 0; comp < numComp; comp++ )
    {
      vvencYUVPlane yuvDes = yuvDestBuf.planes[ comp ];
      vvencYUVPlane yuvLogo = m_cYuvBufLogo.planes[ comp ];
      vvencYUVPlane yuvAlpha = m_cYuvBufAlpha.planes[0];
      
      const int csx = ( (comp == 0) || (m_chromaFormat==VVENC_CHROMA_444) ) ? 0 : 1;
      const int csy = ( (comp == 0) || (m_chromaFormat!=VVENC_CHROMA_420) ) ? 0 : 1;
      const int16_t* src = yuvLogo.ptr;
      int16_t* dst = yuvDes.ptr + ( (logoPosY >> csy) * yuvDes.stride ) + (logoPosX >> csx);
      for( int y = 0; y < yuvLogo.height; y++ )
      {
        const int16_t* alpha = yuvAlpha.ptr + (( y << csy ) * yuvAlpha.stride);
        for( int x = 0; x < yuvLogo.width; x++ )
        {         
          if( m_bAlphaNeeded )
          { 
            const int xA = x << csx; 
            if( alpha[xA] >= 100 )
            {
              dst[x] = src[x];                                        
            }
            else if( comp == 0 && alpha[xA] > 0 )
            {
              dst[x] = (( src[x] * alpha[xA] ) + ( dst[x] * (100-alpha[xA]) )) / 100 ;   
            }
          }
          else
          {
            dst[x] = src[x];            
          }
        }
        src   += yuvLogo.stride;
        dst   += yuvDes.stride;
      }
    }
    
    return 0;
  }

  private:
  bool              m_bInitialized = false;
  bool              m_bLogoReady   = false;
  std::fstream      m_rcLogoFHandleOut;
  vvencChromaFormat m_chromaFormat = VVENC_NUM_CHROMA_FORMAT;
  LogoOverlay       m_cLogoOverlay;
  vvencYUVBuffer    m_cYuvBufLogo;
  vvencYUVBuffer    m_cYuvBufAlpha;
  bool              m_bAlphaNeeded = false;
};

} // namespace apputils

//! \}


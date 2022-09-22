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
/** \file     EncApp.h
    \brief    Encoder application class (header)
*/

#pragma once

#include <ostream>
#include <cstdarg>

#include "vvenc/vvenc.h"
#include "vvenc/vvencCfg.h"

#include "apputils/YuvFileIO.h"
#include "apputils/VVEncAppCfg.h"


//! \ingroup EncoderApp
//! \{

// ====================================================================================================================

extern vvencMsgLevel g_verbosity;
void msgFnc( void*, int level, const char* fmt, va_list args );
void msgApp( int level, const char* fmt, ... );

// ====================================================================================================================

typedef struct vvencEncApp vvencEncApp;
//extern vvencEncApp *g_vvencEncApp;


class EncApp
{
private:

static void changePreset( vvenc_config* c, vvencPresetMode preset )
{
  if( c ) vvenc_init_preset( c, (vvencPresetMode)preset );
}

private:
  apputils::VVEncAppCfg m_cEncAppCfg;                     ///< encoder application configuration
  vvenc_config          m_vvenc_config;
  vvencEncoder         *m_encCtx;                         ///< encoder library class
  apputils::YuvFileIO   m_yuvInputFile;                   ///< input YUV file
  apputils::YuvFileIO   m_yuvReconFile;                   ///< output YUV reconstruction file
  std::fstream          m_bitstream;                      ///< output bitstream file
  unsigned              m_essentialBytes;
  unsigned              m_totalBytes;

public:
  EncApp()
    : m_essentialBytes( 0 )
    , m_totalBytes    ( 0 )
  {
    vvenc_config_default( &m_vvenc_config );

    m_cEncAppCfg.setPresetChangeCallback(changePreset);

  }

  virtual ~EncApp()
  {
  }

  bool  parseCfg( int argc, char* argv[] );           ///< parse configuration file to fill member variables
  int   encode();                                     ///< main encoding function
  void  outputAU ( const vvencAccessUnit& au );            ///< write encoded access units to bitstream
  static void outputYuv( void*, vvencYUVBuffer* );      ///< write reconstructed yuv output

  bool isShowVersionHelp()
  {
    return (m_cEncAppCfg.m_showVersion || m_cEncAppCfg.m_showHelp );
  }

private:
  // file I/O
  bool openFileIO();
  void closeFileIO();

  // statistics
  void printRateSummary( int framesRcvd );
  void printChromaFormat();
};

//! \}


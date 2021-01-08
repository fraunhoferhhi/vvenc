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
/** \file     Basics.h
    \brief    encoder lib basic defines
*/

#pragma once

#include <cstdint>
#include <cstdarg>
#include <vector>
#include "vvenc/vvencDecl.h"
#include "vvenc/vvencConfig.h"


//! \ingroup Interface
//! \{

namespace vvenc {


// ====================================================================================================================

struct VVENC_DECL YUVPlane
{
  int16_t* planeBuf;
  int      width;
  int      height;
  int      stride;

  YUVPlane()
    : planeBuf( nullptr )
    , width   ( 0 )
    , height  ( 0 )
    , stride  ( 0 )
  {
  }
};

struct VVENC_DECL YUVBuffer
{
  YUVPlane yuvPlanes[ MAX_NUM_COMP ];
  uint64_t sequenceNumber;     ///< sequence number of the picture
  uint64_t cts;                ///< composition time stamp in TicksPerSecond (see HEVCEncoderParameter)
  bool     ctsValid;            ///< composition time stamp valid flag (true: valid, false: CTS not set)

  YUVBuffer()
  : sequenceNumber ( 0 )
  , cts            ( 0 )
  , ctsValid        ( false )
  {
  }
};

// ====================================================================================================================

struct VVENC_DECL ChromaQpMappingTableParams
{
  int               m_numQpTables;
  int               m_qpBdOffset;
  bool              m_sameCQPTableForAllChromaFlag;
  int               m_qpTableStartMinus26[MAX_NUM_CQP_MAPPING_TABLES];
  int               m_numPtsInCQPTableMinus1[ MAX_NUM_CQP_MAPPING_TABLES ];
  std::vector<int>  m_deltaQpInValMinus1[ MAX_NUM_CQP_MAPPING_TABLES ];
  std::vector<int>  m_deltaQpOutVal[ MAX_NUM_CQP_MAPPING_TABLES ];

  ChromaQpMappingTableParams()
  : m_numQpTables                   ( 0 )
  , m_qpBdOffset                    ( 12 )
  , m_sameCQPTableForAllChromaFlag  ( true )
  , m_qpTableStartMinus26           { 0 }
  , m_numPtsInCQPTableMinus1        { 0 }
  {
  }
};

struct VVENC_DECL ReshapeCW
{
  std::vector<uint32_t> binCW;
  int       updateCtrl;
  int       adpOption;
  uint32_t  initialCW;
  int       rspPicSize;
  int       rspFps;
  int       rspBaseQP;
  int       rspTid;
  int       rspFpsToIp;
};

// ====================================================================================================================

static inline int getWidthOfComponent( const ChromaFormat& chFmt, const int frameWidth, const int compId )
{
  int w = frameWidth;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case CHROMA_400: w = 0;      break;
      case CHROMA_420:
      case CHROMA_422: w = w >> 1; break;
      default: break;
    }
  }
  return w;
}

static inline int getHeightOfComponent( const ChromaFormat& chFmt, const int frameHeight, const int compId )
{
  int h = frameHeight;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case CHROMA_400: h = 0;      break;
      case CHROMA_420: h = h >> 1; break;
      case CHROMA_422:
      default: break;
    }
  }
  return h;
}

} // namespace vvenc

//! \}


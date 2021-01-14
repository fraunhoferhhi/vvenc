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
/**
  \ingroup vvencExternalInterfaces
  \file    vvenc.h
  \brief   This file contains the external interface of the hhivvcdec SDK.
  \author  christian.lehmann@hhi.fraunhofer.de
  \date    08/08/2020
*/

#pragma once

#include "vvenc/vvencDecl.h"
#include <cstdint>
#include <cstdarg>
#include <string>
#include <vector>

namespace vvenc {


// ====================================================================================================================

static const int MAX_GOP                         = 64; ///< max. value of hierarchical GOP size
static const int MAX_NUM_REF_PICS                = 29; ///< max. number of pictures used for reference
static const int MAX_TLAYER                      =  7; ///< Explicit temporal layer QP offset - max number of temporal layer
static const int MAX_NUM_CQP_MAPPING_TABLES      =  3; ///< Maximum number of chroma QP mapping tables (Cb, Cr and joint Cb-Cr)
static const int MAX_NUM_ALF_ALTERNATIVES_CHROMA =  8;
static const int MCTF_RANGE                      =  2; ///< max number of frames used for MCTF filtering in forward / backward direction

// ====================================================================================================================


/**
  \ingroup VVEncExternalInterfaces
  \enum MsgLevel
  The enum MsgLevel enumerates supported log levels/verbosity.
*/
enum MsgLevel
{
  SILENT  = 0,
  ERROR   = 1,
  WARNING = 2,
  INFO    = 3,
  NOTICE  = 4,
  VERBOSE = 5,
  DETAILS = 6
};


/**
  \ingroup VVEncExternalInterfaces
  \enum Profile
  The enum Profile enumerates supported profiles
*/
enum Profile
{
  PROFILE_NONE                         = 0,
  MAIN_10                              = 1,
  MAIN_10_STILL_PICTURE                = 2,
  MAIN_10_444                          = 3,
  MAIN_10_444_STILL_PICTURE            = 4,
  MULTILAYER_MAIN_10                   = 5,
  MULTILAYER_MAIN_10_STILL_PICTURE     = 6,
  MULTILAYER_MAIN_10_444               = 7,
  MULTILAYER_MAIN_10_444_STILL_PICTURE = 8,
  PROFILE_AUTO
};


/**
  \ingroup VVEncExternalInterfaces
  \enum Tier
  The enum Tier enumerates supported tier
*/
enum Tier
{
  TIER_MAIN = 0,
  TIER_HIGH = 1,
  NUMBER_OF_TIERS
};

/**
  \ingroup VVEncExternalInterfaces
  \enum Name
  The enum Name enumerates supported level names
*/
enum Level
{
  LEVEL_NONE = 0,
  LEVEL1   = 16,
  LEVEL2   = 32,
  LEVEL2_1 = 35,
  LEVEL3   = 48,
  LEVEL3_1 = 51,
  LEVEL4   = 64,
  LEVEL4_1 = 67,
  LEVEL5   = 80,
  LEVEL5_1 = 83,
  LEVEL5_2 = 86,
  LEVEL6   = 96,
  LEVEL6_1 = 99,
  LEVEL6_2 = 102,
  LEVEL6_3 = 105,
  LEVEL15_5 = 255,
};

/**
  \ingroup VVEncExternalInterfaces
  \enum chroma
  chroma formats (according to semantics of chroma_format_idc)
*/
enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum ComponentID
{
  COMP_Y          = 0,
  COMP_Cb         = 1,
  COMP_Cr         = 2,
  MAX_NUM_COMP    = 3,
  COMP_JOINT_CbCr = MAX_NUM_COMP,
  MAX_NUM_TBLOCKS = MAX_NUM_COMP
};

enum ChannelType
{
  CH_L = 0,
  CH_C = 1,
  MAX_NUM_CH = 2
};


/**
  \ingroup VVEnc
  The class SliceType enumerates several supported slice types.
*/
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES = 3
};


/// supported IDR types
enum DecodingRefreshType
{
  DRT_NONE               = -1,
  DRT_CRA                = 0,
  DRT_IDR                = 1,
  DRT_RECOVERY_POINT_SEI = 2
};

enum HashType
{
  HASHTYPE_MD5        = 0,
  HASHTYPE_CRC        = 1,
  HASHTYPE_CHECKSUM   = 2,
  HASHTYPE_NONE       = 3,
  NUMBER_OF_HASHTYPES = 4
};


enum NalUnitType
{
  NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  NAL_UNIT_CODED_SLICE_STSA,        // 1
  NAL_UNIT_CODED_SLICE_RADL,        // 2
  NAL_UNIT_CODED_SLICE_RASL,        // 3

  NAL_UNIT_RESERVED_VCL_4,
  NAL_UNIT_RESERVED_VCL_5,
  NAL_UNIT_RESERVED_VCL_6,

  NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  NAL_UNIT_CODED_SLICE_CRA,         // 9
  NAL_UNIT_CODED_SLICE_GDR,         // 10

  NAL_UNIT_RESERVED_IRAP_VCL_11,
  NAL_UNIT_RESERVED_IRAP_VCL_12,
  NAL_UNIT_DCI,                     // 13
  NAL_UNIT_VPS,                     // 14
  NAL_UNIT_SPS,                     // 15
  NAL_UNIT_PPS,                     // 16
  NAL_UNIT_PREFIX_APS,              // 17
  NAL_UNIT_SUFFIX_APS,              // 18
  NAL_UNIT_PH,                      // 19
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  NAL_UNIT_EOS,                     // 21
  NAL_UNIT_EOB,                     // 22
  NAL_UNIT_PREFIX_SEI,              // 23
  NAL_UNIT_SUFFIX_SEI,              // 24
  NAL_UNIT_FD,                      // 25

  NAL_UNIT_RESERVED_NVCL_26,
  NAL_UNIT_RESERVED_NVCL_27,

  NAL_UNIT_UNSPECIFIED_28,
  NAL_UNIT_UNSPECIFIED_29,
  NAL_UNIT_UNSPECIFIED_30,
  NAL_UNIT_UNSPECIFIED_31,
  NAL_UNIT_INVALID
};

enum PresetMode
{
 NONE      = -1,
 FASTER    = 0,
 FAST      = 1,
 MEDIUM    = 2,
 SLOW      = 3,
 SLOWER    = 4,
 FIRSTPASS = 254,
 TOOLTEST  = 255,
};

enum SegmentMode
{
  SEG_OFF,
  SEG_FIRST,
  SEG_MID,
  SEG_LAST
};

/**
  \ingroup VVEncExternalInterfaces
  \enum ColorFormat
  The enum ColorFormat enumerates supported input color formats.
*/
enum ColorFormat
{
  VVC_CF_INVALID       = -1,             ///< invalid color format
  VVC_CF_YUV420_PLANAR = 0,              ///< YUV420 planar color format
};

static inline ChannelType toChannelType             (const ComponentID id)                         { return ((int)id==(int)COMP_Y)? CH_L : CH_C;                     }
static inline uint32_t    getChannelTypeScaleX      (const ChannelType id, const ChromaFormat fmt) { return ((int)id==(int)COMP_Y || (fmt==CHROMA_444)) ? 0 : 1;     }
static inline uint32_t    getChannelTypeScaleY      (const ChannelType id, const ChromaFormat fmt) { return ((int)id==(int)COMP_Y || (fmt!=CHROMA_420)) ? 0 : 1;     }
static inline uint32_t    getComponentScaleX        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleX(toChannelType(id), fmt);  }
static inline uint32_t    getComponentScaleY        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleY(toChannelType(id), fmt);  }
static inline uint32_t    getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMP;          }
static inline uint32_t    getNumberValidChannels    (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CH;            }
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


} // namespace


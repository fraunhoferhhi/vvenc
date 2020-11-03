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
/** \file     MotionInfo.h
    \brief    motion information handling classes (header)
    \todo     MvField seems to be better to be inherited from Mv
*/

#pragma once

#include "CommonDef.h"
#include "Mv.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for AMVP
struct AMVPInfo
{
  Mv       mvCand[ 7/*AMVP_MAX_NUM_CANDS_MEM*/ ];  ///< array of motion vector predictor candidates
  unsigned numCand;                       ///< number of motion vector predictor candidates
  unsigned available;
};

struct AffineAMVPInfo
{
  Mv       mvCandLT[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for left-top corner
  Mv       mvCandRT[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for right-top corner
  Mv       mvCandLB[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for left-bottom corner
  unsigned numCand;                       ///< number of motion vector predictor candidates
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// class for motion vector with reference index
struct MvField
{
  Mv      mv;
  int16_t refIdx;

  MvField()                                    :            refIdx( NOT_VALID ) {}
  MvField( Mv const & cMv, const int iRefIdx ) : mv( cMv ), refIdx(   iRefIdx ) {}

  void setMvField( Mv const & cMv, const int iRefIdx )
  {
    CHECK( iRefIdx == -1 && cMv != Mv(0,0), "Must not happen." );
    mv     = cMv;
    refIdx = iRefIdx;
  }

  bool operator==( const MvField& other ) const
  {
    CHECK( refIdx == -1 && mv != Mv(0,0), "Error in operator== of MvField." );
    CHECK( other.refIdx == -1 && other.mv != Mv(0,0), "Error in operator== of MvField." );
    return refIdx == other.refIdx && mv == other.mv;
  }
  bool operator!=( const MvField& other ) const
  {
    CHECK( refIdx == -1 && mv != Mv(0,0), "Error in operator!= of MvField." );
    CHECK( other.refIdx == -1 && other.mv != Mv(0,0), "Error in operator!= of MvField." );
    return refIdx != other.refIdx || mv != other.mv;
  }
};

struct MotionInfo
{
  Mv       mv    [NUM_REF_PIC_LIST_01];
  int8_t   refIdx[NUM_REF_PIC_LIST_01] = { NOT_VALID, NOT_VALID };

  uint16_t sliceIdx = 0;
  bool     isInter  = false;
  char     interDir = 0;

  bool operator==( const MotionInfo& mi ) const
  {
    if( isInter  != mi.isInter  ) return false;

    if( sliceIdx != mi.sliceIdx ) return false;
    if( interDir != mi.interDir ) return false;

    if( interDir != 2 )
    {
      if( refIdx[0] != mi.refIdx[0] ) return false;
      if( mv[0]     != mi.mv[0]     ) return false;
    }

    if( interDir != 1 )
    {
      if( refIdx[1] != mi.refIdx[1] ) return false;
      if( mv[1]     != mi.mv[1]     ) return false;
    }

    return true;
  }

  bool operator!=( const MotionInfo& mi ) const
  {
    return !( *this == mi );
  }
};

struct HPMVInfo
{
  Mv       mv    [NUM_REF_PIC_LIST_01];
  int8_t   refIdx[NUM_REF_PIC_LIST_01] = { NOT_VALID, NOT_VALID };

  char     interDir = 0;
  uint8_t  BcwIdx   = 0;
  bool     useAltHpelIf = false;

  HPMVInfo() = default;
  HPMVInfo( const MotionInfo& mi, uint8_t _bcwIdx, bool _useAltHpelIf )
  {
    mv[0] = mi.mv[0];
    mv[1] = mi.mv[1];

    refIdx[0] = mi.refIdx[0];
    refIdx[1] = mi.refIdx[1];

    interDir = mi.interDir;

    BcwIdx       = _bcwIdx;
    useAltHpelIf = _useAltHpelIf;
  }

  bool operator==( const HPMVInfo& mi ) const
  {
    if( interDir != mi.interDir ) return false;

    if( interDir != 2 )
    {
      if( refIdx[0] != mi.refIdx[0] ) return false;
      if( mv[0]     != mi.mv[0]     ) return false;
    }

    if( interDir != 1 )
    {
      if( refIdx[1] != mi.refIdx[1] ) return false;
      if( mv[1]     != mi.mv[1]     ) return false;
    }

    return true;
  }
  
  bool operator==( const MotionInfo& mi ) const
  {
    if( interDir != mi.interDir ) return false;

    if( interDir != 2 )
    {
      if( refIdx[0] != mi.refIdx[0] ) return false;
      if( mv[0]     != mi.mv[0]     ) return false;
    }

    if( interDir != 1 )
    {
      if( refIdx[1] != mi.refIdx[1] ) return false;
      if( mv[1]     != mi.mv[1]     ) return false;
    }

    return true;
  }

  bool operator!=( const HPMVInfo& mi ) const
  {
    return !( *this == mi );
  }

  bool operator!=( const MotionInfo& mi ) const
  {
    return !( *this == mi );
  }
};

struct LutMotionCand
{
  static_vector<HPMVInfo, MAX_NUM_HMVP_CANDS> lut;
};

} // namespace vvenc

//! \}


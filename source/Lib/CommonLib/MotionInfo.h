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
  Mv       mvCand[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of motion vector predictor candidates
  unsigned numCand;                       ///< number of motion vector predictor candidates
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
  bool     isIBCmot = false;
  Mv       bv;

  bool operator==( const MotionInfo& mi ) const
  {
    if( isInter  != mi.isInter  ) return false;
    if( isIBCmot != mi.isIBCmot ) return false;
    if (isInter)
    {
      if (sliceIdx != mi.sliceIdx) return false;
      if (interDir != mi.interDir) return false;

      if (interDir != 2)
      {
        if (refIdx[0] != mi.refIdx[0]) return false;
        if (mv[0] != mi.mv[0]) return false;
      }

      if (interDir != 1)
      {
        if (refIdx[1] != mi.refIdx[1]) return false;
        if (mv[1] != mi.mv[1]) return false;
      }
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
  Mv       bv;

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
    bv           = mi.bv;
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
  static_vector<HPMVInfo, MAX_NUM_HMVP_CANDS> lutIbc;
};

struct IbcBvCand
{
  Mv m_bvCands[IBC_NUM_CANDIDATES];
  int currCnt;
  void resetIbcBvCand()
  {
    for( int i = 0; i < IBC_NUM_CANDIDATES; i++ )
    {
      m_bvCands[ i ].setZero();
    }
    currCnt = 0;
  }
};
} // namespace vvenc

//! \}


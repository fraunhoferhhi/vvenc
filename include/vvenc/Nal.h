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
#pragma once

#include <sstream>
#include <list>
#include "vvenc/vvencDecl.h"
#include "vvenc/vvencConfig.h"

//! \ingroup Interface
//! \{

namespace vvenc {

/**
 * Represents a single NALunit header and the associated RBSPayload
 */
struct VVENC_DECL NALUnit
{
  NalUnitType m_nalUnitType; ///< nal_unit_type
  uint32_t    m_temporalId;  ///< temporal_id
  uint32_t    m_nuhLayerId;  ///< nuh_layer_id
  uint32_t    m_forbiddenZeroBit;
  uint32_t    m_nuhReservedZeroBit;

  NALUnit(const NALUnit &src)
  : m_nalUnitType       (src.m_nalUnitType)
  , m_temporalId        (src.m_temporalId)
  , m_nuhLayerId        (src.m_nuhLayerId)
  , m_forbiddenZeroBit  (src.m_forbiddenZeroBit)
  , m_nuhReservedZeroBit(src.m_nuhReservedZeroBit)

  { }
  /** construct an NALunit structure with given header values. */
  NALUnit(
    NalUnitType nalUnitType,
    uint32_t temporalId = 0,
    uint32_t nuhReservedZeroBit = 0,
    uint32_t forbiddenZeroBit = 0,
    uint32_t nuhLayerId = 0)
    : m_nalUnitType (nalUnitType)
    , m_temporalId  (temporalId)
    , m_nuhLayerId  (nuhLayerId)
    , m_forbiddenZeroBit(forbiddenZeroBit)
    , m_nuhReservedZeroBit(nuhReservedZeroBit)
  {}

  /** default constructor - no initialization; must be performed by user */
  NALUnit() {}

  virtual ~NALUnit() { }

  /** returns true if the NALunit is a slice NALunit */
  bool isSlice()
  {
    return m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL;
  }
  bool isSei()
  {
    return m_nalUnitType == NAL_UNIT_PREFIX_SEI
        || m_nalUnitType == NAL_UNIT_SUFFIX_SEI;
  }

  bool isVcl()
  {
    return m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA
        || m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR;

  }
};

struct OutputNALUnit;

/**
 * A single NALunit, with complete payload in EBSP format.
 */
struct VVENC_DECL NALUnitEBSP : public NALUnit
{
  std::ostringstream m_nalUnitData;

  /**
   * convert the OutputNALUnit nalu into EBSP format by writing out
   * the NALUnit header, then the rbsp_bytes including any
   * emulation_prevention_three_byte symbols.
   */
  NALUnitEBSP(OutputNALUnit& nalu);
};
//! \}
//! \}


/**
 * An AccessUnit is a list of one or more NAL units, according to the
 * working draft.  All NAL units within the object belong to the same
 * access unit.
 *
 * NALUnits held in the AccessUnit list are in EBSP format.  Attempting
 * to insert an OutputNALUnit into the access unit will automatically cause
 * the nalunit to have its headers written and anti-emulation performed.
 *
 * The AccessUnit owns all pointers stored within.  Destroying the
 * AccessUnit will delete all contained objects.
 */
class VVENC_DECL AccessUnit : public std::list<NALUnitEBSP*> // NOTE: Should not inherit from STL.
{
public:
  AccessUnit()
  {
    clearAu();
  }

  ~AccessUnit()
  {
    clearAu();
  }

  void clearAu()
  {
    cts          = 0;
    dts          = 0;
    poc          = 0;
    sliceType     = NUMBER_OF_SLICE_TYPES;
    temporalLayer = 0;
    status        = 0;
    ctsValid      = false;
    dtsValid      = false;
    rap           = false;
    refPic        = false;
    InfoString.clear();

    for (AccessUnit::iterator it = this->begin(); it != this->end(); it++)
    {
      delete *it;
    }
    std::list<NALUnitEBSP*>::clear();
  }

  uint64_t        cts;                                   ///< composition time stamp
  uint64_t        dts;                                   ///< decoding time stamp
  uint64_t        poc;                                   ///< picture order count
  SliceType       sliceType;                              ///< slice type (I/P/B) */
  int             temporalLayer;                          ///< temporal layer
  int             status;
  bool            ctsValid;                               ///< composition time stamp valid flag
  bool            dtsValid;                               ///< decoding time stamp valid flag
  bool            rap;                                    ///< random access point flag
  bool            refPic;                                 ///< reference picture
  std::string     InfoString;
};


static inline const char* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
  case NAL_UNIT_CODED_SLICE_TRAIL:      return "TRAIL";
  case NAL_UNIT_CODED_SLICE_STSA:       return "STSA";
  case NAL_UNIT_CODED_SLICE_RADL:       return "RADL";
  case NAL_UNIT_CODED_SLICE_RASL:       return "RASL";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_GDR:        return "GDR";
  case NAL_UNIT_DCI:                    return "DCI";
  case NAL_UNIT_VPS:                    return "VPS";
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_PREFIX_APS:             return "Prefix APS";
  case NAL_UNIT_SUFFIX_APS:             return "Suffix APS";
  case NAL_UNIT_PH:                     return "PH";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  case NAL_UNIT_FD:                     return "FD";
  default:                              return "UNK";
  }
}


} // namespace vvenc

//! \}


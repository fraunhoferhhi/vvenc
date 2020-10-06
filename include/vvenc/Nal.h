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
#pragma once

#include <sstream>
#include <list>
#include "vvenc/vvencDecl.h"
#include "vvenc/Basics.h"

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
  ~AccessUnit()
  {
    for (AccessUnit::iterator it = this->begin(); it != this->end(); it++)
    {
      delete *it;
    }
  }

  uint64_t        m_uiCts          = 0;                      ///< composition time stamp
  uint64_t        m_uiDts          = 0;                      ///< decoding time stamp
  uint64_t        m_uiPOC          = 0;                      ///< picture order count
  SliceType       m_eSliceType     = NUMBER_OF_SLICE_TYPES;  ///< slice type (I/P/B) */
  int             m_iTemporalLayer = 0;                      ///< temporal layer
  int             m_iStatus        = 0;
  bool            m_bCtsValid      = false;                  ///< composition time stamp valid flag
  bool            m_bDtsValid      = false;                  ///< decoding time stamp valid flag
  bool            m_bRAP           = false;                  ///< random access point flag
  bool            m_bRefPic        = false;                  ///< reference picture
  std::string     m_cInfo;
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


/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Nal.h"
#include <ostream>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

/**
 * A convenience wrapper to NALUnit that also provides a
 * bitstream object.
 */
struct OutputNALUnit : public NALUnit
{
  /**
   * construct an OutputNALunit structure with given header values and
   * storage for a bitstream.  Upon construction the NALunit header is
   * written to the bitstream.
   */
  OutputNALUnit(
    vvencNalUnitType nalUnitType,
    uint32_t temporalID = 0,
    uint32_t reserved_zero_6bits = 0)
  : NALUnit(nalUnitType, temporalID, reserved_zero_6bits)
  , m_Bitstream()
  {}

  OutputNALUnit& operator=(const NALUnit& src)
  {
    m_Bitstream.clear();
    static_cast<NALUnit*>(this)->operator=(src);
    return *this;
  }

  OutputBitstream m_Bitstream;
};

void write(std::ostream& out, OutputNALUnit& nalu);

inline NALUnitEBSP::NALUnitEBSP(OutputNALUnit& nalu)
  : NALUnit(nalu)
{
  write(m_nalUnitData, nalu);
}

} // namespace vvenc

//! \}


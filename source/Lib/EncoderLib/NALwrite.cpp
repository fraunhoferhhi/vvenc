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


#include "NALwrite.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Nal.h"
#include <vector>
#include <algorithm>
#include <ostream>


//! \ingroup EncoderLib
//! \{

namespace vvenc {

static const uint8_t emulation_prevention_three_byte = 3;

void writeNalUnitHeader(std::ostream& out, OutputNALUnit& nalu)       // nal_unit_header()
{
OutputBitstream bsNALUHeader;
  int forbiddenZero = 0;
  bsNALUHeader.write(forbiddenZero, 1);   // forbidden_zero_bit
  int nuhReservedZeroBit = 0;
  bsNALUHeader.write(nuhReservedZeroBit, 1);   // nuh_reserved_zero_bit
  CHECK(nalu.m_nuhLayerId > 63, "nuh_layer_id > 63");
  bsNALUHeader.write(nalu.m_nuhLayerId, 6);       // nuh_layer_id
  bsNALUHeader.write(nalu.m_nalUnitType, 5);      // nal_unit_type
  bsNALUHeader.write(nalu.m_temporalId + 1, 3);   // nuh_temporal_id_plus1

  out.write(reinterpret_cast<const char*>(bsNALUHeader.getByteStream()), bsNALUHeader.getByteStreamLength());
}
/**
 * write nalu to bytestream out, performing RBSP anti startcode
 * emulation as required.  nalu.m_RBSPayload must be byte aligned.
 */
void write(std::ostream& out, OutputNALUnit& nalu)
{
  writeNalUnitHeader(out, nalu);
  /* write out rsbp_byte's, inserting any required
   * emulation_prevention_three_byte's */
  /* 7.4.1 ...
   * emulation_prevention_three_byte is a byte equal to 0x03. When an
   * emulation_prevention_three_byte is present in the NAL unit, it shall be
   * discarded by the decoding process.
   * The last byte of the NAL unit shall not be equal to 0x00.
   * Within the NAL unit, the following three-byte sequences shall not occur at
   * any byte-aligned position:
   *  - 0x000000
   *  - 0x000001
   *  - 0x000002
   * Within the NAL unit, any four-byte sequence that starts with 0x000003
   * other than the following sequences shall not occur at any byte-aligned
   * position:
   *  - 0x00000300
   *  - 0x00000301
   *  - 0x00000302
   *  - 0x00000303
   */
  std::vector<uint8_t>& rbsp   = nalu.m_Bitstream.getFIFO();

  std::vector<uint8_t> outputBuffer;
  outputBuffer.resize(rbsp.size()*2+1); //there can never be enough emulation_prevention_three_bytes to require this much space
  std::size_t outputAmount = 0;
  int         zeroCount    = 0;
  for (std::vector<uint8_t>::iterator it = rbsp.begin(); it != rbsp.end(); it++)
  {
    const uint8_t v=(*it);
    if (zeroCount==2 && v<=3)
    {
      outputBuffer[outputAmount++]=emulation_prevention_three_byte;
      zeroCount=0;
    }

    if (v==0)
    {
      zeroCount++;
    }
    else
    {
      zeroCount=0;
    }
    outputBuffer[outputAmount++]=v;
  }

  /* 7.4.1.1
   * ... when the last byte of the RBSP data is equal to 0x00 (which can
   * only occur when the RBSP ends in a cabac_zero_word), a final byte equal
   * to 0x03 is appended to the end of the data.
   */
  if (zeroCount>0)
  {
    outputBuffer[outputAmount++]=emulation_prevention_three_byte;
  }
  out.write(reinterpret_cast<const char*>(&(*outputBuffer.begin())), outputAmount);
}

} // namespace vvenc

//! \}


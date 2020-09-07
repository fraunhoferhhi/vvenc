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


#include "NALwrite.h"
#include "CommonLib/BitStream.h"
#include <vector>
#include <algorithm>
#include <ostream>
#include "../../../include/vvenc/Nal.h"

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


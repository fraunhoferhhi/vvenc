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


/**
 \file     AnnexBread.cpp
 \brief    reading functions for Annex B byte streams
 */


#include <stdint.h>
#include <vector>
#include "AnnexBread.h"

//! \ingroup DecoderLib
//! \{

namespace vvenc {

/**
 * Parse an AVC AnnexB Bytestream bs to extract a single nalUnit
 * while accumulating bytestream statistics into stats.
 *
 * If EOF occurs while trying to extract a NALunit, an exception
 * of std::ios_base::failure is thrown.  The contsnts of stats will
 * be correct at this point.
 */
static void
_byteStreamNALUnit(
  InputByteStream& bs,
  std::vector<uint8_t>& nalUnit,
  AnnexBStats& stats)
{
  /* At the beginning of the decoding process, the decoder initialises its
   * current position in the byte stream to the beginning of the byte stream.
   * It then extracts and discards each leading_zero_8bits syntax element (if
   * present), moving the current position in the byte stream forward one
   * byte at a time, until the current position in the byte stream is such
   * that the next four bytes in the bitstream form the four-byte sequence
   * 0x00000001.
   */
  while ((bs.eofBeforeNBytes(24/8) || bs.peekBytes(24/8) != 0x000001)
  &&     (bs.eofBeforeNBytes(32/8) || bs.peekBytes(32/8) != 0x00000001))
  {
    uint8_t leading_zero_8bits = bs.readByte();
    if(leading_zero_8bits != 0) { THROW( "Leading zero bits not zero" ); }
    stats.m_numLeadingZero8BitsBytes++;
  }

  /* 1. When the next four bytes in the bitstream form the four-byte sequence
   * 0x00000001, the next byte in the byte stream (which is a zero_byte
   * syntax element) is extracted and discarded and the current position in
   * the byte stream is set equal to the position of the byte following this
   * discarded byte.
   */
  /* NB, the previous step guarantees this will succeed -- if EOF was
   * encountered, an exception will stop execution getting this far */
  if (bs.peekBytes(24/8) != 0x000001)
  {
    uint8_t zero_byte = bs.readByte();
    CHECK( zero_byte != 0, "Zero byte not '0'" );
    stats.m_numZeroByteBytes++;
  }

  /* 2. The next three-byte sequence in the byte stream (which is a
   * start_code_prefix_one_3bytes) is extracted and discarded and the current
   * position in the byte stream is set equal to the position of the byte
   * following this three-byte sequence.
   */
  /* NB, (1) guarantees that the next three bytes are 0x00 00 01 */
  uint32_t start_code_prefix_one_3bytes = bs.readBytes(24/8);
  if(start_code_prefix_one_3bytes != 0x000001) { THROW( "Invalid code prefix" );}
  stats.m_numStartCodePrefixBytes += 3;

  /* 3. NumBytesInNALunit is set equal to the number of bytes starting with
   * the byte at the current position in the byte stream up to and including
   * the last byte that precedes the location of any of the following
   * conditions:
   *   a. A subsequent byte-aligned three-byte sequence equal to 0x000000, or
   *   b. A subsequent byte-aligned three-byte sequence equal to 0x000001, or
   *   c. The end of the byte stream, as determined by unspecified means.
   */
  /* 4. NumBytesInNALunit bytes are removed from the bitstream and the
   * current position in the byte stream is advanced by NumBytesInNALunit
   * bytes. This sequence of bytes is nal_unit( NumBytesInNALunit ) and is
   * decoded using the NAL unit decoding process
   */
  /* NB, (unsigned)x > 2 implies n!=0 && n!=1 */
  while (bs.eofBeforeNBytes(24/8) || bs.peekBytes(24/8) > 2)
  {
    nalUnit.push_back(bs.readByte());
  }

  /* 5. When the current position in the byte stream is:
   *  - not at the end of the byte stream (as determined by unspecified means)
   *  - and the next bytes in the byte stream do not start with a three-byte
   *    sequence equal to 0x000001
   *  - and the next bytes in the byte stream do not start with a four byte
   *    sequence equal to 0x00000001,
   * the decoder extracts and discards each trailing_zero_8bits syntax
   * element, moving the current position in the byte stream forward one byte
   * at a time, until the current position in the byte stream is such that:
   *  - the next bytes in the byte stream form the four-byte sequence
   *    0x00000001 or
   *  - the end of the byte stream has been encountered (as determined by
   *    unspecified means).
   */
  /* NB, (3) guarantees there are at least three bytes available or none */
  while ((bs.eofBeforeNBytes(24/8) || bs.peekBytes(24/8) != 0x000001)
  &&     (bs.eofBeforeNBytes(32/8) || bs.peekBytes(32/8) != 0x00000001))
  {
    uint8_t trailing_zero_8bits = bs.readByte();
    CHECK( trailing_zero_8bits != 0, "Trailing zero bits not '0'" );
    stats.m_numTrailingZero8BitsBytes++;
  }
}

/**
 * Parse an AVC AnnexB Bytestream bs to extract a single nalUnit
 * while accumulating bytestream statistics into stats.
 *
 * Returns false if EOF was reached (NB, nalunit data may be valid),
 *         otherwise true.
 */
bool
byteStreamNALUnit(
  InputByteStream& bs,
  std::vector<uint8_t>& nalUnit,
  AnnexBStats& stats)
{
  bool eof = false;
  try
  {
    _byteStreamNALUnit(bs, nalUnit, stats);
  }
  catch (...)
  {
    eof = true;
  }
  stats.m_numBytesInNALUnit = uint32_t(nalUnit.size());
  return eof;
}

} // namespace vvenc

//! \}


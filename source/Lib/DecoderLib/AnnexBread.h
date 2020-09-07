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
 \file     AnnexBread.h
 \brief    reading functions for Annex B byte streams
 */

#pragma once

#include "CommonLib/CommonDef.h"

#include <stdint.h>
#include <istream>
#include <vector>

//! \ingroup DecoderLib
//! \{

namespace vvenc {

class InputByteStream
{
public:
  /**
   * Create a bytestream reader that will extract bytes from
   * istream.
   *
   * NB, it isn't safe to access istream while in use by a
   * InputByteStream.
   *
   * Side-effects: the exception mask of istream is set to eofbit
   */
  InputByteStream(std::istream& istream)
  : m_NumFutureBytes(0)
  , m_FutureBytes(0)
  , m_Input(istream)
  {
    istream.exceptions(std::istream::eofbit | std::istream::badbit);
  }

  /**
   * Reset the internal state.  Must be called if input stream is
   * modified externally to this class
   */
  void reset()
  {
    m_NumFutureBytes = 0;
    m_FutureBytes = 0;
  }

  /**
   * returns true if an EOF will be encountered within the next
   * n bytes.
   */
  bool eofBeforeNBytes(uint32_t n)
  {
    CHECK(n > 4, "Unsupported look-ahead value");
    if (m_NumFutureBytes >= n)
    {
      return false;
    }

    n -= m_NumFutureBytes;
    try
    {
      for (uint32_t i = 0; i < n; i++)
      {
        m_FutureBytes = (m_FutureBytes << 8) | m_Input.get();
        m_NumFutureBytes++;
      }
    }
    catch (...)
    {
      return true;
    }
    return false;
  }

  /**
   * return the next n bytes in the stream without advancing
   * the stream pointer.
   *
   * Returns: an unsigned integer representing an n byte bigendian
   * word.
   *
   * If an attempt is made to read past EOF, an n-byte word is
   * returned, but the portion that required input bytes beyond EOF
   * is undefined.
   *
   */
  uint32_t peekBytes(uint32_t n)
  {
    eofBeforeNBytes(n);
    return m_FutureBytes >> 8*(m_NumFutureBytes - n);
  }

  /**
   * consume and return one byte from the input.
   *
   * If bytestream is already at EOF prior to a call to readByte(),
   * an exception std::ios_base::failure is thrown.
   */
  uint8_t readByte()
  {
    if (!m_NumFutureBytes)
    {
      uint8_t byte = m_Input.get();
      return byte;
    }
    m_NumFutureBytes--;
    uint8_t wanted_byte = m_FutureBytes >> 8*m_NumFutureBytes;
    m_FutureBytes &= ~(0xff << 8*m_NumFutureBytes);
    return wanted_byte;
  }

  /**
   * consume and return n bytes from the input.  n bytes from
   * bytestream are interpreted as bigendian when assembling
   * the return value.
   */
  uint32_t readBytes(uint32_t n)
  {
    uint32_t val = 0;
    for (uint32_t i = 0; i < n; i++)
    {
      val = (val << 8) | readByte();
    }
    return val;
  }

private:
  uint32_t m_NumFutureBytes; /* number of valid bytes in m_FutureBytes */
  uint32_t m_FutureBytes; /* bytes that have been peeked */
  std::istream& m_Input; /* Input stream to read from */
};

/**
 * Statistics associated with AnnexB bytestreams
 */
struct AnnexBStats
{
  uint32_t m_numLeadingZero8BitsBytes;
  uint32_t m_numZeroByteBytes;
  uint32_t m_numStartCodePrefixBytes;
  uint32_t m_numBytesInNALUnit;
  uint32_t m_numTrailingZero8BitsBytes;

  AnnexBStats& operator+=(const AnnexBStats& rhs)
  {
    m_numLeadingZero8BitsBytes += rhs.m_numLeadingZero8BitsBytes;
    m_numZeroByteBytes += rhs.m_numZeroByteBytes;
    m_numStartCodePrefixBytes += rhs.m_numStartCodePrefixBytes;
    m_numBytesInNALUnit += rhs.m_numBytesInNALUnit;
    m_numTrailingZero8BitsBytes += rhs.m_numTrailingZero8BitsBytes;
    return *this;
  }
};

bool byteStreamNALUnit(InputByteStream& bs, std::vector<uint8_t>& nalUnit, AnnexBStats& stats);

} // namespace vvenc

//! \}


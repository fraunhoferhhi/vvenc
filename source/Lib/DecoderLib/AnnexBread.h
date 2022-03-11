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


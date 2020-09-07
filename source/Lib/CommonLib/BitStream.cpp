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


/** \file     BitStream.cpp
    \brief    class for handling bitstream
*/

#include "BitStream.h"

#include <stdint.h>
#include <vector>
#include <string.h>
#include <memory.h>

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

OutputBitstream::OutputBitstream()
{
  clear();
}

OutputBitstream::~OutputBitstream()
{
}


InputBitstream::InputBitstream()
: m_fifo()
, m_emulationPreventionByteLocation()
, m_fifo_idx(0)
, m_num_held_bits(0)
, m_held_bits(0)
, m_numBitsRead(0)
{ }

InputBitstream::InputBitstream(const InputBitstream &src)
: m_fifo(src.m_fifo)
, m_emulationPreventionByteLocation(src.m_emulationPreventionByteLocation)
, m_fifo_idx(src.m_fifo_idx)
, m_num_held_bits(src.m_num_held_bits)
, m_held_bits(src.m_held_bits)
, m_numBitsRead(src.m_numBitsRead)
{ }

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void InputBitstream::resetToStart()
{
  m_fifo_idx=0;
  m_num_held_bits=0;
  m_held_bits=0;
  m_numBitsRead=0;
}

uint8_t* OutputBitstream::getByteStream() const
{
  return (uint8_t*) &m_fifo.front();
}

uint32_t OutputBitstream::getByteStreamLength()
{
  return uint32_t(m_fifo.size());
}

void OutputBitstream::clear()
{
  m_fifo.clear();
  m_held_bits = 0;
  m_num_held_bits = 0;
}

void OutputBitstream::write   ( uint32_t uiBits, uint32_t uiNumberOfBits )
{
  CHECK( uiNumberOfBits > 32, "Number of bits is exceeds '32'" );
  CHECK( uiNumberOfBits != 32 && (uiBits & (~0 << uiNumberOfBits)) != 0, "Unsupported parameters" );

  /* any modulo 8 remainder of num_total_bits cannot be written this time,
   * and will be held until next time. */
  uint32_t num_total_bits = uiNumberOfBits + m_num_held_bits;
  uint32_t next_num_held_bits = num_total_bits % 8;

  /* form a byte aligned word (write_bits), by concatenating any held bits
   * with the new bits, discarding the bits that will form the next_held_bits.
   * eg: H = held bits, V = n new bits        /---- next_held_bits
   * len(H)=7, len(V)=1: ... ---- HHHH HHHV . 0000 0000, next_num_held_bits=0
   * len(H)=7, len(V)=2: ... ---- HHHH HHHV . V000 0000, next_num_held_bits=1
   * if total_bits < 8, the value of v_ is not used */
  uint8_t next_held_bits = uiBits << (8 - next_num_held_bits);

  if (!(num_total_bits >> 3))
  {
    /* insufficient bits accumulated to write out, append new_held_bits to
     * current held_bits */
    /* NB, this requires that v only contains 0 in bit positions {31..n} */
    m_held_bits |= next_held_bits;
    m_num_held_bits = next_num_held_bits;
    return;
  }

  /* topword serves to justify held_bits to align with the msb of uiBits */
  uint32_t topword = (uiNumberOfBits - next_num_held_bits) & ~((1 << 3) -1);
  uint32_t write_bits = (m_held_bits << topword) | (uiBits >> next_num_held_bits);

  switch (num_total_bits >> 3)
  {
  case 4: m_fifo.push_back(write_bits >> 24);
  case 3: m_fifo.push_back(write_bits >> 16);
  case 2: m_fifo.push_back(write_bits >> 8);
  case 1: m_fifo.push_back(write_bits);
  }

  m_held_bits = next_held_bits;
  m_num_held_bits = next_num_held_bits;
}

void OutputBitstream::writeAlignOne()
{
  uint32_t num_bits = getNumBitsUntilByteAligned();
  write((1 << num_bits) - 1, num_bits);
  return;
}

void OutputBitstream::writeAlignZero()
{
  if (0 == m_num_held_bits)
  {
    return;
  }
  m_fifo.push_back(m_held_bits);
  m_held_bits = 0;
  m_num_held_bits = 0;
}

/**
 - add substream to the end of the current bitstream
 .
 \param  pcSubstream  substream to be added
 */
void OutputBitstream::addSubstream( const OutputBitstream* pcSubstream )
{
  uint32_t uiNumBits = pcSubstream->getNumberOfWrittenBits();

  const std::vector<uint8_t>& rbsp = pcSubstream->getFIFO();
  for (std::vector<uint8_t>::const_iterator it = rbsp.begin(); it != rbsp.end();)
  {
    write(*it++, 8);
  }
  if (uiNumBits&0x7)
  {
    write(pcSubstream->getHeldBits()>>(8-(uiNumBits&0x7)), uiNumBits&0x7);
  }
}

void OutputBitstream::writeByteAlignment()
{
  write( 1, 1);
  writeAlignZero();
}

int OutputBitstream::countStartCodeEmulations()
{
  uint32_t cnt = 0;
  std::vector<uint8_t>& rbsp   = getFIFO();
  for (std::vector<uint8_t>::iterator it = rbsp.begin(); it != rbsp.end();)
  {
    std::vector<uint8_t>::iterator found = it;
    do
    {
      // find the next emulated 00 00 {00,01,02,03}
      // NB, end()-1, prevents finding a trailing two byte sequence
      found = search_n(found, rbsp.end()-1, 2, 0);
      found++;
      // if not found, found == end, otherwise found = second zero byte
      if (found == rbsp.end())
      {
        break;
      }
      if (*(++found) <= 3)
      {
        break;
      }
    } while (true);
    it = found;
    if (found != rbsp.end())
    {
      cnt++;
    }
  }
  return cnt;
}

/**
 * read uiNumberOfBits from bitstream without updating the bitstream
 * state, storing the result in ruiBits.
 *
 * If reading uiNumberOfBits would overrun the bitstream buffer,
 * the bitstream is effectively padded with sufficient zero-bits to
 * avoid the overrun.
 */
void InputBitstream::pseudoRead ( uint32_t uiNumberOfBits, uint32_t& ruiBits )
{
  uint32_t saved_num_held_bits = m_num_held_bits;
  uint8_t saved_held_bits = m_held_bits;
  uint32_t saved_fifo_idx = m_fifo_idx;

  uint32_t num_bits_to_read = std::min(uiNumberOfBits, getNumBitsLeft());
  read(num_bits_to_read, ruiBits);
  ruiBits <<= (uiNumberOfBits - num_bits_to_read);

  m_fifo_idx = saved_fifo_idx;
  m_held_bits = saved_held_bits;
  m_num_held_bits = saved_num_held_bits;
}


void InputBitstream::read (uint32_t uiNumberOfBits, uint32_t& ruiBits)
{
  CHECK( uiNumberOfBits > 32, "Too many bits read" );

  m_numBitsRead += uiNumberOfBits;

  /* NB, bits are extracted from the MSB of each byte. */
  uint32_t retval = 0;
  if (uiNumberOfBits <= m_num_held_bits)
  {
    /* n=1, len(H)=7:   -VHH HHHH, shift_down=6, mask=0xfe
     * n=3, len(H)=7:   -VVV HHHH, shift_down=4, mask=0xf8
     */
    retval = m_held_bits >> (m_num_held_bits - uiNumberOfBits);
    retval &= ~(0xff << uiNumberOfBits);
    m_num_held_bits -= uiNumberOfBits;
    ruiBits = retval;
    return;
  }

  /* all num_held_bits will go into retval
   *   => need to mask leftover bits from previous extractions
   *   => align retval with top of extracted word */
  /* n=5, len(H)=3: ---- -VVV, mask=0x07, shift_up=5-3=2,
   * n=9, len(H)=3: ---- -VVV, mask=0x07, shift_up=9-3=6 */
  uiNumberOfBits -= m_num_held_bits;
  retval = m_held_bits & ~(0xff << m_num_held_bits);
  retval <<= uiNumberOfBits;

  /* number of whole bytes that need to be loaded to form retval */
  /* n=32, len(H)=0, load 4bytes, shift_down=0
   * n=32, len(H)=1, load 4bytes, shift_down=1
   * n=31, len(H)=1, load 4bytes, shift_down=1+1
   * n=8,  len(H)=0, load 1byte,  shift_down=0
   * n=8,  len(H)=3, load 1byte,  shift_down=3
   * n=5,  len(H)=1, load 1byte,  shift_down=1+3
   */
  uint32_t aligned_word = 0;
  uint32_t num_bytes_to_load = (uiNumberOfBits - 1) >> 3;
  CHECK(m_fifo_idx + num_bytes_to_load >= m_fifo.size(), "Exceeded FIFO size");

  switch (num_bytes_to_load)
  {
  case 3: aligned_word  = m_fifo[m_fifo_idx++] << 24;
  case 2: aligned_word |= m_fifo[m_fifo_idx++] << 16;
  case 1: aligned_word |= m_fifo[m_fifo_idx++] <<  8;
  case 0: aligned_word |= m_fifo[m_fifo_idx++];
  }

  /* resolve remainder bits */
  uint32_t next_num_held_bits = (32 - uiNumberOfBits) % 8;

  /* copy required part of aligned_word into retval */
  retval |= aligned_word >> next_num_held_bits;

  /* store held bits */
  m_num_held_bits = next_num_held_bits;
  m_held_bits = aligned_word;

  ruiBits = retval;
}

/**
 * insert the contents of the bytealigned (and flushed) bitstream src
 * into this at byte position pos.
 */
void OutputBitstream::insertAt(const OutputBitstream& src, uint32_t pos)
{
  CHECK(0 != src.getNumberOfWrittenBits() % 8, "Number of written bits is not a multiple of 8");

  std::vector<uint8_t>::iterator at = m_fifo.begin() + pos;
  m_fifo.insert(at, src.m_fifo.begin(), src.m_fifo.end());
}

uint32_t InputBitstream::readOutTrailingBits ()
{
  uint32_t count=0;
  uint32_t uiBits = 0;

  while ( ( getNumBitsLeft() > 0 ) && (getNumBitsUntilByteAligned()!=0) )
  {
    count++;
    read ( 1, uiBits );
  }
  return count;
}

/**
 Extract substream from the current bitstream.

 \param  uiNumBits    number of bits to transfer
 */
InputBitstream *InputBitstream::extractSubstream( uint32_t uiNumBits )
{
  uint32_t uiNumBytes = uiNumBits/8;
  InputBitstream *pResult = new InputBitstream;

  std::vector<uint8_t> &buf = pResult->getFifo();
  buf.reserve((uiNumBits+7)>>3);

  if (m_num_held_bits == 0)
  {
    std::size_t currentOutputBufferSize=buf.size();
    const uint32_t uiNumBytesToReadFromFifo = std::min<uint32_t>(uiNumBytes, (uint32_t)m_fifo.size() - m_fifo_idx);
    buf.resize(currentOutputBufferSize+uiNumBytes);
    memcpy(&(buf[currentOutputBufferSize]), &(m_fifo[m_fifo_idx]), uiNumBytesToReadFromFifo); m_fifo_idx+=uiNumBytesToReadFromFifo;
    if (uiNumBytesToReadFromFifo != uiNumBytes)
    {
      memset(&(buf[currentOutputBufferSize+uiNumBytesToReadFromFifo]), 0, uiNumBytes - uiNumBytesToReadFromFifo);
    }
  }
  else
  {
    for (uint32_t ui = 0; ui < uiNumBytes; ui++)
    {
      uint32_t uiByte;
      read(8, uiByte);
      buf.push_back(uiByte);
    }
  }
  if (uiNumBits&0x7)
  {
    uint32_t uiByte = 0;
    read(uiNumBits&0x7, uiByte);
    uiByte <<= 8-(uiNumBits&0x7);
    buf.push_back(uiByte);
  }
  return pResult;
}

uint32_t InputBitstream::readByteAlignment()
{
  uint32_t code = 0;
  read( 1, code );
  CHECK(code != 1, "Code is not '1'");

  uint32_t numBits = getNumBitsUntilByteAligned();
  if(numBits)
  {
    CHECK(numBits > getNumBitsLeft(), "More bits available than left");
    read( numBits, code );
    CHECK(code != 0, "Code not '0'");
  }
  return numBits+1;
}

} // namespace vvenc

//! \}


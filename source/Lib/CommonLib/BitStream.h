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
/** \file     BitStream.h
    \brief    class for handling bitstream (header)
*/

#pragma once

#include "CommonDef.h"
#include <stdint.h>
#include <vector>

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================
/**
 * Model of a writable bitstream that accumulates bits to produce a
 * bytestream.
 */
class OutputBitstream
{
  /**
   * FIFO for storage of bytes.  Use:
   *  - fifo.push_back(x) to append words
   *  - fifo.clear() to empty the FIFO
   *  - &fifo.front() to get a pointer to the data array.
   *    NB, this pointer is only valid until the next push_back()/clear()
   */
  std::vector<uint8_t> m_fifo;

  uint32_t m_num_held_bits; /// number of bits not flushed to bytestream.
  uint8_t  m_held_bits; /// the bits held and not flushed to bytestream.
                             /// this value is always msb-aligned, bigendian.
public:
  // create / destroy
  OutputBitstream();
  ~OutputBitstream();

  // interface for encoding
  /**
   * append uiNumberOfBits least significant bits of uiBits to
   * the current bitstream
   */
  void        write           ( uint32_t uiBits, uint32_t uiNumberOfBits );

  /** insert one bits until the bitstream is byte-aligned */
  void        writeAlignOne   ();

  /** insert zero bits until the bitstream is byte-aligned */
  void        writeAlignZero  ();

  // utility functions

  /**
   * Return a pointer to the start of the byte-stream buffer.
   * Pointer is valid until the next write/flush/reset call.
   * NB, data is arranged such that subsequent bytes in the
   * bytestream are stored in ascending addresses.
   */
  uint8_t* getByteStream() const;

  /**
   * Return the number of valid bytes available from  getByteStream()
   */
  uint32_t getByteStreamLength();

  /**
   * Reset all internal state.
   */
  void clear();

  /**
   * returns the number of bits that need to be written to
   * achieve byte alignment.
   */
  int getNumBitsUntilByteAligned() const { return (8 - m_num_held_bits) & 0x7; }

  /**
   * Return the number of bits that have been written since the last clear()
   */
  uint32_t getNumberOfWrittenBits() const { return uint32_t(m_fifo.size()) * 8 + m_num_held_bits; }

  void insertAt(const OutputBitstream& src, uint32_t pos);

  /**
   * Return a reference to the internal fifo
   */
  std::vector<uint8_t>& getFIFO() { return m_fifo; }

  uint8_t getHeldBits() const { return m_held_bits; }

  //OutputBitstream& operator= (const OutputBitstream& src);
  /** Return a reference to the internal fifo */
  const std::vector<uint8_t>& getFIFO() const { return m_fifo; }

  void addSubstream( const OutputBitstream* pcSubstream );
  void writeByteAlignment();

  //! returns the number of start code emulations contained in the current buffer
  int countStartCodeEmulations();
};

/**
 * Model of an input bitstream that extracts bits from a predefined
 * bytestream.
 */
class InputBitstream
{
protected:
  std::vector<uint8_t> m_fifo; /// FIFO for storage of complete bytes
  std::vector<uint32_t>    m_emulationPreventionByteLocation;

  uint32_t m_fifo_idx; /// Read index into m_fifo

  uint32_t m_num_held_bits;
  uint8_t m_held_bits;
  uint32_t  m_numBitsRead;

public:
  /**
   * Create a new bitstream reader object that reads from buf.
   */
  InputBitstream();
  virtual ~InputBitstream() { }
  InputBitstream(const InputBitstream &src);

  void resetToStart();

  // interface for decoding
  void        pseudoRead      ( uint32_t uiNumberOfBits, uint32_t& ruiBits );
  void        read            ( uint32_t uiNumberOfBits, uint32_t& ruiBits );
  void        readByte        ( uint32_t &ruiBits )
  {
    CHECK( m_fifo_idx >= m_fifo.size(), "FIFO exceeded" );
    ruiBits = m_fifo[m_fifo_idx++];
#if ENABLE_TRACING
    m_numBitsRead += 8;
#endif
  }

  void        peekPreviousByte( uint32_t &byte )
  {
    CHECK( m_fifo_idx == 0, "FIFO empty" );
    byte = m_fifo[m_fifo_idx - 1];
  }

  uint32_t readOutTrailingBits ();
  uint8_t  getHeldBits() const { return m_held_bits; }
  OutputBitstream& operator= (const OutputBitstream& src);
  uint32_t getByteLocation              ( )                     { return m_fifo_idx                    ; }

  // Peek at bits in word-storage. Used in determining if we have completed reading of current bitstream and therefore slice in LCEC.
  uint32_t peekBits (uint32_t uiBits) { uint32_t tmp; pseudoRead(uiBits, tmp); return tmp; }

  // utility functions
  uint32_t read(uint32_t numberOfBits)      { uint32_t tmp; read(numberOfBits, tmp); return tmp; }
  uint32_t readByte()                   { uint32_t tmp; readByte( tmp ); return tmp; }
  uint32_t getNumBitsUntilByteAligned() { return m_num_held_bits & (0x7); }
  uint32_t getNumBitsLeft()             { return 8*((uint32_t)m_fifo.size() - m_fifo_idx) + m_num_held_bits; }
  InputBitstream *extractSubstream( uint32_t uiNumBits ); // Read the nominated number of bits, and return as a bitstream.
  uint32_t getNumBitsRead()            { return m_numBitsRead; }
  uint32_t readByteAlignment();

  void     pushEmulationPreventionByteLocation ( uint32_t pos )                         { m_emulationPreventionByteLocation.push_back( pos ); }
  uint32_t numEmulationPreventionBytesRead     ()                                   { return (uint32_t) m_emulationPreventionByteLocation.size();    }
  const std::vector<uint32_t> &getEmulationPreventionByteLocation  () const              { return m_emulationPreventionByteLocation;           }
  uint32_t getEmulationPreventionByteLocation  ( uint32_t idx )                         { return m_emulationPreventionByteLocation[ idx ];    }
  void     clearEmulationPreventionByteLocation()                                   { m_emulationPreventionByteLocation.clear();          }
  void     setEmulationPreventionByteLocation  ( const std::vector<uint32_t> &vec )     { m_emulationPreventionByteLocation = vec;            }

  const std::vector<uint8_t> &getFifo() const { return m_fifo; }
        std::vector<uint8_t> &getFifo()       { return m_fifo; }
};

} // namespace vvenc

//! \}


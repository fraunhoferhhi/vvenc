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


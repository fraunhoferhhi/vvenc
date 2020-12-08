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
/**
  \ingroup VVEncoderApp
  \file    BinFileWriter.h
  \brief   This file contains the interface for class BinFileWriter.
  \author  hinz@hhi.fraunhofer.de
  \date    2/19/2013
*/

#pragma once

#include <fstream>
#include "vvenc/vvenc.h"

namespace vvcutilities {

/**
  \ingroup HEVCUtilitiesExternalInterfaces
  The BinFileWriter class.
*/
class BinFileWriter
{
public:
  /// Constructor
  BinFileWriter(){}

  /// Destructor
  virtual ~BinFileWriter() {}

  bool isOpen()
  {
    return m_cOS.is_open() ? true : false;
  }

  /**
    This method opens a file.
    \param[in]  pcFilename pointer to Filename.
    \retval     int, nonzero if failed
		\pre        None
		\post       None
  */
  int open( const char* pcFilename )
  {
    // if there is some stream open just shut it down
    m_cOS.close(); 
    m_cOS.open( pcFilename, std::ios::out | std::ios::binary | std::ios::trunc);
    return m_cOS.is_open() ? 0 : -1;
  }

  /**
    This method opens a file.
    \param[in]  rcAccessUnit reference to Accessunit.
    \retval     int, nonzero if failed
    \pre        None
    \post       None
  */
  int writeAU( const vvenc::VvcAccessUnit& rcAccessUnit )
  {
    if( rcAccessUnit.m_iUsedSize == 0 )
    {
      return 0;
    }

    size_t lBefore = m_cOS.tellp();
    m_cOS.write( (const char*)rcAccessUnit.m_pucBuffer, rcAccessUnit.m_iUsedSize );
    // check if this read was okay
    return (int)(lBefore + rcAccessUnit.m_iUsedSize - m_cOS.tellp());
  }

  /**
    Close the opened file
  */
  void close()
  {
    if( m_cOS.is_open() )
    {
      m_cOS.close();
    }
  }

private:
  std::ofstream m_cOS;
};

} // namespace


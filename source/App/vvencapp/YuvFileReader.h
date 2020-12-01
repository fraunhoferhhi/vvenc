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
  \ingroup VVCUtilitiesExternalInterfaces
  \file    YuvFileReader.h
  \brief   This file contains the interface for class YuvFileReader.
  \author  hinz@hhi.fraunhofer.de
  \date    2/19/2013
*/

#pragma once

#include "vvenc/vvenc.h"
#include <fstream>
#include <cassert>

// this function pointer takes the exported encoder function expCopyUnPack10BitBlk
void (*extCopyUnPack10BitBlk)      ( short* pDes, const int iStride, const int iWidth, const int iHeight, const unsigned char* pSrc ) = NULL;

namespace vvcutilities {

/**
  \ingroup VVCUtilitiesExternalInterfaces
*/
class YuvFileReader
{
public:
  /// Constructor
  YuvFileReader()
  {
    m_cReadBuffer.m_pucDeletePicBuffer = nullptr;
  }

  /// Destructor
  virtual ~YuvFileReader()
  {
    if( m_bInitialized )
    {
      close();
    }
  };

public:
  /**
    This method prints a hello world like string.
    This method is simply used for testing of the build process.
    It prints a text string and waits for you to press a key.
    \param[in]  rcString string that will be output.
    \retval     Void
		\pre        None
		\post       None
  */
  int open( const char* pcFilename, const int iFileBitDepth, const int iDestBitDepth, const int iWidth, const int iHeight,
            int iRepeatTimes = 0, bool bPacked = false )
  {
    if( m_bInitialized ){ assert( !m_bInitialized ); return -1; }

    m_iFileBitDepth = (iFileBitDepth>10) ? 10 : iFileBitDepth;
    m_iDestBitDepth = iDestBitDepth;
    m_iPicSize = (1+!!(iFileBitDepth>8))* iWidth * iHeight * 3 / 2;
    m_iReadWidth = iWidth;
    m_iWidth = iWidth;
    m_iHeight = iHeight;
    m_iRepeatTimes = iRepeatTimes;

    m_bPacked = bPacked;

    if( m_bPacked )
    {
      m_iReadWidth = m_iWidth*10/16;
      m_iPicSize = 2*m_iReadWidth * iHeight * 3 / 2;
      int iRet = allocBuffer( m_cReadBuffer );
      m_cReadBuffer.m_iWidth = m_iReadWidth;
      m_cReadBuffer.m_iStride = m_iReadWidth;
      if( iRet ) { return iRet; }
    }

    // delete buffer if any
    if( m_psBuffer )
    {
      delete [] m_psBuffer;
      m_psBuffer = nullptr;
    }

    // allocate some conversion memory in case we need some
    if( ! ( iFileBitDepth == iDestBitDepth && iDestBitDepth > 8 ) )
    {
      m_psBuffer = new short[iWidth + 16];
    }

    // if there is some stream open just shut it down
    m_cIS.close(); 
    m_cIS.open( pcFilename, std::ios::in | std::ios::binary );
    int iRet = m_cIS.is_open() ? 0 : -1;
    m_bInitialized = true;
    return iRet;
  }

  int close()
  {
    if( !m_bInitialized ){ assert( m_bInitialized ); return -1; }

    delete [] m_psBuffer;
    delete [] m_cReadBuffer.m_pucDeletePicBuffer;
    m_cIS.close();

    m_bInitialized = false;
    return 0;
  }

  template< class PicBufferLocal >
  int allocBuffer( PicBufferLocal& rcPicBuffer )
  {
    if( !m_bInitialized ){ assert( m_bInitialized ); return -1; }

    const int iSizeFactor     = 2;
    const int iAlignmentGuard =16;
    rcPicBuffer.m_iBitDepth = m_iDestBitDepth;
    rcPicBuffer.m_iWidth    = m_iWidth;
    rcPicBuffer.m_iHeight   = m_iHeight;
    rcPicBuffer.m_iStride   = m_iWidth;
    int iLumaSize   = m_iWidth * m_iHeight;
    const int iBufSize = iSizeFactor * iLumaSize * 3 / 2 + 3*iAlignmentGuard;

    rcPicBuffer.m_pucDeletePicBuffer = new (std::nothrow) unsigned char[ iBufSize ];
    if( NULL == rcPicBuffer.m_pucDeletePicBuffer )
    {
      return vvenc::VVENC_NOT_ENOUGH_MEM;
    }

    unsigned char* pY = rcPicBuffer.m_pucDeletePicBuffer + iSizeFactor * ( 0 );
    unsigned char* pU = rcPicBuffer.m_pucDeletePicBuffer + iSizeFactor * ( iLumaSize);
    unsigned char* pV = rcPicBuffer.m_pucDeletePicBuffer + iSizeFactor * ( 5*iLumaSize/4);

    rcPicBuffer.m_pvY = (pY +   iAlignmentGuard) - (((size_t)pY) & (iAlignmentGuard-1));
    rcPicBuffer.m_pvU = (pU + 2*iAlignmentGuard) - (((size_t)pU) & (iAlignmentGuard-1));
    rcPicBuffer.m_pvV = (pV + 3*iAlignmentGuard) - (((size_t)pV) & (iAlignmentGuard-1));

    return 0;
  }

  int skipFrames( unsigned int uiNumFrames )
  {
    if( !m_bInitialized ){ assert( m_bInitialized ); return -1; }

    for( unsigned int ui = 0; ui < uiNumFrames; ui++ )
    m_cIS.seekg(m_iPicSize, m_cIS.cur );

    return 0;
  }


  virtual int readPicture( vvenc::PicBuffer& rcPicBuffer )
  {
    if( !m_bInitialized ){ assert( m_bInitialized ); return -1; }

    if( ! m_cIS.is_open() )
    {
      return -1;
    }

    vvenc::PicBuffer& rcReadBuffer = m_bPacked ? m_cReadBuffer : rcPicBuffer;

    int iRet = xReadPicture( rcReadBuffer );
    if( iRet != 0 && m_iRepeatTimes )
    {
      m_iRepeatTimes--;
      m_cIS.clear();
      m_cIS.seekg(0, m_cIS.beg);
      // try once again
      iRet = xReadPicture( rcReadBuffer );
    }

    if( m_bPacked )
    {
      xConvertPicture( rcPicBuffer, rcReadBuffer );
    }

    return iRet;
  }

protected:

  virtual void xConvertPicture( vvenc::PicBuffer& rcOut, const vvenc::PicBuffer& rcIn)
  {
    extCopyUnPack10BitBlk((short*)rcOut.m_pvY, rcOut.m_iStride, rcOut.m_iWidth, rcOut.m_iHeight, (unsigned char*)rcIn.m_pvY);
    extCopyUnPack10BitBlk((short*)rcOut.m_pvU, rcOut.m_iStride >> 1, rcOut.m_iWidth >> 1, rcOut.m_iHeight >> 1, (unsigned char*)rcIn.m_pvU);
    extCopyUnPack10BitBlk((short*)rcOut.m_pvV, rcOut.m_iStride >> 1, rcOut.m_iWidth >> 1, rcOut.m_iHeight >> 1, (unsigned char*)rcIn.m_pvV);
  }

private:

  int xReadPicture( vvenc::PicBuffer& rcPicBuffer )
  {
    // let's focus on 420
    const int iCWidth  = m_iReadWidth>>1;
    const int iCHeight = m_iHeight>>1;
    const int iCStride = rcPicBuffer.m_iStride>>1;
    assert( rcPicBuffer.m_iBitDepth == m_iDestBitDepth );

    rcPicBuffer.m_iWidth = m_iWidth;
    rcPicBuffer.m_iHeight = m_iHeight;

    const size_t lBefore = m_cIS.tellg();
    xReadPlane( rcPicBuffer.m_pvY, rcPicBuffer.m_iStride, m_iReadWidth, m_iHeight );
    xReadPlane( rcPicBuffer.m_pvU, iCStride, iCWidth, iCHeight );
    xReadPlane( rcPicBuffer.m_pvV, iCStride, iCWidth, iCHeight );
    // check if this read was okay
    const size_t lAfter = m_cIS.tellg();
    return (int)(lBefore + m_iPicSize - lAfter);
  }

  void xReadPlane( void* pvBuffer, const int iStride, const int iWidth, const int iHeight )
  {
    if( m_iFileBitDepth == m_iDestBitDepth )
    {
      if( m_iFileBitDepth > 8 )
      {
        const int iFactor = 2;
        if( iStride == iWidth )
        {
          const int iReadSize = iWidth * iHeight * iFactor;
          m_cIS.read( (char*)pvBuffer, iReadSize);
        }
        else
        {
          char* pc = (char*)pvBuffer;
          const int iReadSize = iFactor * iWidth;
          const int iLineOffset = iStride * iFactor; 
          for( int y = 0; y < iHeight; y++)
          {
            m_cIS.read( pc, iReadSize );
            pc += iLineOffset;
          }
        }
      }
      else
      {
        short* ps = (short*)pvBuffer;
        unsigned char* pucTemp = (unsigned char*) m_psBuffer;
        for( int y = 0; y < iHeight; y++)
        {
          m_cIS.read( (char*)pucTemp, iWidth );
          for( int x = 0; x < iWidth; x++)
          {
            ps[x] = pucTemp[x];
          }
          ps += iStride;
        }
      }
    }
    else if( m_iFileBitDepth == 8 )
    {
      const int iShift = m_iDestBitDepth - 8;
      short* ps = (short*)pvBuffer;
      unsigned char* pucTemp = (unsigned char*) m_psBuffer;
      for( int y = 0; y < iHeight; y++)
      {
        m_cIS.read( (char*)pucTemp, iWidth );
        for( int x = 0; x < iWidth; x++)
        {
          ps[x] = pucTemp[x] << iShift;
        }
        ps += iStride;
      }
    }
    else if( m_iDestBitDepth == 8 )
    {
      const int iShift = m_iFileBitDepth - 8;
      const short sAdd = 1<<(iShift-1);
      short* ps = (short*)pvBuffer;
      short* psTemp = m_psBuffer;
      for( int y = 0; y < iHeight; y++)
      {
        m_cIS.read( (char*)psTemp, 2*iWidth );
        for( int x = 0; x < iWidth; x++)
        {
          ps[x] = (psTemp[x] + sAdd) >> iShift;
        }
        ps += iStride;
      }
    }
    else if( m_iDestBitDepth > m_iFileBitDepth )
    {
      const int iShift = m_iDestBitDepth - m_iFileBitDepth;
      short* ps = (short*)pvBuffer;
      short* psTemp = m_psBuffer;
      for( int y = 0; y < iHeight; y++)
      {
        m_cIS.read( (char*)psTemp, 2*iWidth );
        for( int x = 0; x < iWidth; x++)
        {
          ps[x] = psTemp[x] << iShift;
        }
        ps += iStride;
      }
    }
    else if( m_iFileBitDepth > m_iDestBitDepth )
    {
      const int iShift = m_iFileBitDepth - m_iDestBitDepth;
      const short sAdd = 1<<(iShift-1);
      short* ps = (short*)pvBuffer;
      short* psTemp = m_psBuffer;
      for( int y = 0; y < iHeight; y++)
      {
        m_cIS.read( (char*)psTemp, 2*iWidth );
        for( int x = 0; x < iWidth; x++)
        {
          ps[x] = (psTemp[x] + sAdd) >> iShift;
        }
        ps += iStride;
      }
    }
  }

protected:

  bool m_bInitialized = false;
  short* m_psBuffer   = nullptr;
  int m_iFileBitDepth = 10;
  int m_iDestBitDepth = 10;
  int m_iReadWidth    = 0;
  int m_iWidth        = 0;
  int m_iHeight       = 0;
  int m_iRepeatTimes  = 0;
  bool m_bPacked      = false;
  size_t m_iPicSize   = 0;
  std::ifstream m_cIS;
  vvenc::PicBuffer m_cReadBuffer;
};

} // namespace


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
      int iRet = allocBuffer( m_cReadBuffer, true );
      m_cReadBuffer.m_iWidth = m_iReadWidth;
      m_cReadBuffer.m_iStride = m_iReadWidth;
      if( iRet ) { return iRet; }
    }

    // delete buffer if any
    delete [] m_psBuffer;
    m_psBuffer = NULL;

    // allocate some conversion memory in case we need some
    if( iFileBitDepth != iDestBitDepth )
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
  int allocBuffer( PicBufferLocal& rcPicBuffer, bool bFrameBuffer )
  {
    if( !m_bInitialized ){ assert( m_bInitialized ); return -1; }

    const int iSizeFactor     = (m_iDestBitDepth>8) ? 2 : 1;
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
      const int iFactor = 1 + !!(m_iFileBitDepth > 8);
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
      unsigned char* puc = (unsigned char*)pvBuffer;
      short* psTemp = m_psBuffer;
      for( int y = 0; y < iHeight; y++)
      {
        m_cIS.read( (char*)psTemp, 2*iWidth );
        for( int x = 0; x < iWidth; x++)
        {
          puc[x] = (psTemp[x] + sAdd) >> iShift;
        }
        puc += iStride;
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


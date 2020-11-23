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


/** \file     EncoderIf.cpp
    \brief    encoder lib interface
*/

#include "vvenc/EncoderIf.h"

#include "EncoderLib/EncLib.h"
#include "CommonLib/CommonDef.h"
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_TRAFO
#include "CommonLib/TrQuant_EMT.h"
#endif

//! \ingroup Interface
//! \{

namespace vvenc {

bool tryDecodePicture( Picture* pic, const int expectedPoc, const std::string& bitstreamFileName, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>* apsMap, bool bDecodeUntilPocFound = false, int debugPOC = -1, bool copyToEnc = true );

// ====================================================================================================================

EncoderIf::EncoderIf()
  : m_pEncLib( nullptr )
{
}

EncoderIf::~EncoderIf()
{
  destroyEncoderLib();
}

void EncoderIf::createEncoderLib( const EncCfg& encCfg, YUVWriterIf* yuvWriterIf )
{
  CHECK( m_pEncLib != nullptr, "encoder library already initialized" );
  m_pEncLib = new EncLib;
  m_pEncLib->init( encCfg, yuvWriterIf );
}

void EncoderIf::destroyEncoderLib()
{
  if ( m_pEncLib )
  {
    m_pEncLib->destroy();
    delete m_pEncLib;
    m_pEncLib = nullptr;
  }
}

void EncoderIf::encodePicture( bool flush, const YUVBuffer& yuvInBuf, AccessUnit& au, bool& isQueueEmpty )
{
  CHECK( m_pEncLib == nullptr, "encoder library not initialized" );
  m_pEncLib->encodePicture( flush, yuvInBuf, au, isQueueEmpty );
}

void EncoderIf::printSummary()
{
  CHECK( m_pEncLib == nullptr, "encoder library not initialized" );
  m_pEncLib->printSummary();
}

// ====================================================================================================================

void setMsgFnc( MsgFnc msgFnc )
{
  g_msgFnc = msgFnc;
}

std::string setSIMDExtension( const std::string& simdId )
{
  std::string ret = "NA";
#if ENABLE_SIMD_OPT
#ifdef TARGET_SIMD_X86
  const char* simdSet = read_x86_extension( simdId );
  ret = simdSet;
#endif
  g_pelBufOP.initPelBufOpsX86();
#endif
#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.initTCoeffOpsX86();
#endif
  return ret;
}

bool isTracingEnabled()
{
#if ENABLE_TRACING
  return true;
#else
  return false;
#endif
}

std::string getCompileInfoString()
{
  char convBuf[ 256 ];
  std::string compileInfo;
  snprintf( convBuf, sizeof( convBuf ), NVM_ONOS );      compileInfo += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_COMPILEDBY); compileInfo += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_BITS );      compileInfo += convBuf;
  return compileInfo;
}

void decodeBitstream( const std::string& FileName)
{
  FFwdDecoder ffwdDecoder; 
  Picture cPicture; cPicture.poc=-8000;

  if( tryDecodePicture( &cPicture, -1, FileName, ffwdDecoder, nullptr, false, cPicture.poc, false ))
  {
    msg( ERROR, "decoding failed");
    THROW("error decoding");
  }
}

} // namespace vvenc

//! \}


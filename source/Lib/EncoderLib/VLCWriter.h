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
/** \file     VLCWriter.h
 *  \brief    Writer for high level syntax
 */

#pragma once

#include "CABACWriter.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Slice.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

struct GeneralHrdParams;
struct OlsHrdParams;

#if ENABLE_TRACING

#define WRITE_SCODE( value, length, name)   xWriteSCodeTr ( value, length, name )
#define WRITE_CODE( value, length, name)    xWriteCodeTr ( value, length, name )
#define WRITE_UVLC( value,         name)    xWriteUvlcTr ( value,         name )
#define WRITE_SVLC( value,         name)    xWriteSvlcTr ( value,         name )
#define WRITE_FLAG( value,         name)    xWriteFlagTr ( value,         name )

extern bool g_HLSTraceEnable;
#else
#define WRITE_SCODE( value, length, name)    xWriteSCode ( value, length )
#define WRITE_CODE( value, length, name)     xWriteCode ( value, length )
#define WRITE_UVLC( value,         name)     xWriteUvlc ( value )
#define WRITE_SVLC( value,         name)     xWriteSvlc ( value )
#define WRITE_FLAG( value,         name)     xWriteFlag ( value )

#endif


class VLCWriter
{
protected:

  OutputBitstream*    m_pcBitIf;

  VLCWriter() : m_pcBitIf(NULL) {}
  virtual ~VLCWriter() {}

  void  setBitstream          ( OutputBitstream* p )  { m_pcBitIf = p;  }
  void  xWriteSCode           ( int  code,  uint32_t length );
  void  xWriteCode            ( uint32_t uiCode, uint32_t uiLength );
  void  xWriteUvlc            ( uint32_t uiCode );
  void  xWriteSvlc            ( int  iCode   );
  void  xWriteFlag            ( bool flag );
#if ENABLE_TRACING
  void  xWriteSCodeTr         ( int value,  uint32_t  length, const char *pSymbolName);
  void  xWriteCodeTr          ( uint32_t value, uint32_t  length, const char *pSymbolName);
  void  xWriteUvlcTr          ( uint32_t value,               const char *pSymbolName);
  void  xWriteSvlcTr          ( int  value,                   const char *pSymbolName);
  void  xWriteFlagTr          ( bool flag,                    const char *pSymbolName);
#endif
  void  xWriteRbspTrailingBits();
  bool isByteAligned()      { return (m_pcBitIf->getNumBitsUntilByteAligned() == 0); } ;
};


class HLSWriter : private VLCWriter
{
public:
  HLSWriter() {}
  virtual ~HLSWriter() {}

  void  setBitstream            ( OutputBitstream* p )  { m_pcBitIf = p;  }
  uint32_t  getNumberOfWrittenBits  ()                      { return m_pcBitIf->getNumberOfWrittenBits();  }
  void  codeVUI                 ( const VUI *pcVUI, const SPS* pcSPS );
  void  codeSPS                 ( const SPS* pcSPS );
  void  codePPS                 ( const PPS* pcPPS, const SPS* pcSPS );
  void  codeAPS                 ( const APS* pcAPS );
  void  codeAlfAps              ( const APS* pcAPS );
  void  codeLmcsAps             ( const APS* aps );
  void  codeVPS                 ( const VPS* pcVPS );
  void  codeDCI                 ( const DCI* dci );
  void  codePictureHeader       ( const PicHeader* picHeader, bool writeRbspTrailingBits );
  void  codeSliceHeader         ( const Slice* slice );
  void  codeConstraintInfo      ( const ConstraintInfo* cinfo );
  void  codeProfileTierLevel    ( const ProfileTierLevel* ptl, bool profileTierPresent, int maxNumSubLayersMinus1 );
  void  codeOlsHrdParameters    ( const GeneralHrdParams * generalHrd, const OlsHrdParams *olsHrd , const uint32_t firstSubLayer, const uint32_t maxNumSubLayersMinus1);

  void codeGeneralHrdparameters ( const GeneralHrdParams *hrd);
  void  codeAUD                 ( const int audIrapOrGdrAuFlag, const int pictureType );
  void  codeTilesWPPEntryPoint  ( Slice* pSlice );

  void alfFilter                ( const AlfParam& alfParam, const bool isChroma, const int altIdx );

private:
  void dpb_parameters           ( int maxSubLayersMinus1, bool subLayerInfoFlag, const SPS *pcSPS);
  void xCodeRefPicList          ( const ReferencePictureList* rpl, bool isLongTermPresent, uint32_t ltLsbBitsCount, const bool isForbiddenZeroDeltaPoc, int rplIdx );
  void xCodePredWeightTable     ( const PicHeader *picHeader, const PPS *pps, const SPS *sps );
  void xCodePredWeightTable     ( const Slice* slice );
};

} // namespace vvenc

//! \}
  

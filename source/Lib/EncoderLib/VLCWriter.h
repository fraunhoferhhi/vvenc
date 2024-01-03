/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
  

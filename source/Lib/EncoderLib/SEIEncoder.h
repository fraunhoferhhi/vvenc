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

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
#pragma once

#include "CommonLib/SEI.h"
#include "CommonLib/Unit.h"

#include <deque>
//! \ingroup EncoderLib
//! \{

namespace vvenc {

// forward declarations
class EncHRD;

struct DUData
{
  DUData() : accumBitsDU(0), accumNalsDU(0) {};

  int accumBitsDU;
  int accumNalsDU;
};

//! Initializes different SEI message types based on given encoder configuration parameters
class SEIEncoder
{
public:
  SEIEncoder()
    : m_pcEncCfg      ( nullptr )
    , m_pcEncHRD      ( nullptr )
    , m_isInitialized ( false )
    , m_rapWithLeading( false )
  {};
  virtual ~SEIEncoder(){};

  void init( const VVEncCfg& encCfg, EncHRD& encHRD);
  void initDecodedPictureHashSEI  ( SEIDecodedPictureHash& dphSei, const CPelUnitBuf& pic, std::string &rHashString, const BitDepths &bitDepths);

  void initBufferingPeriodSEI     ( SEIBufferingPeriod& bpSei, bool noLeadingPictures);
  void initPictureTimingSEI       ( SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, const Slice *slice, const uint32_t numDU, const bool bpPresentInAU);
  void initDrapSEI                ( SEIDependentRAPIndication& drapSei) {};

  void initSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics *seiAltTransCharacteristics);
  void initSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume *seiMDCV);
  void initSEIContentLightLevel(SEIContentLightLevelInfo *seiCLL);

private:
  const VVEncCfg* m_pcEncCfg;
  EncHRD*         m_pcEncHRD;
  bool            m_isInitialized;
  bool            m_rapWithLeading;
  uint32_t        m_lastBPSEI[VVENC_MAX_TLAYER];
  uint32_t        m_totalCoded[VVENC_MAX_TLAYER];
};

} // namespace vvenc

//! \}


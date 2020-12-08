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
 \file     SEIread.h
 \brief    reading funtionality for SEI messages
 */

#pragma once

#include "CommonLib/SEI.h"
#include "CommonLib/HRD.h"

//! \ingroup DecoderLib
//! \{

namespace vvenc {

class InputBitstream;


class SEIReader: public VLCReader
{
public:
  SEIReader() {};
  virtual ~SEIReader() {};
  void parseSEImessage(InputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId,const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);

protected:
  void xReadSEImessage                        (SEIMessages& seis, const NalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIuserDataUnregistered          (SEIuserDataUnregistered &sei,          uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDecodingUnitInfo              (SEIDecodingUnitInfo& sei,              uint32_t payloadSize, const SEIBufferingPeriod& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDecodedPictureHash            (SEIDecodedPictureHash& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIBufferingPeriod               (SEIBufferingPeriod& sei,               uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIPictureTiming                 (SEIPictureTiming& sei,                 uint32_t payloadSize, const uint32_t temporalId, const SEIBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIScalableNesting               (SEIScalableNesting& sei, const NalUnitType nalUnitType, const uint32_t nuhLayerId, uint32_t payloadSize, const VPS *vps, const SPS *sps, std::ostream *decodedMessageOutputStream);
  void xCheckScalableNestingConstraints       (const SEIScalableNesting& sei, const NalUnitType nalUnitType, const VPS* vps);
  void xParseSEIFrameFieldinfo                (SEIFrameFieldInfo& sei, const SEIPictureTiming& pt, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream);
  void xParseSEIDependentRAPIndication        (SEIDependentRAPIndication& sei,        uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFramePacking                  (SEIFramePacking& sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIParameterSetsInclusionIndication(SEIParameterSetsInclusionIndication& sei, uint32_t payloadSize,                std::ostream* pDecodedMessageOutputStream);
  void xParseSEIMasteringDisplayColourVolume  (SEIMasteringDisplayColourVolume& sei,  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics& sei,              uint32_t payLoadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIEquirectangularProjection     (SEIEquirectangularProjection &sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISphereRotation                (SEISphereRotation &sei,                uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIOmniViewport                  (SEIOmniViewport& sei,                  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIRegionWisePacking             (SEIRegionWisePacking& sei,             uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIGeneralizedCubemapProjection  (SEIGeneralizedCubemapProjection &sei,  uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISubpictureLevelInfo           (SEISubpicureLevelInfo& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEISampleAspectRatioInfo         (SEISampleAspectRatioInfo& sei,         uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIUserDataRegistered            (SEIUserDataRegistered& sei,            uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIFilmGrainCharacteristics      (SEIFilmGrainCharacteristics& sei,      uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentLightLevelInfo         (SEIContentLightLevelInfo& sei,         uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIAmbientViewingEnvironment     (SEIAmbientViewingEnvironment& sei,     uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);
  void xParseSEIContentColourVolume           (SEIContentColourVolume& sei,           uint32_t payloadSize,                     std::ostream *pDecodedMessageOutputStream);

  void sei_read_scode (std::ostream *pOS, uint32_t length,           int& code, const char *pSymbolName);
  void sei_read_code  (std::ostream *pOS, uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_uvlc  (std::ostream *pOS,                    uint32_t& ruiCode, const char *pSymbolName);
  void sei_read_svlc  (std::ostream *pOS,                        int&  ruiCode, const char *pSymbolName);
  void sei_read_flag  (std::ostream *pOS,                    uint32_t& ruiCode, const char *pSymbolName);

protected:
  HRD m_nestedHrd;
};

} // namespace vvenc

//! \}


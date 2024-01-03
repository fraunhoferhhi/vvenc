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
#pragma once

#include "VLCWriter.h"
#include "CommonLib/SEI.h"
#include "CommonLib/HRD.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class OutputBitstream;

//! \ingroup EncoderLib
//! \{
class SEIWriter : public VLCWriter
{
public:
  SEIWriter() {};
  virtual ~SEIWriter() {};

  void writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, HRD &hrd, bool isNested, const uint32_t temporalId);

protected:
  void xWriteSEIuserDataUnregistered(const SEIuserDataUnregistered &sei);
  void xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SEIBufferingPeriod& bp, const uint32_t temporalId);
  void xWriteSEIDecodedPictureHash(const SEIDecodedPictureHash& sei);
  void xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei);
  void xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SEIBufferingPeriod& bp, const uint32_t temporalId);
  void xWriteSEIFrameFieldInfo(const SEIFrameFieldInfo& sei);
  void xWriteSEIDependentRAPIndication(const SEIDependentRAPIndication& sei);
  void xWriteSEIScalableNesting(OutputBitstream& bs, const SEIScalableNesting& sei);
  void xWriteSEIFramePacking(const SEIFramePacking& sei);
  void xWriteSEIParameterSetsInclusionIndication(const SEIParameterSetsInclusionIndication& sei);
  void xWriteSEIMasteringDisplayColourVolume( const SEIMasteringDisplayColourVolume& sei);
  void xWriteSEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei);
  void xWriteSEIEquirectangularProjection         (const SEIEquirectangularProjection &sei);
  void xWriteSEISphereRotation                    (const SEISphereRotation &sei);
  void xWriteSEIOmniViewport                      (const SEIOmniViewport& sei);
  void xWriteSEIRegionWisePacking                 (const SEIRegionWisePacking &sei);
  void xWriteSEIGeneralizedCubemapProjection      (const SEIGeneralizedCubemapProjection &sei);
  void xWriteSEISubpictureLevelInfo               (const SEISubpicureLevelInfo &sei);
  void xWriteSEISampleAspectRatioInfo             (const SEISampleAspectRatioInfo &sei);

  void xWriteSEIUserDataRegistered(const SEIUserDataRegistered& sei);
  void xWriteSEIFilmGrainCharacteristics(const SEIFilmGrainCharacteristics& sei);
  void xWriteSEIContentLightLevelInfo(const SEIContentLightLevelInfo& sei);
  void xWriteSEIAmbientViewingEnvironment(const SEIAmbientViewingEnvironment& sei);
  void xWriteSEIContentColourVolume(const SEIContentColourVolume &sei);
  void xWriteSEIpayloadData(OutputBitstream &bs, const SEI& sei, HRD &hrd, const uint32_t temporalId);
  void xWriteByteAlign();
protected:
  HRD m_nestingHrd;
};

} // namespace vvenc

//! \}


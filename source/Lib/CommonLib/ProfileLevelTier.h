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

#include "CommonLib/CommonDef.h"
#include <stdint.h>

namespace vvenc {

struct SPS; // Forward declaration.

struct LevelTierFeatures
{
  vvencLevel    level;
  uint32_t      maxLumaPs;
  uint32_t      maxCpb[VVENC_NUMBER_OF_TIERS];    // in units of CpbVclFactor or CpbNalFactor bits
  uint32_t      maxSlicesPerAu;
  uint32_t      maxTilesPerAu;
  uint32_t      maxTileCols;
  uint32_t      maxTileRows;
  uint64_t      maxLumaSr;
  uint32_t      maxBr[VVENC_NUMBER_OF_TIERS];     // in units of BrVclFactor or BrNalFactor bits/s
  uint32_t      minCrBase[VVENC_NUMBER_OF_TIERS];
  uint32_t      getMaxPicWidthInLumaSamples()  const;
  uint32_t      getMaxPicHeightInLumaSamples() const;

  static vvencLevel getLevelForInput( uint32_t width, uint32_t height, bool tier, int temporalRate, int temporalScale, int bitrate );
  static void getMaxTileColsRowsPerLevel( vvencLevel level, uint32_t &maxCols, uint32_t &maxRows );
};


struct ProfileFeatures
{
  vvencProfile             profile;
  const char              *pNameString;
  uint32_t                 maxBitDepth;
  ChromaFormat             maxChromaFormat;

  bool                     canUseLevel15p5;
  uint32_t                 cpbVclFactor;
  uint32_t                 cpbNalFactor;
  uint32_t                 formatCapabilityFactorx1000;
  uint32_t                 minCrScaleFactorx100;
  const LevelTierFeatures *pLevelTiersListInfo;
  bool                     onePictureOnlyFlagMustBe1;

  static const ProfileFeatures *getProfileFeatures(const vvencProfile p);
};


class ProfileLevelTierFeatures
{
  private:
    const ProfileFeatures   *m_pProfile;
    const LevelTierFeatures *m_pLevelTier;
    vvencTier                m_tier;
  public:
    ProfileLevelTierFeatures() : m_pProfile(nullptr), m_pLevelTier(nullptr), m_tier(VVENC_TIER_MAIN) {}

    void extractPTLInformation(const SPS &sps);

    const ProfileFeatures     *getProfileFeatures()   const { return m_pProfile; }
    const LevelTierFeatures   *getLevelTierFeatures() const { return m_pLevelTier; }
    vvencTier                  getTier()              const { return m_tier; }
    uint64_t getCpbSizeInBits()                       const;
    double getMinCr()                                 const;
    uint32_t getMaxDpbSize( uint32_t picSizeMaxInSamplesY ) const;
};


} // namespace

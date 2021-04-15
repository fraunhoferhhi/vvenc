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

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur F�rderung der angewandten Forschung e.V.
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

#include "CommonLib/CommonDef.h"
#include <stdint.h>

namespace vvenc {

struct SPS; // Forward declaration.

struct LevelTierFeatures
{
  Level        level;
  uint32_t      maxLumaPs;
  uint32_t      maxCpb[NUMBER_OF_TIERS];    // in units of CpbVclFactor or CpbNalFactor bits
  uint32_t      maxSlicesPerAu;
  uint32_t      maxTilesPerAu;
  uint32_t      maxTileCols;
  uint64_t      maxLumaSr;
  uint32_t      maxBr[NUMBER_OF_TIERS];     // in units of BrVclFactor or BrNalFactor bits/s
  uint32_t      minCrBase[NUMBER_OF_TIERS];
  uint32_t      getMaxPicWidthInLumaSamples()  const;
  uint32_t      getMaxPicHeightInLumaSamples() const;

  static Level getLevelForInput( uint32_t width, uint32_t height );
};


struct ProfileFeatures
{
  Profile                  profile;
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

  static const ProfileFeatures *getProfileFeatures(const Profile p);
};


class ProfileLevelTierFeatures
{
  private:
    const ProfileFeatures   *m_pProfile;
    const LevelTierFeatures *m_pLevelTier;
    Tier                     m_tier;
  public:
    ProfileLevelTierFeatures() : m_pProfile(nullptr), m_pLevelTier(nullptr), m_tier(Tier::TIER_MAIN) {}

    void extractPTLInformation(const SPS &sps);

    const ProfileFeatures     *getProfileFeatures()   const { return m_pProfile; }
    const LevelTierFeatures   *getLevelTierFeatures() const { return m_pLevelTier; }
    Tier                       getTier()              const { return m_tier; }
    uint64_t getCpbSizeInBits()                       const;
    double getMinCr()                                 const;
    uint32_t getMaxDpbSize( uint32_t picSizeMaxInSamplesY ) const;
};


} // namespace

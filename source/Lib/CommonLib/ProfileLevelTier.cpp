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

/** \file     ProfileLevelTier.cpp
    \brief    Handle profile, level and tier information.
*/


#include "ProfileLevelTier.h"
#include "CommonLib/Slice.h"
#include <math.h>

namespace vvenc {

uint32_t LevelTierFeatures::getMaxPicWidthInLumaSamples()  const
{
  return uint32_t(sqrt(maxLumaPs*8.0));
}

uint32_t LevelTierFeatures::getMaxPicHeightInLumaSamples() const
{
  return uint32_t(sqrt(maxLumaPs*8.0));
}

static const uint64_t MAX_CNFUINT64 = std::numeric_limits<uint64_t>::max();

static const LevelTierFeatures mainLevelTierInfo[] =
{
      //  level,       maxlumaps,      maxcpb[tier],,  maxSlicesPerAu,maxTilesPerAu,cols,rows, maxLumaSr,       maxBr[tier],,    minCr[tier],,
    { VVENC_LEVEL1  ,    36864, {      350,        0 },       16,        1,          1,    1, 552960ULL,     {     128,        0 }, { 2, 2} },
    { VVENC_LEVEL2  ,   122880, {     1500,        0 },       16,        1,          1,    1, 3686400ULL,    {    1500,        0 }, { 2, 2} },
    { VVENC_LEVEL2_1,   245760, {     3000,        0 },       20,        1,          1,    1, 7372800ULL,    {    3000,        0 }, { 2, 2} },
    { VVENC_LEVEL3  ,   552960, {     6000,        0 },       30,        4,          2,    2, 16588800ULL,   {    6000,        0 }, { 2, 2} },
    { VVENC_LEVEL3_1,   983040, {    10000,        0 },       40,        9,          3,    3, 33177600ULL,   {   10000,        0 }, { 2, 2} },
    { VVENC_LEVEL4  ,  2228224, {    12000,    30000 },       75,       25,          5,    5, 66846720ULL,   {   12000,    30000 }, { 4, 4} },
    { VVENC_LEVEL4_1,  2228224, {    20000,    50000 },       75,       25,          5,    5, 133693440ULL,  {   20000,    50000 }, { 4, 4} },
    { VVENC_LEVEL5  ,  8912896, {    25000,   100000 },      200,      110,         10,   11, 267386880ULL,  {   25000,   100000 }, { 6, 4} },
    { VVENC_LEVEL5_1,  8912896, {    40000,   160000 },      200,      110,         10,   11, 534773760ULL,  {   40000,   160000 }, { 8, 4} },
    { VVENC_LEVEL5_2,  8912896, {    60000,   240000 },      200,      110,         10,   11, 1069547520ULL, {   60000,   240000 }, { 8, 4} },
    { VVENC_LEVEL6  , 35651584, {    80000,   240000 },      600,      440,         20,   22, 1069547520ULL, {   60000,   240000 }, { 8, 4} },
    { VVENC_LEVEL6_1, 35651584, {   120000,   480000 },      600,      440,         20,   22, 2139095040ULL, {  120000,   480000 }, { 8, 4} },
    { VVENC_LEVEL6_2, 35651584, {   180000,   800000 },      600,      440,         20,   22, 4278190080ULL, {  240000,   800000 }, { 8, 4} },
    { VVENC_LEVEL6_3, 80216064, {   240000,   800000 },     1000,      990,         30,   33, 4812963840ULL, {  320000,   800000 }, { 8, 4} },
    { VVENC_LEVEL15_5, MAX_UINT,{ MAX_UINT, MAX_UINT }, MAX_UINT, MAX_UINT, MAX_UINT, MAX_UINT, MAX_CNFUINT64, {MAX_UINT, MAX_UINT }, { 0, 0} },
    { VVENC_LEVEL_AUTO    }
};

static const ProfileFeatures validProfiles[] = {
// profile, pNameString, maxBitDepth, maxChrFmt, lvl15.5, cpbvcl, cpbnal, fcf*1000, mincr*100, levelInfo
// most constrained profiles must appear first.
  { VVENC_MAIN_10_STILL_PICTURE, "Main_10_Still_Picture", 10, CHROMA_420, true, 1000, 1100, 1875, 100,
    mainLevelTierInfo, true },
  { VVENC_MULTILAYER_MAIN_10_STILL_PICTURE, "Multilayer_Main_10_Still_Picture", 10, CHROMA_420, true, 1000, 1100,
    1875, 100, mainLevelTierInfo, true },
  { VVENC_MAIN_10_444_STILL_PICTURE, "Main_444_10_Still_Picture", 10, CHROMA_444, true, 2500, 2750, 3750, 75,
    mainLevelTierInfo, true },
  { VVENC_MULTILAYER_MAIN_10_444_STILL_PICTURE, "Multilayer_Main_444_10_Still_Picture", 10, CHROMA_444, true, 2500,
    2750, 3750, 75, mainLevelTierInfo, true },
  { VVENC_MAIN_10, "Main_10", 10, CHROMA_420, false, 1000, 1100, 1875, 100, mainLevelTierInfo, false },
  { VVENC_MULTILAYER_MAIN_10, "Multilayer_Main_10", 10, CHROMA_420, false, 1000, 1100, 1875, 100, mainLevelTierInfo,
    false },
  { VVENC_MAIN_10_444, "Main_444_10", 10, CHROMA_444, false, 2500, 2750, 3750, 75, mainLevelTierInfo, false },
  { VVENC_MULTILAYER_MAIN_10_444, "Multilayer_Main_444_10", 10, CHROMA_444, false, 2500, 2750, 3750, 75,
    mainLevelTierInfo, false },
  { VVENC_PROFILE_AUTO, 0 }
};

const ProfileFeatures *ProfileFeatures::getProfileFeatures(const vvencProfile p)
{
  int i;
  for (i = 0; validProfiles[i].profile != VVENC_PROFILE_AUTO; i++)
  {
    if (validProfiles[i].profile == p)
    {
      return &validProfiles[i];
    }
  }

  return &validProfiles[i];
}

vvencLevel LevelTierFeatures::getMaxLevel(vvencProfile profile)
{
  const ProfileFeatures* pf = ProfileFeatures::getProfileFeatures( profile );
  vvencLevel maxLevel = ( pf && pf->canUseLevel15p5 ) ? vvencLevel::VVENC_LEVEL15_5 : vvencLevel::VVENC_LEVEL6_3;
  return maxLevel;
}

vvencLevel LevelTierFeatures::getLevelForInput( uint32_t width, uint32_t height, bool tier, int temporalRate, int temporalScale, int bitrate )
{
  uint64_t samplesPerSec = ((uint64_t)temporalRate*(uint64_t)width*(uint64_t)height)/temporalScale;
  uint32_t br = ( uint32_t ) bitrate / 1000; // don't assume bitrate if not set, ignore the condition since usually always satisfied

  for (const auto& info: mainLevelTierInfo )
  {
    if ( width <= info.getMaxPicWidthInLumaSamples() &&
        height <=  info.getMaxPicHeightInLumaSamples() &&
        samplesPerSec <=  info.maxLumaSr &&
        br <= info.maxBr[tier?1:0] &&
        info.level != VVENC_LEVEL_AUTO )
    {
      return info.level;
    }
  }

  return VVENC_NUMBER_OF_LEVELS;
}

void LevelTierFeatures::getMaxTileColsRowsPerLevel( vvencLevel level, uint32_t &maxCols, uint32_t &maxRows )
{
  for (const auto& info: mainLevelTierInfo )
  {
    if( level == info.level )
    {
      maxCols = info.maxTileCols;
      maxRows = info.maxTileRows;
      return;
    }
  }

  maxCols = MAX_TILE_COLS;
  maxRows = MAX_TILES / MAX_TILE_COLS;
}

void ProfileLevelTierFeatures::extractPTLInformation(const SPS &sps)
{
  const ProfileTierLevel &spsPtl = sps.profileTierLevel;

  m_pProfile = nullptr;
  m_pLevelTier = nullptr;
  m_tier = spsPtl.tierFlag;

  // Identify the profile from the profile Idc, and possibly other constraints.
  for(int32_t i=0; validProfiles[i].profile != VVENC_PROFILE_AUTO; i++)
  {
    if (spsPtl.profileIdc == validProfiles[i].profile)
    {
      m_pProfile = &(validProfiles[i]);
      break;
    }
  }

  if (m_pProfile != nullptr)
  {
    // Now identify the level:
    const LevelTierFeatures *pLTF = m_pProfile->pLevelTiersListInfo;
    const vvencLevel spsLevelName = spsPtl.levelIdc;
    if (spsLevelName!=VVENC_LEVEL15_5 || m_pProfile->canUseLevel15p5)
    {
      for(int i=0; pLTF[i].level!=VVENC_LEVEL_AUTO; i++)
      {
        if (pLTF[i].level == spsLevelName)
        {
          m_pLevelTier = &(pLTF[i]);
          break;
        }
      }
    }
  }
}

double ProfileLevelTierFeatures::getMinCr() const
{
  return (m_pLevelTier!=0 && m_pProfile!=0) ? (m_pProfile->minCrScaleFactorx100 * m_pLevelTier->minCrBase[m_tier?1:0])/100.0 : 0.0 ;
}

uint64_t ProfileLevelTierFeatures::getCpbSizeInBits() const
{
  return (m_pLevelTier!=0 && m_pProfile!=0) ? uint64_t(m_pProfile->cpbVclFactor) * m_pLevelTier->maxCpb[m_tier?1:0] : uint64_t(0);
}

uint32_t ProfileLevelTierFeatures::getMaxDpbSize( uint32_t picSizeMaxInSamplesY ) const
{
  const uint32_t maxDpbPicBuf = 8;
  uint32_t       maxDpbSize;

  if (m_pLevelTier->level == VVENC_LEVEL15_5)
  {
    // maxDpbSize is unconstrained in this case
    maxDpbSize = std::numeric_limits<uint32_t>::max();
  }
  else if (2 * picSizeMaxInSamplesY <= m_pLevelTier->maxLumaPs)
  {
    maxDpbSize = 2 * maxDpbPicBuf;
  }
  else if (3 * picSizeMaxInSamplesY <= 2 * m_pLevelTier->maxLumaPs)
  {
    maxDpbSize = 3 * maxDpbPicBuf / 2;
  }
  else
  {
    maxDpbSize = maxDpbPicBuf;
  }

  return maxDpbSize;
}

} //namespace

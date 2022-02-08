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


/** \file     Contexts.cpp
 *  \brief    Classes providing probability descriptions and contexts (also contains context initialization values)
 */

#include "Contexts.h"

#include <algorithm>
#include <cstring>

//! \ingroup CommonLib
//! \{

namespace vvenc {

const uint8_t ProbModelTables::m_RenormTable_32[32] =
{
  6,  5,  4,  4,
  3,  3,  3,  3,
  2,  2,  2,  2,
  2,  2,  2,  2,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1
};

const BinFracBits ProbModelTables::m_binFracBits[256] = {
  { { 0x0005c, 0x48000 } }, { { 0x00116, 0x3b520 } }, { { 0x001d0, 0x356cb } }, { { 0x0028b, 0x318a9 } },
  { { 0x00346, 0x2ea40 } }, { { 0x00403, 0x2c531 } }, { { 0x004c0, 0x2a658 } }, { { 0x0057e, 0x28beb } },
  { { 0x0063c, 0x274ce } }, { { 0x006fc, 0x26044 } }, { { 0x007bc, 0x24dc9 } }, { { 0x0087d, 0x23cfc } },
  { { 0x0093f, 0x22d96 } }, { { 0x00a01, 0x21f60 } }, { { 0x00ac4, 0x2122e } }, { { 0x00b89, 0x205dd } },
  { { 0x00c4e, 0x1fa51 } }, { { 0x00d13, 0x1ef74 } }, { { 0x00dda, 0x1e531 } }, { { 0x00ea2, 0x1db78 } },
  { { 0x00f6a, 0x1d23c } }, { { 0x01033, 0x1c970 } }, { { 0x010fd, 0x1c10b } }, { { 0x011c8, 0x1b903 } },
  { { 0x01294, 0x1b151 } }, { { 0x01360, 0x1a9ee } }, { { 0x0142e, 0x1a2d4 } }, { { 0x014fc, 0x19bfc } },
  { { 0x015cc, 0x19564 } }, { { 0x0169c, 0x18f06 } }, { { 0x0176d, 0x188de } }, { { 0x0183f, 0x182e8 } },
  { { 0x01912, 0x17d23 } }, { { 0x019e6, 0x1778a } }, { { 0x01abb, 0x1721c } }, { { 0x01b91, 0x16cd5 } },
  { { 0x01c68, 0x167b4 } }, { { 0x01d40, 0x162b6 } }, { { 0x01e19, 0x15dda } }, { { 0x01ef3, 0x1591e } },
  { { 0x01fcd, 0x15480 } }, { { 0x020a9, 0x14fff } }, { { 0x02186, 0x14b99 } }, { { 0x02264, 0x1474e } },
  { { 0x02343, 0x1431b } }, { { 0x02423, 0x13f01 } }, { { 0x02504, 0x13afd } }, { { 0x025e6, 0x1370f } },
  { { 0x026ca, 0x13336 } }, { { 0x027ae, 0x12f71 } }, { { 0x02894, 0x12bc0 } }, { { 0x0297a, 0x12821 } },
  { { 0x02a62, 0x12494 } }, { { 0x02b4b, 0x12118 } }, { { 0x02c35, 0x11dac } }, { { 0x02d20, 0x11a51 } },
  { { 0x02e0c, 0x11704 } }, { { 0x02efa, 0x113c7 } }, { { 0x02fe9, 0x11098 } }, { { 0x030d9, 0x10d77 } },
  { { 0x031ca, 0x10a63 } }, { { 0x032bc, 0x1075c } }, { { 0x033b0, 0x10461 } }, { { 0x034a5, 0x10173 } },
  { { 0x0359b, 0x0fe90 } }, { { 0x03693, 0x0fbb9 } }, { { 0x0378c, 0x0f8ed } }, { { 0x03886, 0x0f62b } },
  { { 0x03981, 0x0f374 } }, { { 0x03a7e, 0x0f0c7 } }, { { 0x03b7c, 0x0ee23 } }, { { 0x03c7c, 0x0eb89 } },
  { { 0x03d7d, 0x0e8f9 } }, { { 0x03e7f, 0x0e671 } }, { { 0x03f83, 0x0e3f2 } }, { { 0x04088, 0x0e17c } },
  { { 0x0418e, 0x0df0e } }, { { 0x04297, 0x0dca8 } }, { { 0x043a0, 0x0da4a } }, { { 0x044ab, 0x0d7f3 } },
  { { 0x045b8, 0x0d5a5 } }, { { 0x046c6, 0x0d35d } }, { { 0x047d6, 0x0d11c } }, { { 0x048e7, 0x0cee3 } },
  { { 0x049fa, 0x0ccb0 } }, { { 0x04b0e, 0x0ca84 } }, { { 0x04c24, 0x0c85e } }, { { 0x04d3c, 0x0c63f } },
  { { 0x04e55, 0x0c426 } }, { { 0x04f71, 0x0c212 } }, { { 0x0508d, 0x0c005 } }, { { 0x051ac, 0x0bdfe } },
  { { 0x052cc, 0x0bbfc } }, { { 0x053ee, 0x0b9ff } }, { { 0x05512, 0x0b808 } }, { { 0x05638, 0x0b617 } },
  { { 0x0575f, 0x0b42a } }, { { 0x05888, 0x0b243 } }, { { 0x059b4, 0x0b061 } }, { { 0x05ae1, 0x0ae83 } },
  { { 0x05c10, 0x0acaa } }, { { 0x05d41, 0x0aad6 } }, { { 0x05e74, 0x0a907 } }, { { 0x05fa9, 0x0a73c } },
  { { 0x060e0, 0x0a575 } }, { { 0x06219, 0x0a3b3 } }, { { 0x06354, 0x0a1f5 } }, { { 0x06491, 0x0a03b } },
  { { 0x065d1, 0x09e85 } }, { { 0x06712, 0x09cd4 } }, { { 0x06856, 0x09b26 } }, { { 0x0699c, 0x0997c } },
  { { 0x06ae4, 0x097d6 } }, { { 0x06c2f, 0x09634 } }, { { 0x06d7c, 0x09495 } }, { { 0x06ecb, 0x092fa } },
  { { 0x0701d, 0x09162 } }, { { 0x07171, 0x08fce } }, { { 0x072c7, 0x08e3e } }, { { 0x07421, 0x08cb0 } },
  { { 0x0757c, 0x08b26 } }, { { 0x076da, 0x089a0 } }, { { 0x0783b, 0x0881c } }, { { 0x0799f, 0x0869c } },
  { { 0x07b05, 0x0851f } }, { { 0x07c6e, 0x083a4 } }, { { 0x07dd9, 0x0822d } }, { { 0x07f48, 0x080b9 } },
  { { 0x080b9, 0x07f48 } }, { { 0x0822d, 0x07dd9 } }, { { 0x083a4, 0x07c6e } }, { { 0x0851f, 0x07b05 } },
  { { 0x0869c, 0x0799f } }, { { 0x0881c, 0x0783b } }, { { 0x089a0, 0x076da } }, { { 0x08b26, 0x0757c } },
  { { 0x08cb0, 0x07421 } }, { { 0x08e3e, 0x072c7 } }, { { 0x08fce, 0x07171 } }, { { 0x09162, 0x0701d } },
  { { 0x092fa, 0x06ecb } }, { { 0x09495, 0x06d7c } }, { { 0x09634, 0x06c2f } }, { { 0x097d6, 0x06ae4 } },
  { { 0x0997c, 0x0699c } }, { { 0x09b26, 0x06856 } }, { { 0x09cd4, 0x06712 } }, { { 0x09e85, 0x065d1 } },
  { { 0x0a03b, 0x06491 } }, { { 0x0a1f5, 0x06354 } }, { { 0x0a3b3, 0x06219 } }, { { 0x0a575, 0x060e0 } },
  { { 0x0a73c, 0x05fa9 } }, { { 0x0a907, 0x05e74 } }, { { 0x0aad6, 0x05d41 } }, { { 0x0acaa, 0x05c10 } },
  { { 0x0ae83, 0x05ae1 } }, { { 0x0b061, 0x059b4 } }, { { 0x0b243, 0x05888 } }, { { 0x0b42a, 0x0575f } },
  { { 0x0b617, 0x05638 } }, { { 0x0b808, 0x05512 } }, { { 0x0b9ff, 0x053ee } }, { { 0x0bbfc, 0x052cc } },
  { { 0x0bdfe, 0x051ac } }, { { 0x0c005, 0x0508d } }, { { 0x0c212, 0x04f71 } }, { { 0x0c426, 0x04e55 } },
  { { 0x0c63f, 0x04d3c } }, { { 0x0c85e, 0x04c24 } }, { { 0x0ca84, 0x04b0e } }, { { 0x0ccb0, 0x049fa } },
  { { 0x0cee3, 0x048e7 } }, { { 0x0d11c, 0x047d6 } }, { { 0x0d35d, 0x046c6 } }, { { 0x0d5a5, 0x045b8 } },
  { { 0x0d7f3, 0x044ab } }, { { 0x0da4a, 0x043a0 } }, { { 0x0dca8, 0x04297 } }, { { 0x0df0e, 0x0418e } },
  { { 0x0e17c, 0x04088 } }, { { 0x0e3f2, 0x03f83 } }, { { 0x0e671, 0x03e7f } }, { { 0x0e8f9, 0x03d7d } },
  { { 0x0eb89, 0x03c7c } }, { { 0x0ee23, 0x03b7c } }, { { 0x0f0c7, 0x03a7e } }, { { 0x0f374, 0x03981 } },
  { { 0x0f62b, 0x03886 } }, { { 0x0f8ed, 0x0378c } }, { { 0x0fbb9, 0x03693 } }, { { 0x0fe90, 0x0359b } },
  { { 0x10173, 0x034a5 } }, { { 0x10461, 0x033b0 } }, { { 0x1075c, 0x032bc } }, { { 0x10a63, 0x031ca } },
  { { 0x10d77, 0x030d9 } }, { { 0x11098, 0x02fe9 } }, { { 0x113c7, 0x02efa } }, { { 0x11704, 0x02e0c } },
  { { 0x11a51, 0x02d20 } }, { { 0x11dac, 0x02c35 } }, { { 0x12118, 0x02b4b } }, { { 0x12494, 0x02a62 } },
  { { 0x12821, 0x0297a } }, { { 0x12bc0, 0x02894 } }, { { 0x12f71, 0x027ae } }, { { 0x13336, 0x026ca } },
  { { 0x1370f, 0x025e6 } }, { { 0x13afd, 0x02504 } }, { { 0x13f01, 0x02423 } }, { { 0x1431b, 0x02343 } },
  { { 0x1474e, 0x02264 } }, { { 0x14b99, 0x02186 } }, { { 0x14fff, 0x020a9 } }, { { 0x15480, 0x01fcd } },
  { { 0x1591e, 0x01ef3 } }, { { 0x15dda, 0x01e19 } }, { { 0x162b6, 0x01d40 } }, { { 0x167b4, 0x01c68 } },
  { { 0x16cd5, 0x01b91 } }, { { 0x1721c, 0x01abb } }, { { 0x1778a, 0x019e6 } }, { { 0x17d23, 0x01912 } },
  { { 0x182e8, 0x0183f } }, { { 0x188de, 0x0176d } }, { { 0x18f06, 0x0169c } }, { { 0x19564, 0x015cc } },
  { { 0x19bfc, 0x014fc } }, { { 0x1a2d4, 0x0142e } }, { { 0x1a9ee, 0x01360 } }, { { 0x1b151, 0x01294 } },
  { { 0x1b903, 0x011c8 } }, { { 0x1c10b, 0x010fd } }, { { 0x1c970, 0x01033 } }, { { 0x1d23c, 0x00f6a } },
  { { 0x1db78, 0x00ea2 } }, { { 0x1e531, 0x00dda } }, { { 0x1ef74, 0x00d13 } }, { { 0x1fa51, 0x00c4e } },
  { { 0x205dd, 0x00b89 } }, { { 0x2122e, 0x00ac4 } }, { { 0x21f60, 0x00a01 } }, { { 0x22d96, 0x0093f } },
  { { 0x23cfc, 0x0087d } }, { { 0x24dc9, 0x007bc } }, { { 0x26044, 0x006fc } }, { { 0x274ce, 0x0063c } },
  { { 0x28beb, 0x0057e } }, { { 0x2a658, 0x004c0 } }, { { 0x2c531, 0x00403 } }, { { 0x2ea40, 0x00346 } },
  { { 0x318a9, 0x0028b } }, { { 0x356cb, 0x001d0 } }, { { 0x3b520, 0x00116 } }, { { 0x48000, 0x0005c } },
};
void BinProbModel::init( int qp, int initId )
{
  int slope = (initId >> 3) - 4;
  int offset = ((initId & 7) * 18) + 1;
  int inistate = ((slope   * (qp - 16)) >> 1) + offset;
  int state_clip = inistate < 1 ? 1 : inistate > 127 ? 127 : inistate;
  const int p1 = (state_clip << 8);
  m_state[0]   = p1 & MASK_0;
  m_state[1]   = p1 & MASK_1;
}




CtxSet::CtxSet( std::initializer_list<CtxSet> ctxSets )
{
  uint16_t  minOffset = std::numeric_limits<uint16_t>::max();
  uint16_t  maxOffset = 0;
  for( auto iter = ctxSets.begin(); iter != ctxSets.end(); iter++ )
  {
    minOffset = std::min<uint16_t>( minOffset, (*iter).Offset              );
    maxOffset = std::max<uint16_t>( maxOffset, (*iter).Offset+(*iter).Size );
  }
  Offset  = minOffset;
  Size    = maxOffset - minOffset;
}





const std::vector<uint8_t>& ContextSetCfg::getInitTable( unsigned initId )
{
  CHECK( initId >= (unsigned)sm_InitTables.size(),
         "Invalid initId (" << initId << "), only " << sm_InitTables.size() << " tables defined." );
  return sm_InitTables[initId];
}


CtxSet ContextSetCfg::addCtxSet( std::initializer_list<std::initializer_list<uint8_t>> initSet2d )
{
  const std::size_t startIdx  = sm_InitTables[0].size();
  const std::size_t numValues = ( *initSet2d.begin() ).size();
        std::size_t setId     = 0;
  for( auto setIter = initSet2d.begin(); setIter != initSet2d.end() && setId < sm_InitTables.size(); setIter++, setId++ )
  {
    const std::initializer_list<uint8_t>& initSet   = *setIter;
    std::vector<uint8_t>&           initTable = sm_InitTables[setId];
    CHECK( initSet.size() != numValues,
           "Number of init values do not match for all sets (" << initSet.size() << " != " << numValues << ")." );
    initTable.resize( startIdx + numValues );
    std::size_t elemId = startIdx;
    for( auto elemIter = ( *setIter ).begin(); elemIter != ( *setIter ).end(); elemIter++, elemId++ )
    {
      initTable[elemId] = *elemIter;
    }
  }
  return CtxSet( (uint16_t)startIdx, (uint16_t)numValues );
}


#define CNU 35
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables(VVENC_NUMBER_OF_SLICE_TYPES + 1);

// clang-format off
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
  {  18,  27,  15,  18,  28,  45,  26,   7,  23, },
  {  11,  35,  53,  12,   6,  30,  13,  15,  31, },
  {  19,  28,  38,  27,  29,  38,  20,  30,  31, },
  {  12,  13,   8,   8,  13,  12,   5,   9,   9, },
});

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
({
  {  26,  36,  38,  18,  34,  21, },
  {  20,  14,  23,  18,  19,   6, },
  {  27,   6,  15,  25,  19,  37, },
  {   0,   8,   8,  12,  12,   8, },
});

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
({
  {  43,  42,  37,  42,  44, },
  {  43,  35,  37,  34,  52, },
  {  43,  42,  29,  27,  44, },
  {   9,   8,   9,   8,   5, },
});

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
({
  {  28,  29,  28,  29, },
  {  43,  37,  21,  22, },
  {  36,  45,  36,  45, },
  {  12,  13,  12,  13, },
});

const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
({
  {  25,  20, },
  {  25,  12, },
  { CNU, CNU, },
  {   1,   0, },
});

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
  {  57,  60,  46, },
  {  57,  59,  45, },
  {   0,  26,  28, },
  {   5,   4,   8, },
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
  {   6, },
  {  21, },
  {  26, },
  {   4, },
});

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
({
  {  46,  15, },
  {  38,   7, },
  { CNU, CNU, },
  {   5,   5, },
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
  {  18, },
  {  20, },
  {  34, },
  {   4, },
});

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
  {  25, },
  {  26, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
  {  43, },
  {  43, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  {  59, },
  {  60, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
({
  {  25,  58,  45, },
  {  48,  57,  44, },
  { CNU, CNU, CNU, },
  {   4,   4,   4, },
});

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
({
  {   4, },
  {   5, },
  { CNU, },
  {   0, },
});

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet
({
  {  57, },
  {  57, },
  { CNU, },
  {   1, },
});



const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
  {  40,  35, },
  {  40,  35, },
  { CNU, CNU, },
  {   5,   1, },
});

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
({
  {  44, },
  {  36, },
  {  45, },
  {   6, },
});

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
({
  {  13,   6, },
  {  12,  20, },
  {  13,  28, },
  {   1,   5, },
});


const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
  {  25,  59, },
  {  25,  58, },
  {  25,  60, },
  {   5,   8, },
});

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
({
  {  56,  57,  50,  26, },
  {  41,  57,  58,  26, },
  {  33,  49,  50,  25, },
  {   9,  10,   9,   6, },
});

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
({
  {  33,  43, },
  {  33,  36, },
  {  33,  43, },
  {   9,   2, },
});

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
({
  {  26, },
  {  34, },
  {  59, },
  {   4, },
});

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet
({
  {  27, },
  {  27, },
  {  27, },
  {   9, },
});

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
({
  {  25, },
  {  25, },
  {  34, },
  {   5, },
});

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
  {  14,  13,   5,   4,   3,  40, },
  {   7,   6,   5,  12,   4,  40, },
  { CNU, CNU, CNU, CNU, CNU, CNU, },
  {   0,   0,   1,   4,   4,   0, },
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
  {   5,  35, },
  {  20,  35, },
  { CNU, CNU, },
  {   0,   4, },
});

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
  {  19,  13,   6, },
  {  12,  13,  14, },
  { CNU, CNU, CNU, },
  {   4,   0,   0, },
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
  {  35, },
  {  35, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet
({
  {   5, },
  {   4, },
  { CNU, },
  {   1, },
});

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
  {  51,  36, },
  {  44,  43, },
  {  14,  45, },
  {   9,   5, },
});

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
({
  {  19,  21,   0,  28, },
  {  40,  36,   0,  13, },
  {  19,  35,   1,  27, },
  {   1,   4,   1,   0, },
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
  {  12, },
  {   5, },
  {   6, },
  {   4, },
});

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::QtCbf[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  15,   6,   5,  14, },
    {  23,   5,  20,   7, },
    {  15,  12,   5,   7, },
    {   5,   1,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,  37, },
    {  25,  28, },
    {  12,  21, },
    {   5,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   9,  36,  45, },
    {  25,  29,  45, },
    {  33,  28,  36, },
    {   2,   1,   0, },
  })
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  25,  45, },
    {  25,  30, },
    {  18,  31, },
    {   8,   5, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,  14, },
    {  25,  45, },
    {  25,  15, },
    {   5,   8, },
  })
};

const CtxSet ContextSetCfg::SigFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  17,  41,  49,  36,   1,  49,  50,  37,  48,  51,  58,  45, },
    {  17,  41,  42,  29,  25,  49,  43,  37,  33,  58,  51,  30, },
    {  25,  19,  28,  14,  25,  20,  29,  30,  19,  37,  30,  38, },
    {  12,   9,   9,  10,   9,   9,   9,  10,   8,   8,   8,  10, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   9,  49,  50,  36,  48,  59,  59,  38, },
    {  17,  34,  35,  21,  41,  59,  60,  38, },
    {  25,  27,  28,  37,  34,  53,  53,  46, },
    {  12,  12,   9,  13,   4,   5,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  26,  45,  53,  46,  49,  54,  61,  39,  35,  39,  39,  39, },
    {  19,  38,  38,  46,  34,  54,  54,  39,   6,  39,  39,  39, },
    {  11,  38,  46,  54,  27,  39,  39,  39,  44,  39,  39,  39, },
    {   9,  13,   8,   8,   8,   8,   8,   5,   8,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  34,  45,  38,  31,  58,  39,  39,  39, },
    {  35,  45,  53,  54,  44,  39,  39,  39, },
    {  19,  46,  38,  39,  52,  39,  39,  39, },
    {   8,  12,  12,   8,   4,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  19,  54,  39,  39,  50,  39,  39,  39,   0,  39,  39,  39, },
    {  19,  39,  54,  39,  19,  39,  39,  39,  56,  39,  39,  39, },
    {  18,  39,  39,  39,  27,  39,  39,  39,   0,  39,  39,  39, },
    {   8,   8,   8,   8,   8,   0,   4,   4,   0,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  34,  38,  54,  39,  41,  39,  39,  39, },
    {  34,  38,  62,  39,  26,  39,  39,  39, },
    {  11,  39,  39,  39,  19,  39,  39,  39, },
    {   8,   8,   8,   8,   4,   0,   0,   0, },
  })
};

const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  33,  40,  25,  41,  26,  42,  25,  33,  26,  34,  27,  25,  41,  42,  42,  35,  33,  27,  35,  42,  43, },
    {  18,  17,  33,  18,  26,  42,  25,  33,  26,  42,  27,  25,  34,  42,  42,  35,  26,  27,  42,  20,  20, },
    {  33,  25,  18,  26,  34,  27,  25,  26,  19,  42,  35,  33,  19,  27,  35,  35,  34,  42,  20,  43,  20, },
    {   8,   9,  12,  13,  13,  13,  10,  13,  13,  13,  13,  13,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  33,  25,  26,  34,  19,  27,  33,  42,  43,  35,  43, },
    {  25,  25,  26,  11,  19,  27,  33,  42,  35,  35,  43, },
    {  33,  25,  26,  42,  19,  27,  26,  50,  35,  20,  43, },
    {   8,  12,  12,  12,  13,  13,  13,  13,  13,  13,  13, },
  })
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  25,   0,   0,  17,  25,  26,   0,   9,  25,  33,  19,   0,  25,  33,  26,  20,  25,  33,  27,  35,  22, },
    {  17,   0,   1,  17,  25,  18,   0,   9,  25,  33,  34,   9,  25,  18,  26,  20,  25,  18,  19,  27,  29, },
    {  25,   1,  40,  25,  33,  11,  17,  25,  25,  18,   4,  17,  33,  26,  19,  13,  33,  19,  20,  28,  22, },
    {   1,   5,   9,   9,   9,   6,   5,   9,  10,  10,   9,   9,   9,   9,   9,   9,   6,   8,   9,   9,  10, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,   1,  25,  33,  26,  12,  25,  33,  27,  28,  37, },
    {  17,   9,  25,  10,  18,   4,  17,  33,  19,  20,  29, },
    {  40,   9,  25,  18,  26,  35,  25,  26,  35,  28,  37, },
    {   1,   5,   8,   8,   9,   6,   6,   9,   8,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0,   0,  33,  34,  35,  21,  25,  34,  35,  28,  29,  40,  42,  43,  29,  30,  49,  36,  37,  45,  38, },
    {   0,  17,  26,  19,  35,  21,  25,  34,  20,  28,  29,  33,  27,  28,  29,  22,  34,  28,  44,  37,  38, },
    {  25,  25,  11,  27,  20,  21,  33,  12,  28,  21,  22,  34,  28,  29,  29,  30,  36,  29,  45,  30,  23, },
    {   9,   5,  10,  13,  13,  10,   9,  10,  13,  13,  13,   9,  10,  10,  10,  13,   8,   9,  10,  10,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0,  40,  34,  43,  36,  37,  57,  52,  45,  38,  46, },
    {   0,  25,  19,  20,  13,  14,  57,  44,  30,  30,  23, },
    {  40,  33,  27,  28,  21,  37,  36,  37,  45,  38,  46, },
    {   8,   8,   9,  12,  12,  10,   5,   9,   9,   9,  13, },
  })
};

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
    {   6,   6,  12,  14,   6,   4,  14,   7,   6,   4,  29,   7,   6,   6,  12,  28,   7,  13,  13,  35, },
    {   6,  13,  12,   6,   6,  12,  14,  14,  13,  12,  29,   7,   6,  13,  36,  28,  14,  13,   5,  26, },
    {  13,   5,   4,  21,  14,   4,   6,  14,  21,  11,  14,   7,  14,   5,  11,  21,  30,  22,  13,  42, },
    {   8,   5,   4,   5,   4,   4,   5,   4,   1,   0,   4,   1,   0,   0,   0,   0,   1,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  19,   5,   4, },
    {  12,   4,  18, },
    {  12,   4,   3, },
    {   5,   4,   4, },
  })
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
    {   5,   5,  20,  13,  13,  19,  21,   6,  12,  12,  14,  14,   5,   4,  12,  13,   7,  13,  12,  41, },
    {   5,   5,  12,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  13,  21,  14,  20,  12,  34, },
    {  13,   5,   4,   6,  13,  11,  14,   6,   5,   3,  14,  22,   6,   4,   3,   6,  22,  29,  20,  34, },
    {   8,   5,   8,   5,   5,   4,   5,   5,   4,   0,   5,   4,   1,   0,   0,   1,   4,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  11,   5,  27, },
    {  11,   4,  18, },
    {  12,   4,   3, },
    {   6,   5,   5, },
  })
};

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
  {  34, },
  {  34, },
  {  42, },
  {  12, },
});

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
({
  {  28, },
  {  28, },
  { CNU, },
  {   5, },
});

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
  {   2, },
  {  60, },
  {  60, },
  {   0, },
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
  {   2, },
  {   5, },
  {  13, },
  {   4, },
});

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
({
  {  52,  37,  27, },
  {  37,  45,  27, },
  {  28,  52,  42, },
  {   9,   9,  10, },
});

const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
({
  {  17, },
  {   0, },
  {  25, },
  {   1, },
});


const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
({
  {  25,  17, },
  {  25,   9, },
  {  25,   9, },
  {   1,   1, },
});

const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
({
  {  45,  25,  27,   0, },
  {  45,  40,  27,   0, },
  {  29,   0,  28,   0, },
  {   8,   0,   9,   0, },
});


const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
({
  {  41,  57, },
  {  56,  57, },
  { CNU, CNU, },
  {   1,   5, },
});

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
({
  {  42, },
  {  42, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
({
  {  35,  51,  27, },
  {  20,  43,  12, },
  { CNU, CNU, CNU, },
  {   8,   4,   1, },
});

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
({
  {  28, },
  {  28, },
  { CNU, },
  {  13, },
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
  {  59,  26,  50,  60,  38, },
  {  59,  48,  58,  60,  60, },
  { CNU,  34, CNU, CNU, CNU, },
  {   0,   5,   0,   0,   4, },
});

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
({
  {  33,  52,  46,  25,  61,  54,  25,  61,  54, },
  {  13,  23,  46,   4,  61,  54,  19,  46,  54, },
  {  62,  39,  39,  54,  39,  39,  31,  39,  39, },
  {   0,   0,   0,   4,   0,   0,   1,   0,   0, },
});

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
({
  {  11,  26, },
  {  20,  12, },
  {  11,  11, },
  {   0,   0, },
});

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
({
  {  46, },
  {  46, },
  {  46, },
  {   0, },
});

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet
({
  {  25,  35,  38,  25,  28,  38, },
  {  18,  21,  38,  18,  21,  38, },
  {  18,  30,  31,  18,  30,  31, },
  {   4,   1,   4,   4,   1,   4, },
});

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
({
  {   0,  43,  45, },
  {   0,  57,  44, },
  {  17,  42,  36, },
  {   1,   5,   8, },
});

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
({
  {  42,  43,  52, },
  {  27,  36,  45, },
  {  12,  21,  35, },
  {   1,   1,   0, },
});

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
({
  {  18,  35,  45, },
  {  18,  12,  29, },
  {  18,  20,  38, },
  {   5,   8,   8, },
});

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
({
  {  25,  50,  37, },
  {  40,  35,  44, },
  {  25,  28,  38, },
  {  13,  13,   8, },
});

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
({
  {  11, },
  {   3, },
  {  11, },
  {   6, },
});

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
({
  { CNU,   3,   4,   4,   5, },
  { CNU,   2,  10,   3,   3, },
  { CNU,  10,   3,   3,   3, },
  { DWS,   1,   1,   1,   1, },
});

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
({
  {  19,  11,   4,   6, },
  {  18,  11,   4,  28, },
  {  11,   5,   5,  14, },
  {   4,   2,   1,   6, },
});

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
({
  {  35,  25,  46,  28,  33,  38, },
  {   5,  10,  53,  43,  25,  46, },
  {  12,  17,  46,  28,  25,  46, },
  {   1,   4,   4,   5,   8,   8, },
});
// clang-format on

const unsigned ContextSetCfg::NumberOfContexts = (unsigned)ContextSetCfg::sm_InitTables[0].size();


// combined sets
const CtxSet ContextSetCfg::Sao = { ContextSetCfg::SaoMergeFlag, ContextSetCfg::SaoTypeIdx };

const CtxSet ContextSetCfg::Alf = { ContextSetCfg::ctbAlfFlag, ContextSetCfg::ctbAlfAlternative, ContextSetCfg::AlfUseTemporalFilt };

CtxStore::CtxStore()
  : m_CtxBuffer ()
  , m_Ctx       ( nullptr )
{}

CtxStore::CtxStore( bool dummy )
  : m_CtxBuffer ( ContextSetCfg::NumberOfContexts )
  , m_Ctx       ( m_CtxBuffer.data() )
{}

CtxStore::CtxStore( const CtxStore& ctxStore )
  : m_CtxBuffer ( ctxStore.m_CtxBuffer )
  , m_Ctx       ( m_CtxBuffer.data() )
{}

void CtxStore::init( int qp, int initId )
{
  const std::vector<uint8_t>& initTable = ContextSetCfg::getInitTable( initId );
  CHECK( m_CtxBuffer.size() != initTable.size(),
        "Size of init table (" << initTable.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  const std::vector<uint8_t> &rateInitTable = ContextSetCfg::getInitTable(VVENC_NUMBER_OF_SLICE_TYPES);
  CHECK(m_CtxBuffer.size() != rateInitTable.size(),
        "Size of rate init table (" << rateInitTable.size() << ") does not match size of context buffer ("
                                    << m_CtxBuffer.size() << ").");
  int clippedQP = Clip3( 0, MAX_QP, qp );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].init( clippedQP, initTable[k] );
    m_CtxBuffer[k].setLog2WindowSize(rateInitTable[k]);
  }
}

void CtxStore::setWinSizes( const std::vector<uint8_t>& log2WindowSizes )
{
  CHECK( m_CtxBuffer.size() != log2WindowSizes.size(),
        "Size of window size table (" << log2WindowSizes.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].setLog2WindowSize( log2WindowSizes[k] );
  }
}

void CtxStore::loadPStates( const std::vector<uint16_t>& probStates )
{
  CHECK( m_CtxBuffer.size() != probStates.size(),
        "Size of prob states table (" << probStates.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].setState( probStates[k] );
  }
}

void CtxStore::savePStates( std::vector<uint16_t>& probStates ) const
{
  probStates.resize( m_CtxBuffer.size(), uint16_t(0) );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    probStates[k] = m_CtxBuffer[k].getState();
  }
}





Ctx::Ctx()                                                      {}
Ctx::Ctx( const BinProbModel*   dummy ) : m_CtxStore  ( true )  {}

Ctx::Ctx( const Ctx& ctx )
  : m_CtxStore    ( ctx.m_CtxStore    )
{
}

} // namespace vvenc

//! \}


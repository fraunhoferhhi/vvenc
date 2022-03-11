/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD

License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     SampleAdaptiveOffset.h
    \brief    sample adaptive offset class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Unit.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

template<typename T> static inline int sgn( T val )
{
  return ( T( 0 ) < val ) - ( val < T( 0 ) );
}

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_SAO_TRUNCATED_BITDEPTH     10

// ====================================================================================================================
// Class definition
// ====================================================================================================================

  enum Available
  {
    LeftAvail = 1,
    RightAvail = 2,
    AboveAvail = 4,
    BelowAvail = 8,
    AboveLeftAvail = 16,
    AboveRightAvail = 32,
    BelowLeftAvail = 64,
    BelowRightAvail = 128,
  };

class SampleAdaptiveOffset
{
public:

  SampleAdaptiveOffset();
  virtual ~SampleAdaptiveOffset();
  void        SAOProcess      ( CodingStructure& cs, SAOBlkParam* saoBlkParams );
  void        init            ( ChromaFormat format, uint32_t maxCUWidth, uint32_t maxCUHeight, uint32_t lumaBitShift, uint32_t chromaBitShift );
  static int  getMaxOffsetQVal( const int channelBitDepth) { return (1<<(std::min<int>(channelBitDepth,MAX_SAO_TRUNCATED_BITDEPTH)-5))-1; } //Table 9-32, inclusive

protected:
  void deriveLoopFilterBoundaryAvailibility( CodingStructure& cs, const Position& pos, uint8_t& availMask ) const;

  void (*offsetBlock) ( const int     channelBitDepth,
                        const ClpRng& clpRng,
                        int           typeIdx,
                        int*          offset,
                        int           startIdx,
                        const Pel*    srcBlk,
                        Pel*          resBlk,
                        ptrdiff_t     srcStride,
                        ptrdiff_t     resStride,
                        int           width,
                        int           height,
                        uint8_t       availMask,
                        std::vector<int8_t> &signLineBuf1,
                        std::vector<int8_t> &signLineBuf2);

  void invertQuantOffsets       (ComponentID compIdx, int typeIdc, int typeAuxInfo, int* dstOffsets, int* srcOffsets);
  void reconstructBlkSAOParam   (SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES]);
  int  getMergeList             (CodingStructure& cs, int ctuRsAddr, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES]);
  void offsetCTU                (const UnitArea& area, const CPelUnitBuf& src, PelUnitBuf& res, SAOBlkParam& saoblkParam, CodingStructure& cs);
  void xReconstructBlkSAOParams (CodingStructure& cs, SAOBlkParam* saoBlkParams);

#ifdef TARGET_SIMD_X86
  void initSampleAdaptiveOffsetX86();
  template <X86_VEXT vext>
  void _initSampleAdaptiveOffsetX86();
#endif

protected:
  uint32_t    m_offsetStepLog2[MAX_NUM_COMP]; //offset step
  uint32_t    m_numberOfComponents;

  std::vector<int8_t> m_signLineBuf1;
  std::vector<int8_t> m_signLineBuf2;

private:
  bool m_picSAOEnabled[MAX_NUM_COMP];
};

} // namespace vvenc

//! \}


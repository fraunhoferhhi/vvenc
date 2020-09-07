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


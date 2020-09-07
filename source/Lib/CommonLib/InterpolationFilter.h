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
/**
 * \file
 * \brief Declaration of InterpolationFilter class
 */

#pragma once

#include "CommonDef.h"
#include "Common.h"
#include "Unit.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

#define IF_INTERNAL_PREC 14 ///< Number of bits for internal precision
#define IF_FILTER_PREC    6 ///< Log2 of sum of filter taps
#define IF_INTERNAL_OFFS (1<<(IF_INTERNAL_PREC-1)) ///< Offset used internally
#define IF_INTERNAL_PREC_BILINEAR 10 ///< Number of bits for internal precision
#define IF_FILTER_PREC_BILINEAR   4  ///< Bilinear filter coeff precision so that intermediate value will not exceed 16 bit for SIMD - bit exact
/**
 * \brief Interpolation filter class
 */
class InterpolationFilter
{
  static const TFilterCoeff m_lumaFilter4x4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS+1][NTAPS_LUMA];
  static const TFilterCoeff m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS+1][NTAPS_LUMA]; ///< Luma filter taps
  static const TFilterCoeff m_lumaAltHpelIFilter[NTAPS_LUMA]; ///< Luma filter taps
  static const TFilterCoeff m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS+1][NTAPS_CHROMA]; ///< Chroma filter taps
  static const TFilterCoeff m_bilinearFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR]; ///< bilinear filter taps
  static const TFilterCoeff m_bilinearFilterPrec4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR]; ///< bilinear filter taps
public:
  template<bool isFirst, bool isLast>
  static void filterCopy(const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height,                                                       bool biMCForDMVR);

  template<int N, bool isVertical, bool isFirst, bool isLast>
  static void filter    (const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height,                            TFilterCoeff const *coeff, bool biMCForDMVR);
  template<int N>
  void filterHor        (const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride, int width, int height, bool isLast,               TFilterCoeff const *coeff, bool biMCForDMVR);

  template<int N>
  void filterVer        (const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride, int width, int height, bool isFirst, bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR);
  
  template<bool isLast, int w>
  static void filterXxY_N4     (const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);
  template<bool isLast, int w>
  static void filterXxY_N8     (const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);

  static void xWeightedGeoBlk(const ClpRngs &clpRngs, const PredictionUnit &pu, const uint32_t width,
                              const uint32_t height, const ComponentID compIdx, const uint8_t splitDir,
                              PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1);
  void weightedGeoBlk(const ClpRngs &clpRngs, const PredictionUnit &pu, const uint32_t width, const uint32_t height,
                      const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                      PelUnitBuf &predSrc1);
  InterpolationFilter();
  ~InterpolationFilter() {}

  void( *m_filterHor[3][2][2] )( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
  void( *m_filterVer[3][2][2] )( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
  void( *m_filterCopy[2][2] )  ( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, bool biMCForDMVR);
  void( *m_filter4x4  [2][2] ) ( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV );
  void( *m_filter8x8  [3][2] ) ( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV );
  void( *m_filter16x16[3][2] ) ( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV );
  void (*m_weightedGeoBlk)(const ClpRngs &clpRngs, const PredictionUnit &pu, const uint32_t width, const uint32_t height,
                           const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                           PelUnitBuf &predSrc1);

  void initInterpolationFilter( bool enable );
#ifdef TARGET_SIMD_X86
  void initInterpolationFilterX86();
  template <X86_VEXT vext>
  void _initInterpolationFilterX86();
#endif

  void filter4x4  (const ComponentID compID, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, int fracX, int fracY,   bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf = false, int nFilterIdx = 0);
  void filter8x8  (const ComponentID compID, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, int fracX, int fracY,   bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf = false, int nFilterIdx = 0);
  void filter16x16(const ComponentID compID, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, int fracX, int fracY,   bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf = false, int nFilterIdx = 0);
  void filterHor  (const ComponentID compID, Pel const* src, int srcStride, Pel* dst, int dstStride, int width, int height, int frac,               bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf  = false,                             int nFilterIdx = 0, bool biMCForDMVR = false);
  void filterVer  (const ComponentID compID, Pel const* src, int srcStride, Pel* dst, int dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf  = false,                             int nFilterIdx = 0, bool biMCForDMVR = false);

  static TFilterCoeff const * const getChromaFilterTable(const int deltaFract) { return m_chromaFilter[deltaFract]; };
};

} // namespace vvenc

//! \}


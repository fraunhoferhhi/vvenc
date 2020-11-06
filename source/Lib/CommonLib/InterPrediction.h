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
/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#pragma once

#include "InterpolationFilter.h"
#include "Unit.h"
#include "Picture.h"
#include "RdCost.h"
#include "ContextModelling.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

// forward declaration
class Mv;


// ====================================================================================================================
// Class definition
// ====================================================================================================================
class InterPredInterpolation
{
  Pel*                 m_gradX0;
  Pel*                 m_gradY0;
  Pel*                 m_gradX1;
  Pel*                 m_gradY1;
  Pel                  m_gradBuf[2][(AFFINE_MIN_BLOCK_SIZE + 2) * (AFFINE_MIN_BLOCK_SIZE + 2)];
  int                  m_dMvBuf[2][16 * 2];
  Mv*                  m_storedMv;

protected:
  bool                 m_skipPROF;
  bool                 m_encOnly;
  bool                 m_isBi;
  InterpolationFilter  m_if;
  Pel*                 m_filteredBlock        [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMP];
  Pel*                 m_filteredBlockTmp     [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMP];

private:
  int  xRightShiftMSB         ( int numer, int denom );
  void xApplyBDOF             ( PelBuf& yuvDst, const ClpRng& clpRng );
  void(*xFpAddBDOFAvg4)       ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dst, int dstStride, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, unsigned shift, int offset, const ClpRng& clpRng);
  void(*xFpBDOFGradFilter)    ( const Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth);
  void(*xFpCalcBDOFSums)      ( const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int xu, int yu, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGY_GX);

#if ENABLE_SIMD_OPT_BDOF
  void initInterPredictionX86();
  template <X86_VEXT vext>
  void _initInterPredictionX86();
#endif

protected:
  void xWeightedAverage       ( const CodingUnit& cu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const bool bdofApplied );
  void xPredAffineBlk         ( const ComponentID compID, const CodingUnit& cu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool bi, const ClpRng& clpRng, const RefPicList refPicList = REF_PIC_LIST_X);
  void xPredInterBlk          ( const ComponentID compID, const CodingUnit& cu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool bi, const ClpRng& clpRng
                              , const bool bdofApplied
                              , const bool isIBC
                              , const RefPicList refPicList = REF_PIC_LIST_X
                              , const SizeType dmvrWidth = 0
                              , const SizeType dmvrHeight = 0
                              , const bool bilinearMC = false
                              , const Pel* srcPadBuf = NULL
                              , const int32_t srcPadStride = 0
                              );

public:
  InterPredInterpolation();
  virtual ~InterPredInterpolation();
  void    destroy         ();
  void    init            ();

  void    weightedGeoBlk  ( const ClpRngs &clpRngs, CodingUnit& cu, const uint8_t splitDir, int32_t channel,
                            PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1);

  static bool isSubblockVectorSpreadOverLimit(int a, int b, int c, int d, int predType);
};

class DMVR : public InterPredInterpolation
{
  RdCost*                 m_pcRdCost;

  PelStorage              m_yuvPred[NUM_REF_PIC_LIST_01];
  PelStorage              m_yuvTmp[NUM_REF_PIC_LIST_01];
  PelStorage              m_yuvPad[NUM_REF_PIC_LIST_01];
  const Mv m_pSearchOffset[25] = { Mv(-2,-2), Mv(-1,-2), Mv(0,-2), Mv(1,-2), Mv(2,-2),
                                   Mv(-2,-1), Mv(-1,-1), Mv(0,-1), Mv(1,-1), Mv(2,-1),
                                   Mv(-2, 0), Mv(-1, 0), Mv(0, 0), Mv(1, 0), Mv(2, 0),
                                   Mv(-2, 1), Mv(-1, 1), Mv(0, 1), Mv(1, 1), Mv(2, 1),
                                   Mv(-2, 2), Mv(-1, 2), Mv(0, 2), Mv(1, 2), Mv(2, 2) };
private:
  void     xCopyAndPad          ( const CodingUnit& cu, PelUnitBuf& pcPad, RefPicList refId, bool forLuma);
  void     xFinalPaddedMCForDMVR( const CodingUnit& cu, PelUnitBuf* dstBuf, const PelUnitBuf *refBuf, const bool bioApplied, const Mv startMV[NUM_REF_PIC_LIST_01], const Mv& refMV );

protected:
  DMVR();
  virtual ~DMVR();
  void destroy();
  void init                    ( RdCost* pcRdCost, const ChromaFormat chFormat );
  void xProcessDMVR            ( const CodingUnit& cu, PelUnitBuf& pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied );
};

class InterPrediction : public DMVR
{
protected:
  ChromaFormat m_currChromaFormat;

private:
  PelStorage   m_yuvPred[NUM_REF_PIC_LIST_01];
  bool         m_subPuMC;
  PelStorage   m_geoPartBuf[2]; 

  void xPredInterUni            ( const CodingUnit& cu, const RefPicList& refPicList, PelUnitBuf& pcYuvPred, const bool bi, const bool bdofApplied );
  void xPredInterBi             ( const CodingUnit& cu, PelUnitBuf& yuvPred, const bool bdofApplied = false );
  void xSubPuBDOF               ( const CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList& refPicList = REF_PIC_LIST_X );
  bool xCheckIdenticalMotion    ( const CodingUnit& cu ) const;

public:
  InterPrediction();
  virtual ~InterPrediction();

  void    init                  ( RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize);
  void    destroy               ();

  // inter
  bool    motionCompensation    ( CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList refPicList = REF_PIC_LIST_X );
  void    motionCompensationIBC ( CodingUnit& cu, PelUnitBuf& predBuf );
  void    xSubPuMC              ( CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList& eRefPicList = REF_PIC_LIST_X );
  void    motionCompensationGeo ( CodingUnit& cu, PelUnitBuf& predBuf, const MergeCtx& geoMrgCtx );
};

} // namespace vvenc

//! \}


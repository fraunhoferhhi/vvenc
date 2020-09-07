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
/** \file     TrQuant.h
    \brief    transform and quantization class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Unit.h"
#include "Contexts.h"
#include "ContextModelling.h"
#include "UnitPartitioner.h"
#include "Quant.h"
#include "DepQuant.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

typedef void FwdTrans(const TCoeff*, TCoeff*, int, int, int, int);
typedef void InvTrans(const TCoeff*, TCoeff*, int, int, int, int, const TCoeff, const TCoeff);

// ====================================================================================================================
// Class definition
// ====================================================================================================================
typedef std::pair<int, bool> TrMode;
typedef std::pair<int, int>  TrCost;

/// transform and quantization class
class TrQuant
{
public:
  TrQuant();
  ~TrQuant();

  // initialize class
  void init(
             const Quant* otherQuant,
             const int  rdoq                 = 0,
             const bool bUseRDOQTS           = false,
             const bool useSelectiveRDOQ     = false,
             const bool bEnc                 = false,
             const bool useTransformSkipFast = false,
             const int  dqThrValue           = 8
           );

public:
  void invTransformNxN    ( TransformUnit& tu, const ComponentID compID, PelBuf& pResi, const QpParam& cQPs);
  void transformNxN       ( TransformUnit& tu, const ComponentID compID, const QpParam& cQP, TCoeff &uiAbsSum, const Ctx& ctx, const bool loadTr = false);
  void checktransformsNxN ( TransformUnit& tu, std::vector<TrMode> *trModes, const int maxCand);

  void                        invTransformICT     ( const TransformUnit& tu, PelBuf& resCb, PelBuf& resCr );
  std::pair<int64_t,int64_t>  fwdTransformICT     ( const TransformUnit& tu, const PelBuf& resCb, const PelBuf& resCr, PelBuf& resC1, PelBuf& resC2, int jointCbCr = -1 );
  std::vector<int>            selectICTCandidates ( const TransformUnit& tu, CompStorage* resCb, CompStorage* resCr );

  void   setLambdas  ( const double lambdas[MAX_NUM_COMP] )   { m_quant->setLambdas( lambdas ); }
  void   selectLambda( const ComponentID compIdx )            { m_quant->selectLambda( compIdx ); }
  void   getLambdas  ( double (&lambdas)[MAX_NUM_COMP]) const { m_quant->getLambdas( lambdas ); }
  void   scaleLambda ( const double scale)                    { m_quant->scaleLambda(scale);}

  DepQuant* getQuant ()                                       { return m_quant; }

protected:
  TCoeff*   m_plTempCoeff;
  bool      m_bEnc;
  bool      m_scalingListEnabled;
  TCoeff*   m_blk;
  TCoeff*   m_tmp;

private:
  DepQuant* m_quant;          //!< Quantizer
  TCoeff    m_tempInMatrix[48];
  TCoeff    m_tempOutMatrix[48];
  static const int maxAbsIctMode = 3;
  void                      (*m_invICTMem[1+2*maxAbsIctMode])(PelBuf&,PelBuf&);
  std::pair<int64_t,int64_t>(*m_fwdICTMem[1+2*maxAbsIctMode])(const PelBuf&,const PelBuf&,PelBuf&,PelBuf&);
  void                      (**m_invICT)(PelBuf&,PelBuf&);
  std::pair<int64_t,int64_t>(**m_fwdICT)(const PelBuf&,const PelBuf&,PelBuf&,PelBuf&);
  TCoeff    m_mtsCoeffs[NUM_TRAFO_MODES_MTS][MAX_TB_SIZEY * MAX_TB_SIZEY];

  uint32_t xGetLFNSTIntraMode( const Area& tuArea, const uint32_t dirMode );
  bool     xGetTransposeFlag(uint32_t intraMode);
  void     xFwdLfnst    ( const TransformUnit &tu, const ComponentID compID, const bool loadTr = false);
  void     xInvLfnst    ( const TransformUnit &tu, const ComponentID compID);
  void     xFwdLfnstNxN ( int *src, int *dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );
  void     xInvLfnstNxN ( int *src, int *dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );
  void     xSetTrTypes  ( const TransformUnit& tu, const ComponentID compID, const int width, const int height, int &trTypeHor, int &trTypeVer );

  // forward Transform
  void xT               (const TransformUnit& tu, const ComponentID compID, const CPelBuf& resi, CoeffBuf& dstCoeff, const int width, const int height);

  // quantization
  void xQuant           (TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx);

  // dequantization
  void xDeQuant( const TransformUnit& tu,
                       CoeffBuf      &dstCoeff,
                 const ComponentID   &compID,
                 const QpParam       &cQP      );

  // inverse transform
  void xIT     ( const TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pCoeff, PelBuf& pResidual );

  void xGetCoeffEnergy(
                       TransformUnit  &tu,
                 const ComponentID    &compID,
                 const CoeffBuf       &coeffs,
                       double*        diagRatio,
                       double*        horVerRatio );
};// END CLASS DEFINITION TrQuant

} // namespace vvenc

//! \}


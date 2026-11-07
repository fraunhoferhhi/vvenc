/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

#if defined(TARGET_SIMD_X86)  && (ENABLE_SIMD_OPT_QUANT || ENABLE_SIMD_TRAFO)
using namespace x86_simd;
#endif

typedef void FwdTrans(const TCoeff*, TCoeff*, int, int, int, int);
typedef void InvTrans(const TCoeff*, TCoeff*, int, int, int, int, const TCoeff, const TCoeff);

// ====================================================================================================================
// Class definition
// ====================================================================================================================
typedef std::pair<int, bool> TrMode;   // a transform type (mtsIdx) and whether to test it
typedef std::pair<int, int>  TrCost;
// Fixed-capacity lists, because a search builds them for every block: the transform types to try and their costs.
typedef static_vector<TrMode, NUM_TRAFO_MODES_MTS> TrModeList;
typedef static_vector<TrCost, NUM_TRAFO_MODES_MTS> TrCostList;

// The primary transform an LFNST candidate works on, for the transforms of one TU with the same residual
enum LfnstPrimary
{
  LFNST_PRIMARY_NEW,    // transform the residual
  LFNST_PRIMARY_KEEP,   // transform the residual, and keep what an LFNST of this TU will take from it
  LFNST_PRIMARY_KNOWN   // an earlier call on this TU kept it: the residual isn't read
};

/// transform and quantization class
class TrQuant
{
public:
  explicit TrQuant( bool enableOpt = true );
  ~TrQuant();

  // initialize class
  void init(
             const Quant* otherQuant,
             const int  rdoq,
             const bool bUseRDOQTS,
             const bool scalingListsEnabled,
             const bool bEnc,
             const int  thrValue            
           );

public:
  void invTransformNxN    ( TransformUnit& tu, const ComponentID compID, PelBuf& pResi, const QpParam& cQPs);
  void transformNxN       ( TransformUnit& tu, const ComponentID compID, const QpParam& cQP, TCoeff &uiAbsSum, const Ctx& ctx, const bool loadTr = false, const LfnstPrimary lfnstPrimary = LFNST_PRIMARY_NEW);
  void checktransformsNxN ( TransformUnit& tu, TrModeList *trModes, const int maxCand, const ComponentID compID = COMP_Y);
  // Called with true before transforming the residuals of one block with several LFNST indices, which leave these residuals as
  // they are, and with false afterwards. Forgets what was kept.
  void keepPrimaryCoeffs  ( const bool keep );

  void                        invTransformICT     ( const TransformUnit& tu, PelBuf& resCb, PelBuf& resCr );
  std::pair<int64_t,int64_t>  fwdTransformICT     ( const TransformUnit& tu, const PelBuf& resCb, const PelBuf& resCr, PelBuf& resC1, PelBuf& resC2, int jointCbCr = -1 );
  std::vector<int>            selectICTCandidates ( const TransformUnit& tu, CompStorage* resCb, CompStorage* resCr );

  void   setLambdas  ( const double lambdas[MAX_NUM_COMP] )   { m_quant->setLambdas( lambdas ); }
  void   selectLambda( const ComponentID compIdx )            { m_quant->selectLambda( compIdx ); }
  void   getLambdas  ( double (&lambdas)[MAX_NUM_COMP]) const { m_quant->getLambdas( lambdas ); }
  void   scaleLambda ( const double scale)                    { m_quant->scaleLambda(scale);}

  DepQuant* getQuant ()                                       { return m_quant; }

  // joint Cb-Cr residual transforms, indexed by the ICT mode -3..3 (public for the unit test)
  static const int maxAbsIctMode = 3;
  void                      (**m_invICT)(PelBuf&,PelBuf&);
  std::pair<int64_t,int64_t>(**m_fwdICT)(const PelBuf&,const PelBuf&,PelBuf&,PelBuf&);

protected:
  TCoeff*   m_plTempCoeff;
  bool      m_bEnc;
  bool      m_scalingListEnabled;
  TCoeff*   m_blk;
  TCoeff*   m_tmp;

private:
  DepQuant* m_quant;          //!< Quantizer
  TCoeff    m_tempInMatrix[48];
  TCoeff    m_lfnstFwdIn[48];     // what the forward LFNST takes from the primary transform; only xFwdLfnst writes it
  TCoeff    m_tempOutMatrix[48];
  TCoeff   *m_mtsCoeffs[NUM_TRAFO_MODES_MTS];

  // The coefficients LFNST reads from a primary transform, and the transform they came from
  struct KeptPrimaryCoeffs
  {
    TCoeff coeffs[8 * 8];
    int    trTypeHor;
    int    trTypeVer;
    bool   valid;
  };
  KeptPrimaryCoeffs m_keptPrimary[MAX_NUM_COMP + 3];   // per component, then per joint Cb-Cr mode 1 to 3
  bool              m_keepPrimary = false;

  void                      (*m_invICTMem[1+2*maxAbsIctMode])(PelBuf&,PelBuf&);
  std::pair<int64_t,int64_t>(*m_fwdICTMem[1+2*maxAbsIctMode])(const PelBuf&,const PelBuf&,PelBuf&,PelBuf&);

  void (*m_fwdLfnstNxN)( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );
  void (*m_invLfnstNxN)( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize );

  uint32_t xGetLFNSTIntraMode( const Area& tuArea, const uint32_t dirMode );
  bool     xGetTransposeFlag(uint32_t intraMode);
  void     xFwdLfnst    ( const TransformUnit &tu, const ComponentID compID, const bool loadTr, const LfnstPrimary lfnstPrimary );
  void     xInvLfnst    ( const TransformUnit &tu, const ComponentID compID);
  void     xSetTrTypes  ( const TransformUnit& tu, const ComponentID compID, const int width, const int height, int &trTypeHor, int &trTypeVer );

  // forward Transform
  void xT               (const TransformUnit& tu, const ComponentID compID, const CPelBuf& resi, CoeffBuf& dstCoeff, const int width, const int height);
  void xTOrKept         (const TransformUnit& tu, const ComponentID compID, const CPelBuf& resi, CoeffBuf& dstCoeff, const int width, const int height);

  // quantization
  void xQuant           (TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx);

  // dequantization
  void xDeQuant( const TransformUnit& tu,
                       CoeffBuf      &dstCoeff,
                 const ComponentID   &compID,
                 const QpParam       &cQP      );

  // inverse transform
  void xIT     ( const TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pCoeff, PelBuf& pResidual );
  // skipping Transform
  void xTransformSkip(const TransformUnit& tu, const ComponentID& compID, const CPelBuf& resi, TCoeff* psCoeff);
  // inverse skipping transform
  void xITransformSkip(const CCoeffBuf& plCoef, PelBuf& pResidual, const TransformUnit& tu, const ComponentID component);
  
#if defined(TARGET_SIMD_X86)  && (ENABLE_SIMD_TRAFO )
  template<X86_VEXT vext>
  void _initTrQuantX86();
  void initTrQuantX86();
#endif
};// END CLASS DEFINITION TrQuant

} // namespace vvenc

//! \}


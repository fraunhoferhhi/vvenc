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

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
/** \file     RdCost.h
    \brief    RD cost computation classes (header)
*/

#pragma once

#include "CommonDef.h"
#include "Mv.h"
#include "Unit.h"
#include "Slice.h"

#include <math.h>

//! \ingroup CommonLib
//! \{

namespace vvenc {

class DistParam;

// ====================================================================================================================
// Type definition
// ====================================================================================================================

// for function pointer
typedef Distortion (*FpDistFunc) (const DistParam&);

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// distortion parameter class
class DistParam
{
public:
  CPelBuf               org;
  CPelBuf               cur;
  FpDistFunc            distFunc;
  int                   bitDepth;
  int                   subShift;
  ComponentID           compID;
  bool                  applyWeight;     // whether weighted prediction is used or not
  Distortion            maximumDistortionForEarlyExit; /// During cost calculations, if distortion exceeds this value, cost calculations may early-terminate.
  const WPScalingParam* wpCur;           // weighted prediction scaling parameters for current ref
  const CPelBuf*        orgLuma;

  const Pel*            mask;
  int                   maskStride;
  int                   stepX;
  int                   maskStride2;

  DistParam( const CPelBuf& _org, const CPelBuf& _cur,  FpDistFunc _distFunc, int _bitDepth, int _subShift, ComponentID _compID )
  : org(_org), cur(_cur), distFunc(_distFunc), bitDepth(_bitDepth), subShift(_subShift), compID(_compID), applyWeight( false )
    ,maximumDistortionForEarlyExit( MAX_DISTORTION ), wpCur( nullptr ), orgLuma( nullptr)
  { }

  DistParam() : org(), cur(), bitDepth( 0 ), subShift( 0 ), compID( MAX_NUM_COMP ), applyWeight( false )
  ,maximumDistortionForEarlyExit( MAX_DISTORTION ), wpCur( nullptr ), orgLuma( nullptr)
  { }
};

/// RD cost computation class
class RdCost
{
private:
  // for distortion

  FpDistFunc              m_afpDistortFunc[2][DF_TOTAL_FUNCTIONS]; // [eDFunc]
  CostMode                m_costMode;
  double                  m_distortionWeight[MAX_NUM_COMP]; // only chroma values are used.
  double                  m_dLambda;
  double                  m_dLambda_unadjusted; // TODO: check is necessary
  double                  m_DistScaleUnadjusted;

  const uint32_t*         m_reshapeLumaLevelToWeightPLUT;

  uint32_t                m_signalType;
  double                  m_chromaWeight;
  int                     m_lumaBD;
  ChromaFormat            m_cf;
  double                  m_DistScale;
  double                  m_dLambdaMotionSAD;

  // for motion cost
  Mv                      m_mvPredictor;
  Mv                      m_bvPredictors[2];
  double                  m_motionLambda;
  int                     m_iCostScale;
#if IBC_VTM
  double                  m_dCostIBC;
#endif
public:
  RdCost();
  virtual ~RdCost();

  void          create();
#ifdef TARGET_SIMD_X86
  void          initRdCostX86();
  template <X86_VEXT vext>
  void          _initRdCostX86();
#endif

  void          setReshapeParams    ( const uint32_t* pPLUT, double chrWght)    { m_reshapeLumaLevelToWeightPLUT = pPLUT; m_chromaWeight = chrWght; }
  void          setDistortionWeight ( const ComponentID compID, const double distortionWeight ) { m_distortionWeight[compID] = distortionWeight; }
  void          setLambda           ( double dLambda, const BitDepths &bitDepths );
  void          setCostMode         ( CostMode m )                      { m_costMode = m; }

  double        getLambda           ( bool unadj = false )              { return unadj ? m_dLambda_unadjusted : m_dLambda; }
  double        getChromaWeight     ()                                  { return ((m_distortionWeight[COMP_Cb] + m_distortionWeight[COMP_Cr]) / 2.0); }
  double        getDistortionWeight ( const ComponentID compID )  const { return m_distortionWeight[ compID % MAX_NUM_COMP ]; }
  double        calcRdCost          ( uint64_t fracBits, Distortion distortion, bool useUnadjustedLambda = true ) const
  {
    return ( useUnadjustedLambda ? m_DistScaleUnadjusted : m_DistScale ) * double( distortion ) + double( fracBits );
  }

  void          setDistParam        ( DistParam &rcDP, const CPelBuf& org, const Pel* piRefY , int iRefStride, int bitDepth, ComponentID compID, int subShiftMode = 0, bool useHadamard = false );
  DistParam     setDistParam        ( const CPelBuf& org, const CPelBuf& cur, int bitDepth, DFunc dfunc );
  DistParam     setDistParam        ( const Pel* pOrg, const Pel* piRefY, int iOrgStride, int iRefStride, int bitDepth, ComponentID compID, int width, int height, int subShift );
  void          setDistParamGeo     ( DistParam &rcDP, const CPelBuf& org, const Pel *piRefY, int iRefStride, const Pel *mask, int iMaskStride, int stepX, int iMaskStride2, int bitDepth, ComponentID compID );

  double        getMotionLambda     ()                      const { return m_dLambdaMotionSAD; }
  void          selectMotionLambda  ()                            { m_motionLambda = getMotionLambda(); }
  void          setPredictor        ( const Mv& rcMv )            { m_mvPredictor = rcMv; }
  void          setCostScale        ( int iCostScale )            { m_iCostScale = iCostScale; }
  Distortion    getCost             ( uint32_t b )          const { return Distortion( m_motionLambda * b ); }
  // for motion cost
  static uint32_t    xGetExpGolombNumberOfBits( int iVal )
  {
    CHECKD( iVal == std::numeric_limits<int>::min(), "Wrong value" );
    unsigned uiLength2 = 1, uiTemp2 = ( iVal <= 0 ) ? ( unsigned( -iVal ) << 1 ) + 1 : unsigned( iVal << 1 );

    while( uiTemp2 > MAX_CU_SIZE )
    {
      uiLength2 += ( MAX_CU_DEPTH << 1 );
      uiTemp2  >>=   MAX_CU_DEPTH;
    }

    return uiLength2 + ( Log2(uiTemp2) << 1 );
  }
  Distortion     getCostOfVectorWithPredictor( const int x, const int y, const unsigned imvShift )  { return Distortion( m_motionLambda * getBitsOfVectorWithPredictor(x, y, imvShift )); }
  uint32_t       getBitsOfVectorWithPredictor( const int x, const int y, const unsigned imvShift )  { return xGetExpGolombNumberOfBits(((x << m_iCostScale) - m_mvPredictor.hor)>>imvShift) + xGetExpGolombNumberOfBits(((y << m_iCostScale) - m_mvPredictor.ver)>>imvShift); }

  void           saveUnadjustedLambda ();
  void           setReshapeInfo       ( uint32_t type, int lumaBD, ChromaFormat cf )   { m_signalType = type; m_lumaBD = lumaBD; m_cf = cf; }
#if IBC_VTM
  void          setPredictorsIBC(Mv* pcMv)
  {
    for (int i = 0; i < 2; i++)
    {
      m_bvPredictors[i] = pcMv[i];
    }
  }
  void           getMotionCostIBC(int add) { m_dCostIBC = m_dLambdaMotionSAD + add; }
  Distortion     getBvCostMultiplePredsIBC(int x, int y, bool useIMV);
#endif
private:
         Distortion xGetSSE_WTD       ( const DistParam& pcDtParam ) const;

  static Distortion xGetSSE           ( const DistParam& pcDtParam );
  static Distortion xGetSSE4          ( const DistParam& pcDtParam );
  static Distortion xGetSSE8          ( const DistParam& pcDtParam );
  static Distortion xGetSSE16         ( const DistParam& pcDtParam );
  static Distortion xGetSSE32         ( const DistParam& pcDtParam );
  static Distortion xGetSSE64         ( const DistParam& pcDtParam );
  static Distortion xGetSSE128        ( const DistParam& pcDtParam );


  static Distortion xGetSAD           ( const DistParam& pcDtParam );
  static Distortion xGetSAD4          ( const DistParam& pcDtParam );
  static Distortion xGetSAD8          ( const DistParam& pcDtParam );
  static Distortion xGetSAD16         ( const DistParam& pcDtParam );
  static Distortion xGetSAD32         ( const DistParam& pcDtParam );
  static Distortion xGetSAD64         ( const DistParam& pcDtParam );
  static Distortion xGetSAD128        ( const DistParam& pcDtParam );
  static Distortion xGetSADwMask      ( const DistParam &pcDtParam );
  
  static Distortion xCalcHADs2x2      ( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur );
  static Distortion xGetHAD2SADs      ( const DistParam& pcDtParam );
  static Distortion xGetHADs          ( const DistParam& pcDtParam );

#ifdef TARGET_SIMD_X86
  template<X86_VEXT vext>
  static Distortion xGetSSE_SIMD    ( const DistParam& pcDtParam );
  template<int iWidth, X86_VEXT vext>
  static Distortion xGetSSE_NxN_SIMD( const DistParam& pcDtParam );

  template<X86_VEXT vext>
  static Distortion xGetSAD_SIMD    ( const DistParam& pcDtParam );
  template<int iWidth, X86_VEXT vext>
  static Distortion xGetSAD_NxN_SIMD( const DistParam& pcDtParam );

  template<X86_VEXT vext>
  static Distortion xGetHADs_SIMD   ( const DistParam& pcDtParam );
  template<X86_VEXT vext>
  static Distortion xGetHAD2SADs_SIMD( const DistParam &rcDtParam );

  template<X86_VEXT vext> 
  static Distortion xGetSADwMask_SIMD( const DistParam &pcDtParam );
#endif

#if IBC_VTM
  unsigned int   getBitsMultiplePredsIBC(int x, int y, bool useIMV);
  unsigned int   getIComponentBitsIBC(int val);
#endif
public:

  Distortion   getDistPart( const CPelBuf& org, const CPelBuf& cur, int bitDepth, const ComponentID compId, DFunc eDFunc, const CPelBuf* orgLuma = NULL );

};// END CLASS DEFINITION RdCost

} // namespace vvenc

//! \}


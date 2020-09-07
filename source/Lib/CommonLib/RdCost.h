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
  const double*           m_lumaLevelToWeightPLUT; //ALF

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

  double                  m_dCost; // for ibc
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

public:

  Distortion   getDistPart( const CPelBuf& org, const CPelBuf& cur, int bitDepth, const ComponentID compId, DFunc eDFunc, const CPelBuf* orgLuma = NULL );

};// END CLASS DEFINITION RdCost

} // namespace vvenc

//! \}


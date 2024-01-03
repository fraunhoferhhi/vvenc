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

/** \file     RdCostARM.h
    \brief    RD cost computation class, SIMD version
*/

#include <math.h>
#include <limits>

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "../RdCost.h"

namespace vvenc
{

#ifdef TARGET_SIMD_ARM
#if __ARM_ARCH >= 8

template<ARM_VEXT vext, bool isWdt16>
Distortion xGetSAD_MxN_SIMD( const DistParam& rcDtParam )
{
  if( rcDtParam.bitDepth > 10 )
    return isWdt16 ? RdCost::xGetSAD16( rcDtParam ) : RdCost::xGetSAD8( rcDtParam );

  //  assert( rcDtParam.iCols == iWidth);
  const short*    pSrc1       = (const short*) rcDtParam.org.buf;
  const short*    pSrc2       = (const short*) rcDtParam.cur.buf;
  const int       iRows       = rcDtParam.org.height;
  const int       iSubShift   = rcDtParam.subShift;
  const ptrdiff_t iStrideSrc1 = rcDtParam.org.stride << iSubShift;
  const ptrdiff_t iStrideSrc2 = rcDtParam.cur.stride << iSubShift;

  uint32_t uiSum = 0;

  int16x8_t vsum16 = vdupq_n_s16( 0 );

  for( int i = 0; i < ( iRows >> 3 ); i++ )
  {
    // 0
    int16x8_t vsrc1 = vld1q_s16( pSrc1 );
    int16x8_t vsrc2 = vld1q_s16( pSrc2 );

    vsum16 = vabaq_s16( vsum16, vsrc1, vsrc2 );

    if( isWdt16 )
    {
      vsrc1 = vld1q_s16( pSrc1 + 8 );
      vsrc2 = vld1q_s16( pSrc2 + 8 );

      vsum16 = vabaq_s16( vsum16, vsrc1, vsrc2 );
    }

    pSrc1 += iStrideSrc1;
    pSrc2 += iStrideSrc2;

    // 1
    vsrc1 = vld1q_s16( pSrc1 );
    vsrc2 = vld1q_s16( pSrc2 );

    vsum16 = vabaq_s16( vsum16, vsrc1, vsrc2 );

    if( isWdt16 )
    {
      vsrc1 = vld1q_s16( pSrc1 + 8 );
      vsrc2 = vld1q_s16( pSrc2 + 8 );

      vsum16 = vabaq_s16( vsum16, vsrc1, vsrc2 );
    }

    pSrc1 += iStrideSrc1;
    pSrc2 += iStrideSrc2;

    // 2
    vsrc1 = vld1q_s16( pSrc1 );
    vsrc2 = vld1q_s16( pSrc2 );

    vsum16 = vabaq_s16( vsum16, vsrc1, vsrc2 );

    if( isWdt16 )
    {
      vsrc1 = vld1q_s16( pSrc1 + 8 );
      vsrc2 = vld1q_s16( pSrc2 + 8 );

      vsum16 = vabaq_s16( vsum16, vsrc1, vsrc2 );
    }

    pSrc1 += iStrideSrc1;
    pSrc2 += iStrideSrc2;

    // 3
    vsrc1 = vld1q_s16( pSrc1 );
    vsrc2 = vld1q_s16( pSrc2 );

    vsum16 = vabaq_s16( vsum16, vsrc1, vsrc2 );

    if( isWdt16 )
    {
      vsrc1 = vld1q_s16( pSrc1 + 8 );
      vsrc2 = vld1q_s16( pSrc2 + 8 );

      vsum16 = vabaq_s16( vsum16, vsrc1, vsrc2 );
    }

    pSrc1 += iStrideSrc1;
    pSrc2 += iStrideSrc2;
  }

  uiSum = vaddlvq_s16( vsum16 );
  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
}

template<ARM_VEXT vext, bool isCalCentrePos>
void xGetSADX5_16xN_SIMDImp( const DistParam& rcDtParam, Distortion* cost )
{
  int        i, j;
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf - 4;
  int        height     = rcDtParam.org.height;
  int        iSubShift  = rcDtParam.subShift;
  int        iSubStep   = ( 1 << iSubShift );
  ptrdiff_t  iStrideCur = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t  iStrideOrg = rcDtParam.org.stride * iSubStep;

  int16x8_t sum0 = vdupq_n_s16( 0 );
  int16x8_t sum1 = vdupq_n_s16( 0 );
  int16x8_t sum2 = vdupq_n_s16( 0 );
  int16x8_t sum3 = vdupq_n_s16( 0 );
  int16x8_t sum4 = vdupq_n_s16( 0 );

  for( i = 0; i < height; i += iSubStep )
  {
    for( j = 0; j < 16; j += 8 )
    {
      int16x8_t s0 = vld1q_s16( piOrg + j + 0 );
      int16x8_t s1 = vld1q_s16( piCur + j + 0 );
      int16x8_t s2 = vcombine_s16( vld1_s16( piOrg + j + 8 ), vdup_n_s16( 0 ) );
      int16x8_t s3 = vcombine_s16( vld1_s16( piCur + j + 8 ), vdup_n_s16( 0 ) );

      int16x8_t org0, org1, org2, org3, org4;
      org0 = s0;
      org1 = vextq_s16( s0, s2, 1 );
      if( isCalCentrePos )
        org2 = vextq_s16( s0, s2, 2 );
      org3 = vextq_s16( s0, s2, 3 );
      org4 = vextq_s16( s0, s2, 4 );

      int16x8_t cur0, cur1, cur2, cur3, cur4;
      cur4 = s1;
      cur0 = vextq_s16( s1, s3, 4 );
      cur1 = vextq_s16( s1, s3, 3 );
      if( isCalCentrePos )
        cur2 = vextq_s16( s1, s3, 2 );
      cur3 = vextq_s16( s1, s3, 1 );

      sum0 = vabaq_s16( sum0, org0, cur0 );   // komplett insane
      sum1 = vabaq_s16( sum1, org1, cur1 );
      if( isCalCentrePos )
        sum2 = vabaq_s16( sum2, org2, cur2 );
      sum3 = vabaq_s16( sum3, org3, cur3 );
      sum4 = vabaq_s16( sum4, org4, cur4 );
    }

    INCY( piOrg, iStrideOrg );
    INCY( piCur, iStrideCur );
  }

  int32x4_t sum = { vaddlvq_s16( sum0 ), vaddlvq_s16( sum1 ), vaddlvq_s16( sum3 ), vaddlvq_s16( sum4 ) };

  int32x4_t sumTwo;
  if( isCalCentrePos )
    sumTwo = vdupq_n_s32( vaddlvq_s16( sum2 ) );

  // vshlq_n_s32 doesnt work because iSubShift ist not a const.
  sum = vshlq_s32( sum, vdupq_n_s32( iSubShift ) );
  if( isCalCentrePos )
    sumTwo = vshlq_s32( sumTwo, vdupq_n_s32( iSubShift ) );

  sum = vshrq_n_s32( sum, ( 1 + ( DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth ) ) ) );
  if( isCalCentrePos )
    sumTwo = vshrq_n_s32( sumTwo, ( 1 + ( DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth ) ) ) );

  vst1q_lane_u64( (uint64_t*) &cost[ 0 ], (uint64x2_t) sum, 0 );
  if( isCalCentrePos )
    cost[ 2 ] = vgetq_lane_s32( sumTwo, 0 );
  vst1q_lane_u64( (uint64_t*) &cost[ 3 ], (uint64x2_t) sum, 1 );
}

template <ARM_VEXT vext>
void RdCost::xGetSADX5_16xN_SIMD(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos)
{
  if( rcDtParam.bitDepth > 10 )
  {
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }

  if( isCalCentrePos )
    xGetSADX5_16xN_SIMDImp<vext, true>( rcDtParam, cost );
  else
    xGetSADX5_16xN_SIMDImp<vext, false>( rcDtParam, cost );
}

template<ARM_VEXT vext>
void RdCost::_initRdCostARM()
{
  m_afpDistortFunc[0][DF_SAD8   ] = xGetSAD_MxN_SIMD<vext, false>;
  m_afpDistortFunc[0][DF_SAD16  ] = xGetSAD_MxN_SIMD<vext, true>;
	m_afpDistortFuncX5[1] = xGetSADX5_16xN_SIMD<vext>;
}

#else    // !__ARM_ARCH >= 8

template<ARM_VEXT vext>
void RdCost::_initRdCostARM()
{}

#endif   // !__ARM_ARCH >= 8

template void RdCost::_initRdCostARM<SIMDARM>();

#endif   // TARGET_SIMD_ARM

}   // namespace vvenc

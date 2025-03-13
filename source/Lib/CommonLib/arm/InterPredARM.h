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
/** \file     InterPredX86.h
    \brief    SIMD for InterPrediction
*/

//! \ingroup CommonLib
//! \{


#include "CommonDefARM.h"
#include "Rom.h"
#include "InterPrediction.h"



//! \ingroup CommonLib
//! \{

namespace vvenc {

static inline int rightShiftMSB(int numer, int denom)
{
  int shiftIdx = bit_scan_reverse(denom);
  return (numer >> shiftIdx);
}

#if ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_ARM )
#if __ARM_ARCH >= 8


template< ARM_VEXT vext >
static inline void calcBIOSums_Neon(const Pel* srcY0Tmp, const Pel* srcY1Tmp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, const int widthG, const int bitDepth, int limit, int &tmpx, int &tmpy)
{
  const int srcStride = widthG + 2;
  int16x8_t sumAbsGXTmp    = vdupq_n_s16(0);
  int16x8_t sumDIXTmp      = vdupq_n_s16(0);
  int16x8_t sumAbsGYTmp    = vdupq_n_s16(0);
  int16x8_t sumDIYTmp      = vdupq_n_s16(0);
  int16x8_t sumSignGyGxTmp = vdupq_n_s16(0);
  int16_t vals[8] = {1, 1, 1, 1, 1, 1, 0, 0};
  int16x8_t x = vld1q_s16(vals);

  for (int y = 0; y < 3; y++)
  {
    int16x8_t shiftSrcY0Tmp = vshrq_n_s16(vld1q_s16((int16_t*)(srcY0Tmp)), 4);
    int16x8_t shiftSrcY1Tmp = vshrq_n_s16(vld1q_s16((int16_t*)(srcY1Tmp)), 4);

    int16x8_t loadGradX0    = vld1q_s16((int16_t*)(gradX0));
    int16x8_t loadGradX1    = vld1q_s16((int16_t*)(gradX1));
    int16x8_t loadGradY0    = vld1q_s16((int16_t*)(gradY0));
    int16x8_t loadGradY1    = vld1q_s16((int16_t*)(gradY1));
    int16x8_t subTemp1      = vsubq_s16(shiftSrcY1Tmp, shiftSrcY0Tmp);
    int16x8_t packTempX     =  vshrq_n_s16( vaddq_s16(loadGradX0, loadGradX1), 1 );
    int16x8_t packTempY     =  vshrq_n_s16( vaddq_s16(loadGradY0, loadGradY1), 1 );
    int16x8_t gX            = vabsq_s16(packTempX);
    int16x8_t gY            = vabsq_s16(packTempY);
    int16x8_t dIX           = vmulq_s16(subTemp1,vreinterpretq_s16_u16( vsubq_u16( vcleq_s16(packTempX, vdupq_n_s16(0)), vcgeq_s16(packTempX,vdupq_n_s16(0)) )));
    int16x8_t dIY           = vmulq_s16(subTemp1,vreinterpretq_s16_u16( vsubq_u16( vcleq_s16(packTempY, vdupq_n_s16(0)), vcgeq_s16(packTempY,vdupq_n_s16(0)) )));
    int16x8_t signGY_GX     = vmulq_s16(packTempX,vreinterpretq_s16_u16( vsubq_u16( vcleq_s16(packTempY, vdupq_n_s16(0)), vcgeq_s16(packTempY,vdupq_n_s16(0)) )));
    
    sumAbsGXTmp     = vaddq_s16(sumAbsGXTmp, gX);
    sumAbsGYTmp     = vaddq_s16(sumAbsGYTmp, gY);
    sumDIXTmp       = vaddq_s16(sumDIXTmp, dIX);
    sumDIYTmp       = vaddq_s16(sumDIYTmp, dIY);
    sumSignGyGxTmp  = vaddq_s16(sumSignGyGxTmp, signGY_GX);

    srcY0Tmp += srcStride;
    srcY1Tmp += srcStride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;

    shiftSrcY0Tmp = vshrq_n_s16(vld1q_s16((int16_t*)(srcY0Tmp)), 4);
    shiftSrcY1Tmp = vshrq_n_s16(vld1q_s16((int16_t*)(srcY1Tmp)), 4);

    loadGradX0    = vld1q_s16((int16_t*)(gradX0));
    loadGradX1    = vld1q_s16((int16_t*)(gradX1));
    loadGradY0    = vld1q_s16((int16_t*)(gradY0));
    loadGradY1    = vld1q_s16((int16_t*)(gradY1));
    subTemp1      = vsubq_s16(shiftSrcY1Tmp, shiftSrcY0Tmp);
    packTempX     =  vshrq_n_s16( vaddq_s16(loadGradX0, loadGradX1), 1 );
    packTempY     =  vshrq_n_s16( vaddq_s16(loadGradY0, loadGradY1), 1 );

    gX            = vabsq_s16(packTempX);
    gY            = vabsq_s16(packTempY);
    
    dIX           = vmulq_s16(subTemp1,vreinterpretq_s16_u16( vsubq_u16( vcleq_s16(packTempX, vdupq_n_s16(0)), vcgeq_s16(packTempX,vdupq_n_s16(0)) )));
    dIY           = vmulq_s16(subTemp1,vreinterpretq_s16_u16( vsubq_u16( vcleq_s16(packTempY, vdupq_n_s16(0)), vcgeq_s16(packTempY,vdupq_n_s16(0)) )));
    signGY_GX     = vmulq_s16(packTempX,vreinterpretq_s16_u16( vsubq_u16( vcleq_s16(packTempY, vdupq_n_s16(0)), vcgeq_s16(packTempY,vdupq_n_s16(0)) )));
  
    sumAbsGXTmp     = vaddq_s16(sumAbsGXTmp, gX);
    sumAbsGYTmp     = vaddq_s16(sumAbsGYTmp, gY);
    sumDIXTmp       = vaddq_s16(sumDIXTmp, dIX);
    sumDIYTmp       = vaddq_s16(sumDIYTmp, dIY);
    sumSignGyGxTmp  = vaddq_s16(sumSignGyGxTmp, signGY_GX);

    srcY0Tmp += srcStride;
    srcY1Tmp += srcStride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
  }

  int sumAbsGX = vaddvq_s16(vmulq_s16( sumAbsGXTmp, x));
  int sumAbsGY = vaddvq_s16(vmulq_s16( sumAbsGYTmp, x));
  int sumDIX   = vaddvq_s16(vmulq_s16( sumDIXTmp, x));
  int sumDIY   = vaddvq_s16(vmulq_s16( sumDIYTmp, x));
  int sumSignGY_GX  = vaddvq_s16(vmulq_s16( sumSignGyGxTmp, x));

  tmpx = sumAbsGX == 0 ? 0 : rightShiftMSB( sumDIX << 2, sumAbsGX );
  tmpx = Clip3( -limit, limit, tmpx );

  int mainsGxGy = sumSignGY_GX >> 12;
  int secsGxGy  = sumSignGY_GX & ( ( 1 << 12 ) - 1 );
  int tmpData   = tmpx * mainsGxGy;
  tmpData       = ( ( tmpData << 12 ) + tmpx * secsGxGy ) >> 1;
  tmpy = sumAbsGY == 0 ? 0 : rightShiftMSB( ( ( sumDIY << 2 ) - tmpData ), sumAbsGY );
  tmpy = Clip3( -limit, limit, tmpy );
}

template<ARM_VEXT vext>
static inline void addBIOAvg4_Neon(const int16_t* src0, const int16_t* src1, int16_t* dst, ptrdiff_t dstStride, const int16_t* gradX0, const int16_t* gradX1, const int16_t* gradY0, const int16_t* gradY1, ptrdiff_t widthG, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
  const ptrdiff_t src0Stride = widthG + 2;
  const ptrdiff_t src1Stride = widthG + 2;
  const ptrdiff_t gradStride = widthG;
  int32x4_t mm_offset  = vdupq_n_s32( offset );
  int16x4_t vibdimin   = vdup_n_s16( clpRng.min() );
  int16x4_t vibdimax   = vdup_n_s16( clpRng.max() );

  int16x4_t mm_a;
  int16x4_t mm_b;
  int32x4_t mm_sum;
  int16x4_t mm_sum3;

  for( int y = 0; y < 2; y++)
  {
    mm_sum = vdupq_n_s32(0);

    mm_a   = vsub_s16 ( vld1_s16( (const int16_t *) gradX0 ), vld1_s16( (const int16_t *) gradX1 ) );
    mm_b   = vsub_s16 ( vld1_s16( (const int16_t *) gradY0 ), vld1_s16( (const int16_t *) gradY1 ) );

    mm_sum   = vmlal_n_s16      (mm_sum, mm_a, tmpx);
    mm_sum   = vmlal_n_s16      (mm_sum, mm_b, tmpy);
    mm_sum = vaddq_s32      ( vaddw_s16( mm_sum, vld1_s16( (const int16_t *) ( src0 ) ) ), vaddw_s16( mm_offset, vld1_s16( (const int16_t *) ( src1 ) )) );
    mm_sum3 = vmin_s16 (vibdimax, vmax_s16(vibdimin, vqmovn_s32(vshlq_s32( mm_sum, vdupq_n_s32(-1*shift) ))));    

    vst1_s16((int16_t *)dst, mm_sum3);

    dst += dstStride;
    src0 += src0Stride; 
    src1 += src1Stride; 
    gradX0 += gradStride;
    gradX1 += gradStride;
    gradY0 += gradStride;
    gradY1 += gradStride;

    mm_sum = vdupq_n_s32(0);

    mm_a   = vsub_s16 ( vld1_s16( (const int16_t *) gradX0 ), vld1_s16( (const int16_t *) gradX1 ) );
    mm_b   = vsub_s16 ( vld1_s16( (const int16_t *) gradY0 ), vld1_s16( (const int16_t *) gradY1 ) );

    mm_sum   = vmlal_n_s16      (mm_sum, mm_a, tmpx);
    mm_sum   = vmlal_n_s16      (mm_sum, mm_b, tmpy);
    mm_sum = vaddq_s32      ( vaddw_s16( mm_sum, vld1_s16( (const int16_t *) ( src0 ) ) ), vaddw_s16( mm_offset, vld1_s16( (const int16_t *) ( src1 ) )) );
    mm_sum3 = vmin_s16 (vibdimax, vmax_s16(vibdimin, vqmovn_s32(vshlq_s32( mm_sum, vdupq_n_s32(-1*shift) ))));    

    vst1_s16((int16_t *)dst, mm_sum3);

    dst += dstStride;
    src0 += src0Stride; 
    src1 += src1Stride; 
    gradX0 += gradStride;
    gradX1 += gradStride;
    gradY0 += gradStride;
    gradY1 += gradStride;
  }
}

template< ARM_VEXT vext>
void BiOptFlowCoreARMSIMD( const Pel* srcY0,
                        const Pel* srcY1,
                        const Pel* gradX0,
                        const Pel* gradX1,
                        const Pel* gradY0,
                        const Pel* gradY1,
                        const int  width,
                        const int  height,
                              Pel* dstY,
                        const ptrdiff_t dstStride,
                        const int  shiftNum,
                        const int  offset,
                        const int  limit,
                        const ClpRng& clpRng,
                        const int bitDepth )
{
  const int widthG        = width  + 2 * BDOF_EXTEND_SIZE;
  const int stridePredMC  = widthG + 2;
        int offsetPos     = widthG * BDOF_EXTEND_SIZE + BDOF_EXTEND_SIZE;
  const int xUnit         = ( width  >> 2 );
  const int yUnit         = ( height >> 2 );

  const Pel* srcY0Temp;
  const Pel* srcY1Temp;
        Pel *dstY0;
  
  int OffPos;
  int OffPad = 0;

  for( int yu = 0; yu < yUnit; yu++, srcY0 += ( stridePredMC << 2 ), srcY1 += ( stridePredMC << 2 ), dstY += ( dstStride << 2 ), offsetPos += ( widthG << 2 ) )
  {
    srcY0Temp = srcY0;
    srcY1Temp = srcY1;
    dstY0     = dstY;
    
    OffPos = offsetPos;
    OffPad = ( ( yu * widthG ) << 2 );
    for( int xu = 0; xu < xUnit; xu++, srcY0Temp += 4, srcY1Temp += 4, dstY0 += 4, OffPos += 4, OffPad += 4 )
    {
      int tmpx, tmpy;

      calcBIOSums_Neon<vext>( srcY0Temp, srcY1Temp, gradX0 + OffPad, gradX1 + OffPad, gradY0 + OffPad, gradY1 + OffPad, widthG, bitDepth, limit, tmpx, tmpy );

      addBIOAvg4_Neon<vext> ( srcY0Temp + stridePredMC + 1, srcY1Temp + stridePredMC + 1, dstY0, dstStride, gradX0 + OffPos, gradX1 + OffPos, gradY0 + OffPos, gradY1 + OffPos, widthG, tmpx, tmpy, shiftNum, offset, clpRng );
    }  
  }  
}


template<ARM_VEXT vext>
void InterPredInterpolation::_initInterPredictionARM()
{
  xFpBiDirOptFlow     = BiOptFlowCoreARMSIMD<vext>;
}

#else

template<ARM_VEXT vext>
void InterPredInterpolation::_initInterPredictionARM()
{}
#endif

template void InterPredInterpolation::_initInterPredictionARM<SIMDARM>();

#endif
} // namespace vvenc

//! \}

// #endif // TARGET_SIMD_X86
//! \}

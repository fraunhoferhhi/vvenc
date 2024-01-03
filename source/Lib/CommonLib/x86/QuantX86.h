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
/**
 * \file
 * \brief Implementation of quantization functions
 */
//#define USE_AVX2
// ====================================================================================================================
// Includes
// ====================================================================================================================

#pragma once

#include "CommonDefX86.h"
#include "Rom.h"
#include "QuantRDOQ2.h"

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {

#define cond_mm_prefetch(a,b) _mm_prefetch(a,b)
//#define cond_mm_prefetch(a,b)

static constexpr unsigned short levmask[16] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0,0,0,0,0,0};
template<X86_VEXT vext>
static void DeQuantCoreSIMD(const int maxX,const int maxY,const int scale,const TCoeffSig *const piQCoef,const size_t piQCfStride,TCoeff *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum)
{
  // TODO: TCoeffSig!!!
  const int inputMinimum = -(inputMaximum+1);
  const TCoeff transformMinimum = -(transformMaximum+1);
  const int width = maxX+1;

  __m128i vlevmask;
  if (maxX<7)
    vlevmask = _mm_loadu_si128( ( __m128i const * )&levmask[7-maxX] );
  else
    vlevmask = _mm_set_epi64x(0xffffffffffffffff,0xffffffffffffffff);

  if (rightShift>0)
  {
    const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);

    __m128i v_max =  _mm_set1_epi16 ((short)inputMaximum);
    __m128i v_min =  _mm_set1_epi16 ((short)inputMinimum);
    __m128i v_Tmax =  _mm_set1_epi32 ((short)transformMaximum);
    __m128i v_Tmin =  _mm_set1_epi32 ((short)transformMinimum);
    __m128i v_scale = _mm_set1_epi16 ((short)scale);
    __m128i v_add = _mm_set1_epi32 (iAdd);
    __m128i v_rshift = _mm_set1_epi64x (rightShift);

    if (maxX<4)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = maxX == 1 ? _mm_set1_epi32( *( ( int const* ) & piQCoef[y * piQCfStride] ) ) : _mm_loadl_epi64( ( __m128i const* ) &piQCoef[y * piQCfStride] );
        v_level = _mm_and_si128(v_level,vlevmask);
        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level =  _mm_add_epi32(v_level,v_add);
        v_level = _mm_sra_epi32(v_level,v_rshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        if( maxX == 1 )
          _mm_storel_epi64( (__m128i*)(piCoef + y * width), v_level );
        else
          _mm_storeu_si128( (__m128i*)(piCoef + y * width), v_level );
      }
    }
    else
    {
      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=8)
        {
          __m128i v_level = _mm_loadu_si128( ( __m128i const* ) &piQCoef[x + y * piQCfStride] );
          v_level = _mm_and_si128(v_level,vlevmask);
          v_level = _mm_max_epi16 (v_level, v_min);
          v_level = _mm_min_epi16 (v_level, v_max);
          __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
          __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

          v_level = _mm_unpacklo_epi16(v_low,v_high);
          v_level =  _mm_add_epi32(v_level,v_add);
          v_level = _mm_sra_epi32(v_level,v_rshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+x+y*width ), v_level );

          v_level = _mm_unpackhi_epi16(v_low,v_high);
          v_level =  _mm_add_epi32(v_level,v_add);
          v_level = _mm_sra_epi32(v_level,v_rshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+4+x+y*width ), v_level );
        }
      }
    }
  }
  else  // rightshift <0
  {
    __m128i v_max =  _mm_set1_epi16 ((short)inputMaximum);
    __m128i v_min =  _mm_set1_epi16 ((short)inputMinimum);
    __m128i v_Tmax =  _mm_set1_epi32 ((short)transformMaximum);
    __m128i v_Tmin =  _mm_set1_epi32 ((short)transformMinimum);
    __m128i v_scale = _mm_set1_epi16 ((short)scale);
    __m128i v_lshift = _mm_set1_epi64x (-rightShift);

    if (maxX<4)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = maxX == 1 ? _mm_set1_epi32( *( ( int const* ) & piQCoef[y * piQCfStride] ) ) : _mm_loadl_epi64( ( __m128i const* ) &piQCoef[y * piQCfStride] );
        v_level = _mm_and_si128(v_level,vlevmask);

        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level = _mm_sll_epi32(v_level,v_lshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);

        if( maxX == 1 )
        {
          _mm_storel_epi64( (__m128i*)(piCoef + y * width), v_level );
        }
        else
          _mm_storeu_si128(( __m128i * )(piCoef+y*width ), v_level );
      }
    }
    else
    {
      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=8)
        {
          __m128i v_level = _mm_loadu_si128( ( __m128i const* ) &piQCoef[x + y * piQCfStride] );
          v_level = _mm_and_si128(v_level,vlevmask);
          v_level = _mm_max_epi16 (v_level, v_min);
          v_level = _mm_min_epi16 (v_level, v_max);
          __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
          __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

          v_level = _mm_unpacklo_epi16(v_low,v_high);
          v_level = _mm_sll_epi32(v_level,v_lshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+x+y*width ), v_level );

          v_level = _mm_unpackhi_epi16(v_low,v_high);
          v_level = _mm_sll_epi32(v_level,v_lshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+4+x+y*width ), v_level );
        }
      }
    }
  }
}

template<X86_VEXT vext>
static void QuantCoreSIMD(const TransformUnit tu, const ComponentID compID, const CCoeffBuf& piCoef,CoeffSigBuf piQCoef,TCoeff &uiAbsSum, int &lastScanPos,TCoeff *deltaU,const int defaultQuantisationCoefficient,const int iQBits,const int64_t iAdd,const TCoeff entropyCodingMinimum,const TCoeff entropyCodingMaximum,const bool signHiding, const TCoeff m_thrVal)
{
  CoeffCodingContext cctx( tu, compID, signHiding );

  const CompArea &rect      = tu.blocks[compID];
  const uint32_t uiWidth    = rect.width;
  const uint32_t uiHeight   = rect.height;
  const uint32_t log2CGSize = cctx.log2CGSize();

  uiAbsSum = 0;

  const int iCGSize   = 1 << log2CGSize;

  const uint32_t lfnstIdx = tu.cu->lfnstIdx;
  const int iCGNum   = lfnstIdx > 0 ? 1 : std::min<int>(JVET_C0024_ZERO_OUT_TH, uiWidth) * std::min<int>(JVET_C0024_ZERO_OUT_TH, uiHeight) >> cctx.log2CGSize();
  int       iScanPos = ( iCGNum << log2CGSize ) - 1;

  if( lfnstIdx > 0 && ( ( uiWidth == 4 && uiHeight == 4 ) || ( uiWidth == 8 && uiHeight == 8 ) ) )
  {
    iScanPos = 7;
  }

  // Find first non-zero coeff
  for( ; iScanPos > 0; iScanPos-- )
  {
    uint32_t uiBlkPos = cctx.blockPos( iScanPos );
    if( piCoef.buf[uiBlkPos] )
      break;
  }

  //////////////////////////////////////////////////////////////////////////
  //  Loop over sub-sets (coefficient groups)
  //////////////////////////////////////////////////////////////////////////
  
  TCoeff thres = 0, useThres = 0;
  
  if( iQBits )
    thres = TCoeff( ( int64_t( m_thrVal ) << ( iQBits - 1 ) ) );
  else
    thres = TCoeff( ( int64_t( m_thrVal >> 1 ) << iQBits ) );

  useThres = thres / ( defaultQuantisationCoefficient << 2 );

  const bool is4x4sbb = log2CGSize == 4 && cctx.log2CGWidth() == 2;

  int subSetId = iScanPos >> log2CGSize;
  // if more than one 4x4 coding subblock is available, use SIMD to find first subblock with coefficient larger than threshold
  if( is4x4sbb && iScanPos >= 16 )
  {
    for( ; subSetId >= 1; subSetId-- )
    {
      // move the pointer to the beginning of the current subblock
      const int iScanPosinCG = iScanPos & ( iCGSize - 1 );
      const int firstTestPos = iScanPos - iScanPosinCG;
      uint32_t  uiBlkPos     = cctx.blockPos( firstTestPos );

      const __m128i xdfTh = _mm_set1_epi32( useThres );

      // read first line of the subblock and check for coefficients larger than the threshold
      // assumming the subblocks are dense 4x4 blocks in raster scan order with the stride of tuPars.m_width
      __m128i xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &piCoef.buf[uiBlkPos] ) );
      __m128i xdf = _mm_cmpgt_epi32( xl0, xdfTh );

      // same for the next line in the subblock
      uiBlkPos += uiWidth;
      xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &piCoef.buf[uiBlkPos] ) );
      xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

      // and the third line
      uiBlkPos += uiWidth;
      xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &piCoef.buf[uiBlkPos] ) );
      xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

      // and the last line
      uiBlkPos += uiWidth;
      xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &piCoef.buf[uiBlkPos] ) );
      xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

      if( _mm_testz_si128( xdf, xdf ) )
      {
        iScanPos -= iScanPosinCG + 1;
        continue;
      }
      else
      {
        break;
      }
    }
  }

  const int qBits8 = iQBits - 8;
  piQCoef.memset( 0 );

  lastScanPos = iScanPos;

  if( is4x4sbb && ( iScanPos & 15 ) == 15 )
  {
#if defined( USE_AVX2 ) && 0 // sometimes has undefined behavior
    if( vext >= AVX2 )
    {
      const __m256i vNull   = _mm256_setzero_si256();
      const __m256i vQuantCoeff = _mm256_set1_epi32(defaultQuantisationCoefficient);
      const __m256i vAdd    = _mm256_set1_epi64x(iAdd);
      const __m256i vMax    = _mm256_set1_epi32(entropyCodingMaximum);
      const __m256i vMin    = _mm256_set1_epi32(entropyCodingMinimum);
      const __m256i vMask   = _mm256_set_epi32( 0, -1, 0, -1, 0, -1, 0, -1 );
      __m256i vAbsSum = vNull;

      for( subSetId = iScanPos >> log2CGSize; subSetId >= 0; subSetId-- )
      {
        int uiBlockPos = cctx.blockPos( subSetId << log2CGSize );

        for( int line = 0; line < 4; line += 2, uiBlockPos += ( 2 * uiWidth ) )
        {
          __m256i  vLevel = _mm256_castsi128_si256 (         _mm_loadu_si128((__m128i *)&piCoef.buf[uiBlockPos]           )    ); // coeff7,coeff6,coeff5,coeff4,coeff3,coeff2,coeff1,coeff0,
                   vLevel = _mm256_inserti128_si256( vLevel, _mm_loadu_si128((__m128i *)&piCoef.buf[uiBlockPos + uiWidth] ), 1 ); // coeff7,coeff6,coeff5,coeff4,coeff3,coeff2,coeff1,coeff0,
          __m256i vSign = _mm256_cmpgt_epi32 (vNull,vLevel);                            // sign3,sign2,sign1,sign0 FFFF or 0000
          vLevel = _mm256_abs_epi32 (vLevel);
          __m256i vdeltaU0 = _mm256_mul_epu32(vLevel,vQuantCoeff);                      // Tmp2,Tmp0
          __m256i  vdeltaU1 = _mm256_srli_si256(vLevel,4);                              // abs(0,vLevel3,vLevel2,vLevel1)
          vdeltaU1 = _mm256_mul_epu32(vdeltaU1,vQuantCoeff);                            // Tmp3,Tmp1
          __m256i vTmpLevel_0 = _mm256_add_epi64(vdeltaU0,vAdd);
          __m256i vTmpLevel_1 = _mm256_add_epi64(vdeltaU1,vAdd);
          vTmpLevel_0 = _mm256_srli_epi64(vTmpLevel_0,iQBits);                          // Int32 Tmp2,Tmp0
          vTmpLevel_1 = _mm256_srli_epi64(vTmpLevel_1,iQBits);                          // Int32 Tmp3,Tmp1

          if (signHiding)
          {
            __m256i vBS0 = _mm256_slli_epi64(vTmpLevel_0,iQBits);
            __m256i vBS1 = _mm256_slli_epi64(vTmpLevel_1,iQBits);
            vdeltaU0 = _mm256_sub_epi64(vdeltaU0,vBS0);
            vdeltaU1 = _mm256_sub_epi64(vdeltaU1,vBS1);
            vdeltaU0 = _mm256_srli_epi64(vdeltaU0,qBits8);
            vdeltaU1 = _mm256_srli_epi64(vdeltaU1,qBits8);
            vdeltaU0 = _mm256_and_si256(vdeltaU0,vMask);
            vdeltaU1 = _mm256_and_si256(vdeltaU1,vMask);
            vdeltaU1 = _mm256_slli_epi64(vdeltaU1,32);
            vdeltaU0 = _mm256_or_si256(vdeltaU0,vdeltaU1);
            _mm_storeu_si128( ( __m128i * )&deltaU[uiBlockPos],          _mm256_castsi256_si128  (vdeltaU0));
            _mm_storeu_si128( ( __m128i * )&deltaU[uiBlockPos + uiWidth],_mm256_extracti128_si256(vdeltaU0, 1));
          }
          __m256i vquantMag0 = _mm256_and_si256(vTmpLevel_0,vMask);
          __m256i vquantMag1 = _mm256_and_si256(vTmpLevel_1,vMask);
          vquantMag1  = _mm256_slli_epi64(vquantMag1,32);
          vTmpLevel_0 = _mm256_or_si256(vquantMag0,vquantMag1);
          vAbsSum     = _mm256_add_epi32(vAbsSum,vTmpLevel_0);
          vTmpLevel_1 = _mm256_and_si256(vTmpLevel_0,vSign);                            // mask only neg values
          vTmpLevel_0 = _mm256_andnot_si256(vSign,vTmpLevel_0);                         // mask only pos values
          vTmpLevel_0 = _mm256_sub_epi32(vTmpLevel_0,vTmpLevel_1);
          vTmpLevel_0 = _mm256_min_epi32(vMax, _mm256_max_epi32(vMin,vTmpLevel_0));     // clip to 16 Bit
          vTmpLevel_0 = _mm256_packs_epi32(vTmpLevel_0,vTmpLevel_0);
          _mm_storel_epi64( ( __m128i * )&piQCoef.buf[uiBlockPos],          _mm256_castsi256_si128  (vTmpLevel_0));
          _mm_storel_epi64( ( __m128i * )&piQCoef.buf[uiBlockPos + uiWidth],_mm256_extracti128_si256(vTmpLevel_0, 1));
        }
      }

      __m128i xAbsSum = _mm_add_epi32( _mm256_castsi256_si128( vAbsSum ), _mm256_extracti128_si256( vAbsSum, 1 ) );
      xAbsSum = _mm_hadd_epi32( xAbsSum, xAbsSum );
      xAbsSum = _mm_hadd_epi32( xAbsSum, xAbsSum );

      uiAbsSum += _mm_cvtsi128_si32( xAbsSum );
    }  //AVX2
    else
#endif
    {
      const __m128i vNull = _mm_setzero_si128();
      const __m128i vQuantCoeff = _mm_set1_epi32(defaultQuantisationCoefficient);
      const __m128i vAdd  = _mm_set1_epi64x(iAdd);
      const __m128i vMin  = _mm_set1_epi32(entropyCodingMinimum);
      const __m128i vMax  = _mm_set1_epi32(entropyCodingMaximum);
      const __m128i vMask = _mm_set_epi32(0, -1, 0, -1);
      __m128i vAbsSum = vNull;

      for( subSetId = iScanPos >> log2CGSize; subSetId >= 0; subSetId-- )
      {
        int uiBlockPos = cctx.blockPos( subSetId << log2CGSize );

        for( int line = 0; line < 4; line++, uiBlockPos += uiWidth )
        {
          __m128i vLevel = _mm_loadu_si128((__m128i*)&piCoef.buf[uiBlockPos]);      // coeff3,coeff2,coeff1,coeff0,
          __m128i vSign = _mm_cmpgt_epi32 (vNull,vLevel);                           // sign3,sign2,sign1,sign0 FFFF or 0000
          vLevel = _mm_abs_epi32 (vLevel);                                          // abs(vLevel3,vLevel2,vLevel1,vLevel0)
          __m128i vdeltaU0 = _mm_mul_epu32(vLevel,vQuantCoeff);                     // Tmp2,Tmp0
          __m128i vdeltaU1 = _mm_srli_si128(vLevel,4);                             // abs(0,vLevel3,vLevel2,vLevel1)
          vdeltaU1 = _mm_mul_epu32(vdeltaU1,vQuantCoeff);                           // Tmp3,Tmp1
          __m128i vTmpLevel_0 = _mm_add_epi64(vdeltaU0,vAdd);
          __m128i vTmpLevel_1 = _mm_add_epi64(vdeltaU1,vAdd);
          vTmpLevel_0 = _mm_srli_epi64(vTmpLevel_0,iQBits);                         // Int32 Tmp2,Tmp0
          vTmpLevel_1 = _mm_srli_epi64(vTmpLevel_1,iQBits);                         // Int32 Tmp3,Tmp1
          if (signHiding)
          {
            __m128i vBS0 = _mm_slli_epi64(vTmpLevel_0,iQBits);
            __m128i vBS1 = _mm_slli_epi64(vTmpLevel_1,iQBits);
            vdeltaU0 = _mm_sub_epi64(vdeltaU0,vBS0);
            vdeltaU1 = _mm_sub_epi64(vdeltaU1,vBS1);
            vdeltaU0 = _mm_srli_epi64(vdeltaU0,qBits8);
            vdeltaU1 = _mm_srli_epi64(vdeltaU1,qBits8);
            vdeltaU0 = _mm_and_si128(vdeltaU0,vMask);
            vdeltaU1 = _mm_and_si128(vdeltaU1,vMask);
            vdeltaU1 = _mm_slli_epi64(vdeltaU1,32);
            vdeltaU0 = _mm_or_si128(vdeltaU0,vdeltaU1);
           _mm_storeu_si128( ( __m128i * ) &deltaU[uiBlockPos],vdeltaU0);
          }
          __m128i vquantMag0 =  _mm_and_si128(vTmpLevel_0,vMask);
          __m128i vquantMag1 =  _mm_and_si128(vTmpLevel_1,vMask);
          vquantMag1  = _mm_slli_epi64(vquantMag1,32);
          vTmpLevel_0 = _mm_or_si128(vquantMag0,vquantMag1);
          vAbsSum     = _mm_add_epi32(vAbsSum,vTmpLevel_0);
          vTmpLevel_1 = _mm_and_si128(vTmpLevel_0,vSign);                           // mask only neg values
          vTmpLevel_0 = _mm_andnot_si128(vSign,vTmpLevel_0);                        // mask only pos values
          vTmpLevel_0 = _mm_sub_epi32(vTmpLevel_0,vTmpLevel_1);
          vTmpLevel_0 = _mm_min_epi32(vMax, _mm_max_epi32(vMin,vTmpLevel_0));       // clip to 16 Bit
          vTmpLevel_0 = _mm_packs_epi32(vTmpLevel_0,vTmpLevel_0);
          _mm_storel_epi64( ( __m128i * ) &piQCoef.buf[uiBlockPos],vTmpLevel_0);
        }
      }

      vAbsSum = _mm_hadd_epi32( vAbsSum, vAbsSum );
      vAbsSum = _mm_hadd_epi32( vAbsSum, vAbsSum );

      uiAbsSum += _mm_cvtsi128_si32( vAbsSum );
    }
  }
  else
  {
    for( int currPos = 0; currPos <= iScanPos; currPos++ )
    {
      const int uiBlockPos  = cctx.blockPos( currPos );
      const TCoeff iLevel   = piCoef.buf[uiBlockPos];
      const TCoeff iSign    = (iLevel < 0 ? -1: 1);
      const int64_t  tmpLevel = (int64_t)abs(iLevel) * defaultQuantisationCoefficient;
      const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);
      if (signHiding)
      {
        deltaU[uiBlockPos] = (TCoeff)((tmpLevel - ((int64_t)quantisedMagnitude<<iQBits) )>> qBits8);
      }
      uiAbsSum += quantisedMagnitude;
      const TCoeff quantisedCoefficient = quantisedMagnitude * iSign;
      piQCoef.buf[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
    } // for n
  }
}

template<X86_VEXT vext>
static bool NeedRdoqSIMD( const TCoeff* pCoeff, size_t numCoeff, int quantCoeff, int64_t offset, int shift )
{
#if USE_AVX2
  if( vext >= AVX2 && ( numCoeff & 15 ) == 0 )
  {
    __m256i xqnt = _mm256_set1_epi32( quantCoeff );
    __m256i xoff = _mm256_set1_epi64x( offset );

    for( int uiBlockPos = 0; uiBlockPos < numCoeff; uiBlockPos += 16 )
    {
      __m256i xcff  = _mm256_loadu_si256( ( const __m256i* ) &pCoeff[uiBlockPos] );
              xcff  = _mm256_abs_epi32( xcff );

      __m256i xlvl1 = _mm256_mul_epi32( xcff, xqnt );
              xcff  = _mm256_shuffle_epi32( xcff, 1 + ( 3 << 4 ) );
      __m256i xlvl2 = _mm256_mul_epi32( xcff, xqnt );
              xlvl1 = _mm256_add_epi64( xlvl1, xoff );
              xlvl2 = _mm256_add_epi64( xlvl2, xoff );
              xlvl1 = _mm256_srli_epi64( xlvl1, shift );
              xlvl2 = _mm256_srli_epi64( xlvl2, shift );

      __m256i xany  = _mm256_or_si256( xlvl1, xlvl2 );
      
              xcff  = _mm256_loadu_si256( ( const __m256i* ) &pCoeff[uiBlockPos + 8] );
              xcff  = _mm256_abs_epi32( xcff );
              
              xlvl1 = _mm256_mul_epi32( xcff, xqnt );
              xcff  = _mm256_shuffle_epi32( xcff, 1 + ( 3 << 4 ) );
              xlvl2 = _mm256_mul_epi32( xcff, xqnt );
              xlvl1 = _mm256_add_epi64( xlvl1, xoff );
              xlvl2 = _mm256_add_epi64( xlvl2, xoff );
              xlvl1 = _mm256_srli_epi64( xlvl1, shift );
              xlvl2 = _mm256_srli_epi64( xlvl2, shift );

              xany  = _mm256_or_si256( xany, _mm256_or_si256( xlvl1, xlvl2 ) );

      if( !_mm256_testz_si256( xany, xany ) )
      {
        return true;
      }
    }
    return false;
  }
  else if( vext >= AVX2 && ( numCoeff & 7 ) == 0 )
  {
    __m256i xqnt = _mm256_set1_epi32( quantCoeff );
    __m256i xoff = _mm256_set1_epi64x( offset );

    for( int uiBlockPos = 0; uiBlockPos < numCoeff; uiBlockPos += 8 )
    {
      __m256i xcff  = _mm256_loadu_si256( ( const __m256i* ) &pCoeff[uiBlockPos] );
              xcff  = _mm256_abs_epi32( xcff );

      __m256i xlvl1 = _mm256_mul_epi32( xcff, xqnt );
              xcff  = _mm256_shuffle_epi32( xcff, 1 + ( 3 << 4 ) );
      __m256i xlvl2 = _mm256_mul_epi32( xcff, xqnt );
              xlvl1 = _mm256_add_epi64( xlvl1, xoff );
              xlvl2 = _mm256_add_epi64( xlvl2, xoff );
              xlvl1 = _mm256_srli_epi64( xlvl1, shift );
              xlvl2 = _mm256_srli_epi64( xlvl2, shift );

      __m256i xany  = _mm256_or_si256( xlvl1, xlvl2 );

      if( !_mm256_testz_si256( xany, xany ) )
      {
        return true;
      }
    }
    return false;
  }
  else
#endif
  if( ( numCoeff & 3 ) == 0 )
  {
    __m128i xqnt = _mm_set1_epi32( quantCoeff );
    __m128i xoff = _mm_set1_epi64x( offset );

    for( int uiBlockPos = 0; uiBlockPos < numCoeff; uiBlockPos += 4 )
    {
      __m128i xcff  = _mm_loadu_si128( ( const __m128i* ) &pCoeff[uiBlockPos] );
              xcff  = _mm_abs_epi32( xcff );

      __m128i xlvl1 = _mm_mul_epi32( xcff, xqnt );
              xcff  = _mm_shuffle_epi32( xcff, 1 + ( 3 << 4 ) );
      __m128i xlvl2 = _mm_mul_epi32( xcff, xqnt );
              xlvl1 = _mm_add_epi64( xlvl1, xoff );
              xlvl2 = _mm_add_epi64( xlvl2, xoff );
              xlvl1 = _mm_srli_epi64( xlvl1, shift );
              xlvl2 = _mm_srli_epi64( xlvl2, shift );

      __m128i xany  = _mm_or_si128( xlvl1, xlvl2 );

      if( !_mm_test_all_zeros( xany, xany ) )
      {
        return true;
      }
    }
    return false;
  }
  else
  {
    for( int uiBlockPos = 0; uiBlockPos < numCoeff; uiBlockPos++ )
    {
      const TCoeff   iLevel = pCoeff[uiBlockPos];
      const int64_t  tmpLevel = ( int64_t ) std::abs( iLevel ) * quantCoeff;
      const TCoeff quantisedMagnitude = TCoeff( ( tmpLevel + offset ) >> shift );

      if( quantisedMagnitude != 0 )
      {
        return true;
      }
    } // for n
    return false;
  }
}

template<X86_VEXT vext>
void Quant::_initQuantX86()
{
  xDeQuant  = DeQuantCoreSIMD<vext>;
  xQuant    = QuantCoreSIMD  <vext>;
  xNeedRdoq = NeedRdoqSIMD   <vext>;
}
template void Quant::_initQuantX86<SIMDX86>();


} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     TrQuant_EMT.cpp
    \brief    transform and quantization class
*/

#include "TrQuant_EMT.h"
#include "Rom.h"

#include <stdlib.h>
#include <math.h>
#include <memory.h>

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ********************************** DCT-II **********************************
  
#if ENABLE_SIMD_TRAFO
template<int uiTrSize>
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT );

template<>
inline void _fastInverseMM<2>( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
{
  const int rnd_factor  = 1 << (shift - 1);
  const int reducedLine = line - iSkipLine;
  const int cutoff      = 2 - iSkipLine2;

  memset( dst, 0, reducedLine * 2 * sizeof( TCoeff ) );

  for( int k = 0; k < cutoff; k++ )
  {
    const TCoeff* srcPtr = &src[k * line];
    for( int i = 0; i < reducedLine; i++ )
    {
            TCoeff*       dstPtr = &dst[i << 1];
      const TMatrixCoeff*  itPtr =  &iT[k << 1];
      const TCoeff        srcVal = *srcPtr;
      for( int j = 0; j < 2; j++ )
      {
        *dstPtr++ += srcVal * *itPtr++;
      }
      srcPtr++;
    }
  }

  for( int i = 0; i < reducedLine; i++ )
  {
    TCoeff* dstPtr = &dst[i << 1];
    for( int j = 0; j < 2; j++, dstPtr++ )
    {
      *dstPtr = Clip3( outputMinimum, outputMaximum, ( int ) ( *dstPtr + rnd_factor ) >> shift );
    }
  }

  if( iSkipLine )
  {
    memset( dst + ( reducedLine << 1 ), 0, ( iSkipLine << 1 ) * sizeof( TCoeff ) );
  }
}

template<>
inline void _fastInverseMM<4>( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
{
  const int rnd_factor  = 1 << ( shift - 1 );
  const int reducedLine = line - iSkipLine;
  const int cutoff      = 4 - iSkipLine2;

  memset( dst, 0, reducedLine * 4 * sizeof( TCoeff ) );

#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.fastInvCore[0]( iT, src, dst, line, reducedLine, cutoff );
  g_tCoeffOps.roundClip4( dst, 4, reducedLine, 4, outputMinimum, outputMaximum, rnd_factor, shift );
#else
  for( int k = 0; k < cutoff; k++ )
  {
    const TCoeff* srcPtr = &src[k * line];
    for( int i = 0; i < reducedLine; i++ )
    {
            TCoeff*       dstPtr = &dst[i << 2];
      const TMatrixCoeff*  itPtr =  &iT[k << 2];
      for( int j = 0; j < 4; j++ )
      {
        *dstPtr++ += *srcPtr * *itPtr++;
      }
      srcPtr++;
    }
  }

  for( int i = 0; i < reducedLine; i++ )
  {
    TCoeff* dstPtr = &dst[i << 2];
    for( int j = 0; j < 4; j++, dstPtr++ )
    {
      *dstPtr = Clip3( outputMinimum, outputMaximum, ( int ) ( *dstPtr + rnd_factor ) >> shift );
    }
  }
#endif

  if( iSkipLine )
  {
    memset( dst + ( reducedLine << 2 ), 0, ( iSkipLine << 2 ) * sizeof( TCoeff ) );
  }
}

#endif

template< int uiTrSize >
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
{
  const int  rnd_factor  = 1 << (shift - 1);
  const int  reducedLine = line - iSkipLine;
  const int  cutoff      = uiTrSize - iSkipLine2;

  memset( dst, 0, reducedLine * uiTrSize * sizeof( TCoeff ) );

#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.fastInvCore[Log2( uiTrSize ) - 2]( iT, src, dst, line, reducedLine, cutoff );
  g_tCoeffOps.roundClip8( dst, uiTrSize, reducedLine, uiTrSize, outputMinimum, outputMaximum, rnd_factor, shift );
#else
  for( int k = 0; k < cutoff; k++ )
  {
    const TCoeff* srcPtr = &src[k * line];
    for( int i = 0; i < reducedLine; i++ )
    {
            TCoeff*       dstPtr = &dst[i * uiTrSize];
      const TMatrixCoeff*  itPtr =  &iT[k * uiTrSize];
      for( int j = 0; j < uiTrSize; j++ )
      {
        *dstPtr++ += *srcPtr * *itPtr++;
      }
      srcPtr++;
    }
  }

  for( int i = 0; i < reducedLine; i++ )
  {
    TCoeff* dstPtr = &dst[i * uiTrSize];
    for( int j = 0; j < uiTrSize; j++, dstPtr++ )
    {
      *dstPtr = Clip3( outputMinimum, outputMaximum, ( int ) ( *dstPtr + rnd_factor ) >> shift );
    }
  }
#endif

  if( iSkipLine )
  {
    memset( dst + ( reducedLine*uiTrSize ), 0, ( iSkipLine*uiTrSize ) * sizeof( TCoeff ) );
  }
}

//Fast DCT-II transforms
void fastForwardDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  int j;
  int E, O;
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDCT2P2[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O */
    E = src[0] + src[1];
    O = src[0] - src[1];

    dst[0] = (iT[0] * E + add) >> shift;
    dst[line] = (iT[2] * O + add) >> shift;


    src += 2;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<2; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

void fastInverseDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  int j;
  int E, O;
  int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT2P2[TRANSFORM_INVERSE][0];

  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    E = iT[0] * (src[0] + src[line]);
    O = iT[2] * (src[0] - src[line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3(outputMinimum, outputMaximum, (E + add) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (O + add) >> shift);

    src++;
    dst += 2;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 1) * sizeof(TCoeff));
  }
}

/** 4x4 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
void fastForwardDCT2_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  int j;
  TCoeff E[2], O[2];
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDCT2P4[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

    dst[0] = (iT[0] * E[0] + iT[1] * E[1] + add) >> shift;
    dst[2 * line] = (iT[8] * E[0] + iT[9] * E[1] + add) >> shift;
    dst[line] = (iT[4] * O[0] + iT[5] * O[1] + add) >> shift;
    dst[3 * line] = (iT[12] * O[0] + iT[13] * O[1] + add) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<4; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

/** 4x4 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B4( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
#if 0
  const TMatrixCoeff *iT = g_trCoreDCT2P4[0];

  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j;
  int E[2], O[2];
  int add = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = g_trCoreDCT2P4[TRANSFORM_INVERSE][0];

#if ENABLE_SIMD_TRAFO
  TCoeff* orgDst = dst;

#endif
  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    O[0] = iT[1 * 4 + 0] * src[line] + iT[3 * 4 + 0] * src[3 * line];
    O[1] = iT[1 * 4 + 1] * src[line] + iT[3 * 4 + 1] * src[3 * line];
    E[0] = iT[0 * 4 + 0] * src[   0] + iT[2 * 4 + 0] * src[2 * line];
    E[1] = iT[0 * 4 + 1] * src[   0] + iT[2 * 4 + 1] * src[2 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
#if ENABLE_SIMD_TRAFO
    dst[0] = E[0] + O[0];
    dst[1] = E[1] + O[1];
    dst[2] = E[1] - O[1];
    dst[3] = E[0] - O[0];
#else
    dst[0] = Clip3( outputMinimum, outputMaximum, ( E[0] + O[0] + add ) >> shift );
    dst[1] = Clip3( outputMinimum, outputMaximum, ( E[1] + O[1] + add ) >> shift );
    dst[2] = Clip3( outputMinimum, outputMaximum, ( E[1] - O[1] + add ) >> shift );
    dst[3] = Clip3( outputMinimum, outputMaximum, ( E[0] - O[0] + add ) >> shift );
#endif

    src++;
    dst += 4;
  }

#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.roundClip4( orgDst, 4, reducedLine, 4, outputMinimum, outputMaximum, add, shift );

#endif
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 2 ) * sizeof( TCoeff ) );
  }
#endif
}



template< int uiTrSize >
inline void _fastForwardMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TMatrixCoeff* tc )
{
#if !ENABLE_SIMD_TRAFO
  const int  rnd_factor  = 1 << (shift - 1);
#endif
  const int  reducedLine = line - iSkipLine;
  const int  cutoff      = uiTrSize - iSkipLine2;
  TCoeff *pCoef;

#if ENABLE_SIMD_TRAFO
  if( line == 1 )
  {
    g_tCoeffOps.fastFwdCore_1D[Log2( uiTrSize ) - 2]( tc, src, dst, line, reducedLine, cutoff, shift );
  }
  else
  {
    g_tCoeffOps.fastFwdCore_2D[Log2( uiTrSize ) - 2]( tc, src, dst, line, reducedLine, cutoff, shift );
  }
#else
  for( int i = 0; i<reducedLine; i++ )
  {
    pCoef = dst;
    const TMatrixCoeff* iT = tc;
    for( int j = 0; j<cutoff; j++ )
    {
      int iSum = 0;
      for( int k = 0; k<uiTrSize; k++ )
      {
        // dst[j * line + i] += src[i * trSize + k] * t[j * trSize + k]
        iSum += src[k] * iT[k];
      }
      pCoef[i] = (iSum + rnd_factor) >> shift;
      pCoef += line;
      iT += uiTrSize;
    }
    src += uiTrSize;
  }
#endif

  if( iSkipLine )
  {
    pCoef = dst + reducedLine;
    for( int j = 0; j<cutoff; j++ )
    {
      memset(pCoef, 0, sizeof(TCoeff) * iSkipLine);
      pCoef += line;
    }
  }

  if( iSkipLine2 )
  {
    pCoef = dst + line*cutoff;
    memset(pCoef, 0, sizeof(TCoeff) * line * iSkipLine2);
  }
}



/** 8x8 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
void fastForwardDCT2_B8( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff E[4], O[4];
  TCoeff EE[2], EO[2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

  const TMatrixCoeff *iT = g_trCoreDCT2P8[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* E and O*/
    for( k = 0; k < 4; k++ )
    {
      E[k] = src[k] + src[7 - k];
      O[k] = src[k] - src[7 - k];
    }
    /* EE and EO */
    EE[0] = E[0] + E[3];
    EO[0] = E[0] - E[3];
    EE[1] = E[1] + E[2];
    EO[1] = E[1] - E[2];

    dst[0       ] = (iT[ 0] * EE[0] + iT[ 1] * EE[1] + add) >> shift;
    dst[4 * line] = (iT[32] * EE[0] + iT[33] * EE[1] + add) >> shift;
    dst[2 * line] = (iT[16] * EO[0] + iT[17] * EO[1] + add) >> shift;
    dst[6 * line] = (iT[48] * EO[0] + iT[49] * EO[1] + add) >> shift;

    dst[    line] = (iT[ 8] * O[0] + iT[ 9] * O[1] + iT[10] * O[2] + iT[11] * O[3] + add) >> shift;
    dst[3 * line] = (iT[24] * O[0] + iT[25] * O[1] + iT[26] * O[2] + iT[27] * O[3] + add) >> shift;
    dst[5 * line] = (iT[40] * O[0] + iT[41] * O[1] + iT[42] * O[2] + iT[43] * O[3] + add) >> shift;
    dst[7 * line] = (iT[56] * O[0] + iT[57] * O[1] + iT[58] * O[2] + iT[59] * O[3] + add) >> shift;

    src += 8;
    dst++;
  }
  if( iSkipLine )
  {
    dst = pCoef + reducedLine;
    for( j = 0; j < 8; j++ )
    {
      memset( dst, 0, sizeof( TCoeff )*iSkipLine );
      dst += line;
    }
  }
#else
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDCT2P8[TRANSFORM_FORWARD][0] );
#endif
}

/** 8x8 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 0
  const TMatrixCoeff *iT = g_trCoreDCT2P8[0];

  _fastInverseMM<8>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j, k;
  int E[4], O[4];
  int EE[2], EO[2];
  int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT2P8[TRANSFORM_INVERSE][0];

#if ENABLE_SIMD_TRAFO
  TCoeff *orgDst = dst;

#endif
  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for( k = 0; k < 4; k++ )
    {
      O[k] = iT[1 * 8 + k] * src[line] + iT[3 * 8 + k] * src[3 * line] + iT[5 * 8 + k] * src[5 * line] + iT[7 * 8 + k] * src[7 * line];
    }

    EO[0] = iT[2 * 8 + 0] * src[2 * line] + iT[6 * 8 + 0] * src[6 * line];
    EO[1] = iT[2 * 8 + 1] * src[2 * line] + iT[6 * 8 + 1] * src[6 * line];
    EE[0] = iT[0 * 8 + 0] * src[0       ] + iT[4 * 8 + 0] * src[4 * line];
    EE[1] = iT[0 * 8 + 1] * src[0       ] + iT[4 * 8 + 1] * src[4 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];

    for( k = 0; k < 4; k++ )
    {
#if ENABLE_SIMD_TRAFO
      dst[k    ] = E[    k] + O[    k];
      dst[k + 4] = E[3 - k] - O[3 - k];
#else
      dst[k    ] = Clip3( outputMinimum, outputMaximum, ( E[    k] + O[    k] + add ) >> shift );
      dst[k + 4] = Clip3( outputMinimum, outputMaximum, ( E[3 - k] - O[3 - k] + add ) >> shift );
#endif
    }
    src++;
    dst += 8;
  }

#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.roundClip8( orgDst, 8, reducedLine, 8, outputMinimum, outputMaximum, add, shift );

#endif
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 3 ) * sizeof( TCoeff ) );
  }
#endif
}


/** 16x16 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
void fastForwardDCT2_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff E  [8], O  [8];
  TCoeff EE [4], EO [4];
  TCoeff EEE[2], EEO[2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

  const TMatrixCoeff *iT = g_trCoreDCT2P16[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* E and O*/
    for( k = 0; k < 8; k++ )
    {
      E[k] = src[k] + src[15 - k];
      O[k] = src[k] - src[15 - k];
    }
    /* EE and EO */
    for( k = 0; k < 4; k++ )
    {
      EE[k] = E[k] + E[7 - k];
      EO[k] = E[k] - E[7 - k];
    }
    /* EEE and EEO */
    EEE[0] = EE[0] + EE[3];
    EEO[0] = EE[0] - EE[3];
    EEE[1] = EE[1] + EE[2];
    EEO[1] = EE[1] - EE[2];

    dst[ 0       ] = ( iT[ 0     ] * EEE[0] + iT[          1] * EEE[1] + add ) >> shift;
    dst[ 8 * line] = ( iT[ 8 * 16] * EEE[0] + iT[ 8 * 16 + 1] * EEE[1] + add ) >> shift;
    dst[ 4 * line] = ( iT[ 4 * 16] * EEO[0] + iT[ 4 * 16 + 1] * EEO[1] + add ) >> shift;
    dst[12 * line] = ( iT[12 * 16] * EEO[0] + iT[12 * 16 + 1] * EEO[1] + add ) >> shift;

    for( k = 2; k < 16; k += 4 )
    {
      dst[k*line] = ( iT[k * 16] * EO[0] + iT[k * 16 + 1] * EO[1] + iT[k * 16 + 2] * EO[2] + iT[k * 16 + 3] * EO[3] + add ) >> shift;
    }

    for( k = 1; k < 16; k += 2 )
    {
      dst[k*line] = ( iT[k * 16    ] * O[0] + iT[k * 16 + 1] * O[1] + iT[k * 16 + 2] * O[2] + iT[k * 16 + 3] * O[3] +
                      iT[k * 16 + 4] * O[4] + iT[k * 16 + 5] * O[5] + iT[k * 16 + 6] * O[6] + iT[k * 16 + 7] * O[7] + add ) >> shift;
    }

    src += 16;
    dst++;

  }
  if( iSkipLine )
  {
    dst = pCoef + reducedLine;
    for( j = 0; j < 16; j++ )
    {
      memset( dst, 0, sizeof( TCoeff )*iSkipLine );
      dst += line;
    }
  }
#else
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDCT2P16[TRANSFORM_FORWARD][0] );
#endif
}

/** 16x16 inverse transform implemented using partial butterfly structure (1D)
*  \param src            input data (transform coefficients)
*  \param dst            output data (residual)
*  \param shift          specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B16( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
#if ENABLE_SIMD_TRAFO
  const TMatrixCoeff *iT = g_trCoreDCT2P16[TRANSFORM_INVERSE][0];

  _fastInverseMM<16>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j, k;
  int E  [8], O  [8];
  int EE [4], EO [4];
  int EEE[2], EEO[2];
  int add = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = g_trCoreDCT2P16[TRANSFORM_INVERSE][0];

#if ENABLE_SIMD_TRAFO
  TCoeff *orgDst = dst;

#endif
  const int  reducedLine = line - iSkipLine;

  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for( k = 0; k < 8; k++ )
    {
      O[k] = iT[1 * 16 + k] * src[    line] + iT[ 3 * 16 + k] * src[ 3 * line] + iT[ 5 * 16 + k] * src[ 5 * line] + iT[ 7 * 16 + k] * src[ 7 * line] +
        iT[9 * 16 + k] * src[9 * line] + iT[11 * 16 + k] * src[11 * line] + iT[13 * 16 + k] * src[13 * line] + iT[15 * 16 + k] * src[15 * line];
    }
    for( k = 0; k < 4; k++ )
    {
      EO[k] = iT[2 * 16 + k] * src[2 * line] + iT[6 * 16 + k] * src[6 * line] + iT[10 * 16 + k] * src[10 * line] + iT[14 * 16 + k] * src[14 * line];
    }
    EEO[0] = iT[4 * 16    ] * src[4 * line] + iT[12 * 16    ] * src[12 * line];
    EEE[0] = iT[0         ] * src[0       ] + iT[ 8 * 16    ] * src[ 8 * line];
    EEO[1] = iT[4 * 16 + 1] * src[4 * line] + iT[12 * 16 + 1] * src[12 * line];
    EEE[1] = iT[0 * 16 + 1] * src[0       ] + iT[ 8 * 16 + 1] * src[ 8 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for( k = 0; k < 2; k++ )
    {
      EE[k    ] = EEE[    k] + EEO[    k];
      EE[k + 2] = EEE[1 - k] - EEO[1 - k];
    }
    for( k = 0; k < 4; k++ )
    {
      E[k    ] = EE[    k] + EO[    k];
      E[k + 4] = EE[3 - k] - EO[3 - k];
    }
    for( k = 0; k < 8; k++ )
    {
#if ENABLE_SIMD_TRAFO
      dst[k    ] = E[    k] + O[    k];
      dst[k + 8] = E[7 - k] - O[7 - k];
#else
      dst[k    ] = Clip3( outputMinimum, outputMaximum, ( E[    k] + O[    k] + add ) >> shift );
      dst[k + 8] = Clip3( outputMinimum, outputMaximum, ( E[7 - k] - O[7 - k] + add ) >> shift );
#endif
    }
    src++;
    dst += 16;
  }

#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.roundClip8( orgDst, 16, reducedLine, 16, outputMinimum, outputMaximum, add, shift );

#endif
  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 4 ) * sizeof( TCoeff ) );
  }
#endif
}



/** 32x32 forward transform implemented using partial butterfly structure (1D)
*  \param src   input data (residual)
*  \param dst   output data (transform coefficients)
*  \param shift specifies right shift after 1D transform
*  \param line
*/
void fastForwardDCT2_B32( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff E   [16], O   [16];
  TCoeff EE  [ 8], EO  [ 8];
  TCoeff EEE [ 4], EEO [ 4];
  TCoeff EEEE[ 2], EEEO[ 2];
  TCoeff add = ( shift > 0 ) ? ( 1 << ( shift - 1 ) ) : 0;

  const TMatrixCoeff *iT = g_trCoreDCT2P32[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* E and O*/
    for (k = 0;k<16;k++)
    {
      E[k] = src[k] + src[31 - k];
      O[k] = src[k] - src[31 - k];
    }
    /* EE and EO */
    for (k = 0;k<8;k++)
    {
      EE[k] = E[k] + E[15 - k];
      EO[k] = E[k] - E[15 - k];
    }
    /* EEE and EEO */
    for (k = 0;k<4;k++)
    {
      EEE[k] = EE[k] + EE[7 - k];
      EEO[k] = EE[k] - EE[7 - k];
    }
    /* EEEE and EEEO */
    EEEE[0] = EEE[0] + EEE[3];
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];

    dst[0] = (iT[0 * 32 + 0] * EEEE[0] + iT[0 * 32 + 1] * EEEE[1] + add) >> shift;
    dst[16 * line] = (iT[16 * 32 + 0] * EEEE[0] + iT[16 * 32 + 1] * EEEE[1] + add) >> shift;
    dst[8 * line] = (iT[8 * 32 + 0] * EEEO[0] + iT[8 * 32 + 1] * EEEO[1] + add) >> shift;
    dst[24 * line] = (iT[24 * 32 + 0] * EEEO[0] + iT[24 * 32 + 1] * EEEO[1] + add) >> shift;
    for (k = 4;k<32;k += 8)
    {
      dst[k*line] = (iT[k * 32 + 0] * EEO[0] + iT[k * 32 + 1] * EEO[1] + iT[k * 32 + 2] * EEO[2] + iT[k * 32 + 3] * EEO[3] + add) >> shift;
    }
    for (k = 2;k<32;k += 4)
    {
      dst[k*line] = (iT[k * 32 + 0] * EO[0] + iT[k * 32 + 1] * EO[1] + iT[k * 32 + 2] * EO[2] + iT[k * 32 + 3] * EO[3] +
                      iT[k * 32 + 4] * EO[4] + iT[k * 32 + 5] * EO[5] + iT[k * 32 + 6] * EO[6] + iT[k * 32 + 7] * EO[7] + add) >> shift;
    }
    for (k = 1;k<32;k += 2)
    {
      dst[k*line] = (iT[k * 32 + 0] * O[0] + iT[k * 32 + 1] * O[1] + iT[k * 32 + 2] * O[2] + iT[k * 32 + 3] * O[3] +
                      iT[k * 32 + 4] * O[4] + iT[k * 32 + 5] * O[5] + iT[k * 32 + 6] * O[6] + iT[k * 32 + 7] * O[7] +
                      iT[k * 32 + 8] * O[8] + iT[k * 32 + 9] * O[9] + iT[k * 32 + 10] * O[10] + iT[k * 32 + 11] * O[11] +
                      iT[k * 32 + 12] * O[12] + iT[k * 32 + 13] * O[13] + iT[k * 32 + 14] * O[14] + iT[k * 32 + 15] * O[15] + add) >> shift;
    }
    src += 32;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j<32; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
#else
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDCT2P32[TRANSFORM_FORWARD][0] );
#endif
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if ENABLE_SIMD_TRAFO
  const TMatrixCoeff *iT = g_trCoreDCT2P32[TRANSFORM_INVERSE][0];

  _fastInverseMM<32>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j, k;
  int E[16], O[16];
  int EE[8], EO[8];
  int EEE[4], EEO[4];
  int EEEE[2], EEEO[2];
  int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT2P32[TRANSFORM_INVERSE][0];

#if ENABLE_SIMD_TRAFO
  TCoeff *orgDst = dst;

#endif
  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k = 0;k<16;k++)
    {
      O[k] = iT[1 * 32 + k] * src[line] + iT[3 * 32 + k] * src[3 * line] + iT[5 * 32 + k] * src[5 * line] + iT[7 * 32 + k] * src[7 * line] +
        iT[9 * 32 + k] * src[9 * line] + iT[11 * 32 + k] * src[11 * line] + iT[13 * 32 + k] * src[13 * line] + iT[15 * 32 + k] * src[15 * line] +
        iT[17 * 32 + k] * src[17 * line] + iT[19 * 32 + k] * src[19 * line] + iT[21 * 32 + k] * src[21 * line] + iT[23 * 32 + k] * src[23 * line] +
        iT[25 * 32 + k] * src[25 * line] + iT[27 * 32 + k] * src[27 * line] + iT[29 * 32 + k] * src[29 * line] + iT[31 * 32 + k] * src[31 * line];
    }
    for (k = 0;k<8;k++)
    {
      EO[k] = iT[2 * 32 + k] * src[2 * line] + iT[6 * 32 + k] * src[6 * line] + iT[10 * 32 + k] * src[10 * line] + iT[14 * 32 + k] * src[14 * line] +
        iT[18 * 32 + k] * src[18 * line] + iT[22 * 32 + k] * src[22 * line] + iT[26 * 32 + k] * src[26 * line] + iT[30 * 32 + k] * src[30 * line];
    }
    for (k = 0;k<4;k++)
    {
      EEO[k] = iT[4 * 32 + k] * src[4 * line] + iT[12 * 32 + k] * src[12 * line] + iT[20 * 32 + k] * src[20 * line] + iT[28 * 32 + k] * src[28 * line];
    }
    EEEO[0] = iT[8 * 32 + 0] * src[8 * line] + iT[24 * 32 + 0] * src[24 * line];
    EEEO[1] = iT[8 * 32 + 1] * src[8 * line] + iT[24 * 32 + 1] * src[24 * line];
    EEEE[0] = iT[0 * 32 + 0] * src[0] + iT[16 * 32 + 0] * src[16 * line];
    EEEE[1] = iT[0 * 32 + 1] * src[0] + iT[16 * 32 + 1] * src[16 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];
    for (k = 0;k<4;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 4] = EEE[3 - k] - EEO[3 - k];
    }
    for (k = 0;k<8;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 8] = EE[7 - k] - EO[7 - k];
    }
    for (k = 0;k<16;k++)
    {
#if ENABLE_SIMD_TRAFO
      dst[k     ] = E[k     ] + O[k     ];
      dst[k + 16] = E[15 - k] - O[15 - k];
#else
      dst[k] = Clip3(outputMinimum, outputMaximum, (E[k] + O[k] + add) >> shift);
      dst[k + 16] = Clip3(outputMinimum, outputMaximum, (E[15 - k] - O[15 - k] + add) >> shift);
#endif
    }
    src++;
    dst += 32;
  }

#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.roundClip8( orgDst, 32, reducedLine, 32, outputMinimum, outputMaximum, add, shift );

#endif
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 5) * sizeof(TCoeff));
  }
#endif
}

void fastForwardDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
#if !JVET_M0497_MATRIX_MULT
  int rnd_factor = 1 << (shift - 1);

  const int uiTrSize = 64;
  const TMatrixCoeff *iT = g_trCoreDCT2P64[TRANSFORM_FORWARD][0];

  int   j, k;
  TCoeff E[32], O[32];
  TCoeff EE[16], EO[16];
  TCoeff EEE[8], EEO[8];
  TCoeff EEEE[4], EEEO[4];
  TCoeff EEEEE[2], EEEEO[2];
  TCoeff *tmp = dst;

  //bool zo = iSkipLine2 >= 32;
  bool zo = iSkipLine2 != 0;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* E and O*/
    for (k = 0;k<32;k++)
    {
      E[k] = src[k] + src[63 - k];
      O[k] = src[k] - src[63 - k];
    }
    /* EE and EO */
    for (k = 0;k<16;k++)
    {
      EE[k] = E[k] + E[31 - k];
      EO[k] = E[k] - E[31 - k];
    }
    /* EEE and EEO */
    for (k = 0;k<8;k++)
    {
      EEE[k] = EE[k] + EE[15 - k];
      EEO[k] = EE[k] - EE[15 - k];
    }
    /* EEEE and EEEO */
    for (k = 0;k<4;k++)
    {
      EEEE[k] = EEE[k] + EEE[7 - k];
      EEEO[k] = EEE[k] - EEE[7 - k];
    }
    /* EEEEE and EEEEO */
    EEEEE[0] = EEEE[0] + EEEE[3];
    EEEEO[0] = EEEE[0] - EEEE[3];
    EEEEE[1] = EEEE[1] + EEEE[2];
    EEEEO[1] = EEEE[1] - EEEE[2];

    dst[0] = (iT[0 * 64 + 0] * EEEEE[0] + iT[0 * 64 + 1] * EEEEE[1] + rnd_factor) >> shift;
    dst[16 * line] = (iT[16 * 64 + 0] * EEEEO[0] + iT[16 * 64 + 1] * EEEEO[1] + rnd_factor) >> shift;

    if (!zo)
    {
      dst[32 * line] = (iT[32 * 64 + 0] * EEEEE[0] + iT[32 * 64 + 1] * EEEEE[1] + rnd_factor) >> shift;
      dst[48 * line] = (iT[48 * 64 + 0] * EEEEO[0] + iT[48 * 64 + 1] * EEEEO[1] + rnd_factor) >> shift;
    }
    for (k = 8;k<(zo ? 32 : 64);k += 16)
    {
      dst[k*line] = (iT[k * 64 + 0] * EEEO[0] + iT[k * 64 + 1] * EEEO[1] + iT[k * 64 + 2] * EEEO[2] + iT[k * 64 + 3] * EEEO[3] + rnd_factor) >> shift;
    }
    for (k = 4;k<(zo ? 32 : 64);k += 8)
    {
      dst[k*line] = (iT[k * 64 + 0] * EEO[0] + iT[k * 64 + 1] * EEO[1] + iT[k * 64 + 2] * EEO[2] + iT[k * 64 + 3] * EEO[3] +
                      iT[k * 64 + 4] * EEO[4] + iT[k * 64 + 5] * EEO[5] + iT[k * 64 + 6] * EEO[6] + iT[k * 64 + 7] * EEO[7] + rnd_factor) >> shift;
    }
    for (k = 2;k<(zo ? 32 : 64);k += 4)
    {
      dst[k*line] = (iT[k * 64 + 0] * EO[0] + iT[k * 64 + 1] * EO[1] + iT[k * 64 + 2] * EO[2] + iT[k * 64 + 3] * EO[3] +
                      iT[k * 64 + 4] * EO[4] + iT[k * 64 + 5] * EO[5] + iT[k * 64 + 6] * EO[6] + iT[k * 64 + 7] * EO[7] +
                      iT[k * 64 + 8] * EO[8] + iT[k * 64 + 9] * EO[9] + iT[k * 64 + 10] * EO[10] + iT[k * 64 + 11] * EO[11] +
                      iT[k * 64 + 12] * EO[12] + iT[k * 64 + 13] * EO[13] + iT[k * 64 + 14] * EO[14] + iT[k * 64 + 15] * EO[15] + rnd_factor) >> shift;
    }
    for (k = 1;k<(zo ? 32 : 64);k += 2)
    {
      dst[k*line] = (iT[k * 64 + 0] * O[0] + iT[k * 64 + 1] * O[1] + iT[k * 64 + 2] * O[2] + iT[k * 64 + 3] * O[3] +
                      iT[k * 64 + 4] * O[4] + iT[k * 64 + 5] * O[5] + iT[k * 64 + 6] * O[6] + iT[k * 64 + 7] * O[7] +
                      iT[k * 64 + 8] * O[8] + iT[k * 64 + 9] * O[9] + iT[k * 64 + 10] * O[10] + iT[k * 64 + 11] * O[11] +
                      iT[k * 64 + 12] * O[12] + iT[k * 64 + 13] * O[13] + iT[k * 64 + 14] * O[14] + iT[k * 64 + 15] * O[15] +
                      iT[k * 64 + 16] * O[16] + iT[k * 64 + 17] * O[17] + iT[k * 64 + 18] * O[18] + iT[k * 64 + 19] * O[19] +
                      iT[k * 64 + 20] * O[20] + iT[k * 64 + 21] * O[21] + iT[k * 64 + 22] * O[22] + iT[k * 64 + 23] * O[23] +
                      iT[k * 64 + 24] * O[24] + iT[k * 64 + 25] * O[25] + iT[k * 64 + 26] * O[26] + iT[k * 64 + 27] * O[27] +
                      iT[k * 64 + 28] * O[28] + iT[k * 64 + 29] * O[29] + iT[k * 64 + 30] * O[30] + iT[k * 64 + 31] * O[31] + rnd_factor) >> shift;
    }
    src += uiTrSize;
    dst++;
  }

  const int  reducedLine = line - iSkipLine;
  const int  cutoff = uiTrSize - iSkipLine2;
  if (iSkipLine)
  {
    dst = tmp + reducedLine;
    for (j = 0; j<cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
  if (iSkipLine2)
  {
    dst = tmp + line*cutoff;
    memset(dst, 0, sizeof(TCoeff)*line*iSkipLine2);
  }
#else
  _fastForwardMM< 64 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDCT2P64[TRANSFORM_FORWARD][0] );
#endif
}

void fastInverseDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if ENABLE_SIMD_TRAFO
  const TMatrixCoeff *iT = g_trCoreDCT2P64[TRANSFORM_INVERSE][0];

  _fastInverseMM<64>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int rnd_factor = 1 << (shift - 1);
  const int uiTrSize = 64;
  const TMatrixCoeff *iT = g_trCoreDCT2P64[TRANSFORM_INVERSE][0];

#if ENABLE_SIMD_TRAFO
  TCoeff *orgDst = dst;

#endif
  int    j, k;
  TCoeff E[32], O[32];
  TCoeff EE[16], EO[16];
  TCoeff EEE[8], EEO[8];
  TCoeff EEEE[4], EEEO[4];
  TCoeff EEEEE[2], EEEEO[2];
  bool zo = iSkipLine2 >= 32;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k = 0;k<32;k++)
    {
      O[k] = iT[1 * 64 + k] * src[line] + iT[3 * 64 + k] * src[3 * line] + iT[5 * 64 + k] * src[5 * line] + iT[7 * 64 + k] * src[7 * line] +
        iT[9 * 64 + k] * src[9 * line] + iT[11 * 64 + k] * src[11 * line] + iT[13 * 64 + k] * src[13 * line] + iT[15 * 64 + k] * src[15 * line] +
        iT[17 * 64 + k] * src[17 * line] + iT[19 * 64 + k] * src[19 * line] + iT[21 * 64 + k] * src[21 * line] + iT[23 * 64 + k] * src[23 * line] +
        iT[25 * 64 + k] * src[25 * line] + iT[27 * 64 + k] * src[27 * line] + iT[29 * 64 + k] * src[29 * line] + iT[31 * 64 + k] * src[31 * line] +
        (zo ? 0 : (
        iT[33 * 64 + k] * src[33 * line] + iT[35 * 64 + k] * src[35 * line] + iT[37 * 64 + k] * src[37 * line] + iT[39 * 64 + k] * src[39 * line] +
        iT[41 * 64 + k] * src[41 * line] + iT[43 * 64 + k] * src[43 * line] + iT[45 * 64 + k] * src[45 * line] + iT[47 * 64 + k] * src[47 * line] +
        iT[49 * 64 + k] * src[49 * line] + iT[51 * 64 + k] * src[51 * line] + iT[53 * 64 + k] * src[53 * line] + iT[55 * 64 + k] * src[55 * line] +
        iT[57 * 64 + k] * src[57 * line] + iT[59 * 64 + k] * src[59 * line] + iT[61 * 64 + k] * src[61 * line] + iT[63 * 64 + k] * src[63 * line]));
    }
    for (k = 0;k<16;k++)
    {
      EO[k] = iT[2 * 64 + k] * src[2 * line] + iT[6 * 64 + k] * src[6 * line] + iT[10 * 64 + k] * src[10 * line] + iT[14 * 64 + k] * src[14 * line] +
        iT[18 * 64 + k] * src[18 * line] + iT[22 * 64 + k] * src[22 * line] + iT[26 * 64 + k] * src[26 * line] + iT[30 * 64 + k] * src[30 * line] +
        (zo ? 0 : (
        iT[34 * 64 + k] * src[34 * line] + iT[38 * 64 + k] * src[38 * line] + iT[42 * 64 + k] * src[42 * line] + iT[46 * 64 + k] * src[46 * line] +
        iT[50 * 64 + k] * src[50 * line] + iT[54 * 64 + k] * src[54 * line] + iT[58 * 64 + k] * src[58 * line] + iT[62 * 64 + k] * src[62 * line]));
    }
    for (k = 0;k<8;k++)
    {
      EEO[k] = iT[4 * 64 + k] * src[4 * line] + iT[12 * 64 + k] * src[12 * line] + iT[20 * 64 + k] * src[20 * line] + iT[28 * 64 + k] * src[28 * line] +
        (zo ? 0 : (
        iT[36 * 64 + k] * src[36 * line] + iT[44 * 64 + k] * src[44 * line] + iT[52 * 64 + k] * src[52 * line] + iT[60 * 64 + k] * src[60 * line]));
    }
    for (k = 0;k<4;k++)
    {
      EEEO[k] = iT[8 * 64 + k] * src[8 * line] + iT[24 * 64 + k] * src[24 * line] + (zo ? 0 : (iT[40 * 64 + k] * src[40 * line] + iT[56 * 64 + k] * src[56 * line]));
    }
    EEEEO[0] = iT[16 * 64 + 0] * src[16 * line] + (zo ? 0 : iT[48 * 64 + 0] * src[48 * line]);
    EEEEO[1] = iT[16 * 64 + 1] * src[16 * line] + (zo ? 0 : iT[48 * 64 + 1] * src[48 * line]);
    EEEEE[0] = iT[0 * 64 + 0] * src[0] + (zo ? 0 : iT[32 * 64 + 0] * src[32 * line]);
    EEEEE[1] = iT[0 * 64 + 1] * src[0] + (zo ? 0 : iT[32 * 64 + 1] * src[32 * line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for (k = 0;k<2;k++)
    {
      EEEE[k] = EEEEE[k] + EEEEO[k];
      EEEE[k + 2] = EEEEE[1 - k] - EEEEO[1 - k];
    }
    for (k = 0;k<4;k++)
    {
      EEE[k] = EEEE[k] + EEEO[k];
      EEE[k + 4] = EEEE[3 - k] - EEEO[3 - k];
    }
    for (k = 0;k<8;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 8] = EEE[7 - k] - EEO[7 - k];
    }
    for (k = 0;k<16;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 16] = EE[15 - k] - EO[15 - k];
    }
    for (k = 0;k<32;k++)
    {
#if ENABLE_SIMD_TRAFO
      dst[k]      = E[k] + O[k];
      dst[k + 32] = E[31 - k] - O[31 - k];
#else
      dst[k]      = Clip3( outputMinimum, outputMaximum, ( E[k] + O[k] + rnd_factor ) >> shift );
      dst[k + 32] = Clip3( outputMinimum, outputMaximum, ( E[31 - k] - O[31 - k] + rnd_factor ) >> shift );
#endif
    }
    src++;
    dst += uiTrSize;
  }

#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.roundClip8( orgDst, 32, line - iSkipLine, 32, outputMinimum, outputMaximum, rnd_factor, shift );


#endif
  memset( dst, 0, uiTrSize*iSkipLine * sizeof( TCoeff ) );
#endif
}



// ********************************** DST-VII **********************************
void fastForwardDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  int i;
  TCoeff rnd_factor = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P4[TRANSFORM_FORWARD][0];

  int c[4];
  TCoeff *pCoeff = dst;
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0] + src[3];
    c[1] = src[1] + src[3];
    c[2] = src[0] - src[1];
    c[3] = iT[2] * src[2];

    dst[0 * line] = (iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift;
    dst[1 * line] = (iT[2] * (src[0] + src[1] - src[3]) + rnd_factor) >> shift;
    dst[2 * line] = (iT[0] * c[2] + iT[1] * c[0] - c[3] + rnd_factor) >> shift;
    dst[3 * line] = (iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoeff + reducedLine;
    for (i = 0; i<4; i++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}

void fastInverseDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if ENABLE_SIMD_TRAFO
  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDST7P4[TRANSFORM_INVERSE][0] );
#else
  int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P4[TRANSFORM_INVERSE][0];

  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0 * line] + src[2 * line];
    c[1] = src[2 * line] + src[3 * line];
    c[2] = src[0 * line] - src[3 * line];
    c[3] = iT[2] * src[1 * line];

    dst[0] = Clip3(outputMinimum, outputMaximum, (iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift);
    dst[2] = Clip3(outputMinimum, outputMaximum, (iT[2] * (src[0 * line] - src[2 * line] + src[3 * line]) + rnd_factor) >> shift);
    dst[3] = Clip3(outputMinimum, outputMaximum, (iT[1] * c[0] + iT[0] * c[2] - c[3] + rnd_factor) >> shift);

    dst += 4;
    src++;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 2) * sizeof(TCoeff));
  }
#endif
}


void fastForwardDST7_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDST7P8[TRANSFORM_FORWARD][0] );
}


void fastInverseDST7_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDST7P8[TRANSFORM_INVERSE][0]);
}


void fastForwardDST7_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff a[5], b[5], c[5], d[5], t;
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P16[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  const int  cutoff = 16 - iSkipLine2;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 5; k++)
    {
      a[k] = src[    k] + src[11 + k];
      b[k] = src[9 - k] + src[11 + k];
      c[k] = src[    k] - src[ 9 - k];
      d[k] = src[    k] + src[ 9 - k] - src[11 + k];
    }

    t = iT[10] * src[10];

    dst[ 1 * line] = ( iT[ 2]*d[0] + iT[ 5]*d[1] + iT[ 8]*d[2] + iT[11]*d[3] + iT[14]*d[4] + add) >> shift;
    dst[ 4 * line] = ( iT[ 8]*d[0] + iT[14]*d[1] + iT[ 5]*d[2] - iT[ 2]*d[3] - iT[11]*d[4] + add) >> shift;
    dst[ 7 * line] = ( iT[14]*d[0] + iT[ 2]*d[1] - iT[11]*d[2] - iT[ 5]*d[3] + iT[ 8]*d[4] + add) >> shift;
    dst[10 * line] = ( iT[11]*d[0] - iT[ 8]*d[1] - iT[ 2]*d[2] + iT[14]*d[3] - iT[ 5]*d[4] + add) >> shift;
    dst[13 * line] = ( iT[ 5]*d[0] - iT[11]*d[1] + iT[14]*d[2] - iT[ 8]*d[3] + iT[ 2]*d[4] + add) >> shift;

    dst[5 * line] = ( iT[10] * (src[0] + src[1] - src[3] - src[4] + src[6] + src[7] - src[9] - src[10] + src[12] + src[13] - src[15]) + add) >> shift;

    dst[ 0 * line] = ( iT[0]*a[0] + iT[9]*b[0] + iT[1]*a[1] + iT[8]*b[1] + iT[2]*a[2] + iT[7]*b[2] + iT[3]*a[3] + iT[6]*b[3] + iT[4]*a[4] + iT[5]*b[4] + t + add ) >> shift;
    dst[ 2 * line] = ( iT[4]*c[0] - iT[5]*b[0] + iT[9]*c[1] - iT[0]*b[1] + iT[6]*c[2] + iT[3]*a[2] + iT[1]*c[3] + iT[8]*a[3] + iT[7]*a[4] + iT[2]*b[4] - t + add ) >> shift;
    dst[ 3 * line] = ( iT[6]*a[0] + iT[3]*b[0] + iT[2]*c[1] + iT[7]*a[1] + iT[9]*c[2] + iT[0]*a[2] + iT[4]*c[3] - iT[5]*b[3] - iT[1]*a[4] - iT[8]*b[4] + t + add ) >> shift;
    dst[ 6 * line] = ( iT[8]*a[0] + iT[1]*c[0] + iT[6]*c[1] - iT[3]*b[1] - iT[5]*a[2] - iT[4]*b[2] - iT[7]*c[3] - iT[2]*a[3] - iT[0]*c[4] + iT[9]*b[4] + t + add ) >> shift;
    dst[ 8 * line] = ( iT[4]*c[0] + iT[5]*a[0] - iT[0]*c[1] + iT[9]*b[1] - iT[3]*c[2] - iT[6]*a[2] + iT[1]*c[3] - iT[8]*b[3] + iT[2]*c[4] + iT[7]*a[4] - t + add ) >> shift;
    dst[ 9 * line] = ( iT[7]*c[0] + iT[2]*a[0] - iT[4]*a[1] - iT[5]*b[1] - iT[8]*c[2] + iT[1]*b[2] + iT[9]*a[3] + iT[0]*b[3] + iT[3]*c[4] - iT[6]*b[4] + t + add ) >> shift;
    dst[11 * line] = ( iT[9]*a[0] + iT[0]*b[0] - iT[8]*c[1] - iT[1]*a[1] + iT[2]*c[2] - iT[7]*b[2] + iT[6]*a[3] + iT[3]*b[3] - iT[5]*c[4] - iT[4]*a[4] - t + add ) >> shift;
    dst[12 * line] = ( iT[7]*c[0] - iT[2]*b[0] - iT[5]*c[1] - iT[4]*a[1] + iT[8]*a[2] + iT[1]*b[2] - iT[0]*a[3] - iT[9]*b[3] - iT[6]*c[4] + iT[3]*b[4] + t + add ) >> shift;
    dst[14 * line] = ( iT[3]*a[0] + iT[6]*b[0] - iT[7]*a[1] - iT[2]*b[1] + iT[0]*c[2] + iT[9]*a[2] - iT[4]*c[3] - iT[5]*a[3] + iT[8]*c[4] + iT[1]*a[4] - t + add ) >> shift;
    dst[15 * line] = ( iT[1]*c[0] - iT[8]*b[0] - iT[3]*c[1] + iT[6]*b[1] + iT[5]*c[2] - iT[4]*b[2] - iT[7]*c[3] + iT[2]*b[3] + iT[9]*c[4] - iT[0]*b[4] + t + add ) >> shift;

    src += 16;
    dst++;
  }

  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j < cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }

  if (iSkipLine2)
  {
    dst = pCoef + line * cutoff;
    memset(dst, 0, sizeof(TCoeff) * line * iSkipLine2);
  }
#else
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDST7P16[TRANSFORM_FORWARD][0] );
#endif
}


void fastInverseDST7_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff a[5], b[5], c[5], d[5], t;

  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P16[TRANSFORM_INVERSE][0];

  const int  reducedLine = line - iSkipLine;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 5; k++)
    {
      a[k] = src[       k * line] + src[(10 - k) * line];
      b[k] = src[(11 + k) * line] + src[(10 - k) * line];
      c[k] = src[       k * line] - src[(11 + k) * line];
      d[k] = src[       k * line] + src[(11 + k) * line] - src[(10 - k)*line];
    }

    t = iT[10] * src[5 * line];

    dst[ 2] = Clip3(outputMinimum, outputMaximum, (int)( iT[ 2]*d[0] + iT[ 8]*d[1] + iT[14]*d[2] + iT[11]*d[3] + iT[ 5]*d[4] + add ) >> shift);
    dst[ 5] = Clip3(outputMinimum, outputMaximum, (int)( iT[ 5]*d[0] + iT[14]*d[1] + iT[ 2]*d[2] - iT[ 8]*d[3] - iT[11]*d[4] + add ) >> shift);
    dst[ 8] = Clip3(outputMinimum, outputMaximum, (int)( iT[ 8]*d[0] + iT[ 5]*d[1] - iT[11]*d[2] - iT[ 2]*d[3] + iT[14]*d[4] + add ) >> shift);
    dst[11] = Clip3(outputMinimum, outputMaximum, (int)( iT[11]*d[0] - iT[ 2]*d[1] - iT[ 5]*d[2] + iT[14]*d[3] - iT[ 8]*d[4] + add ) >> shift);
    dst[14] = Clip3(outputMinimum, outputMaximum, (int)( iT[14]*d[0] - iT[11]*d[1] + iT[ 8]*d[2] - iT[ 5]*d[3] + iT[ 2]*d[4] + add ) >> shift);

    dst[10] = Clip3(outputMinimum, outputMaximum, (int)( iT[10]*(src[ 0*line]-src[ 2*line]+src[ 3*line]-src[5*line]
                                                                +src[ 6*line]-src[ 8*line]+src[ 9*line]-src[11*line]
                                                                +src[12*line]-src[14*line]+src[15*line]) + add ) >> shift);

    dst[ 0] = Clip3(outputMinimum, outputMaximum, (int)( iT[0]*a[0] + iT[9]*b[0] + iT[2]*a[1] + iT[7]*b[1] + iT[4]*a[2] + iT[5]*b[2] + iT[6]*a[3] + iT[3]*b[3] + iT[8]*a[4] + iT[1]*b[4] + t + add ) >> shift);
    dst[ 1] = Clip3(outputMinimum, outputMaximum, (int)( iT[1]*c[0] - iT[8]*b[0] + iT[5]*c[1] - iT[4]*b[1] + iT[9]*c[2] - iT[0]*b[2] + iT[2]*a[3] + iT[7]*c[3] + iT[6]*a[4] + iT[3]*c[4] + t + add ) >> shift);
    dst[ 3] = Clip3(outputMinimum, outputMaximum, (int)( iT[3]*a[0] + iT[6]*b[0] + iT[0]*c[1] + iT[9]*a[1] + iT[1]*a[2] + iT[8]*c[2] + iT[4]*c[3] - iT[5]*b[3] - iT[2]*a[4] - iT[7]*b[4] - t + add ) >> shift);
    dst[ 4] = Clip3(outputMinimum, outputMaximum, (int)( iT[4]*c[0] - iT[5]*b[0] + iT[6]*c[1] + iT[3]*a[1] + iT[7]*a[2] + iT[2]*b[2] - iT[1]*c[3] + iT[8]*b[3] - iT[9]*c[4] - iT[0]*a[4] - t + add ) >> shift);
    dst[ 6] = Clip3(outputMinimum, outputMaximum, (int)( iT[6]*a[0] + iT[3]*b[0] + iT[9]*c[1] + iT[0]*a[1] - iT[1]*a[2] - iT[8]*b[2] - iT[4]*c[3] - iT[5]*a[3] - iT[2]*c[4] + iT[7]*b[4] + t + add ) >> shift);
    dst[ 7] = Clip3(outputMinimum, outputMaximum, (int)( iT[7]*c[0] - iT[2]*b[0] + iT[8]*a[1] + iT[1]*b[1] - iT[6]*c[2] + iT[3]*b[2] - iT[9]*a[3] - iT[0]*b[3] + iT[5]*c[4] - iT[4]*b[4] + t + add ) >> shift);
    dst[ 9] = Clip3(outputMinimum, outputMaximum, (int)( iT[9]*a[0] + iT[0]*b[0] + iT[2]*c[1] - iT[7]*b[1] - iT[5]*c[2] - iT[4]*a[2] + iT[3]*a[3] + iT[6]*b[3] + iT[8]*c[4] - iT[1]*b[4] - t + add ) >> shift);
    dst[12] = Clip3(outputMinimum, outputMaximum, (int)( iT[1]*c[0] + iT[8]*a[0] - iT[5]*a[1] - iT[4]*b[1] - iT[0]*c[2] + iT[9]*b[2] + iT[7]*c[3] - iT[2]*b[3] - iT[6]*c[4] - iT[3]*a[4] + t + add ) >> shift);
    dst[13] = Clip3(outputMinimum, outputMaximum, (int)( iT[7]*c[0] + iT[2]*a[0] - iT[8]*c[1] + iT[1]*b[1] + iT[3]*c[2] - iT[6]*b[2] + iT[0]*a[3] + iT[9]*b[3] - iT[5]*a[4] - iT[4]*b[4] + t + add ) >> shift);
    dst[15] = Clip3(outputMinimum, outputMaximum, (int)( iT[4]*c[0] + iT[5]*a[0] - iT[3]*c[1] - iT[6]*a[1] + iT[2]*c[2] + iT[7]*a[2] - iT[1]*c[3] - iT[8]*a[3] + iT[0]*c[4] + iT[9]*a[4] - t + add ) >> shift);

    src++;
    dst += 16;
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine * 16) * sizeof(TCoeff));
  }
#else
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDST7P16[TRANSFORM_INVERSE][0]);
#endif
}


void fastForwardDST7_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff a[10][6];
  TCoeff t[2];
  TCoeff b[6];
  TCoeff c[2];

  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;
  const TMatrixCoeff *iT = g_trCoreDST7P32[TRANSFORM_FORWARD][0];
  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  const int  cutoff = 32 - iSkipLine2;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 6; k++)
    {
      a[0][k] = src[     k] - src[11 - k];
      a[1][k] = src[     k] + src[13 + k];
      a[2][k] = src[     k] + src[24 - k];
      a[3][k] = src[     k] - src[26 + k];
      a[4][k] = src[ 6 + k] + src[18 - k];
      a[5][k] = src[ 6 + k] + src[19 + k];
      a[6][k] = src[ 6 + k] - src[31 - k];
      a[7][k] = src[13 + k] - src[24 - k];
      a[8][k] = src[13 + k] + src[26 + k];
      a[9][k] = src[19 + k] + src[31 - k];

      b[k] = src[k] + src[11 - k] - src[13 + k] - src[24 - k] + src[26 + k];
    }
    for (k = 0; k < 2; k++)
    {
      c[k] = src[k] + src[3 - k] - src[5 + k] - src[8 - k] + src[10 + k] + src[13 - k] - src[15 + k] - src[18 - k] + src[20 + k] + src[23 - k] - src[25 + k] - src[28 - k] + src[30 + k];
    }

    t[0] = iT[12] * src[12] + iT[25] * src[25];
    t[1] = iT[12] * src[25] - iT[25] * src[12];

    dst[ 0 * line] = ( iT[0] * a[3][0] + iT[11] * a[6][5] + iT[13] * a[8][0] + iT[24] * a[9][5] + iT[1] * a[3][1] + iT[10] * a[6][4] + iT[14] * a[8][1] + iT[23] * a[9][4] + iT[2] * a[3][2] + iT[9] * a[6][3] + iT[15] * a[8][2] + iT[22] * a[9][3] + iT[3] * a[3][3] + iT[8] * a[6][2] + iT[16] * a[8][3] + iT[21] * a[9][2] + iT[4] * a[3][4] + iT[7] * a[6][1] + iT[17] * a[8][4] + iT[20] * a[9][1] + iT[5] * a[3][5] + iT[6] * a[6][0] + iT[18] * a[8][5] + iT[19] * a[9][0] + t[0] + add) >> shift;
    dst[ 1 * line] = (-iT[0] * a[5][2] + iT[11] * a[0][3] + iT[13] * a[4][2] + iT[24] * a[6][2] + iT[1] * a[9][1] + iT[10] * a[8][4] + iT[14] * a[3][4] + iT[23] * a[6][1] + iT[2] * a[0][0] - iT[9] * a[5][5] + iT[15] * a[6][5] + iT[22] * a[4][5] - iT[3] * a[5][3] + iT[8] * a[0][2] + iT[16] * a[4][3] + iT[21] * a[6][3] + iT[4] * a[9][0] + iT[7] * a[8][5] + iT[17] * a[3][5] + iT[20] * a[6][0] + iT[5] * a[0][1] - iT[6] * a[5][4] + iT[18] * a[6][4] + iT[19] * a[4][4] - t[1] + add) >> shift;
    dst[ 3 * line] = (-iT[0] * a[9][4] - iT[11] * a[5][4] + iT[13] * a[2][1] - iT[24] * a[7][1] - iT[1] * a[0][3] - iT[10] * a[1][3] + iT[14] * a[3][3] + iT[23] * a[2][3] + iT[2] * a[8][5] + iT[9] * a[9][0] + iT[15] * a[6][0] + iT[22] * a[3][5] - iT[3] * a[1][4] - iT[8] * a[0][4] + iT[16] * a[2][4] + iT[21] * a[3][4] - iT[4] * a[5][3] - iT[7] * a[9][3] - iT[17] * a[7][2] + iT[20] * a[2][2] + iT[5] * a[8][0] + iT[6] * a[1][0] - iT[18] * a[4][5] - iT[19] * a[7][0] + t[1] + add) >> shift;
    dst[ 4 * line] = (-iT[0] * a[3][2] - iT[11] * a[2][2] + iT[13] * a[1][2] + iT[24] * a[0][2] + iT[1] * a[6][0] + iT[10] * a[3][5] + iT[14] * a[9][0] + iT[23] * a[8][5] - iT[2] * a[2][3] - iT[9] * a[3][3] + iT[15] * a[0][3] + iT[22] * a[1][3] - iT[3] * a[7][0] + iT[8] * a[2][0] - iT[16] * a[9][5] - iT[21] * a[5][5] + iT[4] * a[4][4] + iT[7] * a[6][4] + iT[17] * a[0][1] - iT[20] * a[5][4] - iT[5] * a[7][4] - iT[6] * a[4][1] + iT[18] * a[8][4] + iT[19] * a[1][4] - t[0] + add) >> shift;
    dst[ 5 * line] = (-iT[0] * a[3][5] - iT[11] * a[6][0] - iT[13] * a[8][5] - iT[24] * a[9][0] + iT[1] * a[6][5] + iT[10] * a[3][0] + iT[14] * a[9][5] + iT[23] * a[8][0] - iT[2] * a[7][4] + iT[9] * a[2][4] - iT[15] * a[9][1] - iT[22] * a[5][1] - iT[3] * a[7][1] - iT[8] * a[4][4] + iT[16] * a[8][1] + iT[21] * a[1][1] + iT[4] * a[6][2] + iT[7] * a[4][2] - iT[17] * a[5][2] + iT[20] * a[0][3] - iT[5] * a[3][2] - iT[6] * a[2][2] + iT[18] * a[1][2] + iT[19] * a[0][2] + t[0] + add) >> shift;
    dst[ 8 * line] = ( iT[0] * a[9][3] + iT[11] * a[8][2] + iT[13] * a[3][2] + iT[24] * a[6][3] + iT[1] * a[1][5] + iT[10] * a[0][5] - iT[14] * a[2][5] - iT[23] * a[3][5] - iT[2] * a[1][3] - iT[9] * a[8][3] + iT[15] * a[7][3] + iT[22] * a[4][2] - iT[3] * a[9][5] - iT[8] * a[5][5] + iT[16] * a[2][0] - iT[21] * a[7][0] - iT[4] * a[1][1] - iT[7] * a[0][1] + iT[17] * a[2][1] + iT[20] * a[3][1] + iT[5] * a[5][1] + iT[6] * a[9][1] + iT[18] * a[7][4] - iT[19] * a[2][4] + t[1] + add) >> shift;
    dst[ 9 * line] = (-iT[0] * a[2][1] - iT[11] * a[3][1] + iT[13] * a[0][1] + iT[24] * a[1][1] + iT[1] * a[7][3] - iT[10] * a[2][3] + iT[14] * a[9][2] + iT[23] * a[5][2] + iT[2] * a[4][0] + iT[9] * a[7][5] - iT[15] * a[1][5] - iT[22] * a[8][5] + iT[3] * a[3][4] + iT[8] * a[2][4] - iT[16] * a[1][4] - iT[21] * a[0][4] + iT[4] * a[6][3] + iT[7] * a[3][2] + iT[17] * a[9][3] + iT[20] * a[8][2] + iT[5] * a[4][5] + iT[6] * a[6][5] + iT[18] * a[0][0] - iT[19] * a[5][5] - t[0] + add) >> shift;
    dst[10 * line] = (-iT[0] * a[6][1] - iT[11] * a[4][1] + iT[13] * a[5][1] - iT[24] * a[0][4] + iT[1] * a[2][2] - iT[10] * a[7][2] - iT[14] * a[5][3] - iT[23] * a[9][3] + iT[2] * a[6][4] + iT[9] * a[4][4] - iT[15] * a[5][4] + iT[22] * a[0][1] - iT[3] * a[2][5] + iT[8] * a[7][5] + iT[16] * a[5][0] + iT[21] * a[9][0] - iT[4] * a[7][0] - iT[7] * a[4][5] + iT[17] * a[8][0] + iT[20] * a[1][0] + iT[5] * a[4][2] + iT[6] * a[7][3] - iT[18] * a[1][3] - iT[19] * a[8][3] + t[0] + add) >> shift;
    dst[11 * line] = ( iT[0] * a[1][3] + iT[11] * a[0][3] - iT[13] * a[2][3] - iT[24] * a[3][3] + iT[1] * a[9][1] + iT[10] * a[5][1] - iT[14] * a[2][4] + iT[23] * a[7][4] + iT[2] * a[8][0] + iT[9] * a[9][5] + iT[15] * a[6][5] + iT[22] * a[3][0] - iT[3] * a[0][2] + iT[8] * a[5][3] - iT[16] * a[6][3] - iT[21] * a[4][3] - iT[4] * a[5][0] + iT[7] * a[0][5] + iT[17] * a[4][0] + iT[20] * a[6][0] - iT[5] * a[9][4] - iT[6] * a[5][4] + iT[18] * a[2][1] - iT[19] * a[7][1] - t[1] + add) >> shift;
    dst[13 * line] = (-iT[0] * a[0][0] - iT[11] * a[1][0] + iT[13] * a[3][0] + iT[24] * a[2][0] - iT[1] * a[5][4] + iT[10] * a[0][1] + iT[14] * a[4][4] + iT[23] * a[6][4] + iT[2] * a[9][3] + iT[9] * a[5][3] - iT[15] * a[2][2] + iT[22] * a[7][2] - iT[3] * a[8][3] - iT[8] * a[9][2] - iT[16] * a[6][2] - iT[21] * a[3][3] + iT[4] * a[1][4] + iT[7] * a[8][4] - iT[17] * a[7][4] - iT[20] * a[4][1] - iT[5] * a[0][5] - iT[6] * a[1][5] + iT[18] * a[3][5] + iT[19] * a[2][5] + t[1] + add) >> shift;
    dst[14 * line] = ( iT[0] * a[4][2] + iT[11] * a[7][3] - iT[13] * a[1][3] - iT[24] * a[8][3] + iT[1] * a[4][1] + iT[10] * a[6][1] + iT[14] * a[0][4] - iT[23] * a[5][1] - iT[2] * a[3][0] - iT[9] * a[2][0] + iT[15] * a[1][0] + iT[22] * a[0][0] - iT[3] * a[6][3] - iT[8] * a[4][3] + iT[16] * a[5][3] - iT[21] * a[0][2] - iT[4] * a[7][5] - iT[7] * a[4][0] + iT[17] * a[8][5] + iT[20] * a[1][5] + iT[5] * a[6][4] + iT[6] * a[3][1] + iT[18] * a[9][4] + iT[19] * a[8][1] - t[0] + add) >> shift;
    dst[15 * line] = (-iT[0] * a[7][4] - iT[11] * a[4][1] + iT[13] * a[8][4] + iT[24] * a[1][4] + iT[1] * a[2][2] + iT[10] * a[3][2] - iT[14] * a[0][2] - iT[23] * a[1][2] + iT[2] * a[2][1] - iT[9] * a[7][1] - iT[15] * a[5][4] - iT[22] * a[9][4] - iT[3] * a[7][5] + iT[8] * a[2][5] - iT[16] * a[9][0] - iT[21] * a[5][0] - iT[4] * a[2][0] - iT[7] * a[3][0] + iT[17] * a[0][0] + iT[20] * a[1][0] - iT[5] * a[2][3] + iT[6] * a[7][3] + iT[18] * a[5][2] + iT[19] * a[9][2] + t[0] + add) >> shift;
    dst[16 * line] = (-iT[0] * a[0][1] + iT[11] * a[5][4] - iT[13] * a[6][4] - iT[24] * a[4][4] + iT[1] * a[0][3] - iT[10] * a[5][2] + iT[14] * a[6][2] + iT[23] * a[4][2] - iT[2] * a[0][5] + iT[9] * a[5][0] - iT[15] * a[6][0] - iT[22] * a[4][0] - iT[3] * a[0][4] - iT[8] * a[1][4] + iT[16] * a[3][4] + iT[21] * a[2][4] + iT[4] * a[0][2] + iT[7] * a[1][2] - iT[17] * a[3][2] - iT[20] * a[2][2] - iT[5] * a[0][0] - iT[6] * a[1][0] + iT[18] * a[3][0] + iT[19] * a[2][0] - t[1] + add) >> shift;
    dst[18 * line] = ( iT[0] * a[0][5] + iT[11] * a[1][5] - iT[13] * a[3][5] - iT[24] * a[2][5] - iT[1] * a[1][0] - iT[10] * a[0][0] + iT[14] * a[2][0] + iT[23] * a[3][0] - iT[2] * a[5][1] + iT[9] * a[0][4] + iT[15] * a[4][1] + iT[22] * a[6][1] - iT[3] * a[8][1] - iT[8] * a[1][1] + iT[16] * a[4][4] + iT[21] * a[7][1] - iT[4] * a[9][2] - iT[7] * a[5][2] + iT[17] * a[2][3] - iT[20] * a[7][3] - iT[5] * a[9][3] - iT[6] * a[8][2] - iT[18] * a[3][2] - iT[19] * a[6][3] + t[1] + add) >> shift;
    dst[20 * line] = (-iT[0] * a[4][0] - iT[11] * a[6][0] - iT[13] * a[0][5] + iT[24] * a[5][0] + iT[1] * a[6][5] + iT[10] * a[4][5] - iT[14] * a[5][5] + iT[23] * a[0][0] - iT[2] * a[6][1] - iT[9] * a[3][4] - iT[15] * a[9][1] - iT[22] * a[8][4] + iT[3] * a[4][4] + iT[8] * a[7][1] - iT[16] * a[1][1] - iT[21] * a[8][1] - iT[4] * a[3][3] - iT[7] * a[2][3] + iT[17] * a[1][3] + iT[20] * a[0][3] + iT[5] * a[7][2] - iT[6] * a[2][2] + iT[18] * a[9][3] + iT[19] * a[5][3] + t[0] + add) >> shift;
    dst[21 * line] = (-iT[0] * a[1][2] - iT[11] * a[8][2] + iT[13] * a[7][2] + iT[24] * a[4][3] - iT[1] * a[1][5] - iT[10] * a[8][5] + iT[14] * a[7][5] + iT[23] * a[4][0] - iT[2] * a[5][2] - iT[9] * a[9][2] - iT[15] * a[7][3] + iT[22] * a[2][3] - iT[3] * a[5][5] - iT[8] * a[9][5] - iT[16] * a[7][0] + iT[21] * a[2][0] - iT[4] * a[8][1] - iT[7] * a[9][4] - iT[17] * a[6][4] - iT[20] * a[3][1] - iT[5] * a[8][4] - iT[6] * a[9][1] - iT[18] * a[6][1] - iT[19] * a[3][4] - t[1] + add) >> shift;
    dst[23 * line] = (-iT[0] * a[8][4] - iT[11] * a[9][1] - iT[13] * a[6][1] - iT[24] * a[3][4] + iT[1] * a[8][2] + iT[10] * a[1][2] - iT[14] * a[4][3] - iT[23] * a[7][2] + iT[2] * a[0][1] + iT[9] * a[1][1] - iT[15] * a[3][1] - iT[22] * a[2][1] - iT[3] * a[5][0] - iT[8] * a[9][0] - iT[16] * a[7][5] + iT[21] * a[2][5] + iT[4] * a[9][5] + iT[7] * a[8][0] + iT[17] * a[3][0] + iT[20] * a[6][5] - iT[5] * a[5][2] + iT[6] * a[0][3] + iT[18] * a[4][2] + iT[19] * a[6][2] + t[1] + add) >> shift;
    dst[24 * line] = (-iT[0] * a[2][3] + iT[11] * a[7][3] + iT[13] * a[5][2] + iT[24] * a[9][2] + iT[1] * a[4][1] + iT[10] * a[7][4] - iT[14] * a[1][4] - iT[23] * a[8][4] - iT[2] * a[4][5] - iT[9] * a[7][0] + iT[15] * a[1][0] + iT[22] * a[8][0] + iT[3] * a[4][3] + iT[8] * a[6][3] + iT[16] * a[0][2] - iT[21] * a[5][3] - iT[4] * a[2][5] - iT[7] * a[3][5] + iT[17] * a[0][5] + iT[20] * a[1][5] + iT[5] * a[2][1] + iT[6] * a[3][1] - iT[18] * a[0][1] - iT[19] * a[1][1] - t[0] + add) >> shift;
    dst[25 * line] = ( iT[0] * a[4][5] + iT[11] * a[6][5] + iT[13] * a[0][0] - iT[24] * a[5][5] + iT[1] * a[3][1] + iT[10] * a[2][1] - iT[14] * a[1][1] - iT[23] * a[0][1] - iT[2] * a[7][2] - iT[9] * a[4][3] + iT[15] * a[8][2] + iT[22] * a[1][2] - iT[3] * a[6][2] - iT[8] * a[3][3] - iT[16] * a[9][2] - iT[21] * a[8][3] - iT[4] * a[2][4] + iT[7] * a[7][4] + iT[17] * a[5][1] + iT[20] * a[9][1] + iT[5] * a[4][0] + iT[6] * a[6][0] + iT[18] * a[0][5] - iT[19] * a[5][0] + t[0] + add) >> shift;
    dst[26 * line] = ( iT[0] * a[8][0] + iT[11] * a[1][0] - iT[13] * a[4][5] - iT[24] * a[7][0] + iT[1] * a[5][4] + iT[10] * a[9][4] + iT[14] * a[7][1] - iT[23] * a[2][1] - iT[2] * a[1][2] - iT[9] * a[0][2] + iT[15] * a[2][2] + iT[22] * a[3][2] - iT[3] * a[9][2] - iT[8] * a[8][3] - iT[16] * a[3][3] - iT[21] * a[6][2] + iT[4] * a[0][4] - iT[7] * a[5][1] + iT[17] * a[6][1] + iT[20] * a[4][1] + iT[5] * a[8][5] + iT[6] * a[1][5] - iT[18] * a[4][0] - iT[19] * a[7][5] - t[1] + add) >> shift;
    dst[28 * line] = (-iT[0] * a[5][1] - iT[11] * a[9][1] - iT[13] * a[7][4] + iT[24] * a[2][4] + iT[1] * a[8][2] + iT[10] * a[9][3] + iT[14] * a[6][3] + iT[23] * a[3][2] - iT[2] * a[9][4] - iT[9] * a[8][1] - iT[15] * a[3][1] - iT[22] * a[6][4] + iT[3] * a[9][0] + iT[8] * a[5][0] - iT[16] * a[2][5] + iT[21] * a[7][5] - iT[4] * a[5][5] + iT[7] * a[0][0] + iT[17] * a[4][5] + iT[20] * a[6][5] + iT[5] * a[1][3] + iT[6] * a[0][3] - iT[18] * a[2][3] - iT[19] * a[3][3] + t[1] + add) >> shift;
    dst[29 * line] = (-iT[0] * a[6][4] - iT[11] * a[3][1] - iT[13] * a[9][4] - iT[24] * a[8][1] + iT[1] * a[7][3] + iT[10] * a[4][2] - iT[14] * a[8][3] - iT[23] * a[1][3] + iT[2] * a[3][5] + iT[9] * a[2][5] - iT[15] * a[1][5] - iT[22] * a[0][5] - iT[3] * a[2][4] - iT[8] * a[3][4] + iT[16] * a[0][4] + iT[21] * a[1][4] - iT[4] * a[4][3] - iT[7] * a[7][2] + iT[17] * a[1][2] + iT[20] * a[8][2] + iT[5] * a[3][0] + iT[6] * a[6][5] + iT[18] * a[8][0] + iT[19] * a[9][5] - t[0] + add) >> shift;
    dst[30 * line] = (-iT[0] * a[7][2] + iT[11] * a[2][2] - iT[13] * a[9][3] - iT[24] * a[5][3] - iT[1] * a[6][0] - iT[10] * a[4][0] + iT[14] * a[5][0] - iT[23] * a[0][5] - iT[2] * a[4][2] - iT[9] * a[6][2] - iT[15] * a[0][3] + iT[22] * a[5][2] + iT[3] * a[2][0] - iT[8] * a[7][0] - iT[16] * a[5][5] - iT[21] * a[9][5] + iT[4] * a[7][1] - iT[7] * a[2][1] + iT[17] * a[9][4] + iT[20] * a[5][4] + iT[5] * a[6][1] + iT[6] * a[4][1] - iT[18] * a[5][1] + iT[19] * a[0][4] + t[0] + add) >> shift;
    dst[31 * line] = (-iT[0] * a[8][5] - iT[11] * a[1][5] + iT[13] * a[4][0] + iT[24] * a[7][5] + iT[1] * a[1][0] + iT[10] * a[8][0] - iT[14] * a[7][0] - iT[23] * a[4][5] + iT[2] * a[8][4] + iT[9] * a[1][4] - iT[15] * a[4][1] - iT[22] * a[7][4] - iT[3] * a[1][1] - iT[8] * a[8][1] + iT[16] * a[7][1] + iT[21] * a[4][4] - iT[4] * a[8][3] - iT[7] * a[1][3] + iT[17] * a[4][2] + iT[20] * a[7][3] + iT[5] * a[1][2] + iT[6] * a[8][2] - iT[18] * a[7][2] - iT[19] * a[4][3] - t[1] + add) >> shift;

    dst[ 2 * line] = (iT[ 4]*b[0] + iT[ 9]*b[1] + iT[14]*b[2] + iT[19]*b[3] + iT[24]*b[4] + iT[29]*b[5] + add) >> shift;
    dst[ 7 * line] = (iT[14]*b[0] + iT[29]*b[1] + iT[19]*b[2] + iT[ 4]*b[3] - iT[ 9]*b[4] - iT[24]*b[5] + add) >> shift;
    dst[12 * line] = (iT[24]*b[0] + iT[14]*b[1] - iT[ 9]*b[2] - iT[29]*b[3] - iT[ 4]*b[4] + iT[19]*b[5] + add) >> shift;
    dst[17 * line] = (iT[29]*b[0] - iT[ 4]*b[1] - iT[24]*b[2] + iT[ 9]*b[3] + iT[19]*b[4] - iT[14]*b[5] + add) >> shift;
    dst[22 * line] = (iT[19]*b[0] - iT[24]*b[1] + iT[ 4]*b[2] + iT[14]*b[3] - iT[29]*b[4] + iT[ 9]*b[5] + add) >> shift;
    dst[27 * line] = (iT[ 9]*b[0] - iT[19]*b[1] + iT[29]*b[2] - iT[24]*b[3] + iT[14]*b[4] - iT[ 4]*b[5] + add) >> shift;

    dst[ 6 * line] = (iT[12]*c[0] + iT[25]*c[1] + add) >> shift;
    dst[19 * line] = (iT[25]*c[0] - iT[12]*c[1] + add) >> shift;

    src += 32;
    dst++;
  }

  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j < cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }

  if (iSkipLine2)
  {
    dst = pCoef + line * cutoff;
    memset(dst, 0, sizeof(TCoeff) * line * iSkipLine2);
  }
#else
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDST7P32[TRANSFORM_FORWARD][0] );
#endif
}


void fastInverseDST7_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff a[10][6];
  TCoeff t[2];
  TCoeff b[6];
  TCoeff c[2];

  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;
  const TMatrixCoeff *iT = g_trCoreDST7P32[TRANSFORM_INVERSE][0];
  const int  reducedLine = line - iSkipLine;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 6; k++)
    {
      a[0][k] = src[      k  * line] + src[(12 - k) * line];
      a[1][k] = src[      k  * line] - src[(13 + k) * line];
      a[2][k] = src[      k  * line] + src[(25 - k) * line];
      a[3][k] = src[      k  * line] - src[(26 + k) * line];
      a[4][k] = src[( 7 + k) * line] + src[(18 - k) * line];
      a[5][k] = src[( 7 + k) * line] - src[(20 + k) * line];
      a[6][k] = src[( 7 + k) * line] + src[(31 - k) * line];
      a[7][k] = src[(13 + k) * line] + src[(25 - k) * line];
      a[8][k] = src[(13 + k) * line] - src[(26 + k) * line];
      a[9][k] = src[(20 + k) * line] + src[(31 - k) * line];

      b[k] = src[k * line] - src[(12-k) * line] + src[(13+k) * line] - src[(25-k) * line] + src[(26+k) * line];
    }
    for (k = 0; k < 2; k++)
    {
      c[k] = src[k * line] - src[(4-k) * line] + src[(5+k) * line] - src[(9-k) * line] + src[(10+k) * line] - src[(14-k) * line] + src[(15+k)*line] - src[(19-k)*line] + src[(20+k)*line] - src[(24-k)*line] + src[(25+k)*line] - src[(29-k)*line] + src[(30+k)*line];
    }

    t[0] = iT[12] * src[6*line] + iT[25] * src[19*line];
    t[1] = iT[25] * src[6*line] - iT[12] * src[19*line];

    dst[ 0] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[1][0] - iT[11] * a[8][0] + iT[13] * a[7][0] + iT[24] * a[4][5] - iT[1] * a[8][5] + iT[10] * a[1][5] + iT[14] * a[4][0] + iT[23] * a[7][5] + iT[2] * a[1][1] - iT[9] * a[8][1] + iT[15] * a[7][1] + iT[22] * a[4][4] - iT[3] * a[8][4] + iT[8] * a[1][4] + iT[16] * a[4][1] + iT[21] * a[7][4] + iT[4] * a[1][2] - iT[7] * a[8][2] + iT[17] * a[7][2] + iT[20] * a[4][3] - iT[5] * a[8][3] + iT[6] * a[1][3] + iT[18] * a[4][2] + iT[19] * a[7][3] + t[0] + add) >> shift);
    dst[ 1] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[4][2] - iT[11] * a[6][2] + iT[13] * a[0][3] + iT[24] * a[5][2] + iT[1] * a[2][0] + iT[10] * a[7][0] + iT[14] * a[5][5] - iT[23] * a[9][5] + iT[2] * a[7][2] + iT[9] * a[2][2] - iT[15] * a[9][3] + iT[22] * a[5][3] - iT[3] * a[6][0] - iT[8] * a[4][0] + iT[16] * a[5][0] + iT[21] * a[0][5] - iT[4] * a[4][1] - iT[7] * a[6][1] + iT[17] * a[0][4] + iT[20] * a[5][1] + iT[5] * a[2][1] + iT[6] * a[7][1] + iT[18] * a[5][4] - iT[19] * a[9][4] + t[1] + add) >> shift);
    dst[ 2] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[2][4] - iT[11] * a[3][4] + iT[13] * a[0][4] + iT[24] * a[1][4] + iT[1] * a[4][3] + iT[10] * a[7][2] + iT[14] * a[1][2] - iT[23] * a[8][2] + iT[2] * a[3][0] - iT[9] * a[6][5] - iT[15] * a[8][0] + iT[22] * a[9][5] - iT[3] * a[6][4] + iT[8] * a[3][1] + iT[16] * a[9][4] - iT[21] * a[8][1] + iT[4] * a[7][3] + iT[7] * a[4][2] - iT[17] * a[8][3] + iT[20] * a[1][3] - iT[5] * a[3][5] - iT[6] * a[2][5] + iT[18] * a[1][5] + iT[19] * a[0][5] + t[1] + add) >> shift);
    dst[ 3] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[5][4] + iT[11] * a[0][1] - iT[13] * a[4][4] - iT[24] * a[6][4] - iT[1] * a[1][3] - iT[10] * a[0][3] + iT[14] * a[2][3] + iT[23] * a[3][3] - iT[2] * a[0][4] - iT[9] * a[1][4] + iT[15] * a[3][4] + iT[22] * a[2][4] + iT[3] * a[0][0] + iT[8] * a[5][5] - iT[16] * a[6][5] - iT[21] * a[4][5] + iT[4] * a[5][0] - iT[7] * a[9][0] + iT[17] * a[7][5] + iT[20] * a[2][5] - iT[5] * a[8][2] + iT[6] * a[9][3] - iT[18] * a[6][3] + iT[19] * a[3][2] + t[0] + add) >> shift);
    dst[ 5] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[1][5] + iT[11] * a[8][5] - iT[13] * a[7][5] - iT[24] * a[4][0] + iT[1] * a[5][1] + iT[10] * a[0][4] - iT[14] * a[4][1] - iT[23] * a[6][1] - iT[2] * a[8][3] + iT[9] * a[9][2] - iT[15] * a[6][2] + iT[22] * a[3][3] - iT[3] * a[0][2] - iT[8] * a[1][2] + iT[16] * a[3][2] + iT[21] * a[2][2] - iT[4] * a[9][4] + iT[7] * a[5][4] + iT[17] * a[2][1] + iT[20] * a[7][1] + iT[5] * a[1][0] - iT[6] * a[8][0] + iT[18] * a[7][0] + iT[19] * a[4][5] - t[0] + add) >> shift);
    dst[ 6] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[7][5] - iT[11] * a[2][5] + iT[13] * a[9][0] - iT[24] * a[5][0] + iT[1] * a[3][4] - iT[10] * a[6][1] - iT[14] * a[8][4] + iT[23] * a[9][1] + iT[2] * a[4][2] + iT[9] * a[7][3] + iT[15] * a[1][3] - iT[22] * a[8][3] - iT[3] * a[2][2] - iT[8] * a[3][2] + iT[16] * a[0][2] + iT[21] * a[1][2] - iT[4] * a[6][4] - iT[7] * a[4][4] + iT[17] * a[5][4] + iT[20] * a[0][1] + iT[5] * a[7][0] + iT[6] * a[2][0] - iT[18] * a[9][5] + iT[19] * a[5][5] - t[1] + add) >> shift);
    dst[ 7] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[6][3] - iT[11] * a[4][3] + iT[13] * a[5][3] + iT[24] * a[0][2] + iT[1] * a[7][1] + iT[10] * a[4][4] - iT[14] * a[8][1] + iT[23] * a[1][1] - iT[2] * a[7][5] - iT[9] * a[4][0] + iT[15] * a[8][5] - iT[22] * a[1][5] + iT[3] * a[7][3] + iT[8] * a[2][3] - iT[16] * a[9][2] + iT[21] * a[5][2] - iT[4] * a[6][5] + iT[7] * a[3][0] + iT[17] * a[9][5] - iT[20] * a[8][0] + iT[5] * a[6][1] - iT[6] * a[3][4] - iT[18] * a[9][1] + iT[19] * a[8][4] - t[1] + add) >> shift);
    dst[ 8] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[1][1] - iT[11] * a[0][1] + iT[13] * a[2][1] + iT[24] * a[3][1] + iT[1] * a[1][3] - iT[10] * a[8][3] + iT[14] * a[7][3] + iT[23] * a[4][2] - iT[2] * a[9][1] + iT[9] * a[8][4] - iT[15] * a[3][4] + iT[22] * a[6][1] + iT[3] * a[5][5] + iT[8] * a[0][0] - iT[16] * a[4][5] - iT[21] * a[6][5] + iT[4] * a[0][5] + iT[7] * a[1][5] - iT[17] * a[3][5] - iT[20] * a[2][5] + iT[5] * a[5][3] - iT[6] * a[9][3] + iT[18] * a[7][2] + iT[19] * a[2][2] - t[0] + add) >> shift);
    dst[10] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[8][3] - iT[11] * a[1][3] - iT[13] * a[4][2] - iT[24] * a[7][3] - iT[1] * a[8][0] + iT[10] * a[1][0] + iT[14] * a[4][5] + iT[23] * a[7][0] + iT[2] * a[5][3] + iT[9] * a[0][2] - iT[15] * a[4][3] - iT[22] * a[6][3] - iT[3] * a[5][0] - iT[8] * a[0][5] + iT[16] * a[4][0] + iT[21] * a[6][0] + iT[4] * a[1][4] + iT[7] * a[0][4] - iT[17] * a[2][4] - iT[20] * a[3][4] - iT[5] * a[1][1] - iT[6] * a[0][1] + iT[18] * a[2][1] + iT[19] * a[3][1] + t[0] + add) >> shift);
    dst[11] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[7][0] + iT[11] * a[2][0] - iT[13] * a[9][5] + iT[24] * a[5][5] + iT[1] * a[2][5] + iT[10] * a[7][5] + iT[14] * a[5][0] - iT[23] * a[9][0] - iT[2] * a[2][1] - iT[9] * a[3][1] + iT[15] * a[0][1] + iT[22] * a[1][1] - iT[3] * a[7][4] - iT[8] * a[4][1] + iT[16] * a[8][4] - iT[21] * a[1][4] + iT[4] * a[3][2] - iT[7] * a[6][3] - iT[17] * a[8][2] + iT[20] * a[9][3] + iT[5] * a[4][2] + iT[6] * a[6][2] - iT[18] * a[0][3] - iT[19] * a[5][2] + t[1] + add) >> shift);
    dst[13] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[9][5] - iT[11] * a[8][0] + iT[13] * a[3][0] - iT[24] * a[6][5] - iT[1] * a[8][5] + iT[10] * a[9][0] - iT[14] * a[6][0] + iT[23] * a[3][5] + iT[2] * a[5][4] - iT[9] * a[9][4] + iT[15] * a[7][1] + iT[22] * a[2][1] - iT[3] * a[1][4] + iT[8] * a[8][4] - iT[16] * a[7][4] - iT[21] * a[4][1] - iT[4] * a[0][2] - iT[7] * a[5][3] + iT[17] * a[6][3] + iT[20] * a[4][3] + iT[5] * a[0][3] + iT[6] * a[1][3] - iT[18] * a[3][3] - iT[19] * a[2][3] + t[0] + add) >> shift);
    dst[15] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[9][1] + iT[11] * a[5][1] + iT[13] * a[2][4] + iT[24] * a[7][4] + iT[1] * a[9][3] - iT[10] * a[5][3] - iT[14] * a[2][2] - iT[23] * a[7][2] - iT[2] * a[9][5] + iT[9] * a[5][5] + iT[15] * a[2][0] + iT[22] * a[7][0] + iT[3] * a[9][4] - iT[8] * a[8][1] + iT[16] * a[3][1] - iT[21] * a[6][4] - iT[4] * a[9][2] + iT[7] * a[8][3] - iT[17] * a[3][3] + iT[20] * a[6][2] + iT[5] * a[9][0] - iT[6] * a[8][5] + iT[18] * a[3][5] - iT[19] * a[6][0] - t[0] + add) >> shift);
    dst[16] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[4][4] + iT[11] * a[7][1] + iT[13] * a[1][1] - iT[24] * a[8][1] + iT[1] * a[6][2] - iT[10] * a[3][3] - iT[14] * a[9][2] + iT[23] * a[8][3] - iT[2] * a[6][1] - iT[9] * a[4][1] + iT[15] * a[5][1] + iT[22] * a[0][4] - iT[3] * a[4][5] - iT[8] * a[6][5] + iT[16] * a[0][0] + iT[21] * a[5][5] - iT[4] * a[6][0] + iT[7] * a[3][5] + iT[17] * a[9][0] - iT[20] * a[8][5] + iT[5] * a[6][3] + iT[6] * a[4][3] - iT[18] * a[5][3] - iT[19] * a[0][2] - t[1] + add) >> shift);
    dst[17] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[7][2] - iT[11] * a[4][3] + iT[13] * a[8][2] - iT[24] * a[1][2] + iT[1] * a[7][1] + iT[10] * a[2][1] - iT[14] * a[9][4] + iT[23] * a[5][4] - iT[2] * a[3][5] + iT[9] * a[6][0] + iT[15] * a[8][5] - iT[22] * a[9][0] - iT[3] * a[2][3] - iT[8] * a[7][3] - iT[16] * a[5][2] + iT[21] * a[9][2] + iT[4] * a[4][5] + iT[7] * a[7][0] + iT[17] * a[1][0] - iT[20] * a[8][0] - iT[5] * a[2][4] - iT[6] * a[3][4] + iT[18] * a[0][4] + iT[19] * a[1][4] - t[1] + add) >> shift);
    dst[18] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[9][0] + iT[11] * a[8][5] - iT[13] * a[3][5] + iT[24] * a[6][0] + iT[1] * a[5][1] - iT[10] * a[9][1] + iT[14] * a[7][4] + iT[23] * a[2][4] + iT[2] * a[0][3] + iT[9] * a[5][2] - iT[15] * a[6][2] - iT[22] * a[4][2] + iT[3] * a[1][2] + iT[8] * a[0][2] - iT[16] * a[2][2] - iT[21] * a[3][2] - iT[4] * a[8][1] + iT[7] * a[1][1] + iT[17] * a[4][4] + iT[20] * a[7][1] + iT[5] * a[9][5] - iT[6] * a[8][0] + iT[18] * a[3][0] - iT[19] * a[6][5] - t[0] + add) >> shift);
    dst[20] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[8][2] - iT[11] * a[9][3] + iT[13] * a[6][3] - iT[24] * a[3][2] + iT[1] * a[0][1] + iT[10] * a[5][4] - iT[14] * a[6][4] - iT[23] * a[4][4] + iT[2] * a[1][5] + iT[9] * a[0][5] - iT[15] * a[2][5] - iT[22] * a[3][5] - iT[3] * a[9][2] + iT[8] * a[5][2] + iT[16] * a[2][3] + iT[21] * a[7][3] + iT[4] * a[5][5] - iT[7] * a[9][5] + iT[17] * a[7][0] + iT[20] * a[2][0] + iT[5] * a[0][4] + iT[6] * a[5][1] - iT[18] * a[6][1] - iT[19] * a[4][1] + t[0] + add) >> shift);
    dst[21] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[2][1] - iT[11] * a[7][1] - iT[13] * a[5][4] + iT[24] * a[9][4] - iT[1] * a[6][2] - iT[10] * a[4][2] + iT[14] * a[5][2] + iT[23] * a[0][3] - iT[2] * a[2][4] - iT[9] * a[7][4] - iT[15] * a[5][1] + iT[22] * a[9][1] - iT[3] * a[6][5] - iT[8] * a[4][5] + iT[16] * a[5][5] + iT[21] * a[0][0] - iT[4] * a[4][0] - iT[7] * a[7][5] - iT[17] * a[1][5] + iT[20] * a[8][5] - iT[5] * a[7][2] - iT[6] * a[4][3] + iT[18] * a[8][2] - iT[19] * a[1][2] + t[1] + add) >> shift);
    dst[22] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[6][1] - iT[11] * a[3][4] - iT[13] * a[9][1] + iT[24] * a[8][4] + iT[1] * a[4][3] + iT[10] * a[6][3] - iT[14] * a[0][2] - iT[23] * a[5][3] + iT[2] * a[7][0] + iT[9] * a[4][5] - iT[15] * a[8][0] + iT[22] * a[1][0] - iT[3] * a[3][1] + iT[8] * a[6][4] + iT[16] * a[8][1] - iT[21] * a[9][4] - iT[4] * a[2][3] - iT[7] * a[3][3] + iT[17] * a[0][3] + iT[20] * a[1][3] - iT[5] * a[7][5] - iT[6] * a[2][5] + iT[18] * a[9][0] - iT[19] * a[5][0] + t[1] + add) >> shift);
    dst[23] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[0][3] - iT[11] * a[1][3] + iT[13] * a[3][3] + iT[24] * a[2][3] - iT[1] * a[8][0] + iT[10] * a[9][5] - iT[14] * a[6][5] + iT[23] * a[3][0] + iT[2] * a[8][2] - iT[9] * a[1][2] - iT[15] * a[4][3] - iT[22] * a[7][2] + iT[3] * a[0][5] + iT[8] * a[5][0] - iT[16] * a[6][0] - iT[21] * a[4][0] + iT[4] * a[8][4] - iT[7] * a[9][1] + iT[17] * a[6][1] - iT[20] * a[3][4] - iT[5] * a[5][4] - iT[6] * a[0][1] + iT[18] * a[4][4] + iT[19] * a[6][4] + t[0] + add) >> shift);
    dst[26] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[3][0] - iT[11] * a[2][0] + iT[13] * a[1][0] + iT[24] * a[0][0] - iT[1] * a[2][5] - iT[10] * a[3][5] + iT[14] * a[0][5] + iT[23] * a[1][5] + iT[2] * a[4][4] + iT[9] * a[6][4] - iT[15] * a[0][1] - iT[22] * a[5][4] - iT[3] * a[4][1] - iT[8] * a[7][4] - iT[16] * a[1][4] + iT[21] * a[8][4] + iT[4] * a[2][2] + iT[7] * a[7][2] + iT[17] * a[5][3] - iT[20] * a[9][3] + iT[5] * a[3][3] - iT[6] * a[6][2] - iT[18] * a[8][3] + iT[19] * a[9][2] - t[1] + add) >> shift);
    dst[27] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[3][3] + iT[11] * a[6][2] + iT[13] * a[8][3] - iT[24] * a[9][2] - iT[1] * a[2][0] - iT[10] * a[3][0] + iT[14] * a[0][0] + iT[23] * a[1][0] - iT[2] * a[6][3] + iT[9] * a[3][2] + iT[15] * a[9][3] - iT[22] * a[8][2] - iT[3] * a[4][0] - iT[8] * a[6][0] + iT[16] * a[0][5] + iT[21] * a[5][0] - iT[4] * a[7][4] - iT[7] * a[2][4] + iT[17] * a[9][1] - iT[20] * a[5][1] - iT[5] * a[4][4] - iT[6] * a[7][1] - iT[18] * a[1][1] + iT[19] * a[8][1] - t[1] + add) >> shift);
    dst[28] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[0][4] + iT[11] * a[5][1] - iT[13] * a[6][1] - iT[24] * a[4][1] + iT[1] * a[9][3] - iT[10] * a[8][2] + iT[14] * a[3][2] - iT[23] * a[6][3] - iT[2] * a[1][0] - iT[9] * a[0][0] + iT[15] * a[2][0] + iT[22] * a[3][0] + iT[3] * a[8][1] - iT[8] * a[9][4] + iT[16] * a[6][4] - iT[21] * a[3][1] - iT[4] * a[5][2] - iT[7] * a[0][3] + iT[17] * a[4][2] + iT[20] * a[6][2] + iT[5] * a[1][5] - iT[6] * a[8][5] + iT[18] * a[7][5] + iT[19] * a[4][0] - t[0] + add) >> shift);
    dst[30] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[5][3] - iT[11] * a[9][3] + iT[13] * a[7][2] + iT[24] * a[2][2] + iT[1] * a[0][1] + iT[10] * a[1][1] - iT[14] * a[3][1] - iT[23] * a[2][1] + iT[2] * a[9][0] - iT[9] * a[5][0] - iT[15] * a[2][5] - iT[22] * a[7][5] - iT[3] * a[5][2] + iT[8] * a[9][2] - iT[16] * a[7][3] - iT[21] * a[2][3] - iT[4] * a[0][0] - iT[7] * a[1][0] + iT[17] * a[3][0] + iT[20] * a[2][0] - iT[5] * a[9][1] + iT[6] * a[5][1] + iT[18] * a[2][4] + iT[19] * a[7][4] + t[0] + add) >> shift);
    dst[31] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[3][5] + iT[11] * a[2][5] - iT[13] * a[1][5] - iT[24] * a[0][5] - iT[1] * a[3][4] - iT[10] * a[2][4] + iT[14] * a[1][4] + iT[23] * a[0][4] + iT[2] * a[3][3] + iT[9] * a[2][3] - iT[15] * a[1][3] - iT[22] * a[0][3] - iT[3] * a[3][2] - iT[8] * a[2][2] + iT[16] * a[1][2] + iT[21] * a[0][2] + iT[4] * a[3][1] + iT[7] * a[2][1] - iT[17] * a[1][1] - iT[20] * a[0][1] - iT[5] * a[3][0] - iT[6] * a[2][0] + iT[18] * a[1][0] + iT[19] * a[0][0] + t[1] + add) >> shift);

    dst[ 4] = Clip3(outputMinimum, outputMaximum, (int)(iT[ 4] * b[0] + iT[14] * b[1] + iT[24] * b[2] + iT[29] * b[3] + iT[19] * b[4] + iT[ 9] * b[5] + add) >> shift);
    dst[ 9] = Clip3(outputMinimum, outputMaximum, (int)(iT[ 9] * b[0] + iT[29] * b[1] + iT[14] * b[2] - iT[ 4] * b[3] - iT[24] * b[4] - iT[19] * b[5] + add) >> shift);
    dst[14] = Clip3(outputMinimum, outputMaximum, (int)(iT[14] * b[0] + iT[19] * b[1] - iT[ 9] * b[2] - iT[24] * b[3] + iT[ 4] * b[4] + iT[29] * b[5] + add) >> shift);
    dst[19] = Clip3(outputMinimum, outputMaximum, (int)(iT[19] * b[0] + iT[ 4] * b[1] - iT[29] * b[2] + iT[ 9] * b[3] + iT[14] * b[4] - iT[24] * b[5] + add) >> shift);
    dst[24] = Clip3(outputMinimum, outputMaximum, (int)(iT[24] * b[0] - iT[ 9] * b[1] - iT[ 4] * b[2] + iT[19] * b[3] - iT[29] * b[4] + iT[14] * b[5] + add) >> shift);
    dst[29] = Clip3(outputMinimum, outputMaximum, (int)(iT[29] * b[0] - iT[24] * b[1] + iT[19] * b[2] - iT[14] * b[3] + iT[ 9] * b[4] - iT[ 4] * b[5] + add) >> shift);

    dst[12] = Clip3(outputMinimum, outputMaximum, (int)(iT[12]*c[0] + iT[25]*c[1] + add) >> shift);
    dst[25] = Clip3(outputMinimum, outputMaximum, (int)(iT[25]*c[0] - iT[12]*c[1] + add) >> shift);

    src++;
    dst += 32;
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine * 32) * sizeof(TCoeff));
  }
#else
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDST7P32[TRANSFORM_INVERSE][0] );
#endif
}


// ********************************** DCT-VIII **********************************
void fastForwardDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  int i;
  int rnd_factor = 1 << (shift - 1);
  const TMatrixCoeff *iT = g_trCoreDCT8P4[TRANSFORM_FORWARD][0];

  int c[4];
  TCoeff *pCoeff = dst;
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0] + src[3];
    c[1] = src[2] + src[0];
    c[2] = src[3] - src[2];
    c[3] = iT[1] * src[1];

    dst[0 * line] = (iT[3] * c[0] + iT[2] * c[1] + c[3] + rnd_factor) >> shift;
    dst[1 * line] = (iT[1] * (src[0] - src[2] - src[3]) + rnd_factor) >> shift;
    dst[2 * line] = (iT[3] * c[2] + iT[2] * c[0] - c[3] + rnd_factor) >> shift;
    dst[3 * line] = (iT[3] * c[1] - iT[2] * c[2] - c[3] + rnd_factor) >> shift;

    src += 4;
    dst++;
  }
  if (iSkipLine)
  {
    dst = pCoeff + reducedLine;
    for (i = 0; i<4; i++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }
}


void fastInverseDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if ENABLE_SIMD_TRAFO
  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDCT8P4[TRANSFORM_INVERSE][0] );
#else
  int i;
  int rnd_factor = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT8P4[TRANSFORM_INVERSE][0];

  int c[4];
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0 * line] + src[3 * line];
    c[1] = src[2 * line] + src[0 * line];
    c[2] = src[3 * line] - src[2 * line];
    c[3] = iT[1] * src[1 * line];

    dst[0] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[0] + iT[2] * c[1] + c[3] + rnd_factor) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (iT[1] * (src[0 * line] - src[2 * line] - src[3 * line]) + rnd_factor) >> shift);
    dst[2] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[2] + iT[2] * c[0] - c[3] + rnd_factor) >> shift);
    dst[3] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[1] - iT[2] * c[2] - c[3] + rnd_factor) >> shift);

    dst += 4;
    src++;
  }
  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 2) * sizeof(TCoeff));
  }
#endif
}


void fastForwardDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
  _fastForwardMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDCT8P8[TRANSFORM_FORWARD][0] );
}


void fastInverseDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDCT8P8[TRANSFORM_INVERSE][0] );
}


void fastForwardDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff a[5], b[5], c[5], d[5], t;
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P16[TRANSFORM_FORWARD][0];

  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  const int  cutoff = 16 - iSkipLine2;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 5; k++)
    {
      a[k] = src[15 - k] + src[ 4 - k];
      b[k] = src[ 6 + k] + src[ 4 - k];
      c[k] = src[15 - k] - src[ 6 + k];
      d[k] = src[15 - k] + src[ 6 + k] - src[ 4 - k];
    }

    t = iT[10] * src[5];

    dst[ 1 * line] = ( - iT[ 2]*d[0] - iT[ 5]*d[1] - iT[ 8]*d[2] - iT[11]*d[3] - iT[14]*d[4] + add) >> shift;
    dst[ 4 * line] = (   iT[ 8]*d[0] + iT[14]*d[1] + iT[ 5]*d[2] - iT[ 2]*d[3] - iT[11]*d[4] + add) >> shift;
    dst[ 7 * line] = ( - iT[14]*d[0] - iT[ 2]*d[1] + iT[11]*d[2] + iT[ 5]*d[3] - iT[ 8]*d[4] + add) >> shift;
    dst[10 * line] = (   iT[11]*d[0] - iT[ 8]*d[1] - iT[ 2]*d[2] + iT[14]*d[3] - iT[ 5]*d[4] + add) >> shift;
    dst[13 * line] = ( - iT[ 5]*d[0] + iT[11]*d[1] - iT[14]*d[2] + iT[ 8]*d[3] - iT[ 2]*d[4] + add) >> shift;

    dst[ 5 * line] = ( - iT[10] * (src[15] + src[14] - src[12] - src[11] + src[9] + src[8] - src[6] - src[5] + src[3] + src[2] - src[0]) + add) >> shift;

    dst[ 0 * line] = (   iT[0]*a[0] + iT[9]*b[0] + iT[1]*a[1] + iT[8]*b[1] + iT[2]*a[2] + iT[7]*b[2] + iT[3]*a[3] + iT[6]*b[3] + iT[4]*a[4] + iT[5]*b[4] + t + add ) >> shift;
    dst[ 2 * line] = (   iT[4]*c[0] - iT[5]*b[0] + iT[9]*c[1] - iT[0]*b[1] + iT[6]*c[2] + iT[3]*a[2] + iT[1]*c[3] + iT[8]*a[3] + iT[7]*a[4] + iT[2]*b[4] - t + add ) >> shift;
    dst[ 3 * line] = ( - iT[6]*a[0] - iT[3]*b[0] - iT[2]*c[1] - iT[7]*a[1] - iT[9]*c[2] - iT[0]*a[2] - iT[4]*c[3] + iT[5]*b[3] + iT[1]*a[4] + iT[8]*b[4] - t + add ) >> shift;
    dst[ 6 * line] = (   iT[8]*a[0] + iT[1]*c[0] + iT[6]*c[1] - iT[3]*b[1] - iT[5]*a[2] - iT[4]*b[2] - iT[7]*c[3] - iT[2]*a[3] - iT[0]*c[4] + iT[9]*b[4] + t + add ) >> shift;
    dst[ 8 * line] = (   iT[4]*c[0] + iT[5]*a[0] - iT[0]*c[1] + iT[9]*b[1] - iT[3]*c[2] - iT[6]*a[2] + iT[1]*c[3] - iT[8]*b[3] + iT[2]*c[4] + iT[7]*a[4] - t + add ) >> shift;
    dst[ 9 * line] = ( - iT[7]*c[0] - iT[2]*a[0] + iT[4]*a[1] + iT[5]*b[1] + iT[8]*c[2] - iT[1]*b[2] - iT[9]*a[3] - iT[0]*b[3] - iT[3]*c[4] + iT[6]*b[4] - t + add ) >> shift;
    dst[11 * line] = ( - iT[9]*a[0] - iT[0]*b[0] + iT[8]*c[1] + iT[1]*a[1] - iT[2]*c[2] + iT[7]*b[2] - iT[6]*a[3] - iT[3]*b[3] + iT[5]*c[4] + iT[4]*a[4] + t + add ) >> shift;
    dst[12 * line] = (   iT[7]*c[0] - iT[2]*b[0] - iT[5]*c[1] - iT[4]*a[1] + iT[8]*a[2] + iT[1]*b[2] - iT[0]*a[3] - iT[9]*b[3] - iT[6]*c[4] + iT[3]*b[4] + t + add ) >> shift;
    dst[14 * line] = (   iT[3]*a[0] + iT[6]*b[0] - iT[7]*a[1] - iT[2]*b[1] + iT[0]*c[2] + iT[9]*a[2] - iT[4]*c[3] - iT[5]*a[3] + iT[8]*c[4] + iT[1]*a[4] - t + add ) >> shift;
    dst[15 * line] = ( - iT[1]*c[0] + iT[8]*b[0] + iT[3]*c[1] - iT[6]*b[1] - iT[5]*c[2] + iT[4]*b[2] + iT[7]*c[3] - iT[2]*b[3] - iT[9]*c[4] + iT[0]*b[4] - t + add ) >> shift;

    src += 16;
    dst++;
  }

  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j < cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }

  if (iSkipLine2)
  {
    dst = pCoef + line * cutoff;
    memset(dst, 0, sizeof(TCoeff) * line * iSkipLine2);
  }
#else
  _fastForwardMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDCT8P16[TRANSFORM_FORWARD][0] );
#endif
}


void fastInverseDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff a[5], b[5], c[5], d[5], t;

  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P16[TRANSFORM_INVERSE][0];

  const int reducedLine = line - iSkipLine;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 5; k++)
    {
      a[k] = src[(15 - k ) * line] + src[( 4 - k) * line];
      b[k] = src[( 6 + k ) * line] + src[( 4 - k) * line];
      c[k] = src[(15 - k ) * line] - src[( 6 + k) * line];
      d[k] = src[(15 - k ) * line] + src[( 6 + k) * line] - src[(4 - k) * line];
    }

    t = iT[10] * src[5*line];

    dst[ 1] = Clip3(outputMinimum, outputMaximum, (int)( - iT[ 2]*d[0] - iT[ 5]*d[1] - iT[ 8]*d[2] - iT[11]*d[3] - iT[14]*d[4] + add) >> shift);
    dst[ 4] = Clip3(outputMinimum, outputMaximum, (int)(   iT[ 8]*d[0] + iT[14]*d[1] + iT[ 5]*d[2] - iT[ 2]*d[3] - iT[11]*d[4] + add) >> shift);
    dst[ 7] = Clip3(outputMinimum, outputMaximum, (int)( - iT[14]*d[0] - iT[ 2]*d[1] + iT[11]*d[2] + iT[ 5]*d[3] - iT[ 8]*d[4] + add) >> shift);
    dst[10] = Clip3(outputMinimum, outputMaximum, (int)(   iT[11]*d[0] - iT[ 8]*d[1] - iT[ 2]*d[2] + iT[14]*d[3] - iT[ 5]*d[4] + add) >> shift);
    dst[13] = Clip3(outputMinimum, outputMaximum, (int)( - iT[ 5]*d[0] + iT[11]*d[1] - iT[14]*d[2] + iT[ 8]*d[3] - iT[ 2]*d[4] + add) >> shift);

    dst[ 5] = Clip3(outputMinimum, outputMaximum, (int)( - iT[10] * (src[15 * line] + src[14 * line] - src[12 * line] - src[11 * line] + src[9 * line] + src[8 * line] - src[6 * line] - src[5 * line] + src[3 * line] + src[2 * line] - src[0 * line]) + add) >> shift);

    dst[ 0] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0]*a[0] + iT[9]*b[0] + iT[1]*a[1] + iT[8]*b[1] + iT[2]*a[2] + iT[7]*b[2] + iT[3]*a[3] + iT[6]*b[3] + iT[4]*a[4] + iT[5]*b[4] + t + add ) >> shift );
    dst[ 2] = Clip3(outputMinimum, outputMaximum, (int)(   iT[4]*c[0] - iT[5]*b[0] + iT[9]*c[1] - iT[0]*b[1] + iT[6]*c[2] + iT[3]*a[2] + iT[1]*c[3] + iT[8]*a[3] + iT[7]*a[4] + iT[2]*b[4] - t + add ) >> shift );
    dst[ 3] = Clip3(outputMinimum, outputMaximum, (int)( - iT[6]*a[0] - iT[3]*b[0] - iT[2]*c[1] - iT[7]*a[1] - iT[9]*c[2] - iT[0]*a[2] - iT[4]*c[3] + iT[5]*b[3] + iT[1]*a[4] + iT[8]*b[4] - t + add ) >> shift );
    dst[ 6] = Clip3(outputMinimum, outputMaximum, (int)(   iT[8]*a[0] + iT[1]*c[0] + iT[6]*c[1] - iT[3]*b[1] - iT[5]*a[2] - iT[4]*b[2] - iT[7]*c[3] - iT[2]*a[3] - iT[0]*c[4] + iT[9]*b[4] + t + add ) >> shift );
    dst[ 8] = Clip3(outputMinimum, outputMaximum, (int)(   iT[4]*c[0] + iT[5]*a[0] - iT[0]*c[1] + iT[9]*b[1] - iT[3]*c[2] - iT[6]*a[2] + iT[1]*c[3] - iT[8]*b[3] + iT[2]*c[4] + iT[7]*a[4] - t + add ) >> shift );
    dst[ 9] = Clip3(outputMinimum, outputMaximum, (int)( - iT[7]*c[0] - iT[2]*a[0] + iT[4]*a[1] + iT[5]*b[1] + iT[8]*c[2] - iT[1]*b[2] - iT[9]*a[3] - iT[0]*b[3] - iT[3]*c[4] + iT[6]*b[4] - t + add ) >> shift );
    dst[11] = Clip3(outputMinimum, outputMaximum, (int)( - iT[9]*a[0] - iT[0]*b[0] + iT[8]*c[1] + iT[1]*a[1] - iT[2]*c[2] + iT[7]*b[2] - iT[6]*a[3] - iT[3]*b[3] + iT[5]*c[4] + iT[4]*a[4] + t + add ) >> shift );
    dst[12] = Clip3(outputMinimum, outputMaximum, (int)(   iT[7]*c[0] - iT[2]*b[0] - iT[5]*c[1] - iT[4]*a[1] + iT[8]*a[2] + iT[1]*b[2] - iT[0]*a[3] - iT[9]*b[3] - iT[6]*c[4] + iT[3]*b[4] + t + add ) >> shift );
    dst[14] = Clip3(outputMinimum, outputMaximum, (int)(   iT[3]*a[0] + iT[6]*b[0] - iT[7]*a[1] - iT[2]*b[1] + iT[0]*c[2] + iT[9]*a[2] - iT[4]*c[3] - iT[5]*a[3] + iT[8]*c[4] + iT[1]*a[4] - t + add ) >> shift );
    dst[15] = Clip3(outputMinimum, outputMaximum, (int)( - iT[1]*c[0] + iT[8]*b[0] + iT[3]*c[1] - iT[6]*b[1] - iT[5]*c[2] + iT[4]*b[2] + iT[7]*c[3] - iT[2]*b[3] - iT[9]*c[4] + iT[0]*b[4] - t + add ) >> shift );

    src++;
    dst += 16;
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine * 16) * sizeof(TCoeff));
  }
#else
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDCT8P16[TRANSFORM_INVERSE][0] );
#endif
}


void fastForwardDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff a[10][6];
  TCoeff t[2];
  TCoeff b[6];
  TCoeff c[2];

  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;
  const TMatrixCoeff *iT = g_trCoreDST7P32[TRANSFORM_FORWARD][0];
  TCoeff *pCoef = dst;
  const int  reducedLine = line - iSkipLine;
  const int  cutoff = 32 - iSkipLine2;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 6; k++)
    {
      a[0][k] = src[31-k] - src[20+k];
      a[1][k] = src[31-k] + src[18-k];
      a[2][k] = src[31-k] + src[ 7+k];
      a[3][k] = src[31-k] - src[ 5-k];
      a[4][k] = src[25-k] + src[13+k];
      a[5][k] = src[25-k] + src[12-k];
      a[6][k] = src[25-k] - src[   k];
      a[7][k] = src[18-k] - src[ 7+k];
      a[8][k] = src[18-k] + src[ 5-k];
      a[9][k] = src[12-k] + src[   k];

      b[k] = src[31-k] + src[20+k] - src[18-k] - src[7+k] + src[5-k];
    }

    for (k = 0; k < 2; k++)
    {
      c[k] = src[31-k] + src[28+k] - src[26-k] - src[23+k] + src[21-k] + src[18+k] - src[16-k] - src[13+k] + src[11-k] + src[8+k] - src[6-k] - src[3+k] + src[1-k];
    }

    t[0] = iT[12] * src[19] + iT[25] * src[6];
    t[1] = iT[12] * src[6] - iT[25] * src[19];

    dst[ 0 * line] = (   iT[0] * a[3][0] + iT[11] * a[6][5] + iT[13] * a[8][0] + iT[24] * a[9][5] + iT[1] * a[3][1] + iT[10] * a[6][4] + iT[14] * a[8][1] + iT[23] * a[9][4] + iT[2] * a[3][2] + iT[9] * a[6][3] + iT[15] * a[8][2] + iT[22] * a[9][3] + iT[3] * a[3][3] + iT[8] * a[6][2] + iT[16] * a[8][3] + iT[21] * a[9][2] + iT[4] * a[3][4] + iT[7] * a[6][1] + iT[17] * a[8][4] + iT[20] * a[9][1] + iT[5] * a[3][5] + iT[6] * a[6][0] + iT[18] * a[8][5] + iT[19] * a[9][0] + t[0] + add) >> shift;
    dst[ 1 * line] = (   iT[0] * a[5][2] - iT[11] * a[0][3] - iT[13] * a[4][2] - iT[24] * a[6][2] - iT[1] * a[9][1] - iT[10] * a[8][4] - iT[14] * a[3][4] - iT[23] * a[6][1] - iT[2] * a[0][0] + iT[9] * a[5][5] - iT[15] * a[6][5] - iT[22] * a[4][5] + iT[3] * a[5][3] - iT[8] * a[0][2] - iT[16] * a[4][3] - iT[21] * a[6][3] - iT[4] * a[9][0] - iT[7] * a[8][5] - iT[17] * a[3][5] - iT[20] * a[6][0] - iT[5] * a[0][1] + iT[6] * a[5][4] - iT[18] * a[6][4] - iT[19] * a[4][4] + t[1] + add) >> shift;
    dst[ 3 * line] = (   iT[0] * a[9][4] + iT[11] * a[5][4] - iT[13] * a[2][1] + iT[24] * a[7][1] + iT[1] * a[0][3] + iT[10] * a[1][3] - iT[14] * a[3][3] - iT[23] * a[2][3] - iT[2] * a[8][5] - iT[9] * a[9][0] - iT[15] * a[6][0] - iT[22] * a[3][5] + iT[3] * a[1][4] + iT[8] * a[0][4] - iT[16] * a[2][4] - iT[21] * a[3][4] + iT[4] * a[5][3] + iT[7] * a[9][3] + iT[17] * a[7][2] - iT[20] * a[2][2] - iT[5] * a[8][0] - iT[6] * a[1][0] + iT[18] * a[4][5] + iT[19] * a[7][0] - t[1] + add) >> shift;
    dst[ 4 * line] = ( - iT[0] * a[3][2] - iT[11] * a[2][2] + iT[13] * a[1][2] + iT[24] * a[0][2] + iT[1] * a[6][0] + iT[10] * a[3][5] + iT[14] * a[9][0] + iT[23] * a[8][5] - iT[2] * a[2][3] - iT[9] * a[3][3] + iT[15] * a[0][3] + iT[22] * a[1][3] - iT[3] * a[7][0] + iT[8] * a[2][0] - iT[16] * a[9][5] - iT[21] * a[5][5] + iT[4] * a[4][4] + iT[7] * a[6][4] + iT[17] * a[0][1] - iT[20] * a[5][4] - iT[5] * a[7][4] - iT[6] * a[4][1] + iT[18] * a[8][4] + iT[19] * a[1][4] - t[0] + add) >> shift;
    dst[ 5 * line] = (   iT[0] * a[3][5] + iT[11] * a[6][0] + iT[13] * a[8][5] + iT[24] * a[9][0] - iT[1] * a[6][5] - iT[10] * a[3][0] - iT[14] * a[9][5] - iT[23] * a[8][0] + iT[2] * a[7][4] - iT[9] * a[2][4] + iT[15] * a[9][1] + iT[22] * a[5][1] + iT[3] * a[7][1] + iT[8] * a[4][4] - iT[16] * a[8][1] - iT[21] * a[1][1] - iT[4] * a[6][2] - iT[7] * a[4][2] + iT[17] * a[5][2] - iT[20] * a[0][3] + iT[5] * a[3][2] + iT[6] * a[2][2] - iT[18] * a[1][2] - iT[19] * a[0][2] - t[0] + add) >> shift;
    dst[ 8 * line] = (   iT[0] * a[9][3] + iT[11] * a[8][2] + iT[13] * a[3][2] + iT[24] * a[6][3] + iT[1] * a[1][5] + iT[10] * a[0][5] - iT[14] * a[2][5] - iT[23] * a[3][5] - iT[2] * a[1][3] - iT[9] * a[8][3] + iT[15] * a[7][3] + iT[22] * a[4][2] - iT[3] * a[9][5] - iT[8] * a[5][5] + iT[16] * a[2][0] - iT[21] * a[7][0] - iT[4] * a[1][1] - iT[7] * a[0][1] + iT[17] * a[2][1] + iT[20] * a[3][1] + iT[5] * a[5][1] + iT[6] * a[9][1] + iT[18] * a[7][4] - iT[19] * a[2][4] + t[1] + add) >> shift;
    dst[ 9 * line] = (   iT[0] * a[2][1] + iT[11] * a[3][1] - iT[13] * a[0][1] - iT[24] * a[1][1] - iT[1] * a[7][3] + iT[10] * a[2][3] - iT[14] * a[9][2] - iT[23] * a[5][2] - iT[2] * a[4][0] - iT[9] * a[7][5] + iT[15] * a[1][5] + iT[22] * a[8][5] - iT[3] * a[3][4] - iT[8] * a[2][4] + iT[16] * a[1][4] + iT[21] * a[0][4] - iT[4] * a[6][3] - iT[7] * a[3][2] - iT[17] * a[9][3] - iT[20] * a[8][2] - iT[5] * a[4][5] - iT[6] * a[6][5] - iT[18] * a[0][0] + iT[19] * a[5][5] + t[0] + add) >> shift;
    dst[10 * line] = ( - iT[0] * a[6][1] - iT[11] * a[4][1] + iT[13] * a[5][1] - iT[24] * a[0][4] + iT[1] * a[2][2] - iT[10] * a[7][2] - iT[14] * a[5][3] - iT[23] * a[9][3] + iT[2] * a[6][4] + iT[9] * a[4][4] - iT[15] * a[5][4] + iT[22] * a[0][1] - iT[3] * a[2][5] + iT[8] * a[7][5] + iT[16] * a[5][0] + iT[21] * a[9][0] - iT[4] * a[7][0] - iT[7] * a[4][5] + iT[17] * a[8][0] + iT[20] * a[1][0] + iT[5] * a[4][2] + iT[6] * a[7][3] - iT[18] * a[1][3] - iT[19] * a[8][3] + t[0] + add) >> shift;
    dst[11 * line] = ( - iT[0] * a[1][3] - iT[11] * a[0][3] + iT[13] * a[2][3] + iT[24] * a[3][3] - iT[1] * a[9][1] - iT[10] * a[5][1] + iT[14] * a[2][4] - iT[23] * a[7][4] - iT[2] * a[8][0] - iT[9] * a[9][5] - iT[15] * a[6][5] - iT[22] * a[3][0] + iT[3] * a[0][2] - iT[8] * a[5][3] + iT[16] * a[6][3] + iT[21] * a[4][3] + iT[4] * a[5][0] - iT[7] * a[0][5] - iT[17] * a[4][0] - iT[20] * a[6][0] + iT[5] * a[9][4] + iT[6] * a[5][4] - iT[18] * a[2][1] + iT[19] * a[7][1] + t[1] + add) >> shift;
    dst[13 * line] = (   iT[0] * a[0][0] + iT[11] * a[1][0] - iT[13] * a[3][0] - iT[24] * a[2][0] + iT[1] * a[5][4] - iT[10] * a[0][1] - iT[14] * a[4][4] - iT[23] * a[6][4] - iT[2] * a[9][3] - iT[9] * a[5][3] + iT[15] * a[2][2] - iT[22] * a[7][2] + iT[3] * a[8][3] + iT[8] * a[9][2] + iT[16] * a[6][2] + iT[21] * a[3][3] - iT[4] * a[1][4] - iT[7] * a[8][4] + iT[17] * a[7][4] + iT[20] * a[4][1] + iT[5] * a[0][5] + iT[6] * a[1][5] - iT[18] * a[3][5] - iT[19] * a[2][5] - t[1] + add) >> shift;
    dst[14 * line] = (   iT[0] * a[4][2] + iT[11] * a[7][3] - iT[13] * a[1][3] - iT[24] * a[8][3] + iT[1] * a[4][1] + iT[10] * a[6][1] + iT[14] * a[0][4] - iT[23] * a[5][1] - iT[2] * a[3][0] - iT[9] * a[2][0] + iT[15] * a[1][0] + iT[22] * a[0][0] - iT[3] * a[6][3] - iT[8] * a[4][3] + iT[16] * a[5][3] - iT[21] * a[0][2] - iT[4] * a[7][5] - iT[7] * a[4][0] + iT[17] * a[8][5] + iT[20] * a[1][5] + iT[5] * a[6][4] + iT[6] * a[3][1] + iT[18] * a[9][4] + iT[19] * a[8][1] - t[0] + add) >> shift;
    dst[15 * line] = (   iT[0] * a[7][4] + iT[11] * a[4][1] - iT[13] * a[8][4] - iT[24] * a[1][4] - iT[1] * a[2][2] - iT[10] * a[3][2] + iT[14] * a[0][2] + iT[23] * a[1][2] - iT[2] * a[2][1] + iT[9] * a[7][1] + iT[15] * a[5][4] + iT[22] * a[9][4] + iT[3] * a[7][5] - iT[8] * a[2][5] + iT[16] * a[9][0] + iT[21] * a[5][0] + iT[4] * a[2][0] + iT[7] * a[3][0] - iT[17] * a[0][0] - iT[20] * a[1][0] + iT[5] * a[2][3] - iT[6] * a[7][3] - iT[18] * a[5][2] - iT[19] * a[9][2] - t[0] + add) >> shift;
    dst[16 * line] = ( - iT[0] * a[0][1] + iT[11] * a[5][4] - iT[13] * a[6][4] - iT[24] * a[4][4] + iT[1] * a[0][3] - iT[10] * a[5][2] + iT[14] * a[6][2] + iT[23] * a[4][2] - iT[2] * a[0][5] + iT[9] * a[5][0] - iT[15] * a[6][0] - iT[22] * a[4][0] - iT[3] * a[0][4] - iT[8] * a[1][4] + iT[16] * a[3][4] + iT[21] * a[2][4] + iT[4] * a[0][2] + iT[7] * a[1][2] - iT[17] * a[3][2] - iT[20] * a[2][2] - iT[5] * a[0][0] - iT[6] * a[1][0] + iT[18] * a[3][0] + iT[19] * a[2][0] - t[1] + add) >> shift;
    dst[18 * line] = (   iT[0] * a[0][5] + iT[11] * a[1][5] - iT[13] * a[3][5] - iT[24] * a[2][5] - iT[1] * a[1][0] - iT[10] * a[0][0] + iT[14] * a[2][0] + iT[23] * a[3][0] - iT[2] * a[5][1] + iT[9] * a[0][4] + iT[15] * a[4][1] + iT[22] * a[6][1] - iT[3] * a[8][1] - iT[8] * a[1][1] + iT[16] * a[4][4] + iT[21] * a[7][1] - iT[4] * a[9][2] - iT[7] * a[5][2] + iT[17] * a[2][3] - iT[20] * a[7][3] - iT[5] * a[9][3] - iT[6] * a[8][2] - iT[18] * a[3][2] - iT[19] * a[6][3] + t[1] + add) >> shift;
    dst[20 * line] = ( - iT[0] * a[4][0] - iT[11] * a[6][0] - iT[13] * a[0][5] + iT[24] * a[5][0] + iT[1] * a[6][5] + iT[10] * a[4][5] - iT[14] * a[5][5] + iT[23] * a[0][0] - iT[2] * a[6][1] - iT[9] * a[3][4] - iT[15] * a[9][1] - iT[22] * a[8][4] + iT[3] * a[4][4] + iT[8] * a[7][1] - iT[16] * a[1][1] - iT[21] * a[8][1] - iT[4] * a[3][3] - iT[7] * a[2][3] + iT[17] * a[1][3] + iT[20] * a[0][3] + iT[5] * a[7][2] - iT[6] * a[2][2] + iT[18] * a[9][3] + iT[19] * a[5][3] + t[0] + add) >> shift;
    dst[21 * line] = (   iT[0] * a[1][2] + iT[11] * a[8][2] - iT[13] * a[7][2] - iT[24] * a[4][3] + iT[1] * a[1][5] + iT[10] * a[8][5] - iT[14] * a[7][5] - iT[23] * a[4][0] + iT[2] * a[5][2] + iT[9] * a[9][2] + iT[15] * a[7][3] - iT[22] * a[2][3] + iT[3] * a[5][5] + iT[8] * a[9][5] + iT[16] * a[7][0] - iT[21] * a[2][0] + iT[4] * a[8][1] + iT[7] * a[9][4] + iT[17] * a[6][4] + iT[20] * a[3][1] + iT[5] * a[8][4] + iT[6] * a[9][1] + iT[18] * a[6][1] + iT[19] * a[3][4] + t[1] + add) >> shift;
    dst[23 * line] = (   iT[0] * a[8][4] + iT[11] * a[9][1] + iT[13] * a[6][1] + iT[24] * a[3][4] - iT[1] * a[8][2] - iT[10] * a[1][2] + iT[14] * a[4][3] + iT[23] * a[7][2] - iT[2] * a[0][1] - iT[9] * a[1][1] + iT[15] * a[3][1] + iT[22] * a[2][1] + iT[3] * a[5][0] + iT[8] * a[9][0] + iT[16] * a[7][5] - iT[21] * a[2][5] - iT[4] * a[9][5] - iT[7] * a[8][0] - iT[17] * a[3][0] - iT[20] * a[6][5] + iT[5] * a[5][2] - iT[6] * a[0][3] - iT[18] * a[4][2] - iT[19] * a[6][2] - t[1] + add) >> shift;
    dst[24 * line] = ( - iT[0] * a[2][3] + iT[11] * a[7][3] + iT[13] * a[5][2] + iT[24] * a[9][2] + iT[1] * a[4][1] + iT[10] * a[7][4] - iT[14] * a[1][4] - iT[23] * a[8][4] - iT[2] * a[4][5] - iT[9] * a[7][0] + iT[15] * a[1][0] + iT[22] * a[8][0] + iT[3] * a[4][3] + iT[8] * a[6][3] + iT[16] * a[0][2] - iT[21] * a[5][3] - iT[4] * a[2][5] - iT[7] * a[3][5] + iT[17] * a[0][5] + iT[20] * a[1][5] + iT[5] * a[2][1] + iT[6] * a[3][1] - iT[18] * a[0][1] - iT[19] * a[1][1] - t[0] + add) >> shift;
    dst[25 * line] = ( - iT[0] * a[4][5] - iT[11] * a[6][5] - iT[13] * a[0][0] + iT[24] * a[5][5] - iT[1] * a[3][1] - iT[10] * a[2][1] + iT[14] * a[1][1] + iT[23] * a[0][1] + iT[2] * a[7][2] + iT[9] * a[4][3] - iT[15] * a[8][2] - iT[22] * a[1][2] + iT[3] * a[6][2] + iT[8] * a[3][3] + iT[16] * a[9][2] + iT[21] * a[8][3] + iT[4] * a[2][4] - iT[7] * a[7][4] - iT[17] * a[5][1] - iT[20] * a[9][1] - iT[5] * a[4][0] - iT[6] * a[6][0] - iT[18] * a[0][5] + iT[19] * a[5][0] - t[0] + add) >> shift;
    dst[26 * line] = (   iT[0] * a[8][0] + iT[11] * a[1][0] - iT[13] * a[4][5] - iT[24] * a[7][0] + iT[1] * a[5][4] + iT[10] * a[9][4] + iT[14] * a[7][1] - iT[23] * a[2][1] - iT[2] * a[1][2] - iT[9] * a[0][2] + iT[15] * a[2][2] + iT[22] * a[3][2] - iT[3] * a[9][2] - iT[8] * a[8][3] - iT[16] * a[3][3] - iT[21] * a[6][2] + iT[4] * a[0][4] - iT[7] * a[5][1] + iT[17] * a[6][1] + iT[20] * a[4][1] + iT[5] * a[8][5] + iT[6] * a[1][5] - iT[18] * a[4][0] - iT[19] * a[7][5] - t[1] + add) >> shift;
    dst[28 * line] = ( - iT[0] * a[5][1] - iT[11] * a[9][1] - iT[13] * a[7][4] + iT[24] * a[2][4] + iT[1] * a[8][2] + iT[10] * a[9][3] + iT[14] * a[6][3] + iT[23] * a[3][2] - iT[2] * a[9][4] - iT[9] * a[8][1] - iT[15] * a[3][1] - iT[22] * a[6][4] + iT[3] * a[9][0] + iT[8] * a[5][0] - iT[16] * a[2][5] + iT[21] * a[7][5] - iT[4] * a[5][5] + iT[7] * a[0][0] + iT[17] * a[4][5] + iT[20] * a[6][5] + iT[5] * a[1][3] + iT[6] * a[0][3] - iT[18] * a[2][3] - iT[19] * a[3][3] + t[1] + add) >> shift;
    dst[29 * line] = (   iT[0] * a[6][4] + iT[11] * a[3][1] + iT[13] * a[9][4] + iT[24] * a[8][1] - iT[1] * a[7][3] - iT[10] * a[4][2] + iT[14] * a[8][3] + iT[23] * a[1][3] - iT[2] * a[3][5] - iT[9] * a[2][5] + iT[15] * a[1][5] + iT[22] * a[0][5] + iT[3] * a[2][4] + iT[8] * a[3][4] - iT[16] * a[0][4] - iT[21] * a[1][4] + iT[4] * a[4][3] + iT[7] * a[7][2] - iT[17] * a[1][2] - iT[20] * a[8][2] - iT[5] * a[3][0] - iT[6] * a[6][5] - iT[18] * a[8][0] - iT[19] * a[9][5] + t[0] + add) >> shift;
    dst[30 * line] = ( - iT[0] * a[7][2] + iT[11] * a[2][2] - iT[13] * a[9][3] - iT[24] * a[5][3] - iT[1] * a[6][0] - iT[10] * a[4][0] + iT[14] * a[5][0] - iT[23] * a[0][5] - iT[2] * a[4][2] - iT[9] * a[6][2] - iT[15] * a[0][3] + iT[22] * a[5][2] + iT[3] * a[2][0] - iT[8] * a[7][0] - iT[16] * a[5][5] - iT[21] * a[9][5] + iT[4] * a[7][1] - iT[7] * a[2][1] + iT[17] * a[9][4] + iT[20] * a[5][4] + iT[5] * a[6][1] + iT[6] * a[4][1] - iT[18] * a[5][1] + iT[19] * a[0][4] + t[0] + add) >> shift;
    dst[31 * line] = (   iT[0] * a[8][5] + iT[11] * a[1][5] - iT[13] * a[4][0] - iT[24] * a[7][5] - iT[1] * a[1][0] - iT[10] * a[8][0] + iT[14] * a[7][0] + iT[23] * a[4][5] - iT[2] * a[8][4] - iT[9] * a[1][4] + iT[15] * a[4][1] + iT[22] * a[7][4] + iT[3] * a[1][1] + iT[8] * a[8][1] - iT[16] * a[7][1] - iT[21] * a[4][4] + iT[4] * a[8][3] + iT[7] * a[1][3] - iT[17] * a[4][2] - iT[20] * a[7][3] - iT[5] * a[1][2] - iT[6] * a[8][2] + iT[18] * a[7][2] + iT[19] * a[4][3] + t[1] + add) >> shift;

    dst[ 2 * line] = (   iT[ 4] * b[0] + iT[ 9] * b[1] + iT[14] * b[2] + iT[19] * b[3] + iT[24] * b[4] + iT[29] * b[5] + add) >> shift;
    dst[ 7 * line] = ( - iT[14] * b[0] - iT[29] * b[1] - iT[19] * b[2] - iT[ 4] * b[3] + iT[ 9] * b[4] + iT[24] * b[5] + add) >> shift;
    dst[12 * line] = (   iT[24] * b[0] + iT[14] * b[1] - iT[ 9] * b[2] - iT[29] * b[3] - iT[ 4] * b[4] + iT[19] * b[5] + add) >> shift;
    dst[17 * line] = ( - iT[29] * b[0] + iT[ 4] * b[1] + iT[24] * b[2] - iT[ 9] * b[3] - iT[19] * b[4] + iT[14] * b[5] + add) >> shift;
    dst[22 * line] = (   iT[19] * b[0] - iT[24] * b[1] + iT[ 4] * b[2] + iT[14] * b[3] - iT[29] * b[4] + iT[ 9] * b[5] + add) >> shift;
    dst[27 * line] = ( - iT[ 9] * b[0] + iT[19] * b[1] - iT[29] * b[2] + iT[24] * b[3] - iT[14] * b[4] + iT[ 4] * b[5] + add) >> shift;

    dst[ 6 * line] = (   iT[12] * c[0] + iT[25] * c[1] + add) >> shift;
    dst[19 * line] = ( - iT[25] * c[0] + iT[12] * c[1] + add) >> shift;

    src += 32;
    dst++;
  }

  if (iSkipLine)
  {
    dst = pCoef + reducedLine;
    for (j = 0; j < cutoff; j++)
    {
      memset(dst, 0, sizeof(TCoeff)*iSkipLine);
      dst += line;
    }
  }

  if (iSkipLine2)
  {
    dst = pCoef + line * cutoff;
    memset(dst, 0, sizeof(TCoeff) * line * iSkipLine2);
  }
#else
  _fastForwardMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, g_trCoreDCT8P32[TRANSFORM_FORWARD][0] );
#endif
}


void fastInverseDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if !JVET_M0497_MATRIX_MULT
  int j, k;
  TCoeff a[10][6];
  TCoeff t[2];
  TCoeff b[6];
  TCoeff c[2];
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P32[TRANSFORM_INVERSE][0];

  const int  reducedLine = line - iSkipLine;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 6; k++)
    {
      a[0][k] = src[(31 - k)*line] - src[(20 + k)*line];
      a[1][k] = src[(31 - k)*line] + src[(18 - k)*line];
      a[2][k] = src[(31 - k)*line] + src[( 7 + k)*line];
      a[3][k] = src[(31 - k)*line] - src[( 5 - k)*line];
      a[4][k] = src[(25 - k)*line] + src[(13 + k)*line];
      a[5][k] = src[(25 - k)*line] + src[(12 - k)*line];
      a[6][k] = src[(25 - k)*line] - src[      k *line];
      a[7][k] = src[(18 - k)*line] - src[( 7 + k)*line];
      a[8][k] = src[(18 - k)*line] + src[( 5 - k)*line];
      a[9][k] = src[(12 - k)*line] + src[      k *line];

      b[k] = src[(31 - k)*line] + src[(20 + k)*line] - src[(18 - k)*line] - src[(7 + k)*line] + src[(5 - k)*line];
    }

    for (k = 0; k < 2; k++)
    {
      c[k] = src[(31 - k)*line] + src[(28 + k)*line] - src[(26 - k)*line] - src[(23 + k)*line] + src[(21 - k)*line] + src[(18 + k)*line] - src[(16 - k)*line] - src[(13 + k)*line] + src[(11 - k)*line] + src[(8 + k)*line] - src[(6 - k)*line] - src[(3 + k)*line] + src[(1 - k)*line];
    }

    t[0] = iT[12] * src[19 * line] + iT[25] * src[ 6 * line];
    t[1] = iT[12] * src[ 6 * line] - iT[25] * src[19 * line];

    dst[ 0] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[3][0] + iT[11] * a[6][5] + iT[13] * a[8][0] + iT[24] * a[9][5] + iT[1] * a[3][1] + iT[10] * a[6][4] + iT[14] * a[8][1] + iT[23] * a[9][4] + iT[2] * a[3][2] + iT[9] * a[6][3] + iT[15] * a[8][2] + iT[22] * a[9][3] + iT[3] * a[3][3] + iT[8] * a[6][2] + iT[16] * a[8][3] + iT[21] * a[9][2] + iT[4] * a[3][4] + iT[7] * a[6][1] + iT[17] * a[8][4] + iT[20] * a[9][1] + iT[5] * a[3][5] + iT[6] * a[6][0] + iT[18] * a[8][5] + iT[19] * a[9][0] + t[0] + add) >> shift);
    dst[ 1] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[5][2] - iT[11] * a[0][3] - iT[13] * a[4][2] - iT[24] * a[6][2] - iT[1] * a[9][1] - iT[10] * a[8][4] - iT[14] * a[3][4] - iT[23] * a[6][1] - iT[2] * a[0][0] + iT[9] * a[5][5] - iT[15] * a[6][5] - iT[22] * a[4][5] + iT[3] * a[5][3] - iT[8] * a[0][2] - iT[16] * a[4][3] - iT[21] * a[6][3] - iT[4] * a[9][0] - iT[7] * a[8][5] - iT[17] * a[3][5] - iT[20] * a[6][0] - iT[5] * a[0][1] + iT[6] * a[5][4] - iT[18] * a[6][4] - iT[19] * a[4][4] + t[1] + add) >> shift);
    dst[ 3] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[9][4] + iT[11] * a[5][4] - iT[13] * a[2][1] + iT[24] * a[7][1] + iT[1] * a[0][3] + iT[10] * a[1][3] - iT[14] * a[3][3] - iT[23] * a[2][3] - iT[2] * a[8][5] - iT[9] * a[9][0] - iT[15] * a[6][0] - iT[22] * a[3][5] + iT[3] * a[1][4] + iT[8] * a[0][4] - iT[16] * a[2][4] - iT[21] * a[3][4] + iT[4] * a[5][3] + iT[7] * a[9][3] + iT[17] * a[7][2] - iT[20] * a[2][2] - iT[5] * a[8][0] - iT[6] * a[1][0] + iT[18] * a[4][5] + iT[19] * a[7][0] - t[1] + add) >> shift);
    dst[ 4] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[3][2] - iT[11] * a[2][2] + iT[13] * a[1][2] + iT[24] * a[0][2] + iT[1] * a[6][0] + iT[10] * a[3][5] + iT[14] * a[9][0] + iT[23] * a[8][5] - iT[2] * a[2][3] - iT[9] * a[3][3] + iT[15] * a[0][3] + iT[22] * a[1][3] - iT[3] * a[7][0] + iT[8] * a[2][0] - iT[16] * a[9][5] - iT[21] * a[5][5] + iT[4] * a[4][4] + iT[7] * a[6][4] + iT[17] * a[0][1] - iT[20] * a[5][4] - iT[5] * a[7][4] - iT[6] * a[4][1] + iT[18] * a[8][4] + iT[19] * a[1][4] - t[0] + add) >> shift);
    dst[ 5] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[3][5] + iT[11] * a[6][0] + iT[13] * a[8][5] + iT[24] * a[9][0] - iT[1] * a[6][5] - iT[10] * a[3][0] - iT[14] * a[9][5] - iT[23] * a[8][0] + iT[2] * a[7][4] - iT[9] * a[2][4] + iT[15] * a[9][1] + iT[22] * a[5][1] + iT[3] * a[7][1] + iT[8] * a[4][4] - iT[16] * a[8][1] - iT[21] * a[1][1] - iT[4] * a[6][2] - iT[7] * a[4][2] + iT[17] * a[5][2] - iT[20] * a[0][3] + iT[5] * a[3][2] + iT[6] * a[2][2] - iT[18] * a[1][2] - iT[19] * a[0][2] - t[0] + add) >> shift);
    dst[ 8] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[9][3] + iT[11] * a[8][2] + iT[13] * a[3][2] + iT[24] * a[6][3] + iT[1] * a[1][5] + iT[10] * a[0][5] - iT[14] * a[2][5] - iT[23] * a[3][5] - iT[2] * a[1][3] - iT[9] * a[8][3] + iT[15] * a[7][3] + iT[22] * a[4][2] - iT[3] * a[9][5] - iT[8] * a[5][5] + iT[16] * a[2][0] - iT[21] * a[7][0] - iT[4] * a[1][1] - iT[7] * a[0][1] + iT[17] * a[2][1] + iT[20] * a[3][1] + iT[5] * a[5][1] + iT[6] * a[9][1] + iT[18] * a[7][4] - iT[19] * a[2][4] + t[1] + add) >> shift);
    dst[ 9] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[2][1] + iT[11] * a[3][1] - iT[13] * a[0][1] - iT[24] * a[1][1] - iT[1] * a[7][3] + iT[10] * a[2][3] - iT[14] * a[9][2] - iT[23] * a[5][2] - iT[2] * a[4][0] - iT[9] * a[7][5] + iT[15] * a[1][5] + iT[22] * a[8][5] - iT[3] * a[3][4] - iT[8] * a[2][4] + iT[16] * a[1][4] + iT[21] * a[0][4] - iT[4] * a[6][3] - iT[7] * a[3][2] - iT[17] * a[9][3] - iT[20] * a[8][2] - iT[5] * a[4][5] - iT[6] * a[6][5] - iT[18] * a[0][0] + iT[19] * a[5][5] + t[0] + add) >> shift);
    dst[10] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[6][1] - iT[11] * a[4][1] + iT[13] * a[5][1] - iT[24] * a[0][4] + iT[1] * a[2][2] - iT[10] * a[7][2] - iT[14] * a[5][3] - iT[23] * a[9][3] + iT[2] * a[6][4] + iT[9] * a[4][4] - iT[15] * a[5][4] + iT[22] * a[0][1] - iT[3] * a[2][5] + iT[8] * a[7][5] + iT[16] * a[5][0] + iT[21] * a[9][0] - iT[4] * a[7][0] - iT[7] * a[4][5] + iT[17] * a[8][0] + iT[20] * a[1][0] + iT[5] * a[4][2] + iT[6] * a[7][3] - iT[18] * a[1][3] - iT[19] * a[8][3] + t[0] + add) >> shift);
    dst[11] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[1][3] - iT[11] * a[0][3] + iT[13] * a[2][3] + iT[24] * a[3][3] - iT[1] * a[9][1] - iT[10] * a[5][1] + iT[14] * a[2][4] - iT[23] * a[7][4] - iT[2] * a[8][0] - iT[9] * a[9][5] - iT[15] * a[6][5] - iT[22] * a[3][0] + iT[3] * a[0][2] - iT[8] * a[5][3] + iT[16] * a[6][3] + iT[21] * a[4][3] + iT[4] * a[5][0] - iT[7] * a[0][5] - iT[17] * a[4][0] - iT[20] * a[6][0] + iT[5] * a[9][4] + iT[6] * a[5][4] - iT[18] * a[2][1] + iT[19] * a[7][1] + t[1] + add) >> shift);
    dst[13] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[0][0] + iT[11] * a[1][0] - iT[13] * a[3][0] - iT[24] * a[2][0] + iT[1] * a[5][4] - iT[10] * a[0][1] - iT[14] * a[4][4] - iT[23] * a[6][4] - iT[2] * a[9][3] - iT[9] * a[5][3] + iT[15] * a[2][2] - iT[22] * a[7][2] + iT[3] * a[8][3] + iT[8] * a[9][2] + iT[16] * a[6][2] + iT[21] * a[3][3] - iT[4] * a[1][4] - iT[7] * a[8][4] + iT[17] * a[7][4] + iT[20] * a[4][1] + iT[5] * a[0][5] + iT[6] * a[1][5] - iT[18] * a[3][5] - iT[19] * a[2][5] - t[1] + add) >> shift);
    dst[14] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[4][2] + iT[11] * a[7][3] - iT[13] * a[1][3] - iT[24] * a[8][3] + iT[1] * a[4][1] + iT[10] * a[6][1] + iT[14] * a[0][4] - iT[23] * a[5][1] - iT[2] * a[3][0] - iT[9] * a[2][0] + iT[15] * a[1][0] + iT[22] * a[0][0] - iT[3] * a[6][3] - iT[8] * a[4][3] + iT[16] * a[5][3] - iT[21] * a[0][2] - iT[4] * a[7][5] - iT[7] * a[4][0] + iT[17] * a[8][5] + iT[20] * a[1][5] + iT[5] * a[6][4] + iT[6] * a[3][1] + iT[18] * a[9][4] + iT[19] * a[8][1] - t[0] + add) >> shift);
    dst[15] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[7][4] + iT[11] * a[4][1] - iT[13] * a[8][4] - iT[24] * a[1][4] - iT[1] * a[2][2] - iT[10] * a[3][2] + iT[14] * a[0][2] + iT[23] * a[1][2] - iT[2] * a[2][1] + iT[9] * a[7][1] + iT[15] * a[5][4] + iT[22] * a[9][4] + iT[3] * a[7][5] - iT[8] * a[2][5] + iT[16] * a[9][0] + iT[21] * a[5][0] + iT[4] * a[2][0] + iT[7] * a[3][0] - iT[17] * a[0][0] - iT[20] * a[1][0] + iT[5] * a[2][3] - iT[6] * a[7][3] - iT[18] * a[5][2] - iT[19] * a[9][2] - t[0] + add) >> shift);
    dst[16] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[0][1] + iT[11] * a[5][4] - iT[13] * a[6][4] - iT[24] * a[4][4] + iT[1] * a[0][3] - iT[10] * a[5][2] + iT[14] * a[6][2] + iT[23] * a[4][2] - iT[2] * a[0][5] + iT[9] * a[5][0] - iT[15] * a[6][0] - iT[22] * a[4][0] - iT[3] * a[0][4] - iT[8] * a[1][4] + iT[16] * a[3][4] + iT[21] * a[2][4] + iT[4] * a[0][2] + iT[7] * a[1][2] - iT[17] * a[3][2] - iT[20] * a[2][2] - iT[5] * a[0][0] - iT[6] * a[1][0] + iT[18] * a[3][0] + iT[19] * a[2][0] - t[1] + add) >> shift);
    dst[18] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[0][5] + iT[11] * a[1][5] - iT[13] * a[3][5] - iT[24] * a[2][5] - iT[1] * a[1][0] - iT[10] * a[0][0] + iT[14] * a[2][0] + iT[23] * a[3][0] - iT[2] * a[5][1] + iT[9] * a[0][4] + iT[15] * a[4][1] + iT[22] * a[6][1] - iT[3] * a[8][1] - iT[8] * a[1][1] + iT[16] * a[4][4] + iT[21] * a[7][1] - iT[4] * a[9][2] - iT[7] * a[5][2] + iT[17] * a[2][3] - iT[20] * a[7][3] - iT[5] * a[9][3] - iT[6] * a[8][2] - iT[18] * a[3][2] - iT[19] * a[6][3] + t[1] + add) >> shift);
    dst[20] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[4][0] - iT[11] * a[6][0] - iT[13] * a[0][5] + iT[24] * a[5][0] + iT[1] * a[6][5] + iT[10] * a[4][5] - iT[14] * a[5][5] + iT[23] * a[0][0] - iT[2] * a[6][1] - iT[9] * a[3][4] - iT[15] * a[9][1] - iT[22] * a[8][4] + iT[3] * a[4][4] + iT[8] * a[7][1] - iT[16] * a[1][1] - iT[21] * a[8][1] - iT[4] * a[3][3] - iT[7] * a[2][3] + iT[17] * a[1][3] + iT[20] * a[0][3] + iT[5] * a[7][2] - iT[6] * a[2][2] + iT[18] * a[9][3] + iT[19] * a[5][3] + t[0] + add) >> shift);
    dst[21] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[1][2] + iT[11] * a[8][2] - iT[13] * a[7][2] - iT[24] * a[4][3] + iT[1] * a[1][5] + iT[10] * a[8][5] - iT[14] * a[7][5] - iT[23] * a[4][0] + iT[2] * a[5][2] + iT[9] * a[9][2] + iT[15] * a[7][3] - iT[22] * a[2][3] + iT[3] * a[5][5] + iT[8] * a[9][5] + iT[16] * a[7][0] - iT[21] * a[2][0] + iT[4] * a[8][1] + iT[7] * a[9][4] + iT[17] * a[6][4] + iT[20] * a[3][1] + iT[5] * a[8][4] + iT[6] * a[9][1] + iT[18] * a[6][1] + iT[19] * a[3][4] + t[1] + add) >> shift);
    dst[23] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[8][4] + iT[11] * a[9][1] + iT[13] * a[6][1] + iT[24] * a[3][4] - iT[1] * a[8][2] - iT[10] * a[1][2] + iT[14] * a[4][3] + iT[23] * a[7][2] - iT[2] * a[0][1] - iT[9] * a[1][1] + iT[15] * a[3][1] + iT[22] * a[2][1] + iT[3] * a[5][0] + iT[8] * a[9][0] + iT[16] * a[7][5] - iT[21] * a[2][5] - iT[4] * a[9][5] - iT[7] * a[8][0] - iT[17] * a[3][0] - iT[20] * a[6][5] + iT[5] * a[5][2] - iT[6] * a[0][3] - iT[18] * a[4][2] - iT[19] * a[6][2] - t[1] + add) >> shift);
    dst[24] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[2][3] + iT[11] * a[7][3] + iT[13] * a[5][2] + iT[24] * a[9][2] + iT[1] * a[4][1] + iT[10] * a[7][4] - iT[14] * a[1][4] - iT[23] * a[8][4] - iT[2] * a[4][5] - iT[9] * a[7][0] + iT[15] * a[1][0] + iT[22] * a[8][0] + iT[3] * a[4][3] + iT[8] * a[6][3] + iT[16] * a[0][2] - iT[21] * a[5][3] - iT[4] * a[2][5] - iT[7] * a[3][5] + iT[17] * a[0][5] + iT[20] * a[1][5] + iT[5] * a[2][1] + iT[6] * a[3][1] - iT[18] * a[0][1] - iT[19] * a[1][1] - t[0] + add) >> shift);
    dst[25] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[4][5] - iT[11] * a[6][5] - iT[13] * a[0][0] + iT[24] * a[5][5] - iT[1] * a[3][1] - iT[10] * a[2][1] + iT[14] * a[1][1] + iT[23] * a[0][1] + iT[2] * a[7][2] + iT[9] * a[4][3] - iT[15] * a[8][2] - iT[22] * a[1][2] + iT[3] * a[6][2] + iT[8] * a[3][3] + iT[16] * a[9][2] + iT[21] * a[8][3] + iT[4] * a[2][4] - iT[7] * a[7][4] - iT[17] * a[5][1] - iT[20] * a[9][1] - iT[5] * a[4][0] - iT[6] * a[6][0] - iT[18] * a[0][5] + iT[19] * a[5][0] - t[0] + add) >> shift);
    dst[26] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[8][0] + iT[11] * a[1][0] - iT[13] * a[4][5] - iT[24] * a[7][0] + iT[1] * a[5][4] + iT[10] * a[9][4] + iT[14] * a[7][1] - iT[23] * a[2][1] - iT[2] * a[1][2] - iT[9] * a[0][2] + iT[15] * a[2][2] + iT[22] * a[3][2] - iT[3] * a[9][2] - iT[8] * a[8][3] - iT[16] * a[3][3] - iT[21] * a[6][2] + iT[4] * a[0][4] - iT[7] * a[5][1] + iT[17] * a[6][1] + iT[20] * a[4][1] + iT[5] * a[8][5] + iT[6] * a[1][5] - iT[18] * a[4][0] - iT[19] * a[7][5] - t[1] + add) >> shift);
    dst[28] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[5][1] - iT[11] * a[9][1] - iT[13] * a[7][4] + iT[24] * a[2][4] + iT[1] * a[8][2] + iT[10] * a[9][3] + iT[14] * a[6][3] + iT[23] * a[3][2] - iT[2] * a[9][4] - iT[9] * a[8][1] - iT[15] * a[3][1] - iT[22] * a[6][4] + iT[3] * a[9][0] + iT[8] * a[5][0] - iT[16] * a[2][5] + iT[21] * a[7][5] - iT[4] * a[5][5] + iT[7] * a[0][0] + iT[17] * a[4][5] + iT[20] * a[6][5] + iT[5] * a[1][3] + iT[6] * a[0][3] - iT[18] * a[2][3] - iT[19] * a[3][3] + t[1] + add) >> shift);
    dst[29] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[6][4] + iT[11] * a[3][1] + iT[13] * a[9][4] + iT[24] * a[8][1] - iT[1] * a[7][3] - iT[10] * a[4][2] + iT[14] * a[8][3] + iT[23] * a[1][3] - iT[2] * a[3][5] - iT[9] * a[2][5] + iT[15] * a[1][5] + iT[22] * a[0][5] + iT[3] * a[2][4] + iT[8] * a[3][4] - iT[16] * a[0][4] - iT[21] * a[1][4] + iT[4] * a[4][3] + iT[7] * a[7][2] - iT[17] * a[1][2] - iT[20] * a[8][2] - iT[5] * a[3][0] - iT[6] * a[6][5] - iT[18] * a[8][0] - iT[19] * a[9][5] + t[0] + add) >> shift);
    dst[30] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[7][2] + iT[11] * a[2][2] - iT[13] * a[9][3] - iT[24] * a[5][3] - iT[1] * a[6][0] - iT[10] * a[4][0] + iT[14] * a[5][0] - iT[23] * a[0][5] - iT[2] * a[4][2] - iT[9] * a[6][2] - iT[15] * a[0][3] + iT[22] * a[5][2] + iT[3] * a[2][0] - iT[8] * a[7][0] - iT[16] * a[5][5] - iT[21] * a[9][5] + iT[4] * a[7][1] - iT[7] * a[2][1] + iT[17] * a[9][4] + iT[20] * a[5][4] + iT[5] * a[6][1] + iT[6] * a[4][1] - iT[18] * a[5][1] + iT[19] * a[0][4] + t[0] + add) >> shift);
    dst[31] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[8][5] + iT[11] * a[1][5] - iT[13] * a[4][0] - iT[24] * a[7][5] - iT[1] * a[1][0] - iT[10] * a[8][0] + iT[14] * a[7][0] + iT[23] * a[4][5] - iT[2] * a[8][4] - iT[9] * a[1][4] + iT[15] * a[4][1] + iT[22] * a[7][4] + iT[3] * a[1][1] + iT[8] * a[8][1] - iT[16] * a[7][1] - iT[21] * a[4][4] + iT[4] * a[8][3] + iT[7] * a[1][3] - iT[17] * a[4][2] - iT[20] * a[7][3] - iT[5] * a[1][2] - iT[6] * a[8][2] + iT[18] * a[7][2] + iT[19] * a[4][3] + t[1] + add) >> shift);

    dst[ 2] = Clip3(outputMinimum, outputMaximum, (int)(   iT[ 4] * b[0] + iT[ 9] * b[1] + iT[14] * b[2] + iT[19] * b[3] + iT[24] * b[4] + iT[29] * b[5] + add) >> shift);
    dst[ 7] = Clip3(outputMinimum, outputMaximum, (int)( - iT[14] * b[0] - iT[29] * b[1] - iT[19] * b[2] - iT[ 4] * b[3] + iT[ 9] * b[4] + iT[24] * b[5] + add) >> shift);
    dst[12] = Clip3(outputMinimum, outputMaximum, (int)(   iT[24] * b[0] + iT[14] * b[1] - iT[ 9] * b[2] - iT[29] * b[3] - iT[ 4] * b[4] + iT[19] * b[5] + add) >> shift);
    dst[17] = Clip3(outputMinimum, outputMaximum, (int)( - iT[29] * b[0] + iT[ 4] * b[1] + iT[24] * b[2] - iT[ 9] * b[3] - iT[19] * b[4] + iT[14] * b[5] + add) >> shift);
    dst[22] = Clip3(outputMinimum, outputMaximum, (int)(   iT[19] * b[0] - iT[24] * b[1] + iT[ 4] * b[2] + iT[14] * b[3] - iT[29] * b[4] + iT[ 9] * b[5] + add) >> shift);
    dst[27] = Clip3(outputMinimum, outputMaximum, (int)( - iT[ 9] * b[0] + iT[19] * b[1] - iT[29] * b[2] + iT[24] * b[3] - iT[14] * b[4] + iT[ 4] * b[5] + add) >> shift);

    dst[ 6] = Clip3(outputMinimum, outputMaximum, (int)(   iT[12] * c[0] + iT[25] * c[1] + add) >> shift);
    dst[19] = Clip3(outputMinimum, outputMaximum, (int)( - iT[25] * c[0] + iT[12] * c[1] + add) >> shift);

    src++;
    dst += 32;
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine * 32) * sizeof(TCoeff));
  }
#else
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDCT8P32[TRANSFORM_INVERSE][0] );
#endif
}

#if ENABLE_SIMD_TRAFO

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP 1

}   // namespace vvenc

#include "Unit.h"

namespace vvenc {
  
void cpyCoeffCore( const Pel* src, ptrdiff_t stride, TCoeff* dst, unsigned width, unsigned height )
{
#define CPYCOEFF_OP( ADDR ) dst[ADDR] = src[ADDR];
#define CPYCOEFF_INC src += stride; dst += width;

  SIZE_AWARE_PER_EL_OP( CPYCOEFF_OP, CPYCOEFF_INC );

#undef CPYCOEFF_INC
#undef CPYCOEFF_OP
}


void cpyResiCore( const TCoeff* src, Pel* dst, ptrdiff_t stride, unsigned width, unsigned height )
{
#define CPYRESI_OP( ADDR ) dst[ADDR] = Pel( src[ADDR] );
#define CPYRESI_INC dst += stride; src += width;

  SIZE_AWARE_PER_EL_OP( CPYRESI_OP, CPYRESI_INC );

#undef CPYRESI_INC
#undef CPYRESI_OP
}


void clipCore( TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift )
{
#define CLIP_OP( ADDR ) dst[ADDR] = Clip3( outputMin, outputMax, ( dst[ADDR] + round ) >> shift )
#define CLIP_INC        dst      += stride

  SIZE_AWARE_PER_EL_OP( CLIP_OP, CLIP_INC );

#undef CLIP_INC
#undef CLIP_OP
}


template<unsigned trSize>
void fastInvCore_( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned lines, unsigned reducedLines, unsigned rows )
{
  for( int k = 0; k < rows; k++ )
  {
    const TCoeff* srcPtr = &src[k * lines];
    for( int i = 0; i < reducedLines; i++ )
    {
            TCoeff*       dstPtr = &dst[i * trSize];
      const TMatrixCoeff*  itPtr =  &it[k * trSize];
      for( int j = 0; j < trSize; j++ )
      {
        *dstPtr++ += *srcPtr * *itPtr++;
      }
      srcPtr++;
    }
  }
}


template<unsigned trSize>
void fastFwdCore( const TMatrixCoeff* tc, const TCoeff* src, TCoeff* dst, unsigned line, unsigned reducedLine, unsigned cutoff, int shift )
{
  const int rnd_factor = 1 << ( shift - 1 );

  for( int i = 0; i < reducedLine; i++ )
  {
          TCoeff*       dstPtr = dst;
    const TMatrixCoeff* iT     = tc;

    for( int j = 0; j < cutoff; j++ )
    {
      int sum = 0;

      for( int k = 0; k < trSize; k++ )
      {
        // dst[j * line + i] += src[i * trSize + k] * t[j * trSize + k]
        sum += src[k] * iT[k];
      }

      dstPtr[i] = ( sum + rnd_factor ) >> shift;
      dstPtr   += line;
      iT       += trSize;
    }

    src += trSize;
  }
}


TCoeffOps::TCoeffOps()
{
  cpyResi4        = cpyResiCore;
  cpyResi8        = cpyResiCore;
  cpyCoeff4       = cpyCoeffCore;
  cpyCoeff8       = cpyCoeffCore;
  roundClip4      = clipCore;
  roundClip8      = clipCore;
  fastInvCore[0]  = fastInvCore_< 4>;
  fastInvCore[1]  = fastInvCore_< 8>;
  fastInvCore[2]  = fastInvCore_<16>;
  fastInvCore[3]  = fastInvCore_<32>;
  fastInvCore[4]  = fastInvCore_<64>;
  fastFwdCore_1D[0] = fastFwdCore< 4>;
  fastFwdCore_1D[1] = fastFwdCore< 8>;
  fastFwdCore_1D[2] = fastFwdCore<16>;
  fastFwdCore_1D[3] = fastFwdCore<32>;
  fastFwdCore_1D[4] = fastFwdCore<64>;
  fastFwdCore_2D[0] = fastFwdCore< 4>;
  fastFwdCore_2D[1] = fastFwdCore< 8>;
  fastFwdCore_2D[2] = fastFwdCore<16>;
  fastFwdCore_2D[3] = fastFwdCore<32>;
  fastFwdCore_2D[4] = fastFwdCore<64>;
}

TCoeffOps g_tCoeffOps;

#endif


} // namespace vvenc

//! \}


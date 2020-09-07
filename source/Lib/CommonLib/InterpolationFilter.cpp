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
 * \brief Implementation of InterpolationFilter class
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "InterpolationFilter.h"
#include "Rom.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Tables
// ====================================================================================================================
const TFilterCoeff InterpolationFilter::m_lumaFilter4x4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  {  0, 1,  -5, 62,  8,  -3,  1,  0 },
  {  0, 2,  -8, 60, 13,  -4,  1,  0 },
  {  0, 3, -10, 58, 17,  -5,  1,  0 }, //1/4
  {  0, 3, -11, 52, 26,  -8,  2,  0 },
  {  0, 2,  -9, 47, 31, -10,  3,  0 },
  {  0, 3, -11, 45, 34, -10,  3,  0 },
  {  0, 3, -11, 40, 40, -11,  3,  0 }, //1/2
  {  0, 3, -10, 34, 45, -11,  3,  0 },
  {  0, 3, -10, 31, 47,  -9,  2,  0 },
  {  0, 2,  -8, 26, 52, -11,  3,  0 },
  {  0, 1,  -5, 17, 58, -10,  3,  0 }, //3/4
  {  0, 1,  -4, 13, 60,  -8,  2,  0 },
  {  0, 1,  -3,  8, 62,  -5,  1,  0 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 },
  {  0, 0,   0,  0, 64,   0,  0,  0 },
};

const TFilterCoeff InterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 4, -11, 52, 26,  -8,  3, -1 },
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 },
  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  { -1, 3,  -8, 26, 52, -11,  4, -1 },
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
  {  0, 1,  -4, 13, 60,  -8,  3, -1 },
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 },
  {  0, 0,   0,  0, 64,   0,  0,  0 },
};

const TFilterCoeff InterpolationFilter::m_lumaAltHpelIFilter[NTAPS_LUMA] = {  0, 3, 9, 20, 20, 9, 3, 0 };
const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -1, 63,  2,  0 },
  { -2, 62,  4,  0 },
  { -2, 60,  7, -1 },
  { -2, 58, 10, -2 },
  { -3, 57, 12, -2 },
  { -4, 56, 14, -2 },
  { -4, 55, 15, -2 },
  { -4, 54, 16, -2 },
  { -5, 53, 18, -2 },
  { -6, 52, 20, -2 },
  { -6, 49, 24, -3 },
  { -6, 46, 28, -4 },
  { -5, 44, 29, -4 },
  { -4, 42, 30, -4 },
  { -4, 39, 33, -4 },
  { -4, 36, 36, -4 },
  { -4, 33, 39, -4 },
  { -4, 30, 42, -4 },
  { -4, 29, 44, -5 },
  { -4, 28, 46, -6 },
  { -3, 24, 49, -6 },
  { -2, 20, 52, -6 },
  { -2, 18, 53, -5 },
  { -2, 16, 54, -4 },
  { -2, 15, 55, -4 },
  { -2, 14, 56, -4 },
  { -2, 12, 57, -3 },
  { -2, 10, 58, -2 },
  { -1,  7, 60, -2 },
  {  0,  4, 62, -2 },
  {  0,  2, 63, -1 },
  {  0,  0, 64,  0 },
};

const TFilterCoeff InterpolationFilter::m_bilinearFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
  { 64,  0, },
  { 60,  4, },
  { 56,  8, },
  { 52, 12, },
  { 48, 16, },
  { 44, 20, },
  { 40, 24, },
  { 36, 28, },
  { 32, 32, },
  { 28, 36, },
  { 24, 40, },
  { 20, 44, },
  { 16, 48, },
  { 12, 52, },
  { 8, 56, },
  { 4, 60, },
};

const TFilterCoeff InterpolationFilter::m_bilinearFilterPrec4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
  { 16,  0, },
  { 15,  1, },
  { 14,  2, },
  { 13, 3, },
  { 12, 4, },
  { 11, 5, },
  { 10, 6, },
  { 9, 7, },
  { 8, 8, },
  { 7, 9, },
  { 6, 10, },
  { 5, 11, },
  { 4, 12, },
  { 3, 13, },
  { 2, 14, },
  { 1, 15, }
};
// ====================================================================================================================
// Private member functions
// ====================================================================================================================

InterpolationFilter::InterpolationFilter()
{
  m_filterHor[0][0][0] = filter<8, false, false, false>;
  m_filterHor[0][0][1] = filter<8, false, false, true>;
  m_filterHor[0][1][0] = filter<8, false, true, false>;
  m_filterHor[0][1][1] = filter<8, false, true, true>;

  m_filterHor[1][0][0] = filter<4, false, false, false>;
  m_filterHor[1][0][1] = filter<4, false, false, true>;
  m_filterHor[1][1][0] = filter<4, false, true, false>;
  m_filterHor[1][1][1] = filter<4, false, true, true>;

  m_filterHor[2][0][0] = filter<2, false, false, false>;
  m_filterHor[2][0][1] = filter<2, false, false, true>;
  m_filterHor[2][1][0] = filter<2, false, true, false>;
  m_filterHor[2][1][1] = filter<2, false, true, true>;

  m_filterVer[0][0][0] = filter<8, true, false, false>;
  m_filterVer[0][0][1] = filter<8, true, false, true>;
  m_filterVer[0][1][0] = filter<8, true, true, false>;
  m_filterVer[0][1][1] = filter<8, true, true, true>;

  m_filterVer[1][0][0] = filter<4, true, false, false>;
  m_filterVer[1][0][1] = filter<4, true, false, true>;
  m_filterVer[1][1][0] = filter<4, true, true, false>;
  m_filterVer[1][1][1] = filter<4, true, true, true>;

  m_filterVer[2][0][0] = filter<2, true, false, false>;
  m_filterVer[2][0][1] = filter<2, true, false, true>;
  m_filterVer[2][1][0] = filter<2, true, true, false>;
  m_filterVer[2][1][1] = filter<2, true, true, true>;

  m_filterCopy[0][0]   = filterCopy<false, false>;
  m_filterCopy[0][1]   = filterCopy<false, true>;
  m_filterCopy[1][0]   = filterCopy<true, false>;
  m_filterCopy[1][1]   = filterCopy<true, true>;

  m_filter4x4[0][0] = filterXxY_N8<false, 4>;
  m_filter4x4[0][1] = filterXxY_N8<true , 4>;
  m_filter4x4[1][0] = filterXxY_N4<false, 4>;
  m_filter4x4[1][1] = filterXxY_N4<true , 4>;
  
  m_filter8x8[0][0] = filterXxY_N8<false, 8>;
  m_filter8x8[0][1] = filterXxY_N8<true , 8>;
  m_filter8x8[1][0] = filterXxY_N4<false, 8>;
  m_filter8x8[1][1] = filterXxY_N4<true , 8>;
  
  m_filter16x16[0][0] = filterXxY_N8<false, 16>;
  m_filter16x16[0][1] = filterXxY_N8<true , 16>;
  m_filter16x16[1][0] = filterXxY_N4<false, 16>;
  m_filter16x16[1][1] = filterXxY_N4<true , 16>;
//  m_weightedTriangleBlk = xWeightedTriangleBlk;
  m_weightedGeoBlk = xWeightedGeoBlk;
}


/**
 * \brief Apply unit FIR filter to a block of samples
 *
 * \param bitDepth   bitDepth of samples
 * \param src        Pointer to source samples
 * \param srcStride  Stride of source samples
 * \param dst        Pointer to destination samples
 * \param dstStride  Stride of destination samples
 * \param width      Width of block
 * \param height     Height of block
 * \param isFirst    Flag indicating whether it is the first filtering operation
 * \param isLast     Flag indicating whether it is the last filtering operation
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<bool isFirst, bool isLast>
void InterpolationFilter::filterCopy( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height, bool biMCForDMVR)
{
  int row, col;

  if ( isFirst == isLast )
  {
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        dst[col] = src[col];
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else if ( isFirst )
  {
    const unsigned shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));

    if (biMCForDMVR)
    {
      int shift10BitOut, offset;
      if ((clpRng.bd - IF_INTERNAL_PREC_BILINEAR) > 0)
      {
        shift10BitOut = (clpRng.bd - IF_INTERNAL_PREC_BILINEAR);
        offset = (1 << (shift10BitOut - 1));
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = (src[col] + offset) >> shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
      else
      {
        shift10BitOut = (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = src[col] << shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
    }
    else
    {
      for (row = 0; row < height; row++)
      {
        for (col = 0; col < width; col++)
        {
          Pel val = leftShiftU(src[col], shift);
          dst[col] = val - (Pel)IF_INTERNAL_OFFS;
        }

        src += srcStride;
        dst += dstStride;
      }
    }
  }
  else
  {
    const unsigned shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));

    if (biMCForDMVR)
    {
      int shift10BitOut, offset;
      if ((clpRng.bd - IF_INTERNAL_PREC_BILINEAR) > 0)
      {
        shift10BitOut = (clpRng.bd - IF_INTERNAL_PREC_BILINEAR);
        offset = (1 << (shift10BitOut - 1));
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = (src[col] + offset) >> shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
      else
      {
        shift10BitOut = (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = src[col] << shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
    }
    else
    {
      const Pel offset = ((1) << (shift - 1)) + IF_INTERNAL_OFFS;
      for (row = 0; row < height; row++)
      {
        for (col = 0; col < width; col++)
        {
          Pel val = src[ col ];
          val = rightShiftU((val + offset), shift);

          dst[col] = ClipPel( val, clpRng );
        }

        src += srcStride;
        dst += dstStride;
      }
    }
  }
}

/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<int N, bool isVertical, bool isFirst, bool isLast>
void InterpolationFilter::filter(const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR)
{
  int row, col;

  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if ( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if ( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if ( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }

  int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  int offset;
  int headRoom = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
  int shift    = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  CHECK(shift < 0, "Negative shift");

  if ( isLast )
  {
    shift += (isFirst) ? 0 : headRoom;
    offset = 1 << (shift - 1);
    offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift -= (isFirst) ? headRoom : 0;
    offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
  }

  if (biMCForDMVR)
  {
    if( isFirst )
    {
      shift = IF_FILTER_PREC_BILINEAR - (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
      offset = 1 << (shift - 1);
    }
    else
    {
      shift = 4;
      offset = 1 << (shift - 1);
    }
  }
  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      int sum;

      sum  = src[ col + 0 * cStride] * c[0];
      sum += src[ col + 1 * cStride] * c[1];
      if ( N >= 4 )
      {
        sum += src[ col + 2 * cStride] * c[2];
        sum += src[ col + 3 * cStride] * c[3];
      }
      if ( N >= 6 )
      {
        sum += src[ col + 4 * cStride] * c[4];
        sum += src[ col + 5 * cStride] * c[5];
      }
      if ( N == 8 )
      {
        sum += src[ col + 6 * cStride] * c[6];
        sum += src[ col + 7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if ( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<int N>
void InterpolationFilter::filterHor(const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR)
{
  if( N == 8 )
  {
    m_filterHor[0][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if( N == 4 )
  {
    m_filterHor[1][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if( N == 2 )
  {
    m_filterHor[2][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else
  {
    THROW( "Invalid tap number" );
  }
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<int N>
void InterpolationFilter::filterVer(const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, bool isFirst, bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR)
{
  if( N == 8 )
  {
    m_filterVer[0][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if( N == 4 )
  {
    m_filterVer[1][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if( N == 2 )
  {
    m_filterVer[2][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else{
    THROW( "Invalid tap number" );
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * \brief Filter a block of Luma/Chroma samples (horizontal)
 *
 * \param  compID     Chroma component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
void InterpolationFilter::filterHor(const ComponentID compID, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, int frac, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf, int nFilterIdx, bool biMCForDMVR)
{
  if( frac == 0 )
  {
    m_filterCopy[true][isLast](clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else if( isLuma( compID ) )
  {
    CHECK( frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    if( nFilterIdx == 1 )
    {
      filterHor<NTAPS_BILINEAR>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_bilinearFilterPrec4[frac], biMCForDMVR);
    }
    else
    {
      if (frac == 8 && useAltHpelIf)
      {
        filterHor<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaAltHpelIFilter, biMCForDMVR);
      }
      else if ((width == 4 && height == 4) || (width == 4 && height == (4 + NTAPS_LUMA - 1)))
      {
        filterHor<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter4x4[frac], biMCForDMVR);
      }
      else
      {
       filterHor<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac], biMCForDMVR);
      }
    }
  }
  else
  {
    const uint32_t csx = getComponentScaleX( compID, fmt );
    CHECK( frac < 0 || csx >= 2 || ( frac << ( 1 - csx ) ) >= CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac << ( 1 - csx )], biMCForDMVR);
  }
}


/**
 * \brief Filter a block of Luma/Chroma samples (vertical)
 *
 * \param  compID     Colour component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
void InterpolationFilter::filterVer(const ComponentID compID, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf, int nFilterIdx, bool biMCForDMVR)
{
  if( frac == 0 )
  {
    m_filterCopy[isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else if( isLuma( compID ) )
  {
    CHECK( frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    if (nFilterIdx == 1)
    {
      filterVer<NTAPS_BILINEAR>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_bilinearFilterPrec4[frac], biMCForDMVR);
    }
    else
    {
      if (frac == 8 && useAltHpelIf)
      {
        filterVer<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaAltHpelIFilter, biMCForDMVR);
      }
      else if (width == 4 && height == 4)
      {
        filterVer<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter4x4[frac], biMCForDMVR);
      }
      else
      {
        filterVer<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter[frac], biMCForDMVR);
      }
    }
  }
  else
  {
    const uint32_t csy = getComponentScaleY( compID, fmt );
    CHECK( frac < 0 || csy >= 2 || ( frac << ( 1 - csy ) ) >= CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    filterVer<NTAPS_CHROMA>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac << (1 - csy)], biMCForDMVR);
  }
}

void InterpolationFilter::filter4x4( const ComponentID compID, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng,bool useAltHpelIf/*= false*/,int nFilterIdx /*= 0*/ )
{
  const int vFilterSize = nFilterIdx == 1 ? 2 : ( isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA );
  
  if( vFilterSize == 8 )
  {
   CHECKD( !isLuma( compID ), "8-tap filter is only allowed for luma!" );

    const TFilterCoeff* vCoeff = useAltHpelIf ? m_lumaAltHpelIFilter : m_lumaFilter4x4[fracY];
    const TFilterCoeff* hCoeff = useAltHpelIf ? m_lumaAltHpelIFilter : m_lumaFilter4x4[fracX];

    m_filter4x4[0][isLast]( clpRng, src,srcStride,  dst,dstStride, width, height, hCoeff, vCoeff );
  }
  else if( vFilterSize == 4 )
  {
    CHECKD( !isChroma( compID ), "4-tap filter is only allowed for luma!" );

    const int csx = getComponentScaleX( compID, fmt );
    const int csy = getComponentScaleY( compID, fmt );

    m_filter4x4[1][isLast]( clpRng, src,srcStride, dst,dstStride, width, height, m_chromaFilter[fracX << ( 1 - csx )], m_chromaFilter[fracY << ( 1 - csy )] );
  }
  else
  {
    THROW( "4x4 interpolation filter does not support bilinear filtering!" );
  }
}
void InterpolationFilter::filter8x8( const ComponentID compID, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height,  int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng,bool useAltHpelIf,int nFilterIdx /*= 0*/ )
{
  const int vFilterSize = nFilterIdx == 1 ? 2 : ( isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA );

  if( vFilterSize == 8 )
  {
    CHECKD( !isLuma( compID ), "8-tap filter is only allowed for luma!" );

    const TFilterCoeff* vCoeff = (useAltHpelIf && fracY==8) ? m_lumaAltHpelIFilter : m_lumaFilter[fracY];
    const TFilterCoeff* hCoeff = (useAltHpelIf && fracX==8) ? m_lumaAltHpelIFilter : m_lumaFilter[fracX];

    m_filter8x8[0][isLast]( clpRng,src,srcStride, dst,dstStride, width, height, hCoeff, vCoeff);
  }
  else if( vFilterSize == 4 )
  {
    CHECKD( !isChroma( compID ), "4-tap filter is only allowed for luma!" );

    const int csx = getComponentScaleX( compID, fmt );
    const int csy = getComponentScaleY( compID, fmt );

    m_filter8x8[1][isLast]( clpRng, src,srcStride, dst,dstStride, width, height, m_chromaFilter[fracX << ( 1 - csx )], m_chromaFilter[fracY << ( 1 - csy )] );
  }
  else
  {
    THROW( "8x8 interpolation filter does not support bilinear filtering!" );
  }
}

void InterpolationFilter::filter16x16( const ComponentID compID, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng,bool useAltHpelIf, int nFilterIdx /*= 0*/ )
{
  const int vFilterSize = nFilterIdx == 1 ? 2 : ( isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA );

  if( vFilterSize == 8 )
  {
    CHECKD( !isLuma( compID ), "8-tap filter is only allowed for luma!" );

    const TFilterCoeff* vCoeff = (useAltHpelIf && fracY==8) ? m_lumaAltHpelIFilter : m_lumaFilter[fracY];
    const TFilterCoeff* hCoeff = (useAltHpelIf && fracX==8) ? m_lumaAltHpelIFilter : m_lumaFilter[fracX];

    m_filter16x16[0][isLast]( clpRng, src, srcStride, dst,dstStride, width, height, hCoeff, vCoeff );
  }
  else if( vFilterSize == 4 )
  {
    CHECKD( !isChroma( compID ), "4-tap filter is only allowed for luma!" );

    const int csx = getComponentScaleX( compID, fmt );
    const int csy = getComponentScaleY( compID, fmt );

    m_filter16x16[1][isLast]( clpRng, src,srcStride, dst,dstStride, width, height, m_chromaFilter[fracX << ( 1 - csx )], m_chromaFilter[fracY << ( 1 - csy )] );
  }
  else
  {
    THROW( "16x16 interpolation filter does not support bilinear filtering!" );
  }
}

template<bool isLast, int w>
void InterpolationFilter::filterXxY_N4( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* _dst, int dstStride, int width, int height, const TFilterCoeff *coeffH, const TFilterCoeff *coeffV )
{
  int row, col;

  Pel cH[4];
  cH[0] = coeffH[0]; cH[1] = coeffH[1];
  cH[2] = coeffH[2]; cH[3] = coeffH[3];
  Pel cV[4];
  cV[0] = coeffV[0]; cV[1] = coeffV[1];
  cV[2] = coeffV[2]; cV[3] = coeffV[3];

  int offset1st, offset2nd;
  int headRoom   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st   = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if( isLast )
  {
    shift1st  -= headRoom;
    shift2nd  += headRoom;
    offset1st  = -IF_INTERNAL_OFFS << shift1st;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS << shift1st;
    offset2nd = 0;
  }

  src -= 1 + srcStride;

  int *tmp = ( int * ) alloca( w * height * sizeof( int ) );
  memset( tmp, 0, w * height * sizeof( int ) );

  int** dst = ( int ** ) alloca( height * sizeof( int * ) );

  for( int i = 0; i < height; i++ ) dst[i] = &tmp[i * w];

  for( row = 0; row < ( height + 3 ); row++ )
  {
    for( col = 0; col < w; col++ )
    {
      int sum;

      sum  = src[col    ] * cH[0];
      sum += src[col + 1] * cH[1];
      sum += src[col + 2] * cH[2];
      sum += src[col + 3] * cH[3];

      sum = ( sum + offset1st ) >> shift1st;

      if( row >= 0 && row < ( height + 0 ) ) dst[row    ][col] += sum * cV[0];
      if( row >= 1 && row < ( height + 1 ) ) dst[row - 1][col] += sum * cV[1]; 
      if( row >= 2 && row < ( height + 2 ) ) dst[row - 2][col] += sum * cV[2];

      if( row >= 3 )
      {
        int val = ( dst[row - 3][col] + sum * cV[3] + offset2nd ) >> shift2nd;
        if( isLast )
        {
          val = ClipPel( val, clpRng );
        }
        _dst[col] = val;
      }
    }

    src += srcStride;
    if( row >= 3 ) _dst += dstStride;
  }
}


template<bool isLast, int w>
void InterpolationFilter::filterXxY_N8( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* _dst, int dstStride, int width, int h, const TFilterCoeff *coeffH, const TFilterCoeff *coeffV )
{
  int row, col;

  Pel cH[8];
  cH[0] = coeffH[0]; cH[1] = coeffH[1];
  cH[2] = coeffH[2]; cH[3] = coeffH[3];
  cH[4] = coeffH[4]; cH[5] = coeffH[5];
  cH[6] = coeffH[6]; cH[7] = coeffH[7];
  Pel cV[8];
  cV[0] = coeffV[0]; cV[1] = coeffV[1];
  cV[2] = coeffV[2]; cV[3] = coeffV[3];
  cV[4] = coeffV[4]; cV[5] = coeffV[5];
  cV[6] = coeffV[6]; cV[7] = coeffV[7];

  int offset1st, offset2nd;
  int headRoom   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st   = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if( isLast )
  {
    shift1st  -= headRoom;
    shift2nd  += headRoom;
    offset1st  = -IF_INTERNAL_OFFS << shift1st;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS << shift1st;
    offset2nd = 0;
  }

  src -= 3 * ( 1 + srcStride );

  int *tmp = ( int * ) alloca( w * h * sizeof( int ) );
  memset( tmp, 0, w * h * sizeof( int ) );

  int** dst = ( int ** ) alloca( h * sizeof( int * ) );

  for( int i = 0; i < h; i++ ) dst[i] = &tmp[i * w];

  for( row = 0; row < ( h + 7 ); row++ )
  {
    for( col = 0; col < w; col++ )
    {
      int sum;

      sum  = src[col    ] * cH[0];
      sum += src[col + 1] * cH[1];
      sum += src[col + 2] * cH[2];
      sum += src[col + 3] * cH[3];
      sum += src[col + 4] * cH[4];
      sum += src[col + 5] * cH[5];
      sum += src[col + 6] * cH[6];
      sum += src[col + 7] * cH[7];

      sum = ( sum + offset1st ) >> shift1st;

      if( row >= 0 && row < ( h + 0 ) ) dst[row    ][col] += sum * cV[0];
      if( row >= 1 && row < ( h + 1 ) ) dst[row - 1][col] += sum * cV[1]; 
      if( row >= 2 && row < ( h + 2 ) ) dst[row - 2][col] += sum * cV[2];
      if( row >= 3 && row < ( h + 3 ) ) dst[row - 3][col] += sum * cV[3];
      if( row >= 4 && row < ( h + 4 ) ) dst[row - 4][col] += sum * cV[4];
      if( row >= 5 && row < ( h + 5 ) ) dst[row - 5][col] += sum * cV[5];
      if( row >= 6 && row < ( h + 6 ) ) dst[row - 6][col] += sum * cV[6];

      if( row >= 7 )
      {
        int val = ( dst[row - 7][col] + sum * cV[7] + offset2nd ) >> shift2nd;
        if( isLast )
        {
          val = ClipPel( val, clpRng );
        }
        _dst[col] = val;
      }
    }

    src += srcStride;
    if( row >= 7 ) _dst += dstStride;
  }
}

void InterpolationFilter::weightedGeoBlk(const ClpRngs &clpRngs, const PredictionUnit &pu, const uint32_t width,
                                         const uint32_t height,
                                         const ComponentID compIdx, const uint8_t splitDir,PelUnitBuf &predDst,
                                         PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  m_weightedGeoBlk(clpRngs, pu, width, height, compIdx, splitDir, predDst, predSrc0, predSrc1);
}

void InterpolationFilter::xWeightedGeoBlk(const ClpRngs &clpRngs, const PredictionUnit &pu, const uint32_t width,
                                          const uint32_t height,
                                          const ComponentID compIdx, const uint8_t splitDir,PelUnitBuf &predDst,
                                          PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  Pel *   dst        = predDst.get(compIdx).buf;
  Pel *   src0       = predSrc0.get(compIdx).buf;
  Pel *   src1       = predSrc1.get(compIdx).buf;
  int32_t strideDst  = predDst.get(compIdx).stride - width;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride - width;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride - width;

  const char log2WeightBase = 3;
  // const ClpRng clipRng        = pu.cu->cs->slice->clpRngs[compIdx]   // pu.cu->slice->clpRngs().comp[compIdx];
  const int32_t  clipbd         = clpRngs[compIdx].bd;
  const int32_t  shiftWeighted  = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
  const int32_t  offsetWeighted = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);
  const uint32_t scaleX         = getComponentScaleX(compIdx, pu.chromaFormat);
  const uint32_t scaleY         = getComponentScaleY(compIdx, pu.chromaFormat);

  int16_t  angle  = g_GeoParams[splitDir][0];
  int16_t  wIdx   = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t  hIdx   = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t  stepX  = 1 << scaleX;
  int16_t  stepY  = 0;
  int16_t *weight = nullptr;
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -(int) ((GEO_WEIGHT_MASK_SIZE << scaleY) + pu.lwidth());
    weight =
      &g_globalGeoWeights[g_angle2mask[angle]]
                         [(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[hIdx][wIdx][splitDir][1]) * GEO_WEIGHT_MASK_SIZE
                          + g_weightOffset[hIdx][wIdx][splitDir][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX = -1 << scaleX;
    stepY = (GEO_WEIGHT_MASK_SIZE << scaleY) + pu.lwidth();
    weight =
      &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[hIdx][wIdx][splitDir][1] * GEO_WEIGHT_MASK_SIZE
                                               + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[hIdx][wIdx][splitDir][0])];
  }
  else
  {
    stepY  = (GEO_WEIGHT_MASK_SIZE << scaleY) - pu.lwidth();
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[hIdx][wIdx][splitDir][1] * GEO_WEIGHT_MASK_SIZE
                                                      + g_weightOffset[hIdx][wIdx][splitDir][0]];
  }
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *dst++ = ClipPel(rightShiftU((*weight * (*src0++) + ((8 - *weight) * (*src1++)) + offsetWeighted), shiftWeighted),clpRngs[compIdx]);
      weight += stepX;
    }
    dst += strideDst;
    src0 += strideSrc0;
    src1 += strideSrc1;
    weight += stepY;
  }
}

/**
 * \brief turn on SIMD fuc
 *
 * \param bEn   enabled of SIMD function for interpolation
 */
void InterpolationFilter::initInterpolationFilter( bool enable )
{
#if ENABLE_SIMD_OPT_MCIF
#ifdef TARGET_SIMD_X86
  if ( enable )
  {
    initInterpolationFilterX86();
  }
#endif
#endif
}

} // namespace vvenc

//! \}


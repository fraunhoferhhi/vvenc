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


/** \file     Buffer.cpp
 *  \brief    Low-overhead class describing 2D memory layout
 */

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// unit needs to come first due to a forward declaration
#include "Unit.h"
#include "Slice.h"
#include "InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

void weightCiipCore( Pel* res, const Pel* src, const int numSamples, int numIntra )
{
  if( numIntra == 1 )
  {
    for (int n = 0; n < numSamples; n+=2)
    {
      res[n  ] = (res[n  ] + src[n  ] + 1) >> 1;
      res[n+1] = (res[n+1] + src[n+1] + 1) >> 1;
    }
  }
  else
  {
    const Pel* scale   = numIntra ? src : res;
    const Pel* unscale = numIntra ? res : src;

    for (int n = 0; n < numSamples; n+=2)
    {
      res[n  ] = (unscale[n  ] + 3*scale[n  ] + 2) >> 2;
      res[n+1] = (unscale[n+1] + 3*scale[n+1] + 2) >> 2;
    }
  }
}

template< unsigned inputSize, unsigned outputSize >
void mipMatrixMulCore( Pel* res, const Pel* input, const uint8_t* weight, const int maxVal, const int inputOffset, bool transpose )
{
  Pel buffer[ outputSize*outputSize];

  int sum = 0;
  for( int i = 0; i < inputSize; i++ )
  {
    sum += input[i];
  }
  const int offset = (1 << (MIP_SHIFT_MATRIX - 1)) - MIP_OFFSET_MATRIX * sum + (inputOffset << MIP_SHIFT_MATRIX);
  CHECK( inputSize != 4 * (inputSize >> 2), "Error, input size not divisible by four" );

  Pel* mat = transpose ? buffer : res;
  unsigned posRes = 0;
  for( unsigned n = 0; n < outputSize*outputSize; n++ )
  {
    int tmp0 = input[0] * weight[0];
    int tmp1 = input[1] * weight[1];
    int tmp2 = input[2] * weight[2];
    int tmp3 = input[3] * weight[3];
    if( 8 == inputSize )
    {
      tmp0 += input[4] * weight[4];
      tmp1 += input[5] * weight[5];
      tmp2 += input[6] * weight[6];
      tmp3 += input[7] * weight[7];
    }
    mat[posRes++] = Clip3<int>( 0, maxVal, ((tmp0 + tmp1 + tmp2 + tmp3 + offset) >> MIP_SHIFT_MATRIX) );

    weight += inputSize;
  }

  if( transpose )
  {
    for( int j = 0; j < outputSize; j++ )
    {
      for( int i = 0; i < outputSize; i++ )
      {
        res[j * outputSize + i] = buffer[i * outputSize + j];
      }
    }
  }
}

template< typename T >
void addAvgCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int dstStride, int width, int height, unsigned rshift, int offset, const ClpRng& clpRng )
{
#define ADD_AVG_CORE_OP( ADDR ) dest[ADDR] = ClipPel( rightShiftU( ( src1[ADDR] + src2[ADDR] + offset ), rshift ), clpRng )
#define ADD_AVG_CORE_INC    \
  src1 += src1Stride;       \
  src2 += src2Stride;       \
  dest +=  dstStride;       \

  SIZE_AWARE_PER_EL_OP( ADD_AVG_CORE_OP, ADD_AVG_CORE_INC );

#undef ADD_AVG_CORE_OP
#undef ADD_AVG_CORE_INC
}

template<typename T>
void addWeightedAvgCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int destStride, int width, int height, unsigned rshift, int offset, int w0, int w1, const ClpRng& clpRng )
{
#define ADD_WGHT_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShiftU( ( src1[ADDR]*w0 + src2[ADDR]*w1 + offset ), rshift ), clpRng )
#define ADD_WGHT_AVG_INC     \
    src1 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

  SIZE_AWARE_PER_EL_OP( ADD_WGHT_AVG_OP, ADD_WGHT_AVG_INC );

#undef ADD_WGHT_AVG_OP
#undef ADD_WGHT_AVG_INC
}

template<typename T>
void subsCore( const T* src0, int src0Stride, const T* src1, int src1Stride, T* dest, int destStride, int width, int height )
{
#define SUBS_INC                \
  dest += destStride;  \
  src0 += src0Stride;  \
  src1 += src1Stride;  \

#define SUBS_OP( ADDR ) dest[ADDR] = src0[ADDR] - src1[ADDR]

  SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
}

void removeHighFreq(int16_t* dst, int dstStride, const int16_t* src, int srcStride, int width, int height)
{
#define REM_HF_INC  \
 src += srcStride; \
 dst += dstStride; \

#define REM_HF_OP( ADDR )      dst[ADDR] =             2 * dst[ADDR] - src[ADDR]

 SIZE_AWARE_PER_EL_OP(REM_HF_OP, REM_HF_INC);

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP
}

template<typename T>
void reconstructCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int dstStride, int width, int height, const ClpRng& clpRng )
{
#define RECO_CORE_OP( ADDR ) dest[ADDR] = ClipPel( src1[ADDR] + src2[ADDR], clpRng )
#define RECO_CORE_INC     \
  src1 += src1Stride;     \
  src2 += src2Stride;     \
  dest +=  dstStride;     \

  SIZE_AWARE_PER_EL_OP( RECO_CORE_OP, RECO_CORE_INC );

#undef RECO_CORE_OP
#undef RECO_CORE_INC
}

template<typename T>
void recoCore( const T* src1, const T* src2, T* dest, int numSamples, const ClpRng& clpRng )
{
  for( int n = 0; n < numSamples; n+=2)
  {
    dest[n]   = ClipPel( src1[n]   + src2[n], clpRng );
    dest[n+1] = ClipPel( src1[n+1] + src2[n+1], clpRng );
  }
}

template<typename T>
void copyClipCore( const T* src, Pel* dst, int numSamples, const ClpRng& clpRng )
{
  for( int n = 0; n < numSamples; n+=2)
  {
    dst[n]   = ClipPel( src[n]   , clpRng );
    dst[n+1] = ClipPel( src[n+1] , clpRng );
  }
}

template< typename T >
void addAvgCore( const T* src1, const T* src2, T* dest, int numSamples, unsigned rshift, int offset, const ClpRng& clpRng )
{
  for( int n = 0; n < numSamples; n+=2)
  {
    dest[n]   = ClipPel( rightShiftU( ( src1[n]   + src2[n]   + offset ), rshift ), clpRng );
    dest[n+1] = ClipPel( rightShiftU( ( src1[n+1] + src2[n+1] + offset ), rshift ), clpRng );
  }
}

template< typename T >
void roundGeoCore( const T* src, T* dest, const int numSamples, unsigned rshift, int offset, const ClpRng &clpRng)
{
  for( int i = 0; i < numSamples; i+=2)
  {
    dest[i]   = ClipPel(rightShiftU(src[i  ] + offset, rshift), clpRng);
    dest[i+1] = ClipPel(rightShiftU(src[i+1] + offset, rshift), clpRng);
  }
}

template<typename T>
void linTfCore( const T* src, int srcStride, Pel* dst, int dstStride, int width, int height, int scale, unsigned shift, int offset, const ClpRng& clpRng, bool bClip )
{
#define LINTF_CORE_INC  \
  src += srcStride;     \
  dst += dstStride;     \

  if( bClip )
  {
#define LINTF_CORE_OP( ADDR ) dst[ADDR] = ( Pel ) ClipPel( rightShiftU( scale * src[ADDR], shift ) + offset, clpRng )

  SIZE_AWARE_PER_EL_OP( LINTF_CORE_OP, LINTF_CORE_INC );

#undef LINTF_CORE_OP
  }
  else
  {
#define LINTF_CORE_OP( ADDR ) dst[ADDR] = ( Pel ) ( rightShiftU( scale * src[ADDR], shift ) + offset )

  SIZE_AWARE_PER_EL_OP( LINTF_CORE_OP, LINTF_CORE_INC );

#undef LINTF_CORE_OP
  }
#undef LINTF_CORE_INC
}

template<typename T, int N>
void transposeNxNCore( const Pel* src, int srcStride, Pel* dst, int dstStride )
{
  for( int i = 0; i < N; i++ )
  {
    for( int j = 0; j < N; j++ )
    {
      dst[j * dstStride] = src[j];
    }

    dst++;
    src += srcStride;
  }
}

template<typename T>
void copyClipCore( const T* src, int srcStride, Pel* dst, int dstStride, int width, int height, const ClpRng& clpRng )
{
#define RECO_OP( ADDR ) dst[ADDR] = ClipPel( src[ADDR], clpRng )
#define RECO_INC      \
    src += srcStride; \
    dst += dstStride; \

  SIZE_AWARE_PER_EL_OP( RECO_OP, RECO_INC );

#undef RECO_OP
#undef RECO_INC
}

void copyBufferCore( const char* src, int srcStride, char* dst, int dstStride, int numBytes, int height)
{
  for( int i = 0; i < height; i++, src += srcStride, dst += dstStride )
  {
    memcpy( dst, src, numBytes );
  }
}

void applyLutCore( const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, const Pel* lut )
{
#define RSP_SGNL_OP( ADDR ) dst[ADDR] = lut[src[ADDR]]
#define RSP_SGNL_INC        src      += srcStride; dst += dstStride;

  SIZE_AWARE_PER_EL_OP( RSP_SGNL_OP, RSP_SGNL_INC )

#undef RSP_SGNL_OP
#undef RSP_SGNL_INC
}

void fillMapPtr_Core( void** ptrMap, const ptrdiff_t mapStride, int width, int height, void* val )
{
  if( width == mapStride )
  {
    std::fill_n( ptrMap, width * height, val );
  }
  else
  {
    while( height-- )
    {
      std::fill_n( ptrMap, width, val );
      ptrMap += mapStride;
    }
  }
}

PelBufferOps::PelBufferOps()
{
  isInitX86Done = false;

  addAvg            = addAvgCore<Pel>;
  reco              = recoCore<Pel>;
  copyClip          = copyClipCore<Pel>;
  roundGeo          = roundGeoCore<Pel>;

  addAvg4           = addAvgCore<Pel>;
  addAvg8           = addAvgCore<Pel>;
  addAvg16          = addAvgCore<Pel>;

  sub4              = subsCore<Pel>;
  sub8              = subsCore<Pel>;

  wghtAvg4          = addWeightedAvgCore<Pel>;
  wghtAvg8          = addWeightedAvgCore<Pel>;

  copyClip4         = copyClipCore<Pel>;
  copyClip8         = copyClipCore<Pel>;

  reco4             = reconstructCore<Pel>;
  reco8             = reconstructCore<Pel>;

  linTf4            = linTfCore<Pel>;
  linTf8            = linTfCore<Pel>;

  copyBuffer        = copyBufferCore;

  removeHighFreq8   = removeHighFreq;
  removeHighFreq4   = removeHighFreq;

  transpose4x4      = transposeNxNCore<Pel,4>;
  transpose8x8      = transposeNxNCore<Pel,8>;
  mipMatrixMul_4_4  = mipMatrixMulCore<4,4>;
  mipMatrixMul_8_4  = mipMatrixMulCore<8,4>;
  mipMatrixMul_8_8  = mipMatrixMulCore<8,8>;
  weightCiip        = weightCiipCore;
  roundIntVector    = nullptr;

  applyLut          = applyLutCore;

  fillPtrMap        = fillMapPtr_Core;
}

PelBufferOps g_pelBufOP = PelBufferOps();

template<>
void AreaBuf<Pel>::addWeightedAvg(const AreaBuf<const Pel>& other1, const AreaBuf<const Pel>& other2, const ClpRng& clpRng, const int8_t BcwIdx)
{
  const int8_t w0 = getBcwWeight( BcwIdx, REF_PIC_LIST_0 );
  const int8_t w1 = getBcwWeight( BcwIdx, REF_PIC_LIST_1 );
  const int8_t log2WeightBase = g_BcwLog2WeightBase;
  const Pel* src0 = other1.buf;
  const Pel* src2 = other2.buf;
        Pel* dest =        buf;

  const int src1Stride = other1.stride;
  const int src2Stride = other2.stride;
  const int destStride =        stride;
  const int clipbd     = clpRng.bd;
  const int shiftNum   = std::max<int>( 2, ( IF_INTERNAL_PREC - clipbd ) ) + log2WeightBase;
  const int offset     = ( 1 << ( shiftNum - 1 ) ) + ( IF_INTERNAL_OFFS << log2WeightBase );

  if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.wghtAvg8( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, w0, w1, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.wghtAvg4( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, w0, w1, clpRng );
  }
  else
  {
#define WGHT_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShiftU( ( src0[ADDR]*w0 + src2[ADDR]*w1 + offset ), shiftNum ), clpRng )
#define WGHT_AVG_INC    \
    src0 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( WGHT_AVG_OP, WGHT_AVG_INC );

#undef WGHT_AVG_OP
#undef WGHT_AVG_INC
  }
}

template<>
void AreaBuf<Pel>::rspSignal( const Pel* pLUT)
{
  g_pelBufOP.applyLut( buf, stride, buf, stride, width, height, pLUT );
}


template<>
void AreaBuf<Pel>::rspSignal( const AreaBuf<const Pel>& other, const Pel* pLUT)
{
  g_pelBufOP.applyLut( other.buf, other.stride, buf, stride, width, height, pLUT );
}

template<>
void AreaBuf<Pel>::scaleSignal(const int scale, const bool dir, const ClpRng& clpRng)
{
        Pel* dst = buf;
  const Pel* src = buf;
  const int maxAbsclipBD = (1<<clpRng.bd) - 1;

  if (dir) // forward
  {
    if (width == 1)
    {
      THROW("Blocks of width = 1 not supported");
    }
    else
    {
      for (unsigned y = 0; y < height; y++)
      {
        for (unsigned x = 0; x < width; x++)
        {
          int sign = src[x] >= 0 ? 1 : -1;
          int absval = sign * src[x];
          dst[x] = (Pel)Clip3(-maxAbsclipBD, maxAbsclipBD, sign * (((absval << CSCALE_FP_PREC) + (scale >> 1)) / scale));
        }
        dst += stride;
        src += stride;
      }
    }
  }
  else // inverse
  {
    for (unsigned y = 0; y < height; y++)
    {
      for (unsigned x = 0; x < width; x++)
      {
        int val    = Clip3<int>((-maxAbsclipBD - 1), maxAbsclipBD, (int)src[x]);
        int sign   = src[x] >= 0 ? 1 : -1;
        int absval = sign * val;
               val = sign * ((absval * scale + (1 << (CSCALE_FP_PREC - 1))) >> CSCALE_FP_PREC);
        if (sizeof(Pel) == 2) // avoid overflow when storing data
        {
          val = Clip3<int>(-32768, 32767, val);
        }
        dst[x] = (Pel)val;
      }
      dst += stride;
      src += stride;
    }
  }
}

template<>
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel>& other1, const AreaBuf<const Pel>& other2, const ClpRng& clpRng)
{
  const Pel* src0 = other1.buf;
  const Pel* src2 = other2.buf;
        Pel* dest =        buf;

  const unsigned src1Stride = other1.stride;
  const unsigned src2Stride = other2.stride;
  const unsigned destStride =        stride;
  const int      clipbd     = clpRng.bd;
  const unsigned shiftNum   = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + 1;
  const int      offset     = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;

#if ENABLE_SIMD_OPT_BUFFER
  if( destStride == width )
  {
    g_pelBufOP.addAvg(src0, src2, dest, width * height, shiftNum, offset, clpRng);
  }
  else if ((width & 15) == 0)
  {
    g_pelBufOP.addAvg16(src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng);
  }
  else if( width > 2 && height > 2 && width == destStride )
  {
    g_pelBufOP.addAvg16(src0, src1Stride<<2, src2, src2Stride<<2, dest, destStride<<2, width<<2, height>>2, shiftNum, offset, clpRng);
  }
  else if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.addAvg8( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.addAvg4( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else
#endif
  {
#define ADD_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShiftU( ( src0[ADDR] + src2[ADDR] + offset ), shiftNum ), clpRng )
#define ADD_AVG_INC     \
    src0 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( ADD_AVG_OP, ADD_AVG_INC );

#undef ADD_AVG_OP
#undef ADD_AVG_INC
  }
}

template<>
void AreaBuf<Pel>::subtract( const AreaBuf<const Pel>& minuend, const AreaBuf<const Pel>& subtrahend )
{
  CHECKD( width  != minuend.width,     "Incompatible size" );
  CHECKD( height != minuend.height,    "Incompatible size" );
  CHECKD( width  != subtrahend.width,  "Incompatible size");
  CHECKD( height != subtrahend.height, "Incompatible size");
  
        Pel* dest =            buf;
  const Pel* mins = minuend   .buf;
  const Pel* subs = subtrahend.buf;


  const unsigned destStride =            stride;
  const unsigned minsStride = minuend.   stride;
  const unsigned subsStride = subtrahend.stride;

#if ENABLE_SIMD_OPT_BUFFER
  if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.sub8( mins, minsStride, subs, subsStride, dest, destStride, width, height );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.sub4( mins, minsStride, subs, subsStride, dest, destStride, width, height );
  }
  else
#endif
  {
#define SUBS_INC                \
    dest +=            stride;  \
    mins += minuend   .stride;  \
    subs += subtrahend.stride;  \

#define SUBS_OP( ADDR ) dest[ADDR] = mins[ADDR] - subs[ADDR]

    SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
  }
}

template<>
void AreaBuf<const Pel>::calcVarianceSplit( const AreaBuf<const Pel>& Org, const uint32_t  size, int& varh,int& varv) const
{
  CHECK( Org.width != Org.height, "Incompatible size!" );
  int stride = Org.stride;
  const Pel* src;
  Pel data;
  double variance=0;
  double mean=0;
  int64_t sum[4]={0,0,0,0};
  int64_t sum_sqr[4]={0,0,0,0};
  uint32_t halfsize =size>>1;
  uint32_t off[4]={0,halfsize,size*halfsize,size*halfsize+halfsize};
  int n,x,y;

  for( n = 0; n < 4; n++)
  {
    src = Org.buf+off[n];
    for( y = 0; y < halfsize; y++)
    {
      for(x = 0; x < halfsize; x++)
      {
        data=src[y*stride+x];
        sum[n]+=data;
        sum_sqr[n]+= data*data;
      }
    }
  }
  int num=size*(size>>1);
  // varhu
  mean=(double)(sum[0]+sum[1])/(num);
  variance =  (double)(sum_sqr[0]+sum_sqr[1])/(num) - (mean*mean);
  varh =(int)(variance+0.5);
  // varhl
  mean=(double)(sum[2]+sum[3])/(num);
  variance =  (double)(sum_sqr[2]+sum_sqr[3])/(num) - (mean*mean);
  varh +=(int)(variance+0.5);
  // varvl
  mean=(double)(sum[0]+sum[2])/(num);
  variance =  (double)(sum_sqr[0]+sum_sqr[2])/(num) - (mean*mean);
  varv =(int)(variance+0.5);
  // varvr
  mean=(double)(sum[1]+sum[3])/(num);
  variance =  (double)(sum_sqr[1]+sum_sqr[3])/(num) - (mean*mean);
  varv +=(int)(variance+0.5);
}

template<>
void AreaBuf<Pel>::copyClip( const AreaBuf<const Pel>& src, const ClpRng& clpRng )
{
  const Pel* srcp = src.buf;
        Pel* dest =     buf;

  const unsigned srcStride  = src.stride;
  const unsigned destStride = stride;

  if( destStride == width)
  {
    g_pelBufOP.copyClip(srcp, dest, width * height, clpRng);
  }
  else if ((width & 7) == 0)
  {
    g_pelBufOP.copyClip8(srcp, srcStride, dest, destStride, width, height, clpRng);
  }
  else if ((width & 3) == 0)
  {
    g_pelBufOP.copyClip4(srcp, srcStride, dest, destStride, width, height, clpRng);
  }
  else
  {
    for( int y = 0; y < height; y++ )
    {
      dest[0] = ClipPel( srcp[0], clpRng);
      dest[1] = ClipPel( srcp[1], clpRng);
      srcp += srcStride;
      dest += destStride;
    }                                                         \
  }
}

template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel>& pred, const AreaBuf<const Pel>& resi, const ClpRng& clpRng )
{
  const Pel* src1 = pred.buf;
  const Pel* src2 = resi.buf;
        Pel* dest =      buf;

  const unsigned src1Stride = pred.stride;
  const unsigned src2Stride = resi.stride;
  const unsigned destStride =      stride;
  if( src2Stride == width )
  {
    g_pelBufOP.reco( pred.buf, resi.buf, buf, width * height, clpRng );
  }
  else if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.reco8( src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.reco4( src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng );
  }
  else if( ( width & 1 ) == 0 )
  {
    for( int y = 0; y < height; y++ )
    {
      dest[0] = ClipPel( src1[0] + src2[0], clpRng);
      dest[1] = ClipPel( src1[1] + src2[1], clpRng);
      src1 += src1Stride;
      src2 += src2Stride;
      dest += destStride;
    }                        
  }
  else
  {
    CHECKD( width != 1, "Expecting width to be '1'!" );

    for( int y = 0; y < height; y++ )
    {
      dest[0] = ClipPel( src1[0] + src2[0], clpRng );

      src1 += src1Stride;
      src2 += src2Stride;
      dest += destStride;
    }
  }
}

template<>
void AreaBuf<Pel>::linearTransform( const int scale, const unsigned shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  const Pel* src = buf;
        Pel* dst = buf;

  if( stride == width)
  {
    if( width > 2 && height > 2 )
    {
      g_pelBufOP.linTf8( src, stride<<2, dst, stride<<2, width<<2, height>>2, scale, shift, offset, clpRng, bClip );
    }
    else
    {
      g_pelBufOP.linTf4( src, stride<<1, dst, stride<<1, width<<1, height>>1, scale, shift, offset, clpRng, bClip );
    }
  }
  else if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.linTf8( src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.linTf4( src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip );
  }
  else
  {
    if( bClip )
    {
      for( int y = 0; y < height; y++ )
      {
        dst[0] = ( Pel ) ClipPel( rightShiftU( scale * src[0], shift ) + offset, clpRng );
        dst[1] = ( Pel ) ClipPel( rightShiftU( scale * src[1], shift ) + offset, clpRng );
        src += stride;
        dst += stride;
      }
    }
    else
    {
      for( int y = 0; y < height; y++ )
      {
        dst[0] = ( Pel ) ( rightShiftU( scale * src[0], shift ) + offset );
        dst[1] = ( Pel ) ( rightShiftU( scale * src[1], shift ) + offset );
        src += stride;
        dst += stride;
      }
    }
  }
}

#if ENABLE_SIMD_OPT_BUFFER

template<>
void AreaBuf<Pel>::transposedFrom( const AreaBuf<const Pel>& other )
{
  CHECK( width != other.height || height != other.width, "Incompatible size" );

  if( ( width & 3 ) != 0 || ( height & 3 ) != 0 )
  {
          Pel* dst =       buf;
    const Pel* src = other.buf;
    width          = other.height;
    height         = other.width;
    stride         = stride < width ? width : stride;

    for( unsigned y = 0; y < other.height; y++ )
    {
      for( unsigned x = 0; x < other.width; x++ )
      {
        dst[y + x*stride] = src[x + y * other.stride];
      }
    }
  }
  else if( ( width & 7 ) != 0 || ( height & 7 ) != 0 )
  {
    const Pel* src = other.buf;

    for( unsigned y = 0; y < other.height; y += 4 )
    {
      Pel* dst = buf + y;

      for( unsigned x = 0; x < other.width; x += 4 )
      {
        g_pelBufOP.transpose4x4( &src[x], other.stride, dst, stride );

        dst += 4 * stride;
      }

      src += 4 * other.stride;
    }
  }
  else
  {
    const Pel* src = other.buf;

    for( unsigned y = 0; y < other.height; y += 8 )
    {
      Pel* dst = buf + y;

      for( unsigned x = 0; x < other.width; x += 8 )
      {
        g_pelBufOP.transpose8x8( &src[x], other.stride, dst, stride );

        dst += 8 * stride;
      }

      src += 8 * other.stride;
    }
  }
}
#endif

template<>
void AreaBuf<Pel>::weightCiip( const AreaBuf<const Pel>& intra, const int numIntra )
{
  CHECK(width == 2, "Width of 2 is not supported");
  g_pelBufOP.weightCiip( buf, intra.buf, width * height, numIntra );
}

template<>
void AreaBuf<MotionInfo>::fill( const MotionInfo& val )
{
  if( width == stride )
  {
    std::fill_n( buf, width * height, val );
  }
  else
  {
    MotionInfo* dst = buf;

    for( int y = 0; y < height; y++, dst += stride )
    {
      std::fill_n( dst, width, val );
    }
  }
}

PelStorage::PelStorage()
{
  for( uint32_t i = 0; i < MAX_NUM_COMP; i++ )
  {
    m_origin[i] = nullptr;
  }
}

PelStorage::~PelStorage()
{
  destroy();
}

void PelStorage::create( const UnitArea& _UnitArea )
{
  create( _UnitArea.chromaFormat, _UnitArea.blocks[0] );
  m_maxArea = _UnitArea;
}

void PelStorage::create( const ChromaFormat &_chromaFormat, const Area& _area )
{
  CHECK( !bufs.empty(), "Trying to re-create an already initialized buffer" );

  chromaFormat = _chromaFormat;

  const uint32_t numComp = getNumberValidComponents( _chromaFormat );

  uint32_t bufSize = 0;
  for( uint32_t i = 0; i < numComp; i++ )
  {
    const ComponentID compID = ComponentID( i );
    const unsigned totalWidth  = _area.width  >> getComponentScaleX( compID, _chromaFormat );
    const unsigned totalHeight = _area.height >> getComponentScaleY( compID, _chromaFormat );

    const uint32_t area = totalWidth * totalHeight;
    CHECK( !area, "Trying to create a buffer with zero area" );
    bufSize += area;
  }

  bufSize += 1; // for SIMD DMVR on the bottom right corner, which overreads the lines by 1 sample

  //allocate one buffer
  m_origin[0] = ( Pel* ) xMalloc( Pel, bufSize );

  Pel* topLeft = m_origin[0];
  for( uint32_t i = 0; i < numComp; i++ )
  {
    const ComponentID compID = ComponentID( i );
    const unsigned totalWidth  = _area.width  >> getComponentScaleX( compID, _chromaFormat );
    const unsigned totalHeight = _area.height >> getComponentScaleY( compID, _chromaFormat );
    const uint32_t area = totalWidth * totalHeight;

    bufs.push_back( PelBuf( topLeft, totalWidth, totalWidth, totalHeight ) );
    topLeft += area;
  }

  m_maxArea = UnitArea( _chromaFormat, _area );
}

void PelStorage::create( const ChromaFormat &_chromaFormat, const Area& _area, const unsigned _maxCUSize, const unsigned _margin, const unsigned _alignment, const bool _scaleChromaMargin )
{
  CHECK( !bufs.empty(), "Trying to re-create an already initialized buffer" );

  chromaFormat = _chromaFormat;

  const uint32_t numComp = getNumberValidComponents( _chromaFormat );

  unsigned extHeight = _area.height;
  unsigned extWidth  = _area.width;

  if( _maxCUSize )
  {
    extHeight = ( ( _area.height + _maxCUSize - 1 ) / _maxCUSize ) * _maxCUSize;
    extWidth  = ( ( _area.width  + _maxCUSize - 1 ) / _maxCUSize ) * _maxCUSize;
  }

  for( uint32_t i = 0; i < numComp; i++ )
  {
    const ComponentID compID = ComponentID( i );
    const unsigned scaleX = getComponentScaleX( compID, _chromaFormat );
    const unsigned scaleY = getComponentScaleY( compID, _chromaFormat );

    unsigned scaledHeight = extHeight >> scaleY;
    unsigned scaledWidth  = extWidth  >> scaleX;
    unsigned ymargin      = _margin >> (_scaleChromaMargin?scaleY:0);
    unsigned xmargin      = _margin >> (_scaleChromaMargin?scaleX:0);
    unsigned totalWidth   = scaledWidth + 2*xmargin;
    unsigned totalHeight  = scaledHeight +2*ymargin;

    if( _alignment )
    {
      // make sure buffer lines are align
      CHECK( _alignment != MEMORY_ALIGN_DEF_SIZE, "Unsupported alignment" );
      totalWidth = ( ( totalWidth + _alignment - 1 ) / _alignment ) * _alignment;
    }
    uint32_t area = totalWidth * totalHeight;
    CHECK( !area, "Trying to create a buffer with zero area" );

    m_origin[i] = ( Pel* ) xMalloc( Pel, area );
    Pel* topLeft = m_origin[i] + totalWidth * ymargin + xmargin;
    bufs.push_back( PelBuf( topLeft, totalWidth, _area.width >> scaleX, _area.height >> scaleY ) );
  }

  m_maxArea = UnitArea( _chromaFormat, _area );
}

void PelStorage::createFromBuf( PelUnitBuf buf )
{
  chromaFormat = buf.chromaFormat;

  const uint32_t numCh = getNumberValidComponents( chromaFormat );

  bufs.resize(numCh);

  for( uint32_t i = 0; i < numCh; i++ )
  {
    PelBuf cPelBuf = buf.get( ComponentID( i ) );
    bufs[i] = PelBuf( cPelBuf.bufAt( 0, 0 ), cPelBuf.stride, cPelBuf.width, cPelBuf.height );
  }
}

void PelStorage::compactResize( const UnitArea& area )
{
  CHECK( bufs.size() < area.blocks.size(), "Cannot increase buffer size when compacting!" );

  for( uint32_t i = 0; i < area.blocks.size(); i++ )
  {
    CHECK( m_maxArea.blocks[i].area() < area.blocks[i].area(), "Cannot increase buffer size when compacting!" );

    bufs[i].Size::operator=( area.blocks[i].size() );
    bufs[i].stride = bufs[i].width;
  }
}

void PelStorage::takeOwnership( PelStorage& other )
{
  chromaFormat = other.chromaFormat;

  const uint32_t numCh = getNumberValidComponents( chromaFormat );

  bufs.resize(numCh);

  for( uint32_t i = 0; i < numCh; i++ )
  {
    PelBuf cPelBuf = other.get( ComponentID( i ) );
    bufs[i] = PelBuf( cPelBuf.bufAt( 0, 0 ), cPelBuf.stride, cPelBuf.width, cPelBuf.height );
    std::swap( m_origin[i], other.m_origin[i]);
  }

  m_maxArea = other.m_maxArea;

  other.destroy();
}


void PelStorage::swap( PelStorage& other )
{
  const uint32_t numCh = getNumberValidComponents( chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    // check this otherwise it would turn out to get very weird
    CHECK( chromaFormat                   != other.chromaFormat                  , "Incompatible formats" );
    CHECK( get( ComponentID( i ) )        != other.get( ComponentID( i ) )       , "Incompatible formats" );
    CHECK( get( ComponentID( i ) ).stride != other.get( ComponentID( i ) ).stride, "Incompatible formats" );

    std::swap( bufs[i].buf,    other.bufs[i].buf );
    std::swap( bufs[i].stride, other.bufs[i].stride );
    std::swap( m_origin[i],    other.m_origin[i] );
  }
}

void PelStorage::destroy()
{
  chromaFormat = NUM_CHROMA_FORMAT;
  for( uint32_t i = 0; i < MAX_NUM_COMP; i++ )
  {
    if( m_origin[i] )
    {
      xFree( m_origin[i] );
      m_origin[i] = nullptr;
    }
  }
  bufs.clear();
}

PelBuf PelStorage::getBuf( const ComponentID CompID )
{
  return bufs[CompID];
}

const CPelBuf PelStorage::getBuf( const ComponentID CompID ) const
{
  return bufs[CompID];
}

PelBuf PelStorage::getBuf( const CompArea& blk )
{
  const PelBuf& r = bufs[blk.compID];
  return PelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

const CPelBuf PelStorage::getBuf( const CompArea& blk ) const
{
  const PelBuf& r = bufs[blk.compID];
  return CPelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

PelUnitBuf PelStorage::getBuf( const UnitArea& unit )
{
  return ( chromaFormat == CHROMA_400 ) ? PelUnitBuf( chromaFormat, getBuf( unit.Y() ) ) : PelUnitBuf( chromaFormat, getBuf( unit.Y() ), getBuf( unit.Cb() ), getBuf( unit.Cr() ) );
}

const CPelUnitBuf PelStorage::getBuf( const UnitArea& unit ) const
{
  return ( chromaFormat == CHROMA_400 ) ? CPelUnitBuf( chromaFormat, getBuf( unit.Y() ) ) : CPelUnitBuf( chromaFormat, getBuf( unit.Y() ), getBuf( unit.Cb() ), getBuf( unit.Cr() ) );
}

PelUnitBuf PelStorage::getBuf(const int strY, const int strCb, const int strCr, const UnitArea& unit)
{
  CHECKD( unit.Y().width > bufs[COMP_Y].width && unit.Y().height > bufs[COMP_Y].height, "unsuported request" );
  CHECKD( strY > bufs[COMP_Y].stride, "unsuported request" );
  CHECKD( strCb > bufs[COMP_Cb].stride, "unsuported request" );
  CHECKD( strCr > bufs[COMP_Cr].stride, "unsuported request" );
  return (chromaFormat == CHROMA_400) ? PelUnitBuf(chromaFormat, PelBuf( bufs[COMP_Y].buf, strY, unit.Y())) : PelUnitBuf(chromaFormat, PelBuf( bufs[COMP_Y].buf, strY, unit.Y()), PelBuf( bufs[COMP_Cb].buf, strCb, unit.Cb()), PelBuf( bufs[COMP_Cr].buf, strCr, unit.Cr()));
}

const CPelUnitBuf PelStorage::getBuf(const int strY, const int strCb, const int strCr, const UnitArea& unit) const
{
  CHECKD( unit.Y().width > bufs[COMP_Y].width && unit.Y().height > bufs[COMP_Y].height, "unsuported request" );
  CHECKD( strY > bufs[COMP_Y].stride, "unsuported request" );
  CHECKD( strCb > bufs[COMP_Cb].stride, "unsuported request" );
  CHECKD( strCr > bufs[COMP_Cr].stride, "unsuported request" );
  return (chromaFormat == CHROMA_400) ? CPelUnitBuf(chromaFormat, CPelBuf( bufs[COMP_Y].buf, strY, unit.Y())) : CPelUnitBuf(chromaFormat, CPelBuf( bufs[COMP_Y].buf, strY, unit.Y()), CPelBuf( bufs[COMP_Cb].buf, strCb, unit.Cb()), CPelBuf( bufs[COMP_Cr].buf, strCr, unit.Cr()));
}

PelUnitBuf PelStorage::getBufPart(const UnitArea& unit)
{
  CHECKD( unit.Y().width > bufs[COMP_Y].width && unit.Y().height > bufs[COMP_Y].height, "unsuported request" );
  return (chromaFormat == CHROMA_400) ? PelUnitBuf(chromaFormat, PelBuf( bufs[COMP_Y].buf, bufs[COMP_Y].stride, unit.Y())) : PelUnitBuf(chromaFormat, PelBuf( bufs[COMP_Y].buf, bufs[COMP_Y].stride, unit.Y()), PelBuf( bufs[COMP_Cb].buf, bufs[COMP_Cb].stride, unit.Cb()), PelBuf( bufs[COMP_Cr].buf, bufs[COMP_Cr].stride, unit.Cr()));
}

const CPelUnitBuf PelStorage::getBufPart(const UnitArea& unit) const
{
  CHECKD(unit.Y().width > bufs[COMP_Y].width && unit.Y().height > bufs[COMP_Y].height, "unsuported request");
  return (chromaFormat == CHROMA_400) ? CPelUnitBuf(chromaFormat, CPelBuf(bufs[COMP_Y].buf, unit.Y().width, unit.Y())) : CPelUnitBuf(chromaFormat, CPelBuf(bufs[COMP_Y].buf, unit.Y().width, unit.Y()), CPelBuf(bufs[COMP_Cb].buf, unit.Cb().width, unit.Cb()), CPelBuf(bufs[COMP_Cr].buf, unit.Cr().width, unit.Cr()));
}

const CPelUnitBuf PelStorage::getCompactBuf(const UnitArea& unit) const
{
  CHECKD( unit.Y().width > bufs[COMP_Y].width && unit.Y().height > bufs[COMP_Y].height, "unsuported request" );

  PelUnitBuf ret;
  ret.chromaFormat = chromaFormat;
  ret.bufs.resize_noinit( chromaFormat == CHROMA_400 ? 1 : 3 );
  
  ret.Y   ().buf = bufs[COMP_Y ].buf; ret.Y ().width = ret.Y ().stride = unit.Y ().width; ret.Y ().height = unit.Y ().height;
  if( chromaFormat != CHROMA_400 )
  {
    ret.Cb().buf = bufs[COMP_Cb].buf; ret.Cb().width = ret.Cb().stride = unit.Cb().width; ret.Cb().height = unit.Cb().height;
    ret.Cr().buf = bufs[COMP_Cr].buf; ret.Cr().width = ret.Cr().stride = unit.Cr().width; ret.Cr().height = unit.Cr().height;
  }

  return ret;
}

PelUnitBuf PelStorage::getCompactBuf(const UnitArea& unit)
{
  CHECKD( unit.Y().width > bufs[COMP_Y].width && unit.Y().height > bufs[COMP_Y].height, "unsuported request" );

  PelUnitBuf ret;
  ret.chromaFormat = chromaFormat;
  ret.bufs.resize_noinit( chromaFormat == CHROMA_400 ? 1 : 3 );

  ret.Y   ().buf = bufs[COMP_Y ].buf; ret.Y ().width = ret.Y ().stride = unit.Y ().width; ret.Y ().height = unit.Y ().height;
  if( chromaFormat != CHROMA_400 )
  {
    ret.Cb().buf = bufs[COMP_Cb].buf; ret.Cb().width = ret.Cb().stride = unit.Cb().width; ret.Cb().height = unit.Cb().height;
    ret.Cr().buf = bufs[COMP_Cr].buf; ret.Cr().width = ret.Cr().stride = unit.Cr().width; ret.Cr().height = unit.Cr().height;
  }

  return ret;
}

const CPelBuf PelStorage::getCompactBuf(const CompArea& carea) const
{
  return CPelBuf( bufs[carea.compID].buf, carea.width, carea);
}

PelBuf PelStorage::getCompactBuf(const CompArea& carea)
{
  return PelBuf( bufs[carea.compID].buf, carea.width, carea);
}

void copyPadToPelUnitBuf( PelUnitBuf pelUnitBuf, const vvencYUVBuffer& yuvBuffer, const ChromaFormat& chFmt )
{
  CHECK( pelUnitBuf.bufs.size() == 0, "pelUnitBuf not initialized" );
  pelUnitBuf.chromaFormat = chFmt;
  const int numComp = getNumberValidComponents( chFmt );
  for ( int i = 0; i < numComp; i++ )
  {
    const vvencYUVPlane& src = yuvBuffer.planes[ i ];
    CHECK( src.ptr == nullptr, "yuvBuffer not setup" );
    PelBuf& dest = pelUnitBuf.bufs[i];
    CHECK( dest.buf == nullptr, "yuvBuffer not setup" );

    for( int y = 0; y < src.height; y++ )
    {
      ::memcpy( dest.buf + y*dest.stride, src.ptr + y*src.stride, src.width * sizeof(int16_t) );

      // pad right if required
      for( int x = src.width; x < dest.width; x++ )
      {
        dest.buf[ x + y*dest.stride] = dest.buf[ src.width - 1 + y*dest.stride];
      }
    }

    // pad bottom if required
    for( int y = src.height; y < dest.height; y++ )
    {
      ::memcpy( dest.buf + y*dest.stride, dest.buf + (src.height-1)*dest.stride, dest.width * sizeof(int16_t) );
    }
  }
}
/*
void setupPelUnitBuf( const YUVBuffer& yuvBuffer, PelUnitBuf& pelUnitBuf, const ChromaFormat& chFmt )
{
  CHECK( pelUnitBuf.bufs.size() != 0, "pelUnitBuf already in use" );
  pelUnitBuf.chromaFormat = chFmt;
  const int numComp = getNumberValidComponents( chFmt );
  for ( int i = 0; i < numComp; i++ )
  {
    const YUVBuffer::Plane& yuvPlane = yuvBuffer.planes[ i ];
    CHECK( yuvPlane.ptr == nullptr, "yuvBuffer not setup" );
    PelBuf area( yuvPlane.ptr, yuvPlane.stride, yuvPlane.width, yuvPlane.height );
    pelUnitBuf.bufs.push_back( area );
  }
}
*/
void setupYuvBuffer ( const PelUnitBuf& pelUnitBuf, vvencYUVBuffer& yuvBuffer, const Window* confWindow )
{
  const ChromaFormat chFmt = pelUnitBuf.chromaFormat;
  const int numComp        = getNumberValidComponents( chFmt );
  for ( int i = 0; i < numComp; i++ )
  {
    const ComponentID compId = ComponentID( i );
          PelBuf area        = pelUnitBuf.get( compId );
    const int sx             = getComponentScaleX( compId, chFmt );
    const int sy             = getComponentScaleY( compId, chFmt );
    vvencYUVPlane& yuvPlane = yuvBuffer.planes[ i ];
    CHECK( yuvPlane.ptr != nullptr, "yuvBuffer already in use" );
    yuvPlane.ptr             = area.bufAt( confWindow->winLeftOffset >> sx, confWindow->winTopOffset >> sy );
    yuvPlane.width           = ( ( area.width  << sx ) - ( confWindow->winLeftOffset + confWindow->winRightOffset  ) ) >> sx;
    yuvPlane.height          = ( ( area.height << sy ) - ( confWindow->winTopOffset  + confWindow->winBottomOffset ) ) >> sy;
    yuvPlane.stride          = area.stride;
  }
}

} // namespace vvenc

//! \}


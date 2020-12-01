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

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
/** \file     Buffer.h
 *  \brief    Low-overhead class describing 2D memory layout
 */

#pragma once

#ifndef __IN_UNIT_H__
#error "Include Unit.h not Buffer.h"
#endif

#include "Common.h"
#include "CommonDef.h"
#include "MotionInfo.h"

#include <string.h>
#include <type_traits>
#include <typeinfo>

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ---------------------------------------------------------------------------
// AreaBuf struct
// ---------------------------------------------------------------------------

struct PelBufferOps
{
  PelBufferOps();

  bool isInitX86Done;

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  void initPelBufOpsX86();
  template<X86_VEXT vext>
  void _initPelBufOpsX86();
#endif
  void ( *roundGeo )      ( const Pel* src, Pel* dest, const int numSamples, unsigned rshift, int offset, const ClpRng &clpRng);
  void ( *addAvg )        ( const Pel* src0, const Pel* src1, Pel* dst, int numsamples, unsigned shift, int offset, const ClpRng& clpRng );
  void ( *reco  )         ( const Pel* src0, const Pel* src1, Pel* dst, int numSamples, const ClpRng& clpRng );
  void ( *copyClip )      ( const Pel* src0,                  Pel* dst, int numSamples, const ClpRng& clpRng );
  void ( *addAvg4 )       ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dst, int dstStride, int width, int height,       unsigned shift, int offset, const ClpRng& clpRng );
  void ( *addAvg8 )       ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dst, int dstStride, int width, int height,       unsigned shift, int offset, const ClpRng& clpRng );
  void ( *addAvg16 )      ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dst, int dstStride, int width, int height,       unsigned shift, int offset, const ClpRng& clpRng );
  void ( *copyClip4 )     ( const Pel* src0, int src0Stride,                                  Pel* dst, int dstStride, int width, int height,                                   const ClpRng& clpRng );
  void ( *copyClip8 )     ( const Pel* src0, int src0Stride,                                  Pel* dst, int dstStride, int width, int height,                                   const ClpRng& clpRng );
  void ( *reco4 )         ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dst, int dstStride, int width, int height,                                   const ClpRng& clpRng );
  void ( *reco8 )         ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dst, int dstStride, int width, int height,                                   const ClpRng& clpRng );
  void ( *linTf4 )        ( const Pel* src0, int src0Stride,                                  Pel* dst, int dstStride, int width, int height, int scale, unsigned shift, int offset, const ClpRng& clpRng, bool bClip );
  void ( *linTf8 )        ( const Pel* src0, int src0Stride,                                  Pel* dst, int dstStride, int width, int height, int scale, unsigned shift, int offset, const ClpRng& clpRng, bool bClip );
  void ( *copyBuffer )    ( const char* src, int srcStride, char* dst, int dstStride, int width, int height );
  void ( *padding )       ( Pel* dst, int stride, int width, int height, int padSize);
#if ENABLE_SIMD_OPT_BCW
  void ( *removeHighFreq8)( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height);
  void ( *removeHighFreq4)( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height);
#endif
  void ( *transpose4x4 )  ( const Pel* src,  int srcStride, Pel* dst, int dstStride );
  void ( *transpose8x8 )  ( const Pel* src,  int srcStride, Pel* dst, int dstStride );
  void ( *roundIntVector) ( int* v, int size, unsigned int nShift, const int dmvLimit);
  void ( *mipMatrixMul_4_4)( Pel* res, const Pel* input, const uint8_t* weight, const int maxVal, const int offset, bool transpose );
  void ( *mipMatrixMul_8_4)( Pel* res, const Pel* input, const uint8_t* weight, const int maxVal, const int offset, bool transpose );
  void ( *mipMatrixMul_8_8)( Pel* res, const Pel* input, const uint8_t* weight, const int maxVal, const int offset, bool transpose );
  void ( *weightCiip)     ( Pel* res, const Pel* intra, const int numSamples, int numIntra );
  void ( *applyLut )      ( const Pel* src, const ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride, int width, int height, const Pel* lut );
  void ( *fillPtrMap )    ( void** ptrMap, const ptrdiff_t mapStride, int width, int height, void* val );
};

extern PelBufferOps g_pelBufOP;

template<typename T>
struct AreaBuf : public Size
{
  T*        buf;
  int       stride;
  // the proper type causes awful lot of errors

  AreaBuf()                                                                               : Size(),                  buf( NULL ), stride( 0 )          { }
  AreaBuf( T *_buf, const Size& size )                                                    : Size( size ),            buf( _buf ), stride( size.width ) { }
  AreaBuf( T *_buf, const int& _stride, const Size& size )                                : Size( size ),            buf( _buf ), stride( _stride )    { }
  AreaBuf( T *_buf, const SizeType& _width, const SizeType& _height )                     : Size( _width, _height ), buf( _buf ), stride( _width )     { }
  AreaBuf( T *_buf, const int& _stride, const SizeType& _width, const SizeType& _height ) : Size( _width, _height ), buf( _buf ), stride( _stride )    { }
//  AreaBuf( const AreaBuf<typename std::remove_const<T>::type >& other )                         : Size( other ),           buf( other.buf ), stride( other.stride ) { }

  operator AreaBuf<const T>() const { auto ret = *reinterpret_cast< const AreaBuf<const T>* >( this ); return ret; }

  void fill                 ( const T &val );
  void memset               ( const int val );

  void copyFrom             ( const AreaBuf<const T>& other );
  void reconstruct          ( const AreaBuf<const T>& pred, const AreaBuf<const T>& resi, const ClpRng& clpRng);
  void copyClip             ( const AreaBuf<const T>& src, const ClpRng& clpRng);

  void subtract             ( const AreaBuf<const T>& minuend, const AreaBuf<const T>& subtrahend );
  void extendBorderPel(unsigned marginX, unsigned marginY);

  void addAvg               ( const AreaBuf<const T>& other1, const AreaBuf<const T>& other2, const ClpRng& clpRng );
  T    getAvg               () const;
  void padBorderPel         ( unsigned marginX, unsigned marginY, int dir );
  void addWeightedAvg(const AreaBuf<const T>& other1, const AreaBuf<const T>& other2, const ClpRng& clpRng, const int8_t BcwIdx);
  void removeHighFreq       ( const AreaBuf<const T>& other, const bool bClip, const ClpRng& clpRng);
  void extendBorderPelTop   ( int x, int size, int margin );
  void extendBorderPelBot   ( int x, int size, int margin );
  void extendBorderPelLft   ( int y, int size, int margin );
  void extendBorderPelRgt   ( int y, int size, int margin );

  void linearTransform      ( const int scale, const unsigned shift, const int offset, bool bClip, const ClpRng& clpRng );

  void transposedFrom       ( const AreaBuf<const T>& other );
  void weightCiip           ( const AreaBuf<const T>& intra, const int numIntra );

  void rspSignal            ( const Pel* pLUT );
  void rspSignal            ( const AreaBuf<const T>& other, const Pel* pLUT );
  void scaleSignal          ( const int scale, const bool dir , const ClpRng& clpRng);
  bool compare              ( const AreaBuf<const T>& other ) const;

        T& at( const int& x, const int& y )          { return buf[y * stride + x]; }
  const T& at( const int& x, const int& y ) const    { return buf[y * stride + x]; }

        T& at( const Position& pos )                 { return buf[pos.y * stride + pos.x]; }
  const T& at( const Position& pos ) const           { return buf[pos.y * stride + pos.x]; }


        T* bufAt( const int& x, const int& y )       { return &at( x, y ); }
  const T* bufAt( const int& x, const int& y ) const { return &at( x, y ); }

        T* bufAt( const Position& pos )              { return &at( pos ); }
  const T* bufAt( const Position& pos ) const        { return &at( pos ); }

  AreaBuf<      T> subBuf( const Position& pos, const Size& size )                                    { return AreaBuf<      T>( bufAt( pos  ), stride, size   ); }
  AreaBuf<const T> subBuf( const Position& pos, const Size& size )                              const { return AreaBuf<const T>( bufAt( pos  ), stride, size   ); }
  AreaBuf<      T> subBuf( const int& x, const int& y, const unsigned& _w, const unsigned& _h )       { return AreaBuf<      T>( bufAt( x, y ), stride, _w, _h ); }
  AreaBuf<const T> subBuf( const int& x, const int& y, const unsigned& _w, const unsigned& _h ) const { return AreaBuf<const T>( bufAt( x, y ), stride, _w, _h ); }
};

typedef AreaBuf<      Pel>  PelBuf;
typedef AreaBuf<const Pel> CPelBuf;

typedef AreaBuf<      TCoeff>  CoeffBuf;
typedef AreaBuf<const TCoeff> CCoeffBuf;

typedef AreaBuf<      MotionInfo>  MotionBuf;
typedef AreaBuf<const MotionInfo> CMotionBuf;

typedef AreaBuf<      TCoeff>  PLTescapeBuf;
typedef AreaBuf<const TCoeff> CPLTescapeBuf;

typedef AreaBuf<      bool>  PLTtypeBuf;
typedef AreaBuf<const bool> CPLTtypeBuf;

typedef AreaBuf<      LoopFilterParam>  LFPBuf;
typedef AreaBuf<const LoopFilterParam> CLFPBuf;

#define SIZE_AWARE_PER_EL_OP( OP, INC )                     \
if( ( width & 7 ) == 0 )                                    \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 8 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
      OP( x + 2 );                                          \
      OP( x + 3 );                                          \
      OP( x + 4 );                                          \
      OP( x + 5 );                                          \
      OP( x + 6 );                                          \
      OP( x + 7 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else if( ( width & 3 ) == 0 )                               \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 4 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
      OP( x + 2 );                                          \
      OP( x + 3 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else if( ( width & 1 ) == 0 )                               \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 2 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else                                                        \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x++ )                        \
    {                                                       \
      OP( x );                                              \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}


template<typename T>
T AreaBuf <T> ::getAvg() const
{
  const T* src = buf;
  int64_t  acc = 0;

#define AVG_INC      src += stride
#define AVG_OP(ADDR) acc += src[ADDR]
  SIZE_AWARE_PER_EL_OP(AVG_OP, AVG_INC);
#undef AVG_INC
#undef AVG_OP

  return T ((acc + (area() >> 1)) / area());
}

template<>
void AreaBuf<MotionInfo>::fill( const MotionInfo& val );

template<typename T>
void AreaBuf<T>::fill(const T &val)
{
  if( T( 0 ) == val )
  {
    GCC_WARNING_DISABLE_class_memaccess
    if( width == stride )
    {
      ::memset( buf, 0, width * height * sizeof( T ) );
    }
    else
    {
      T* dest = buf;
      size_t line = width * sizeof( T );

      for( unsigned y = 0; y < height; y++ )
      {
        ::memset( dest, 0, line );

        dest += stride;
      }
    }
    GCC_WARNING_RESET
  }
  else
  {
    if( width == stride )
    {
      std::fill_n( buf, width * height, val );
    }
    else
    {
      T* dest = buf;

      for( int y = 0; y < height; y++, dest += stride )
      {
        std::fill_n( dest, width, val );
      }
    }
  }
}

template<typename T>
void AreaBuf<T>::memset( const int val )
{
  GCC_WARNING_DISABLE_class_memaccess
  if( width == stride )
  {
    ::memset( buf, val, width * height * sizeof( T ) );
  }
  else
  {
    T* dest = buf;
    size_t line = width * sizeof( T );

    for( int y = 0; y < height; y++, dest += stride )
    {
      ::memset( dest, val, line );
    }
  }
  GCC_WARNING_RESET
}

template<typename T>
void AreaBuf<T>::copyFrom( const AreaBuf<const T>& other )
{
#if !defined(__GNUC__) || __GNUC__ > 5
  static_assert( std::is_trivially_copyable<T>::value, "Type T is not trivially_copyable" );
#endif

  g_pelBufOP.copyBuffer( ( const char* ) other.buf, other.stride * sizeof( T ), ( char* ) buf, stride * sizeof( T ), width * sizeof( T ), height );
}

template<typename T>
void AreaBuf<T>::subtract( const AreaBuf<const T>& minuend, const AreaBuf<const T>& subtrahend )
{
  CHECK( width  != minuend.width,  "Incompatible size" );
  CHECK( height != minuend.height, "Incompatible size" );
  CHECK( width  != subtrahend.width, "Incompatible size");
  CHECK( height != subtrahend.height, "Incompatible size");

        T* dest =       buf;
  const T* mins = minuend.buf;
  const T* subs = subtrahend.buf;

#define SUBS_INC          \
  dest +=       stride;    \
  mins += minuend.stride;   \
  subs += subtrahend.stride; \

#define SUBS_OP( ADDR ) dest[ADDR] = mins[ADDR] - subs[ADDR]

  SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
}


template<typename T>
void AreaBuf<T>::copyClip( const AreaBuf<const T>& src, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::copyClip( const AreaBuf<const Pel>& src, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::reconstruct( const AreaBuf<const T>& pred, const AreaBuf<const T>& resi, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel>& pred, const AreaBuf<const Pel>& resi, const ClpRng& clpRng );

template<>
void AreaBuf<Pel>::rspSignal( const AreaBuf<const Pel>& other, const Pel* pLUT);

template<typename T>
void AreaBuf<T>::rspSignal( const AreaBuf<const T>& other, const Pel* pLUT)
{
  THROW( "Type not supported" );
}


template<typename T>
void AreaBuf<T>::addAvg( const AreaBuf<const T>& other1, const AreaBuf<const T>& other2, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel>& other1, const AreaBuf<const Pel>& other2, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::linearTransform( const int scale, const unsigned shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::linearTransform( const int scale, const unsigned shift, const int offset, bool bClip, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::removeHighFreq( const AreaBuf<const T>& other, const bool bClip, const ClpRng& clpRng )
{
  const T*  src       = other.buf;
  const int srcStride = other.stride;

        T*  dst       = buf;
  const int dstStride = stride;

#if ENABLE_SIMD_OPT_BCW
  if (!bClip)
  {
    if(!(width & 7))
      g_pelBufOP.removeHighFreq8(dst, dstStride, src, srcStride, width, height);
    else if (!(width & 3))
      g_pelBufOP.removeHighFreq4(dst, dstStride, src, srcStride, width, height);
    else
      CHECK(true, "Not supported");
  }
  else
  {
#endif

#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride; \

#define REM_HF_OP_CLIP( ADDR ) dst[ADDR] = ClipPel<T>( 2 * dst[ADDR] - src[ADDR], clpRng )
#define REM_HF_OP( ADDR )      dst[ADDR] =             2 * dst[ADDR] - src[ADDR]

  if( bClip )
  {
    SIZE_AWARE_PER_EL_OP( REM_HF_OP_CLIP, REM_HF_INC );
  }
  else
  {
    SIZE_AWARE_PER_EL_OP( REM_HF_OP,      REM_HF_INC );
  }

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP

#if ENABLE_SIMD_OPT_BCW
  }
#endif
}


template<typename T>
void AreaBuf<T>::extendBorderPelLft( int y, int size, int margin )
{
  T* p = buf + y * stride;
  int h = size + y;

  // do left and right margins
  for (; y < h; y++)
  {
    for (int x = 0; x < margin; x++)
    {
      *(p - margin + x) = p[0];
    }
    p += stride;
  }
}

template<typename T>
void AreaBuf<T>::extendBorderPelTop( int x, int size, int margin )
{
  T* p = buf + x;

  // p is now (-marginX, 0)
  for (int y = 0; y < margin; y++)
  {
    ::memcpy(p - (y + 1) * stride, p, sizeof(T) * size);
  }
}

template<typename T>
void AreaBuf<T>::extendBorderPelRgt( int y, int size, int margin )
{
  T* p = buf + y * stride;
  int h = size + y;

  // do left and right margins
  for( ; y < h; y++)
  {
    for (int x = 0; x < margin; x++)
    {
      p[width + x] = p[width - 1];
    }
    p += stride;
  }
}

template<typename T>
void AreaBuf<T>::extendBorderPelBot( int x, int size, int margin )
{
  T* p = buf + (height-1)*stride + x;

  // p is now the (-margin, height-1)
  for (int y = 0; y < margin; y++)
  {
    ::memcpy(p + (y + 1) * stride, p, sizeof(T) * size);
  }
}

template<typename T>
void AreaBuf<T>::extendBorderPel(unsigned marginX, unsigned marginY)
{
  T* p = buf;
  int h = height;
  int w = width;
  int s = stride;

  CHECK((w + 2 * marginX) > s, "Size of buffer too small to extend");
  // do left and right margins
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < marginX; x++)
    {
      *(p - marginX + x) = p[0];
      p[w + x] = p[w - 1];
    }
    p += s;
  }

  // p is now the (0,height) (bottom left of image within bigger picture
  p -= (s + marginX);
  // p is now the (-margin, height-1)
  for (int y = 0; y < marginY; y++)
  {
    ::memcpy(p + (y + 1) * s, p, sizeof(T) * (w + (marginX << 1)));
  }

  // p is still (-marginX, height-1)
  p -= ((h - 1) * s);
  // p is now (-marginX, 0)
  for (int y = 0; y < marginY; y++)
  {
    ::memcpy(p - (y + 1) * s, p, sizeof(T) * (w + (marginX << 1)));
  }
}

template<typename T>
void AreaBuf<T>::padBorderPel( unsigned marginX, unsigned marginY, int dir )
{
  T*  p = buf;
  int s = stride;
  int h = height;
  int w = width;

  CHECK( w  > s, "Size of buffer too small to extend" );

  // top-left margin
  if ( dir == 1 )
  {
    for( int y = 0; y < marginY; y++ )
    {
      for( int x = 0; x < marginX; x++ )
      {
        p[x] = p[marginX];
      }
      p += s;
    }
  }

  // bottom-right margin
  if ( dir == 2 )
  {
    p = buf + s * ( h - marginY ) + w - marginX;

    for( int y = 0; y < marginY; y++ )
    {
      for( int x = 0; x < marginX; x++ )
      {
        p[x] = p[-1];
      }
      p += s;
    }
  }
}


#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<> void AreaBuf<Pel>::transposedFrom( const AreaBuf<const Pel>& other );
#endif

template<typename T>
void AreaBuf<T>::transposedFrom( const AreaBuf<const T>& other )
{
  CHECK( width * height != other.width * other.height, "Incompatible size" );

        T* dst  =       buf;
  const T* src  = other.buf;
  width         = other.height;
  height        = other.width;
  stride        = stride < width ? width : stride;

  for( unsigned y = 0; y < other.height; y++ )
  {
    for( unsigned x = 0; x < other.width; x++ )
    {
      dst[y + x*stride] = src[x + y*other.stride];
    }
  }
}

template<typename T>
bool AreaBuf <T> ::compare( const AreaBuf<const T>& other ) const
{
        T* mine   =       buf;
  const T* theirs = other.buf;
  if( width != other.width) return false;
  if( height != other.height) return false;

  for( unsigned y = 0; y < other.height; y++ )
  {
    for( unsigned x = 0; x < other.width; x++ )
    {
      if( mine[x + y*stride] != theirs[x+y*other.stride])
      {
        return false;
      }
    }
  }
  return true;
}

#ifndef DONT_UNDEF_SIZE_AWARE_PER_EL_OP
#undef SIZE_AWARE_PER_EL_OP
#endif // !DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// ---------------------------------------------------------------------------
// UnitBuf struct
// ---------------------------------------------------------------------------

struct UnitArea;

template<typename T>
struct UnitBuf
{
  typedef static_vector<AreaBuf<T>,       MAX_NUM_COMP> UnitBufBuffers;
  typedef static_vector<AreaBuf<const T>, MAX_NUM_COMP> ConstUnitBufBuffers;

  ChromaFormat chromaFormat;
  UnitBufBuffers bufs;

  UnitBuf() : chromaFormat( NUM_CHROMA_FORMAT ) { }
  UnitBuf( const ChromaFormat _chromaFormat, const UnitBufBuffers&  _bufs ) : chromaFormat( _chromaFormat ), bufs( _bufs ) { }
  UnitBuf( const ChromaFormat _chromaFormat,       UnitBufBuffers&& _bufs ) : chromaFormat( _chromaFormat ), bufs( std::forward<UnitBufBuffers>( _bufs ) ) { }
  UnitBuf( const ChromaFormat _chromaFormat, const AreaBuf<T>&  blkY )      : chromaFormat( _chromaFormat ), bufs{ blkY } { }
  UnitBuf( const ChromaFormat _chromaFormat,       AreaBuf<T>&& blkY )      : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY) } { }
  UnitBuf( const ChromaFormat _chromaFormat, const AreaBuf<T>&  blkY, const AreaBuf<T>&  blkCb, const AreaBuf<T>  &blkCr ) : chromaFormat( _chromaFormat ), bufs{ blkY, blkCb, blkCr } { }
  UnitBuf( const ChromaFormat _chromaFormat,       AreaBuf<T>&& blkY,       AreaBuf<T>&& blkCb,       AreaBuf<T> &&blkCr ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY), std::forward<AreaBuf<T> >(blkCb), std::forward<AreaBuf<T> >(blkCr) } { }

  operator UnitBuf<const T>() const
  {
    return UnitBuf<const T>( chromaFormat, ConstUnitBufBuffers( bufs.begin(), bufs.end() ) );
  }

        AreaBuf<T>& get( const ComponentID comp )        { return bufs[comp]; }
  const AreaBuf<T>& get( const ComponentID comp )  const { return bufs[comp]; }

        AreaBuf<T>& Y()        { return bufs[0]; }
  const AreaBuf<T>& Y()  const { return bufs[0]; }
        AreaBuf<T>& Cb()       { return bufs[1]; }
  const AreaBuf<T>& Cb() const { return bufs[1]; }
        AreaBuf<T>& Cr()       { return bufs[2]; }
  const AreaBuf<T>& Cr() const { return bufs[2]; }
  bool valid          () const { return bufs.size() != 0; }

  void fill                 ( const T& val );
  void copyFrom             ( const UnitBuf<const T> &other, const bool lumaOnly = false, const bool chromaOnly = false );
  void reconstruct          ( const UnitBuf<const T>& pred, const UnitBuf<const T>& resi, const ClpRngs& clpRngs );
  void copyClip             ( const UnitBuf<const T> &src, const ClpRngs& clpRngs, const bool lumaOnly = false, const bool chromaOnly = false );
  void subtract             ( const UnitBuf<const T>& minuend, const UnitBuf<const T>& subtrahend );
  void addAvg               ( const UnitBuf<const T>& other1, const UnitBuf<const T>& other2, const ClpRngs& clpRngs, const bool chromaOnly = false, const bool lumaOnly = false);
  void addWeightedAvg       ( const UnitBuf<const T>& other1, const UnitBuf<const T>& other2, const ClpRngs& clpRngs, const uint8_t BcwIdx = BCW_DEFAULT, const bool chromaOnly = false, const bool lumaOnly = false);
  void padBorderPel         ( unsigned margin, int dir );
  void extendBorderPel      ( unsigned margin, bool scale = false);
  void extendBorderPelTop   ( int x, int size, int margin );
  void extendBorderPelBot   ( int x, int size, int margin );
  void extendBorderPelLft   ( int y, int size, int margin );
  void extendBorderPelRgt   ( int y, int size, int margin );

  void removeHighFreq       ( const UnitBuf<const T>& other, const bool bClip, const ClpRngs& clpRngs);

        UnitBuf<      T> subBuf (const UnitArea& subArea);
  const UnitBuf<const T> subBuf (const UnitArea& subArea) const;
};

typedef UnitBuf<      Pel>  PelUnitBuf;
typedef UnitBuf<const Pel> CPelUnitBuf;

typedef UnitBuf<      TCoeff>  CoeffUnitBuf;
typedef UnitBuf<const TCoeff> CCoeffUnitBuf;

template<typename T>
void UnitBuf<T>::fill( const T &val )
{
  for( size_t i = 0; i < bufs.size(); i++ )
  {
    bufs[i].fill( val );
  }
}

template<typename T>
void UnitBuf<T>::copyFrom(const UnitBuf<const T> &other, const bool lumaOnly, const bool chromaOnly)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  CHECK(lumaOnly && chromaOnly, "Not allowed to have both lumaOnly and chromaOnly selected");
  const size_t compStart = chromaOnly ? 1 : 0;
  const size_t compEnd = lumaOnly ? 1 : (unsigned)bufs.size();
  for(size_t i = compStart; i < compEnd; i++)
  {
    bufs[i].copyFrom( other.bufs[i] );
  }
}

template<typename T>
void UnitBuf<T>::subtract( const UnitBuf<const T>& minuend, const UnitBuf<const T>& subtrahend )
{
  CHECK( chromaFormat != minuend.chromaFormat, "Incompatible formats" );
  CHECK( chromaFormat != subtrahend.chromaFormat, "Incompatible formats");

  for( size_t i = 0; i < bufs.size(); i++ )
  {
    bufs[i].subtract( minuend.bufs[i], subtrahend.bufs[i] );
  }
}

template<typename T>
void UnitBuf<T>::copyClip(const UnitBuf<const T> &src, const ClpRngs &clpRngs, const bool lumaOnly, const bool chromaOnly)
{
  CHECK( chromaFormat != src.chromaFormat, "Incompatible formats" );

  CHECK(lumaOnly && chromaOnly, "Not allowed to have both lumaOnly and chromaOnly selected");
  const size_t compStart = chromaOnly ? 1 : 0;
  const size_t compEnd = lumaOnly ? 1 : bufs.size();
  for (size_t i = compStart; i < compEnd; i++)
  {
    bufs[i].copyClip( src.bufs[i], clpRngs.comp[i] );
  }
}

template<typename T>
void UnitBuf<T>::reconstruct(const UnitBuf<const T>& pred, const UnitBuf<const T>& resi, const ClpRngs& clpRngs)
{
  CHECK( chromaFormat != pred.chromaFormat, "Incompatible formats" );
  CHECK( chromaFormat != resi.chromaFormat, "Incompatible formats" );

  for( size_t i = 0; i < bufs.size(); i++ )
  {
    bufs[i].reconstruct( pred.bufs[i], resi.bufs[i], clpRngs.comp[i] );
  }
}

template<typename T>
void UnitBuf<T>::addAvg(const UnitBuf<const T>& other1, const UnitBuf<const T>& other2, const ClpRngs& clpRngs, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend   = lumaOnly   ? 1 : bufs.size();

  CHECK( lumaOnly && chromaOnly, "should not happen" );

  for( size_t i = istart; i < iend; i++)
  {
    bufs[i].addAvg( other1.bufs[i], other2.bufs[i], clpRngs.comp[i]);
  }
}

template<typename T>
void UnitBuf<T>::addWeightedAvg(const UnitBuf<const T>& other1, const UnitBuf<const T>& other2, const ClpRngs& clpRngs, const uint8_t BcwIdx /* = BCW_DEFAULT */, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend = lumaOnly ? 1 : bufs.size();

  CHECK(lumaOnly && chromaOnly, "should not happen");

  for (size_t i = istart; i < iend; i++)
  {
    bufs[i].addWeightedAvg(other1.bufs[i], other2.bufs[i], clpRngs.comp[i], BcwIdx);
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPelTop   ( int x, int size, int margin )
{
  for( size_t i = 0; i < bufs.size(); i++ )
  {
    int csx = getComponentScaleX(ComponentID(i), chromaFormat);
    int csy = getComponentScaleY(ComponentID(i), chromaFormat);
    bufs[i].extendBorderPelTop( x>>csx, size>>csx, margin>>csy );
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPelBot   ( int x, int size, int margin )
{
  for( size_t i = 0; i < bufs.size(); i++ )
  {
    int csx = getComponentScaleX(ComponentID(i), chromaFormat);
    int csy = getComponentScaleY(ComponentID(i), chromaFormat);
    bufs[i].extendBorderPelBot( x>>csx, size>>csx, margin>>csy );
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPelLft   ( int y, int size, int margin )
{
  for( size_t i = 0; i < bufs.size(); i++ )
  {
    int csx = getComponentScaleX(ComponentID(i), chromaFormat);
    int csy = getComponentScaleY(ComponentID(i), chromaFormat);
    bufs[i].extendBorderPelLft( y>>csy, size>>csy, margin>>csx );
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPelRgt   ( int y, int size, int margin )
{
  for( size_t i = 0; i < bufs.size(); i++ )
  {
    int csx = getComponentScaleX(ComponentID(i), chromaFormat);
    int csy = getComponentScaleY(ComponentID(i), chromaFormat);
    bufs[i].extendBorderPelRgt( y>>csy, size>>csy, margin>>csx );
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPel( unsigned margin, bool scaleMargin )
{
  if( ! scaleMargin )
  {
    for( size_t i = 0; i < bufs.size(); i++ )
    {
      bufs[i].extendBorderPel( margin, margin );
    }
  }
  else
  {
    for (unsigned i = 0; i < bufs.size(); i++)
    {
      bufs[i].extendBorderPel(margin >> getComponentScaleX(ComponentID(i), chromaFormat), margin >> getComponentScaleY(ComponentID(i), chromaFormat));
    }
  }
}

template<typename T>
void UnitBuf<T>::padBorderPel( unsigned margin, int dir )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].padBorderPel( margin >> getComponentScaleX( ComponentID( i ), chromaFormat ), margin >> getComponentScaleY( ComponentID( i ), chromaFormat ), dir );
  }
}

template<typename T>
void UnitBuf<T>::removeHighFreq( const UnitBuf<const T>& other, const bool bClip, const ClpRngs& clpRngs)
{
  bufs[0].removeHighFreq(other.bufs[0], bClip, clpRngs.comp[0]);
}

template<typename T>
UnitBuf<T> UnitBuf<T>::subBuf( const UnitArea& subArea )
{
  UnitBuf<T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( auto &subAreaBuf : bufs )
  {
    subBuf.bufs.push_back( subAreaBuf.subBuf( subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size() ) );
    blockIdx++;
  }

  return subBuf;
}


template<typename T>
const UnitBuf<const T> UnitBuf<T>::subBuf( const UnitArea& subArea ) const
{
  UnitBuf<const T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( const auto &subAreaBuf : bufs )
  {
    subBuf.bufs.push_back( subAreaBuf.subBuf( subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size() ) );
    blockIdx++;
  }

  return subBuf;
}

// ---------------------------------------------------------------------------
// PelStorage struct (PelUnitBuf which allocates its own memory)
// ---------------------------------------------------------------------------

struct UnitArea;
struct CompArea;

struct PelStorage : public PelUnitBuf
{
  PelStorage();
  ~PelStorage();

  void swap( PelStorage& other );
  void createFromBuf( PelUnitBuf buf );
  void takeOwnership( PelStorage& other );
  void create( const UnitArea& _unit );
  void create( const ChromaFormat &_chromaFormat, const Area& _area );
  void create( const ChromaFormat &_chromaFormat, const Area& _area, const unsigned _maxCUSize, const unsigned _margin = 0, const unsigned _alignment = 0, const bool _scaleChromaMargin = true );
  void destroy();

         PelBuf getBuf( const CompArea& blk );
  const CPelBuf getBuf( const CompArea& blk ) const;

         PelBuf getBuf( const ComponentID CompID );
  const CPelBuf getBuf( const ComponentID CompID ) const;

         PelUnitBuf getBuf( const UnitArea& unit );
  const CPelUnitBuf getBuf( const UnitArea& unit ) const;
               Pel* getOrigin( const int id ) const { return m_origin[id]; }

         PelUnitBuf getBuf(const int strY, const int strCb, const int strCr, const UnitArea& unit);
  const CPelUnitBuf getBuf(const int strY, const int strCb, const int strCr, const UnitArea& unit) const;

         PelUnitBuf getBufPart( const UnitArea& unit );
  const CPelUnitBuf getBufPart( const UnitArea& unit ) const;

         PelUnitBuf getCompactBuf(const UnitArea& unit);
  const CPelUnitBuf getCompactBuf(const UnitArea& unit) const;

         PelBuf     getCompactBuf(const CompArea& blk);
  const CPelBuf     getCompactBuf(const CompArea& blk) const;

private:

  Pel* m_origin[MAX_NUM_COMP];
};

struct CompStorage : public PelBuf
{
  CompStorage () { m_memory = nullptr; }
  ~CompStorage() { if (valid()) delete [] m_memory; }

  void create( const Size& size )
  {
    CHECK( m_memory, "Trying to re-create an already initialized buffer" );
    m_memory = new Pel [ size.area() ];
    *static_cast<PelBuf*>(this) = PelBuf( m_memory, size );
  }
  void destroy()
  {
    if (valid()) delete [] m_memory;
    m_memory = nullptr;
  }
  bool valid() { return m_memory != nullptr; }
private:
  Pel* m_memory;
};

template<int NumEntries>
struct SortedPelUnitBufs
{
  void create(ChromaFormat cform, int maxWidth, int maxHeight)
  {
    m_pacBufs.resize(NumEntries + 1);
    m_acStorage.resize(NumEntries + 1);
    for (size_t i = 0; i <= NumEntries; i++)
    {
      m_acStorage[i].create(cform, Area(0, 0, maxWidth, maxHeight));
    }
  }

  void destroy()
  {
    for (size_t i = 0; i <= NumEntries; i++)
    {
      m_acStorage[i].destroy();
    }
  }

  void reset()
  {
    m_sortedList.clear();
  }

  void reduceTo(int numModes)
  {
    CHECK( numModes > m_sortedList.size(), "not enough buffers");
    m_sortedList.resize(numModes);
  }

  void prepare( const UnitArea& ua, int numModes)
  {
    CHECK( numModes > NumEntries, "not enough buffers");
    m_sortedList.resize(numModes);
    for (size_t i = 0; i < numModes; i++)
    {
      m_pacBufs[i] = m_acStorage[i].getCompactBuf(ua);
      m_sortedList[i] = &m_pacBufs[i];
    }

    m_pacBufs[numModes] = m_acStorage[numModes].getCompactBuf(ua);
    m_TestBuf = &m_pacBufs[numModes];
  }

  PelUnitBuf* getBufFromSortedList( int idx)              const { return m_sortedList.size() > idx ? m_sortedList[idx]: nullptr; }
  PelBuf&     getTestBuf          ( ComponentID compId)   const { return m_TestBuf->bufs[compId]; }
  PelUnitBuf& getTestBuf          ()                      const { return *m_TestBuf; }

  void swap( unsigned pos1, unsigned pos2 )
  {
    CHECK( pos1 >= m_sortedList.size(), "index out of range" );
    CHECK( pos2 >= m_sortedList.size(), "index out of range" );
    std::swap(m_sortedList[pos1], m_sortedList[pos2]);
  }

  void insert(int insertPos, int RdListSize)
  {
    if (insertPos != -1)
    {
      for (int i = RdListSize - 1; i > insertPos; i--)
      {
        std::swap(m_sortedList[i - 1], m_sortedList[i]);
      }
      std::swap(m_TestBuf, m_sortedList[insertPos]);
    }
  }

private:
  PelUnitBuf*                             m_TestBuf;
  static_vector<PelUnitBuf*,NumEntries>   m_sortedList;
  static_vector<PelUnitBuf, NumEntries+1> m_pacBufs;
  static_vector<PelStorage, NumEntries+1> m_acStorage;
};

struct YUVBuffer;
struct Window;

void setupPelUnitBuf( const YUVBuffer& yuvBuffer, PelUnitBuf& pelUnitBuf, const ChromaFormat& chFmt );
void setupYuvBuffer ( const PelUnitBuf& pelUnitBuf, YUVBuffer& yuvBuffer, const Window* confWindow );

} // namespace vvenc

//! \}


/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     MCTF.cpp
\brief    MCTF class
*/

#include "MCTF.h"
#include <math.h>
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_buffer.h"
#include "Utilities/NoMallocThreadPool.h"

namespace vvenc {

#ifdef TRACE_ENABLE_ITT
static __itt_string_handle* itt_handle_est = __itt_string_handle_create( "MCTF_est" );
static __itt_domain* itt_domain_MCTF_est   = __itt_domain_create( "MCTFEst" );
static __itt_string_handle* itt_handle_flt = __itt_string_handle_create( "MCTF_flt" );
static __itt_domain* itt_domain_MCTF_flt   = __itt_domain_create( "MCTFFlt" );
#endif

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

const double MCTF::m_chromaFactor     =  0.55;
const double MCTF::m_sigmaMultiplier  =  9.0;
const double MCTF::m_sigmaZeroPoint   = 10.0;
const int MCTF::m_range               = VVENC_MCTF_RANGE;
const int MCTF::m_motionVectorFactor  = 16;
const int MCTF::m_padding             = MCTF_PADDING;
const int16_t MCTF::m_interpolationFilter8[16][8] =
{
  {   0,   0,   0,  64,   0,   0,   0,   0 },   //0
  {   0,   1,  -3,  64,   4,  -2,   0,   0 },   //1 -->-->
  {   0,   1,  -6,  62,   9,  -3,   1,   0 },   //2 -->
  {   0,   2,  -8,  60,  14,  -5,   1,   0 },   //3 -->-->
  {   0,   2,  -9,  57,  19,  -7,   2,   0 },   //4
  {   0,   3, -10,  53,  24,  -8,   2,   0 },   //5 -->-->
  {   0,   3, -11,  50,  29,  -9,   2,   0 },   //6 -->
  {   0,   3, -11,  44,  35, -10,   3,   0 },   //7 -->-->
  {   0,   1,  -7,  38,  38,  -7,   1,   0 },   //8
  {   0,   3, -10,  35,  44, -11,   3,   0 },   //9 -->-->
  {   0,   2,  -9,  29,  50, -11,   3,   0 },   //10-->
  {   0,   2,  -8,  24,  53, -10,   3,   0 },   //11-->-->
  {   0,   2,  -7,  19,  57,  -9,   2,   0 },   //12
  {   0,   1,  -5,  14,  60,  -8,   2,   0 },   //13-->-->
  {   0,   1,  -3,   9,  62,  -6,   1,   0 },   //14-->
  {   0,   0,  -2,   4,  64,  -3,   1,   0 }    //15-->-->
};

const int16_t MCTF::m_interpolationFilter4[16][4] =
{
  {  0, 64,  0,  0 },    //0
  { -2, 62,  4,  0 },    //1 -->-->
  { -2, 58, 10, -2 },    //2 -->
  { -4, 56, 14, -2 },    //3 -->-->
  { -4, 54, 16, -2 },    //4
  { -6, 52, 20, -2 },    //5 -->-->
  { -6, 46, 28, -4 },    //6 -->
  { -4, 42, 30, -4 },    //7 -->-->
  { -4, 36, 36, -4 },    //8
  { -4, 30, 42, -4 },    //9 -->-->
  { -4, 28, 46, -6 },    //10-->
  { -2, 20, 52, -6 },    //11-->-->
  { -2, 16, 54, -4 },    //12
  { -2, 14, 56, -4 },    //13-->-->
  { -2, 10, 58, -2 },    //14-->
  {  0,  4, 62, -2 },    //15-->-->
};

const double MCTF::m_refStrengths[3][4] =
{ // abs(POC offset)
  //  1,    2     3     4
  {0.85, 0.57, 0.41, 0.33},  // m_range * 2
  {1.13, 0.97, 0.81, 0.57},  // m_range
  {0.30, 0.30, 0.30, 0.30}   // otherwise
};

const int    MCTF::m_cuTreeThresh[4] = { 75, 60,     30, 15 };
const double MCTF::m_cuTreeCenter    =           45;

int motionErrorLumaInt( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int besterror )
{
  int error = 0;

  for( int y1 = 0; y1 < h; y1++ )
  {
    const Pel* origRowStart   = org + y1 * origStride;
    const Pel* bufferRowStart = buf + y1 * buffStride;

    for( int x1 = 0; x1 < w; x1 += 2 )
    {
      int diff = origRowStart[x1] - bufferRowStart[x1];
      error += diff * diff;
      diff = origRowStart[x1 + 1] - bufferRowStart[x1 + 1];
      error += diff * diff;
    }
    if( error > besterror )
    {
      return error;
    }
  }

  return error;
}

int motionErrorLumaFrac6( const Pel *org, const ptrdiff_t origStride, const Pel *buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t *xFilter, const int16_t *yFilter, const int bitDepth, const int besterror )
{
  int error = 0;
  Pel tempArray[64 + 8][64];
  int sum, base;
  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

  for( int y1 = 1; y1 < h + 7; y1++ )
  {
    const int yOffset = y1 - 3;
    const Pel *sourceRow = buf + yOffset * buffStride;
    for( int x1 = 0; x1 < w; x1++ )
    {
      sum = 0;
      base = x1 - 3;
      const Pel *rowStart = sourceRow + base;

      sum += xFilter[1] * rowStart[1];
      sum += xFilter[2] * rowStart[2];
      sum += xFilter[3] * rowStart[3];
      sum += xFilter[4] * rowStart[4];
      sum += xFilter[5] * rowStart[5];
      sum += xFilter[6] * rowStart[6];

      sum = ( sum + ( 1 << 5 ) ) >> 6;
      sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );

      tempArray[y1][x1] = sum;
    }
  }

  for( int y1 = 0; y1 < h; y1++ )
  {
    const Pel *origRow = org + y1 * origStride;
    for( int x1 = 0; x1 < w; x1++ )
    {
      sum = 0;
      sum += yFilter[1] * tempArray[y1 + 1][x1];
      sum += yFilter[2] * tempArray[y1 + 2][x1];
      sum += yFilter[3] * tempArray[y1 + 3][x1];
      sum += yFilter[4] * tempArray[y1 + 4][x1];
      sum += yFilter[5] * tempArray[y1 + 5][x1];
      sum += yFilter[6] * tempArray[y1 + 6][x1];

      sum = ( sum + ( 1 << 5 ) ) >> 6;
      sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );

      error += ( sum - origRow[x1] ) * ( sum - origRow[x1] );
    }
    if( error > besterror )
    {
      return error;
    }
  }

  return error;
}

int motionErrorLumaFrac4( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror )
{
  int error = 0;
  Pel tempArray[64 + 4][64];
  int sum, base;
  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

  for( int y1 = 0; y1 < h + 3; y1++ )
  {
    const int yOffset = y1 - 1;
    const Pel* sourceRow = buf + yOffset * buffStride;
    for( int x1 = 0; x1 < w; x1++ )
    {
      sum = 0;
      base = x1 - 1;
      const Pel* rowStart = sourceRow + base;

      sum += xFilter[0] * rowStart[0];
      sum += xFilter[1] * rowStart[1];
      sum += xFilter[2] * rowStart[2];
      sum += xFilter[3] * rowStart[3];

      sum = ( sum + ( 1 << 5 ) ) >> 6;
      sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );

      tempArray[y1][x1] = sum;
    }
  }

  for( int y1 = 0; y1 < h; y1++ )
  {
    const Pel* origRow = org + y1 * origStride;
    for( int x1 = 0; x1 < w; x1++ )
    {
      sum = 0;
      sum += yFilter[0] * tempArray[y1 + 0][x1];
      sum += yFilter[1] * tempArray[y1 + 1][x1];
      sum += yFilter[2] * tempArray[y1 + 2][x1];
      sum += yFilter[3] * tempArray[y1 + 3][x1];

      sum = ( sum + ( 1 << 5 ) ) >> 6;
      sum = sum < 0 ? 0 : ( sum > maxSampleValue ? maxSampleValue : sum );

      error += ( sum - origRow[x1] ) * ( sum - origRow[x1] );
    }
    if( error > besterror )
    {
      return error;
    }
  }

  return error;
}

void applyFrac8Core_6Tap( const Pel* org, const ptrdiff_t origStride, Pel* dst, const ptrdiff_t dstStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  const int numFilterTaps   = 7;
  const int centreTapOffset = 3;
  const int maxValue        = ( 1 << bitDepth ) - 1;

  Pel tempArray[64 + numFilterTaps][64];

  for( int by = 1; by < h + numFilterTaps - 1; by++ )
  {
    const int yOffset = by - centreTapOffset;
    const Pel *sourceRow = org + yOffset * origStride;
    for( int bx = 0; bx < w; bx++ )
    {
      int base = bx - centreTapOffset;
      const Pel *rowStart = sourceRow + base;

      int sum = 0;
      sum += xFilter[1] * rowStart[1];
      sum += xFilter[2] * rowStart[2];
      sum += xFilter[3] * rowStart[3];
      sum += xFilter[4] * rowStart[4];
      sum += xFilter[5] * rowStart[5];
      sum += xFilter[6] * rowStart[6];

      sum = ( sum + ( 1 << 5 ) ) >> 6;
      tempArray[by][bx] = sum;
    }
  }

  Pel *dstRow = dst;
  for( int by = 0; by < h; by++, dstRow += dstStride )
  {
    Pel *dstPel = dstRow;
    for( int bx = 0; bx < w; bx++, dstPel++ )
    {
      int sum = 0;

      sum += yFilter[1] * tempArray[by + 1][bx];
      sum += yFilter[2] * tempArray[by + 2][bx];
      sum += yFilter[3] * tempArray[by + 3][bx];
      sum += yFilter[4] * tempArray[by + 4][bx];
      sum += yFilter[5] * tempArray[by + 5][bx];
      sum += yFilter[6] * tempArray[by + 6][bx];

      sum = ( sum + ( 1 << 5 ) ) >> 6;
      sum = sum < 0 ? 0 : ( sum > maxValue ? maxValue : sum );
      *dstPel = sum;
    }
  }
}

void applyFrac8Core_4Tap( const Pel* org, const ptrdiff_t origStride, Pel* dst, const ptrdiff_t dstStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth )
{
  const int numFilterTaps   = 3;
  const int centreTapOffset = 1;
  const int maxValue        = ( 1 << bitDepth ) - 1;

  Pel tempArray[64 + numFilterTaps][64];

  for( int by = 0; by < h + numFilterTaps; by++ )
  {
    const int yOffset    = by - centreTapOffset;
    const Pel* sourceRow = org + yOffset * origStride;

    for( int bx = 0; bx < w; bx++ )
    {
      int base = bx - centreTapOffset;
      const Pel* rowStart = sourceRow + base;

      int sum = 0;
      sum += xFilter[0] * rowStart[0];
      sum += xFilter[1] * rowStart[1];
      sum += xFilter[2] * rowStart[2];
      sum += xFilter[3] * rowStart[3];

      sum = ( sum + ( 1 << 5 ) ) >> 6;
      tempArray[by][bx] = sum;
    }
  }

  Pel* dstRow = dst;
  for( int by = 0; by < h; by++, dstRow += dstStride )
  {
    Pel* dstPel = dstRow;
    for( int bx = 0; bx < w; bx++, dstPel++ )
    {
      int sum = 0;
      sum += yFilter[0] * tempArray[by + 0][bx];
      sum += yFilter[1] * tempArray[by + 1][bx];
      sum += yFilter[2] * tempArray[by + 2][bx];
      sum += yFilter[3] * tempArray[by + 3][bx];

      sum = ( sum + ( 1 << 5 ) ) >> 6;
      sum = sum < 0 ? 0 : ( sum > maxValue ? maxValue : sum );
      *dstPel = sum;
    }
  }
}

inline static float fastExp( float n, float d )
{
  // using the e^x ~= ( 1 + x/n )^n for n -> inf
  float x = 1.0f + n / ( d * 1024 );
  x *= x; x *= x; x *= x; x *= x;
  x *= x; x *= x; x *= x; x *= x;
  x *= x; x *= x;
  return x;
}

void applyBlockCore( const CPelBuf& src, PelBuf& dst, const CompArea& blk, const ClpRng& clpRng, const Pel** correctedPics, int numRefs, const int* verror, const double refStrenghts[4], double weightScaling, double sigmaSq )
{
  const int         w = blk.width;
  const int         h = blk.height;
  const int        bx = blk.x;
  const int        by = blk.y;

  const ptrdiff_t srcStride = src.stride;
  const ptrdiff_t dstStride = dst.stride;

  const Pel *srcPel = src.bufAt( bx, by );
        Pel *dstPel = dst.bufAt( bx, by );

  const Pel maxSampleValue = clpRng.max();

  int vnoise[2 * VVENC_MCTF_RANGE] = { 0, };
  float vsw[2 * VVENC_MCTF_RANGE] = { 0.0f, };
  float vww[2 * VVENC_MCTF_RANGE] = { 0.0f, };

  int minError = 9999999;

  for( int i = 0; i < numRefs; i++ )
  {
    int64_t variance = 0, diffsum = 0;
    const ptrdiff_t refStride = w;
    const Pel *     refPel    = correctedPics[i];
    for( int y1 = 0; y1 < h; y1++ )
    {
      for( int x1 = 0; x1 < w; x1++ )
      {
        const Pel pix = *( srcPel + srcStride * y1 + x1 );
        const Pel ref = *( refPel + refStride * y1 + x1 );

        const int diff = pix - ref;
        variance += diff * diff;
        if( x1 != w - 1 )
        {
          const Pel pixR = *( srcPel + srcStride * y1 + x1 + 1 );
          const Pel refR = *( refPel + refStride * y1 + x1 + 1 );
          const int diffR = pixR - refR;
          diffsum += ( diffR - diff ) * ( diffR - diff );
        }
        if( y1 != h - 1 )
        {
          const Pel pixD = *( srcPel + srcStride * y1 + x1 + srcStride );
          const Pel refD = *( refPel + refStride * y1 + x1 + refStride );
          const int diffD = pixD - refD;
          diffsum += ( diffD - diff ) * ( diffD - diff );
        }
      }
    }
    const int cntV = w * h;
    const int cntD = 2 * cntV - w - h;
    vnoise[i] = ( int ) round( ( 15.0 * cntD / cntV * variance + 5.0 ) / ( diffsum + 5.0 ) );
    minError = std::min( minError, verror[i] );
  }

  for( int i = 0; i < numRefs; i++ )
  {
    const int error = verror[i];
    const int noise = vnoise[i];
    float ww = 1, sw = 1;
    ww *= ( noise < 25 ) ? 1.0 : 0.6;
    sw *= ( noise < 25 ) ? 1.0 : 0.8;
    ww *= ( error < 50 ) ? 1.2 : ( ( error > 100 ) ? 0.6 : 1.0 );
    sw *= ( error < 50 ) ? 1.0 : 0.8;
    ww *= ( ( minError + 1.0 ) / ( error + 1.0 ) );

    vww[i] = ww * weightScaling * refStrenghts[i];
    vsw[i] = sw * 2 * sigmaSq;
  }

  for( int y = 0; y < h; y++ )
  {
    for( int x = 0; x < w; x++ )
    {
      const Pel orgVal  = *( srcPel + srcStride * y + x );
      float temporalWeightSum = 1.0;
      float newVal = ( float ) orgVal;

      for( int i = 0; i < numRefs; i++ )
      {
        const Pel* pCorrectedPelPtr = correctedPics[i] + y * w + x;
        const int    refVal = *pCorrectedPelPtr;
        const int    diff   = refVal - orgVal;
        const float  diffSq = diff * diff;

        float weight = vww[i] * fastExp( -diffSq, vsw[i] );
        newVal += weight * refVal;
        temporalWeightSum += weight;
      }
      newVal /= temporalWeightSum;
      Pel sampleVal = ( Pel ) ( newVal + 0.5 );
      sampleVal = ( sampleVal < 0 ? 0 : ( sampleVal > maxSampleValue ? maxSampleValue : sampleVal ) );
      *( dstPel + dstStride * y + x ) = sampleVal;
    }
  }
}

double calcVarCore( const Pel* org, const ptrdiff_t origStride, const int w, const int h )
{
  // calculate average
  int avg = 0;
  for( int y1 = 0; y1 < h; y1++ )
  {
    for( int x1 = 0; x1 < w; x1++ )
    {
      avg = avg + *( org + x1 + y1 * origStride );
    }
  }
  avg <<= 4;
  avg = avg / ( w * h );

  // calculate variance
  int64_t variance = 0;
  for( int y1 = 0; y1 < h; y1++ )
  {
    for( int x1 = 0; x1 < w; x1++ )
    {
      int pix = *( org + x1 + y1 * origStride ) << 4;
      variance = variance + ( pix - avg ) * ( pix - avg );
    }
  }

  return variance / 256.0;
}

MCTF::MCTF()
  : m_encCfg    ( nullptr )
  , m_threadPool( nullptr )
  , m_filterPoc ( 0 )
  , m_lastPicIn ( nullptr )
{
  m_motionErrorLumaIntX     = motionErrorLumaInt;
  m_motionErrorLumaInt8     = motionErrorLumaInt;
  m_motionErrorLumaFracX[0] = motionErrorLumaFrac6;
  m_motionErrorLumaFrac8[0] = motionErrorLumaFrac6;
  m_motionErrorLumaFracX[1] = motionErrorLumaFrac4;
  m_motionErrorLumaFrac8[1] = motionErrorLumaFrac4;
  m_applyFrac[0][0]         = applyFrac8Core_6Tap;
  m_applyFrac[0][1]         = applyFrac8Core_4Tap;
  m_applyFrac[1][0]         = applyFrac8Core_6Tap;
  m_applyFrac[1][1]         = applyFrac8Core_4Tap;
  m_applyBlock              = applyBlockCore;
  m_calcVar                 = calcVarCore;

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_MCTF
  initMCTF_X86();
#endif
}

MCTF::~MCTF()
{
}

void MCTF::init( const VVEncCfg& encCfg, NoMallocThreadPool* threadPool )
{
  CHECK( encCfg.m_vvencMCTF.numFrames != encCfg.m_vvencMCTF.numStrength, "should have been checked before" );

  m_encCfg     = &encCfg;
  m_threadPool = threadPool;
  m_area       = Area( 0, 0, m_encCfg->m_PadSourceWidth, m_encCfg->m_PadSourceHeight );
  m_filterPoc  = 0;

  // TLayer (TL) dependent definition of drop frames: TL = 4,  TL = 3,  TL = 2,  TL = 1,  TL = 0
  const static int sMCTFSpeed[5] { 0, 0, ((3<<12) + (2<<9) + (2<<6) + (0<<3) + 0),   ((3<<12) + (3<<9) + (2<<6) + (1<<3) + 0),   ((3<<12) + (3<<9) + (2<<6) + (2<<3) + 2) };

  m_MCTFSpeedVal     = sMCTFSpeed[ m_encCfg->m_vvencMCTF.MCTFSpeed ];
  m_lowResFltSearch  = m_encCfg->m_vvencMCTF.MCTFSpeed > 0;
  m_searchPttrn      = m_encCfg->m_vvencMCTF.MCTFSpeed > 0 ? ( m_encCfg->m_vvencMCTF.MCTFSpeed >= 3 ? 2 : 1 ) : 0;
  m_mctfUnitSize     = m_encCfg->m_vvencMCTF.MCTFUnitSize;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================


void MCTF::initPicture( Picture* pic )
{
  pic->getOrigBuf().extendBorderPel( MCTF_PADDING, MCTF_PADDING );
  pic->setSccFlags( m_encCfg );
}

void MCTF::processPictures( const PicList& picList, bool flush, AccessUnitList& auList, PicList& doneList, PicList& freeList )
{
  // ensure this is only processed if necessary 
  if( !flush && (picList.empty() || ( m_lastPicIn == picList.back())))
  {
    return;
  }
  m_lastPicIn = picList.back();

  // filter one picture (either all or up to frames to be encoded)
  if( picList.size()
      && m_filterPoc <= picList.back()->poc
      && ( m_encCfg->m_framesToBeEncoded <= 0 || m_filterPoc < m_encCfg->m_framesToBeEncoded ) )
  {
    // setup fifo of pictures to be filtered
    std::deque<Picture*> picFifo;
    int filterIdx = 0;
    for( auto pic : picList )
    {
      const int minPoc = m_filterPoc - VVENC_MCTF_RANGE;
      const int maxPoc = m_encCfg->m_vvencMCTF.MCTFFutureReference ? m_filterPoc + VVENC_MCTF_RANGE : m_filterPoc;
      if( pic->poc >= minPoc && pic->poc <= maxPoc )
      {
        picFifo.push_back( pic );
        if( pic->poc < m_filterPoc )
        {
          filterIdx += 1;
        }
      }
    }
    CHECK( picFifo.empty(), "MCTF: no pictures to be filtered found" );
    CHECK( filterIdx >= (int)picFifo.size(), "MCTF: picture filter error" );
    CHECK( picFifo[ filterIdx ]->poc != m_filterPoc, "MCTF: picture filter error" );
    // filter picture (when more than 1 picture is available for processing)
    if( picFifo.size() > 1 )
    {
      filter( picFifo, filterIdx );
    }
    // set picture done
    doneList.push_back( picFifo[ filterIdx ] );
  }

  // mark pictures not needed anymore
  for( auto pic : picList )
  {
    if( pic->poc > m_filterPoc - VVENC_MCTF_RANGE )
      break;
    freeList.push_back( pic );
  }
  m_filterPoc += 1;
}


void MCTF::filter( const std::deque<Picture*>& picFifo, int filterIdx )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_MCTF );

  Picture* pic = picFifo[ filterIdx ];

  const int mctfIdx            = pic->gopEntry ? pic->gopEntry->m_mctfIndex : -1;
  const double overallStrength = mctfIdx >= 0 ? m_encCfg->m_vvencMCTF.MCTFStrengths[ mctfIdx ] : -1.0;
  bool  isFilterThisFrame      = mctfIdx >= 0;

  int dropFrames = ( m_encCfg->m_usePerceptQPA ? VVENC_MCTF_RANGE >> 1 : 0 );
  if( mctfIdx >= 0 )
  {
    const int idxTLayer = m_encCfg->m_vvencMCTF.numFrames - (mctfIdx + 1);
    const int threshold = (m_MCTFSpeedVal >> (idxTLayer * 3)) & 7;

    dropFrames          = std::min(VVENC_MCTF_RANGE, threshold);
    isFilterThisFrame   = threshold < VVENC_MCTF_RANGE;
  }

  const int filterFrames = VVENC_MCTF_RANGE - dropFrames;

  int dropFramesFront = std::min( std::max(                                          filterIdx - filterFrames, 0 ), dropFrames );
  int dropFramesBack  = std::min( std::max( static_cast<int>( picFifo.size() ) - 1 - filterIdx - filterFrames, 0 ), dropFrames );

  if( !pic->useScMCTF )
  {
    isFilterThisFrame = false;
  }

  pic->m_picShared->m_picAuxQpOffset = 0;

  if ( isFilterThisFrame || pic->gopEntry->m_isStartOfGop )
  {
    const PelStorage& origBuf = pic->getOrigBuffer();
          PelStorage& fltrBuf = pic->getFilteredOrigBuffer();

    // subsample original picture so it only needs to be done once
    PelStorage origSubsampled2;
    PelStorage origSubsampled4;
    subsampleLuma( origBuf,         origSubsampled2 );
    subsampleLuma( origSubsampled2, origSubsampled4 );

    // determine motion vectors
    std::deque<TemporalFilterSourcePicInfo> srcFrameInfo;
    for ( int i = dropFramesFront; i < picFifo.size() - dropFramesBack; i++ )
    {
      Picture* curPic = picFifo[ i ];
      if ( curPic->poc == m_filterPoc )
      {
        continue;
      }
      srcFrameInfo.push_back( TemporalFilterSourcePicInfo() );
      TemporalFilterSourcePicInfo &srcPic = srcFrameInfo.back();

      const int wInBlks = ( m_area.width  + m_mctfUnitSize - 1 ) / m_mctfUnitSize;
      const int hInBlks = ( m_area.height + m_mctfUnitSize - 1 ) / m_mctfUnitSize;

      srcPic.picBuffer.createFromBuf( curPic->getOrigBuf() );
      srcPic.mvs.allocate( wInBlks, hInBlks );

      {
        const int width  = m_area.width;
        const int height = m_area.height;
        Array2D<MotionVector> mv_0( width / ( m_mctfUnitSize * 8 ) + 1, height / ( m_mctfUnitSize * 8 ) + 1 );
        Array2D<MotionVector> mv_1( width / ( m_mctfUnitSize * 4 ) + 1, height / ( m_mctfUnitSize * 4 ) + 1 );
        Array2D<MotionVector> mv_2( width / ( m_mctfUnitSize * 2 ) + 1, height / ( m_mctfUnitSize * 2 ) + 1 );

        PelStorage bufferSub2;
        PelStorage bufferSub4;

        subsampleLuma( srcPic.picBuffer, bufferSub2 );
        subsampleLuma( bufferSub2,       bufferSub4 );

        motionEstimationLuma( mv_0, origSubsampled4, bufferSub4,       2 * m_mctfUnitSize );
        motionEstimationLuma( mv_1, origSubsampled2, bufferSub2,       2 * m_mctfUnitSize, &mv_0, 2 );
        motionEstimationLuma( mv_2, origBuf,         srcPic.picBuffer, 2 * m_mctfUnitSize, &mv_1, 2 );

        motionEstimationLuma( srcPic.mvs, origBuf,   srcPic.picBuffer,     m_mctfUnitSize, &mv_2, 1, true );
      }

      srcPic.index = std::min( 3, std::abs( curPic->poc - m_filterPoc ) - 1 );
    }

    // filter
    if( pic->useScMCTF )
    {
      fltrBuf.create( m_encCfg->m_internChromaFormat, m_area, 0, m_padding );
      bilateralFilter( origBuf, srcFrameInfo, fltrBuf, overallStrength );
    }

    if( m_encCfg->m_blockImportanceMapping || m_encCfg->m_usePerceptQPA || pic->gopEntry->m_isStartOfGop )
    {
      const int ctuSize        = m_encCfg->m_bimCtuSize;
      const int widthInCtus    = ( m_area.width  + ctuSize - 1 ) / ctuSize;
      const int heightInCtus   = ( m_area.height + ctuSize - 1 ) / ctuSize;
      const int numCtu         = widthInCtus * heightInCtus;
      const int ctuBlocks      = ctuSize / m_mctfUnitSize;

      std::vector<double> sumError( numCtu * 2, 0 );
      std::vector<uint32_t> sumRMS( numCtu * 2, 0 ); // RMS of motion estimation error
      std::vector<double> blkCount( numCtu * 2, 0 );

      int distFactor[2] = { 3,3 };

      for( auto& srcPic : srcFrameInfo )
      {
        if( srcPic.index >= 2 )
        {
          continue;
        }

        int dist = srcPic.index;
        distFactor[dist]--;

        for( int y = 0; y < srcPic.mvs.h(); y++ ) // going over ref pic in block steps
        {
          for( int x = 0; x < srcPic.mvs.w(); x++ )
          {
            const int ctuX    = x / ctuBlocks;
            const int ctuY    = y / ctuBlocks;
            const int ctuId   = ctuY * widthInCtus + ctuX;
            const auto& mvBlk = srcPic.mvs.get( x, y );
            sumError[dist * numCtu + ctuId] += mvBlk.error;
            sumRMS  [dist * numCtu + ctuId] += mvBlk.rmsme;
            blkCount[dist * numCtu + ctuId] += mvBlk.overlap;
          }
        }
      }

      if( distFactor[0] < 3 && distFactor[1] < 3 && ( m_encCfg->m_usePerceptQPA || pic->gopEntry->m_isStartOfGop ) )
      {
        const double bd12bScale = double (m_encCfg->m_internalBitDepth[CH_L] < 12 ? 1 << (12 - m_encCfg->m_internalBitDepth[CH_L]) : 1);
        double meanRmsAcrossPic = 0.0;

        for( int i = 0; i < numCtu; i++ ) // start noise estimation with motion errors
        {
          const Position pos ((i % widthInCtus) * ctuSize, (i / widthInCtus) * ctuSize);
          const CompArea ctuArea  = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x, pos.y, ctuSize, ctuSize)), pic->Y());
          const unsigned avgIndex = pic->getOrigBuf (ctuArea).getAvg() >> (m_encCfg->m_internalBitDepth[CH_L] - 3); // one of 8 mean level regions

          sumRMS[i] = std::min (sumRMS[i], sumRMS[i + numCtu]);
          meanRmsAcrossPic += bd12bScale * sumRMS[i] / blkCount[i];

          if (bd12bScale * sumRMS[i] < pic->m_picShared->m_minNoiseLevels[avgIndex] * blkCount[i])
          {
            pic->m_picShared->m_minNoiseLevels[avgIndex] = uint8_t (0.5 + bd12bScale * sumRMS[i] / blkCount[i]); // scaled to 12 bit, see also QPA
          }
        }

        if( pic->gopEntry->m_isStartOfGop && !pic->useScMCTF && m_encCfg->m_vvencMCTF.MCTF > 0 && meanRmsAcrossPic > numCtu * 27.0)
        {
          // force filter
          fltrBuf.create( m_encCfg->m_internChromaFormat, m_area, 0, m_padding );
          bilateralFilter( origBuf, srcFrameInfo, fltrBuf, overallStrength );
        }
      }

      if( !m_encCfg->m_blockImportanceMapping || !pic->useScMCTF )
      {
        CHECKD( !pic->m_picShared->m_ctuBimQpOffset.empty(), "BIM disabled, but offset vector not empty!" );
        return;
      }

      pic->m_picShared->m_ctuBimQpOffset.resize( numCtu, 0 );

      if( distFactor[0] < 3 && distFactor[1] < 3 )
      {
        const double weight = std::min( 1.0, overallStrength );
        int sumCtuQpOffsets = 0;

        for( int i = 0; i < numCtu; i++ )
        {
          const int avgErrD1 = ( int ) ( ( sumError[i         ] / blkCount[i         ] ) * distFactor[0] );
          const int avgErrD2 = ( int ) ( ( sumError[i + numCtu] / blkCount[i + numCtu] ) * distFactor[1] );
          int weightedErr = std::max( avgErrD1, avgErrD2 ) + abs( avgErrD2 - avgErrD1 ) * 3;
          weightedErr     = ( int ) ( weightedErr * weight + ( 1 - weight ) * m_cuTreeCenter );

          int qpOffset = 0;

          if( weightedErr > m_cuTreeThresh[0] )
          {
            qpOffset = 2;
          }
          else if( weightedErr > m_cuTreeThresh[1] )
          {
            qpOffset = 1;
          }
          else if( weightedErr < m_cuTreeThresh[3] )
          {
            qpOffset = -2;
          }
          else if( weightedErr < m_cuTreeThresh[2] )
          {
            qpOffset = -1;
          }

          pic->m_picShared->m_ctuBimQpOffset[i] = qpOffset;
          sumCtuQpOffsets += qpOffset;
        }

        pic->m_picShared->m_picAuxQpOffset = ( sumCtuQpOffsets + ( sumCtuQpOffsets < 0 ? -(numCtu >> 1) : numCtu >> 1 ) ) / numCtu; // pic average
        for( int i = 0; i < numCtu; i++ )
        {
          pic->m_picShared->m_ctuBimQpOffset[i] -= pic->m_picShared->m_picAuxQpOffset; // delta-QP relative to above average, see xGetQPForPicture
        }
      }
      else
      {
        std::fill( pic->m_picShared->m_ctuBimQpOffset.begin(), pic->m_picShared->m_ctuBimQpOffset.end(), 0 );
      }
    }
  }
  else
  {
    pic->m_picShared->m_ctuBimQpOffset.resize( 0 );
  }
}

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

void MCTF::subsampleLuma(const PelStorage &input, PelStorage &output, const int factor) const
{
  const int newWidth = input.Y().width / factor;
  const int newHeight = input.Y().height / factor;
  output.create(CHROMA_400, Area(0, 0, newWidth, newHeight), 0, m_padding);

  const Pel* srcRow = input.Y().buf;
  const int srcStride = input.Y().stride;
  Pel* dstRow = output.Y().buf;
  const int dstStride = output.Y().stride;

  for (int y = 0; y < newHeight; y++, srcRow+=factor*srcStride, dstRow+=dstStride)
  {
    const Pel* inRow      = srcRow;
    const Pel* inRowBelow = srcRow+srcStride;
    Pel* target     = dstRow;

    for (int x = 0; x < newWidth; x++)
    {
      target[x] = (inRow[0] + inRowBelow[0] + inRow[1] + inRowBelow[1] + 2) >> 2;
      inRow += 2;
      inRowBelow += 2;
    }
  }
  output.extendBorderPel(m_padding, m_padding);
}

int MCTF::motionErrorLuma(const PelStorage &orig,
  const PelStorage &buffer,
  const int x,
  const int y,
  int dx,
  int dy,
  const int bs,
  const int besterror = MAX_INT) const
{
  int fx = dx & 0xf;
  int fy = dy & 0xf;

  int error = 0;// dx * 10 + dy * 10;

  CHECKD( bs & 7, "Blocksize has to be a multiple of 8!" );

  const int w = std::min<int>( bs, orig.Y().width  - x ) & ~7;
  const int h = std::min<int>( bs, orig.Y().height - y ) & ~7;

  CHECK( !w || !h, "Incompatible sizes!" );

  if( ( fx | fy ) == 0 )
  {
    dx /= m_motionVectorFactor;
    dy /= m_motionVectorFactor;

    const int  origStride = orig.Y().stride;
    const Pel* org        = orig.Y().buf + x + y * origStride;
    const int  buffStride = buffer.Y().stride;
    const Pel* buf        = buffer.Y().buf + x + dx + ( y + dy ) * buffStride;

    return m_motionErrorLumaInt8( org, origStride, buf, buffStride, w, h, besterror );
  }
  else if( m_lowResFltSearch )
  {
    dx >>= 4;
    dy >>= 4;

    const int  origStride = orig.Y().stride;
    const Pel* org        = orig.Y().buf + x + y * origStride;
    const int  buffStride = buffer.Y().stride;
    const Pel* buf        = buffer.Y().buf + x + dx + ( y + dy ) * buffStride;

    const int16_t *xFilter = m_interpolationFilter4[fx];
    const int16_t *yFilter = m_interpolationFilter4[fy];

    return m_motionErrorLumaFrac8[1]( org, origStride, buf, buffStride, w, h, xFilter, yFilter, m_encCfg->m_internalBitDepth[CH_L], besterror );
  }
  else
  {
    dx >>= 4;
    dy >>= 4;

    const int  origStride = orig.Y().stride;
    const Pel* org        = orig.Y().buf + x + y * origStride;
    const int  buffStride = buffer.Y().stride;
    const Pel* buf        = buffer.Y().buf + x + dx + ( y + dy ) * buffStride;

    const int16_t *xFilter = m_interpolationFilter8[fx];
    const int16_t *yFilter = m_interpolationFilter8[fy];

    return m_motionErrorLumaFrac8[0]( org, origStride, buf, buffStride, w,h, xFilter, yFilter, m_encCfg->m_internalBitDepth[CH_L], besterror );
  }

  return error;
}

bool MCTF::estimateLumaLn( std::atomic_int& blockX_, std::atomic_int* prevLineX, Array2D<MotionVector> &mvs, const PelStorage &orig, const PelStorage &buffer, const int blockSize,
  const Array2D<MotionVector> *previous, const int factor, const bool doubleRes, int blockY ) const
{
  PROFILER_SCOPE_AND_STAGE( 1, _TPROF, P_MCTF_SEARCH );

  const int stepSize = blockSize;
  const int origWidth  = orig.Y().width;

  for( int blockX = blockX_.load(); blockX + 7 <= origWidth; blockX += stepSize, blockX_.store( blockX) )
  {
    if( prevLineX && blockX >= prevLineX->load() ) return false;

    int range = doubleRes ? 0 : ( m_searchPttrn == 2 ? 3 : 5 );
    const int stepSize = blockSize;

    MotionVector best;

    if (previous == NULL)
    {
      range = m_searchPttrn == 2 ? 5 : 8;
    }
    else
    {
      for( int py = -1; py <= 1; py++ )
      {
        int testy = blockY / (2 * blockSize) + py;
        if( (testy >= 0) && (testy < previous->h()) )
        {
          for (int px = -1; px <= 1; px++)
          {
            int testx = blockX / (2 * blockSize) + px;
            if ((testx >= 0) && (testx < previous->w()) )
            {
              const MotionVector& old = previous->get(testx, testy);
              int error = motionErrorLuma(orig, buffer, blockX, blockY, old.x * factor, old.y * factor, blockSize, best.error);
              if (error < best.error)
              {
                best.set(old.x * factor, old.y * factor, error);
              }
            }
          }
        }
      }

      int error = motionErrorLuma( orig, buffer, blockX, blockY, 0, 0, blockSize, best.error );
      if( error < best.error )
      {
        best.set( 0, 0, error );
      }
    }
    MotionVector prevBest = best;
    const int d = previous == NULL && m_searchPttrn == 2 ? 2 : 1;
    for( int y2 = prevBest.y / m_motionVectorFactor - range; y2 <= prevBest.y / m_motionVectorFactor + range; y2 += d )
    {
      for( int x2 = prevBest.x / m_motionVectorFactor - range; x2 <= prevBest.x / m_motionVectorFactor + range; x2 += d )
      {
        int error = motionErrorLuma( orig, buffer, blockX, blockY, x2 * m_motionVectorFactor, y2 * m_motionVectorFactor, blockSize, best.error );
        if( error < best.error )
        {
          best.set( x2 * m_motionVectorFactor, y2 * m_motionVectorFactor, error );
        }
      }
    }
    if (doubleRes)
    { // merge into one loop, probably with precision array (here [12, 3] or maybe [4, 1]) with setable number of iterations
      PROFILER_SCOPE_AND_STAGE( 1, _TPROF, P_MCTF_SEARCH_SUBPEL );

      prevBest = best;
      static const int range[] = { 12, 6, 4 };
      int doubleRange = range[m_searchPttrn];

      // first iteration, 49 - 1 or 16 checks
      for( int y2 = -doubleRange; y2 <= doubleRange; y2 += 4 )
      {
        for( int x2 = -doubleRange; x2 <= doubleRange; x2 += 4 )
        {
          if( x2 || y2 )
          {
            int error = motionErrorLuma( orig, buffer, blockX, blockY, prevBest.x + x2, prevBest.y + y2, blockSize, best.error );
            if( error < best.error )
            {
              best.set( prevBest.x + x2, prevBest.y + y2, error );
            }
          }
        }
      }

      prevBest = best;
      doubleRange = 2;
      // second iteration, 9 - 1 checks
      for( int y2 = -doubleRange; y2 <= doubleRange; y2 += 2 )
      {
        for( int x2 = -doubleRange; x2 <= doubleRange; x2 += 2 )
        {
          if( x2 || y2 )
          {
            int error = motionErrorLuma( orig, buffer, blockX, blockY, prevBest.x + x2, prevBest.y + y2, blockSize, best.error );
            if( error < best.error )
            {
              best.set( prevBest.x + x2, prevBest.y + y2, error );
            }
          }
        }
      }

      prevBest = best;
      doubleRange = 1;
      // third iteration, 9 - 1 checks
      for (int y2 = -doubleRange; y2 <= doubleRange; y2++)
      {
        for (int x2 = -doubleRange; x2 <= doubleRange; x2++)
        {
          if( x2 || y2 )
          {
            int error = motionErrorLuma( orig, buffer, blockX, blockY, prevBest.x + x2, prevBest.y + y2, blockSize, best.error );
            if( error < best.error )
            {
              best.set( prevBest.x + x2, prevBest.y + y2, error );
            }
          }
        }
      }
    } 
    if( blockY > 0 )
    {
      MotionVector aboveMV = mvs.get( blockX / stepSize, ( blockY - stepSize ) / stepSize );
      int error = motionErrorLuma( orig, buffer, blockX, blockY, aboveMV.x, aboveMV.y, blockSize, best.error );
      if( error < best.error )
      {
        best.set( aboveMV.x, aboveMV.y, error );
      }
    }
    if( blockX > 0 )
    {
      MotionVector leftMV = mvs.get( ( blockX - stepSize ) / stepSize, blockY / stepSize );
      int error = motionErrorLuma( orig, buffer, blockX, blockY, leftMV.x, leftMV.y, blockSize, best.error );
      if( error < best.error )
      {
        best.set( leftMV.x, leftMV.y, error );
      }
    }

    const int w = std::min<int>( blockSize, orig.Y().width  - blockX ) & ~7;
    const int h = std::min<int>( blockSize, orig.Y().height - blockY ) & ~7;

    const double dvar = m_calcVar( orig.Y().bufAt( blockX, blockY ), orig.Y().stride, w, h );
    const double mse  = best.error / double( w * h );

    best.error   = ( int ) ( 20 * ( ( best.error + 5.0 ) / ( dvar + 5.0 ) ) + mse / 50.0 );
    best.rmsme   = uint16_t( 0.5 + sqrt( mse ) );
    best.overlap = ( ( double ) w * h ) / ( m_mctfUnitSize * m_mctfUnitSize );

    mvs.get(blockX / stepSize, blockY / stepSize) = best;
  }

  return true;
}

void MCTF::motionEstimationLuma(Array2D<MotionVector> &mvs, const PelStorage &orig, const PelStorage &buffer, const int blockSize, const Array2D<MotionVector> *previous, const int factor, const bool doubleRes) const
{
  const int stepSize = blockSize;
  const int origHeight = orig.Y().height;

  if( m_threadPool )
  {
    struct EstParams
    {
      std::atomic_int blockX;
      std::atomic_int* prevLineX;
      Array2D<MotionVector> *mvs;
      const PelStorage* orig; 
      const PelStorage* buffer; 
      const Array2D<MotionVector> *previous; 
      int   blockSize; 
      int   factor; 
      bool  doubleRes;
      int   blockY;
      const MCTF* mctf;
    };

    std::vector<EstParams> EstParamsArray( origHeight/stepSize + 1 );

    WaitCounter taskCounter;

    for( int n = 0, blockY = 0; blockY + 7 <= origHeight; blockY += stepSize, n++ )
    {
      static auto task = []( int tId, EstParams* params)
      {
        ITT_TASKSTART( itt_domain_MCTF_est, itt_handle_est );

        bool ret = params->mctf->estimateLumaLn( params->blockX, params->prevLineX, *params->mvs, *params->orig, *params->buffer, params->blockSize, params->previous, params->factor, params->doubleRes, params->blockY );

        ITT_TASKEND( itt_domain_MCTF_est, itt_handle_est );
        return ret;
      };

      EstParams& cEstParams = EstParamsArray[n];
      cEstParams.blockX = 0;
      cEstParams.prevLineX = n == 0 ? nullptr : &EstParamsArray[n-1].blockX;
      cEstParams.mvs = &mvs; 
      cEstParams.orig = &orig;
      cEstParams.buffer = &buffer; 
      cEstParams.previous = previous;
      cEstParams.blockSize = blockSize; 
      cEstParams.factor = factor;
      cEstParams.doubleRes = doubleRes;
      cEstParams.mctf = this;
      cEstParams.blockY = blockY;

      m_threadPool->addBarrierTask<EstParams>( task, &cEstParams, &taskCounter);
    }
    taskCounter.wait();
  }
  else
  {
    for( int blockY = 0; blockY + 7 <= origHeight; blockY += stepSize )
    {
      std::atomic_int blockX( 0 ), prevBlockX( orig.Y().width + stepSize );
      estimateLumaLn( blockX, blockY ? &prevBlockX : nullptr, mvs, orig, buffer, blockSize, previous, factor, doubleRes, blockY );
    }

  }
}

void MCTF::xFinalizeBlkLine( const PelStorage &orgPic, std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo, PelStorage &newOrgPic, int yStart, const double sigmaSqCh[MAX_NUM_CH], double overallStrength ) const
{
  PROFILER_SCOPE_AND_STAGE( 1, _TPROF, P_MCTF_APPLY );

  const int numRefs = int(srcFrameInfo.size());

  int refStrengthRow = 2;
  if( numRefs == m_range * 2 )
  {
    refStrengthRow = 0;
  }
  else if( numRefs == m_range )
  {
    refStrengthRow = 1;
  }

  // max 64*64*8*2 = 2^(6+6+3+1)=2^16=64kbps, usually 16*16*8*2=2^(4+4+3+1)=4kbps, and allow for overread of one line
  Pel* dstBufs = ( Pel* ) alloca( sizeof( Pel ) * ( numRefs * m_mctfUnitSize * m_mctfUnitSize + m_mctfUnitSize ) );

  for( int c = 0; c < getNumberValidComponents( m_encCfg->m_internChromaFormat ); c++ )
  {
    const ComponentID compID = ( ComponentID ) c;
    const int height    = orgPic.bufs[c].height;
    const int width     = orgPic.bufs[c].width;

    const double sigmaSq = sigmaSqCh[ toChannelType( compID) ];
    const double weightScaling = overallStrength * ( isChroma( compID ) ? m_chromaFactor : 0.4 );
    const ClpRng clpRng{ m_encCfg->m_internalBitDepth[toChannelType( compID )] };

    const int blkSizeY = m_mctfUnitSize >> getComponentScaleY( compID, m_encCfg->m_internChromaFormat );
    const int blkSizeX = m_mctfUnitSize >> getComponentScaleX( compID, m_encCfg->m_internChromaFormat );
    const int yOut     = yStart         >> getComponentScaleY( compID, m_encCfg->m_internChromaFormat );

    for( int by = yOut, yBlkAddr = yStart / m_mctfUnitSize; by < std::min( yOut + blkSizeY, height ); by += blkSizeY, yBlkAddr++ )
    {
      const int h = std::min( blkSizeY, height - by );

      for( int bx = 0, xBlkAddr = 0; bx < width; bx += blkSizeX, xBlkAddr++ )
      {
        const int w = std::min( blkSizeX, width - bx );

        const int csx = getComponentScaleX( compID, m_encCfg->m_internChromaFormat );
        const int csy = getComponentScaleY( compID, m_encCfg->m_internChromaFormat );

        const Pel* correctedPics[2 * VVENC_MCTF_RANGE] = { nullptr, };
              Pel* currDst = dstBufs;
        int verror     [2 * VVENC_MCTF_RANGE] = { 0,   };
        double refStr  [2 * VVENC_MCTF_RANGE] = { 0.0, };

        for( int i = 0; i < numRefs; i++, currDst += w * h )
        {
          const Pel* srcImage = srcFrameInfo[i].picBuffer.bufs[compID].buf;
          const int srcStride = srcFrameInfo[i].picBuffer.bufs[compID].stride;

                Pel* dst      = currDst;
          const int dstStride = w;
          correctedPics[i]    = dst;

          const MotionVector& mv = srcFrameInfo[i].mvs.get( xBlkAddr, yBlkAddr);
          const int dx   = mv.x >> csx;
          const int dy   = mv.y >> csy;
          const int xInt = mv.x >> ( 4 + csx );
          const int yInt = mv.y >> ( 4 + csy );

          const int yOffset = by + yInt;
          const int xOffset = bx + xInt;
          const Pel* src = srcImage + yOffset * srcStride + xOffset;

          if( m_lowResFltApply ) // || isChroma( compID )
          {
            const int16_t* xFilter = m_interpolationFilter4[dx & 0xf];
            const int16_t* yFilter = m_interpolationFilter4[dy & 0xf]; // will add 6 bit.

            m_applyFrac[toChannelType( compID )][1]( src, srcStride, dst, dstStride, w, h, xFilter, yFilter, m_encCfg->m_internalBitDepth[toChannelType( compID )] );
          }
          else
          {
            const int16_t* xFilter = m_interpolationFilter8[dx & 0xf];
            const int16_t* yFilter = m_interpolationFilter8[dy & 0xf]; // will add 6 bit.

            m_applyFrac[toChannelType( compID )][0]( src, srcStride, dst, dstStride, w, h, xFilter, yFilter, m_encCfg->m_internalBitDepth[toChannelType( compID )] );
          }

          verror[i] = mv.error;
          refStr[i] = m_refStrengths[refStrengthRow][srcFrameInfo[i].index];
        }

        m_applyBlock( orgPic.bufs[c], newOrgPic.bufs[c], CompArea( compID, orgPic.chromaFormat, Area( bx, by, w, h ) ), clpRng, correctedPics, numRefs, verror, refStr, weightScaling, sigmaSq );
      }
    }
  }
}

void MCTF::bilateralFilter(const PelStorage& orgPic, std::deque<TemporalFilterSourcePicInfo>& srcFrameInfo, PelStorage& newOrgPic, double overallStrength) const
{
  const double lumaSigmaSq = (m_encCfg->m_QP - m_sigmaZeroPoint) * (m_encCfg->m_QP - m_sigmaZeroPoint) * m_sigmaMultiplier;
  const double chromaSigmaSq = 30 * 30;

  double sigmaSqCh[MAX_NUM_CH];
  for(int c=0; c< getNumberValidChannels(m_encCfg->m_internChromaFormat); c++)
  {
    const ChannelType ch=(ChannelType)c;
    const Pel maxSampleValue = (1<<m_encCfg->m_internalBitDepth[ch])-1;
    const double bitDepthDiffWeighting=1024.0 / (maxSampleValue+1);
    sigmaSqCh[ch] = ( isChroma( ch ) ? chromaSigmaSq : lumaSigmaSq ) / ( bitDepthDiffWeighting * bitDepthDiffWeighting );
  }

  if( m_threadPool )
  {
    struct FltParams
    {
      const PelStorage *orgPic; 
      std::deque<TemporalFilterSourcePicInfo> *srcFrameInfo; 
      PelStorage *newOrgPic;
      const double *sigmaSqCh;
      double overallStrength;
      const MCTF* mctf;
      int yStart; 
    };

    std::vector<FltParams> FltParamsArray( orgPic.Y().height/ m_mctfUnitSize + 1 );

    WaitCounter taskCounter;

    for (int n = 0, yStart = 0; yStart < orgPic.Y().height; yStart += m_mctfUnitSize, n++)
    {
      static auto task = []( int tId, FltParams* params)
      {
        ITT_TASKSTART( itt_domain_MCTF_flt, itt_handle_flt );

        params->mctf->xFinalizeBlkLine( *params->orgPic, *params->srcFrameInfo, *params->newOrgPic, params->yStart, params->sigmaSqCh, params->overallStrength );

        ITT_TASKEND( itt_domain_MCTF_flt, itt_handle_flt );
        return true;
      };

      FltParams& cFltParams = FltParamsArray[n];
      cFltParams.orgPic = &orgPic; 
      cFltParams.srcFrameInfo = &srcFrameInfo; 
      cFltParams.newOrgPic = &newOrgPic;
      cFltParams.sigmaSqCh = sigmaSqCh;
      cFltParams.overallStrength = overallStrength;
      cFltParams.mctf = this;
      cFltParams.yStart = yStart;

      m_threadPool->addBarrierTask<FltParams>( task, &cFltParams, &taskCounter);
    }
    taskCounter.wait();
  }
  else
  {
    for (int yStart = 0; yStart < orgPic.Y().height; yStart += m_mctfUnitSize )
    {
      xFinalizeBlkLine( orgPic, srcFrameInfo, newOrgPic, yStart, sigmaSqCh, overallStrength );
    }
  }
}

} // namespace vvenc

//! \}

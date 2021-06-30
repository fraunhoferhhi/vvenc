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


/** \file     RdCost.cpp
    \brief    RD cost computation class
*/

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

#include "RdCost.h"
#include "Rom.h"
#include "UnitPartitioner.h"


//! \ingroup CommonLib
//! \{

namespace vvenc {

RdCost::RdCost()
  : m_afpDistortFunc{ { nullptr, }, { nullptr, } }
{
}

RdCost::~RdCost()
{
}

void RdCost::setLambda( double dLambda, const BitDepths &bitDepths )
{
  m_dLambda          = dLambda;
  m_DistScale        = double(1<<SCALE_BITS) / m_dLambda;
  m_dLambdaMotionSAD = sqrt(m_dLambda);
}


// Initialize Function Pointer by [eDFunc]
void RdCost::create()
{
  m_signalType                 = RESHAPE_SIGNAL_NULL;
  m_chromaWeight               = 1.0;
  m_lumaBD                     = 10;
  m_afpDistortFunc[0][DF_SSE    ] = RdCost::xGetSSE;
  m_afpDistortFunc[0][DF_SSE2   ] = RdCost::xGetSSE;
  m_afpDistortFunc[0][DF_SSE4   ] = RdCost::xGetSSE4;
  m_afpDistortFunc[0][DF_SSE8   ] = RdCost::xGetSSE8;
  m_afpDistortFunc[0][DF_SSE16  ] = RdCost::xGetSSE16;
  m_afpDistortFunc[0][DF_SSE32  ] = RdCost::xGetSSE32;
  m_afpDistortFunc[0][DF_SSE64  ] = RdCost::xGetSSE64;
  m_afpDistortFunc[0][DF_SSE128 ] = RdCost::xGetSSE128;

  m_afpDistortFunc[0][DF_SAD    ] = RdCost::xGetSAD;
  m_afpDistortFunc[0][DF_SAD2   ] = RdCost::xGetSAD;
  m_afpDistortFunc[0][DF_SAD4   ] = RdCost::xGetSAD4;
  m_afpDistortFunc[0][DF_SAD8   ] = RdCost::xGetSAD8;
  m_afpDistortFunc[0][DF_SAD16  ] = RdCost::xGetSAD16;
  m_afpDistortFunc[0][DF_SAD32  ] = RdCost::xGetSAD32;
  m_afpDistortFunc[0][DF_SAD64  ] = RdCost::xGetSAD64;
  m_afpDistortFunc[0][DF_SAD128 ] = RdCost::xGetSAD128;

  m_afpDistortFunc[0][DF_HAD    ] = RdCost::xGetHADs;
  m_afpDistortFunc[0][DF_HAD2   ] = RdCost::xGetHADs;
  m_afpDistortFunc[0][DF_HAD4   ] = RdCost::xGetHADs;
  m_afpDistortFunc[0][DF_HAD8   ] = RdCost::xGetHADs;
  m_afpDistortFunc[0][DF_HAD16  ] = RdCost::xGetHADs;
  m_afpDistortFunc[0][DF_HAD32  ] = RdCost::xGetHADs;
  m_afpDistortFunc[0][DF_HAD64  ] = RdCost::xGetHADs;
  m_afpDistortFunc[0][DF_HAD128 ] = RdCost::xGetHADs;

  //  m_afpDistortFunc[0][DF_SAD_INTERMEDIATE_BITDEPTH] = RdCost::xGetSAD;
  m_afpDistortFunc[0][DF_HAD_2SAD ] = RdCost::xGetHAD2SADs;

  m_afpDistortFunc[0][DF_SAD_WITH_MASK] = RdCost::xGetSADwMask;
  // m_afpDistortFunc[1] can be used in any case
  memcpy( m_afpDistortFunc[1], m_afpDistortFunc[0], sizeof(m_afpDistortFunc)/2);

#if ENABLE_SIMD_OPT_DIST
#ifdef TARGET_SIMD_X86
  initRdCostX86();
#endif
#endif

  m_costMode      = VVENC_COST_STANDARD_LOSSY;
  m_motionLambda  = 0;
  m_iCostScale    = 0;
}

#if ENABLE_MEASURE_SEARCH_SPACE
static Distortion xMeasurePredSearchSpaceInterceptor( const DistParam& dp )
{
  g_searchSpaceAcc.addPrediction( dp.cur.width, dp.cur.height, toChannelType( dp.compID ) );
  return dp.xDistFunc( dp );
}

#endif
void RdCost::setDistParam( DistParam &rcDP, const CPelBuf& org, const Pel* piRefY, int iRefStride, int bitDepth, ComponentID compID, int subShiftMode, bool useHadamard )
{
  rcDP.bitDepth   = bitDepth;
  rcDP.compID     = compID;

  // set Original & Curr Pointer / Stride
  rcDP.org        = org;

  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;

  // set Block Width / Height
  rcDP.cur.width    = org.width;
  rcDP.cur.height   = org.height;
  rcDP.maximumDistortionForEarlyExit = MAX_DISTORTION;

  const int base = (rcDP.bitDepth > 10 || rcDP.applyWeight) ? 1 : 0;
  if( !useHadamard )
  {
    rcDP.distFunc = m_afpDistortFunc[base][ DF_SAD + Log2( org.width ) ];
  }
  else
  {
    rcDP.distFunc = m_afpDistortFunc[base][ DF_HAD + Log2( org.width ) ];
  }

  // initialize
  rcDP.subShift  = 0;

  if( subShiftMode == 1 )
  {
    if( rcDP.org.height > 32 && ( rcDP.org.height & 15 ) == 0 )
    {
      rcDP.subShift = 4;
    }
    else if( rcDP.org.height > 16 && ( rcDP.org.height & 7 ) == 0 )
    {
      rcDP.subShift = 3;
    }
    else if( rcDP.org.height > 8 && ( rcDP.org.height & 3 ) == 0 )
    {
      rcDP.subShift = 2;
    }
    else if( ( rcDP.org.height & 1 ) == 0 )
    {
      rcDP.subShift = 1;
    }
  }
  else if( subShiftMode == 2 )
  {
    if( rcDP.org.height > 8 && rcDP.org.width <= 64 )
    {
      rcDP.subShift = 1;
    }
  }
  else if (subShiftMode == 3)
  {
    if (rcDP.org.height > 8)
    {
      rcDP.subShift = 1;
    }
  }

#if ENABLE_MEASURE_SEARCH_SPACE
  rcDP.xDistFunc = rcDP.distFunc;
  rcDP.distFunc  = xMeasurePredSearchSpaceInterceptor;
#endif
}


DistParam RdCost::setDistParam( const CPelBuf& org, const CPelBuf& cur, int bitDepth, DFunc dfunc )
{
  int index = dfunc;
  if( dfunc != DF_HAD && dfunc != DF_HAD_2SAD )
  {
    index += Log2(org.width);
  }

  const int base = bitDepth > 10 ? 1:0; //TBD: check does SDA ever overflow
#if ENABLE_MEASURE_SEARCH_SPACE
  DistParam rcDP( org, cur, m_afpDistortFunc[base][index], bitDepth, 0, COMP_Y );
  rcDP.xDistFunc = rcDP.distFunc;
  rcDP.distFunc  = xMeasurePredSearchSpaceInterceptor;
  return rcDP;
#else
  return DistParam( org, cur, m_afpDistortFunc[base][index], bitDepth, 0, COMP_Y );
#endif
}

DistParam RdCost::setDistParam( const Pel* pOrg, const Pel* piRefY, int iOrgStride, int iRefStride, int bitDepth, ComponentID compID, int width, int height, int subShift, bool isDMVR )
{
  DistParam rcDP;
  rcDP.bitDepth   = bitDepth;
  rcDP.compID     = compID;

  rcDP.org.buf    = pOrg;
  rcDP.org.stride = iOrgStride;
  rcDP.org.width  = width;
  rcDP.org.height = height;

  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;
  rcDP.cur.width  = width;
  rcDP.cur.height = height;
  rcDP.subShift   = subShift;

  //  CHECK( useHadamard || rcDP.useMR, "only used in xDMVRCost with these default parameters (so far...)" );
  const int base = (rcDP.bitDepth > 10) ? 1 : 0;

  rcDP.distFunc = m_afpDistortFunc[base][ DF_SAD + Log2( width ) ];

#if ENABLE_MEASURE_SEARCH_SPACE
  if( !isDMVR )
  {
    // DMVT is part of the decoder complexity
    rcDP.xDistFunc = rcDP.distFunc;
    rcDP.distFunc = xMeasurePredSearchSpaceInterceptor;
  }

#endif
  return rcDP;
}

Distortion RdCost::getDistPart( const CPelBuf& org, const CPelBuf& cur, int bitDepth, const ComponentID compId, DFunc eDFunc, const CPelBuf* orgLuma )
{
  DistParam dp( org, cur, nullptr, bitDepth, 0, compId );
# if ENABLE_MEASURE_SEARCH_SPACE
  g_searchSpaceAcc.addPrediction( dp.cur.width, dp.cur.height, toChannelType( dp.compID ) );
#endif
  Distortion dist;
  if( orgLuma )
  {
    CHECKD( eDFunc != DF_SSE_WTD, "mismatch func and parameter")
    dp.orgLuma  = orgLuma;
    dist = RdCost::xGetSSE_WTD( dp );
  }
  else
  {
    if( ( org.width == 1 ) )
    {
      dist = xGetSSE( dp );
    }
    else
    {
      const int base = (bitDepth > 10) ? 1 : 0;
      dist = m_afpDistortFunc[base][eDFunc + Log2(org.width)](dp);
    }
  }
  if (isChroma(compId))
  {
    return ((Distortion) (m_distortionWeight[ compId ] * dist));
  }
  else
  {
    return dist;
  }
}

// ====================================================================================================================
// Distortion functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xGetSAD( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg           = rcDtParam.org.buf;
  const Pel* piCur           = rcDtParam.cur.buf;
  const int  iCols           = rcDtParam.org.width;
        int  iRows           = rcDtParam.org.height;
  const int  iSubShift       = rcDtParam.subShift;
  const int  iSubStep        = ( 1 << iSubShift );
  const int  iStrideCur      = rcDtParam.cur.stride * iSubStep;
  const int  iStrideOrg      = rcDtParam.org.stride * iSubStep;
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    for (int n = 0; n < iCols; n++ )
    {
      uiSum += abs( piOrg[n] - piCur[n] );
    }
    if (rcDtParam.maximumDistortionForEarlyExit < ( uiSum >> distortionShift ))
    {
      return ( uiSum >> distortionShift );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return ( uiSum >> distortionShift );
}

Distortion RdCost::xGetSAD4( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iSubShift     = rcDtParam.subShift;
  int  iSubStep      = ( 1 << iSubShift );
  int  iStrideCur    = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg    = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD8( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD16( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


Distortion RdCost::xGetSAD128( const DistParam &rcDtParam )
{
  const Pel* piOrg  = rcDtParam.org.buf;
  const Pel* piCur  = rcDtParam.cur.buf;
  int  iRows        = rcDtParam.org.height;
  int  iCols        = rcDtParam.org.width;
  int  iSubShift    = rcDtParam.subShift;
  int  iSubStep     = ( 1 << iSubShift );
  int  iStrideCur   = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg   = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    for (int n = 0; n < iCols; n+=16 )
    {
      uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] );
      uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] );
      uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] );
      uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] );
      uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] );
      uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] );
      uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] );
      uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] );
      uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] );
      uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] );
      uiSum += abs( piOrg[n+10] - piCur[n+10] );
      uiSum += abs( piOrg[n+11] - piCur[n+11] );
      uiSum += abs( piOrg[n+12] - piCur[n+12] );
      uiSum += abs( piOrg[n+13] - piCur[n+13] );
      uiSum += abs( piOrg[n+14] - piCur[n+14] );
      uiSum += abs( piOrg[n+15] - piCur[n+15] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD32( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


Distortion RdCost::xGetSAD64( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    uiSum += abs( piOrg[48] - piCur[48] );
    uiSum += abs( piOrg[49] - piCur[49] );
    uiSum += abs( piOrg[50] - piCur[50] );
    uiSum += abs( piOrg[51] - piCur[51] );
    uiSum += abs( piOrg[52] - piCur[52] );
    uiSum += abs( piOrg[53] - piCur[53] );
    uiSum += abs( piOrg[54] - piCur[54] );
    uiSum += abs( piOrg[55] - piCur[55] );
    uiSum += abs( piOrg[56] - piCur[56] );
    uiSum += abs( piOrg[57] - piCur[57] );
    uiSum += abs( piOrg[58] - piCur[58] );
    uiSum += abs( piOrg[59] - piCur[59] );
    uiSum += abs( piOrg[60] - piCur[60] );
    uiSum += abs( piOrg[61] - piCur[61] );
    uiSum += abs( piOrg[62] - piCur[62] );
    uiSum += abs( piOrg[63] - piCur[63] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


// --------------------------------------------------------------------------------------------------------------------
// SSE
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xGetSSE( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iCols            = rcDtParam.org.width;
  int  iStrideCur       = rcDtParam.cur.stride;
  int  iStrideOrg       = rcDtParam.org.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int iTemp;

  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n++ )
    {
      iTemp = piOrg[n  ] - piCur[n  ];
      uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE4( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK( rcDtParam.org.width != 4, "Invalid size" );
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {

    iTemp = piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE8( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK( rcDtParam.org.width != 8, "Invalid size" );
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[4] - piCur[4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[5] - piCur[5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[6] - piCur[6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[7] - piCur[7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE16( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK( rcDtParam.org.width != 16, "Invalid size" );
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {

    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE128( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }
  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iCols         = rcDtParam.org.width;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n+=16 )
    {

      iTemp = piOrg[n+ 0] - piCur[n+ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 1] - piCur[n+ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 2] - piCur[n+ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 3] - piCur[n+ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 4] - piCur[n+ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 5] - piCur[n+ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 6] - piCur[n+ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 7] - piCur[n+ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 8] - piCur[n+ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 9] - piCur[n+ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+10] - piCur[n+10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+11] - piCur[n+11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+12] - piCur[n+12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+13] - piCur[n+13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+14] - piCur[n+14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+15] - piCur[n+15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE32( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {

    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE64( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[32] - piCur[32]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[33] - piCur[33]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[34] - piCur[34]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[35] - piCur[35]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[36] - piCur[36]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[37] - piCur[37]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[38] - piCur[38]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[39] - piCur[39]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[40] - piCur[40]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[41] - piCur[41]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[42] - piCur[42]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[43] - piCur[43]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[44] - piCur[44]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[45] - piCur[45]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[46] - piCur[46]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[47] - piCur[47]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[48] - piCur[48]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[49] - piCur[49]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[50] - piCur[50]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[51] - piCur[51]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[52] - piCur[52]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[53] - piCur[53]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[54] - piCur[54]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[55] - piCur[55]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[56] - piCur[56]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[57] - piCur[57]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[58] - piCur[58]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[59] - piCur[59]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[60] - piCur[60]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[61] - piCur[61]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[62] - piCur[62]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[63] - piCur[63]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

// --------------------------------------------------------------------------------------------------------------------
// HADAMARD with step (used in fractional search)
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xCalcHADs2x2( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  Distortion satd = 0;
  TCoeff diff[4], m[4];

  diff[0] = piOrg[0             ] - piCur[0];
  diff[1] = piOrg[1             ] - piCur[1];
  diff[2] = piOrg[iStrideOrg    ] - piCur[0 + iStrideCur];
  diff[3] = piOrg[iStrideOrg + 1] - piCur[1 + iStrideCur];
  m[0] = diff[0] + diff[2];
  m[1] = diff[1] + diff[3];
  m[2] = diff[0] - diff[2];
  m[3] = diff[1] - diff[3];
  
  satd += abs(m[0] + m[1]) >> 2;
  satd += abs(m[0] - m[1]);
  satd += abs(m[2] + m[3]);
  satd += abs(m[2] - m[3]);

  return satd;
}

static Distortion xCalcHADs4x4( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k;
  Distortion satd = 0;
  TCoeff diff[16], m[16], d[16];

  for( k = 0; k < 16; k+=4 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  /*===== hadamard transform =====*/
  m[ 0] = diff[ 0] + diff[12];
  m[ 1] = diff[ 1] + diff[13];
  m[ 2] = diff[ 2] + diff[14];
  m[ 3] = diff[ 3] + diff[15];
  m[ 4] = diff[ 4] + diff[ 8];
  m[ 5] = diff[ 5] + diff[ 9];
  m[ 6] = diff[ 6] + diff[10];
  m[ 7] = diff[ 7] + diff[11];
  m[ 8] = diff[ 4] - diff[ 8];
  m[ 9] = diff[ 5] - diff[ 9];
  m[10] = diff[ 6] - diff[10];
  m[11] = diff[ 7] - diff[11];
  m[12] = diff[ 0] - diff[12];
  m[13] = diff[ 1] - diff[13];
  m[14] = diff[ 2] - diff[14];
  m[15] = diff[ 3] - diff[15];

  d[ 0] = m[ 0] + m[ 4];
  d[ 1] = m[ 1] + m[ 5];
  d[ 2] = m[ 2] + m[ 6];
  d[ 3] = m[ 3] + m[ 7];
  d[ 4] = m[ 8] + m[12];
  d[ 5] = m[ 9] + m[13];
  d[ 6] = m[10] + m[14];
  d[ 7] = m[11] + m[15];
  d[ 8] = m[ 0] - m[ 4];
  d[ 9] = m[ 1] - m[ 5];
  d[10] = m[ 2] - m[ 6];
  d[11] = m[ 3] - m[ 7];
  d[12] = m[12] - m[ 8];
  d[13] = m[13] - m[ 9];
  d[14] = m[14] - m[10];
  d[15] = m[15] - m[11];

  m[ 0] = d[ 0] + d[ 3];
  m[ 1] = d[ 1] + d[ 2];
  m[ 2] = d[ 1] - d[ 2];
  m[ 3] = d[ 0] - d[ 3];
  m[ 4] = d[ 4] + d[ 7];
  m[ 5] = d[ 5] + d[ 6];
  m[ 6] = d[ 5] - d[ 6];
  m[ 7] = d[ 4] - d[ 7];
  m[ 8] = d[ 8] + d[11];
  m[ 9] = d[ 9] + d[10];
  m[10] = d[ 9] - d[10];
  m[11] = d[ 8] - d[11];
  m[12] = d[12] + d[15];
  m[13] = d[13] + d[14];
  m[14] = d[13] - d[14];
  m[15] = d[12] - d[15];

  d[ 0] = m[ 0] + m[ 1];
  d[ 1] = m[ 0] - m[ 1];
  d[ 2] = m[ 2] + m[ 3];
  d[ 3] = m[ 3] - m[ 2];
  d[ 4] = m[ 4] + m[ 5];
  d[ 5] = m[ 4] - m[ 5];
  d[ 6] = m[ 6] + m[ 7];
  d[ 7] = m[ 7] - m[ 6];
  d[ 8] = m[ 8] + m[ 9];
  d[ 9] = m[ 8] - m[ 9];
  d[10] = m[10] + m[11];
  d[11] = m[11] - m[10];
  d[12] = m[12] + m[13];
  d[13] = m[12] - m[13];
  d[14] = m[14] + m[15];
  d[15] = m[15] - m[14];

  for (k=0; k<16; ++k)
  {
    satd += abs(d[k]);
  }

  satd -= abs( d[0] );
  satd += abs( d[0] ) >> 2;
  satd = ((satd+1)>>1);

  return satd;
}

static Distortion xCalcHADs8x8( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj;
  Distortion sad = 0;
  TCoeff diff[64], m1[8][8], m2[8][8], m3[8][8];

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      sad += abs(m2[i][j]);
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad=((sad+2)>>2);

  return sad;
}

static Distortion xCalcHADs16x8( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{   //need to add SIMD implementation ,JCA
  int k, i, j, jj, sad = 0;
  int diff[128], m1[8][16], m2[8][16];
  for( k = 0; k < 128; k += 16 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    diff[k + 8] = piOrg[8] - piCur[8];
    diff[k + 9] = piOrg[9] - piCur[9];
    diff[k + 10] = piOrg[10] - piCur[10];
    diff[k + 11] = piOrg[11] - piCur[11];
    diff[k + 12] = piOrg[12] - piCur[12];
    diff[k + 13] = piOrg[13] - piCur[13];
    diff[k + 14] = piOrg[14] - piCur[14];
    diff[k + 15] = piOrg[15] - piCur[15];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 8; j++ )
  {
    jj = j << 4;

    m2[j][0] = diff[jj    ] + diff[jj + 8];
    m2[j][1] = diff[jj + 1] + diff[jj + 9];
    m2[j][2] = diff[jj + 2] + diff[jj + 10];
    m2[j][3] = diff[jj + 3] + diff[jj + 11];
    m2[j][4] = diff[jj + 4] + diff[jj + 12];
    m2[j][5] = diff[jj + 5] + diff[jj + 13];
    m2[j][6] = diff[jj + 6] + diff[jj + 14];
    m2[j][7] = diff[jj + 7] + diff[jj + 15];
    m2[j][8] = diff[jj    ] - diff[jj + 8];
    m2[j][9] = diff[jj + 1] - diff[jj + 9];
    m2[j][10] = diff[jj + 2] - diff[jj + 10];
    m2[j][11] = diff[jj + 3] - diff[jj + 11];
    m2[j][12] = diff[jj + 4] - diff[jj + 12];
    m2[j][13] = diff[jj + 5] - diff[jj + 13];
    m2[j][14] = diff[jj + 6] - diff[jj + 14];
    m2[j][15] = diff[jj + 7] - diff[jj + 15];

    m1[j][0] = m2[j][0] + m2[j][4];
    m1[j][1] = m2[j][1] + m2[j][5];
    m1[j][2] = m2[j][2] + m2[j][6];
    m1[j][3] = m2[j][3] + m2[j][7];
    m1[j][4] = m2[j][0] - m2[j][4];
    m1[j][5] = m2[j][1] - m2[j][5];
    m1[j][6] = m2[j][2] - m2[j][6];
    m1[j][7] = m2[j][3] - m2[j][7];
    m1[j][8] = m2[j][8] + m2[j][12];
    m1[j][9] = m2[j][9] + m2[j][13];
    m1[j][10] = m2[j][10] + m2[j][14];
    m1[j][11] = m2[j][11] + m2[j][15];
    m1[j][12] = m2[j][8] - m2[j][12];
    m1[j][13] = m2[j][9] - m2[j][13];
    m1[j][14] = m2[j][10] - m2[j][14];
    m1[j][15] = m2[j][11] - m2[j][15];

    m2[j][0] = m1[j][0] + m1[j][2];
    m2[j][1] = m1[j][1] + m1[j][3];
    m2[j][2] = m1[j][0] - m1[j][2];
    m2[j][3] = m1[j][1] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][6];
    m2[j][5] = m1[j][5] + m1[j][7];
    m2[j][6] = m1[j][4] - m1[j][6];
    m2[j][7] = m1[j][5] - m1[j][7];
    m2[j][8] = m1[j][8] + m1[j][10];
    m2[j][9] = m1[j][9] + m1[j][11];
    m2[j][10] = m1[j][8] - m1[j][10];
    m2[j][11] = m1[j][9] - m1[j][11];
    m2[j][12] = m1[j][12] + m1[j][14];
    m2[j][13] = m1[j][13] + m1[j][15];
    m2[j][14] = m1[j][12] - m1[j][14];
    m2[j][15] = m1[j][13] - m1[j][15];

    m1[j][0] = m2[j][0] + m2[j][1];
    m1[j][1] = m2[j][0] - m2[j][1];
    m1[j][2] = m2[j][2] + m2[j][3];
    m1[j][3] = m2[j][2] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][5];
    m1[j][5] = m2[j][4] - m2[j][5];
    m1[j][6] = m2[j][6] + m2[j][7];
    m1[j][7] = m2[j][6] - m2[j][7];
    m1[j][8] = m2[j][8] + m2[j][9];
    m1[j][9] = m2[j][8] - m2[j][9];
    m1[j][10] = m2[j][10] + m2[j][11];
    m1[j][11] = m2[j][10] - m2[j][11];
    m1[j][12] = m2[j][12] + m2[j][13];
    m1[j][13] = m2[j][12] - m2[j][13];
    m1[j][14] = m2[j][14] + m2[j][15];
    m1[j][15] = m2[j][14] - m2[j][15];
  }

  //vertical
  for( i = 0; i < 16; i++ )
  {
    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for( i = 0; i < 8; i++ )
  {
    for( j = 0; j < 16; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad = ( int ) ( sad / sqrt( 16.0 * 8 ) * 2 );

  return sad;
}

static Distortion xCalcHADs8x16( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[128], m1[16][8], m2[16][8];
  for( k = 0; k < 128; k += 8 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 16; j++ )
  {
    jj = j << 3;

    m2[j][0] = diff[jj] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for( i = 0; i < 8; i++ )
  {
    m1[0][i] = m2[0][i] + m2[8][i];
    m1[1][i] = m2[1][i] + m2[9][i];
    m1[2][i] = m2[2][i] + m2[10][i];
    m1[3][i] = m2[3][i] + m2[11][i];
    m1[4][i] = m2[4][i] + m2[12][i];
    m1[5][i] = m2[5][i] + m2[13][i];
    m1[6][i] = m2[6][i] + m2[14][i];
    m1[7][i] = m2[7][i] + m2[15][i];
    m1[8][i] = m2[0][i] - m2[8][i];
    m1[9][i] = m2[1][i] - m2[9][i];
    m1[10][i] = m2[2][i] - m2[10][i];
    m1[11][i] = m2[3][i] - m2[11][i];
    m1[12][i] = m2[4][i] - m2[12][i];
    m1[13][i] = m2[5][i] - m2[13][i];
    m1[14][i] = m2[6][i] - m2[14][i];
    m1[15][i] = m2[7][i] - m2[15][i];

    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];
    m2[8][i] = m1[8][i] + m1[12][i];
    m2[9][i] = m1[9][i] + m1[13][i];
    m2[10][i] = m1[10][i] + m1[14][i];
    m2[11][i] = m1[11][i] + m1[15][i];
    m2[12][i] = m1[8][i] - m1[12][i];
    m2[13][i] = m1[9][i] - m1[13][i];
    m2[14][i] = m1[10][i] - m1[14][i];
    m2[15][i] = m1[11][i] - m1[15][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];
    m1[8][i] = m2[8][i] + m2[10][i];
    m1[9][i] = m2[9][i] + m2[11][i];
    m1[10][i] = m2[8][i] - m2[10][i];
    m1[11][i] = m2[9][i] - m2[11][i];
    m1[12][i] = m2[12][i] + m2[14][i];
    m1[13][i] = m2[13][i] + m2[15][i];
    m1[14][i] = m2[12][i] - m2[14][i];
    m1[15][i] = m2[13][i] - m2[15][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
    m2[8][i] = m1[8][i] + m1[9][i];
    m2[9][i] = m1[8][i] - m1[9][i];
    m2[10][i] = m1[10][i] + m1[11][i];
    m2[11][i] = m1[10][i] - m1[11][i];
    m2[12][i] = m1[12][i] + m1[13][i];
    m2[13][i] = m1[12][i] - m1[13][i];
    m2[14][i] = m1[14][i] + m1[15][i];
    m2[15][i] = m1[14][i] - m1[15][i];
  }

  for( i = 0; i < 16; i++ )
  {
    for( j = 0; j < 8; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad = ( int ) ( sad / sqrt( 16.0 * 8 ) * 2 );

  return sad;
}

static Distortion xCalcHADs4x8( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[32], m1[8][4], m2[8][4];
  for( k = 0; k < 32; k += 4 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 8; j++ )
  {
    jj = j << 2;
    m2[j][0] = diff[jj] + diff[jj + 2];
    m2[j][1] = diff[jj + 1] + diff[jj + 3];
    m2[j][2] = diff[jj] - diff[jj + 2];
    m2[j][3] = diff[jj + 1] - diff[jj + 3];

    m1[j][0] = m2[j][0] + m2[j][1];
    m1[j][1] = m2[j][0] - m2[j][1];
    m1[j][2] = m2[j][2] + m2[j][3];
    m1[j][3] = m2[j][2] - m2[j][3];
  }

  //vertical
  for( i = 0; i < 4; i++ )
  {
    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for( i = 0; i < 8; i++ )
  {
    for( j = 0; j < 4; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad = ( int ) ( sad / sqrt( 4.0 * 8 ) * 2 );

  return sad;
}

static Distortion xCalcHADs8x4( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[32], m1[4][8], m2[4][8];
  for( k = 0; k < 32; k += 8 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 4; j++ )
  {
    jj = j << 3;

    m2[j][0] = diff[jj] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for( i = 0; i < 8; i++ )
  {
    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
  }

  for( i = 0; i < 4; i++ )
  {
    for( j = 0; j < 8; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad = ( int ) ( sad / sqrt( 4.0 * 8 ) * 2 );

  return sad;
}

Distortion RdCost::xGetHAD2SADs( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  Distortion distHad = xGetHADs( rcDtParam );
  Distortion distSad = 0;
  {
    CHECKD( (rcDtParam.org.width != rcDtParam.org.stride) || (rcDtParam.cur.stride != rcDtParam.org.stride) , "this functions assumes compact, aligned buffering");

    const Pel* piOrg  = rcDtParam.org.buf;
    const Pel* piCur  = rcDtParam.cur.buf;
    int  iRows        = rcDtParam.org.height>>2;
    int  iCols        = rcDtParam.org.width<<2;

    Distortion uiSum = 0;

    for( int y = 0; y < iRows;  y++ )
    {
      for (int n = 0; n < iCols; n+=16 )
      {
        uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] );
        uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] );
        uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] );
        uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] );
        uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] );
        uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] );
        uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] );
        uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] );
        uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] );
        uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] );
        uiSum += abs( piOrg[n+10] - piCur[n+10] );
        uiSum += abs( piOrg[n+11] - piCur[n+11] );
        uiSum += abs( piOrg[n+12] - piCur[n+12] );
        uiSum += abs( piOrg[n+13] - piCur[n+13] );
        uiSum += abs( piOrg[n+14] - piCur[n+14] );
        uiSum += abs( piOrg[n+15] - piCur[n+15] );
      }
      piOrg += iCols;
      piCur += iCols;
    }

    distSad = (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
  }

  return std::min( distHad, 2*distSad);
}

Distortion RdCost::xGetHADs( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iRows = rcDtParam.org.height;
  const int  iCols = rcDtParam.org.width;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;

  int  x = 0, y = 0;

  Distortion uiSum = 0;

  if( iCols > iRows && ( iRows & 7 ) == 0 && ( iCols & 15 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHADs16x8( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 8;
      piCur += iStrideCur * 8;
    }
  }
  else if( iCols < iRows && ( iCols & 7 ) == 0 && ( iRows & 15 ) == 0 )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x16( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 16;
      piCur += iStrideCur * 16;
    }
  }
  else if( iCols > iRows && ( iRows & 3 ) == 0 && ( iCols & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x4( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 4;
      piCur += iStrideCur * 4;
    }
  }
  else if( iCols < iRows && ( iCols & 3 ) == 0 && ( iRows & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHADs4x8( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 8;
      piCur += iStrideCur * 8;
    }
  }
  else if( ( iRows % 8 == 0 ) && ( iCols % 8 == 0 ) )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x8( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8*iStrideOrg;
      piCur += 8*iStrideCur;
    }
  }
  else if( ( iRows % 4 == 0 ) && ( iCols % 4 == 0 ) )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHADs4x4( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4*iStrideOrg;
      piCur += 4*iStrideCur;
    }
  }
  else if( ( iRows % 2 == 0 ) && ( iCols % 2 == 0 ) )
  {
    for( y = 0; y < iRows; y += 2 )
    {
      for( x = 0; x < iCols; x += 2 )
      {
        uiSum += xCalcHADs2x2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 2*iStrideOrg;
      piCur += 2*iStrideCur;
    }
  }
  else
  {
    THROW( "Invalid size" );
  }

  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


void RdCost::saveUnadjustedLambda()
{
  m_dLambda_unadjusted = m_dLambda;
  m_DistScaleUnadjusted = m_DistScale;
}


inline Distortion getWeightedMSE(const Pel org, const Pel cur, const int64_t fixedPTweight, unsigned uiShift)
{
  const Intermediate_Int iTemp = org - cur;
  return Intermediate_Int((fixedPTweight*(iTemp*iTemp) + (1 << 15)) >> uiShift);
}

Distortion RdCost::xGetSSE_WTD( const DistParam &rcDtParam ) const
{
  if( rcDtParam.applyWeight )
  {
    THROW("no support");
  }
        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iCols = rcDtParam.org.width;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma        = rcDtParam.orgLuma->buf;
  const int  iStrideOrgLuma   = rcDtParam.orgLuma->stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = 16 + (DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1);

  // cf, column factor, offset of the second column, to be set to '0' for width of '1'
  const int cf =  1 - ( iCols & 1 );
  CHECK( ( iCols & 1 ) && iCols != 1, "Width can only be even or equal to '1'!" );

  if ((m_signalType == RESHAPE_SIGNAL_SDR || m_signalType == RESHAPE_SIGNAL_HLG) && rcDtParam.compID != COMP_Y)
  {
    const int64_t fixedPTweight = (int64_t)(m_chromaWeight * (double)(1 << 16));

    for( ; iRows != 0; iRows-- )
    {
      for (int n = 0; n < iCols; n+=2 )
      {
        uiSum += getWeightedMSE( piOrg[n   ], piCur[n   ], fixedPTweight, uiShift );
        uiSum += getWeightedMSE( piOrg[n+cf], piCur[n+cf], fixedPTweight, uiShift );
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
    }
  }
  else if( rcDtParam.compID == COMP_Y )
  {
    for( ; iRows != 0; iRows-- )
    {
      for (int n = 0; n < iCols; n+=2 )
      {
        uiSum += getWeightedMSE( piOrg[n   ], piCur[n   ], m_reshapeLumaLevelToWeightPLUT[piOrgLuma[n   ]], uiShift );
        uiSum += getWeightedMSE( piOrg[n+cf], piCur[n+cf], m_reshapeLumaLevelToWeightPLUT[piOrgLuma[n+cf]], uiShift );
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
      piOrgLuma += iStrideOrgLuma;
    }
  }
  else
  {
    const ComponentID compId = rcDtParam.compID;
    const size_t  cShiftX = getComponentScaleX(compId,  m_cf);
    const size_t  cShiftY = getComponentScaleY(compId,  m_cf);

    for( ; iRows != 0; iRows-- )
    {
      for (int n = 0; n < iCols; n+=2 )
      {
        uiSum += getWeightedMSE( piOrg[n   ], piCur[n   ], m_reshapeLumaLevelToWeightPLUT[piOrgLuma[(n   )<<cShiftX]], uiShift );
        uiSum += getWeightedMSE( piOrg[n+cf], piCur[n+cf], m_reshapeLumaLevelToWeightPLUT[piOrgLuma[(n+cf)<<cShiftX]], uiShift );
      }
      piOrg += iStrideOrg;
      piCur += iStrideCur;
      piOrgLuma += iStrideOrgLuma<<cShiftY;
    }
  }

  // if the width is '1', cf is '0', the differences are counted double and need to be normalized
  return ( uiSum >> ( 1 - cf ) );
}

void RdCost::setDistParamGeo(DistParam &rcDP, const CPelBuf &org, const Pel *piRefY, int iRefStride, const Pel *mask,
                          int iMaskStride, int stepX, int iMaskStride2, int bitDepth, ComponentID compID)
{
  rcDP.bitDepth = bitDepth;
  rcDP.compID   = compID;

  // set Original & Curr Pointer / Stride
  rcDP.org        = org;
  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;

  // set Mask
  rcDP.mask        = mask;
  rcDP.maskStride  = iMaskStride;
  rcDP.stepX       = stepX;
  rcDP.maskStride2 = iMaskStride2;

  // set Block Width / Height
  rcDP.cur.width                     = org.width;
  rcDP.cur.height                    = org.height;
  rcDP.maximumDistortionForEarlyExit = MAX_DISTORTION;

  // set Cost function for motion estimation with Mask
  rcDP.distFunc = m_afpDistortFunc[0][DF_SAD_WITH_MASK];
}

Distortion RdCost::xGetSADwMask(const DistParam &rcDtParam)
{
  const Pel *    org             = rcDtParam.org.buf;
  const Pel *    cur             = rcDtParam.cur.buf;
  const Pel *    mask            = rcDtParam.mask;
  const int      cols            = rcDtParam.org.width;
  int            rows            = rcDtParam.org.height;
  const int      subShift        = rcDtParam.subShift;
  const int      subStep         = (1 << subShift);
  const int      strideCur       = rcDtParam.cur.stride * subStep;
  const int      strideOrg       = rcDtParam.org.stride * subStep;
  const int      strideMask      = rcDtParam.maskStride * subStep;
  const int      stepX           = rcDtParam.stepX;
  const int      strideMask2     = rcDtParam.maskStride2;
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  Distortion sum = 0;
  for (; rows != 0; rows -= subStep)
  {
    for (int n = 0; n < cols; n++)
    {
      sum += abs(org[n] - cur[n]) * *mask;
      mask += stepX;
    }
    org += strideOrg;
    cur += strideCur;
    mask += strideMask;
    mask += strideMask2;
  }
  sum <<= subShift;
  return (sum >> distortionShift);
}

Distortion RdCost::getBvCostMultiplePredsIBC(int x, int y, bool useIMV)
{
  return Distortion(m_dCostIBC * getBitsMultiplePredsIBC(x, y, useIMV));
}

static inline unsigned getIComponentBitsIBC( int val )
{
  if( !val ) return 1;

  const unsigned int l2 = floorLog2( (val <= 0) ? (-val << 1) + 1 : (val << 1) );

  return (l2 << 1) + 1;
}

unsigned int RdCost::getBitsMultiplePredsIBC(int x, int y, bool useIMV)
{
  int rmvH[2];
  int rmvV[2];
  rmvH[0] = x - m_bvPredictors[0].hor;
  rmvH[1] = x - m_bvPredictors[1].hor;

  rmvV[0] = y - m_bvPredictors[0].ver;
  rmvV[1] = y - m_bvPredictors[1].ver;
  int absCand[2];
  absCand[0] = abs(rmvH[0]) + abs(rmvV[0]);
  absCand[1] = abs(rmvH[1]) + abs(rmvV[1]);

  if (useIMV && x % 4 == 0 && y % 4 == 0)
  {
    int rmvHQP[2];
    int rmvVQP[2];

    int imvShift = 2;
    int offset = 1 << (imvShift - 1);

    rmvHQP[0] = (x >> 2) - ((m_bvPredictors[0].hor + offset) >> 2);
    rmvHQP[1] = (x >> 2) - ((m_bvPredictors[1].hor + offset) >> 2);
    rmvVQP[0] = (y >> 2) - ((m_bvPredictors[0].ver + offset) >> 2);
    rmvVQP[1] = (y >> 2) - ((m_bvPredictors[1].ver + offset) >> 2);

    int absCandQP[2];
    absCandQP[0] = abs(rmvHQP[0]) + abs(rmvVQP[0]);
    absCandQP[1] = abs(rmvHQP[1]) + abs(rmvVQP[1]);
    unsigned int candBits0QP, candBits1QP;
    if (absCand[0] < absCand[1])
    {
      unsigned int candBits0 = getIComponentBitsIBC(rmvH[0]) + getIComponentBitsIBC(rmvV[0]);
      if (absCandQP[0] < absCandQP[1])
      {
        candBits0QP = getIComponentBitsIBC(rmvHQP[0]) + getIComponentBitsIBC(rmvVQP[0]);
        return candBits0QP < candBits0 ? candBits0QP : candBits0;
      }
      else
      {
        candBits1QP = getIComponentBitsIBC(rmvHQP[1]) + getIComponentBitsIBC(rmvVQP[1]);
        return candBits1QP < candBits0 ? candBits1QP : candBits0;
      }
    }
    else
    {
      unsigned int candBits1 = getIComponentBitsIBC(rmvH[1]) + getIComponentBitsIBC(rmvV[1]);
      if (absCandQP[0] < absCandQP[1])
      {
        candBits0QP = getIComponentBitsIBC(rmvHQP[0]) + getIComponentBitsIBC(rmvVQP[0]);
        return candBits0QP < candBits1 ? candBits0QP : candBits1;
      }
      else
      {
        candBits1QP = getIComponentBitsIBC(rmvHQP[1]) + getIComponentBitsIBC(rmvVQP[1]);
        return candBits1QP < candBits1 ? candBits1QP : candBits1;
      }
    }
  }
  else
  {
    if (absCand[0] < absCand[1])
    {
      return getIComponentBitsIBC(rmvH[0]) + getIComponentBitsIBC(rmvV[0]);
    }
    else
    {
      return getIComponentBitsIBC(rmvH[1]) + getIComponentBitsIBC(rmvV[1]);
    }
  }
}

} // namespace vvenc

//! \}


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


/** \file     LoopFilter.cpp
    \brief    deblocking filter
*/

#include "LoopFilter.h"
#include "Slice.h"
#include "Mv.h"
#include "Unit.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

#include "Quant.h"

#ifdef TARGET_SIMD_X86
#include "CommonDefX86.h"
#endif

namespace vvenc {

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define DEFAULT_INTRA_TC_OFFSET 2 ///< Default intra TC offset

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint16_t LoopFilter::sm_tcTable[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,4,4,4,4,5,5,5,5,7,7,8,9,10,10,11,13,14,15,17,19,21,24,25,29,33,36,41,45,51,57,64,71,80,89,100,112,125,141,157,177,198,222,250,280,314,352,395
};
const uint8_t LoopFilter::sm_betaTable[MAX_QP + 1] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
  , 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88
};

// ====================================================================================================================
// utility functions
// ====================================================================================================================

#define INCX( ptr, stride ) { ptr++; }
#define INCY( ptr, stride ) { ptr += ( stride ); }
#define OFFSETX( ptr, stride, x ) { ptr += ( x ); }
#define OFFSETY( ptr, stride, y ) { ptr += ( y ) * ( stride ); }
#define OFFSET( ptr, stride, x, y ) { ptr += ( x ) + ( y ) * ( stride ); }
#define GET_OFFSETX( ptr, stride, x ) ( ( ptr ) + ( x ) )
#define GET_OFFSETY( ptr, stride, y ) ( ( ptr ) + ( y ) * ( stride ) )
#define GET_OFFSET( ptr, stride, x, y ) ( ( ptr ) + ( x ) + ( y ) * ( stride ) )

static bool isAvailable( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction )
{
  return ( ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) ) && ( !bEnforceTileRestriction || CU::isSameTile(cu, cu2) ) );
}

#define BsSet( val, compIdx ) (   ( val ) << ( ( compIdx ) << 1 ) )     
#define BsGet( val, compIdx ) ( ( ( val ) >> ( ( compIdx ) << 1 ) ) & 3 )

static const int dbCoeffs7[7] = { 59, 50, 41, 32, 23, 14,  5 };
static const int dbCoeffs5[5] = { 58, 45, 32, 19,  6 };
static const int dbCoeffs3[3] = { 53, 32, 11 };

static inline void xBilinearFilter( Pel* srcP, Pel* srcQ, ptrdiff_t offset, int refMiddle, int refP, int refQ, int numberPSide, int numberQSide, const int* dbCoeffsP, const int* dbCoeffsQ, int tc )
{
  int src;
  const char tc7[7] = { 6, 5, 4, 3, 2, 1, 1 };
  const char tc3[3] = { 6, 4, 2 };
  const char *tcP = ( numberPSide == 3 ) ? tc3 : tc7;
  const char *tcQ = ( numberQSide == 3 ) ? tc3 : tc7;

  for( int pos = 0; pos < numberPSide; pos++ )
  {
    src = srcP[-offset * pos];
    int cvalue = ( tc * tcP[pos] ) >> 1;
    srcP[-offset * pos] = Clip3( src - cvalue, src + cvalue, ( ( refMiddle * dbCoeffsP[pos] + refP * ( 64 - dbCoeffsP[pos] ) + 32 ) >> 6 ) );
  }

  for( int pos = 0; pos < numberQSide; pos++ )
  {
    src = srcQ[ offset * pos];
    int cvalue = ( tc * tcQ[pos] ) >> 1;
    srcQ[ offset * pos] = Clip3( src - cvalue, src + cvalue, ( ( refMiddle * dbCoeffsQ[pos] + refQ * ( 64 - dbCoeffsQ[pos] ) + 32 ) >> 6 ) );
  }
}

void xFilteringPandQCore( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc )
{
  CHECK( numberPSide <= 3 && numberQSide <= 3, "Short filtering in long filtering function" );
  CHECK( numberPSide != 3 && numberPSide != 5 && numberPSide != 7, "invalid numberPSide" );
  CHECK( numberQSide != 3 && numberQSide != 5 && numberQSide != 7, "invalid numberQSide" );

  const int*       dbCoeffsP    = numberPSide == 7 ? dbCoeffs7 : ( numberPSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const int*       dbCoeffsQ    = numberQSide == 7 ? dbCoeffs7 : ( numberQSide == 5 ) ? dbCoeffs5 : dbCoeffs3;

  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    Pel* srcP = src + step * i - offset;
    Pel* srcQ = src + step * i;


    int refP = 0;
    int refQ = 0;
    int refMiddle = 0;

    switch( numberPSide )
    {
    case 7: refP = ( srcP[-6 * offset] + srcP[-7 * offset] + 1 ) >> 1; break;
    case 3: refP = ( srcP[-2 * offset] + srcP[-3 * offset] + 1 ) >> 1; break;
    case 5: refP = ( srcP[-4 * offset] + srcP[-5 * offset] + 1 ) >> 1; break;
    }

    switch( numberQSide )
    {
    case 7: refQ = ( srcQ[6 * offset] + srcQ[7 * offset] + 1 ) >> 1; break;
    case 3: refQ = ( srcQ[2 * offset] + srcQ[3 * offset] + 1 ) >> 1; break;
    case 5: refQ = ( srcQ[4 * offset] + srcQ[5 * offset] + 1 ) >> 1; break;
    }

    if( numberPSide == numberQSide )
    {
      if( numberPSide == 5 )
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] ) + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + 8 ) >> 4;
      }
      else
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] ) + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + +srcP[-6 * offset] + srcQ[6 * offset] + 8 ) >> 4;
      }
    }
    else
    {
      Pel* srcPt = srcP;
      Pel* srcQt = srcQ;
      ptrdiff_t offsetP = -offset;
      ptrdiff_t offsetQ = offset;

      int newNumberQSide = numberQSide;
      int newNumberPSide = numberPSide;

      if( numberQSide > numberPSide )
      {
        std::swap( srcPt, srcQt );
        std::swap( offsetP, offsetQ );
        newNumberQSide = numberPSide;
        newNumberPSide = numberQSide;
      }

      if( newNumberPSide == 7 && newNumberQSide == 5 )
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] ) + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + 8 ) >> 4;
      }
      else if( newNumberPSide == 7 && newNumberQSide == 3 )
      {
        refMiddle = ( 2 * ( srcPt[0] + srcQt[0] ) + srcQt[0] + 2 * ( srcQt[offsetQ] + srcQt[2 * offsetQ] ) + srcPt[offsetP] + srcQt[offsetQ] + srcPt[2 * offsetP] + srcPt[3 * offsetP] + srcPt[4 * offsetP] + srcPt[5 * offsetP] + srcPt[6 * offsetP] + 8 ) >> 4;
      }
      else //if (newNumberPSide == 5 && newNumberQSide == 3)
      {
        refMiddle = ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + 4 ) >> 3;
      }
    }

    xBilinearFilter( srcP, srcQ, offset, refMiddle, refP, refQ, numberPSide, numberQSide, dbCoeffsP, dbCoeffsQ, tc );
  }
}

/**
- Deblocking for the luminance component with strong or weak filter
.
\param piSrc           pointer to picture data
\param iOffset         offset value for picture data
\param tc              tc value
\param sw              decision strong/weak filter
\param bPartPNoFilter  indicator to disable filtering on partP
\param bPartQNoFilter  indicator to disable filtering on partQ
\param iThrCut         threshold value for weak filter decision
\param bFilterSecondP  decision weak filter/no filter for partP
\param bFilterSecondQ  decision weak filter/no filter for partQ
\param bitDepthLuma    luma bit depth
*/
void xPelFilterLumaCorePel( Pel* piSrc, const ptrdiff_t iOffset, const int tc, const bool sw,const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng )
{
  const Pel m0  = piSrc[-4 * iOffset];
  const Pel m1  = piSrc[-3 * iOffset];
  const Pel m2  = piSrc[-2 * iOffset];
  const Pel m3  = piSrc[-1 * iOffset];
  const Pel m4  = piSrc[ 0          ];
  const Pel m5  = piSrc[ 1 * iOffset];
  const Pel m6  = piSrc[ 2 * iOffset];
  const Pel m7  = piSrc[ 3 * iOffset];

  const char tc3[3] = { 3, 2, 1 };

  if (sw)
  {
    piSrc[-3 * iOffset ] = Clip3( m1 - tc3[2] * tc, m1 + tc3[2] * tc, ( 2*m0 + 3*m1 +   m2 +   m3 +   m4                      + 4 ) >> 3 );
    piSrc[-2 * iOffset ] = Clip3( m2 - tc3[1] * tc, m2 + tc3[1] * tc, (          m1 +   m2 +   m3 +   m4                      + 2 ) >> 2 );
    piSrc[-1 * iOffset ] = Clip3( m3 - tc3[0] * tc, m3 + tc3[0] * tc, (          m1 + 2*m2 + 2*m3 + 2*m4 +   m5               + 4 ) >> 3 );
    piSrc[ 0           ] = Clip3( m4 - tc3[0] * tc, m4 + tc3[0] * tc, (                 m2 + 2*m3 + 2*m4 + 2*m5 +   m6        + 4 ) >> 3 );
    piSrc[ 1 * iOffset ] = Clip3( m5 - tc3[1] * tc, m5 + tc3[1] * tc, (                        m3 +   m4 +   m5 +   m6        + 2 ) >> 2 );
    piSrc[ 2 * iOffset ] = Clip3( m6 - tc3[2] * tc, m6 + tc3[2] * tc, (                        m3 +   m4 +   m5 + 3*m6 + 2*m7 + 4 ) >> 3 );
  }
  else
  {
    /* Weak filter */
    int delta = ( 9 * ( m4 - m3 ) - 3 * ( m5 - m2 ) + 8 ) >> 4;

    if ( abs(delta) < iThrCut )
    {
      delta = Clip3( -tc, tc, delta );
      const int tc2 = tc >> 1;

      piSrc[-iOffset * 1]   = ClipPel( m3 + delta, clpRng);
      if( bFilterSecondP )
      {
        const int delta1    = Clip3( -tc2, tc2, ( ( ( ( m1 + m3 + 1 ) >> 1 ) - m2 + delta ) >> 1 ) );
        piSrc[-iOffset * 2] = ClipPel( m2 + delta1, clpRng);
      }

      piSrc[0]              = ClipPel( m4 - delta, clpRng);
      if( bFilterSecondQ )
      {
        const int delta2    = Clip3( -tc2, tc2, ( ( ( ( m6 + m4 + 1 ) >> 1 ) - m5 - delta ) >> 1 ) );
        piSrc[iOffset]      = ClipPel( m5 + delta2, clpRng);
      }
    }
  }
}

inline void xPelFilterLumaCore( Pel* piSrc, const ptrdiff_t step, const ptrdiff_t offset, const int tc, const bool sw, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng )
{
  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    xPelFilterLumaCorePel( piSrc + step * i, offset, tc, sw, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );
  }
}

/**
- Deblocking of one line/column for the chrominance component
.
\param piSrc           pointer to picture data
\param iOffset         offset value for picture data
\param tc              tc value
\param bPartPNoFilter  indicator to disable filtering on partP
\param bPartQNoFilter  indicator to disable filtering on partQ
\param bitDepthChroma  chroma bit depth
*/
static inline void xPelFilterChroma( Pel* piSrc, const ptrdiff_t iOffset, const int tc, const bool sw, const bool bPartPNoFilter, const bool bPartQNoFilter, const ClpRng& clpRng, const bool largeBoundary, const bool isChromaHorCTBBoundary )
{
  int delta;

  const Pel m0 = piSrc[-iOffset * 4];
  const Pel m1 = piSrc[-iOffset * 3];
  const Pel m2 = piSrc[-iOffset * 2];
  const Pel m3 = piSrc[-iOffset];
  const Pel m4 = piSrc[0];
  const Pel m5 = piSrc[iOffset];
  const Pel m6 = piSrc[iOffset * 2];
  const Pel m7 = piSrc[iOffset * 3];

  if (sw)
  {
    if (isChromaHorCTBBoundary)
    {
      piSrc[-iOffset * 1] = Clip3(m3 - tc, m3 + tc, ((3 * m2 + 2 * m3 + m4 + m5 + m6 + 4) >> 3)); // p0
      piSrc[0] = Clip3(m4 - tc, m4 + tc, ((2 * m2 + m3 + 2 * m4 + m5 + m6 + m7 + 4) >> 3)); // q0
      piSrc[iOffset * 1] = Clip3(m5 - tc, m5 + tc, ((m2 + m3 + m4 + 2 * m5 + m6 + 2 * m7 + 4) >> 3));  // q1
      piSrc[iOffset * 2] = Clip3(m6 - tc, m6 + tc, ((m3 + m4 + m5 + 2 * m6 + 3 * m7 + 4) >> 3));       // q2
    }
    else
    {
    piSrc[-iOffset * 3] = Clip3(m1 - tc, m1 + tc, ((3 * m0 + 2 * m1 + m2 + m3 + m4 + 4) >> 3));       // p2
    piSrc[-iOffset * 2] = Clip3(m2 - tc, m2 + tc, ((2 * m0 + m1 + 2 * m2 + m3 + m4 + m5 + 4) >> 3));  // p1
    piSrc[-iOffset * 1] = Clip3(m3 - tc, m3 + tc, ((m0 + m1 + m2 + 2 * m3 + m4 + m5 + m6 + 4) >> 3)); // p0
    piSrc[0]            = Clip3(m4 - tc, m4 + tc, ((m1 + m2 + m3 + 2 * m4 + m5 + m6 + m7 + 4) >> 3)); // q0
    piSrc[iOffset * 1]  = Clip3(m5 - tc, m5 + tc, ((m2 + m3 + m4 + 2 * m5 + m6 + 2 * m7 + 4) >> 3));  // q1
    piSrc[iOffset * 2]  = Clip3(m6 - tc, m6 + tc, ((m3 + m4 + m5 + 2 * m6 + 3 * m7 + 4) >> 3));       // q2
  }
  }
  else
  {
    delta = Clip3(-tc, tc, ((((m4 - m3) << 2) + m2 - m5 + 4) >> 3));
    piSrc[-iOffset] = ClipPel(m3 + delta, clpRng);
    piSrc[0] = ClipPel(m4 - delta, clpRng);
  }


  if( bPartPNoFilter )
  {
    if (largeBoundary)
    {
      piSrc[-iOffset * 3] = m1; // p2
      piSrc[-iOffset * 2] = m2; // p1
    }
    piSrc[-iOffset] = m3;
  }
  if( bPartQNoFilter )
  {
    if (largeBoundary)
    {
      piSrc[iOffset * 1] = m5; // q1
      piSrc[iOffset * 2] = m6; // q2
    }
    piSrc[ 0      ] = m4;
  }
}
// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

LoopFilter::LoopFilter()
{
  m_origin[0] = Position{ 0, 0 };
  m_origin[1] = Position{ 0, 0 };

  xPelFilterLuma  = xPelFilterLumaCore;
  xFilteringPandQ = xFilteringPandQCore;

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_DBLF
  initLoopFilterX86();
#endif
}

LoopFilter::~LoopFilter()
{}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - call deblocking function for every CU
 .
 \param  pcPic   picture class (Pic) pointer
 */
void LoopFilter::loopFilterPic( CodingStructure& cs, bool calcFilterStrength ) const
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_DEBLOCK_FILTER );
  const PreCalcValues& pcv = *cs.pcv;


#if ENABLE_TRACING

  for (int y = 0; y < pcv.heightInCtus; y++)
  {
    for (int x = 0; x < pcv.widthInCtus; x++)
    {
      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUSizeLog2, y << pcv.maxCUSizeLog2, pcv.maxCUSize, pcv.maxCUSize ) );
      DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
      DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );

      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L, TREE_D ), CH_L ) )
        if( currCU.Y().valid() )
        {
          DTRACE(g_trace_ctx, D_CRC, "CU LumaPos %d %d", currCU.Y().x, currCU.Y().y);
          DTRACE_CCRC(g_trace_ctx, D_CRC, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Y()), COMP_Y, &currCU.Y());
        }
     }
  }

  for (int y = 0; y < pcv.heightInCtus; y++)
  {
    for (int x = 0; x < pcv.widthInCtus; x++)
    {
      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUSizeLog2, y << pcv.maxCUSizeLog2, pcv.maxCUSize, pcv.maxCUSize ) );
      DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
      DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );

      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C, TREE_D ), CH_C ) )
        if( currCU.Cb().valid() )
        {
          if( ! currCU.Y().valid() )   DTRACE(g_trace_ctx, D_CRC, "CU chroma Pos %d %d", currCU.Cb().x, currCU.Cb().y);
          DTRACE_CCRC(g_trace_ctx, D_CRC, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Cb()), COMP_Cb, &currCU.Cb());
          DTRACE_CCRC(g_trace_ctx, D_CRC, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Cr()), COMP_Cr, &currCU.Cb());
        }
     }
  }

#endif

  if( cs.pps->deblockingFilterControlPresent && cs.pps->deblockingFilterDisabled && !cs.pps->deblockingFilterOverrideEnabled )
  {
    return;
  }

  if( calcFilterStrength )
  {
    cs.getLoopFilterParamBuf( EDGE_VER ).memset( 0 );
    cs.getLoopFilterParamBuf( EDGE_HOR ).memset( 0 );

    for (int y = 0; y < pcv.heightInCtus; y++)
    {
      for (int x = 0; x < pcv.widthInCtus; x++)
      {
        const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUSizeLog2, y << pcv.maxCUSizeLog2, pcv.maxCUSize, pcv.maxCUSize ) );

        LoopFilter::calcFilterStrengthsCTU( cs, ctuArea, false );
      }
    }
  }

  for( unsigned y = 0; y < pcv.heightInCtus; y++ )
  {
    loopFilterPicLine( cs, MAX_NUM_CH, y, 0, NUM_EDGE_DIR );
  }

#if ENABLE_TRACING 
  for (int y = 0; y < pcv.heightInCtus; y++)
  {
    for (int x = 0; x < pcv.widthInCtus; x++)
    {
      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUSizeLog2, y << pcv.maxCUSizeLog2, pcv.maxCUSize, pcv.maxCUSize ) );
      DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
      DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );

      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L, TREE_D ), CH_L ) )
        if( currCU.Y().valid() )
        {
          DTRACE(g_trace_ctx, D_CRC, "CU LumaPos %d %d", currCU.Y().x, currCU.Y().y);
          DTRACE_CCRC(g_trace_ctx, D_CRC, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Y()), COMP_Y, &currCU.Y());
        }
     }
  }

  for (int y = 0; y < pcv.heightInCtus; y++)
  {
    for (int x = 0; x < pcv.widthInCtus; x++)
    {
      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUSizeLog2, y << pcv.maxCUSizeLog2, pcv.maxCUSize, pcv.maxCUSize ) );
      DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
      DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );

      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C, TREE_D ), CH_C ) )
        if( currCU.Cb().valid() )
        {
          if( ! currCU.Y().valid() )   DTRACE(g_trace_ctx, D_CRC, "CU chroma Pos %d %d", currCU.Cb().x, currCU.Cb().y);
          DTRACE_CCRC(g_trace_ctx, D_CRC, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Cb()), COMP_Cb, &currCU.Cb());
          DTRACE_CCRC(g_trace_ctx, D_CRC, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Cr()), COMP_Cr, &currCU.Cb());
        }
     }
  }
#endif

  DTRACE_PIC_COMP(D_REC_CB_LUMA_LF,   cs, cs.getRecoBuf(), COMP_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMP_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMP_Cr);
}

void LoopFilter::loopFilterPicLine( CodingStructure &cs, const ChannelType chType, const int ctuLine, const int offset, const DeblockEdgeDir edgeDir ) const
{
  const PreCalcValues &pcv = *cs.pcv;

  const bool frstLine = ctuLine == 0;

  const int ly = frstLine ? 0 : ( ctuLine * pcv.maxCUSize + offset );
  const int lh = frstLine ? pcv.maxCUSize + offset : pcv.maxCUSize;

  if( ly >= pcv.lumaHeight )
  {
    return;
  }

  PelUnitBuf recoBuf = cs.picture->getRecoBuf();

  UnitArea prevCtuArea;

  const UnitArea ctuArea = clipArea( UnitArea( pcv.chrFormat, Area( 0, ly, pcv.maxCUSize, lh ) ), *cs.picture );

  if( edgeDir == NUM_EDGE_DIR || edgeDir == EDGE_VER )
  {
    xDeblockArea<EDGE_VER>( cs, ctuArea, chType, recoBuf );
  }

  prevCtuArea = ctuArea;

  for( unsigned x = 1; x < pcv.widthInCtus; x++ )
  {
    const UnitArea ctuArea = clipArea( UnitArea( pcv.chrFormat, Area( x << pcv.maxCUSizeLog2, ly, pcv.maxCUSize, lh ) ), *cs.picture );

    if( edgeDir == NUM_EDGE_DIR || edgeDir == EDGE_VER )
    {
      xDeblockArea<EDGE_VER>( cs, ctuArea, chType, recoBuf );
    }
    if( edgeDir == NUM_EDGE_DIR || edgeDir == EDGE_HOR )
    {
      xDeblockArea<EDGE_HOR>( cs, prevCtuArea, chType, recoBuf );
    }

    prevCtuArea = ctuArea;
  }

  if( edgeDir == NUM_EDGE_DIR || edgeDir == EDGE_HOR )
  {
    xDeblockArea<EDGE_HOR>( cs, prevCtuArea, chType, recoBuf );
  }
}

void LoopFilter::calcFilterStrengthsCTU( CodingStructure& cs, const UnitArea& ctuArea, const bool clearLFP )
{
  if( clearLFP )
  {
    UnitScale lfScale = cs.getScaling( UnitScale::LF_PARAM_MAP, CH_L );
    Area      lfArea  ( lfScale.scale( clipArea( ctuArea.Y(), cs.picture->Y() ) ) );
    cs.getLoopFilterParamBuf( EDGE_VER ).subBuf( lfArea.pos(), lfArea.size() ).memset( 0 );
    cs.getLoopFilterParamBuf( EDGE_HOR ).subBuf( lfArea.pos(), lfArea.size() ).memset( 0 );
  }

  const int maxNumChannelType = cs.pcv->chrFormat != CHROMA_400 && CS::isDualITree( cs ) ? 2 : 1;

  for( int ch = 0; ch < maxNumChannelType; ch++ )
  {
    const ChannelType chType = ChannelType( ch );

    for( const auto& cuLf : cs.traverseCUs( CS::getArea( cs, ctuArea, chType, TREE_D ), chType ) )
    {
      LoopFilter::calcFilterStrengths( cuLf );
    }
  }
}

void LoopFilter::loopFilterCTU( CodingStructure &cs, const ChannelType chType, const int ctuCol, const int ctuLine, const int offset, const DeblockEdgeDir edgeDir ) const
{
  const PreCalcValues &pcv = *cs.pcv;

  const bool frstLine = ctuLine == 0;

  const int ly = frstLine ? 0 : ( ctuLine * pcv.maxCUSize + ( offset ) );
  const int lh = frstLine ? pcv.maxCUSize + ( offset ) : pcv.maxCUSize;

  if( ly >= pcv.lumaHeight )
  {
    return;
  }
  
  PelUnitBuf recoBuf = cs.picture->getRecoBuf();

  const UnitArea ctuArea = clipArea( UnitArea( pcv.chrFormat, Area( ctuCol << pcv.maxCUSizeLog2, ly, pcv.maxCUSize, lh ) ), *cs.picture );

  if( edgeDir == NUM_EDGE_DIR || edgeDir == EDGE_VER )
  {
    xDeblockArea<EDGE_VER>( cs, ctuArea, chType, recoBuf );
  }
  if( edgeDir == NUM_EDGE_DIR || edgeDir == EDGE_HOR )
  {
    xDeblockArea<EDGE_HOR>( cs, ctuArea, chType, recoBuf );
  }
}
// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 Deblocking filter process in CU-based (the same function as conventional's)

 \param cu               the CU to be deblocked
 \param edgeDir          the direction of the edge in block boundary (horizontal/vertical), which is added newly
*/
template<DeblockEdgeDir edgeDir>
void LoopFilter::xDeblockArea( const CodingStructure& cs, const UnitArea& area, const ChannelType chType, PelUnitBuf& picRecoBuf ) const
{
  if( cs.slice->deblockingFilterDisable )
  {
    return;
  }

  const PreCalcValues& pcv = *cs.pcv;

  const Area lumaArea      = area.Y().valid() ? static_cast<const Area&>( area.Y() ) : Area( recalcPosition( pcv.chrFormat, CH_C, CH_L, area.chromaPos() ), recalcSize( pcv.chrFormat, CH_C, CH_L, area.chromaSize() ) );

  bool doLuma   =   chType == MAX_NUM_CH || isLuma  ( chType );
  bool doChroma = ( chType == MAX_NUM_CH || isChroma( chType ) ) && pcv.chrFormat != CHROMA_400;

  static constexpr int incx = 4;
  static constexpr int incy = 4;

  const int csx = getChannelTypeScaleX( CH_C, pcv.chrFormat );
  const int csy = getChannelTypeScaleY( CH_C, pcv.chrFormat );

  const UnitScale scale = cs.getScaling( UnitScale::LF_PARAM_MAP, CH_L );

  const int lfpPtrLOffset = scale.scaleHor( incx );

  const LoopFilterParam* lfpPtr = cs.picture->cs->getLFPMapPtr( edgeDir );
  ptrdiff_t lfpStride     = cs.picture->cs->getLFPMapStride();
  OFFSET( lfpPtr, lfpStride, scale.scaleHor( lumaArea.x ), scale.scaleVer( lumaArea.y ) );

  for( int dy = 0; dy < lumaArea.height; dy += incy )
  {
    LoopFilterParam const* lineLfpPtr = lfpPtr;
    
    const int dyInCtu = doChroma ? ( ( area.chromaPos().y + ( dy >> csy ) ) & ( pcv.maxCUSizeMask >> csy ) ) : 0;

    for( int dx = 0; dx < lumaArea.width; dx += incx )
    {
      if( doLuma && BsGet( lineLfpPtr->bs, COMP_Y ) )
      {
        xEdgeFilterLuma<edgeDir>( cs, lumaArea.pos().offset( dx, dy ), *lineLfpPtr, picRecoBuf );
      }

      const int dxInCtu = doChroma ? ( ( area.chromaPos().x + ( dx >> csx ) ) & ( pcv.maxCUSizeMask >> csx ) ) : 0;

      if( doChroma
          && ( ( edgeDir == EDGE_VER && ( dxInCtu & ( DEBLOCK_SMALLEST_BLOCK - 1 ) ) == 0 )
            || ( edgeDir == EDGE_HOR && ( dyInCtu & ( DEBLOCK_SMALLEST_BLOCK - 1 ) ) == 0 ) )
          && ( BsGet( lineLfpPtr->bs, COMP_Cb ) | BsGet( lineLfpPtr->bs, COMP_Cr ) ) )
      {
        xEdgeFilterChroma<edgeDir>( cs, area.chromaPos().offset( dx >> csx, dy >> csy ), *lineLfpPtr, picRecoBuf );
      }

      OFFSETX( lineLfpPtr, lfpStride, lfpPtrLOffset );
    }

    OFFSETY( lfpPtr, lfpStride, scale.scaleVer( incy ) );
  }
}

void LoopFilter::getMaxFilterLength( const CodingUnit& cu, int& maxFilterLenghtLumaHor, int& maxFilterLenghtLumaVer )
{
  const Area&            areal      = cu.blocks[COMP_Y];
  const PreCalcValues&   pcv        = *cu.cs->pcv;
  const Position         lfpPos     = cu.cs->getScaling( UnitScale::LF_PARAM_MAP, CH_L ).scale( areal.pos() );
  LoopFilterParam*       lfpPtrH    = cu.cs->picture->cs->getLFPMapPtr( EDGE_HOR );
  LoopFilterParam*       lfpPtrV    = cu.cs->picture->cs->getLFPMapPtr( EDGE_VER );
  ptrdiff_t              lfpStride  = cu.cs->picture->cs->getLFPMapStride();

  OFFSET( lfpPtrH, lfpStride, lfpPos.x, lfpPos.y );
  OFFSET( lfpPtrV, lfpStride, lfpPos.x, lfpPos.y );

  for( int dy = 0; dy < areal.height; dy += pcv.minCUSize )
  {
    for( int dx = 0, n = 0; dx < areal.width; dx += pcv.minCUSize, n++ )
    {
      int filterLengthHor = std::max( ( lfpPtrH[n].sideMaxFiltLength & 7 ) >= 5 ? 4 : 0, ( lfpPtrH[n].sideMaxFiltLength >> 4 ) & 7 );
      if( maxFilterLenghtLumaHor < filterLengthHor )
      {
        maxFilterLenghtLumaHor = filterLengthHor;
      }

      int filterLengthVer = std::max( ( lfpPtrV[n].sideMaxFiltLength & 7 ) >= 5 ? 4 : 0, ( lfpPtrV[n].sideMaxFiltLength >> 4 ) & 7 );
      if( maxFilterLenghtLumaVer < filterLengthVer )
      {
        maxFilterLenghtLumaVer = filterLengthVer;
      }
    }

    INCY( lfpPtrH, lfpStride );
    INCY( lfpPtrV, lfpStride );
  }

  maxFilterLenghtLumaHor = maxFilterLenghtLumaHor ? ( ( maxFilterLenghtLumaHor < 5 ? 4 : 8 ) ) : 0;
  maxFilterLenghtLumaVer = maxFilterLenghtLumaVer ? ( ( maxFilterLenghtLumaVer < 5 ? 4 : 8 ) ) : 0;
}


namespace CU
{
  TransformUnit const* getTU( const CodingUnit& cu, const Position& pos, const ChannelType chType )
  {
    CHECKD( !cu.blocks[chType].contains( pos ), "Position must be contained within the CU for TU fetching!" );

    const TransformUnit* ptu = cu.firstTU;

    while( !( ptu->blocks[chType].x + ptu->blocks[chType].width > pos.x && ptu->blocks[chType].y + ptu->blocks[chType].height > pos.y ) )
    {
      ptu = ptu->next;
    }

    return ptu;
  }
}

template<DeblockEdgeDir edgeDir> inline PosType parlPos( const Position& );
template<DeblockEdgeDir edgeDir> inline PosType perpPos( const Position& );

template<DeblockEdgeDir edgeDir> inline SizeType parlSize( const Size& );
template<DeblockEdgeDir edgeDir> inline SizeType perpSize( const Size& );

template<> inline PosType parlPos<EDGE_HOR>( const Position& pos ) { return pos.x; }
template<> inline PosType perpPos<EDGE_HOR>( const Position& pos ) { return pos.y; }
template<> inline PosType parlPos<EDGE_VER>( const Position& pos ) { return pos.y; }
template<> inline PosType perpPos<EDGE_VER>( const Position& pos ) { return pos.x; }

template<> inline SizeType parlSize<EDGE_HOR>( const Size& size ) { return size.width; }
template<> inline SizeType perpSize<EDGE_HOR>( const Size& size ) { return size.height; }
template<> inline SizeType parlSize<EDGE_VER>( const Size& size ) { return size.height; }
template<> inline SizeType perpSize<EDGE_VER>( const Size& size ) { return size.width; }

// set / get functions
LFCUParam xGetLoopfilterParam( const CodingUnit& cu );

// filtering functions
template<DeblockEdgeDir edgeDir>
void xGetBoundaryStrengthSingle             ( LoopFilterParam& lfp, const CodingUnit& cu, const Position &localPos, const CodingUnit &cuP );
template<DeblockEdgeDir edgeDir>
void xSetEdgeFilterInsidePu                 ( const CodingUnit &cu, const Area &area, const bool bValue );

template<DeblockEdgeDir edgeDir>
void xSetMaxFilterLengthPQFromTransformSizes( const CodingUnit& cu, const TransformUnit& currTU, const bool bValue, bool deriveBdStrngt );
template<DeblockEdgeDir edgeDir>
void xSetMaxFilterLengthPQForCodingSubBlocks( const CodingUnit& cu );


LFCUParam xGetLoopfilterParam               ( const CodingUnit& cu );
  
bool isCrossedByVirtualBoundaries           ( const SPS* pps, const Area& area, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[] );
void xDeriveEdgefilterParam                 ( const Position pos, const int numVerVirBndry, const int numHorVirBndry, const int verVirBndryPos[], const int horVirBndryPos[], bool& verEdgeFilter, bool& horEdgeFilter );

void LoopFilter::calcFilterStrengths( const CodingUnit& cu, bool clearLF )
{
  if( cu.slice->deblockingFilterDisable )
  {
    return;
  }

  const PreCalcValues& pcv = *cu.cs->pcv;
  const Area area          = cu.blocks[cu.chType];

  bool horEdgeFilter    = false;
  bool verEdgeFilter    = false;
  int  numHorVirBndry   = 0;
  int  numVerVirBndry   = 0;
  int  horVirBndryPos[] = { 0, 0, 0 };
  int  verVirBndryPos[] = { 0, 0, 0 };

  const uint8_t channelScaleX = getChannelTypeScaleX( cu.chType, cu.chromaFormat );
  const uint8_t channelScaleY = getChannelTypeScaleY( cu.chType, cu.chromaFormat );
  
  if( clearLF )
  {
    const unsigned uiPelsInPartX   = pcv.minCUSize >> channelScaleX;
    const unsigned uiPelsInPartY   = pcv.minCUSize >> channelScaleY;
    LoopFilterParam* lfpPtrH       = cu.cs->picture->cs->getLFPMapPtr( EDGE_HOR );
    LoopFilterParam* lfpPtrV       = cu.cs->picture->cs->getLFPMapPtr( EDGE_VER );
    ptrdiff_t        lfpStride     = cu.cs->picture->cs->getLFPMapStride();
    const Position         lfpPos  = cu.cs->picture->cs->getScaling( UnitScale::LF_PARAM_MAP, cu.chType ).scale( area.pos() );
    OFFSET( lfpPtrH, lfpStride, lfpPos.x, lfpPos.y );
    OFFSET( lfpPtrV, lfpStride, lfpPos.x, lfpPos.y );

    const size_t bufLen = ( area.width / uiPelsInPartX ) * sizeof( LoopFilterParam );
    for( int y = 0; y < area.height; y += uiPelsInPartY )
    {
      memset( lfpPtrH, 0, bufLen );
      memset( lfpPtrV, 0, bufLen );

      INCY( lfpPtrH, lfpStride );
      INCY( lfpPtrV, lfpStride );
    }
  }

  const bool isCuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries( cu.cs->sps,
                                                                            Area( area.x << channelScaleX, area.y << channelScaleY, area.width << channelScaleX, area.height << channelScaleY ),
                                                                            numHorVirBndry, numVerVirBndry,
                                                                            horVirBndryPos, verVirBndryPos );
  if( isCuCrossedByVirtualBoundaries )
  {
    CHECK( numHorVirBndry >= (int)( sizeof(horVirBndryPos) / sizeof(horVirBndryPos[0]) ), "Too many virtual boundaries" );
    CHECK( numHorVirBndry >= (int)( sizeof(verVirBndryPos) / sizeof(verVirBndryPos[0]) ), "Too many virtual boundaries" );
  }
  

  static constexpr int subBlockSize = 8;
  LFCUParam stLFCUParam         { xGetLoopfilterParam( cu ) };
  const UnitScale scaling       = cu.cs->getScaling( UnitScale::LF_PARAM_MAP, cu.chType );
  // for SUBPU ATMVP and Affine, more PU deblocking needs to be found, for ISP the chroma block will be deferred to the last luma block,
  // so the processing order is different. For all other cases the boundary strenght can be directly obtained in the TU loop.
  const bool refineBs     = ( cu.mergeFlag && cu.mergeType == MRG_TYPE_SUBPU_ATMVP ) || cu.affine || cu.ispMode;

  const int maskBlkX = ~( ( 1 << scaling.posx ) - 1 );
  const int maskBlkY = ~( ( 1 << scaling.posy ) - 1 );

  for( const TransformUnit* currTU = cu.firstTU; currTU; currTU = currTU->next )
  {
    const Area& areaTu = currTU->blocks[currTU->chType];

    verEdgeFilter = ( areaTu.x & maskBlkX ) == area.x ? stLFCUParam.leftEdge : true;
    horEdgeFilter = ( areaTu.y & maskBlkY ) == area.y ? stLFCUParam.topEdge  : true;

    if( isCuCrossedByVirtualBoundaries )
    {
      xDeriveEdgefilterParam( Position( ( areaTu.x & maskBlkX ) << channelScaleX, ( areaTu.y & maskBlkY ) << channelScaleY ),
                              numVerVirBndry, numHorVirBndry,
                              verVirBndryPos, horVirBndryPos,
                              verEdgeFilter,  horEdgeFilter );
    }

    xSetMaxFilterLengthPQFromTransformSizes<EDGE_VER>( cu, *currTU, verEdgeFilter, !refineBs );
    xSetMaxFilterLengthPQFromTransformSizes<EDGE_HOR>( cu, *currTU, horEdgeFilter, !refineBs );
  }

  if( !refineBs ) return;

  if( ( cu.mergeFlag && cu.mergeType == MRG_TYPE_SUBPU_ATMVP ) || cu.affine )
  {
    CHECK( cu.chType != CH_L, "This path is only valid for single tree blocks!" );

    for( int off = subBlockSize; off < area.width; off += subBlockSize )
    {
      const Area mvBlockV( cu.Y().x + off, cu.Y().y, subBlockSize, cu.Y().height );
      verEdgeFilter = true;
      if( isCuCrossedByVirtualBoundaries )
      {
        xDeriveEdgefilterParam( mvBlockV,
                                numVerVirBndry, 0,
                                verVirBndryPos, horVirBndryPos,
                                verEdgeFilter, horEdgeFilter );
      }

      xSetEdgeFilterInsidePu<EDGE_VER>( cu, mvBlockV, verEdgeFilter );
    }

    xSetMaxFilterLengthPQForCodingSubBlocks<EDGE_VER>( cu );

    for( int off = subBlockSize; off < area.height; off += subBlockSize )
    {
      const Area mvBlockH( cu.Y().x, cu.Y().y + off, cu.Y().width, subBlockSize );
      horEdgeFilter = true;
      if( isCuCrossedByVirtualBoundaries )
      {
        xDeriveEdgefilterParam( mvBlockH,
                                0, numHorVirBndry,
                                verVirBndryPos, horVirBndryPos,
                                verEdgeFilter, horEdgeFilter );
      }

      xSetEdgeFilterInsidePu<EDGE_HOR>( cu, mvBlockH, horEdgeFilter );
    }

    xSetMaxFilterLengthPQForCodingSubBlocks<EDGE_HOR>( cu );
  }

  const unsigned uiPelsInPartX = pcv.minCUSize >> channelScaleX;
  const unsigned uiPelsInPartY = pcv.minCUSize >> channelScaleY;
  const Position        lfpPos = scaling.scale( area.pos() );

  const CodingUnit* cuP        = CU::getLeft( cu );
  const ChannelType chType     = cu.chType;

  {
    LoopFilterParam* lfpPtrV   = cu.cs->picture->cs->getLFPMapPtr( EDGE_VER );
    ptrdiff_t        lfpStride = cu.cs->picture->cs->getLFPMapStride();
    OFFSET( lfpPtrV, lfpStride, lfpPos.x, lfpPos.y );

    for( int y = 0; y < area.height; y += uiPelsInPartY )
    {
      LoopFilterParam* lineLfpPtrV = lfpPtrV;
      
      cuP = !cuP || cuP->blocks[chType].y + cuP->blocks[chType].height > area.y + y ? cuP : cu.cs->getCU( Position{ area.x - 1, area.y + y }, chType, TREE_D );

      for( int x = 0; x < area.width; x += uiPelsInPartX )
      {
        if( lineLfpPtrV->filterEdge( chType ) ) xGetBoundaryStrengthSingle<EDGE_VER>( *lineLfpPtrV, cu, Position{ area.x + x, area.y + y }, x ? cu : *cuP );

        lineLfpPtrV->bs &= ~BsSet( 3, MAX_NUM_COMP );
        INCX( lineLfpPtrV, lfpStride );
      }

      INCY( lfpPtrV, lfpStride );
    }
  }

  cuP = CU::getAbove( cu );

  {
    LoopFilterParam* lfpPtrH   = cu.cs->picture->cs->getLFPMapPtr( EDGE_HOR );
    ptrdiff_t        lfpStride = cu.cs->picture->cs->getLFPMapStride();
    OFFSET( lfpPtrH, lfpStride, lfpPos.x, lfpPos.y );

    for( int y = 0; y < area.height; y += uiPelsInPartY )
    {
      LoopFilterParam* lineLfpPtrH = lfpPtrH;

      for( int x = 0; x < area.width; x += uiPelsInPartX )
      {
        cuP = ( y || ( cuP && cuP->blocks[chType].x + cuP->blocks[chType].width > area.x + x ) ) ? cuP : cu.cs->getCU( Position{ area.x + x, area.y - 1 }, chType, TREE_D );

        if( lineLfpPtrH->filterEdge( chType ) ) xGetBoundaryStrengthSingle<EDGE_HOR>( *lineLfpPtrH, cu, Position{ area.x + x, area.y + y }, y ? cu : *cuP );

        lineLfpPtrH->bs &= ~BsSet( 3, MAX_NUM_COMP );
        INCX( lineLfpPtrH, lfpStride );
      }

      INCY( lfpPtrH, lfpStride );
    }
  }
}

void LoopFilter::loopFilterCu( const CodingUnit& cu, ChannelType chType, DeblockEdgeDir edgeDir, PelUnitBuf& dbBuffer )
{
  if( edgeDir == EDGE_VER )
  {
    xDeblockArea<EDGE_VER>( *cu.cs, cu, chType, dbBuffer );
  }
  else
  {
    xDeblockArea<EDGE_HOR>( *cu.cs, cu, chType, dbBuffer );
  }
}

inline bool isCrossedByVirtualBoundaries( const SPS* sps, const Area& area, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[] )
{
  numHorVirBndry = 0;
  numVerVirBndry = 0;
  if( !sps->virtualBoundariesEnabled )
  {
    return false;
  }

  for( int i = 0; i < sps->numHorVirtualBoundaries; i++ )
  {
    if( area.y <= sps->virtualBoundariesPosY[ i ] && sps->virtualBoundariesPosY[ i ] < area.y + area.height )
    {
      horVirBndryPos[numHorVirBndry++] = sps->virtualBoundariesPosY[ i ];
    }
  }
  for( int i = 0; i < sps->numVerVirtualBoundaries; i++ )
  {
    if( area.x <= sps->virtualBoundariesPosX[ i ] && sps->virtualBoundariesPosX[ i ] < area.x + area.width )
    {
      verVirBndryPos[numVerVirBndry++] = sps->virtualBoundariesPosX[ i ];
    }
  }

  return numHorVirBndry > 0 || numVerVirBndry > 0;
}

inline void xDeriveEdgefilterParam( const Position pos, const int numVerVirBndry, const int numHorVirBndry, const int verVirBndryPos[], const int horVirBndryPos[], bool& verEdgeFilter, bool& horEdgeFilter )
{
  for (int i = 0; i < numVerVirBndry; i++)
  {
    if (verVirBndryPos[i] == pos.x)
    {
      verEdgeFilter = false;
      break;
    }
  }

  for (int i = 0; i < numHorVirBndry; i++)
  {
    if (horVirBndryPos[i] == pos.y)
    {
      horEdgeFilter = false;
      break;
    }
  }
}


template<DeblockEdgeDir edgeDir>
void xSetMaxFilterLengthPQFromTransformSizes( const CodingUnit& cu, const TransformUnit& currTU, const bool bValue, bool deriveBdStrngt )
{
  const PreCalcValues &pcv = *cu.cs->pcv;

  ChannelType start = CH_L;
  ChannelType end   = CH_C;

  const bool dt = CS::isDualITree( *cu.cs ) || cu.treeType != TREE_D;

  if( dt )
  {
    if( cu.chType == CH_L )
    {
      end = CH_L;
    }
    else
    {
      start = CH_C;
    }
  }

  if( start != end && !currTU.Cb().valid() )
  {
    end = CH_L;
  }

  const int csx    = ( start != end ) ? getChannelTypeScaleX( CH_C, pcv.chrFormat ) : 0;
  const int csy    = ( start != end ) ? getChannelTypeScaleY( CH_C, pcv.chrFormat ) : 0;
  const Area& area = currTU.blocks[end];

  for( int ct = start; ct <= end; ct++ )
  {
    const ChannelType ch  = ( ChannelType ) ct;
    const TreeType    tt  = isLuma( ch ) ? TREE_L : TREE_C;
    const TreeType    ttfst = isLuma( start ) ? TREE_L : TREE_C;
    const bool        vld = isLuma( ch ) ? currTU.Y().valid() : currTU.Cb().valid();
    if( vld && perpPos<edgeDir>( currTU.blocks[ch] ) != 0 )
    {
      LoopFilterParam* lfpPtr    = cu.cs->picture->cs->getLFPMapPtr( edgeDir );
      const UnitScale scaling    = cu.cs->picture->cs->getScaling( UnitScale::LF_PARAM_MAP, ch );
      ptrdiff_t lfpStride        = cu.cs->picture->cs->getLFPMapStride();
      OFFSET( lfpPtr, lfpStride, scaling.scaleHor( currTU.blocks[ch].x ), scaling.scaleVer( currTU.blocks[ch].y ) );

      const int         inc      = edgeDir ? pcv.minCUSize >> getChannelTypeScaleX( ch, cu.chromaFormat )
                                           : pcv.minCUSize >> getChannelTypeScaleY( ch, cu.chromaFormat );

      const CodingUnit* cuNeigh  = edgeDir ? CU::getAbove( cu ): CU::getLeft( cu );
      const CodingUnit* cuP      = ( cuNeigh && perpPos<edgeDir>( currTU.blocks[ch   ] ) == perpPos<edgeDir>( cu.blocks[ch   ] ) ) ? cuNeigh : &cu;
      const CodingUnit* cuPfstCh = ( cuNeigh && perpPos<edgeDir>( currTU.blocks[start] ) == perpPos<edgeDir>( cu.blocks[start] ) ) ? cuNeigh : &cu;
      const int         incFst   = edgeDir ? pcv.minCUSize >> getChannelTypeScaleX( ChannelType( start ), cu.chromaFormat )
                                           : pcv.minCUSize >> getChannelTypeScaleY( ChannelType( start ), cu.chromaFormat );

      if( cuP == &cu && cuNeigh == nullptr && ( edgeDir ? cu.blocks[ch].pos().y : cu.blocks[ch].pos().x ) > 0 ) //TODO: check for !pps.getLoopFilterAcrossSlicesEnabledFlag() || !pps.getLoopFilterAcrossTilesEnabledFlag()
      {
        const Position posP   { currTU.blocks[   ch].x - ( 1 - edgeDir ), currTU.blocks[   ch].y - edgeDir };
        const Position posPfst{ currTU.blocks[start].x - ( 1 - edgeDir ), currTU.blocks[start].y - edgeDir };
        cuP      = cuP     ->blocks[   ch].contains( posP    ) ? cuP      : cu.cs->getCU( posP,    ch,    tt );
        cuPfstCh = cuPfstCh->blocks[start].contains( posPfst ) ? cuPfstCh : cu.cs->getCU( posPfst, start, ttfst );
      }

      for( int d = 0, dFst = 0; d < parlSize<edgeDir>( currTU.blocks[ch] ); d += inc, dFst += incFst )
      {
        const Position  posQ     { currTU.blocks[ch].x + edgeDir * d, currTU.blocks[ch].y + ( 1 - edgeDir ) * d };
        const Position  posP     = posQ.offset( -( 1 - edgeDir ), -edgeDir );
        const Position  posPfst  = currTU.blocks[start].offset( edgeDir ? dFst : -1, edgeDir ? -1 : dFst );
        const int sizeQSide      = perpSize<edgeDir>( currTU.blocks[ch] );
                        cuP      = parlPos<edgeDir>( cuP->     blocks[ch   ] ) + parlSize<edgeDir>( cuP->     blocks[ch   ] ) > parlPos<edgeDir>( posP )    ? cuP      : cu.cs->getCU( posP,                 ch     , tt );
                        cuPfstCh = start != end && ch == end && deriveBdStrngt                                                                                                                         
                                 ? parlPos<edgeDir>( cuPfstCh->blocks[start] ) + parlSize<edgeDir>( cuPfstCh->blocks[start] ) > parlPos<edgeDir>( posPfst ) ? cuPfstCh : cu.cs->getCU( posPfst, ChannelType( start ), ttfst )
                                 : cuP;
        const TransformUnit &tuP = cuP->firstTU->next == nullptr ? *cuP->firstTU : *CU::getTU( *cuP, posP, ch );
        const int sizePSide      = perpSize<edgeDir>( tuP.blocks[ch] );
        LoopFilterParam& lfp     = *lfpPtr;

        if( ct == start )
        {
          lfp.setFilterEdge( cu.chType, bValue );
          if( ( lfp.bs || perpPos<edgeDir>( currTU.blocks[ch] ) == perpPos<edgeDir>( cu.blocks[ch] ) ) && bValue )
          {
            lfp.bs |= BsSet( 3, MAX_NUM_COMP );
          }
          else
          {
            lfp.bs |= BsSet( 1, MAX_NUM_COMP );
          }
        }

        if( ch == CH_L )
        {
          uint8_t maxFLPQ;

          bool smallBlock = ( sizePSide <= 4 ) || ( sizeQSide <= 4 );
          if( smallBlock )
          {
            maxFLPQ = 17;
          }
          else
          {
            maxFLPQ   = ( sizePSide >= 32 ) ? ( cuP->affine ? 5 : 7 ) : 3;
            maxFLPQ <<= 4;
            maxFLPQ  += ( sizeQSide >= 32 ) ? 7 : 3;
          }

          maxFLPQ    += 128;
          lfp.sideMaxFiltLength = maxFLPQ;
        }
        else
        {
          lfp.setFilterCMFL( ( sizeQSide >= 8 && sizePSide >= 8 ) ? 1 : 0 );
        }

        if( ct == end && deriveBdStrngt )
        {
          if( lfp.filterEdge( cu.chType ) ) xGetBoundaryStrengthSingle<edgeDir>( lfp, cu, Position( ( area.x + edgeDir * d ) << csx, ( area.y + ( 1 - edgeDir ) * d ) << csy ), *cuPfstCh );
          lfp.bs &= ~BsSet( 3, MAX_NUM_COMP );
        }

        OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
      }
    }
  }
}

template<DeblockEdgeDir edgeDir>
void xSetMaxFilterLengthPQForCodingSubBlocks( const CodingUnit& cu )
{
  static constexpr
        int subBlockSize = 8;
  const int minCUWidth   = cu.cs->pcv->minCUSize;
  const int minCUHeight  = cu.cs->pcv->minCUSize;
  const int xInc         = edgeDir ? minCUWidth   : subBlockSize;
  const int yInc         = edgeDir ? subBlockSize : minCUHeight;
  
  const UnitScale  scaling   = cu.cs->picture->cs->getScaling( UnitScale::LF_PARAM_MAP, CH_L );
  const Position   lfpPos    = scaling.scale( cu.lumaPos() );
  LoopFilterParam* lfpPtrL   = cu.cs->picture->cs->getLFPMapPtr( edgeDir );
  ptrdiff_t        lfpStride = cu.cs->picture->cs->getLFPMapStride();

  OFFSET( lfpPtrL, lfpStride, lfpPos.x, lfpPos.y );

  for( int y = 0; y < cu.Y().height; y += yInc )
  {
    LoopFilterParam* lfpPtr = lfpPtrL;

    for( int x = 0; x < cu.Y().width; x += xInc )
    {
      const PosType perpVal = edgeDir ? y : x;

      uint8_t maxFLP, maxFLQ;
      uint8_t te = 0;
      if( lfpPtr->sideMaxFiltLength & 128 )
      {
        te = 128;
        maxFLQ = std::min<int>(   lfpPtr->sideMaxFiltLength        & 7, 5 );
        maxFLP =                ( lfpPtr->sideMaxFiltLength >> 4 ) & 7;

        if( perpVal > 0 )
        {
          maxFLP = std::min<int>( maxFLP, 5 );
        }
      }
      else if( perpVal > 0 && (                                          ( GET_OFFSET( lfpPtr, lfpStride, -    ( 1 - edgeDir ), -    edgeDir )->sideMaxFiltLength & 128 ) || ( perpVal + 4 >= perpSize<edgeDir>( cu.Y() ) ) || ( GET_OFFSET( lfpPtr, lfpStride,     ( 1 - edgeDir ),     edgeDir )->sideMaxFiltLength & 128 ) ) )
      {
        maxFLP = maxFLQ = 1;
      }
      else if( perpVal > 0 && ( ( edgeDir ? ( y == 8 ) : ( x == 8 ) ) || ( GET_OFFSET( lfpPtr, lfpStride, -2 * ( 1 - edgeDir ), -2 * edgeDir )->sideMaxFiltLength & 128 ) || ( perpVal + 8 >= perpSize<edgeDir>( cu.Y() ) ) || ( GET_OFFSET( lfpPtr, lfpStride, 2 * ( 1 - edgeDir ), 2 * edgeDir )->sideMaxFiltLength & 128 ) ) )
      {
        maxFLP = maxFLQ = 2;
      }
      else
      {
        maxFLP = maxFLQ = 3;
      }
      unsigned newVal = maxFLP;
      newVal <<= 4;
      newVal += maxFLQ;
      newVal += te;

      lfpPtr->sideMaxFiltLength = newVal;

      OFFSETX( lfpPtr, lfpStride, scaling.scaleHor( xInc ) );
    }

    OFFSETY( lfpPtrL, lfpStride, scaling.scaleVer( yInc ) );
  }
}

template<DeblockEdgeDir edgeDir>
void xSetEdgeFilterInsidePu( const CodingUnit &cu, const Area &area, const bool bValue )
{
  const PreCalcValues &  pcv       = *cu.cs->pcv;
  const Position         lfpPos    =  cu.cs->getScaling( UnitScale::LF_PARAM_MAP, cu.chType ).scale( area.pos() );
  LoopFilterParam*       lfpPtr    =  cu.cs->picture->cs->getLFPMapPtr( edgeDir );
  ptrdiff_t              lfpStride =  cu.cs->picture->cs->getLFPMapStride();

  OFFSET( lfpPtr, lfpStride, lfpPos.x, lfpPos.y );

  const int inc = edgeDir ? pcv.minCUSize >> getChannelTypeScaleX( cu.chType, cu.chromaFormat )
                          : pcv.minCUSize >> getChannelTypeScaleY( cu.chType, cu.chromaFormat );

  for( int d = 0; d < parlSize<edgeDir>( area ); d += inc )
  {
    lfpPtr->setFilterEdge( cu.chType, bValue );

    if( lfpPtr->bs && bValue )
    {
      lfpPtr->bs |= BsSet( 3, MAX_NUM_COMP );
    }
      
    OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
  }
}

template<DeblockEdgeDir edgeDir>
void xGetBoundaryStrengthSingle( LoopFilterParam& lfp, const CodingUnit& cuQ, const Position &localPos, const CodingUnit& cuP )
{
  const Slice      &sliceQ = *cuQ.slice;
  const ChannelType chType = cuQ.chType;
  const Position    &cuPos = cuQ.blocks[chType].pos();
  const Position     &posQ = localPos;
  const Position      posP { posQ.x - !edgeDir, posQ.y - edgeDir };

  const TransformUnit &tuQ = cuQ.firstTU->next == nullptr ? *cuQ.firstTU : *CU::getTU( cuQ, posQ, chType );
  const TransformUnit &tuP = cuP.firstTU->next == nullptr ? *cuP.firstTU : *CU::getTU( cuP, posP, chType ); //TODO: check this: based on chType of the current cu, because cuQ.chType and cuP.chType are not the same when local dual-tree is applied
  
  const bool hasLuma   = cuQ.Y(). valid();
  const bool hasChroma = cuQ.Cb().valid();

  bool cuPcIsIntra = false;
  int  chrmBS      = 2;

  if( hasLuma )
  {
    lfp.qp[0] = ( cuQ.qp + cuP.qp + 1 ) >> 1;
  }

  if( hasChroma )
  {
    const bool isPQDiffCh     = !chType && cuP.treeType != TREE_D;
    const TransformUnit &tuQc = cuQ.ispMode ? *cuQ.lastTU : tuQ;
    const Position      posPc = isPQDiffCh ? recalcPosition( cuQ.chromaFormat, chType, CH_C, posP ) : Position();
    const CodingUnit    &cuPc = isPQDiffCh ? *cuQ.cs->getCU(       posPc, CH_C, TREE_C ) :   cuP;
    const TransformUnit &tuPc = isPQDiffCh ? *    CU::getTU( cuPc, posPc, CH_C )         : ( cuP.ispMode ? *cuP.lastTU : tuP );
    const int qpBdOffset      = cuQ.cs->sps->qpBDOffset[1];

    for( int chromaIdx = 1; chromaIdx <= 2; chromaIdx++ )
    {
      const QpParam cQP( tuPc, ComponentID( chromaIdx ), false );
      const QpParam cQQ( tuQc, ComponentID( chromaIdx ), false );

      const int baseQp_P = cQP.Qp( 0 ) - qpBdOffset;
      const int baseQp_Q = cQQ.Qp( 0 ) - qpBdOffset;
      lfp.qp[chromaIdx] = ( ( baseQp_Q + baseQp_P + 1 ) >> 1 );
    }


    cuPcIsIntra = CU::isIntra( cuPc );
    
    if( cuPcIsIntra )
    {
      chrmBS  = ( MODE_INTRA == cuPc.predMode && cuPc.bdpcmM[CH_C] ) && ( MODE_INTRA == cuQ.predMode && cuQ.bdpcmM[CH_C] ) ? 0 : 2;
    }
  }

  const int bsMask = ( hasLuma   ? BsSet( 3, COMP_Y ) :  0 ) |
                                   BsSet( 3, MAX_NUM_COMP  ) |
                     ( hasChroma ? BsSet( 3, COMP_Cb ) : 0 ) |
                     ( hasChroma ? BsSet( 3, COMP_Cr ) : 0 );

  //-- Set BS for Intra MB : BS = 4 or 3
  if( MODE_INTRA == cuP.predMode || MODE_INTRA == cuQ.predMode )
  {
    const int edgeIdx = ( perpPos<edgeDir>( localPos ) - perpPos<edgeDir>( cuPos ) ) / 4;
    int bsY = ( MODE_INTRA == cuP.predMode && cuP.bdpcmM[CH_L] ) && ( MODE_INTRA == cuQ.predMode && cuQ.bdpcmM[CH_L] ) ? 0 : 2;
    if( cuQ.ispMode && edgeIdx )
    {
      lfp.bs |= BsSet( bsY, COMP_Y ) & bsMask;
    }
    else
    {
      lfp.bs |= ( BsSet( bsY, COMP_Y ) + BsSet( chrmBS, COMP_Cb ) + BsSet( chrmBS, COMP_Cr ) ) & bsMask;
    }
    return;
  }
  else if( cuPcIsIntra )
  {
    lfp.bs |= ( BsSet( chrmBS, COMP_Cb ) + BsSet( chrmBS, COMP_Cr ) );
  }

  if( ( lfp.bs & bsMask ) && ( cuP.ciip || cuQ.ciip ) )
  {
    lfp.bs |= ( BsSet( 2, COMP_Y ) + BsSet( 2, COMP_Cb ) + BsSet( 2, COMP_Cr ) ) & bsMask;
    return;
  }

  unsigned tmpBs = 0;
  //-- Set BS for not Intra MB : BS = 2 or 1 or 0
  // Y
  if( lfp.bs & bsMask )
  {
    tmpBs |= BsSet( ( TU::getCbf( tuQ, COMP_Y  ) || TU::getCbf( tuP, COMP_Y  )                                   ) ? 1 : 0, COMP_Y  );
    if (!(MODE_INTRA != cuP.predMode && MODE_INTRA != cuQ.predMode && cuPcIsIntra))
    {
      tmpBs |= BsSet((TU::getCbf(tuQ, COMP_Cb) || TU::getCbf(tuP, COMP_Cb) || tuQ.jointCbCr || tuP.jointCbCr) ? 1 : 0, COMP_Cb);
      tmpBs |= BsSet((TU::getCbf(tuQ, COMP_Cr) || TU::getCbf(tuP, COMP_Cr) || tuQ.jointCbCr || tuP.jointCbCr) ? 1 : 0, COMP_Cr);
    }
  }

  if( BsGet( tmpBs, COMP_Y ) == 1 )
  {
    lfp.bs |= tmpBs & bsMask;
    return;
  }

  if( cuP.ciip || cuQ.ciip )
  {
    lfp.bs |= 1 & bsMask;
    return;
  }

  if( !hasLuma )
  {
    lfp.bs |= tmpBs & bsMask;
    return;
  }

  if( BsGet( lfp.bs, MAX_NUM_COMP ) != 0 && BsGet( lfp.bs, MAX_NUM_COMP ) != 3 )
  {
    lfp.bs |= tmpBs & bsMask;
    return;
  }

  if( hasChroma )
  {
    lfp.bs |= tmpBs & bsMask;
  }

  if( cuP.predMode != cuQ.predMode && hasLuma )
  {
    lfp.bs |= 1 & bsMask;
    return;
  }

  const MotionInfo&     miQ     = cuQ.cs->getMotionInfo( posQ );
  const MotionInfo&     miP     = cuP.cs->getMotionInfo( posP );
  const Slice&          sliceP  = *cuP.slice;

  static constexpr int nThreshold = ( 1 << MV_FRACTIONAL_BITS_INTERNAL ) >> 1;

  if( sliceQ.isInterB() || sliceP.isInterB() )
  {
    const Picture *piRefP0 = CU::isIBC( cuP ) ? sliceP.pic : 0 <= miP.refIdx[0] ? sliceP.getRefPic( REF_PIC_LIST_0, miP.refIdx[0] ) : nullptr;
    const Picture *piRefP1 = CU::isIBC( cuP ) ? nullptr    : 0 <= miP.refIdx[1] ? sliceP.getRefPic( REF_PIC_LIST_1, miP.refIdx[1] ) : nullptr;
    const Picture *piRefQ0 = CU::isIBC( cuQ ) ? sliceQ.pic : 0 <= miQ.refIdx[0] ? sliceQ.getRefPic( REF_PIC_LIST_0, miQ.refIdx[0] ) : nullptr;
    const Picture *piRefQ1 = CU::isIBC( cuQ ) ? nullptr    : 0 <= miQ.refIdx[1] ? sliceQ.getRefPic( REF_PIC_LIST_1, miQ.refIdx[1] ) : nullptr;

    unsigned uiBs = 0;

    //th can be optimized
    if( ( piRefP0 == piRefQ0 && piRefP1 == piRefQ1 ) || ( piRefP0 == piRefQ1 && piRefP1 == piRefQ0 ) )
    {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_DBLF
      const __m128i xmvP = _mm_unpacklo_epi64( 0 <= miP.refIdx[0] ? _mm_loadl_epi64( ( const __m128i* ) &miP.mv[0] ) : _mm_setzero_si128(), 0 <= miP.refIdx[1] ? _mm_loadl_epi64( ( const __m128i* ) &miP.mv[1] ) : _mm_setzero_si128() );
      const __m128i xmvQ = _mm_unpacklo_epi64( 0 <= miQ.refIdx[0] ? _mm_loadl_epi64( ( const __m128i* ) &miQ.mv[0] ) : _mm_setzero_si128(), 0 <= miQ.refIdx[1] ? _mm_loadl_epi64( ( const __m128i* ) &miQ.mv[1] ) : _mm_setzero_si128() );
      const __m128i xth  = _mm_set1_epi32( nThreshold - 1 );
#else
      Mv mvP[2] = { { 0, 0 }, { 0, 0 } }, mvQ[2] = { { 0, 0 }, { 0, 0 } };

      if( 0 <= miP.refIdx[0] ) { mvP[0] = miP.mv[0]; }
      if( 0 <= miP.refIdx[1] ) { mvP[1] = miP.mv[1]; }
      if( 0 <= miQ.refIdx[0] ) { mvQ[0] = miQ.mv[0]; }
      if( 0 <= miQ.refIdx[1] ) { mvQ[1] = miQ.mv[1]; }
#endif
      if( piRefP0 != piRefP1 )   // Different L0 & L1
      {
        if( piRefP0 == piRefQ0 )
        {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_DBLF
          __m128i
          xdiff = _mm_sub_epi32  ( xmvQ, xmvP );
          xdiff = _mm_abs_epi32  ( xdiff );
          xdiff = _mm_cmpgt_epi32( xdiff, xth );
          uiBs  = _mm_testz_si128( xdiff, xdiff ) ? 0 : 1;
#else
          uiBs = ( ( abs( mvQ[0].hor - mvP[0].hor ) >= nThreshold ) || ( abs( mvQ[0].ver - mvP[0].ver ) >= nThreshold ) ||
                   ( abs( mvQ[1].hor - mvP[1].hor ) >= nThreshold ) || ( abs( mvQ[1].ver - mvP[1].ver ) >= nThreshold ) )
                 ? 1 : 0;
#endif
        }
        else
        {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_DBLF
          __m128i
          xmvQ1 = _mm_shuffle_epi32( xmvQ, ( 2 << 0 ) + ( 3 <<  2 ) + ( 0 << 4 ) + ( 1 << 6 ) );
          __m128i
          xdiff = _mm_sub_epi32  ( xmvQ1, xmvP );
          xdiff = _mm_abs_epi32  ( xdiff );
          xdiff = _mm_cmpgt_epi32( xdiff, xth );
          uiBs  = _mm_testz_si128( xdiff, xdiff ) ? 0 : 1;
#else
          uiBs = ( ( abs( mvQ[1].hor - mvP[0].hor ) >= nThreshold ) || ( abs( mvQ[1].ver - mvP[0].ver ) >= nThreshold ) ||
                   ( abs( mvQ[0].hor - mvP[1].hor ) >= nThreshold ) || ( abs( mvQ[0].ver - mvP[1].ver ) >= nThreshold ) )
                 ? 1 : 0;
#endif
        }
      }
      else    // Same L0 & L1
      {

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_DBLF
        __m128i
        xmvQ1 = _mm_shuffle_epi32( xmvQ, ( 2 << 0 ) + ( 3 << 2 ) + ( 0 << 4 ) + ( 1 << 6 ) );
        __m128i
        xdiff = _mm_sub_epi32( xmvQ1, xmvP );
        xdiff = _mm_abs_epi32( xdiff );
        xdiff = _mm_cmpgt_epi32( xdiff, xth );
        uiBs  = _mm_testz_si128( xdiff, xdiff ) ? 0 : 1;

        xdiff = _mm_sub_epi32( xmvQ, xmvP );
        xdiff = _mm_abs_epi32( xdiff );
        xdiff = _mm_cmpgt_epi32( xdiff, xth );
        uiBs &= _mm_testz_si128( xdiff, xdiff ) ? 0 : 1;
#else
        uiBs = ( ( abs( mvQ[0].hor - mvP[0].hor ) >= nThreshold ) || ( abs( mvQ[0].ver - mvP[0].ver ) >= nThreshold ) ||
                 ( abs( mvQ[1].hor - mvP[1].hor ) >= nThreshold ) || ( abs( mvQ[1].ver - mvP[1].ver ) >= nThreshold ) )
               &&
               ( ( abs( mvQ[1].hor - mvP[0].hor ) >= nThreshold ) || ( abs( mvQ[1].ver - mvP[0].ver ) >= nThreshold ) ||
                 ( abs( mvQ[0].hor - mvP[1].hor ) >= nThreshold ) || ( abs( mvQ[0].ver - mvP[1].ver ) >= nThreshold ) )
               ? 1 : 0;
#endif
      }
    }
    else // for all different Ref_Idx
    {
      uiBs = 1;
    }

    lfp.bs |= ( uiBs + tmpBs ) & bsMask;
    return;
  }

  // pcSlice->isInterP()
  CHECK( CU::isInter( cuP ) && 0 > miP.refIdx[0], "Invalid reference picture list index" );
  CHECK( CU::isInter( cuP ) && 0 > miQ.refIdx[0], "Invalid reference picture list index" );

  const Picture *piRefP0 = ( CU::isIBC( cuP ) ? sliceP.pic : sliceP.getRefPic( REF_PIC_LIST_0, miP.refIdx[0] ) );
  const Picture *piRefQ0 = ( CU::isIBC( cuQ ) ? sliceQ.pic : sliceQ.getRefPic( REF_PIC_LIST_0, miQ.refIdx[0] ) );

  if( piRefP0 != piRefQ0 )
  {
    lfp.bs |= ( tmpBs + 1 ) & bsMask;
    return;
  }

  Mv mvP0 = miP.mv[0];
  Mv mvQ0 = miQ.mv[0];

  lfp.bs |= ( ( ( abs( mvQ0.hor - mvP0.hor ) >= nThreshold ) || ( abs( mvQ0.ver - mvP0.ver ) >= nThreshold ) ) ? ( tmpBs + 1 ) : tmpBs ) & bsMask;
}

LFCUParam xGetLoopfilterParam( const CodingUnit& cu )
{
  const Slice& slice = *cu.slice;
  if( slice.deblockingFilterDisable )
  {
    return LFCUParam{/*false,*/ false, false};
  }

  const Position pos = cu.blocks[cu.chType].pos();

  LFCUParam stLFCUParam;                   ///< status structure
  stLFCUParam.leftEdge     = ( 0 < pos.x ) && isAvailable ( cu, *CU::getLeft ( cu ), !slice.pps->loopFilterAcrossSlicesEnabled, !slice.pps->loopFilterAcrossTilesEnabled );
  stLFCUParam.topEdge      = ( 0 < pos.y ) && isAvailable ( cu, *CU::getAbove( cu ), !slice.pps->loopFilterAcrossSlicesEnabled, !slice.pps->loopFilterAcrossTilesEnabled );
  return stLFCUParam;
}


static inline int xCalcDP( const Pel* piSrc, const ptrdiff_t iOffset, const bool isChromaHorCTBBoundary = false )
{
  if (isChromaHorCTBBoundary)
  {
    return abs( piSrc[-iOffset * 2] - 2 * piSrc[-iOffset * 2] + piSrc[-iOffset] );
  }
  else
  {
    return abs( piSrc[-iOffset * 3] - 2 * piSrc[-iOffset * 2] + piSrc[-iOffset] );
  }
}

static inline int xCalcDQ( const Pel* piSrc, const ptrdiff_t iOffset )
{
  return abs( piSrc[0] - 2 * piSrc[iOffset] + piSrc[iOffset * 2] );
}

inline bool xUseStrongFiltering( Pel* piSrc, const ptrdiff_t iOffset, const int d, const int beta, const int tc, bool sidePisLarge = false, bool sideQisLarge = false, int maxFilterLengthP = 7, int maxFilterLengthQ = 7, bool isChromaHorCTBBoundary = false )
{
  const Pel m3 = piSrc[-1 * iOffset];
  const Pel m4 = piSrc[ 0          ];

  int shift_beta = (sidePisLarge || sideQisLarge) ? 4 : 2;
  if (!(d < (beta >> shift_beta) && abs(m3 - m4) < ((tc * 5 + 1) >> 1))) return false;

  const Pel m0 = piSrc[-4 * iOffset];
  const Pel m7 = piSrc[ 3 * iOffset];

  const Pel m2 = piSrc[-iOffset * 2];
  int       sp3 = abs(m0 - m3);
  if (isChromaHorCTBBoundary)
  {
    sp3 = abs( m2 - m3 );
  }
  int       sq3      = abs( m7 - m4 );
  const int d_strong = sp3 + sq3;

  if( sidePisLarge || sideQisLarge )
  {
    if( sidePisLarge )
    {
      const Pel mP4 = piSrc[-iOffset * maxFilterLengthP - iOffset];
      if (maxFilterLengthP == 7)
      {
         const Pel mP5 = piSrc[-iOffset * 5];
         const Pel mP6 = piSrc[-iOffset * 6];
         const Pel mP7 = piSrc[-iOffset * 7];;
         sp3 = sp3 + abs(mP5 - mP6 - mP7 + mP4);
      }
      sp3 = ( sp3 + abs( m0 - mP4 ) + 1 ) >> 1;
    }
    if( sideQisLarge )
    {
      const Pel m11 = piSrc[ iOffset * maxFilterLengthQ];
      if (maxFilterLengthQ == 7)
      {
        const Pel m8 = piSrc[iOffset * 4];
        const Pel m9 = piSrc[iOffset * 5];
        const Pel m10 = piSrc[iOffset * 6];;
        sq3 = sq3 + abs(m8 - m9 - m10 + m11);
      }
      sq3 = ( sq3 + abs( m11 - m7 ) + 1 ) >> 1;
    }
    return sp3 + sq3 < ( beta * 3 >> 5 );
  }
  else
  {
    return d_strong < ( beta >> 3 );
  }
}

template<DeblockEdgeDir edgeDir>
void LoopFilter::xEdgeFilterLuma( const CodingStructure& cs, const Position& pos, const LoopFilterParam& lfp, PelUnitBuf& picReco ) const
{
        PelBuf&    picYuvRec   = picReco.bufs[ COMP_Y ];
        Pel*      piSrc        = picYuvRec.bufAt( pos.offset( -m_origin[0].x, -m_origin[0].y ) );
  const ptrdiff_t iStride      = picYuvRec.stride;
  const SPS &     sps          = *cs.sps;
  const Slice &   slice        = *cs.slice;
  const int       bitDepthLuma = sps.bitDepths[ CH_L ];
  const ClpRng &  clpRng       ( cs.slice->clpRngs[ COMP_Y ] );

  const int  betaOffsetDiv2    = slice.deblockingFilterBetaOffsetDiv2[COMP_Y];
  const int  tcOffsetDiv2      = slice.deblockingFilterTcOffsetDiv2[COMP_Y];

  ptrdiff_t offset, srcStep;

  if( edgeDir == EDGE_VER )
  {
    offset   = 1;
    srcStep  = iStride;
  }
  else  // (edgeDir == EDGE_HOR)
  {
    offset   = iStride;
    srcStep  = 1;
  }

#if ENABLE_SIMD_DBLF && defined( TARGET_SIMD_X86 )
  if( offset == 1 )
  {
    _mm_prefetch( (char *) &piSrc[0 * srcStep - 4], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[1 * srcStep - 4], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[2 * srcStep - 4], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[3 * srcStep - 4], _MM_HINT_T0 );
  }
  else
  {
    _mm_prefetch( (char *) &piSrc[( 0 - 4 ) * offset], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 1 - 4 ) * offset], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 2 - 4 ) * offset], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 3 - 4 ) * offset], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 4 - 4 ) * offset], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 5 - 4 ) * offset], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 6 - 4 ) * offset], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 7 - 4 ) * offset], _MM_HINT_T0 );
  }
#endif // ENABLE_SIMD_OPT

  const unsigned uiBs = BsGet( lfp.bs, COMP_Y );

  CHECK( uiBs > 2, "baem0" );
  
  if( !uiBs )
  {
    return;
  }

  int iQP = lfp.qp[0];

  int maxFilterLengthP = ( lfp.sideMaxFiltLength >> 4 ) &   7;
  int maxFilterLengthQ =   lfp.sideMaxFiltLength        &   7;

  bool sidePisLarge = maxFilterLengthP > 3;
  bool sideQisLarge = maxFilterLengthQ > 3;

  if( edgeDir == EDGE_HOR && pos.y % sps.CTUSize == 0 )
  {
    sidePisLarge = false;
  }
 
  const int iIndexTC  = Clip3( 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, int( iQP + DEFAULT_INTRA_TC_OFFSET * ( uiBs - 1 ) + ( tcOffsetDiv2 << 1 ) ) );
  const int iIndexB   = Clip3( 0, MAX_QP, iQP + ( betaOffsetDiv2 << 1 ) );

      const int iTc = bitDepthLuma < 10 ? ((sm_tcTable[iIndexTC] + (1 << (9 - bitDepthLuma))) >> (10 - bitDepthLuma)) : ((sm_tcTable[iIndexTC]) << (bitDepthLuma - 10));
  const int iBeta     = sm_betaTable[iIndexB ] << ( bitDepthLuma - 8 );
  const int iSideThreshold = ( iBeta + ( iBeta >> 1 ) ) >> 3;
  const int iThrCut   = iTc * 10;

  bool bPartPNoFilter = false;
  bool bPartQNoFilter = false;

  if( !bPartPNoFilter || !bPartQNoFilter )
  {
    const int dp0 = xCalcDP( piSrc + srcStep * 0, offset );
    const int dq0 = xCalcDQ( piSrc + srcStep * 0, offset );
    const int dp3 = xCalcDP( piSrc + srcStep * 3, offset );
    const int dq3 = xCalcDQ( piSrc + srcStep * 3, offset );
    const int d0 = dp0 + dq0;
    const int d3 = dp3 + dq3;

    if( sidePisLarge || sideQisLarge )
    {
      const int dp0L = sidePisLarge ? ( ( dp0 + xCalcDP( piSrc + srcStep * 0 - 3 * offset, offset ) + 1 ) >> 1 ) : dp0;
      const int dq0L = sideQisLarge ? ( ( dq0 + xCalcDQ( piSrc + srcStep * 0 + 3 * offset, offset ) + 1 ) >> 1 ) : dq0;
      const int dp3L = sidePisLarge ? ( ( dp3 + xCalcDP( piSrc + srcStep * 3 - 3 * offset, offset ) + 1 ) >> 1 ) : dp3;
      const int dq3L = sideQisLarge ? ( ( dq3 + xCalcDQ( piSrc + srcStep * 3 + 3 * offset, offset ) + 1 ) >> 1 ) : dq3;

      const int d0L = dp0L + dq0L;
      const int d3L = dp3L + dq3L;

      const int dL = d0L + d3L;

      if( dL < iBeta )
      {
        Pel* src0 = piSrc + srcStep * 0;
        Pel* src3 = piSrc + srcStep * 3;

        // adjust decision so that it is not read beyond p5 is maxFilterLengthP is 5 and q5 if maxFilterLengthQ is 5
        const bool swL = xUseStrongFiltering( src0, offset, 2 * d0L, iBeta, iTc, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ )
                      && xUseStrongFiltering( src3, offset, 2 * d3L, iBeta, iTc, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ );
        if( swL )
        {
          xFilteringPandQ( piSrc, srcStep, offset, sidePisLarge ? maxFilterLengthP : 3, sideQisLarge ? maxFilterLengthQ : 3, iTc );

          return;
        }
      }
    }

    //if( !useLongtapFilter )
    {
      const int dp = dp0 + dp3;
      const int dq = dq0 + dq3;
      const int d  = d0  + d3;

      bPartPNoFilter = false;
      bPartQNoFilter = false;

      if( d < iBeta )
      {
        bool bFilterP = false;
        bool bFilterQ = false;
        if (maxFilterLengthP > 1 && maxFilterLengthQ > 1)
        {
          bFilterP = (dp < iSideThreshold);
          bFilterQ = (dq < iSideThreshold);
        }
        bool sw = false;
        if( maxFilterLengthP > 2 && maxFilterLengthQ > 2 )
        {
          sw = xUseStrongFiltering( piSrc + srcStep * 0, offset, 2 * d0, iBeta, iTc )
            && xUseStrongFiltering( piSrc + srcStep * 3, offset, 2 * d3, iBeta, iTc );
        }

        xPelFilterLuma( piSrc, srcStep, offset, iTc, sw, iThrCut, bFilterP, bFilterQ, clpRng );
      }
    }
  }
}

template<DeblockEdgeDir edgeDir>
void LoopFilter::xEdgeFilterChroma( const CodingStructure &cs, const Position &pos, const LoopFilterParam& lfp, PelUnitBuf& picReco ) const
{
  const PreCalcValues &pcv               = *cs.pcv;

  const ChromaFormat nChromaFormat       = pcv.chrFormat;
  const int          csy                 = getChannelTypeScaleY( CH_C, nChromaFormat );
  const unsigned     uiPelsInPartChromaH = pcv.minCUSize  >> getChannelTypeScaleX( CH_C, nChromaFormat );
  const unsigned     uiPelsInPartChromaV = pcv.minCUSize >> csy;

  PelBuf             picYuvRecCb         = picReco.bufs[ COMP_Cb ];
  PelBuf             picYuvRecCr         = picReco.bufs[ COMP_Cr ];
  Pel *              piSrcCb             = picYuvRecCb.bufAt( pos.offset( -m_origin[1].x, -m_origin[1].y ) );
  Pel *              piSrcCr             = picYuvRecCr.bufAt( pos.offset( -m_origin[1].x, -m_origin[1].y ) );
  const ptrdiff_t    iStride             = picYuvRecCb.stride;
  const SPS &        sps                 = *cs.sps;
  const Slice &      slice               = *cs.slice;
  const int          bitDepthChroma      = sps.bitDepths[ CH_C ];

  const int          tcOffsetDiv2[2] = { slice.deblockingFilterTcOffsetDiv2[COMP_Cb], slice.deblockingFilterTcOffsetDiv2[COMP_Cr] };
  const int          betaOffsetDiv2[2] = { slice.deblockingFilterBetaOffsetDiv2[COMP_Cb], slice.deblockingFilterBetaOffsetDiv2[COMP_Cr] };

  ptrdiff_t offset, srcStep;
  unsigned  uiLoopLength;

  if( edgeDir == EDGE_VER )
  {
    offset       = 1;
    srcStep      = iStride;
    uiLoopLength = uiPelsInPartChromaV;
  }
  else
  {
    offset       = iStride;
    srcStep      = 1;
    uiLoopLength = uiPelsInPartChromaH;
  }

  unsigned bS[2];

  unsigned tmpBs = lfp.bs;
  bS[0] = BsGet( tmpBs, COMP_Cb );
  bS[1] = BsGet( tmpBs, COMP_Cr );

  CHECK( bS[0] > 2, "baem1" );
  CHECK( bS[1] > 2, "baem2" );

  if( bS[0] <= 0 && bS[1] <= 0 )
  {
    return;
  }

  bool largeBoundary = lfp.filterCMFL();
  bool isChromaHorCTBBoundary = false;

  if( edgeDir == EDGE_HOR && ( pos.y & ( pcv.maxCUSizeMask >> csy ) ) == 0 )
  {
    isChromaHorCTBBoundary = true;
  }

  bool bPartPNoFilter = false;
  bool bPartQNoFilter = false; 

  if( !bPartPNoFilter || !bPartQNoFilter )
  for( unsigned chromaIdx = 0; chromaIdx < 2; chromaIdx++ )
  {
    if( bS[chromaIdx] == 2 || ( largeBoundary && bS[chromaIdx] == 1 ) )
    {
      const ClpRng& clpRng( cs.slice->clpRngs[ComponentID( chromaIdx + 1 )] );

      int iQP = lfp.qp[chromaIdx + 1];
      const int iIndexTC = Clip3<int>(0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, iQP + DEFAULT_INTRA_TC_OFFSET * (bS[chromaIdx] - 1) + (tcOffsetDiv2[chromaIdx] << 1));
        const int iTc = bitDepthChroma < 10 ? ((sm_tcTable[iIndexTC] + (1 << (9 - bitDepthChroma))) >> (10 - bitDepthChroma)) : ((sm_tcTable[iIndexTC]) << (bitDepthChroma - 10));
      Pel* piSrcChroma   = chromaIdx == 0 ? piSrcCb : piSrcCr;

      bool useLongFilter = false;

      if( largeBoundary )
      {
        const int iBitdepthScale = 1 << ( sps.bitDepths[CH_C] - 8 );
        const int indexB = Clip3<int>(0, MAX_QP, iQP + (betaOffsetDiv2[chromaIdx] << 1));
        const int beta   = sm_betaTable[indexB] * iBitdepthScale;

        const int dp0 = xCalcDP( piSrcChroma, offset, isChromaHorCTBBoundary );
        const int dq0 = xCalcDQ( piSrcChroma, offset );
        const int subSamplingShift = ( edgeDir == EDGE_VER ) ? getChannelTypeScaleY( CH_C, nChromaFormat ) : getChannelTypeScaleX( CH_C, nChromaFormat );
        const int dp3 = ( subSamplingShift == 1 ) ? xCalcDP(piSrcChroma + srcStep, offset, isChromaHorCTBBoundary) : xCalcDP(piSrcChroma + srcStep*3, offset, isChromaHorCTBBoundary);
        const int dq3 = ( subSamplingShift == 1 ) ? xCalcDQ(piSrcChroma + srcStep, offset) : xCalcDQ(piSrcChroma + srcStep*3, offset);

        const int d0 = dp0 + dq0;
        const int d3 = dp3 + dq3;
        const int d  = d0  + d3;

        if( d < beta )
        {
          useLongFilter = true;
          const bool sw = xUseStrongFiltering( piSrcChroma, offset, 2 * d0, beta, iTc, false, false, 7, 7, isChromaHorCTBBoundary )
                       && xUseStrongFiltering( piSrcChroma + ( ( subSamplingShift == 1 ) ? srcStep : srcStep*3 ), offset, 2 * d3, beta, iTc, false, false, 7, 7, isChromaHorCTBBoundary );
          for( unsigned uiStep = 0; uiStep < uiLoopLength; uiStep++ )
          {
            xPelFilterChroma( piSrcChroma + srcStep * uiStep, offset, iTc, sw, bPartPNoFilter, bPartQNoFilter, clpRng, largeBoundary, isChromaHorCTBBoundary );
          }
        }
      }
      if ( !useLongFilter )
      {
        for( unsigned uiStep = 0; uiStep < uiLoopLength; uiStep++ )
        {
          xPelFilterChroma( piSrcChroma + srcStep * uiStep, offset, iTc, false, bPartPNoFilter, bPartQNoFilter, clpRng, largeBoundary, isChromaHorCTBBoundary );
        }
      }
    }
  }
}

}

//! \}

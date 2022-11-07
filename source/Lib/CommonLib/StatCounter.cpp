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


#include "CommonDef.h"
#include "StatCounter.h"

#include <string>
#include <ostream>
#include <sstream>
#include <vector>
#include <cstdarg>

//! \ingroup CommonLib
//! \{

namespace vvenc {

#define OUTPUT(...) { sprintf( cStr, __VA_ARGS__ ); m_str << cStr; }
#define OUTSTR( _w, ... ) { m_str << std::setw( _w ) << __VA_ARGS__; }
#define OUTSTRC( _w, _c ) { m_str << std::setfill( _c ) << std::setw( _w  ) << _c << std::setfill( ' ' ); }
#define OUTSTRF( _w, _p, ... ) { m_str << std::fixed << std::setw( _w ) << std::setprecision(_p) << __VA_ARGS__; }
#define OUTPUT_COND_SIZE_IDX(_stype,_idx,...) { sprintf( cStr, __VA_ARGS__ ); m_str << cStr; }
#define MIN_SIZE_IDX 2
#define IDX_TO_SIZE(_i) (1<<_i)

////////////////////////////////////////////////////////////////////////////////////
// report2D

template<typename T>
std::ostream& StatCounters::report2D( std::ostream& os, const StatCounter2DSet<T>& cntSet, bool axisInBlockSizes, bool absoluteNumbers, bool weightedByArea, bool secondColumnInPercentage, bool ratiosWithinSingleElement, int refCntId )
{
  char cStr[512];
  std::stringstream m_str;

  CHECK( !axisInBlockSizes && weightedByArea, "Mode is not supported" );

  //OUTPUT( "DISTRIBUTION (in %s \r\n", isAbsolute ? "number of occurances)": ( !isWeightedByArea ? "in % of blocks)" : "in % of area)" ) );
  const size_t xDim = cntSet.getDimHor();
  const size_t yDim = cntSet.getDimVer();
  const size_t totalNumCnt = cntSet.getNumCntTypes();
  const int firstSizeIdx = axisInBlockSizes ? MIN_SIZE_IDX: 0;

  double totalArea = 0.0;
  if( weightedByArea && !ratiosWithinSingleElement )
  {
    for( int j = firstSizeIdx; j < yDim; j++ )
    {
      for( int cntIdx = firstSizeIdx; cntIdx < totalNumCnt; cntIdx++ )
      {
        for( int i = 0; i < xDim; i++ )
        {
          size_t blockArea = IDX_TO_SIZE( j )  * IDX_TO_SIZE( i );
          totalArea += blockArea * cntSet[cntIdx][j][i];
        }
      }
    }
  }

  // Get max string length across all counter names
  int maxCntNameLen = 0;
  for( int cntIdx = 0; cntIdx < totalNumCnt; cntIdx++ )
  {
    int len = (int)cntSet[cntIdx].getName().length();
    if( len > maxCntNameLen )
      maxCntNameLen = len;
  }
  int leftW = 12 + maxCntNameLen + 2;

  // Determine the max symbols (numbers) in the string of the counter output
  const size_t maxSymbolsInMantissa = 6;
  size_t numSymbolsInMantissa = 0;
  for( int j = firstSizeIdx; j < yDim; j++ )
  {
    for( int i = firstSizeIdx; i < xDim; i++ )
    {
      for( int cntIdx = 0; cntIdx < totalNumCnt; cntIdx++ )
      {
        std::stringstream ss;
        ss << std::fixed << std::setprecision( 1 ) << ( cntSet[cntIdx][j][i] );
        const size_t syms = ss.str().size();
        if( syms > numSymbolsInMantissa )
          numSymbolsInMantissa = syms;
      }
    }
  }

  // Derive scaling factor depending of the range of the numbers
  int64_t scalingFactor = 1;
  while( numSymbolsInMantissa > maxSymbolsInMantissa + 3 )
  {
    scalingFactor *= 1000;
    numSymbolsInMantissa -= 3;
  }
  numSymbolsInMantissa = std::max<size_t>( maxSymbolsInMantissa, numSymbolsInMantissa );
  const size_t numSymbolsInExp = absoluteNumbers && secondColumnInPercentage ? 6: 0; // second value in percentage
  const size_t numTotalValSyms = numSymbolsInMantissa + numSymbolsInExp + 1;
  if( absoluteNumbers && scalingFactor > 1 )
  {
    OUTPUT( "Scaling factor = %lld\r\n", (long long)scalingFactor );
  }

  // Legend over X-axis

  OUTSTR( leftW, " " );
  for( int i = firstSizeIdx; i < xDim; i++ )
  {
    OUTSTR( numTotalValSyms, (axisInBlockSizes ? IDX_TO_SIZE( i ): i) );
  }
  OUTSTR( 0, "\r\n" );
  OUTSTR( leftW, " " );
  OUTSTRC( ((int)(numTotalValSyms * (xDim - firstSizeIdx))), '-' );
  OUTSTR( 0, "\r\n" );

  // Data over Y and X axis

  // Derive counters that contains the data
  std::vector<T> cntAccum;
  std::vector<T> cntAccumDimVer( xDim );
  for( int cntIdx = 0; cntIdx < totalNumCnt; cntIdx++ )
  {
    cntAccum.push_back( cntSet[cntIdx].total() );
  }

  // --- Loop over heights ---
  for( int j = firstSizeIdx; j < yDim; j++ )
  {
    OUTSTR( 9, (axisInBlockSizes ? IDX_TO_SIZE( j ): j)  );

    // --- Loop over counters ---
    int outputCount = 0;
    for( int cntIdx = 0; cntIdx < totalNumCnt; cntIdx++ )
    {
      // Skip counter if it doesn't contain data
      if( cntAccum[cntIdx] == 0 )
        continue;

      // Counter Name

      OUTSTR( outputCount == 0 ? 0 : 12, " | " );
      OUTSTR( maxCntNameLen, std::left << cntSet[cntIdx].getName().c_str() << std::internal );
      OUTSTR( 2, " " );
      outputCount++;

      // --- Loop over widths ---
      for( int i = firstSizeIdx; i < xDim; i++ )
      {
        double val = (double)cntSet[cntIdx][j][i];
          
        if( val > 0 )
        {
          OUTSTR( 0, " " );
          if( absoluteNumbers )
          {
            OUTSTR( numSymbolsInMantissa, (int)(val / (double)scalingFactor) );
            if( secondColumnInPercentage )
            {
              double totalAmount = ratiosWithinSingleElement ? ( refCntId >= 0 ? cntSet[refCntId][j][i] : cntSet.total( i, j ) ) : ( refCntId >= 0 ? cntSet.total( refCntId ) : cntSet.total() );
              val = val * (double)100.0 / totalAmount;
              OUTSTRF( numSymbolsInExp, 1, val );
            }
          }
          else
          {
            double totalAmount = ratiosWithinSingleElement ? ( refCntId >= 0 ? cntSet[refCntId][j][i] : cntSet.total( i, j ) ) : ( refCntId >= 0 ? cntSet.total( refCntId ) : cntSet.total() );
            if( !weightedByArea )
            {
              val = ((cntSet[cntIdx][j][i])) * (double)100.0 / totalAmount;
              OUTSTRF( numSymbolsInMantissa, 1, val );
            }
            else
            {
              int blockArea = IDX_TO_SIZE( j )  * IDX_TO_SIZE( i );
              val = (blockArea * (cntSet[cntIdx][j][i])) * (double)100.0 / ( ratiosWithinSingleElement ? totalAmount * blockArea : totalArea );
              OUTSTRF( numSymbolsInMantissa, 1, val );
            }
          }
          cntAccumDimVer[i] += cntSet[cntIdx][j][i];
        }
        else
        {
          OUTSTR( numTotalValSyms, "." );
        }
      }
      OUTSTR( 0, "\r\n" );
    }
    // Empty line to separate heights
    OUTSTR( 0, "\r\n" );
  }

  // Total over y-axis
  OUTSTR( 12, "   " );
  OUTSTR( maxCntNameLen, std::left << "Total" << std::internal );
  OUTSTR( 2, " " );
  for( int i = firstSizeIdx; i < xDim; i++ )
  {
    OUTSTR( 0, " " );
    OUTSTR( numSymbolsInMantissa, (int)(cntAccumDimVer[i] / (double)scalingFactor) );
    OUTSTR( numSymbolsInExp, "." );
  }

  OUTSTR( 0, "\r\n" );


  os << m_str.str();
  return os;
}

template std::ostream& StatCounters::report2D<double>( std::ostream& os, const StatCounter2DSet<double>& counters, bool axisInBlockSizes, bool absoluteNumbers, bool weightedByArea, bool secondColumnInPercentage, bool ratiosWithinSingleElement, int refCntId );
template std::ostream& StatCounters::report2D<int64_t>( std::ostream& os, const StatCounter2DSet<int64_t>& counters, bool axisInBlockSizes, bool absoluteNumbers, bool weightedByArea, bool secondColumnInPercentage, bool ratiosWithinSingleElement, int refCntId );

} // namespace vvenc

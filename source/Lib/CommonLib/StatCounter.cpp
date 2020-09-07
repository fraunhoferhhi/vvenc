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
  OUTSTRC( (numTotalValSyms * (xDim - firstSizeIdx)), '-' );
  OUTSTR( 0, "\r\n" );

  // Data over Y and X axis

  // Derive counters that contains the data
  std::vector<T> cntAccum;
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

  os << m_str.str();
  return os;
}

template std::ostream& StatCounters::report2D<double>( std::ostream& os, const StatCounter2DSet<double>& counters, bool axisInBlockSizes, bool absoluteNumbers, bool weightedByArea, bool secondColumnInPercentage, bool ratiosWithinSingleElement, int refCntId );
template std::ostream& StatCounters::report2D<int64_t>( std::ostream& os, const StatCounter2DSet<int64_t>& counters, bool axisInBlockSizes, bool absoluteNumbers, bool weightedByArea, bool secondColumnInPercentage, bool ratiosWithinSingleElement, int refCntId );

} // namespace vvenc

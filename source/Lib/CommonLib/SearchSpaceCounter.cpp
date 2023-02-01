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


#include "CommonDef.h"
#include "SearchSpaceCounter.h"
#include "Unit.h"

#if ENABLE_MEASURE_SEARCH_SPACE

namespace vvenc {

inline int Log2( uint32_t x )
{
  return floorLog2( x );
}

void SearchSpaceAccumulator::addQuant( const struct UnitArea& area, int chType )
{
  int wIdx = Log2( area.blocks[chType].width );
  int hIdx = Log2( area.blocks[chType].height );

  quant[currIsIntra][currTId][wIdx][hIdx][chType]++;
}

void SearchSpaceAccumulator::addPartition( const struct UnitArea& area, int chType )
{
  if( chType != CH_C )
  {
    int wIdx = Log2( area.Y().width );
    int hIdx = Log2( area.Y().height );

    parts[currIsIntra][currTId][wIdx][hIdx][0]++;
  }

  if( chType != CH_L )
  {
    int wIdx = Log2( area.Cb().width );
    int hIdx = Log2( area.Cb().height );

    // will only be called once per chType, i.e. once per two blocks
    parts[currIsIntra][currTId][wIdx][hIdx][1]++;
    parts[currIsIntra][currTId][wIdx][hIdx][1]++;
  }
}

void SearchSpaceAccumulator::addPrediction( const int w, const int h, int chType )
{
  int wIdx = Log2( w );
  int hIdx = Log2( h );

  preds[currIsIntra][currTId][wIdx][hIdx][chType]++;
}

void SearchSpaceAccumulator::printQuantizationStats() const
{
  std::cout << std::endl << std::endl << "QUANTIZATION STATS" << std::endl;
    
  printStats( quant );
}

void SearchSpaceAccumulator::printPredictionsStats() const
{
  std::cout << std::endl << std::endl << "PREDICTION STATS" << std::endl;

  printStats( preds );
}

void SearchSpaceAccumulator::printPartitioningStats() const
{
  std::cout << std::endl << std::endl << "PARTITIONING STATS" << std::endl;
    
  printStats( parts );
}

void SearchSpaceAccumulator::printStats( const size_t stat[2][6][8][8][2] ) const
{
  std::cout << std::fixed << std::setprecision( 2 ) << std::setw( 10 );

  std::cout << " I-Slices" << std::endl;

  size_t sumArea = 0;

  for( int wIdx = 2; wIdx <= 7; wIdx++ )
  {
    for( int hIdx = 2; hIdx <= 7; hIdx++ )
    {
      sumArea += ( 1 <<   wIdx       ) * ( 1 <<   hIdx       ) * stat[1][0][wIdx    ][hIdx    ][0];
      sumArea += ( 1 << ( wIdx - 1 ) ) * ( 1 << ( hIdx - 1 ) ) * stat[1][0][wIdx - 1][hIdx - 1][1];
    }
  }

  std::cout << "  Intra: tested " << ( sumArea / 1000 ) << "k samples: " << ( 100.0 * sumArea / ( picW * picH * 1.5 * slices[1][0] ) ) << "%" << std::endl;

  sumArea = 0;
  size_t numSlices = 0;
    
  std::cout << std::endl << " P/B-Slices" << std::endl;

  for( int t = 0; t < 6; t++ )
  {
    size_t sumAreaTId = 0;

    for( int wIdx = 2; wIdx <= 7; wIdx++ )
    {
      for( int hIdx = 2; hIdx <= 7; hIdx++ )
      {
        sumAreaTId += ( 1 <<   wIdx       ) * ( 1 <<   hIdx       ) * stat[0][t][wIdx    ][hIdx    ][0];
        sumAreaTId += ( 1 << ( wIdx - 1 ) ) * ( 1 << ( hIdx - 1 ) ) * stat[0][t][wIdx - 1][hIdx - 1][1];
      }
    }
      
    //if( sumAreaTId != 0 )
    //{
      std::cout << "  TID " << t << ": tested " << ( sumAreaTId / 1000 ) << "k samples: " << ( 100.0 * ( sumAreaTId ? sumAreaTId / ( picW * picH * 1.5 * slices[0][t] ) : 0 ) ) << "%" << std::endl;
      sumArea += sumAreaTId;
    //}
    numSlices += slices[0][t];
  }
    
  std::cout << "  TID *: tested " << ( sumArea / 1000 ) << "k samples: " << ( 100.0 * sumArea / ( picW * picH * 1.5 * numSlices ) ) << "%" << std::endl;
}

SearchSpaceAccumulator g_searchSpaceAcc;

} // namespace vvenc

#endif

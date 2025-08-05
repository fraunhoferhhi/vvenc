/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
#pragma once

#if ENABLE_MEASURE_SEARCH_SPACE

namespace vvenc {

struct SearchSpaceAccumulator
{
  // intra -> tId -> wIdx -> hIdx -> chType
  size_t parts[2][6][8][8][2];
  size_t quant[2][6][8][8][2];
  size_t preds[2][6][8][8][2];
  size_t picW;
  size_t picH;
  size_t slices[2][6];
  int    currTId;
  bool   currIsIntra;

  SearchSpaceAccumulator()
  {
    memset( parts,  0, sizeof( parts ) );
    memset( quant,  0, sizeof( quant ) );
    memset( preds,  0, sizeof( preds ) );
    memset( slices, 0, sizeof( slices ) );
  }

  ~SearchSpaceAccumulator()
  {
    printPredictionsStats();
    printQuantizationStats();
    printPartitioningStats();
  }

  void addSlice( bool intra, int tId )
  {
    slices[intra][tId]++;

    currIsIntra = intra;
    currTId     = tId;
  }

  void addQuant     ( const struct UnitArea& area, int chType );
  void addPartition ( const struct UnitArea& area, int chType );
  void addPrediction( const int w, const int h, int chType );

private:

  void printQuantizationStats() const;
  void printPartitioningStats() const;
  void printPredictionsStats()  const;

  void printStats( const size_t stat[2][6][8][8][2] ) const;
};

extern SearchSpaceAccumulator g_searchSpaceAcc;

} // namespace vvenc

#endif

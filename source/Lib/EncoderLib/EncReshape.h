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
 /** \file     EncReshape.h
     \brief    encoder reshaping header and class (header)
 */

#pragma once

#include "vvenc/vvencCfg.h"
#include "CommonLib/Reshape.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================
struct SeqInfo
{
  SeqInfo()
  : binVar { 0.0 }
  , binHist { 0.0 }
  , normVar { 0.0 }
  , nonZeroCnt(0)
  , weightVar ( 0.0 )
  , weightNorm ( 0.0 )
  , minBinVar ( 0.0 )
  , maxBinVar ( 0.0 )
  , meanBinVar ( 0.0 )
  , ratioStdU ( 0.0 )
  , ratioStdV ( 0.0 )
  {
  }
  double binVar[PIC_ANALYZE_CW_BINS];
  double binHist[PIC_ANALYZE_CW_BINS];
  double normVar[PIC_ANALYZE_CW_BINS];
  int    nonZeroCnt;
  double weightVar;
  double weightNorm;
  double minBinVar;
  double maxBinVar;
  double meanBinVar;
  double ratioStdU;
  double ratioStdV;
};

class EncReshape : public Reshape
{
private:
  bool                    m_exceedSTD;
  std::vector<uint32_t>   m_binImportance;
  int                     m_tcase;
  int                     m_rateAdpMode;
  bool                    m_useAdpCW;
  uint16_t                m_initCWAnalyze;
  vvencReshapeCW          m_reshapeCW;
  Pel                     m_cwLumaWeight[PIC_CODE_CW_BINS];
  double                  m_chromaWeight;
  int                     m_chromaAdj;
  int                     m_binNum;
  SeqInfo                 m_srcSeqStats;
  SeqInfo                 m_rspSeqStats;
public:

  EncReshape();
  ~EncReshape();

  void init( const VVEncCfg& encCfg );
  void destroy();

  void calcSeqStats     ( Picture& pic, SeqInfo &stats);
  void preAnalyzerLMCS  ( Picture& pic, const uint32_t signalType, const SliceType sliceType, const vvencReshapeCW& reshapeCW);
  void preAnalyzerHDR   ( Picture& pic, const SliceType sliceType, const vvencReshapeCW& reshapeCW );
  void bubbleSortDsd    ( double *array, int * idx, int n);
  void cwPerturbation   ( int startBinIdx, int endBinIdx, uint16_t maxCW);
  void cwReduction      ( int startBinIdx, int endBinIdx);
  void deriveReshapeParametersSDR ( bool *intraAdp, bool *interAdp);
  void deriveReshapeParameters    ( double *array, int start, int end, vvencReshapeCW respCW, double &alpha, double &beta);
  void initLUTfromdQPModel  ();
  void constructReshaperLMCS();
  void adjustLmcsPivot      ();
  vvencReshapeCW * getReshapeCW  () { return &m_reshapeCW; }
  Pel*        getWeightTable() { return m_cwLumaWeight; }
  double      getCWeight    () { return m_chromaWeight; }

};// END CLASS DEFINITION EncReshape

} // namespace vvenc

//! \}

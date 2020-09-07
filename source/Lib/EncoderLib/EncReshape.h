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
 /** \file     EncReshape.h
     \brief    encoder reshaping header and class (header)
 */

#pragma once

#include "../../../include/vvenc/EncCfg.h"
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
  ReshapeCW               m_reshapeCW;
  Pel                     m_cwLumaWeight[PIC_CODE_CW_BINS];
  double                  m_chromaWeight;
  int                     m_chromaAdj;
  int                     m_binNum;
  SeqInfo                 m_srcSeqStats;
  SeqInfo                 m_rspSeqStats;
public:

  EncReshape();
  ~EncReshape();

  void init( const EncCfg& encCfg );
  void destroy();

  void calcSeqStats     ( Picture& pic, SeqInfo &stats);
  void preAnalyzerLMCS  ( Picture& pic, const uint32_t signalType, const SliceType sliceType, const ReshapeCW& reshapeCW);
  void preAnalyzerHDR   ( Picture& pic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT);
  void bubbleSortDsd    ( double *array, int * idx, int n);
  void cwPerturbation   ( int startBinIdx, int endBinIdx, uint16_t maxCW);
  void cwReduction      ( int startBinIdx, int endBinIdx);
  void deriveReshapeParametersSDR ( bool *intraAdp, bool *interAdp);
  void deriveReshapeParameters    ( double *array, int start, int end, ReshapeCW respCW, double &alpha, double &beta);
  void initLUTfromdQPModel  ();
  void constructReshaperLMCS();
  void adjustLmcsPivot      ();
  ReshapeCW * getReshapeCW  () { return &m_reshapeCW; }
  Pel*        getWeightTable() { return m_cwLumaWeight; }
  double      getCWeight    () { return m_chromaWeight; }

};// END CLASS DEFINITION EncReshape

} // namespace vvenc

//! \}

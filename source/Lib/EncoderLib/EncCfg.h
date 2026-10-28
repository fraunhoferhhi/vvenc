/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     EncCfg.h
    \brief    encoder internal configuration class (header)
*/

#pragma once

#include "vvenc/vvencCfg.h"

#define VVENC_MAX_NUM_INTENSITIES             256
#define VVENC_MAX_NUM_MODEL_VALUES            6

namespace vvenc {

typedef struct vvencFG
{
  bool      m_fgcSEIEnabled;
  bool      m_fgcSEICancelFlag;
  bool      m_fgcSEIPersistenceFlag;
  uint8_t   m_fgcSEIModelID;
  bool      m_fgcSEISepColourDescPresentFlag;
  uint8_t   m_fgcSEIBlendingModeID;
  uint8_t   m_fgcSEILog2ScaleFactor;
  bool      m_fgcSEICompModelPresent[VVENC_MAX_NUM_COMP];
  bool      m_fgcSEIPerPictureSEI;
  uint8_t   m_fgcSEINumModelValuesMinus1[VVENC_MAX_NUM_COMP];
  uint8_t   m_fgcSEINumIntensityIntervalMinus1[VVENC_MAX_NUM_COMP];
  uint8_t   m_fgcSEIIntensityIntervalLowerBound[VVENC_MAX_NUM_COMP][VVENC_MAX_NUM_INTENSITIES];
  uint8_t   m_fgcSEIIntensityIntervalUpperBound[VVENC_MAX_NUM_COMP][VVENC_MAX_NUM_INTENSITIES];
  uint32_t  m_fgcSEICompModelValue[VVENC_MAX_NUM_COMP][VVENC_MAX_NUM_INTENSITIES][VVENC_MAX_NUM_MODEL_VALUES];
}vvencFG;

struct VVEncCfg : public vvenc_config
{

  VVEncCfg();

  VVEncCfg& operator= ( const vvenc_config& extern_cfg );

  // m_QP is the coding QP (getCodingQp), which differs from the configured QP with CuTreePicStrength;
  // vvenc_get_config still returns the configured one

  // Both kinds of block importance mapping deliver per-CTU QP offsets through PicShared::m_ctuBimQpOffset and a
  // picture offset through m_picAuxQpOffset. BIM 2 takes both from MCTF; CU-tree (BIM 1) takes the picture offset from
  // MCTF (with CuTreePicStrength from its own look-ahead too) and the CTU offsets from its own look-ahead.
  bool hasBlockQpOffsets() const { return m_blockImportanceMapping > 0; }
  bool useCuTree()         const { return m_blockImportanceMapping == 1; }

  bool      m_stageParallelProc;
  bool      m_salienceBasedOpt;
  bool      m_rateCap;
  int       m_log2GopSize;
  int       m_maxTLayer;
  int       m_bimCtuSize;
  unsigned  m_MaxQT[3];
  int       m_maxMergeRdCandNumTotal;
  int       m_mergeRdCandQuotaRegular;
  int       m_mergeRdCandQuotaRegularSmallBlk;
  int       m_mergeRdCandQuotaSubBlk;
  int       m_mergeRdCandQuotaCiip;
  int       m_mergeRdCandQuotaGpm;
  bool      m_reuseCuResults;
  int       m_splitCostThrParamId;
  vvencFG   m_fg;
  int       m_internalUsePerceptQPATempFiltISlice;
  bool      m_disableForce2ndOderFilter;
  double    m_cuTreeScale;      // scale of CU-tree's QP offsets, decoded from the 5-bit fixed point m_cuTreeStrength
  double    m_cuTreePicScale;   // scale of CU-tree's picture QP offsets, decoded from m_cuTreePicStrength (0: off, the QP cascade)
  int       m_configuredQP;     // the QP as configured; m_QP is the coding QP (getCodingQp)
  double    m_QPFraction;       // m_QPFraction in QP units (0 <= value < 1)
  double    m_fgaStrength;      // Film grain strength as the fraction of the analysed strength (0 <= value <  4)


private:
  void xInitCfgMembers();
};

// The QP the encoder codes at for a configuration. CuTreePicStrength replaces the QP cascade, which puts the
// unreferenced pictures 7 to 9 QP above the configured QP (QPA on, GOP 32), with picture offsets that only lower the
// referenced pictures' QPs. Without a correction the same QP would code about 8 QP finer, at several times the rate, so
// the coding QP adds round( 3 + 2 * strength ) (7 at strength 2, where it was measured), which keeps the mean rate
// at a given QP within about x0.8 to x1.2 of master's (15 clips, QP 22 to 37), like the CRF offset x264 and x265
// apply for MB-tree (mbtree_offset). Every QP-dependent default and tool uses the coding QP.
int getCodingQp( const vvenc_config& c );

}



/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     EncCfg.cpp
    \brief    encoder internal configuration class
*/


#include "EncCfg.h"
#include "CommonLib/CommonDef.h"

namespace vvenc {

VVEncCfg::VVEncCfg()
  : m_stageParallelProc( false )
  , m_salienceBasedOpt( true )
  , m_rateCap ( false )
  , m_bimCtuSize( 64 )
  , m_MaxQT { 128, 128, 128 }
{
  m_fg.m_fgcSEIEnabled = false;
  m_fg.m_fgcSEICompModelPresent[0] = false;
  m_fg.m_fgcSEICompModelPresent[1] = false;
  m_fg.m_fgcSEICompModelPresent[2] = false;
}

VVEncCfg& VVEncCfg::operator= ( const vvenc_config& extern_cfg )
{
  *((vvenc_config*)this) = extern_cfg;
  xInitCfgMembers();

  return *this;
}

void VVEncCfg::xInitCfgMembers()
{
  m_stageParallelProc = m_numThreads > 0 && m_maxParallelFrames > 0;
  m_log2GopSize       = floorLog2( m_GOPSize );
  m_maxTLayer         = m_picReordering && m_GOPSize > 1 ? vvenc::ceilLog2( m_GOPSize ) : 0;
  m_bimCtuSize        = m_CTUSize;
  m_MaxQT[0] = m_MaxQT[1] = m_MaxQT[2] = m_CTUSize;
  m_rateCap = m_RCMaxBitrate > 0 && m_RCMaxBitrate < INT32_MAX && m_RCTargetBitrate == 0;

  if ( this->m_fga )
  {
    memset( m_fg.m_fgcSEIIntensityIntervalLowerBound, 0, sizeof(uint8_t) * 3 * VVENC_MAX_NUM_INTENSITIES );
    memset( m_fg.m_fgcSEIIntensityIntervalUpperBound, 255, sizeof(uint8_t) * 3 * VVENC_MAX_NUM_INTENSITIES );
    memset( m_fg.m_fgcSEICompModelValue, 0, sizeof(uint32_t) * 3 * VVENC_MAX_NUM_INTENSITIES * VVENC_MAX_NUM_MODEL_VALUES );
    m_fg.m_fgcSEIEnabled = true;
    m_fg.m_fgcSEICompModelPresent[0] = true; // FGA is performed only for the luma plane
    m_fg.m_fgcSEICompModelPresent[1] = true;
    m_fg.m_fgcSEICompModelPresent[2] = true;
    m_fg.m_fgcSEICancelFlag = false;
    m_fg.m_fgcSEIPersistenceFlag = true;
    m_fg.m_fgcSEIModelID = 0;
    m_fg.m_fgcSEISepColourDescPresentFlag = false;
    m_fg.m_fgcSEIBlendingModeID = 0;
    m_fg.m_fgcSEILog2ScaleFactor = 0;
    m_fg.m_fgcSEIPerPictureSEI = false;
    m_fg.m_fgcSEINumIntensityIntervalMinus1[0] = 0;
    m_fg.m_fgcSEINumIntensityIntervalMinus1[1] = 0;
    m_fg.m_fgcSEINumIntensityIntervalMinus1[2] = 0;
    m_fg.m_fgcSEINumModelValuesMinus1[0] = 0;
    m_fg.m_fgcSEINumModelValuesMinus1[1] = 0;
    m_fg.m_fgcSEINumModelValuesMinus1[2] = 0;
  }
}

}


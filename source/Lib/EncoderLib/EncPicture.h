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
/** \file     EncPicture.h
    \brief    encode picture (header)
*/

#pragma once

#include "EncSlice.h"
#include "EncSampleAdaptiveOffset.h"
#include "EncAdaptiveLoopFilter.h"
#include "CommonLib/LoopFilter.h"
#include "Utilities/NoMallocThreadPool.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class EncGOP;

// ---------------------------------------------------------------------------------------------------------------------

class EncPicture
{
  private:
    const VVEncCfg*          m_pcEncCfg;
    EncSlice                 m_SliceEncoder;
    LoopFilter               m_LoopFilter;
    EncAdaptiveLoopFilter    m_ALF;

    BitEstimator             m_BitEstimator;
    CABACWriter              m_CABACEstimator;
    CtxCache                 m_CtxCache;
    RateCtrl*                m_pcRateCtrl;

  public:
    WaitCounter              m_ctuTasksDoneCounter;

  public:
    EncPicture()
      : m_pcEncCfg      ( nullptr )
      , m_CABACEstimator( m_BitEstimator )
      , m_pcRateCtrl    ( nullptr )
    {}
    virtual ~EncPicture() {}

    void init                   ( const VVEncCfg& encCfg,
                                  std::vector<int>* const globalCtuQpVector,
                                  const SPS& sps,
                                  const PPS& pps,
                                  RateCtrl& rateCtrl,
                                  NoMallocThreadPool* threadPool );
    void compressPicture        ( Picture& pic, EncGOP& gopEncoder );
    void skipCompressPicture    ( Picture& pic, ParameterSetMap<APS>& shrdApsMap );
    void finalizePicture        ( Picture& pic );

  protected:
    void xInitPicEncoder        ( Picture& pic );
    void xWriteSliceData        ( Picture& pic );
    void xCalcDistortion        ( Picture& pic, const SPS& sps );

    void xInitSliceColFromL0Flag( Slice* slice ) const;
    void xInitSliceCheckLDC     ( Slice* slice ) const;
};

} // namespace vvenc

//! \}


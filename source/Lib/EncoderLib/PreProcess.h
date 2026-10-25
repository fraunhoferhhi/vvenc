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


/** \file     PreProcess.h
    \brief
*/


#pragma once

#include "CommonLib/CommonDef.h"
#include "EncStage.h"
#include "GOPCfg.h"
#include "BitAllocation.h"
#include "CommonLib/MCTF.h"
#include <map>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================

class PreProcess : public EncStage
{
  private:
    const VVEncCfg* m_encCfg;
    GOPCfg          m_gopCfg;
    int             m_lastPoc;
    // adaptive intra refresh: pictures held ahead of the GOP structure (a GOP plus one, so that a cut anywhere in
    // the next GOP is confirmed before that GOP is laid out); 0 = off, GOP entries are handed out on arrival
    int             m_lookahead   = 0;
    int             m_lastSeenPoc = std::numeric_limits<int>::min();   // newest picture the cut detector has seen
    // adaptive intra refresh: whether an I picture pays at a cut or a slice type adaptation candidate is decided from
    // a motion search on low-resolution luma (the MCTF's integer search) against a DC-intra estimate
    MCTF            m_motionSearch;
    PelStorage      m_halfRes;                                 // half resolution luma, a stage of the downsampling
    std::map<std::pair<int, int>, double> m_predCosts;      // prediction error by (POC, reference POC)
    std::map<int, double>                 m_dcIntraCosts;   // DC-intra estimate by POC
    bool            m_isHighRes;
    bool            m_doSTA;
    bool            m_doTempDown;
    bool            m_doVisAct;
    bool            m_doVisActQpa;
    bool            m_cappedCQF;

    // Scene cut detection for the film grain analysis: the luma histogram of the previous picture and its distance to
    // the one before it, the picture's shared data (it is marked once the next picture confirms the cut) and the
    // latest cut found.
    static constexpr int FGA_HIST_BINS = 64;
    std::vector<int> m_fgaPrevHist;
    int              m_fgaPrevPoc      = std::numeric_limits<int>::min();
    int              m_fgaPrevDistance = 0;
    PicShared*       m_fgaPrevShared   = nullptr;
    int              m_fgaLastCutPoc   = std::numeric_limits<int>::min();

  public:
    PreProcess( MsgLog& _m );
    virtual ~PreProcess();

    void init( const VVEncCfg& encCfg, bool isFinalPass, NoMallocThreadPool* threadPool );

    const GOPCfg* getGOPCfg() const { return &m_gopCfg; };
    int           lookahead() const { return m_lookahead; }

  protected:
    virtual void initPicture    ( Picture* pic );
    virtual void processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList );

  private:
    void     xFreeUnused          ( Picture* pic, const PicList& picList, PicList& doneList, PicList& freeList ) const;
    void     xGetPrevPics         ( const Picture* pic, const PicList& picList, const Picture* prevPics[ NUM_QPA_PREV_FRAMES ] ) const;
    Picture* xGetPrevTL0Pic       ( const Picture* pic, const PicList& picList ) const;
    Picture* xGetStartOfLastGop   ( const PicList& picList, int maxPoc = INT32_MAX ) const;
    void     xLinkPrevQpaBufs     ( Picture* pic, const PicList& picList ) const;
    void     xGetVisualActivity   ( Picture* pic, const PicList& picList ) const;
    void     xGetSpatialActivity  ( Picture* pic, bool doLuma, bool doChroma, VisAct va[ MAX_NUM_CH ] ) const;
    void     xGetTemporalActivity ( Picture* curPic, const Picture* refPic1, const Picture* refPic2, VisAct& va ) const;
    void     xDetectSTA           ( Picture* pic, const PicList& picList );
    void     xDetectScc           ( Picture* pic ) const;
    void     xDisableTempDown     ( Picture* pic, const PicList& picList, const int thresh = INT32_MAX );
    void     xDetectSceneCut      ( Picture* pic, const PicList& picList );
    void     xMakeLowRes          ( Picture* pic );
    double   xPredCost            ( const Picture* cur, const Picture* ref );
    double   xDcIntraCost         ( const Picture* pic );
    bool     xIntraPays           ( const Picture* pic, const Picture* ref );
    const Picture* xFindPoc       ( const PicList& picList, int poc ) const;
};

} // namespace vvenc

//! \}


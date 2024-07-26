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


/** \file     PreProcess.h
    \brief
*/


#pragma once

#include "CommonLib/CommonDef.h"
#include "EncStage.h"
#include "GOPCfg.h"
#include "BitAllocation.h"

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
    bool            m_isHighRes;
    bool            m_doSTA;
    bool            m_doTempDown;
    bool            m_doVisAct;
    bool            m_doVisActQpa;
    bool            m_cappedCQF;

  public:
    PreProcess( MsgLog& _m );
    virtual ~PreProcess();

    void init( const VVEncCfg& encCfg, bool isFinalPass );

    const GOPCfg* getGOPCfg() const { return &m_gopCfg; };

  protected:
    virtual void initPicture    ( Picture* pic );
    virtual void processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList );

  private:
    void     xFreeUnused          ( Picture* pic, const PicList& picList, PicList& doneList, PicList& freeList ) const;
    void     xGetPrevPics         ( const Picture* pic, const PicList& picList, const Picture* prevPics[ NUM_QPA_PREV_FRAMES ] ) const;
    Picture* xGetPrevTL0Pic       ( const Picture* pic, const PicList& picList ) const;
    Picture* xGetStartOfLastGop   ( const PicList& picList ) const;
    void     xLinkPrevQpaBufs     ( Picture* pic, const PicList& picList ) const;
    void     xGetVisualActivity   ( Picture* pic, const PicList& picList ) const;
    void     xGetSpatialActivity  ( Picture* pic, bool doLuma, bool doChroma, VisAct va[ MAX_NUM_CH ] ) const;
    void     xGetTemporalActivity ( Picture* curPic, const Picture* refPic1, const Picture* refPic2, VisAct& va ) const;
    void     xDetectSTA           ( Picture* pic, const PicList& picList );
    void     xDetectScc           ( Picture* pic ) const;
    void     xDisableTempDown     ( Picture* pic, const PicList& picList );
};

} // namespace vvenc

//! \}


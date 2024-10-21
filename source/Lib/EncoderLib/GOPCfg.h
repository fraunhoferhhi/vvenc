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

/** \file     GOPCfg.h
    \brief
*/

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/Slice.h"
#include "Utilities/MsgLog.h"


//! \ingroup EncoderLib
//! \{

namespace vvenc {

class PicShared;

class GOPCfg
{
  typedef std::vector<GOPEntry> GOPEntryList;

  private:
    MsgLog&                      msg;

    std::vector<GOPEntryList>    m_defaultGopLists;
    GOPEntryList                 m_remainGopList;

    GOPEntryList*                m_gopList;
    std::vector<int>             m_pocToGopIdx;

    std::vector<const GOPEntry*> m_defaultRPLList;
    std::vector<int>             m_maxDecPicBuffering;
    std::vector<int>             m_numReorderPics;

    const vvencMCTF*             m_mctfCfg;

    bool m_picReordering;
    int  m_refreshType;
    int  m_fixIntraPeriod;
    bool m_poc0idr;
    int  m_maxGopSize;
    int  m_defGopSize;
    int  m_nextListIdx;
    int  m_gopNum;
    int  m_nextPoc;
    int  m_pocOffset;
    int  m_cnOffset;
    int  m_numTillGop;
    int  m_numTillIntra;
    int  m_maxTid;
    int  m_firstPassMode;
    int  m_defaultNumActive[ 2 ];
    int  m_minIntraDist;
    int  m_lastIntraPOC;
    bool m_adjustNoLPcodingOrder;

  public:
    GOPCfg( MsgLog& _m )
      : msg               ( _m )
      , m_gopList         ( nullptr )
      , m_mctfCfg         ( nullptr )
      , m_picReordering   ( false )
      , m_refreshType     ( 0 )
      , m_fixIntraPeriod  ( 0 )
      , m_poc0idr         ( false )
      , m_maxGopSize      ( 0 )
      , m_defGopSize      ( 0 )
      , m_nextListIdx     ( 0 )
      , m_gopNum          ( 0 )
      , m_nextPoc         ( 0 )
      , m_pocOffset       ( 0 )
      , m_cnOffset        ( 0 )
      , m_numTillGop      ( 0 )
      , m_numTillIntra    ( 0 )
      , m_maxTid          ( 0 )
      , m_firstPassMode   ( 0 )
      , m_defaultNumActive{ 0, 0 }
      , m_minIntraDist    ( -1 )
      , m_lastIntraPOC    ( -1 )
      , m_adjustNoLPcodingOrder( false )
    {
    };

    ~GOPCfg()
    {
    };

    void initGopList( int refreshType, bool poc0idr, int intraPeriod, int gopSize, int leadFrames, bool bPicReordering, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ], const vvencMCTF& mctfCfg, int firstPassMode, int m_minIntraDist );
    void getNextGopEntry( GOPEntry& gopEntry );
    void startIntraPeriod( GOPEntry& gopEntry );
    void fixStartOfLastGop( GOPEntry& gopEntry );
    void getDefaultRPLLists( RPLList& rpl0, RPLList& rpl1 ) const;
    void setLastIntraSTA( int poc ) { m_lastIntraPOC = poc; }

    int  getMaxTLayer() const                             { return m_maxTid; }
    const std::vector<int>& getMaxDecPicBuffering() const { return m_maxDecPicBuffering; }
    const std::vector<int>& getNumReorderPics() const     { return m_numReorderPics; }
    int  getDefaultNumActive( int l ) const               { return m_defaultNumActive[ l ]; }

    bool isSTAallowed( int poc ) const;
    bool hasNonZeroTemporalId() const;
    bool hasLeadingPictures() const;
    bool isChromaDeltaQPEnabled() const;

  private:
    int  xGetMinPoc          ( int maxGopSize, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ] ) const;
    void xCreateGopList      ( int maxGopSize, int gopSize, int pocOffset, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ], const GOPEntryList* prevGopList, GOPEntryList& gopList ) const;
    void xGetPrevGopRefs     ( const GOPEntryList* prevGopList, std::vector< std::pair<int, int> >& prevGopRefs ) const;
    void xPruneGopList       ( int gopSize, GOPEntryList& gopList ) const;
    void xGetRefsOfNextGop   ( int gopSize, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ], int pocOffset, std::vector<int>& pocList ) const;
    void xSetMctfIndex       ( int maxGopSize, GOPEntryList& gopList ) const;
    void xSetSkipFirstPass   ( GOPEntryList& gopList ) const;
    void xCreatePocToGopIdx  ( const GOPEntryList& gopList, bool bShift, std::vector<int>& pocToGopIdx ) const;
    void xSetSTSA            ( GOPEntryList& gopList, const std::vector<int>& pocToGopIdx ) const;
    void xSetBckwdOnly       ( GOPEntryList& gopList ) const;
    void xSetVTL             ( GOPEntryList& gopList ) const;
    void xSetDefaultRPL      ( std::vector<GOPEntryList>& defaultLists );
    void xSetDBPConstraints  ( std::vector<GOPEntryList>& defaultLists );
    bool xCheckDBPConstraints( const GOPEntryList& gopList ) const;
    void xAddRefPicsBckwd    ( std::vector<int>& deltaList, const GOPEntry* gopEntry, const std::vector<GOPEntry*>& availList ) const;
    void xAddRefPicsFwd      ( std::vector<int>& deltaList, const GOPEntry* gopEntry, const std::vector<GOPEntry*>& availList ) const;
    void xAddRefPicsPrevGOP  ( std::vector<int>& deltaList, const GOPEntry* gopEntry, const std::vector< std::pair<int, int> >& prevGopRefs ) const;
    int  xGetMaxTid          ( const GOPEntryList& gopList ) const;
    int  xGetMaxRefPics      ( const GOPEntry& gopEntry ) const;
    int  xGetMaxNumReorder   ( const GOPEntry& gopEntry, const GOPEntryList& gopList ) const;
    void xAdjustNoLPcodingOrder( GOPEntry& gopEntry, const int orgGopId ) const;
};

} // namespace vvenc

//! \}


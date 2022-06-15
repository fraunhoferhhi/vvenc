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


/** \file     GOPCfg.cpp
    \brief    
*/

#include "GOPCfg.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GOPCfg::initGopList( int refreshType, int intraPeriod, int gopSize, bool bLowDelay, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ], const vvencMCTF& mctfCfg )
{
  CHECK( gopSize < 1, "gop size has to be greater than 0" );

  m_mctfCfg            = &mctfCfg;
  m_refreshType        = refreshType;
  m_fixIntraPeriod     = intraPeriod;
  m_maxGopSize         = gopSize;
  m_defGopSize         = m_fixIntraPeriod > 0 ? std::min( m_maxGopSize, m_fixIntraPeriod ) : m_maxGopSize;

  const int numGops    = m_fixIntraPeriod > 0 ? m_fixIntraPeriod / m_defGopSize               : 0;
  const int remainSize = m_fixIntraPeriod > 0 ? m_fixIntraPeriod - ( m_defGopSize * numGops ) : 0;
  const int minPrevPoc = xGetMinPoc( m_maxGopSize, cfgGopList );
  const int numDefault = ( ( -1 * minPrevPoc ) + m_maxGopSize - 1 ) / m_maxGopSize + 1;

  // setup default gop lists
  GOPEntryList* prevGopList = nullptr;
  int pocOffset             = 0;
  m_defaultGopLists.resize( numDefault );
  for( int i = 0; i < numDefault; i++ )
  {
    xCreateGopList( m_maxGopSize, m_defGopSize, pocOffset, cfgGopList, prevGopList, m_defaultGopLists[ i ] );
    prevGopList = &m_defaultGopLists[ i ];
    pocOffset += m_defGopSize;
  }
  if( remainSize && remainSize != m_defGopSize )
  {
    prevGopList = numGops < (int)m_defaultGopLists.size() ? &m_defaultGopLists[ numGops ] : &m_defaultGopLists.back();
    pocOffset   = numGops * m_defGopSize;
    xCreateGopList( m_maxGopSize, remainSize, pocOffset, cfgGopList, prevGopList, m_remainGopList );
  }

  // set some defaults based on gop list layouts
  xSetDefaultRPL    ( m_defaultGopLists );
  xSetDBPConstraints( m_defaultGopLists );
  // sanity check
  for( int i = 0; i < (int)m_defaultGopLists.size() + 1; i++ )
  {
    GOPEntryList& gopList = i < (int)m_defaultGopLists.size() ? m_defaultGopLists[ i ] : m_remainGopList;
    if( ! xCheckDBPConstraints( gopList ) )
    {
      THROW( "gop lists do not fullfill default DPB constraints" );
    }
  }

  // start with first gop list
  m_gopList     = &m_defaultGopLists[ 0 ];
  m_nextListIdx = std::min( 1, (int)m_defaultGopLists.size() - 1 );
  xCreatePocToGopIdx( *m_gopList, m_refreshType == VVENC_DRT_IDR2, m_pocToGopIdx );

  // lets start with poc 0
  m_gopNum       = 0;
  m_nextPoc      = 0;
  m_pocOffset    = 0;
  m_cnOffset     = 0;
  CHECK( m_refreshType == VVENC_DRT_IDR2 && ( m_fixIntraPeriod == 1 || bLowDelay ), "refresh type idr2 only for random access possible" );
  m_numTillGop   = m_refreshType == VVENC_DRT_IDR2 ? (int)m_gopList->size() - 1 : 0;
  m_numTillIntra = m_refreshType == VVENC_DRT_IDR2 ? (int)m_gopList->size() - 1 : 0;

  // TODO (jb): cleanup
#if 0
  for( int i = 0; i < (int)m_defaultGopLists.size() + 1; i++ )
  {
    GOPEntryList& gopList = i < (int)m_defaultGopLists.size() ? m_defaultGopLists[ i ] : m_remainGopList;
    std::cout << "GOPList " << i << ", def = " << ( i < (int)m_defaultGopLists.size() ) << ", size = " << gopList.size() << std::endl;
    for( int j = 0; j < (int)gopList.size(); j++ )
    {
      const auto& gopEntry = gopList[ j ];
      std::cout << "    Frame" << ( j + 1 )
                << " POC= " << gopEntry.m_POC
                << " TID= " << gopEntry.m_temporalId
                << " MCTF= " << gopEntry.m_mctfIndex
                << " qpOffset= " << gopEntry.m_QPOffset
                << " modelOffset= " << gopEntry.m_QPOffsetModelOffset
                << " modelScale= " << gopEntry.m_QPOffsetModelScale
                << " NumL0= " << gopEntry.m_numRefPics[ 0 ]
                << " L0: " << gopEntry.m_deltaRefPics[ 0 ][ 0 ]
                << " "     << gopEntry.m_deltaRefPics[ 0 ][ 1 ]
                << " "     << gopEntry.m_deltaRefPics[ 0 ][ 2 ]
                << " "     << gopEntry.m_deltaRefPics[ 0 ][ 3 ]
                << " "     << gopEntry.m_deltaRefPics[ 0 ][ 4 ]
                << " "     << gopEntry.m_deltaRefPics[ 0 ][ 5 ]
                << " NumL1= " << gopEntry.m_numRefPics[ 1 ]
                << " L1: " << gopEntry.m_deltaRefPics[ 1 ][ 0 ]
                << " "     << gopEntry.m_deltaRefPics[ 1 ][ 1 ]
                << " "     << gopEntry.m_deltaRefPics[ 1 ][ 2 ]
                << " "     << gopEntry.m_deltaRefPics[ 1 ][ 3 ]
                << " "     << gopEntry.m_deltaRefPics[ 1 ][ 4 ]
                << " "     << gopEntry.m_deltaRefPics[ 1 ][ 5 ]
                << std::endl;
    }
  }
#endif
}

void GOPCfg::getNextGopEntry( GOPEntry& gopEntry )
{
  const int  pocInGop   = m_nextPoc - m_pocOffset;
  const int  gopId      = m_pocToGopIdx[ pocInGop % (int)m_pocToGopIdx.size() ];
  const bool isLeading0 = m_refreshType != VVENC_DRT_IDR2 && m_nextPoc == 0 ? true : false; // for non IDR2, we have a leading poc 0

  gopEntry             = (*m_gopList)[ gopId ];
  gopEntry.m_POC       = m_nextPoc;
  gopEntry.m_codingNum = isLeading0 ? 0 : m_cnOffset + gopId;
  gopEntry.m_gopNum    = m_gopNum;

  // mark start of intra
  if( m_numTillIntra == 0 )
  {
    gopEntry.m_sliceType      = 'I';
    gopEntry.m_isStartOfIntra = true;
  }

  // check for end of current gop
  CHECK( m_numTillIntra == 0 && m_numTillGop != 0, "start of new intra period only at start of new gop expected" );
  if( m_numTillGop == 0 )
  {
    const int prevGopSize = (int)m_gopList->size();
    // check for start of new intra period
    if( m_numTillIntra == 0 )
    {
      m_gopList      = &m_defaultGopLists[ 0 ];
      m_nextListIdx  = std::min( 1, (int)m_defaultGopLists.size() - 1 );
      m_numTillIntra = m_fixIntraPeriod;
    }
    else
    {
      const int remainSize = m_numTillIntra > 0 ? std::min( m_defGopSize, m_numTillIntra ) : m_defGopSize;
      if( remainSize == (int)m_defaultGopLists[ m_nextListIdx ].size() && prevGopSize == m_defGopSize )
      {
        m_gopList     = &m_defaultGopLists[ m_nextListIdx ];
        m_nextListIdx = std::min( m_nextListIdx + 1, (int)m_defaultGopLists.size() - 1 );
      }
      else
      {
        CHECK( remainSize != (int)m_remainGopList.size() || prevGopSize != m_defGopSize, "remaining size does not match size of pre-calculated gop list" );
        m_gopList = &m_remainGopList;
      }
    }
    xCreatePocToGopIdx( *m_gopList, m_refreshType == VVENC_DRT_IDR2, m_pocToGopIdx );
    m_numTillGop  = (int)m_gopList->size();
    m_cnOffset   += isLeading0 ? 1 : prevGopSize;
    if( ! isLeading0 )
    {
      m_pocOffset += prevGopSize;
      m_gopNum    += 1;
    }
  }

  // continue with next pic
  m_nextPoc      += 1;
  m_numTillGop   -= 1;
  if( m_numTillIntra > 0 )
  {
    m_numTillIntra -= 1;
  }

  // TODO (jb): cleanup
#if 0
  std::cout << "         (poc= " << gopEntry.m_POC
            << " gopNum= " << gopEntry.m_gopNum
            << " codingNum= " << gopEntry.m_codingNum
            << " TID= " << gopEntry.m_temporalId
            << " MCTF= " << gopEntry.m_mctfIndex
            << " qpOffset= " << gopEntry.m_QPOffset
            << " modelOffset= " << gopEntry.m_QPOffsetModelOffset
            << " modelScale= " << gopEntry.m_QPOffsetModelScale
            << " NumL0= " << gopEntry.m_numRefPics[ 0 ]
            << " L0= " << gopEntry.m_deltaRefPics[ 0 ][ 0 ]
            << " " << gopEntry.m_deltaRefPics[ 0 ][ 1 ]
            << " " << gopEntry.m_deltaRefPics[ 0 ][ 2 ]
            << " " << gopEntry.m_deltaRefPics[ 0 ][ 3 ]
            << " " << gopEntry.m_deltaRefPics[ 0 ][ 4 ]
            << " " << gopEntry.m_deltaRefPics[ 0 ][ 5 ]
            << " NumL1= " << gopEntry.m_numRefPics[ 1 ]
            << " L1= " << gopEntry.m_deltaRefPics[ 1 ][ 0 ]
            << " " << gopEntry.m_deltaRefPics[ 1 ][ 1 ]
            << " " << gopEntry.m_deltaRefPics[ 1 ][ 2 ]
            << " " << gopEntry.m_deltaRefPics[ 1 ][ 3 ]
            << " " << gopEntry.m_deltaRefPics[ 1 ][ 4 ]
            << " " << gopEntry.m_deltaRefPics[ 1 ][ 5 ]
            << ")" << std::endl;
#endif
}

void GOPCfg::getDefaultRPLLists( RPLList& rpl0, RPLList& rpl1 ) const
{
  const int numRpl = (int)m_defaultRPLList.size();
  // TODO (jb): check size + 1, fix getNumRPL() as well
  rpl0.resize( numRpl + 1 );
  rpl1.resize( numRpl + 1 );
  for( int i = 0; i < numRpl; i++ )
  {
    rpl0[ i ].initFromGopEntry( *m_defaultRPLList[ i ], 0 );
    rpl1[ i ].initFromGopEntry( *m_defaultRPLList[ i ], 1 );
  }
}

bool GOPCfg::hasNonZeroTemporalId() const
{
  return m_maxTid > 0;
}

bool GOPCfg::hasLeadingPictures() const
{
  for( const auto& gopEntry : m_defaultGopLists.back() )
  {
    if( ! gopEntry.m_useBckwdOnly )
    {
      return true;
    }
  }
  return false;
}

bool GOPCfg::isChromaDeltaQPEnabled() const
{
  for( const auto& gopEntry : m_defaultGopLists.back() )
  {
    if( gopEntry.m_CbQPoffset || gopEntry.m_CrQPoffset )
    {
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int GOPCfg::xGetMinPoc( int maxGopSize, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ] ) const
{
  int minPoc = 0;
  for( int i = 0; i < maxGopSize; i++ )
  {
    for( int l = 0; l < 2; l++ )
    {
      for( int j = 0; j < cfgGopList[ i ].m_numRefPics[ l ]; j++ )
      {
        const int refPoc = cfgGopList[ i ].m_POC - cfgGopList[ i ].m_deltaRefPics[ l ][ j ];
        if( refPoc < minPoc )
          minPoc = refPoc;
      }
    }
  }
  return minPoc;
}

void GOPCfg::xCreateGopList( int maxGopSize, int gopSize, int pocOffset, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ], const GOPEntryList* prevGopList, GOPEntryList& gopList ) const
{
  const bool bSkipPrev = prevGopList == nullptr || (int)prevGopList->size() < maxGopSize;
  std::vector< std::pair<int, int> > prevGopRefs;
  if( ! bSkipPrev )
  {
    xGetPrevGopRefs( prevGopList, prevGopRefs );
  }

  //
  // copy given gop configuration
  //

  gopList.clear();
  gopList.resize( maxGopSize );
  for( int i = 0; i < maxGopSize; i++ )
  {
    gopList[ i ].setDefaultGOPEntry();
    gopList[ i ].copyFromGopCfg( cfgGopList[ i ] );
  }

  // find max temporal id
  const int maxTid = xGetMaxTid( gopList );

  // prune gop list
  CHECK( gopSize > maxGopSize, "only pruning of gop list supported" );
  if( gopSize < maxGopSize )
  {
    xPruneGopList( gopSize, gopList );
  }

  // 
  // fix prediction pattern
  //

  // use list of available entries for prediction
  std::vector<GOPEntry*> availList;
  availList.resize( gopSize + 1, nullptr );
  GOPEntry e1;
  if( prevGopRefs.size() == 0 || prevGopRefs[ 0 ].first == 0 )
  {
    // insert leading poc 0 to be available
    CHECK( prevGopRefs.size() && prevGopRefs[ 0 ].first == 0 && prevGopRefs[ 0 ].second != 0, "gop structure should start and end with temporalId 0" );
    e1.setDefaultGOPEntry();
    e1.m_POC = 0;
    availList[ 0 ] = &e1;
    // remove leading poc 0 from prev 
    if( prevGopRefs.size() && prevGopRefs[ 0 ].first == 0 )
    {
      for( int i = 0; i < (int)prevGopRefs.size() - 1; i++ )
      {
        prevGopRefs[ i ] = prevGopRefs[ i + 1 ];
      }
      prevGopRefs.pop_back();
    }
  }

  // loop over gop list
  std::vector<int> newFwd;
  std::vector<int> newBckwd;
  std::vector<int> availFwd;
  std::vector<int> availBckwd;
  newFwd.reserve( MAX_REF_PICS );
  newBckwd.reserve( MAX_REF_PICS );
  availFwd.reserve( MAX_REF_PICS );
  availBckwd.reserve( MAX_REF_PICS );
  for( int i = 0; i < gopList.size(); i++ )
  {
    GOPEntry* gopEntry = &gopList[ i ];

    // loop over both reference lists
    for( int l = 0; l < 2; l++ )
    {
      newFwd.clear();
      newBckwd.clear();
      availFwd.clear();
      availBckwd.clear();

      // fill list of possible reference pictures in forward / backward direction
      xAddRefPicsFwd( availFwd, gopEntry, availList );
      xAddRefPicsBckwd( availBckwd, gopEntry, availList );
      // access to previous gop not used/allowed for non-ref pics at highest temporal layer
      if( gopEntry->m_temporalId <= 1 || gopEntry->m_temporalId < maxTid )
      {
        xAddRefPicsPrevGOP( availBckwd, gopEntry, prevGopRefs );
      }

      // check / fix all active references
      for( int j = 0; j < gopEntry->m_numRefPicsActive[ l ]; j++ )
      {
        const int delta = gopEntry->m_deltaRefPics[ l ][ j ];
        CHECK( delta == 0, "error in gop configuration: try to reference own picture" );
        // start with forward or backward prediction
        std::vector<int>& sameDir   = delta < 0 ? availFwd   : availBckwd;
        std::vector<int>& invertDir = delta < 0 ? availBckwd : availFwd;
        // check if candidates are available in same direction list
        if( sameDir.size() )
        {
          // try to find correct match
          auto itr = find( sameDir.begin(), sameDir.end(), delta );
          if( itr == sameDir.end() )
          {
            // no match found, choose replacement
            itr = sameDir.begin();
          }
          // use delta candidate and remove candidate to prevent doublets
          std::vector<int>& newDst = *itr < 0 ? newFwd : newBckwd;
          newDst.push_back( *itr );
          sameDir.erase( itr );
        }
        else if( invertDir.size() )
        {
          // corret match not possible, we are inverting the prediction direction
          auto itr = invertDir.begin();
          // use delta candidate and remove candidate to prevent doublets
          std::vector<int>& newDst = *itr < 0 ? newFwd : newBckwd;
          newDst.push_back( *itr );
          invertDir.erase( itr );
        }
      }

      // clear old delta list
      std::memset( gopEntry->m_deltaRefPics[ l ], 0, sizeof( gopEntry->m_deltaRefPics[ l ] ) );

      // ensure new delta lists are sorted
      std::sort( newFwd.begin(), newFwd.end(),     []( auto& a, auto& b ){ return a > b; } );
      std::sort( newBckwd.begin(), newBckwd.end(), []( auto& a, auto& b ){ return a < b; } );

      // copy new delta poc list
      CHECK( (int)newFwd.size() + (int)newBckwd.size() >= VVENC_MAX_NUM_REF_PICS, "number of delta ref pic entries exceeds array bounds" );
      gopEntry->m_numRefPics[ l ]       = (int)newFwd.size() + (int)newBckwd.size();
      gopEntry->m_numRefPicsActive[ l ] = (int)newFwd.size() + (int)newBckwd.size();
      std::vector<int>& src1 = l == 0 ? newBckwd : newFwd;
      std::vector<int>& src2 = l == 0 ? newFwd   : newBckwd;
      int dstIdx = 0;
      for( int j = 0; j < (int)src1.size(); j++ )
      {
        gopEntry->m_deltaRefPics[ l ][ dstIdx ] = src1[ j ];
        dstIdx += 1;
      }
      for( int j = 0; j < (int)src2.size(); j++ )
      {
        gopEntry->m_deltaRefPics[ l ][ dstIdx ] = src2[ j ];
        dstIdx += 1;
      }
    }

    // available for later use as ref
    availList[ gopEntry->m_POC ] = gopEntry;
  }

  //
  // add entries to ref lists, which are required as reference for later pictures in coding order
  //

  // create fake entry as start of potential next gop, 
  // to ensure pictures referenced from next gop are keept alive
  GOPEntry e2;
  e2.setDefaultGOPEntry();
  e2.m_POC           = 2 * maxGopSize;
  e2.m_temporalId    = 0;
  const bool bIsLast = m_maxGopSize != gopSize;
  if( ! bIsLast )
  {
    std::vector<int> pocList;
    pocList.reserve( gopSize );
    xGetRefsOfNextGop( gopSize, cfgGopList, pocOffset, pocList );
    e2.m_numRefPics[ 0 ] = (int)pocList.size();
    for( int i = 0; i < (int)pocList.size(); i++ )
    {
      const int reqPoc = pocList[ i ] + maxGopSize; // shift poc from next to current gop
      e2.m_deltaRefPics[ 0 ][ i ] = e2.m_POC - reqPoc;
    }
  }
  const GOPEntry* prevEntry = &e2;

  // traverse gop list from back to front
  for( int i = gopSize - 1; i >= 0; i-- )
  {
    GOPEntry* gopEntry = &gopList[ i ];
    // loop over all ref lists
    for( int j = 0; j < 2; j++ )
    {
      // loop over all delta POCs in previous entry
      for( int k = 0; k < prevEntry->m_numRefPics[ j ]; k++ )
      {
        const int reqPoc = prevEntry->m_POC - prevEntry->m_deltaRefPics[ j ][ k ];
        if( reqPoc == gopEntry->m_POC )
        {
          continue;
        }
        bool bAvail = false;
        // search if required poc is also part of the ref lists of current gop entry
        for( int l = 0; l < 2; l++ )
        {
          for( int m = 0; m < gopEntry->m_numRefPics[ l ]; m++ )
          {
            const int availPoc = gopEntry->m_POC - gopEntry->m_deltaRefPics[ l ][ m ];
            if( reqPoc == availPoc )
            {
              bAvail = true;
              break;
            }
          }
          if( bAvail )
          {
            break;
          }
        }
        // if required poc not found, add this poc to current entry lists
        if( ! bAvail )
        {
          const int delta   = gopEntry->m_POC - reqPoc;
          const int addList = delta > 0 ? 0 : 1; // add to backward L0 or forward L1 list
          CHECK( gopEntry->m_numRefPics[ addList ] >= VVENC_MAX_NUM_REF_PICS, "out of array bounds" );
          gopEntry->m_deltaRefPics[ addList ][ gopEntry->m_numRefPics[ addList ] ] = delta;
          gopEntry->m_numRefPics[ addList ] += 1;
          // small optimization: sort additional poc by value to decrease required bits for encoding the list
          const int sign = delta > 0 ? 1 : -1;
          for( int n = gopEntry->m_numRefPics[ addList ] - 1; n > gopEntry->m_numRefPicsActive[ addList ]; n-- )
          {
            if( gopEntry->m_deltaRefPics[ addList ][ n ] * sign > gopEntry->m_deltaRefPics[ addList ][ n - 1 ] * sign )
              break;
            std::swap( gopEntry->m_deltaRefPics[ addList ][ n ], gopEntry->m_deltaRefPics[ addList ][ n - 1 ] );
          }
        }
      }
    }
    prevEntry = gopEntry;
  }

  // poc to gop map for fastwr access
  std::vector<int> pocToGopIdx;
  xCreatePocToGopIdx( gopList, false, pocToGopIdx );

  // STSA, forward B flags, MCTF index
  xSetSTSA     ( gopList, pocToGopIdx );
  xSetBckwdOnly( gopList );
  xSetMctfIndex( maxGopSize, gopList );

  // mark first gop entry
  gopList[ 0 ].m_isStartOfGop = true;
}

void GOPCfg::xGetPrevGopRefs( const GOPEntryList* prevGopList, std::vector< std::pair<int, int> >& prevGopRefs ) const
{
  if( ! prevGopList )
  {
    return;
  }

  prevGopRefs.reserve( MAX_REF_PICS );

  const int gopSize         = (int)prevGopList->size();
  const GOPEntry& lastEntry = prevGopList->back();

  // last encoded picture available for reference
  prevGopRefs.push_back( std::make_pair( lastEntry.m_POC, lastEntry.m_temporalId ) );

  // insert all available reference pictures
  for( int l = 0; l < 2; l++ )
  {
    for( int i = 0; i < lastEntry.m_numRefPics[ l ]; i++ )
    {
      // check referenced poc already found
      const int refPoc = lastEntry.m_POC - lastEntry.m_deltaRefPics[ l ][ i ];
      bool bFound = false;
      for( int j = 0; j < (int)prevGopRefs.size(); j++ )
      {
        if( prevGopRefs[ j ].first == refPoc )
        {
          bFound = true;
          break;
        }
      }
      if( bFound )
      {
        continue;
      }
      // find temporal layer of referenced poc
      int refTid = -1;
      int cmpPoc = refPoc;
      if( cmpPoc < 0 )
      {
        // ensure value positive
        const int numGops = -1 * cmpPoc / gopSize + 1;
        cmpPoc += numGops * gopSize;
      }
      cmpPoc = cmpPoc % gopSize;
      for( int j = 0; j < gopSize; j++ )
      {
        if( ( (*prevGopList)[ j ].m_POC % gopSize ) == cmpPoc )
        {
          refTid = (*prevGopList)[ j ].m_temporalId;
          break;
        }
      }
      CHECK( refTid < 0, "error in gop configuration: gop entry not found or temporalId negative" );
      // store referenced poc
      prevGopRefs.push_back( std::make_pair( refPoc, refTid ) );
    }
  }

  // correct poc of referenced pictures by previous gop size
  for( int i = 0; i < (int)prevGopRefs.size(); i++ )
  {
    prevGopRefs[ i ].first -= gopSize;
    CHECK( prevGopRefs[ i ].first > 0, "error in gop configuration: reference picture points to future gop" );
  }

  std::sort( prevGopRefs.begin(), prevGopRefs.end(), []( auto& a, auto& b ){ return a.first > b.first; } );
}

void GOPCfg::xPruneGopList( int gopSize, GOPEntryList& gopList ) const
{
  const int oldSize = (int)gopList.size();
  CHECK( oldSize <= gopSize, "gop list to short for prunning" );

  std::vector<GOPEntry> prunedList;
  prunedList.reserve( gopSize );
  for( int i = 0; i < (int)gopList.size(); i++ )
  {
    if( gopList[ i ].m_POC == oldSize )
    {
      // always include end of old gop, which is the start of following gop
      prunedList.push_back( gopList[ i ] );
      // fix poc and delta lists of new end of pruned gop
      GOPEntry& gopEntry = prunedList.back();
      const int sizeDiff = oldSize - gopSize;
      for( int l = 0; l < 2; l++ )
      {
        for( int j = 0; j < gopEntry.m_numRefPics[ l ]; j++ )
        {
          gopEntry.m_deltaRefPics[ l ][ j ] = std::min( gopEntry.m_deltaRefPics[ l ][ j ] - sizeDiff, gopSize );
        }
      }
      gopEntry.m_POC -= sizeDiff;
    }
    else if( gopList[ i ].m_POC < gopSize )
    {
      prunedList.push_back( gopList[ i ] );
    }
  }

  gopList.clear();
  gopList.resize( (int)prunedList.size() );
  for( int i = 0; i < (int)prunedList.size(); i++ )
  {
    gopList[ i ] = prunedList[ i ];
  }

  CHECK( gopList.size() != gopSize, "pruned gop list incomplete" );
}

void GOPCfg::xGetRefsOfNextGop( int gopSize, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ], int pocOffset, std::vector<int>& pocList ) const
{
  const int minPoc = -1 * ( pocOffset + gopSize );
  for( int i = 0; i < gopSize; i++ )
  {
    for( int l = 0; l < 2; l++ )
    {
      for( int j = 0; j < cfgGopList[ i ].m_numRefPics[ l ]; j++ )
      {
        const int refPoc = cfgGopList[ i ].m_POC - cfgGopList[ i ].m_deltaRefPics[ l ][ j ];
        if( refPoc >= minPoc && refPoc < 0 )
        {
          auto itr = find( pocList.begin(), pocList.end(), refPoc );
          if( itr == pocList.end() )
            pocList.push_back( refPoc );
        }
      }
    }
  }
  std::sort( pocList.begin(), pocList.end() );
}

void GOPCfg::xSetMctfIndex( int maxGopSize, GOPEntryList& gopList ) const
{
  for( auto& gopEntry : gopList )
  {
    int poc = gopEntry.m_POC;
    // for pruned gops max poc is start / end of gop and should be filtered, but the used poc has been changed 
    // to pruned gop size and does not fit the original config anymore => set back original max gop size poc
    if( (int)gopList.size() < maxGopSize && poc == (int)gopList.size() )
    {
      poc = maxGopSize;
    }
    // find mctf index
    for( int i = m_mctfCfg->numFrames - 1; i >= 0; i-- )
    {
      if( ( poc % m_mctfCfg->MCTFFrames[ i ] ) == 0 )
      {
        gopEntry.m_mctfIndex = i;
        break;
      }
    }
  }
}

void GOPCfg::xCreatePocToGopIdx( const GOPEntryList& gopList, bool bShift, std::vector<int>& pocToGopIdx ) const
{
  const int gopSize   = (int)gopList.size();
  const int pocOffset = bShift ? -1 : 0;

  pocToGopIdx.clear();
  pocToGopIdx.resize( gopSize, -1 );

  for( int i = 0; i < gopSize; i++ )
  {
    const GOPEntry& gopEntry = gopList[ i ];
    const int poc            = ( gopEntry.m_POC + pocOffset ) % gopSize;
    CHECK( gopEntry.m_POC > gopSize || gopEntry.m_POC < 1, "error: poc out of range" );
    CHECK( pocToGopIdx[ poc ] != -1,                       "error: multiple entries in gop list map to same poc" );
    pocToGopIdx[ poc ] = i;
  }
  for( int i = 0; i < gopSize; i++ )
  {
    CHECK( pocToGopIdx[ i ] < 0, "error: poc not found in gop list" );
  }
}

void GOPCfg::xSetSTSA( GOPEntryList& gopList, const std::vector<int>& pocToGopIdx ) const
{
  // check STSA condition satisfied for each GOPEntry
  std::vector<bool> isLocalSTSA;
  isLocalSTSA.resize( gopList.size(), true );
  for( int i = 0; i < (int)gopList.size(); i++ )
  {
    const GOPEntry& gopEntry = gopList[ i ];
    // check all ref pics have higher temporal level
    for( int l = 0; l < 2 && isLocalSTSA[ i ]; l++ )
    {
      for( int j = 0; j < gopEntry.m_numRefPics[ l ]; j ++ )
      {
        const int refPoc = gopEntry.m_POC - gopEntry.m_deltaRefPics[ l ][ j ];
        if( refPoc < 0 )
        {
          continue;
        }
        const int gopId  = pocToGopIdx[ refPoc % (int)pocToGopIdx.size() ];
        const GOPEntry& refEntry = gopList[ gopId ];
        if( refEntry.m_temporalId >= gopEntry.m_temporalId )
        {
          isLocalSTSA[ i ] = false;
          break;
        }
      }
    }
  }

  // accumulate STSA condition for all pics of same temporal level
  const int maxTid = xGetMaxTid( gopList );
  std::vector<bool> isTlSTSA;
  isTlSTSA.resize( maxTid + 1, true );
  for( int i = 0; i < (int)gopList.size(); i++ )
  {
    const GOPEntry& gopEntry = gopList[ i ];
    const int tid            = gopEntry.m_temporalId;
    const bool isSTSA        = isTlSTSA[ tid ] && isLocalSTSA[ i ];
    isTlSTSA[ tid ]          = isSTSA;
  }

  // set STSA flag for each GOPEntry
  for( auto& gopEntry : gopList )
  {
    const int tid = gopEntry.m_temporalId;
    gopEntry.m_isSTSA = isTlSTSA[ tid ];
  }
}

void GOPCfg::xSetBckwdOnly( GOPEntryList& gopList ) const
{
  for( auto& gopEntry : gopList )
  {
    bool useBckwdOnly = true;
    for( int l = 0; l < 2 && useBckwdOnly; l++ )
    {
      for( int j = 0; j < gopEntry.m_numRefPics[ l ]; j ++ )
      {
        if( gopEntry.m_deltaRefPics[ l ][ j ] < 0 )
        {
          useBckwdOnly = false;
          break;
        }
      }
    }
    gopEntry.m_useBckwdOnly = useBckwdOnly;
  }
}

void GOPCfg::xSetDefaultRPL( std::vector<GOPEntryList>& defaultLists )
{
  m_defaultRPLList.clear();

  // default RPL index is entry position in gop
  GOPEntryList& gopList = defaultLists.back();
  for( auto& gopEntry : gopList )
  {
    gopEntry.m_defaultRPLIdx = (int)m_defaultRPLList.size();
    m_defaultRPLList.push_back( &gopEntry );
  }

  // in prev lists set extra index if RPL list deviates from default list
  for( int i = 0; i < (int)defaultLists.size() - 1; i++ )
  {
    GOPEntryList& prevList = defaultLists[ i ];
    CHECK( gopList.size() != prevList.size(), "default gop list and first gop list have to have same size" );
    for( int i = 0; i < (int)prevList.size(); i++ )
    {
      CHECK( prevList[ i ].m_POC != gopList[ i ].m_POC, "default gop list and first gop List have to have same poc layout" );
      GOPEntry& gopEntry = prevList[ i ];
      GOPEntry& cmpEntry = gopList[ i ];
      bool haveSameList  = true;
      for( int l = 0; l < 2 && haveSameList; l++ )
      {
        if( gopEntry.m_numRefPics[ l ] != cmpEntry.m_numRefPics[ l ] )
        {
          haveSameList = false;
          break;
        }
        for( int j = 0; j < gopEntry.m_numRefPics[ l ]; j ++ )
        {
          if( gopEntry.m_deltaRefPics[ l ][ j ] != cmpEntry.m_deltaRefPics[ l ][ j ] )
          {
            haveSameList = false;
            break;
          }
        }
      }
      if( haveSameList )
      {
        gopEntry.m_defaultRPLIdx = cmpEntry.m_defaultRPLIdx;
      }
      else
      {
        gopEntry.m_defaultRPLIdx = (int)m_defaultRPLList.size();
        m_defaultRPLList.push_back( &gopEntry );
      }
    }
  }

  std::vector<int> hist[ 2 ];
  hist[ 0 ].resize( MAX_NUM_REF, 0 );
  hist[ 1 ].resize( MAX_NUM_REF, 0 );
  for( auto gopEntry : m_defaultRPLList )
  {
    for( int l = 0; l < 2; l++ )
    {
      CHECK( gopEntry->m_numRefPicsActive[ l ] < 0 || gopEntry->m_numRefPicsActive[ l ] >= MAX_NUM_REF, "array index out of bounds");
      hist[ l ][ gopEntry->m_numRefPicsActive[ l ] ] += 1;
    }
  }
  for( int l = 0; l < 2; l++ )
  {
    int bestIdx = 0;
    int maxVal  = -1;
    for( int j = 0; j < (int)hist[ l ].size(); j++ )
    {
      if( hist[ l ][ j ] > maxVal )
      {
        bestIdx = j;
        maxVal  = hist[ l ][ j ];
      }
    }
    m_defaultNumActive[ l ] = bestIdx;
  }
}

void GOPCfg::xSetDBPConstraints( std::vector<GOPEntryList>& defaultLists )
{
  const GOPEntryList& gopList = defaultLists.back();

  m_maxTid = xGetMaxTid( gopList );

  m_maxDecPicBuffering.resize( m_maxTid + 1, 1 );
  m_numReorderPics.resize    ( m_maxTid + 1, 0 );

  // max DPB size and number reorder pics per temporal level
  for( const auto& gopEntry : gopList )
  {
    const int numRefPics = xGetMaxRefPics   ( gopEntry );
    const int numReorder = xGetMaxNumReorder( gopEntry, gopList );
    const int tid        = gopEntry.m_temporalId;
    m_maxDecPicBuffering[ tid ] = std::max( m_maxDecPicBuffering[ tid ], numRefPics + 1 );
    m_numReorderPics    [ tid ] = std::max( m_numReorderPics[ tid ], numReorder );
  }

  // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] - 1, inclusive
  m_maxDecPicBuffering[ 0 ] = std::max( m_maxDecPicBuffering[ 0 ], m_numReorderPics[ 0 ] +1 );
  for( int tid = 1; tid < m_maxTid; tid++ )
  {
    // a lower layer can not have higher reorder value than a higher layer
    m_numReorderPics[ tid ] = std::max( m_numReorderPics[ tid ], m_numReorderPics[ tid - 1 ] );
    // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] - 1, inclusive
    m_maxDecPicBuffering[ tid ] = std::max( m_maxDecPicBuffering[ tid ], m_numReorderPics[ tid ] +1 );
    // a lower layer can not have higher DPB size value than a higher layer
    m_maxDecPicBuffering[ tid ] = std::max( m_maxDecPicBuffering[ tid ], m_maxDecPicBuffering[ tid - 1 ] );
  }
}

bool GOPCfg::xCheckDBPConstraints( const GOPEntryList& gopList ) const
{
  if( gopList.size() && gopList[ 0 ].m_temporalId != 0 )
  {
    msg.log( VVENC_ERROR, "first entry in gop list must have temporal id 0" );
    return false;
  }
  const int maxTid = xGetMaxTid( gopList );
  if( maxTid > m_maxTid )
  {
    msg.log( VVENC_ERROR, "max temporal level exceeds overall maximum value" );
    return false;
  }
  for( const auto& gopEntry : gopList )
  {
    const int numRefPics = xGetMaxRefPics   ( gopEntry );
    const int numReorder = xGetMaxNumReorder( gopEntry, gopList );
    const int tid        = gopEntry.m_temporalId;
    if( numRefPics + 1 > m_maxDecPicBuffering[ tid ] )
    {
      msg.log( VVENC_ERROR, "max DPB size exceeds overall maximum value" );
      return false;
    }
    if( numReorder > m_numReorderPics[ tid ] )
    {
      msg.log( VVENC_ERROR, "max number reorder pics exceeds overall maximum value" );
      return false;
    }
  }
  return true;
}

void GOPCfg::xAddRefPicsBckwd( std::vector<int>& deltaList, const GOPEntry* gopEntry, const std::vector<GOPEntry*>& availList ) const
{
  for( int i = ( gopEntry->m_POC - 1 ); i >= 0; i-- )
  {
    const GOPEntry* refEntry = availList[ i ];
    if( refEntry )
    {
      if( refEntry->m_temporalId <= gopEntry->m_temporalId )
      {
        deltaList.push_back( gopEntry->m_POC - refEntry->m_POC );
        CHECK( deltaList.back() <= 0, "error in backward list: try to access future ref" );
      }
    }
  }
}

void GOPCfg::xAddRefPicsFwd( std::vector<int>& deltaList, const GOPEntry* gopEntry, const std::vector<GOPEntry*>& availList ) const
{
  for( int i = ( gopEntry->m_POC + 1 ); i < (int)availList.size(); i++ )
  {
    const GOPEntry* refEntry = availList[ i ];
    if( refEntry )
    {
      if( refEntry->m_temporalId <= gopEntry->m_temporalId )
      {
        deltaList.push_back( gopEntry->m_POC - refEntry->m_POC );
        CHECK( deltaList.back() >= 0, "error in forward list: try to access past ref" );
      }
    }
  }
}

void GOPCfg::xAddRefPicsPrevGOP( std::vector<int>& deltaList, const GOPEntry* gopEntry, const std::vector< std::pair<int, int> >& prevGopRefs ) const
{
  for( int i = 0; i < (int)prevGopRefs.size(); i++ )
  {
    if( prevGopRefs[ i ].second <= gopEntry->m_temporalId )
    {
      deltaList.push_back( gopEntry->m_POC - prevGopRefs[ i ].first );
      CHECK( deltaList.back() <= 0, "error in backward list: try to access future ref" );
    }
  }
}

int GOPCfg::xGetMaxTid( const GOPEntryList& gopList ) const
{
  int maxTid = 0;
  for( auto& gopEntry : gopList )
  {
    if( gopEntry.m_temporalId > maxTid )
    {
      maxTid = gopEntry.m_temporalId;
    }
  }
  return maxTid;
}

int GOPCfg::xGetMaxRefPics( const GOPEntry& gopEntry ) const
{
  int numRefPics = gopEntry.m_numRefPics[ 0 ];
  for( int j = 0; j < gopEntry.m_numRefPics[ 1 ]; j++ )
  {
    bool inL0 = false;
    for( int k = 0; k < gopEntry.m_numRefPics[ 0 ]; k++ )
    {
      if( gopEntry.m_deltaRefPics[ 1 ][ j ] == gopEntry.m_deltaRefPics[ 0 ][ k ] )
      {
        inL0 = true;
        break;
      }
    }
    if( ! inL0 )
    {
      numRefPics += 1;
    }
  }
  return numRefPics;
}

int GOPCfg::xGetMaxNumReorder( const GOPEntry& gopEntry, const GOPEntryList& gopList ) const
{
  int maxCodingNum = 0;
  for( int j = 0; j < (int)gopList.size(); j++ )
  {
    if( gopList[ j ].m_POC <= gopEntry.m_POC )
    {
      maxCodingNum = j;
    }
  }
  int numReorder = 0;
  for( int j = 0; j < maxCodingNum; j++ )
  {
    const GOPEntry& cmpEntry = gopList[ j ];
    if( cmpEntry.m_POC > gopEntry.m_POC && cmpEntry.m_temporalId <= gopEntry.m_temporalId )
    {
      numReorder += 1;
    }
  }
  return numReorder;
}

} // namespace vvenc

//! \}


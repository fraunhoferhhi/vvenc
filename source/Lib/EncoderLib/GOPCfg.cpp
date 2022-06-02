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

void GOPCfg::initGopList( int refreshType, int intraPeriod, int gopSize, bool bPicReordering, const vvencGOPEntry cfgGopList[ VVENC_MAX_GOP ], const vvencMCTF& mctfCfg )
{
  CHECK( gopSize < 1, "gop size has to be greater than 0" );

  m_mctfCfg          = &mctfCfg;
  m_refreshType      = refreshType;
  m_fixedIntraPeriod = intraPeriod;
  m_maxGopSize       = gopSize;
  m_gopMode          = m_fixedIntraPeriod != 1 ? ( bPicReordering ? GM_RA : GM_LD ) : GM_AI;
  m_defGopSize       = m_fixedIntraPeriod > 0 ? std::min( m_maxGopSize, m_fixedIntraPeriod ) : m_maxGopSize;
  m_maxNumRefs       = m_gopMode == GM_LD && m_maxGopSize >= 8 ? MAX_NUM_ACTIVE_REFS_LD8 : MAX_NUM_ACTIVE_REFS_RA;

  const int numGops    = m_fixedIntraPeriod > 0 ? m_fixedIntraPeriod / m_defGopSize               : 0;
  const int remainSize = m_fixedIntraPeriod > 0 ? m_fixedIntraPeriod - ( m_defGopSize * numGops ) : 0;
  const int numDefault = m_gopMode == GM_RA ? 2 : ( m_gopMode == GM_LD ? m_maxNumRefs : 1 );

  // setup default gop lists
  std::vector<GOPEntryList*> prevGopLists;
  prevGopLists.reserve( numDefault );
  m_defaultGopLists.resize( numDefault );
  for( int i = 0; i < numDefault; i++ )
  {
    xCreateGopList( m_gopMode, m_maxNumRefs, m_maxGopSize, m_defGopSize, prevGopLists, m_defaultGopLists[ i ] );
    prevGopLists.push_back( &m_defaultGopLists[ i ] );
  }
  if( remainSize && remainSize != m_defGopSize )
  {
    const int numPrev = std::min( numDefault - 1, numGops );
    prevGopLists.clear();
    prevGopLists.resize( numPrev );
    for( int i = 0; i < numPrev; i++ )
    {
      prevGopLists[ i ] = &m_defaultGopLists.back();
    }
    xCreateGopList( m_gopMode, m_maxNumRefs, m_maxGopSize, remainSize, prevGopLists, m_remainGopList );
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
  CHECK( m_refreshType == VVENC_DRT_IDR2 && m_gopMode != GM_RA, "refresh type idr2 only for random access possible" );
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
      m_numTillIntra = m_fixedIntraPeriod;
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

void GOPCfg::xCreateGopList( GOPMode gopMode, int maxNumRefs, int maxGopSize, int gopSize, const std::vector<GOPEntryList*>& prevGopLists, GOPEntryList& gopList ) const
{
  switch( gopMode )
  {
    case GM_AI:
      CHECK( gopSize != 1, "for all intra gop gop size has to be 1" );
      xCreateGopListAI( gopList );
      break;
    case GM_LD:
      xCreateGopListLD( maxNumRefs, maxGopSize, gopSize, prevGopLists, gopList );
      break;
    case GM_RA:
      xCreateGopListRA( maxNumRefs, maxGopSize, gopSize, prevGopLists, gopList );
      break;
    default:
      THROW( "unknown gop mode" );
  };

  xSetMctfIndex( maxGopSize, gopList );
}

void GOPCfg::xCreateGopListRA( int maxNumRefs, int maxGopSize, int gopSize, const std::vector<GOPEntryList*>& prevGopLists, GOPEntryList& gopList ) const
{
  gopList.clear();
  gopList.reserve( maxGopSize );

  // prepare pseudo previous gop, containing TL0 and TL1 pic
  int prevGopTL01[ 2 ] = { 0, 0 };
  if( prevGopLists.size() )
  {
    const GOPEntryList& prevList = *prevGopLists.back();
    const int prevGopSize = (int)prevList.size();
    for( const auto gopEntry : prevList )
    {
      if( gopEntry.m_temporalId == 0 || gopEntry.m_temporalId == 1 )
      {
        const int prevPoc = ( gopEntry.m_POC % prevGopSize ) - prevGopSize;
        prevGopTL01[ gopEntry.m_temporalId ] = prevPoc;
      }
    }
  }

  //
  // fill in gop entries with default dyadic pattern for maximum gop size
  //

  GOPEntry e1;
  e1.setDefaultGOPEntry();
  e1.m_POC        = maxGopSize;
  e1.m_temporalId = 0;
  gopList.push_back( e1 );
  xAddDyadicGopEntry( 0, maxGopSize, 1, gopList );

  // find max temporal id
  const int maxTid = xGetMaxTid( gopList );

  //
  // prune gop list
  //

  CHECK( gopSize > maxGopSize, "only pruning of gop list supported" );
  if( gopSize < maxGopSize )
  {
    std::list<GOPEntry> prunedList;
    prunedList.push_back( gopList[ 0 ] ); // always include TL01, end of pruned gop and start of following gop
    for( int i = 1; i < (int)gopList.size(); i++ )
    {
      if( gopList[ i ].m_POC < gopSize )
      {
        prunedList.push_back( gopList[ i ] );
      }
    }

    gopList.clear();
    gopList.reserve( gopSize );
    for( const auto& gopEntry : prunedList )
    {
      gopList.push_back( gopEntry );
    }

    // fix poc of TL0 pic
    gopList[ 0 ].m_POC = gopSize;

    CHECK( gopList.size() != gopSize, "pruned gop list incomplete" );
  }

  //
  // construct prediction pattern
  //

  const bool skip24 = ENABLE_SKIP24 && maxGopSize == 32;
  // use list of available entries for prediction and insert poc 0 to be always available
  std::vector<GOPEntry*> availList;
  availList.resize( gopSize + 1, nullptr );
  e1.setDefaultGOPEntry();
  e1.m_POC = 0;
  availList[ 0 ] = &e1;
  for( int i = 0; i < gopList.size(); i++ )
  {
    GOPEntry* gopEntry = &gopList[ i ];

    // LIST 0 (backward prediction list)

    std::vector<int> deltaList;
    deltaList.reserve( maxNumRefs );

    // first, search for available pics in backward direction of current gop
    xAddRefPicsBckwd( deltaList, gopEntry, availList, maxNumRefs, skip24 );
    // second, fill missing delta poc entries with pics from previous GOPs, if allowed (is not used/allowed for non-ref pics at the highest temporal layer) 
    if( (int)deltaList.size() < maxNumRefs && ( gopEntry->m_temporalId <= 1 || gopEntry->m_temporalId < maxTid ) )
    {
      xAddRefPicsPrevGOP( deltaList, gopEntry, maxNumRefs, prevGopTL01 );
    }
    // third, if still refs missing, search into forward direction
    if( (int)deltaList.size() < maxNumRefs )
    {
      xAddRefPicsFwd( deltaList, gopEntry, availList, maxNumRefs );
    }

    // copy delta poc list 0
    CHECK( (int)deltaList.size() >= VVENC_MAX_NUM_REF_PICS, "number of delta ref pic entries exceeds array bounds" );
    gopEntry->m_numRefPics[ 0 ]       = (int)deltaList.size();
    gopEntry->m_numRefPicsActive[ 0 ] = (int)deltaList.size();
    for( int j = 0; j < gopEntry->m_numRefPics[ 0 ]; j++ )
    {
      gopEntry->m_deltaRefPics[ 0 ][ j ] = deltaList[ j ];
    }

    // LIST 1 (forward prediction list)

    deltaList.clear();

    // first, search for available pics in forward direction
    xAddRefPicsFwd( deltaList, gopEntry, availList, maxNumRefs );
    // second, if refs missing, search into backward direction
    if( (int)deltaList.size() < maxNumRefs )
    {
      xAddRefPicsBckwd( deltaList, gopEntry, availList, maxNumRefs, false );
    }
    // third, fill missing delta poc entries with pics from previous GOPs, if allowed (is not used/allowed for non-ref pics at the highest temporal layer) 
    if( (int)deltaList.size() < maxNumRefs && ( gopEntry->m_temporalId <= 1 || gopEntry->m_temporalId < maxTid ) )
    {
      xAddRefPicsPrevGOP( deltaList, gopEntry, maxNumRefs, prevGopTL01 );
    }

    // copy delta poc list 1
    CHECK( (int)deltaList.size() >= VVENC_MAX_NUM_REF_PICS, "number of delta ref pic entries exceeds array bounds" );
    gopEntry->m_numRefPics[ 1 ]       = (int)deltaList.size();
    gopEntry->m_numRefPicsActive[ 1 ] = (int)deltaList.size();
    for( int j = 0; j < gopEntry->m_numRefPics[ 1 ]; j++ )
    {
      gopEntry->m_deltaRefPics[ 1 ][ j ] = deltaList[ j ];
    }

    // available for later use as ref
    availList[ gopEntry->m_POC ] = gopEntry;
  }

  //
  // add entries to ref lists, which are required as reference for later pictures in coding order
  //

  // create fake entry as start of potential next gop, to ensure TL0 and TL1 are kept alive
  GOPEntry e2;
  e2.setDefaultGOPEntry();
  e2.m_POC        = gopList[ 0 ].m_POC + gopSize;
  e2.m_temporalId = gopList[ 0 ].m_temporalId;
  // add TL0 pics of previous gop
  CHECK( gopList[ 0 ].m_POC != gopSize,  "first entry in dyadic gop list must have poc equal gop size" );
  CHECK( gopList[ 0 ].m_temporalId != 0, "first entry in dyadic gop list must have temporal id 0" );
  for( int l = 0; l < 2; l++ )
  {
    e2.m_deltaRefPics[ l ][ 0 ] = e2.m_POC - gopList[ 0 ].m_POC;
    e2.m_deltaRefPics[ l ][ 1 ] = e2.m_POC - 0;
    e2.m_numRefPics[ l ] = 2;
  }
  // add TL1, if available
  if( gopSize > 1 && gopList[ 1 ].m_temporalId == e2.m_temporalId + 1)
  {
    e2.m_deltaRefPics[ 0 ][ e2.m_numRefPics[ 0 ] ]  = e2.m_POC - gopList[ 1 ].m_POC;
    e2.m_numRefPics[ 0 ]                           += 1;
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
          const int deltaPoc = gopEntry->m_POC - reqPoc;
          const int addIdx   = deltaPoc > 0 ? 0 : 1; // add to backward L0 or forward L1 list
          CHECK( gopEntry->m_numRefPics[ addIdx ] >= VVENC_MAX_NUM_REF_PICS, "out of array bounds" );
          gopEntry->m_deltaRefPics[ addIdx ][ gopEntry->m_numRefPics[ addIdx ] ] = deltaPoc;
          gopEntry->m_numRefPics[ addIdx ] += 1;
          // small optimization: sort additional poc by value to decrease required bits for encoding the list
          const int sign = deltaPoc > 0 ? 1 : -1;
          for( int n = gopEntry->m_numRefPics[ addIdx ] - 1; n > maxNumRefs; n-- )
          {
            if( gopEntry->m_deltaRefPics[ addIdx ][ n ] * sign > gopEntry->m_deltaRefPics[ addIdx ][ n - 1 ] * sign )
              break;
            std::swap( gopEntry->m_deltaRefPics[ addIdx ][ n ], gopEntry->m_deltaRefPics[ addIdx ][ n - 1 ] );
          }
        }
      }
    }
    prevEntry = gopEntry;
  }

  //
  // fill in qp model parameters
  //

  CHECK( maxGopSize > 32, "array out of bounds" );
  const int* qpOffset       = maxGopSize > 16 ? GOP32_QPOFFSET              : GOP16_QPOFFSET;
  const double* modelOffset = maxGopSize > 16 ? GOP32_QPOFFSET_MODEL_OFFSET : GOP16_QPOFFSET_MODEL_OFFSET;
  const double* modelScale  = maxGopSize > 16 ? GOP32_QPOFFSET_MODEL_SCALE  : GOP16_QPOFFSET_MODEL_SCALE;
  for( int i = 0; i < gopList.size(); i++ )
  {
    GOPEntry* gopEntry = &gopList[ i ];
    const int tid      = gopEntry->m_temporalId;
    gopEntry->m_sliceType           = 'B';
    gopEntry->m_QPOffset            = qpOffset   [ tid ];
    gopEntry->m_QPOffsetModelOffset = modelOffset[ tid ];
    gopEntry->m_QPOffsetModelScale  = modelScale [ tid ];
  }

  // poc to gop map for fastwr access
  std::vector<int> pocToGopIdx;
  xCreatePocToGopIdx( gopList, false, pocToGopIdx );

  // STSA and forward B flags
  xSetSTSA     ( gopList, pocToGopIdx );
  xSetBckwdOnly( gopList );

  // mark first gop entry
  gopList[ 0 ].m_isStartOfGop = true;
}

void GOPCfg::xCreateGopListLD( int maxNumRefs, int maxGopSize, int gopSize, const std::vector<GOPEntryList*>& prevGopLists, GOPEntryList& gopList ) const
{
  gopList.clear();
  gopList.resize( gopSize );

  // prepare pseudo list of pics from previous gop
  std::vector<int> prevGopPocs;
  prevGopPocs.reserve( prevGopLists.size() + 1 );
  // poc 0 always available
  prevGopPocs.push_back( 0 );
  // add pocs from previous gops
  int prevPoc = 0;
  for( int i = (int)prevGopLists.size() - 1; i >= 0; i-- )
  {
    prevPoc -= (int)prevGopLists[ i ]->size();
    prevGopPocs.push_back( prevPoc );
  }

  //
  // fill in gop entries with ordered pattern
  //

  for( int i = 0; i < gopSize; i++ )
  {
    GOPEntry& gopEntry = gopList[ i ];
    gopEntry.setDefaultGOPEntry();
    gopEntry.m_POC        = i + 1;
    gopEntry.m_temporalId = 0;
    gopEntry.m_sliceType  = 'B';
    gopEntry.m_QPOffset   = 1;
    if( i < gopSize - 1 )
    {
      gopEntry.m_QPOffset            = i % 2 == 0 ? 5 : 4;
      gopEntry.m_QPOffsetModelOffset = -6.5;
      gopEntry.m_QPOffsetModelScale  = 0.2590;
    }
  }

  //
  // construct prediction pattern
  //

  for( int i = 0; i < gopList.size(); i++ )
  {
    GOPEntry* gopEntry = &gopList[ i ];

    std::vector<int> deltaList;
    deltaList.reserve( maxNumRefs );

    // use next poc (-1) and start pics of previous gops
    // if not enough previous gops available try to fill with nearby pocs
    const int furthestPoc  = prevGopPocs.back();
          int nearbyRefs   = std::max( maxNumRefs - (int)prevGopPocs.size(), 1 );
          int nearbyPoc    = gopEntry->m_POC - 1;
          int prevGopStart = 0;
    while( nearbyRefs && nearbyPoc >= furthestPoc )
    {
      // skip pocs from previous gops if already used as nearby poc
      if( prevGopStart < (int)prevGopPocs.size() && prevGopPocs[ prevGopStart ] == nearbyPoc )
      {
        prevGopStart += 1;
        const int prevAvail = (int)prevGopPocs.size() - prevGopStart;
        if( prevAvail < maxNumRefs - 1 )
        {
          nearbyRefs += 1;
        }
      }
      deltaList.push_back( gopEntry->m_POC - nearbyPoc );
      nearbyRefs -= 1;
      nearbyPoc  -= 1;
    }
    // copy remaining pocs from previous gops
    while( (int)deltaList.size() < maxNumRefs && prevGopStart < (int)prevGopPocs.size() )
    {
      deltaList.push_back( gopEntry->m_POC - prevGopPocs[ prevGopStart ] );
      prevGopStart += 1;
    }

    // copy delta poc list
    CHECK( (int)deltaList.size() >= VVENC_MAX_NUM_REF_PICS, "number of delta ref pic entries exceeds array bounds" );
    for( int l = 0; l < 2; l++ )
    {
      gopEntry->m_numRefPics[ l ]       = (int)deltaList.size();
      gopEntry->m_numRefPicsActive[ l ] = (int)deltaList.size();
      for( int j = 0; j < gopEntry->m_numRefPics[ l ]; j++ )
      {
        gopEntry->m_deltaRefPics[ l ][ j ] = deltaList[ j ];
      }
    }
  }

  // mark first gop entry
  gopList.back().m_isStartOfGop = true;
}

void GOPCfg::xCreateGopListAI( GOPEntryList& gopList ) const
{
  gopList.clear();
  gopList.resize( 1 );

  gopList[ 0 ].setDefaultGOPEntry();
  gopList[ 0 ].m_POC                   = 1;
  gopList[ 0 ].m_temporalId            = 0;
  gopList[ 0 ].m_sliceType             = 'I';
  gopList[ 0 ].m_isStartOfGop          = true;
  // TODO (jb): check smaller number of active ref pics for AI, number has to be set to 1 at least
  gopList[ 0 ].m_numRefPicsActive[ 0 ] = 4;
  gopList[ 0 ].m_numRefPicsActive[ 1 ] = 4;
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

// TODO (jb): check over all temporal level over all GOPs
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
    const bool isSTSA        = isTlSTSA[ tid ] & isLocalSTSA[ i ];
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

void GOPCfg::xAddDyadicGopEntry( int start, int end, int tid, GOPEntryList& gopList ) const
{
  if( end - start <= 1 )
    return;

  const int mid = start + ( ( end - start + 1 ) / 2 );

  GOPEntry e1;
  e1.setDefaultGOPEntry();
  e1.m_POC        = mid;
  e1.m_temporalId = tid;
  gopList.push_back( e1 );

  xAddDyadicGopEntry( start, mid, tid + 1, gopList );
  xAddDyadicGopEntry( mid, end, tid + 1, gopList );
}

void GOPCfg::xAddRefPicsBckwd( std::vector<int>& deltaList, const GOPEntry* gopEntry, const std::vector<GOPEntry*>& availList, int maxNumRefs, bool skip24 ) const
{
  int refTid = gopEntry->m_temporalId;
  for( int i = ( gopEntry->m_POC - 1 ); i >= 0; i-- )
  {
    const GOPEntry* refEntry = availList[ i ];
    if( refEntry )
    {
      if( skip24 && refEntry->m_POC == 24 && gopEntry->m_POC > 28 )
        continue;
      if( refEntry->m_temporalId < refTid || refEntry->m_temporalId == 0 )
      {
        deltaList.push_back( gopEntry->m_POC - refEntry->m_POC );
        refTid = refEntry->m_temporalId;
      }
    }
    if( (int)deltaList.size() >= maxNumRefs )
      break;
  }
}

void GOPCfg::xAddRefPicsFwd( std::vector<int>& deltaList, const GOPEntry* gopEntry, const std::vector<GOPEntry*>& availList, int maxNumRefs ) const
{
  int refTid = gopEntry->m_temporalId;
  for( int i = ( gopEntry->m_POC + 1 ); i < (int)availList.size(); i++ )
  {
    const GOPEntry* refEntry = availList[ i ];
    if( refEntry )
    {
      if( refEntry->m_temporalId < refTid || refEntry->m_temporalId == 0 )
      {
        deltaList.push_back( gopEntry->m_POC - refEntry->m_POC );
        refTid = refEntry->m_temporalId;
      }
    }
    if( (int)deltaList.size() >= maxNumRefs )
      break;
  }
}

void GOPCfg::xAddRefPicsPrevGOP( std::vector<int>& deltaList, const GOPEntry* gopEntry, const int maxNumRefs, const int prevGopTL01[ 2 ] ) const
{
  // use prev TID 1 pic, if possible
  if( prevGopTL01[ 1 ] && gopEntry->m_temporalId > 0 )
  {
    deltaList.push_back( gopEntry->m_POC - prevGopTL01[ 1 ] );
  }
  // if still necessary, use also prev TID 0 pic
  if( prevGopTL01[ 0 ] && (int)deltaList.size() < maxNumRefs  )
  {
    deltaList.push_back( gopEntry->m_POC - prevGopTL01[ 0 ] );
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


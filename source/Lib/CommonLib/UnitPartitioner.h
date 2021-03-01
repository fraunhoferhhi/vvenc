/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */
/** \file     UnitPartitioner.h
 *  \brief    Provides a class for partitioning management
 */

#pragma once

#include "Unit.h"
#include "CommonDef.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

typedef std::vector<UnitArea> Partitioning;

//////////////////////////////////////////////////////////////////////////
// PartManager class - manages the partitioning tree
//
// contains the currently processed partitioning area (currArea)
// as well as the all partitioning decisions that led to this area
// being processed (in m_partStack).
//////////////////////////////////////////////////////////////////////////

enum PartSplit
{
  CTU_LEVEL        = 0,
  CU_QUAD_SPLIT,

  CU_HORZ_SPLIT,
  CU_VERT_SPLIT,
  CU_TRIH_SPLIT,
  CU_TRIV_SPLIT,
  TU_MAX_TR_SPLIT,
  TU_NO_ISP,
  TU_1D_HORZ_SPLIT,
  TU_1D_VERT_SPLIT,
  SBT_VER_HALF_POS0_SPLIT,
  SBT_VER_HALF_POS1_SPLIT,
  SBT_HOR_HALF_POS0_SPLIT,
  SBT_HOR_HALF_POS1_SPLIT,
  SBT_VER_QUAD_POS0_SPLIT,
  SBT_VER_QUAD_POS1_SPLIT,
  SBT_HOR_QUAD_POS0_SPLIT,
  SBT_HOR_QUAD_POS1_SPLIT,
  NUM_PART_SPLIT,
  CU_MT_SPLIT             = 1000, ///< dummy element to indicate the MT (multi-type-tree) split
  CU_BT_SPLIT             = 1001, ///< dummy element to indicate the BT split
  CU_DONT_SPLIT           = 2000  ///< dummy element to indicate no splitting
};



struct PartLevel
{
  PartSplit    split;
  Partitioning parts;
  unsigned     idx;
  bool         checkdIfImplicit;
  bool         isImplicit;
  PartSplit    implicitSplit;
  PartSplit    firstSubPartSplit;
  bool         canQtSplit;
  bool         qgEnable;
  bool         qgChromaEnable;
  int          modeType;

  PartLevel();
  PartLevel( const PartSplit _split, const Partitioning&  _parts );
  PartLevel( const PartSplit _split,       Partitioning&& _parts );
  void init();
};

// set depending on max QT / BT possibilities
typedef static_vector<PartLevel, 2 * MAX_CU_DEPTH + 1> PartitioningStack;

class Partitioner
{
protected:
  PartitioningStack m_partStack;
#if _DEBUG
  UnitArea          m_currArea;
#endif

public:
  unsigned currDepth;
  unsigned currQtDepth;
  unsigned currTrDepth;
  unsigned currBtDepth;
  unsigned currMtDepth;
  unsigned currSubdiv;
  Position currQgPos;
  Position currQgChromaPos;

  unsigned currImplicitBtDepth;
  ChannelType chType;
  TreeType treeType;
  ModeType modeType;

  unsigned maxBTD;
  unsigned maxBtSize;
  unsigned minTSize;
  unsigned maxTtSize;
  unsigned minQtSize;

  const PartLevel& currPartLevel          () const { return m_partStack.back(); }
  const UnitArea&  currArea               () const { return currPartLevel().parts[currPartIdx()]; }
  const unsigned   currPartIdx            () const { return currPartLevel().idx; }
  const PartitioningStack& getPartStack   () const { return m_partStack; }
  const bool currQgEnable                 () const { return currPartLevel().qgEnable; }
  const bool currQgChromaEnable           () const { return currPartLevel().qgChromaEnable; }

  SplitSeries getSplitSeries              () const;
  ModeTypeSeries getModeTypeSeries        () const;

  void initCtu                            ( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice );
  void splitCurrArea                      ( const PartSplit split, const CodingStructure &cs );
  void exitCurrSplit                      ();
  bool nextPart                           ( const CodingStructure &cs, bool autoPop = false );
  bool hasNextPart                        ();

  void setCUData                          ( CodingUnit& cu );

  void copyState                          ( const Partitioner& other );

public:
  void canSplit                           ( const CodingStructure &cs, bool& canNo, bool& canQt, bool& canBh, bool& canBv, bool& canTh, bool& canTv );
  bool canSplit                           ( const PartSplit split, const CodingStructure &cs );
  bool canSplitISP                        ( const PartSplit split, const CodingStructure& cs, CodingUnit& cu );
  bool isSplitImplicit                    ( const PartSplit split, const CodingStructure &cs );
  PartSplit getImplicitSplit              (                        const CodingStructure &cs );
  bool isSepTree                          (                        const CodingStructure &cs );
  bool isConsInter                        () { return modeType == MODE_TYPE_INTER; }
  bool isConsIntra                        () { return modeType == MODE_TYPE_INTRA; }

  void setMaxMinDepth                     ( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const;
};

//////////////////////////////////////////////////////////////////////////
// Partitioner namespace - contains methods calculating the actual splits
//////////////////////////////////////////////////////////////////////////

namespace PartitionerImpl
{
  void getCUSubPartitions     ( Partitioning &sub, const UnitArea& cuArea,  const CodingStructure &cs, const PartSplit splitType = CU_QUAD_SPLIT );
  void getMaxTuTiling         ( Partitioning &sub, const UnitArea& curArea, const CodingStructure &cs );
  void getTUIntraSubPartitions( Partitioning &sub, const UnitArea& tuArea,  const CodingStructure &cs, const PartSplit splitType, const TreeType treeType);
  void getSbtTuTiling         ( Partitioning &sub, const UnitArea& curArea, const CodingStructure &cs, const PartSplit splitType );
};

} // namespace vvenc

//! \}


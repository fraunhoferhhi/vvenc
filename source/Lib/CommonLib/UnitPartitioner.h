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
/** \file     UnitPartitioner.h
 *  \brief    Provides a class for partitioning management
 */

#pragma once

#include "Unit.h"
#include "CommonDef.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

static_assert( MAX_CU_TILING_PARTITIONS >= 4, "Minimum required number of partitions for the Partitioning type is 4!" );
typedef std::vector <UnitArea> Partitioning;

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
#if ISP_VVC
  bool canSplitISP                        ( const PartSplit split, const CodingStructure& cs, CodingUnit& cu );
#endif 
  bool isSplitImplicit                    ( const PartSplit split, const CodingStructure &cs );
  PartSplit getImplicitSplit              (                        const CodingStructure &cs );
  bool isSepTree                          (                        const CodingStructure &cs );
  bool isConsInter                        () { return modeType == MODE_TYPE_INTER; }
  bool isConsIntra                        () { return modeType == MODE_TYPE_INTRA; }

  void setMaxMinDepth                     ( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const;
};

namespace PartitionerFactory
{
  Partitioner* get( const Slice& slice );
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


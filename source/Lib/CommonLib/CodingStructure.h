/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#pragma once

#include "Unit.h"
#include "CommonDef.h"
#include "UnitPartitioner.h"
#include "Slice.h"

#include <vector>
#include <mutex>

//! \ingroup CommonLib
//! \{

namespace vvenc {

struct Picture;


enum PictureType
{
  PIC_RECONSTRUCTION = 0,
  PIC_ORIGINAL,
  PIC_ORIGINAL_RSP,
  PIC_PREDICTION,
  PIC_RESIDUAL,
  PIC_RECON_WRAP,
  PIC_SAO_TEMP,
  NUM_PIC_TYPES,
  PIC_ORIGINAL_RSP_REC,
};

// ---------------------------------------------------------------------------
// coding structure
// ---------------------------------------------------------------------------

class CodingStructure
{
public:

  UnitArea         area;
  UnitArea         _maxArea;

  Picture*         picture;
  CodingStructure* parent;
  CodingStructure* lumaCS;
  Slice*           slice;

  UnitScale        unitScale[MAX_NUM_COMP];

  int         baseQP;
  int         prevQP[MAX_NUM_CH];
  int         currQP[MAX_NUM_CH];
  int         chromaQpAdj; //th this seems to belong to CUCtx rather than to cs
  const SPS*  sps;
  const PPS*  pps;
  PicHeader*  picHeader;
  APS*        alfAps[ALF_CTB_MAX_NUM_APS];
  APS*        lmcsAps;
  APS*        scalinglistAps;
  const VPS*  vps;
  const PreCalcValues* pcv;

  CodingStructure( XUCache& unitCache, std::mutex* mutex );
  void createPicLevel( const UnitArea& _unit, const PreCalcValues* _pcv );
  void createForSearch( const ChromaFormat _chromaFormat, const Area& _area );
  void destroy();
  void releaseIntermediateData();

  void rebindPicBufs();
  void createCoeffs();
  void destroyCoeffs();

  void allocateVectorsAtPicLevel();

  // ---------------------------------------------------------------------------
  // global accessors
  // ---------------------------------------------------------------------------

  const CodingUnit*    getCU(const Position& pos, const ChannelType _chType, const TreeType _treeType) const;
  const TransformUnit* getTU(const Position& pos, const ChannelType _chType, const int subTuIdx = -1) const;

  CodingUnit*          getCU(const Position& pos, const ChannelType _chType, const TreeType _treeType);
  CodingUnit*          getLumaCU( const Position& pos );
  TransformUnit*       getTU(const Position& pos, const ChannelType _chType, const int subTuIdx = -1);

  const CodingUnit*    getCU(const ChannelType& _chType, const TreeType _treeType) const { return getCU(area.blocks[_chType].pos(), _chType, _treeType); }
  const TransformUnit* getTU(const ChannelType& _chType) const { return getTU(area.blocks[_chType].pos(), _chType); }

  CodingUnit*          getCU(const ChannelType& _chType, const TreeType _treeType ) { return getCU(area.blocks[_chType].pos(), _chType, _treeType); }
  TransformUnit*       getTU(const ChannelType& _chType ) { return getTU(area.blocks[_chType].pos(), _chType); }

  const CodingUnit*    getCURestricted(const Position& pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType, const TreeType treeType) const;
  const CodingUnit*    getCURestricted(const Position& pos, const CodingUnit& curCu,    const ChannelType _chType) const;
  const TransformUnit* getTURestricted(const Position& pos, const TransformUnit& curTu, const ChannelType _chType) const;

  CodingUnit&     addCU(const UnitArea& unit, const ChannelType _chType, CodingUnit* cuInit = nullptr);
  TransformUnit&  addTU(const UnitArea& unit, const ChannelType _chType, CodingUnit* cu, TransformUnit* tuInit = nullptr);
  void addEmptyTUs( Partitioner &partitioner, CodingUnit* cu );

  CUTraverser     traverseCUs(const UnitArea& _unit, const ChannelType _chType);
  TUTraverser     traverseTUs(const UnitArea& _unit, const ChannelType _chType);

  cCUTraverser    traverseCUs(const UnitArea& _unit, const ChannelType _chType) const;
  cTUTraverser    traverseTUs(const UnitArea& _unit, const ChannelType _chType) const;
  // ---------------------------------------------------------------------------
  // encoding search utilities
  // ---------------------------------------------------------------------------

  double      cost;
  double      costDbOffset;
  double      lumaCost;
  uint64_t    fracBits;
  Distortion  dist;
  Distortion  interHad;

  void initStructData  ( const int QP = MAX_INT, const bool skipMotBuf = true, const UnitArea* area = nullptr );
  void initSubStructure(      CodingStructure& cs, const ChannelType chType, const UnitArea& subArea, const bool isTuEnc, PelStorage* pOrgBuffer = nullptr, PelStorage* pRspBuffer = nullptr);
  void compactResize   ( const UnitArea& area );

  void copyStructure   (const CodingStructure& cs, const ChannelType chType, const TreeType treeType, const bool copyTUs = false, const bool copyRecoBuffer = false);
  void useSubStructure (      CodingStructure& cs, const ChannelType chType, const TreeType treeType, const UnitArea& subArea, const bool cpyRecoToPic = true);

  void clearTUs( bool force = false );
  void clearCUs( bool force = false );

  void createTempBuffers( const bool isTopLayer );
  void destroyTempBuffers();
private:
  void createInternals(const UnitArea& _unit, const bool isTopLayer);

public:


  std::vector<    CodingUnit*> cus;
  std::vector< TransformUnit*> tus;

  LutMotionCand motionLut;
  std::vector<LutMotionCand> motionLutBuf;
  void addMiToLut(static_vector<HPMVInfo, MAX_NUM_HMVP_CANDS>& lut, const HPMVInfo &mi);

private:

  // needed for TU encoding
  bool m_isTuEnc;

  CodingUnit**      m_cuPtr   [MAX_NUM_CH];

  unsigned m_numCUs;
  unsigned m_numTUs;

  CUCache& m_cuCache;
  TUCache& m_tuCache;
  std::mutex* m_unitCacheMutex;

  std::vector<SAOBlkParam> m_sao;

  PelStorage  m_pred;
  PelStorage  m_resi;
  PelStorage  m_reco;
  PelStorage  m_rspreco;
  PelStorage* m_org;
  PelStorage* m_rsporg;

  TCoeffSig*  m_coeffs [MAX_NUM_COMP];
  int         m_offsets[MAX_NUM_COMP];

  std::vector<Mv>   m_dmvrMvCache;
  int               m_dmvrMvCacheOffset;

  MotionInfo*       m_motionBuf;

  LoopFilterParam*  m_lfParam[NUM_EDGE_DIR];

  Size              m_mapSize[MAX_NUM_CH];


public:
  CodingStructure*  bestParent;
  bool              resetIBCBuffer;

  MotionBuf getMotionBuf( const     Area& _area );
  MotionBuf getMotionBuf( const UnitArea& _area ) { return getMotionBuf( _area.Y() ); }
  MotionBuf getMotionBuf()                        { return getMotionBuf(  area.Y() ); }

  const CMotionBuf getMotionBuf( const     Area& _area ) const;
  const CMotionBuf getMotionBuf( const UnitArea& _area ) const { return getMotionBuf( _area.Y() ); }
  const CMotionBuf getMotionBuf()                        const { return getMotionBuf(  area.Y() ); }

  MotionInfo& getMotionInfo( const Position& pos );
  const MotionInfo& getMotionInfo( const Position& pos ) const;

  MotionInfo const* getMiMapPtr()    const { return m_motionBuf; }
  MotionInfo      * getMiMapPtr()          { return m_motionBuf; }
  ptrdiff_t         getMiMapStride() const { return ( ptrdiff_t ) g_miScaling.scaleHor( area.Y().width ); }

  LFPBuf getLoopFilterParamBuf(const DeblockEdgeDir& edgeDir);
  const CLFPBuf getLoopFilterParamBuf(const DeblockEdgeDir& edgeDir) const;

  LoopFilterParam const* getLFPMapPtr   ( const DeblockEdgeDir edgeDir ) const { return m_lfParam[edgeDir]; }
  LoopFilterParam      * getLFPMapPtr   ( const DeblockEdgeDir edgeDir )       { return m_lfParam[edgeDir]; }
  ptrdiff_t              getLFPMapStride() const { return ( ptrdiff_t ) m_mapSize[CH_L].width; }

  UnitScale getScaling(const UnitScale::ScaliningType type, const ChannelType chType = CH_L) const
  {
    return type == UnitScale::MI_MAP ? g_miScaling : unitScale[chType];
  }

public:
  // ---------------------------------------------------------------------------
  // temporary (shadowed) data accessors
  // ---------------------------------------------------------------------------
         PelBuf       getPredBuf(const CompArea& blk)           { return getBuf(blk,  PIC_PREDICTION); }
  const CPelBuf       getPredBuf(const CompArea& blk)     const { return getBuf(blk,  PIC_PREDICTION); }
         PelUnitBuf   getPredBuf(const UnitArea& unit)          { return getBuf(unit, PIC_PREDICTION); }
  const CPelUnitBuf   getPredBuf(const UnitArea& unit)    const { return getBuf(unit, PIC_PREDICTION); }

         PelBuf       getResiBuf(const CompArea& blk)           { return getBuf(blk,  PIC_RESIDUAL); }
  const CPelBuf       getResiBuf(const CompArea& blk)     const { return getBuf(blk,  PIC_RESIDUAL); }
         PelUnitBuf   getResiBuf(const UnitArea& unit)          { return getBuf(unit, PIC_RESIDUAL); }
  const CPelUnitBuf   getResiBuf(const UnitArea& unit)    const { return getBuf(unit, PIC_RESIDUAL); }

         PelBuf       getRecoBuf(const CompArea& blk)           { return getBuf(blk,  PIC_RECONSTRUCTION); }
  const CPelBuf       getRecoBuf(const CompArea& blk)     const { return getBuf(blk,  PIC_RECONSTRUCTION); }
         PelUnitBuf   getRecoBuf(const UnitArea& unit)          { return getBuf(unit, PIC_RECONSTRUCTION); }
  const CPelUnitBuf   getRecoBuf(const UnitArea& unit)    const { return getBuf(unit, PIC_RECONSTRUCTION); }

         PelBuf       getOrgBuf(const CompArea& blk)            { return getBuf(blk,  PIC_ORIGINAL); }
  const CPelBuf       getOrgBuf(const CompArea& blk)      const { return getBuf(blk,  PIC_ORIGINAL); }
         PelUnitBuf   getOrgBuf(const UnitArea& unit)           { return getBuf(unit, PIC_ORIGINAL); }
  const CPelUnitBuf   getOrgBuf(const UnitArea& unit)     const { return getBuf(unit, PIC_ORIGINAL); }

         PelBuf       getRspOrgBuf(const CompArea& blk)         { return getBuf(blk,  PIC_ORIGINAL_RSP); }
  const CPelBuf       getRspOrgBuf(const CompArea& blk)   const { return getBuf(blk,  PIC_ORIGINAL_RSP); }

         PelBuf        getRspRecoBuf(const CompArea &blk)         { return getBuf(blk, PIC_ORIGINAL_RSP_REC); }
  const CPelBuf        getRspRecoBuf(const CompArea &blk)   const { return getBuf(blk, PIC_ORIGINAL_RSP_REC); }

         PelUnitBuf&  getRecoBufRef()                           { return m_reco; }
         PelBuf&      getRspRecoBuf()                           { return m_rspreco.Y(); }
  const CPelBuf       getRspRecoBuf()                     const { return m_rspreco.Y(); }

         PelBuf&      getRspOrgBuf()                            { return m_rsporg->Y(); }
  const CPelBuf       getRspOrgBuf()                      const { return m_rsporg->Y(); }

         PelBuf       getOrgBuf(const ComponentID compID)       { return m_org->get(compID); }
  const CPelBuf       getOrgBuf(const ComponentID compID) const { return m_org->get(compID); }
         PelUnitBuf&  getOrgBuf()                               { return *m_org; }
  const CPelUnitBuf   getOrgBuf()                         const { return *m_org; }

  // pred buffer
         PelBuf       getPredBuf(const ComponentID compID)      { return m_pred.get(compID); }
  const CPelBuf       getPredBuf(const ComponentID compID)const { return m_pred.get(compID); }
         PelUnitBuf&  getPredBuf()                              { return m_pred; }
  const CPelUnitBuf   getPredBuf()                        const { return m_pred; }

  // resi buffer
         PelBuf       getResiBuf(const ComponentID compID)      { return m_resi.get(compID); }
  const CPelBuf       getResiBuf(const ComponentID compID)const { return m_resi.get(compID); }
         PelUnitBuf&  getResiBuf()                              { return m_resi; }
  const CPelUnitBuf   getResiBuf()                        const { return m_resi; }

  // reco buffer
  const CPelBuf       getRecoBuf(const ComponentID compID)const { return m_reco.get(compID); }
         PelBuf       getRecoBuf(const ComponentID compID)      { return m_reco.get(compID); }
         PelUnitBuf&  getRecoBuf()                              { return m_reco; }
  const CPelUnitBuf   getRecoBuf()                        const { return m_reco; }

private:

         PelBuf       getBuf(const CompArea& blk,  const PictureType type);
  const CPelBuf       getBuf(const CompArea& blk,  const PictureType type) const;
         PelUnitBuf   getBuf(const UnitArea& unit, const PictureType type);
  const CPelUnitBuf   getBuf(const UnitArea& unit, const PictureType type) const;
};


static inline uint32_t getNumberValidTBlocks(const PreCalcValues& pcv) { return (pcv.chrFormat==CHROMA_400) ? 1 : MAX_NUM_COMP; }

} // namespace vvenc

//! \}


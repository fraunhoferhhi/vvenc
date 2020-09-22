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
  PIC_ORIGINAL_LOCAL,
#if ISP_VVC
  PIC_ORIGINAL_RSP_REC,
#endif
};

// ---------------------------------------------------------------------------
// coding structure
// ---------------------------------------------------------------------------

class CodingStructure
{
public:

  UnitArea         area;

  Picture*         picture;
  CodingStructure* parent;
  CodingStructure* refCS;
  CodingStructure* bestCS;
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
  void create( const UnitArea& _unit, const bool isTopLayer, const PreCalcValues* _pcv );
  void create( const ChromaFormat _chromaFormat, const Area& _area, const bool isTopLayer );
  void destroy();
  void releaseIntermediateData();

  void rebindPicBufs();
  void createCoeffs();
  void destroyCoeffs();

  void allocateVectorsAtPicLevel();

  // ---------------------------------------------------------------------------
  // global accessors
  // ---------------------------------------------------------------------------

  bool isDecomp (const Position& pos, const ChannelType _chType) const;
  bool isDecomp (const Position& pos, const ChannelType _chType);
  void setDecomp(const CompArea& area, const bool _isCoded = true);
  void setDecomp(const UnitArea& area, const bool _isCoded = true);

  const CodingUnit     *getCU(const Position& pos, const ChannelType _chType, const TreeType _treeType) const;
  const PredictionUnit *getPU(const Position& pos, const ChannelType _chType) const;
  const TransformUnit  *getTU(const Position& pos, const ChannelType _chType, const int subTuIdx = -1) const;

  CodingUnit     *getCU(const Position& pos, const ChannelType _chType, const TreeType _treeType);
  CodingUnit     *getLumaCU( const Position& pos );
  PredictionUnit *getPU(const Position& pos, const ChannelType _chType);
  TransformUnit  *getTU(const Position& pos, const ChannelType _chType, const int subTuIdx = -1);

  const CodingUnit     *getCU(const ChannelType& _chType, const TreeType _treeType) const { return getCU(area.blocks[_chType].pos(), _chType, _treeType); }
  const PredictionUnit *getPU(const ChannelType& _chType) const { return getPU(area.blocks[_chType].pos(), _chType); }
  const TransformUnit  *getTU(const ChannelType& _chType) const { return getTU(area.blocks[_chType].pos(), _chType); }

  CodingUnit     *getCU(const ChannelType& _chType, const TreeType _treeType ) { return getCU(area.blocks[_chType].pos(), _chType, _treeType); }
  PredictionUnit *getPU(const ChannelType& _chType ) { return getPU(area.blocks[_chType].pos(), _chType); }
  TransformUnit  *getTU(const ChannelType& _chType ) { return getTU(area.blocks[_chType].pos(), _chType); }

  const CodingUnit     *getCURestricted(const Position& pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType, const TreeType treeType) const;
  const CodingUnit     *getCURestricted(const Position& pos, const CodingUnit& curCu,                               const ChannelType _chType) const;
  const PredictionUnit *getPURestricted(const Position& pos, const PredictionUnit& curPu,                           const ChannelType _chType) const;
  const TransformUnit  *getTURestricted(const Position& pos, const TransformUnit& curTu,                            const ChannelType _chType) const;

  CodingUnit&     addCU(const UnitArea& unit, const ChannelType _chType);
  PredictionUnit& addPU(const UnitArea& unit, const ChannelType _chType, CodingUnit* cu);
  TransformUnit&  addTU(const UnitArea& unit, const ChannelType _chType, CodingUnit* cu);
  void addEmptyTUs( Partitioner &partitioner, CodingUnit* cu );

  CUTraverser     traverseCUs(const UnitArea& _unit, const ChannelType _chType);
  TUTraverser     traverseTUs(const UnitArea& _unit, const ChannelType _chType);

  cCUSecureTraverser secureTraverseCUs(const UnitArea& _unit, const ChannelType _chType) const;
  cCUTraverser    traverseCUs(const UnitArea& _unit, const ChannelType _chType) const;
  cTUTraverser    traverseTUs(const UnitArea& _unit, const ChannelType _chType) const;
  // ---------------------------------------------------------------------------
  // encoding search utilities
  // ---------------------------------------------------------------------------

  static_vector<double, NUM_ENC_FEATURES> features;

  double      cost;
  double      costDbOffset;
  double      lumaCost;
  uint64_t    fracBits;
  Distortion  dist;
  Distortion  interHad;

  void initStructData  ( const int QP = MAX_INT, const bool skipMotBuf = false);
  void initSubStructure(      CodingStructure& cs, const ChannelType chType, const UnitArea& subArea, const bool isTuEnc, PelStorage* pOrgBuffer = nullptr, PelStorage* pRspBuffer = nullptr);

  void copyStructure   (const CodingStructure& cs, const ChannelType chType, const TreeType treeType, const bool copyTUs = false, const bool copyRecoBuffer = false);
  void useSubStructure (const CodingStructure& cs, const ChannelType chType, const TreeType treeType, const UnitArea& subArea, const bool cpyReco );

  void clearTUs();
  void clearPUs();
  void clearCUs();
  const int signalModeCons( const PartSplit split, Partitioner &partitioner, const ModeType modeTypeParent ) const;

private:
  void createInternals(const UnitArea& _unit, const bool isTopLayer);

public:


  std::vector<    CodingUnit*> cus;
  std::vector<PredictionUnit*> pus;
  std::vector< TransformUnit*> tus;

  LutMotionCand motionLut;
  std::vector<LutMotionCand> motionLutBuf;
  void addMiToLut(static_vector<HPMVInfo, MAX_NUM_HMVP_CANDS>& lut, const HPMVInfo &mi);

private:

  // needed for TU encoding
  bool m_isTuEnc;

  CodingUnit      **m_cuPtr   [MAX_NUM_CH];
  PredictionUnit  **m_puPtr   [MAX_NUM_CH];
  TransformUnit   **m_tuPtr   [MAX_NUM_CH];
  bool             *m_isDecomp[MAX_NUM_CH];
#if ISP_VVC_IDX
  unsigned         *m_tuIdx   [MAX_NUM_CH];
#endif

  unsigned m_numCUs;
  unsigned m_numPUs;
  unsigned m_numTUs;

  CUCache& m_cuCache;
  PUCache& m_puCache;
  TUCache& m_tuCache;
  std::mutex* m_unitCacheMutex;

  std::vector<SAOBlkParam> m_sao;

  PelStorage m_pred;
  PelStorage m_resi;
  PelStorage m_reco;
  PelStorage m_rspreco;
  PelStorage* m_org;
  PelStorage* m_rsporg;

  TCoeff *m_coeffs [MAX_NUM_COMP];
  Pel    *m_pcmbuf [MAX_NUM_COMP];
  bool   *m_runType[MAX_NUM_CH];
  int     m_offsets[MAX_NUM_COMP];

  MotionInfo *m_motionBuf;

  LoopFilterParam *m_lfParam[NUM_EDGE_DIR];

  Size             m_mapSize[MAX_NUM_CH];


public:
  CodingStructure*  bestParent;

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

  UnitScale getScaling(const UnitScale::ScaliningType type, const ChannelType chType = CH_L)
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

#if ISP_VVC
         PelBuf        getRspRecoBuf(const CompArea &blk)         { return getBuf(blk, PIC_ORIGINAL_RSP_REC); }
  const CPelBuf        getRspRecoBuf(const CompArea &blk)   const { return getBuf(blk, PIC_ORIGINAL_RSP_REC); }
#endif

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


static inline uint32_t getNumberValidTBlocks(const PreCalcValues& pcv) { return (pcv.chrFormat==CHROMA_400) ? 1 : ( pcv.multiBlock422 ? MAX_NUM_TBLOCKS : MAX_NUM_COMP ); }

} // namespace vvenc

//! \}


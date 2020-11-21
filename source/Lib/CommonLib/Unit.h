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
/** \file     Unit.h
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#pragma once

#include "CommonDef.h"
#include "Common.h"
#include "Mv.h"
#include "MotionInfo.h"

#include <iterator>

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ---------------------------------------------------------------------------
// tools
// ---------------------------------------------------------------------------

  inline Position recalcPosition(const ChromaFormat _cf, const ComponentID srcCId, const ComponentID dstCId, const Position& pos)
{
  if( toChannelType( srcCId ) == toChannelType( dstCId ) )
  {
    return pos;
  }
  else if (isLuma(srcCId) && isChroma(dstCId))
  {
    return Position(pos.x >> getComponentScaleX(dstCId, _cf), pos.y >> getComponentScaleY(dstCId, _cf));
  }
  else
  {
    return Position(pos.x << getComponentScaleX(srcCId, _cf), pos.y << getComponentScaleY(srcCId, _cf));
  }
}

inline Position recalcPosition( const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Position& pos )
{
  if( srcCHt == dstCHt )
  {
    return pos;
  }
  else if( isLuma( srcCHt ) && isChroma( dstCHt ) )
  {
    return Position( pos.x >> getChannelTypeScaleX( dstCHt, _cf ), pos.y >> getChannelTypeScaleY( dstCHt, _cf ) );
  }
  else
  {
    return Position( pos.x << getChannelTypeScaleX( srcCHt, _cf ), pos.y << getChannelTypeScaleY( srcCHt, _cf ) );
  }
}

inline Size recalcSize( const ChromaFormat _cf, const ComponentID srcCId, const ComponentID dstCId, const Size& size )
{
  if( toChannelType( srcCId ) == toChannelType( dstCId ) )
  {
    return size;
  }
  else if( isLuma( srcCId ) && isChroma( dstCId ) )
  {
    return Size( size.width >> getComponentScaleX( dstCId, _cf ), size.height >> getComponentScaleY( dstCId, _cf ) );
  }
  else
  {
    return Size( size.width << getComponentScaleX( srcCId, _cf ), size.height << getComponentScaleY( srcCId, _cf ) );
  }
}

inline Size recalcSize( const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Size& size )
{
  if( srcCHt == dstCHt )
  {
    return size;
  }
  else if( isLuma( srcCHt ) && isChroma( dstCHt ) )
  {
    return Size( size.width >> getChannelTypeScaleX( dstCHt, _cf ), size.height >> getChannelTypeScaleY( dstCHt, _cf ) );
  }
  else
  {
    return Size( size.width << getChannelTypeScaleX( srcCHt, _cf ), size.height << getChannelTypeScaleY( srcCHt, _cf ) );
  }
}

// ---------------------------------------------------------------------------
// block definition
// ---------------------------------------------------------------------------

struct CompArea : public Area
{
  CompArea() : Area(), chromaFormat(NUM_CHROMA_FORMAT), compID(MAX_NUM_TBLOCKS)                                                                                                                                 { }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const Area& _area, const bool isLuma = false)                                          : Area(_area),          chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const Position& _pos, const Size& _size, const bool isLuma = false)                    : Area(_pos, _size),    chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const uint32_t _x, const uint32_t _y, const uint32_t _w, const uint32_t _h, const bool isLuma = false) : Area(_x, _y, _w, _h), chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }

  ChromaFormat chromaFormat;
  ComponentID compID;

  Position chromaPos() const;
  Position lumaPos()   const;

  Size     chromaSize() const;
  Size     lumaSize()   const;

  Position compPos( const ComponentID compID ) const;
  Position chanPos( const ChannelType chType ) const;

  Position topLeftComp    (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, *this);                                                     }
  Position topRightComp   (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { (PosType) (x + width - 1), y                          }); }
  Position bottomLeftComp (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { x                        , (PosType) (y + height - 1 )}); }
  Position bottomRightComp(const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { (PosType) (x + width - 1), (PosType) (y + height - 1 )}); }

  bool valid() const { return chromaFormat < NUM_CHROMA_FORMAT && compID < MAX_NUM_TBLOCKS && width != 0 && height != 0; }

  const bool operator==(const CompArea& other) const
  {
    if (chromaFormat != other.chromaFormat) return false;
    if (compID       != other.compID)       return false;

    return Position::operator==(other) && Size::operator==(other);
  }

  const bool operator!=(const CompArea& other) const { return !(operator==(other)); }

  void     repositionTo      (const Position& newPos)       { Position::repositionTo(newPos); }
  void     positionRelativeTo(const CompArea& origCompArea) { Position::relativeTo(origCompArea); }

private:

  void xRecalcLumaToChroma();
};

inline CompArea clipArea(const CompArea& compArea, const Area& boundingBox)
{
  return CompArea(compArea.compID, compArea.chromaFormat, clipArea((const Area&) compArea, boundingBox));
}

// ---------------------------------------------------------------------------
// unit definition
// ---------------------------------------------------------------------------

typedef static_vector<CompArea, MAX_NUM_TBLOCKS> UnitBlocksType;

struct UnitArea
{
  ChromaFormat chromaFormat;
  UnitBlocksType blocks;

  UnitArea() : chromaFormat(NUM_CHROMA_FORMAT) { }
  UnitArea(const ChromaFormat _chromaFormat);
  UnitArea(const ChromaFormat _chromaFormat, const Area& area);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea&  blkY);
  UnitArea(const ChromaFormat _chromaFormat,       CompArea&& blkY);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea & blkY, const CompArea&  blkCb, const CompArea&  blkCr);
  UnitArea(const ChromaFormat _chromaFormat,       CompArea&& blkY,       CompArea&& blkCb,       CompArea&& blkCr);
        CompArea& Y()                                  { return blocks[COMP_Y];  }
  const CompArea& Y()                            const { return blocks[COMP_Y];  }
        CompArea& Cb()                                 { return blocks[COMP_Cb]; }
  const CompArea& Cb()                           const { return blocks[COMP_Cb]; }
        CompArea& Cr()                                 { return blocks[COMP_Cr]; }
  const CompArea& Cr()                           const { return blocks[COMP_Cr]; }

        CompArea& block(const ComponentID comp)       { return blocks[comp]; }
  const CompArea& block(const ComponentID comp) const { return blocks[comp]; }

  bool contains(const UnitArea& other) const;
  bool contains(const UnitArea& other, const ChannelType chType) const;

        CompArea& operator[]( const int n )       { return blocks[n]; }
  const CompArea& operator[]( const int n ) const { return blocks[n]; }

  const bool operator==(const UnitArea& other) const
  {
    if (chromaFormat != other.chromaFormat)   return false;
    if (blocks.size() != other.blocks.size()) return false;

    for (uint32_t i = 0; i < blocks.size(); i++)
    {
      if (blocks[i] != other.blocks[i]) return false;
    }

    return true;
  }

  void repositionTo(const UnitArea& unit);

  const bool operator!=(const UnitArea& other) const { return !(*this == other); }

  const Position& lumaPos () const { return Y(); }
  const Size&     lumaSize() const { return Y(); }

  const Position& chromaPos () const { return Cb(); }
  const Size&     chromaSize() const { return Cb(); }

  const UnitArea  singleComp(const ComponentID compID) const;
  const UnitArea  singleChan(const ChannelType chType) const;

  const SizeType  lwidth()  const { return Y().width; }  /*! luma width  */
  const SizeType  lheight() const { return Y().height; } /*! luma height */

  const PosType   lx() const { return Y().x; }           /*! luma x-pos */
  const PosType   ly() const { return Y().y; }           /*! luma y-pos */

  bool valid() const { return chromaFormat != NUM_CHROMA_FORMAT && blocks.size() > 0; }
};

inline UnitArea clipArea(const UnitArea& area, const UnitArea& boundingBox)
{
  UnitArea ret(area.chromaFormat);

  for (uint32_t i = 0; i < area.blocks.size(); i++)
  {
    ret.blocks.push_back(clipArea(area.blocks[i], boundingBox.blocks[i]));
  }

  return ret;
}

struct UnitAreaRelative : public UnitArea
{
  UnitAreaRelative(const UnitArea& origUnit, const UnitArea& unit)
  {
    *((UnitArea*)this) = unit;
    for(uint32_t i = 0; i < blocks.size(); i++)
    {
      blocks[i].positionRelativeTo(origUnit.blocks[i]);
    }
  }
};

// ---------------------------------------------------------------------------
// coding unit
// ---------------------------------------------------------------------------

} // namespace vvenc

#define __IN_UNIT_H__
#include "Buffer.h"
#undef __IN_UNIT_H__

namespace vvenc {

struct TransformUnit;
class  CodingStructure;


// ---------------------------------------------------------------------------
// prediction unit
// ---------------------------------------------------------------------------

struct IntraPredictionData
{
  uint8_t  intraDir[MAX_NUM_CH];
  uint8_t  multiRefIdx;
  bool     mipTransposedFlag;
};

struct InterPredictionData
{
  InterPredictionData() : mvdL0SubPu(nullptr) {}

  bool        mergeFlag;
  bool        regularMergeFlag;
  bool        ciip;
  bool        mvRefine;
  bool        mmvdMergeFlag;
  uint8_t     mergeIdx;
  uint8_t     geoSplitDir;
  uint8_t     geoMergeIdx0;
  uint8_t     geoMergeIdx1;
  uint8_t     interDir;
  uint8_t     mcControl; // mmvd(bio), luma/chroma
  uint32_t    mmvdMergeIdx;
  MergeType   mergeType;
  Mv*         mvdL0SubPu;

  uint8_t     mvpIdx  [NUM_REF_PIC_LIST_01];
  uint8_t     mvpNum  [NUM_REF_PIC_LIST_01];
  Mv          mvd     [NUM_REF_PIC_LIST_01];
  Mv          mv      [NUM_REF_PIC_LIST_01];
  int16_t     refIdx  [NUM_REF_PIC_LIST_01];
  Mv          mvdAffi [NUM_REF_PIC_LIST_01][3];
  Mv          mvAffi  [NUM_REF_PIC_LIST_01][3];
};



struct CodingUnit : public UnitArea, public IntraPredictionData, public InterPredictionData
{
  CodingStructure*  cs;
  Slice*            slice;
  ChannelType       chType;

  PredMode          predMode;

  uint8_t           depth;   // number of all splits, applied with generalized splits
  uint8_t           qtDepth; // number of applied quad-splits, before switching to the multi-type-tree (mtt)
  // a triple split would increase the mtDepth by 1, but the qtDepth by 2 in the first and last part and by 1 in the middle part (because of the 1-2-1 split proportions)
  uint8_t           btDepth; // number of applied binary splits, after switching to the mtt (or it's equivalent)
  uint8_t           mtDepth; // the actual number of splits after switching to mtt (equals btDepth if only binary splits are allowed)
  int8_t            chromaQpAdj;
  int8_t            qp;
  SplitSeries       splitSeries;
  TreeType          treeType;
  ModeType          modeType;
  ModeTypeSeries    modeTypeSeries;
  bool              skip;
  bool              mmvdSkip;
  bool              colorTransform;
  bool              geo;
  bool              rootCbf;
  bool              mipFlag;
  bool              affine;
  uint8_t           affineType;
  uint8_t           imv;
  uint8_t           sbtInfo;
  uint8_t           mtsFlag;
  uint8_t           lfnstIdx;
  uint8_t           BcwIdx;
  int8_t            imvNumCand;
  uint8_t           smvdMode;
  uint8_t           ispMode;
  uint8_t           bdpcmM[MAX_NUM_CH];
  uint32_t          tileIdx;

  // needed for fast imv mode decisions

  CodingUnit() : chType( CH_L ) {}
  CodingUnit(const UnitArea& unit);
  CodingUnit(const ChromaFormat _chromaFormat, const Area& area);

  void initData();
  void initPuData();

  CodingUnit& operator= ( const CodingUnit& other );
  CodingUnit& operator= ( const InterPredictionData& other);
  CodingUnit& operator= ( const IntraPredictionData& other);
  CodingUnit& operator= ( const MotionInfo& mi);

  // for accessing motion information, which can have higher resolution than PUs (should always be used, when accessing neighboring motion information)
  const MotionInfo& getMotionInfo () const;
  const MotionInfo& getMotionInfo ( const Position& pos ) const;
  MotionBuf         getMotionBuf  ();
  CMotionBuf        getMotionBuf  () const;


  unsigned       idx;
  CodingUnit*    next;

  TransformUnit* firstTU;
  TransformUnit* lastTU;
};

// ---------------------------------------------------------------------------
// transform unit
// ---------------------------------------------------------------------------

struct TransformUnit : public UnitArea
{
  CodingUnit*      cu;
  CodingStructure* cs;
  ChannelType      chType;
  int              chromaAdj;
  uint8_t          depth;
  bool             noResidual;
  uint8_t          jointCbCr;
  uint8_t          mtsIdx      [ MAX_NUM_TBLOCKS ];
  uint8_t          cbf         [ MAX_NUM_TBLOCKS ];
  int16_t          lastPos     [ MAX_NUM_TBLOCKS ];


  unsigned         idx;
  TransformUnit*   next;
  TransformUnit*   prev;

  TransformUnit                           () : chType( CH_L ) { }
  TransformUnit                           ( const UnitArea& unit);
  TransformUnit                           ( const ChromaFormat _chromaFormat, const Area& area);
  void          initData                  ();
  void          init                      ( TCoeff **coeffs);

  TransformUnit& operator=                ( const TransformUnit& other);
  void          copyComponentFrom         ( const TransformUnit& other, const ComponentID compID);
  void          checkTuNoResidual         ( unsigned idx );
  int           getTbAreaAfterCoefZeroOut ( ComponentID compID) const;

       CoeffBuf getCoeffs                 ( ComponentID id)       { return  CoeffBuf(m_coeffs[id], blocks[id]); }
const CCoeffBuf getCoeffs                 ( ComponentID id) const { return CCoeffBuf(m_coeffs[id], blocks[id]); }

private:
  TCoeff* m_coeffs[ MAX_NUM_TBLOCKS ];
};

// ---------------------------------------------------------------------------
// Utility class for easy for-each like unit traversing
// ---------------------------------------------------------------------------

template<typename T>
class UnitIterator : public std::iterator<std::forward_iterator_tag, T>
{
private:
  T* m_punit;

public:
  UnitIterator(           ) : m_punit( nullptr ) { }
  UnitIterator( T* _punit ) : m_punit( _punit  ) { }

  typedef T&       reference;
  typedef T const& const_reference;
  typedef T*       pointer;
  typedef T const* const_pointer;

  reference        operator*()                                      { return *m_punit; }
  const_reference  operator*()                                const { return *m_punit; }
  pointer          operator->()                                     { return  m_punit; }
  const_pointer    operator->()                               const { return  m_punit; }

  UnitIterator<T>& operator++()                                     { m_punit = m_punit->next; return *this; }
  UnitIterator<T>  operator++( int )                                { auto x = *this; ++( *this ); return x; }
  bool             operator!=( const UnitIterator<T>& other ) const { return m_punit != other.m_punit; }
  bool             operator==( const UnitIterator<T>& other ) const { return m_punit == other.m_punit; }
};

template<typename T>
class UnitTraverser
{
private:
  T* m_begin;
  T* m_end;

public:
  UnitTraverser(                    ) : m_begin( nullptr ), m_end( nullptr ) { }
  UnitTraverser( T* _begin, T* _end ) : m_begin( _begin  ), m_end( _end    ) { }

  typedef T                     value_type;
  typedef size_t                size_type;
  typedef T&                    reference;
  typedef T const&              const_reference;
  typedef T*                    pointer;
  typedef T const*              const_pointer;
  typedef UnitIterator<T>       iterator;
  typedef UnitIterator<const T> const_iterator;

  iterator        begin()        { return UnitIterator<T>( m_begin ); }
  const_iterator  begin()  const { return UnitIterator<T>( m_begin ); }
  const_iterator  cbegin() const { return UnitIterator<T>( m_begin ); }
  iterator        end()          { return UnitIterator<T>( m_end   ); }
  const_iterator  end()    const { return UnitIterator<T>( m_end   ); }
  const_iterator  cend()   const { return UnitIterator<T>( m_end   ); }
};

typedef UnitTraverser<CodingUnit>     CUTraverser;
typedef UnitTraverser<TransformUnit>  TUTraverser;

typedef UnitTraverser<const CodingUnit>     cCUTraverser;
typedef UnitTraverser<const TransformUnit>  cTUTraverser;

template<typename T>
struct SecureUnitTraverser
{
  T* begin;
  T* last;

  SecureUnitTraverser(                     ) : begin( nullptr ), last( nullptr ) { }
  SecureUnitTraverser( T* _begin, T* _last ) : begin( _begin  ), last( _last   ) { }
};

typedef SecureUnitTraverser< const CodingUnit>     cCUSecureTraverser;
typedef SecureUnitTraverser< const TransformUnit>  cTUSecureTraverser;

} // namespace vvenc

//! \}


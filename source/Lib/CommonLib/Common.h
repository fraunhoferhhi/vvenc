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
/** \file     Common.h
 *  \brief    Common 2D-geometrical structures
 */

#pragma once

#include "CommonDef.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

typedef int PosType;
typedef uint32_t SizeType;
struct Position
{
  PosType x;
  PosType y;

  Position()                                   : x(0),  y(0)  { }
  Position(const PosType _x, const PosType _y) : x(_x), y(_y) { }

  bool operator!=(const Position& other)              const { return x != other.x || y != other.y; }
  bool operator==(const Position& other)              const { return x == other.x && y == other.y; }

  Position offset(const Position pos)                 const { return Position(x + pos.x, y + pos.y); }
  Position offset(const PosType _x, const PosType _y) const { return Position(x + _x   , y + _y   ); }
  void     repositionTo(const Position newPos)              { x  = newPos.x; y  = newPos.y; }
  void     relativeTo  (const Position origin)              { x -= origin.x; y -= origin.y; }

  Position operator-( const Position& other )         const { return{ x - other.x, y - other.y }; }
};

struct Size
{
  SizeType width;
  SizeType height;

  Size()                                              : width(0),      height(0)       { }
  Size(const SizeType _width, const SizeType _height) : width(_width), height(_height) { }

  bool operator!=(const Size& other)      const { return (width != other.width) || (height != other.height); }
  bool operator==(const Size& other)      const { return (width == other.width) && (height == other.height); }
  uint32_t area()                         const { return (uint32_t) width * (uint32_t) height; }
};

struct Area : public Position, public Size
{
  Area()                                                                         : Position(),       Size()       { }
  Area(const Position& _pos, const Size& _size)                                  : Position(_pos),   Size(_size)  { }
  Area(const PosType _x, const PosType _y, const SizeType _w, const SizeType _h) : Position(_x, _y), Size(_w, _h) { }

        Position& pos()                           { return *this; }
  const Position& pos()                     const { return *this; }
        Size&     size()                          { return *this; }
  const Size&     size()                    const { return *this; }

  const Position& topLeft()                 const { return *this; }
        Position  topRight()                const { return { (PosType) (x + width - 1), y                          }; }
        Position  bottomLeft()              const { return { x                        , (PosType) (y + height - 1) }; }
        Position  bottomRight()             const { return { (PosType) (x + width - 1), (PosType) (y + height - 1) }; }
        Position  center()                  const { return { (PosType) (x + width / 2), (PosType) (y + height / 2) }; }

  bool contains(const Position& _pos)       const { return ((unsigned)(_pos.x-x) < (unsigned)width) && ((unsigned)(_pos.y-y) < (unsigned)height); }
  bool contains(const Area& _area)          const { return contains(_area.pos()) && contains(_area.bottomRight()); }

  bool operator!=(const Area& other)        const { return (Size::operator!=(other)) || (Position::operator!=(other)); }
  bool operator==(const Area& other)        const { return (Size::operator==(other)) && (Position::operator==(other)); }
};

struct UnitScale
{
  enum ScaliningType
  {
    UNIT_MAP,
    LF_PARAM_MAP,
    MI_MAP
  };

  UnitScale()                 : posx( 0), posy( 0), area(posx+posy) {}
  UnitScale( int sx, int sy ) : posx(sx), posy(sy), area(posx+posy) {}
  int posx;
  int posy;
  int area;

  template<typename T> T scaleHor ( const T &in ) const { return in >> posx; }
  template<typename T> T scaleVer ( const T &in ) const { return in >> posy; }
  template<typename T> T scaleArea( const T &in ) const { return in >> area; }

  Position scale( const Position& pos  ) const { return { pos.x >> posx, pos.y >> posy }; }
  Size     scale( const Size     &size ) const { return { size.width >> posx, size.height >> posy }; }
  Area     scale( const Area    &_area ) const { return Area( scale( _area.pos() ), scale( _area.size() ) ); }
};

inline ptrdiff_t rsAddr(const Position& pos, const uint32_t stride, const UnitScale &unitScale )
{
  return (ptrdiff_t)(stride >> unitScale.posx) * (ptrdiff_t)(pos.y >> unitScale.posy) + (ptrdiff_t)(pos.x >> unitScale.posx);
}

inline ptrdiff_t rsAddr(const Position& pos, const Position& origin, const uint32_t stride, const UnitScale &unitScale )
{
  return (ptrdiff_t)(stride >> unitScale.posx) * (ptrdiff_t)((pos.y - origin.y) >> unitScale.posy) + (ptrdiff_t)((pos.x - origin.x) >> unitScale.posx);
}

inline ptrdiff_t rsAddr(const Position& pos, const uint32_t stride )
{
  return (ptrdiff_t)stride * (ptrdiff_t)pos.y + (ptrdiff_t)pos.x;
}

inline ptrdiff_t rsAddr(const Position& pos, const Position& origin, const uint32_t stride )
{
  return (ptrdiff_t)stride * (ptrdiff_t)(pos.y - origin.y) + (ptrdiff_t)(pos.x - origin.x);
}

inline Area clipArea(const Area& _area, const Area& boundingBox)
{
  Area area = _area;

  if (area.x + area.width > boundingBox.x + boundingBox.width)
  {
    area.width = boundingBox.x + boundingBox.width - area.x;
  }

  if (area.y + area.height > boundingBox.y + boundingBox.height)
  {
    area.height = boundingBox.y + boundingBox.height - area.y;
  }

  return area;
}


} // namespace vvenc

//! \}


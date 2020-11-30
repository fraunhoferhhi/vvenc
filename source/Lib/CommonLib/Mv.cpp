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

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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


/** \file     Mv.cpp
    \brief    motion vector class
*/

#include "Mv.h"
#include "Common.h"
#include "CodingStructure.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

const MvPrecision Mv::m_amvrPrecision[4] = { MV_PRECISION_QUARTER, MV_PRECISION_INT, MV_PRECISION_4PEL, MV_PRECISION_HALF }; // for cu.imv=0, 1, 2 and 3
const MvPrecision Mv::m_amvrPrecAffine[3] = { MV_PRECISION_QUARTER, MV_PRECISION_SIXTEENTH, MV_PRECISION_INT }; // for cu.imv=0, 1 and 2
const MvPrecision Mv::m_amvrPrecIbc[3] = { MV_PRECISION_INT, MV_PRECISION_INT, MV_PRECISION_4PEL }; // for cu.imv=0, 1 and 2

void roundAffineMv( int& mvx, int& mvy, int nShift )
{
  const int nOffset = 1 << (nShift - 1);
  mvx = (mvx + nOffset - (mvx >= 0)) >> nShift;
  mvy = (mvy + nOffset - (mvy >= 0)) >> nShift;
}

void clipMv( Mv& rcMv, const Position& pos, const struct Size& size, const PreCalcValues& pcv )
{
  if( pcv.wrapArround )
  {
    return;
  }
  int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int iOffset = 8;
  int iHorMax = ( pcv.lumaWidth + iOffset - ( int ) pos.x - 1 ) << iMvShift;
  int iHorMin = ( -( int ) pcv.maxCUSize   - iOffset - ( int ) pos.x + 1 ) << iMvShift;

  int iVerMax = ( pcv.lumaHeight + iOffset - ( int ) pos.y - 1 ) << iMvShift;
  int iVerMin = ( -( int ) pcv.maxCUSize   - iOffset - ( int ) pos.y + 1 ) << iMvShift;

  rcMv.hor = ( std::min( iHorMax, std::max( iHorMin, rcMv.hor ) ) );
  rcMv.ver = ( std::min( iVerMax, std::max( iVerMin, rcMv.ver ) ) );
}

bool wrapClipMv( Mv& rcMv, const Position& pos, const struct Size& size, const CodingStructure& cs )
{
  bool wrapRef = true;
  int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int iOffset = 8;
  int iHorMax = ( cs.pcv->lumaWidth + cs.pcv->maxCUSize - size.width + iOffset - ( int ) pos.x - 1 ) << iMvShift;
  int iHorMin = ( -( int ) cs.pcv->maxCUSize                     - iOffset - ( int ) pos.x + 1 ) << iMvShift;
  int iVerMax = ( cs.pcv->lumaHeight + iOffset - ( int ) pos.y - 1 ) << iMvShift;
  int iVerMin = ( -( int ) cs.pcv->maxCUSize   - iOffset - ( int ) pos.y + 1 ) << iMvShift;
  int mvX = rcMv.hor;

  if(mvX > iHorMax)
  {
    mvX -= ( cs.pps->wrapAroundOffset << iMvShift );
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }
  if(mvX < iHorMin)
  {
    mvX += ( cs.pps->wrapAroundOffset << iMvShift );
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }

  rcMv.hor = ( mvX );
  rcMv.ver = ( std::min( iVerMax, std::max( iVerMin, rcMv.ver ) ) );
  return wrapRef;
}

} // namespace vvenc

//! \}


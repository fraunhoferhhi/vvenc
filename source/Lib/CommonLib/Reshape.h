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
 /** \file     Reshape.h
     \brief    reshaping header and class (header)
 */

#pragma once

#include "CommonDef.h"
#include "Rom.h"
#include "Slice.h"
#include "Unit.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class ReshapeData
{
protected:
  LmcsParam               m_sliceReshapeInfo;
  bool                    m_CTUFlag;
  int                     m_chromaScale;
  int                     m_lumaBD;
  int                     m_vpduX;
  int                     m_vpduY;
  double                  m_chromaWeightRS;
  std::vector<Pel>        m_invLUT;
  std::vector<Pel>        m_fwdLUT;
  std::vector<Pel>        m_reshapePivot;
  std::vector<int>        m_chromaAdjHelpLUT;
  std::vector<uint32_t>   m_reshapeLumaLevelToWeightPLUT;

public:
  ReshapeData()
    : m_CTUFlag         ( false )
    , m_chromaScale     ( 1 << CSCALE_FP_PREC )
    , m_lumaBD          ( 0 )
    , m_vpduX           ( -1 )
    , m_vpduY           ( -1 )
    , m_chromaWeightRS  ( 0.0 )
  {}

  virtual ~ReshapeData() {}

  void   copyReshapeData( const ReshapeData& d )          { *this = d; }

  const Pel* getFwdLUT()                            const { return m_fwdLUT.data(); }
  const Pel* getInvLUT()                            const { return m_invLUT.data(); }
  double getChromaWeight()                          const { return m_chromaWeightRS; }
  const uint32_t* getReshapeLumaLevelToWeightPLUT() const { return m_reshapeLumaLevelToWeightPLUT.data(); }

  bool   getCTUFlag()                               const { return m_CTUFlag; }
  void   setCTUFlag( bool b )                             { m_CTUFlag = b; }

  bool   isVPDUprocessed( int x, int y )            const { return ( ( x == m_vpduX ) && ( y == m_vpduY ) ); }
  void   setVPDULoc     ( int x, int y )                  { m_vpduX = x; m_vpduY = y; }

  LmcsParam& getSliceReshaperInfo()                       { return m_sliceReshapeInfo; }

  int    calculateChromaAdjVpduNei( const TransformUnit& tu, const CompArea& , const TreeType _treeType );

protected:
  int    getPWLIdxInv( int lumaVal )       const;
  int    calculateChromaAdj( Pel avgLuma ) const;
};

class Reshape : public ReshapeData
{
protected:
  std::vector<uint16_t>   m_binCW;
  uint16_t                m_initCW;
  bool                    m_reshape;
  std::vector<Pel>        m_inputPivot;
  std::vector<int32_t>    m_fwdScaleCoef;
  std::vector<int32_t>    m_invScaleCoef;
  int                     m_reshapeLUTSize;

  uint32_t                m_signalType;
  std::vector<double>     m_lumaLevelToWeightPLUT; //ALF

public:
  Reshape()
    : m_reshape (true)
  {}

  virtual ~Reshape() {}

  void createDec(int bitDepth);

  void constructReshaper();

  bool getReshapeFlag()                           const { return m_reshape; }
  void setReshapeFlag(bool b)                           { m_reshape = b; }
  const double* getlumaLevelToWeightPLUT()        const { return m_lumaLevelToWeightPLUT.data(); }

  void initLumaLevelToWeightTableReshape();
  void updateReshapeLumaLevelToWeightTableChromaMD (const Pel* ILUT);
  void restoreReshapeLumaLevelToWeightTable        ();
  void updateReshapeLumaLevelToWeightTable         (LmcsParam &sliceReshape, Pel* wtTable, double cwt);
};// END CLASS DEFINITION Reshape

} // namespace vvenc

//! \}



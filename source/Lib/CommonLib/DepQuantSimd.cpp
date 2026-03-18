/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
#include "DepQuantSimd.h"

#if ENABLE_SIMD_OPT_QUANT
#if defined ( TARGET_SIMD_ARM )
#include "arm/CommonDefARM.h"
#endif
#if defined ( TARGET_SIMD_X86 )
#include "x86/CommonDefX86.h"
#endif

//! \ingroup CommonLib
//! \{

namespace vvenc
{
namespace DQIntern
{
void CommonCtx::updateSimd( const ScanInfo& scanInfo, const int prevId, int stateId, StateMem& curr )
{
  uint8_t* sbbFlags = m_currSbbCtx[stateId].sbbFlags;
  uint8_t* levels = m_currSbbCtx[stateId].levels;
  uint16_t maxDist = m_nbInfo[scanInfo.scanIdx - 1].maxDist;
  uint16_t sbbSize = scanInfo.sbbSize;
  std::size_t setCpSize = ( maxDist > sbbSize ? maxDist - sbbSize : 0 ) * sizeof( uint8_t );
  if( prevId >= 0 )
  {
    ::memcpy( sbbFlags, m_prevSbbCtx[prevId].sbbFlags, scanInfo.numSbb * sizeof( uint8_t ) );
    ::memcpy( levels + scanInfo.scanIdx + sbbSize, m_prevSbbCtx[prevId].levels + scanInfo.scanIdx + sbbSize,
              setCpSize );
  }
  else
  {
    ::memset( sbbFlags, 0, scanInfo.numSbb * sizeof( uint8_t ) );
    ::memset( levels + scanInfo.scanIdx + sbbSize, 0, setCpSize );
  }
  sbbFlags[scanInfo.sbbPos] = !!curr.numSig[stateId];

  const int sigNSbb = ( ( scanInfo.nextSbbRight ? sbbFlags[scanInfo.nextSbbRight] : false ) ||
                                ( scanInfo.nextSbbBelow ? sbbFlags[scanInfo.nextSbbBelow] : false )
                            ? 1
                            : 0 );
  curr.refSbbCtxId[stateId] = stateId;
  const BinFracBits sbbBits = m_sbbFlagBits[sigNSbb];

  curr.sbbBits0[stateId] = sbbBits.intBits[0];
  curr.sbbBits1[stateId] = sbbBits.intBits[1];

  if( sigNSbb ||
      ( ( scanInfo.nextSbbRight && scanInfo.nextSbbBelow ) ? sbbFlags[scanInfo.nextSbbBelow + 1] : false ) )
  {
    const int scanBeg = scanInfo.scanIdx - scanInfo.sbbSize;
    const NbInfoOut* nbOut = m_nbInfo + scanBeg;
    const uint8_t* absLevels = levels + scanBeg;

    for( int id = 0; id < scanInfo.sbbSize; id++, nbOut++ )
    {
      const int idAddr = ( id << 2 ) + stateId;

      if( nbOut->num )
      {
        TCoeff sumAbs = 0, sumAbs1 = 0, sumNum = 0;
#define UPDATE( k )                                                                                                    \
{                                                                                                                    \
  TCoeff t = absLevels[nbOut->outPos[k]];                                                                            \
  sumAbs += t;                                                                                                       \
  sumAbs1 += std::min<TCoeff>( 4 + ( t & 1 ), t );                                                                   \
  sumNum += !!t;                                                                                                     \
}
        switch( nbOut->num )
        {
        default:
        case 5:
          UPDATE( 4 );
        case 4:
          UPDATE( 3 );
        case 3:
          UPDATE( 2 );
        case 2:
          UPDATE( 1 );
        case 1:
          UPDATE( 0 );
        }
#undef UPDATE
        curr.tplAcc[idAddr] = ( sumNum << 5 ) | sumAbs1;
        curr.sum1st[idAddr] = ( uint8_t )std::min( 255, sumAbs );
      }
    }
  }
}
} // namespace DQIntern

} // namespace vvenc

//! \}
#endif // ENABLE_SIMD_OPT_QUANT

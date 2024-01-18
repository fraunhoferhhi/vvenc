/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

#include "DepQuant.h"
#include "TrQuant.h"
#include "CodingStructure.h"
#include "UnitTools.h"
#ifdef TARGET_SIMD_X86
#  include "x86/CommonDefX86.h"
#  include <simde/x86/sse4.1.h>
#if defined( USE_SSE41 ) || !defined( REAL_TARGET_X86 )
#  include <simde/x86/sse4.2.h>
#endif
#endif

#include <bitset>

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if USE_SSE41 && defined( REAL_TARGET_X86 )
#define _my_cmpgt_epi64( a, b ) simde_mm_cmpgt_epi64( a, b )
#else
#define _my_cmpgt_epi64( a, b ) _mm_cmpgt_epi64( a, b )
#endif


namespace DQIntern
{
  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q   S T A T E                                                  =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  static constexpr int64_t rdCostInit = std::numeric_limits<int64_t>::max() >> 1;

  struct Decisions
  {
    int64_t   rdCost[4];
    TCoeffSig absLevel[4];
    int8_t    prevId[4];
  };

  template<X86_VEXT vext>
  class State;

  struct StateMem
  {
    uint8_t tpl[64];
    uint8_t sum[64];
    uint8_t val[64];

    struct
    {
      uint8_t sig[4];
      uint8_t cff[4];
    } ctx;

    int64_t  rdCost[4];

    int32_t  sbbBits0[4];
    int32_t  sbbBits1[4];

    uint8_t  numSig[4];
    int8_t   refSbbCtxId[4];

    int32_t  cffBits1[RateEstimator::sm_maxNumGtxCtx + 3];
    int      remRegBins[4];

    int      cffBitsCtxOffset;
    bool     anyRemRegBinsLt4;
    unsigned effWidth;
    unsigned effHeight;
    int      initRemRegBins;
  };

  struct SbbCtx
  {
    uint8_t*  sbbFlags;
    uint8_t*  levels;
  };

  template<X86_VEXT vext>
  class CommonCtx
  {
  public:
    CommonCtx() : m_currSbbCtx( m_allSbbCtx ), m_prevSbbCtx( m_currSbbCtx + 4 ) {}

    inline void swap() { std::swap(m_currSbbCtx, m_prevSbbCtx); }

    inline void reset( const TUParameters& tuPars, const RateEstimator &rateEst)
    {
      m_nbInfo = tuPars.m_scanId2NbInfoOut;
      ::memcpy( m_sbbFlagBits, rateEst.sigSbbFracBits(), 2*sizeof(BinFracBits) );
      const int numSbb    = tuPars.m_numSbb;
      const int chunkSize = numSbb + tuPars.m_numCoeff;
      uint8_t*  nextMem   = m_memory;
      for( int k = 0; k < 8; k++, nextMem += chunkSize )
      {
        m_allSbbCtx[k].sbbFlags = nextMem;
        m_allSbbCtx[k].levels   = nextMem + numSbb;
      }
    }

    inline void update( const ScanInfo &scanInfo, const int prevId, int stateId, StateMem &curr )
    {
      uint8_t*    sbbFlags  = m_currSbbCtx[stateId].sbbFlags;
      uint8_t*    levels    = m_currSbbCtx[stateId].levels;
      uint16_t    maxDist   = m_nbInfo[ scanInfo.scanIdx - 1 ].maxDist;
      uint16_t    sbbSize   = scanInfo.sbbSize;
      std::size_t setCpSize = ( maxDist > sbbSize ? maxDist - sbbSize : 0 ) * sizeof(uint8_t);
      if( prevId >= 0 )
      {
        ::memcpy( sbbFlags, m_prevSbbCtx[prevId].sbbFlags, scanInfo.numSbb * sizeof( uint8_t ) );
        ::memcpy( levels + scanInfo.scanIdx + sbbSize, m_prevSbbCtx[prevId].levels + scanInfo.scanIdx + sbbSize, setCpSize );
      }
      else
      {
        ::memset( sbbFlags, 0, scanInfo.numSbb * sizeof( uint8_t ) );
        ::memset( levels + scanInfo.scanIdx + sbbSize, 0, setCpSize );
      }
      sbbFlags[scanInfo.sbbPos] = !!curr.numSig[stateId];

      const int       sigNSbb   = ( ( scanInfo.nextSbbRight ? sbbFlags[scanInfo.nextSbbRight] : false ) || ( scanInfo.nextSbbBelow ? sbbFlags[scanInfo.nextSbbBelow] : false ) ? 1 : 0 );
      curr.refSbbCtxId[stateId] = stateId;
      const BinFracBits sbbBits = m_sbbFlagBits[sigNSbb];

      curr.sbbBits0[stateId] = sbbBits.intBits[0];
      curr.sbbBits1[stateId] = sbbBits.intBits[1];

      if( sigNSbb || ( ( scanInfo.nextSbbRight && scanInfo.nextSbbBelow ) ? sbbFlags[scanInfo.nextSbbBelow  + 1] : false ) )
      {
        const int         scanBeg   = scanInfo.scanIdx - scanInfo.sbbSize;
        const NbInfoOut*  nbOut     = m_nbInfo + scanBeg;
        const uint8_t*    absLevels = levels   + scanBeg;

        for( int id = 0; id < scanInfo.sbbSize; id++, nbOut++ )
        {
          const int idAddr = ( id << 2 ) + stateId;

          if( nbOut->num )
          {
            TCoeff sumAbs = 0, sumAbs1 = 0, sumNum = 0;
#define UPDATE(k) {TCoeff t=absLevels[nbOut->outPos[k]]; sumAbs+=t; sumAbs1+=std::min<TCoeff>(4+(t&1),t); sumNum+=!!t; }
            switch( nbOut->num )
            {
            default:
            case 5:
              UPDATE(4);
            case 4:
              UPDATE(3);
            case 3:
              UPDATE(2);
            case 2:
              UPDATE(1);
            case 1:
              UPDATE(0);
            }
#undef UPDATE
            curr.tpl[idAddr] = ( sumNum << 5 ) | sumAbs1;
            curr.sum[idAddr] = ( uint8_t ) std::min( 255, sumAbs );
          }
        }
      }
    }

    inline void updateAllLvls( const ScanInfo &scanInfo, const StateMem &curr )
    {
      uint8_t *levels0 = m_currSbbCtx[0].levels + scanInfo.scanIdx;
      uint8_t *levels1 = m_currSbbCtx[1].levels + scanInfo.scanIdx;
      uint8_t *levels2 = m_currSbbCtx[2].levels + scanInfo.scanIdx;
      uint8_t *levels3 = m_currSbbCtx[3].levels + scanInfo.scanIdx;

      const int regSize = 16;
      const int ctxSize = scanInfo.sbbSize << 2;

      const __m128i vshuf0 = _mm_setr_epi8(  0,  4,  8, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
      const __m128i vshuf1 = _mm_setr_epi8(  1,  5,  9, 13, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
      const __m128i vshuf2 = _mm_setr_epi8(  2,  6, 10, 14, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
      const __m128i vshuf3 = _mm_setr_epi8(  3,  7, 11, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );

      for( int i = 0, j = 0; i < ctxSize; i += regSize, j += 4 )
      {
        __m128i in  = _mm_loadu_si128( ( const __m128i* ) &curr.val[i] );

        _mm_storeu_si32( &levels0[j], _mm_shuffle_epi8( in, vshuf0 ) );
        _mm_storeu_si32( &levels1[j], _mm_shuffle_epi8( in, vshuf1 ) );
        _mm_storeu_si32( &levels2[j], _mm_shuffle_epi8( in, vshuf2 ) );
        _mm_storeu_si32( &levels3[j], _mm_shuffle_epi8( in, vshuf3 ) );
      }
    }

  private:
    const NbInfoOut*            m_nbInfo;
    BinFracBits                 m_sbbFlagBits[2];
    SbbCtx                      m_allSbbCtx  [8];
    SbbCtx*                     m_currSbbCtx;
    SbbCtx*                     m_prevSbbCtx;
    uint8_t                     m_memory[ 8 * ( MAX_TB_SIZEY * MAX_TB_SIZEY + MLS_GRP_NUM ) ];
  };

  template<X86_VEXT vext>
  class State
  {
    friend class CommonCtx<vext>;
  public:
    State( const RateEstimator& rateEst, CommonCtx<vext>& commonCtx, const int stateId )
      : m_stateId         ( stateId )
      , m_sigFracBitsArray( rateEst.sigFlagBits(stateId) )
      , m_gtxFracBitsArray( rateEst.gtxFracBits() )
      , m_commonCtx       ( commonCtx )
    {
    }

    static inline void updateStates( const ScanInfo &scanInfo, const Decisions &decisions, StateMem &prev, StateMem &curr )
    {
      int8_t s[4] = { 0 }, t[4] = { 0 }, l[4] = { 0 };

#if 1
      __m128i v254_4 = _mm_setr_epi16( 254, 254, 254, 254,  4,  4,  4,  4 );
      __m128i v01    = _mm_setr_epi16(   1,   1,   1,   1,  1,  1,  1,  1 );
      __m128i v032   = _mm_setr_epi8 (   0,   0,   0,   0, 32, 32, 32, 32, 0, 0, 0, 0, 0, 0, 0, 0 );
      __m128i vn1    = _mm_set1_epi8 (  -1 );

      static_assert( sizeof( curr.rdCost ) == sizeof( decisions.rdCost ), "Non-matching array size" );
      memcpy( curr.rdCost, decisions.rdCost, sizeof( decisions.rdCost ) );

      // in signalling, the coeffs are always max 16 bit!
      __m128i v = _mm_loadu_si64( decisions.absLevel );
      v = _mm_unpacklo_epi64( v, v );
      __m128i p = _mm_loadu_si32( decisions.prevId );
      _mm_storeu_si32( s, p ); // store previous state indexes
      p = _mm_shuffle_epi32( p, 0 ); 
      __m128i n2  = _mm_cmplt_epi8( p, vn1 );
      __m128i a_1 = _mm_and_si128( v, v01 );
      __m128i a_m = _mm_min_epi16( v, _mm_add_epi16( v254_4, a_1 ) );
      a_m = _mm_packs_epi16( a_m, vn1 );
      a_m = _mm_or_si128   ( a_m, _mm_sign_epi8( v032, a_m ) );
      a_m = _mm_andnot_si128( n2, a_m );
      _mm_storeu_si32( l, a_m ); // store abs value
      a_m = _mm_shuffle_epi32( a_m, 1 );
      _mm_storeu_si32( t, a_m ); // store store capped abs value
#else
      for( int i = 0; i < 4; ++i )
      {
        s[ i ]               = decisions[ i ].prevId;
        int min4_or_5        = std::min<TCoeff>( 4 + ( decisions[ i ].absLevel & 1 ), decisions[ i ].absLevel );
        t[ i ]               = decisions[ i ].prevId > -2 ? min4_or_5 : 0;
        t[ i ]              |= t[i] ? 32 : 0;
        l[ i ]               = decisions[ i ].prevId > -2 ? std::min( decisions[i].absLevel, 255 ) : 0;
        //all_above_minus_two &= decision[ i ].prevId > -2;
      }
#endif

      {
        const int ctxSize = 16 * 4;
        const int regSize = 16;

        __m128i vshuf     = _mm_loadu_si32 ( s );
                vshuf     = _mm_shuffle_epi32( vshuf, 0 );
        __m128i vshufmask = _mm_cmplt_epi8 ( vshuf, _mm_setzero_si128() );
        vshuf             = _mm_add_epi8   ( vshuf, _mm_setr_epi8( 0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12 ) );
        vshuf             = _mm_blendv_epi8( vshuf, _mm_set1_epi8( -1 ), vshufmask );

        for( int i = 0; i < ctxSize; i += regSize )
        {
          __m128i vtpl = _mm_loadu_si128( ( const __m128i* ) &prev.tpl[i] );
          vtpl = _mm_shuffle_epi8( vtpl, vshuf );
          _mm_storeu_si128( ( __m128i* ) &curr.tpl[i], vtpl );

          __m128i vval = _mm_loadu_si128( ( const __m128i* ) &prev.val[i] );
          vval = _mm_shuffle_epi8( vval, vshuf );
          _mm_storeu_si128( ( __m128i* ) &curr.val[i], vval );

          __m128i vsum = _mm_loadu_si128( ( const __m128i* ) &prev.sum[i] );
          vsum = _mm_shuffle_epi8( vsum, vshuf );
          _mm_storeu_si128( ( __m128i* ) &curr.sum[i], vsum );
        }

        __m128i numSig = _mm_loadu_si32( prev.numSig );
        numSig = _mm_shuffle_epi8( numSig, vshuf );
        __m128i lvls   = _mm_loadu_si32( l );
        lvls   = _mm_cmpgt_epi8( lvls, _mm_setzero_si128() );
        numSig = _mm_subs_epi8( numSig, lvls );
        _mm_storeu_si32( curr.numSig, numSig );

        __m128i rsc = _mm_loadu_si32( prev.refSbbCtxId );
        rsc         = _mm_shuffle_epi8( rsc, vshuf );
        rsc         = _mm_blendv_epi8( rsc, vshuf, vshuf );
        _mm_storeu_si32( curr.refSbbCtxId, rsc );

        vshuf = _mm_cvtepi8_epi32( vshuf );
        vshuf = _mm_shuffle_epi8( vshuf, _mm_setr_epi8( 0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12 ) );
        vshuf = _mm_slli_epi32( vshuf, 2 );
        vshuf = _mm_add_epi8( vshuf,
                              _mm_blendv_epi8( _mm_setr_epi8( 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3 ),
                                               _mm_setzero_si128(),
                                               vshuf ) );

        __m128i rrb = _mm_loadu_si128( ( const __m128i* ) prev.remRegBins );
        rrb = _mm_shuffle_epi8( rrb, vshuf );
        rrb = _mm_sub_epi32( rrb, _mm_blendv_epi8( _mm_set1_epi32( 1 ), _mm_setzero_si128(), vshuf ) );
        __m128i mlvl = _mm_loadu_si32( l );
        rrb = _mm_blendv_epi8( rrb, _mm_set1_epi32( curr.initRemRegBins ), vshuf );
        
        __m128i mbins = _mm_cvtepi8_epi32( mlvl );
        __m128i madd  = _mm_cmpeq_epi32( mbins, _mm_set1_epi32( 1 ) );
        __m128i mmore = _mm_and_si128( _mm_cmpgt_epi32( mbins, _mm_set1_epi32( 1 ) ), _mm_set1_epi32( 3 ) );
        madd = _mm_sub_epi32( madd, mmore );
        madd = _mm_blendv_epi8( madd, _mm_setzero_si128(), _mm_cmplt_epi32(rrb, _mm_set1_epi32(4)));
        rrb  = _mm_add_epi32( rrb, madd );
        _mm_storeu_si128( ( __m128i* ) curr.remRegBins, rrb );
        rrb = _mm_cmplt_epi32( rrb, _mm_set1_epi32( 4 ) );

        curr.anyRemRegBinsLt4 = !_mm_test_all_zeros( rrb, rrb );

        __m128i lvl1 = _mm_loadu_si32( l );

        if( scanInfo.currNbInfoSbb.numInv )
        {
          //auto adds8 = []( uint8_t a, uint8_t b )
          //{
          //  uint8_t c = a + b;
          //  if( c < a ) c = -1;
          //  return c;
          //};
          //
          //auto update_deps_scalar = [&]( int k )
          //{
          //  for( int i = 0; i < 4; i++ )
          //  {
          //    int addr = ( scanInfo.currNbInfoSbb.invInPos[k] << 2 ) + i;
          //    curr.sum[addr] = adds8( curr.sum[addr], decisions[i].absLevel );
          //  }
          //};

          auto update_deps_vec = [&]( int k )
          {
            int addr = scanInfo.currNbInfoSbb.invInPos[k] << 2;
            __m128i msum = _mm_loadu_si32( &curr.sum[addr] );
            msum = _mm_adds_epu8( msum, mlvl );
            _mm_storeu_si32( &curr.sum[addr], msum );
          };

          switch( scanInfo.currNbInfoSbb.numInv )
          {
          default:
          case 5:
            update_deps_vec( 4 );
          case 4:
            update_deps_vec( 3 );
          case 3:
            update_deps_vec( 2 );
          case 2:
            update_deps_vec( 1 );
          case 1:
            update_deps_vec( 0 );
          }
        }

        int addr = ( scanInfo.insidePos << 2 );
        _mm_storeu_si32( &curr.val[addr], lvl1 );
      }

      {
        __m128i tpl1 = _mm_loadu_si32( t );

        auto update_deps = [&]( int k )
        {
          int addr = scanInfo.currNbInfoSbb.invInPos[k] << 2;
          __m128i tpl = _mm_loadu_si32( &curr.tpl[addr] );
          tpl = _mm_add_epi8( tpl, tpl1 );
          _mm_storeu_si32( &curr.tpl[addr], tpl );
        };

        switch( scanInfo.currNbInfoSbb.numInv )
        {
        default:
        case 5:
          update_deps( 4 );
        case 4:
          update_deps( 3 );
        case 3:
          update_deps( 2 );
        case 2:
          update_deps( 1 );
        case 1:
          update_deps( 0 );
        }
      }

      {
        __m128i ones    = _mm_set1_epi32( 1 );
        __m128i tplAcc  = _mm_loadu_si128( ( __m128i * ) &curr.tpl[ ( scanInfo.nextInsidePos << 2 ) ] );
        tplAcc          = _mm_cvtepu8_epi32( tplAcc );

        __m128i sumAbs1 = _mm_and_si128 ( tplAcc, _mm_set1_epi32( 31 ) );
        __m128i sumNum  = _mm_srli_epi32( tplAcc, 5 );
        __m128i sumGt1  = _mm_sub_epi32 ( sumAbs1, sumNum );
        sumGt1  = _mm_min_epi32( sumGt1, _mm_set1_epi32( 4 ) );
        sumGt1  = _mm_add_epi32( _mm_set1_epi32( scanInfo.gtxCtxOffsetNext ), sumGt1 );

        sumAbs1 = _mm_add_epi32( sumAbs1, ones );
        sumAbs1 = _mm_srai_epi32( sumAbs1, 1 );
        sumAbs1 = _mm_min_epi32( sumAbs1, _mm_set1_epi32( 3 ) );

        sumAbs1 = _mm_add_epi32( _mm_set1_epi32( scanInfo.sigCtxOffsetNext ), sumAbs1 );
        sumAbs1 = _mm_packs_epi32( sumAbs1, sumAbs1 );
        sumAbs1 = _mm_packs_epi16( sumAbs1, sumAbs1 );
        _mm_storeu_si32( curr.ctx.sig, sumAbs1 );

        sumGt1  = _mm_packs_epi32( sumGt1, sumGt1 );
        sumGt1  = _mm_packs_epi16( sumGt1, sumGt1 );
        _mm_storeu_si32( curr.ctx.cff, sumGt1 );

        curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
      }
    }

    static inline void updateStatesEOS(const ScanInfo &scanInfo, const Decisions &decisions, StateMem& prev, const StateMem& skip, StateMem& curr, CommonCtx<vext> &commonCtx)
    {
      bool rem_reg_all_gte_4 = true;

      int8_t s[4] = { 0 }, l[4] = { 0 };

      for( int i = 0; i < 4; ++i )
      {
        s[i]              = decisions.prevId[i] >= 4 ? -2 : decisions.prevId[i];
        l[i]              = s[i] > -2 ? std::min<int>( decisions.absLevel[i], 254 + ( decisions.absLevel[i] & 1 ) ) : 0;
        curr.rdCost[i]    = decisions.rdCost[i];
      }

      {
        const int ctxSize = 16 * 4;
        const int regSize = 16;

        __m128i vshuf     = _mm_loadu_si32( s );
                vshuf     = _mm_shuffle_epi32( vshuf, 0 );
        __m128i vshufmask = _mm_cmplt_epi8 ( vshuf, _mm_setzero_si128() );
        vshuf             = _mm_add_epi8   ( vshuf, _mm_setr_epi8( 0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12 ) );
        vshuf             = _mm_blendv_epi8( vshuf, _mm_set1_epi8( -1 ), vshufmask );

        for( int i = 0; i < ctxSize; i += regSize )
        {
          __m128i vval = _mm_loadu_si128( ( const __m128i* ) &prev.val[i] );
          vval = _mm_shuffle_epi8( vval, vshuf );
          _mm_storeu_si128( ( __m128i* ) &curr.val[i], vval );
        }

        __m128i numSig = _mm_loadu_si32( prev.numSig );
        numSig = _mm_shuffle_epi8( numSig, vshuf );
        __m128i lvls   = _mm_loadu_si32( l );
        lvls   = _mm_cmpgt_epi8( lvls, _mm_setzero_si128() );
        numSig = _mm_subs_epi8( numSig, lvls );
        _mm_storeu_si32( curr.numSig, numSig );
      }

      {
        __m128i lvl1 = _mm_loadu_si32( l );
        int addr = ( scanInfo.insidePos << 2 );
        _mm_storeu_si32( &curr.val[addr], lvl1 );
      }

      commonCtx.updateAllLvls( scanInfo, curr );

      memset( curr.val, 0, sizeof( curr.val ) );
      memset( curr.tpl, 0, sizeof( curr.tpl ) );
      memset( curr.sum, 0, sizeof( curr.sum ) );

      for( int i = 0; i < 4; i++ )
      {
        int prevId = decisions.prevId[i];
        int level  = decisions.absLevel[i];

        if( prevId > -2 )
        {
          int remRegBins = 0;

          if( prevId  >= 4 )
          {
            CHECKD( level != 0, "cannot happen" );
            remRegBins = skip.remRegBins[prevId - 4];
          }
          else if( prevId >= 0 )
          {
            remRegBins = prev.remRegBins[prevId] - 1;
            if( remRegBins >= 4 )
            {
              remRegBins -= ( level < 2 ? level : 3 );
            }
          }
          else
          {
            remRegBins = curr.initRemRegBins;
            if( remRegBins >= 4 )
            {
              remRegBins -= ( level < 2 ? level : 3 );
            }
          }

          curr.remRegBins[i] = remRegBins;

          const int refId = prevId < 0 ? -1 : ( prevId < 4 ? prev.refSbbCtxId[prevId] : prevId - 4 );
          commonCtx.update( scanInfo, refId, i, curr );

          rem_reg_all_gte_4 &= remRegBins >= 4;
        }
      }

      curr.anyRemRegBinsLt4 = !rem_reg_all_gte_4;
      memset( curr.numSig, 0, sizeof( curr.numSig ) );

      {
        __m128i ones    = _mm_set1_epi32( 1 );
        __m128i tplAcc  = _mm_loadu_si128( ( __m128i * ) &curr.tpl[ ( scanInfo.nextInsidePos << 2 ) ] );
        tplAcc          = _mm_cvtepu8_epi32( tplAcc );

        __m128i sumAbs1 = _mm_and_si128 ( tplAcc, _mm_set1_epi32( 31 ) );
        __m128i sumNum  = _mm_srli_epi32( tplAcc, 5 );
        __m128i sumGt1  = _mm_sub_epi32 ( sumAbs1, sumNum );
        sumGt1  = _mm_min_epi32( sumGt1, _mm_set1_epi32( 4 ) );
        sumGt1  = _mm_add_epi32( _mm_set1_epi32( scanInfo.gtxCtxOffsetNext ), sumGt1 );

        sumAbs1 = _mm_add_epi32( sumAbs1, ones );
        sumAbs1 = _mm_srai_epi32( sumAbs1, 1 );
        sumAbs1 = _mm_min_epi32( sumAbs1, _mm_set1_epi32( 3 ) );

        sumAbs1 = _mm_add_epi32( _mm_set1_epi32( scanInfo.sigCtxOffsetNext ), sumAbs1 );
        sumAbs1 = _mm_packs_epi32( sumAbs1, sumAbs1 );
        sumAbs1 = _mm_packs_epi16( sumAbs1, sumAbs1 );
        _mm_storeu_si32( curr.ctx.sig, sumAbs1 );

        sumGt1  = _mm_packs_epi32( sumGt1, sumGt1 );
        sumGt1  = _mm_packs_epi16( sumGt1, sumGt1 );
        _mm_storeu_si32( curr.ctx.cff, sumGt1 );

        curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
      }
    }

    inline void init( StateMem &state )
    {
      state.rdCost [m_stateId] = rdCostInit;
      state.ctx.cff[m_stateId] =  0;
      state.ctx.sig[m_stateId] =  0;
      state.numSig [m_stateId] =  0;
      state.refSbbCtxId[m_stateId]
                               = -1;
      state.remRegBins[m_stateId]
                               =  4;
      state.cffBitsCtxOffset   =  0;
      m_goRicePar     = 0;
      m_goRiceZero    = 0;
    }

    void checkRdCosts( const ScanPosType spt, const PQData& pqDataA, const PQData& pqDataB, Decisions& decisions, int idxAZ, int idxB, const StateMem& state ) const
    {
      const int32_t*  goRiceTab = g_goRiceBits[m_goRicePar];
      int64_t         rdCostA   = state.rdCost[m_stateId] + pqDataA.deltaDist;
      int64_t         rdCostB   = state.rdCost[m_stateId] + pqDataB.deltaDist;
      int64_t         rdCostZ   = state.rdCost[m_stateId];

      if( state.remRegBins[m_stateId] >= 4 )
      {
        const CoeffFracBits &cffBits = m_gtxFracBitsArray[state.ctx.cff[m_stateId]];
        const BinFracBits    sigBits = m_sigFracBitsArray[state.ctx.sig[m_stateId]];

        if( pqDataA.absLevel < 4 )
          rdCostA += cffBits.bits[ pqDataA.absLevel ];
        else
        {
          const unsigned value = ( pqDataA.absLevel - 4 ) >> 1;
          rdCostA += cffBits.bits[ pqDataA.absLevel - ( value << 1 ) ] + goRiceTab[ std::min<unsigned>( value, RICEMAX - 1 ) ];
        }

        if( pqDataB.absLevel < 4 )
          rdCostB += cffBits.bits[ pqDataB.absLevel ];
        else
        {
          const unsigned value = ( pqDataB.absLevel - 4 ) >> 1;
          rdCostB += cffBits.bits[ pqDataB.absLevel - ( value << 1 ) ] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
        }

        if( spt == SCAN_ISCSBB )
        {
          rdCostA += sigBits.intBits[ 1 ];
          rdCostB += sigBits.intBits[ 1 ];
          rdCostZ += sigBits.intBits[ 0 ];
        }
        else if( spt == SCAN_SOCSBB )
        {
          rdCostA += state.sbbBits1[m_stateId] + sigBits.intBits[ 1 ];
          rdCostB += state.sbbBits1[m_stateId] + sigBits.intBits[ 1 ];
          rdCostZ += state.sbbBits1[m_stateId] + sigBits.intBits[ 0 ];
        }
        else if( state.numSig[m_stateId] )
        {
          rdCostA += sigBits.intBits[ 1 ];
          rdCostB += sigBits.intBits[ 1 ];
          rdCostZ += sigBits.intBits[ 0 ];
        }
        else
        {
          rdCostZ = rdCostInit;
        }
      }
      else
      {
        rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[ pqDataA.absLevel <= m_goRiceZero ? pqDataA.absLevel - 1 : std::min<int>( pqDataA.absLevel, RICEMAX - 1 ) ];
        rdCostB += ( 1 << SCALE_BITS ) + goRiceTab[ pqDataB.absLevel <= m_goRiceZero ? pqDataB.absLevel - 1 : std::min<int>( pqDataB.absLevel, RICEMAX - 1 ) ];
        rdCostZ += goRiceTab[ m_goRiceZero ];
      }

      if( rdCostA < rdCostZ && rdCostA < decisions.rdCost[idxAZ] )
      {
        decisions.rdCost  [idxAZ] = rdCostA;
        decisions.absLevel[idxAZ] = pqDataA.absLevel;
        decisions.prevId  [idxAZ] = m_stateId;
      }
      else if( rdCostZ < decisions.rdCost[idxAZ] )
      {
        decisions.rdCost  [idxAZ] = rdCostZ;
        decisions.absLevel[idxAZ] = 0;
        decisions.prevId  [idxAZ] = m_stateId;
      }

      if( rdCostB < decisions.rdCost[idxB] )
      {
        decisions.rdCost  [idxB] = rdCostB;
        decisions.absLevel[idxB] = pqDataB.absLevel;
        decisions.prevId  [idxB] = m_stateId;
      }
    }

    // has to be called as a first check, assumes no decision has been made yet
    static void checkAllRdCosts( const ScanPosType spt, State* states, const PQData* pqData, Decisions& decisions, const StateMem& state )
    {
      // State mapping
      // decision 0: either A from 0 (pq0), or B from 1 (pq2), or 0 from 0
      // decision 1: either A from 2 (pq3), or B from 3 (pq1), or 0 from 2
      // decision 2: either A from 1 (pq0), or B from 0 (pq2), or 0 from 1
      // decision 3: either A from 3 (pq3), or B from 2 (pq1), or 0 from 3

      __m128i mrd01 = _mm_loadu_si128( ( const __m128i* ) &state.rdCost[0] );
      __m128i mrd23 = _mm_loadu_si128( ( const __m128i* ) &state.rdCost[2] );

      //int64_t         rdCostA   = state.rdCost[m_stateId] + pqDataA.deltaDist;
      //int64_t         rdCostB   = state.rdCost[m_stateId] + pqDataB.deltaDist;
      //int64_t         rdCostZ   = state.rdCost[m_stateId];
      __m128i rdCostZ01 = _mm_unpacklo_epi64( mrd01, mrd23 );
      __m128i rdCostZ23 = _mm_unpackhi_epi64( mrd01, mrd23 );
      __m128i deltaDist = _mm_unpacklo_epi64( _mm_loadu_si64( &pqData[2].deltaDist ), _mm_loadu_si64( &pqData[1].deltaDist ) );
      __m128i rdCostB01 = _mm_add_epi64( rdCostZ23, deltaDist );
      __m128i rdCostB23 = _mm_add_epi64( rdCostZ01, deltaDist );
              deltaDist = _mm_unpacklo_epi64( _mm_loadu_si64( &pqData[0].deltaDist ), _mm_loadu_si64( &pqData[3].deltaDist ) );
      __m128i rdCostA01 = _mm_add_epi64( rdCostZ01, deltaDist );
      __m128i rdCostA23 = _mm_add_epi64( rdCostZ23, deltaDist );

      //const CoeffFracBits &cffBits = m_gtxFracBitsArray[state.ctx.cff[m_stateId]];
      //const BinFracBits    sigBits = m_sigFracBitsArray[state.ctx.sig[m_stateId]];
      //
      //rdCostA += cffBits.bits[ pqDataA.absLevel ];
      //rdCostB += cffBits.bits[ pqDataB.absLevel ];
      __m128i sgbts02   = _mm_unpacklo_epi64( _mm_loadu_si64( &states[0].m_sigFracBitsArray[state.ctx.sig[0]] ),
                                              _mm_loadu_si64( &states[2].m_sigFracBitsArray[state.ctx.sig[2]] ) );
      __m128i sgbts13   = _mm_unpacklo_epi64( _mm_loadu_si64( &states[1].m_sigFracBitsArray[state.ctx.sig[1]] ),
                                              _mm_loadu_si64( &states[3].m_sigFracBitsArray[state.ctx.sig[3]] ) );

      {
        __m128i sgbts02_0 = _mm_shuffle_epi32( sgbts02, 0 + ( 2 << 2 ) + ( 0 << 4 ) + ( 2 << 6 ) );
        __m128i sgbts02_1 = _mm_shuffle_epi32( sgbts02, 1 + ( 3 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );
        __m128i sgbts13_0 = _mm_shuffle_epi32( sgbts13, 0 + ( 2 << 2 ) + ( 0 << 4 ) + ( 2 << 6 ) );
        __m128i sgbts13_1 = _mm_shuffle_epi32( sgbts13, 1 + ( 3 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

        sgbts02 = _mm_unpacklo_epi64( sgbts02_0, sgbts02_1 );
        sgbts13 = _mm_unpacklo_epi64( sgbts13_0, sgbts13_1 );
      }

      {
        // coeff context is indepndent of state
        auto &base = states->m_gtxFracBitsArray;

        int32_t cffBitsArr[4] =
        {
          base[state.ctx.cff[1]].bits[pqData[2].absLevel],
          base[state.ctx.cff[3]].bits[pqData[1].absLevel],
          base[state.ctx.cff[0]].bits[pqData[2].absLevel],
          base[state.ctx.cff[2]].bits[pqData[1].absLevel],
        };

        __m128i cffBits = _mm_loadu_si128( ( const __m128i* ) cffBitsArr );
        __m128i add     = _mm_cvtepi32_epi64( cffBits );
        rdCostB01 = _mm_add_epi64( rdCostB01, add );
        add       = _mm_cvtepi32_epi64( _mm_unpackhi_epi64( cffBits, cffBits ) );
        rdCostB23 = _mm_add_epi64( rdCostB23, add );
      }

      {
        // coeff context is indepndent of state
        auto &base = states->m_gtxFracBitsArray;

        int32_t cffBitsArr[4] =
        {
          base[state.ctx.cff[0]].bits[pqData[0].absLevel],
          base[state.ctx.cff[2]].bits[pqData[3].absLevel],
          base[state.ctx.cff[1]].bits[pqData[0].absLevel],
          base[state.ctx.cff[3]].bits[pqData[3].absLevel],
        };

        __m128i cffBits = _mm_loadu_si128( ( const __m128i* ) cffBitsArr );
        __m128i add     = _mm_cvtepi32_epi64( cffBits );
        rdCostA01 = _mm_add_epi64( rdCostA01, add );
        add       = _mm_cvtepi32_epi64( _mm_unpackhi_epi64( cffBits, cffBits ) );
        rdCostA23 = _mm_add_epi64( rdCostA23, add );
      }

      if( spt == SCAN_ISCSBB )
      {
        //  rdCostZ += sigBits.intBits[ 0 ];
        rdCostZ01 = _mm_add_epi64( rdCostZ01, _mm_cvtepi32_epi64( sgbts02 ) );
        rdCostZ23 = _mm_add_epi64( rdCostZ23, _mm_cvtepi32_epi64( sgbts13 ) );

        sgbts02   = _mm_unpackhi_epi64( sgbts02, sgbts02 );
        sgbts13   = _mm_unpackhi_epi64( sgbts13, sgbts13 );

        //  rdCostB += sigBits.intBits[ 1 ];
        rdCostB01 = _mm_add_epi64( rdCostB01, _mm_cvtepi32_epi64( sgbts13 ) );
        rdCostB23 = _mm_add_epi64( rdCostB23, _mm_cvtepi32_epi64( sgbts02 ) );

        //  rdCostA += sigBits.intBits[ 1 ];
        rdCostA01 = _mm_add_epi64( rdCostA01, _mm_cvtepi32_epi64( sgbts02 ) );
        rdCostA23 = _mm_add_epi64( rdCostA23, _mm_cvtepi32_epi64( sgbts13 ) );
      }
      else if( spt == SCAN_SOCSBB )
      {
        //  rdCostA += m_sbbFracBits.intBits[ 1 ] + sigBits.intBits[ 1 ];
        //  rdCostB += m_sbbFracBits.intBits[ 1 ] + sigBits.intBits[ 1 ];
        //  rdCostZ += m_sbbFracBits.intBits[ 1 ] + sigBits.intBits[ 0 ];
        __m128i sbbBits = _mm_loadu_si128( ( const __m128i* ) state.sbbBits1 );
        sbbBits = _mm_shuffle_epi32( sbbBits, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

        rdCostZ01 = _mm_add_epi64( rdCostZ01, _mm_cvtepi32_epi64( sgbts02 ) );
        rdCostZ23 = _mm_add_epi64( rdCostZ23, _mm_cvtepi32_epi64( sgbts13 ) );

        __m128i add = _mm_cvtepi32_epi64( sbbBits );
        rdCostB23 = _mm_add_epi64( rdCostB23, add );
        rdCostA01 = _mm_add_epi64( rdCostA01, add );
        rdCostZ01 = _mm_add_epi64( rdCostZ01, add );
        add = _mm_cvtepi32_epi64( _mm_unpackhi_epi64( sbbBits, sbbBits ) );
        rdCostB01 = _mm_add_epi64( rdCostB01, add );
        rdCostA23 = _mm_add_epi64( rdCostA23, add );
        rdCostZ23 = _mm_add_epi64( rdCostZ23, add );

        sgbts02   = _mm_unpackhi_epi64( sgbts02, sgbts02 );
        sgbts13   = _mm_unpackhi_epi64( sgbts13, sgbts13 );
        rdCostB01 = _mm_add_epi64( rdCostB01, _mm_cvtepi32_epi64( sgbts13 ) );
        rdCostB23 = _mm_add_epi64( rdCostB23, _mm_cvtepi32_epi64( sgbts02 ) );

        rdCostA01 = _mm_add_epi64( rdCostA01, _mm_cvtepi32_epi64( sgbts02 ) );
        rdCostA23 = _mm_add_epi64( rdCostA23, _mm_cvtepi32_epi64( sgbts13 ) );
      }
      else
      {
        //else if( state.numSig[m_stateId] )
        //{
        //  rdCostA += sigBits.intBits[ 1 ];
        //  rdCostB += sigBits.intBits[ 1 ];
        //  rdCostZ += sigBits.intBits[ 0 ];
        //}
        //else
        //{
        //  rdCostZ = decisionA.rdCost;
        //}

        __m128i numSig = _mm_loadu_si32( state.numSig );

        rdCostZ01 = _mm_add_epi64(  rdCostZ01, _mm_cvtepi32_epi64( sgbts02 ) );
        rdCostZ23 = _mm_add_epi64( rdCostZ23, _mm_cvtepi32_epi64( sgbts13 ) );

        __m128i mask13 = _mm_shuffle_epi8( numSig, _mm_setr_epi8( 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3 ) );
        mask13    = _mm_cmpgt_epi8( mask13, _mm_setzero_si128() );
        __m128i mask02 = _mm_shuffle_epi8( numSig, _mm_setr_epi8( 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2 ) );
        mask02    = _mm_cmpgt_epi8( mask02, _mm_setzero_si128() );

        sgbts02   = _mm_unpackhi_epi64( sgbts02, sgbts02 );
        sgbts13   = _mm_unpackhi_epi64( sgbts13, sgbts13 );

        rdCostB01 = _mm_add_epi64( rdCostB01, _mm_and_si128( mask13, _mm_cvtepi32_epi64( sgbts13 ) ) );
        rdCostB23 = _mm_add_epi64( rdCostB23, _mm_and_si128( mask02, _mm_cvtepi32_epi64( sgbts02 ) ) );

        rdCostA01 = _mm_add_epi64( rdCostA01, _mm_and_si128( mask02, _mm_cvtepi32_epi64( sgbts02 ) ) );
        rdCostA23 = _mm_add_epi64( rdCostA23, _mm_and_si128( mask13, _mm_cvtepi32_epi64( sgbts13 ) ) );

        __m128i rdMax = _mm_loadu_si64( &rdCostInit );
        rdMax = _mm_unpacklo_epi64( rdMax, rdMax );

        rdCostZ01 = _mm_blendv_epi8( rdMax, rdCostZ01, mask02 );
        rdCostZ23 = _mm_blendv_epi8( rdMax, rdCostZ23, mask13 );
      }

      // decision 0: either A from 0 (pq0), or B from 1 (pq2), or 0 from 0
      // decision 1: either A from 2 (pq3), or B from 3 (pq1), or 0 from 2
      // decision 2: either A from 1 (pq0), or B from 0 (pq2), or 0 from 1
      // decision 3: either A from 3 (pq3), or B from 2 (pq1), or 0 from 3
      // Z0, or A0, or B0
      // Z1, or A1, or B1
      // B2, or Z2, or A2
      // B3, or Z3, or A3

      __m128i rdBest01 = rdCostZ01;
      __m128i rdBest23 = rdCostB23;

      __m128i valBest = _mm_setr_epi32(                  0,                  0, pqData[2].absLevel, pqData[1].absLevel );
      __m128i valCand = _mm_setr_epi32( pqData[0].absLevel, pqData[3].absLevel,                  0,                  0 );

      __m128i idxBest = _mm_setr_epi32( 0, 2, 0, 2 );
      __m128i idxCand = _mm_setr_epi32( 0, 2, 1, 3 );

      __m128i chng01 = _my_cmpgt_epi64( rdBest01, rdCostA01 );
      __m128i chng23 = _my_cmpgt_epi64( rdBest23, rdCostZ23 );
      __m128i chng   = _mm_blend_epi16( chng01, chng23, ( 3 << 2 ) + ( 3 << 6 ) ); // 00110011
      chng           = _mm_shuffle_epi32( chng, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

      rdBest01 = _mm_blendv_epi8( rdBest01, rdCostA01, chng01 );
      rdBest23 = _mm_blendv_epi8( rdBest23, rdCostZ23, chng23 );

      valBest = _mm_blendv_epi8( valBest, valCand, chng );
      idxBest = _mm_blendv_epi8( idxBest, idxCand, chng );

      
      valCand = _mm_setr_epi32( pqData[2].absLevel, pqData[1].absLevel, pqData[0].absLevel, pqData[3].absLevel );
      idxCand = _mm_setr_epi32( 1, 3, 1, 3 );

      chng01 = _my_cmpgt_epi64( rdBest01, rdCostB01 );
      chng23 = _my_cmpgt_epi64( rdBest23, rdCostA23 );
      chng   = _mm_blend_epi16( chng01, chng23, ( 3 << 2 ) + ( 3 << 6 ) ); // 00110011
      chng   = _mm_shuffle_epi32( chng, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

      rdBest01 = _mm_blendv_epi8( rdBest01, rdCostB01, chng01 );
      rdBest23 = _mm_blendv_epi8( rdBest23, rdCostA23, chng23 );

      valBest = _mm_blendv_epi8( valBest, valCand, chng );
      idxBest = _mm_blendv_epi8( idxBest, idxCand, chng );


      valBest = _mm_packs_epi32( valBest, _mm_setzero_si128() );
      idxBest = _mm_packs_epi32( idxBest, _mm_setzero_si128() );
      idxBest = _mm_packs_epi16( idxBest, _mm_setzero_si128() );


      _mm_storeu_si128( ( __m128i* ) &decisions.rdCost[0], rdBest01 );
      _mm_storeu_si128( ( __m128i* ) &decisions.rdCost[2], rdBest23 );

      _mm_storeu_si64( decisions.absLevel, valBest );
      _mm_storeu_si32( decisions.prevId,   idxBest );
    }

    void checkRdCostsOdd1( const ScanPosType spt, const PQData& pqDataA, Decisions& decisions, int idxA, int idxZ, const StateMem& state ) const
    {
      CHECKD( pqDataA.absLevel != 1, "" );

      int64_t         rdCostA   = state.rdCost[m_stateId] + pqDataA.deltaDist;
      int64_t         rdCostZ   = state.rdCost[m_stateId];

      if( state.remRegBins[m_stateId] >= 4 )
      {
        const BinFracBits sigBits = m_sigFracBitsArray[state.ctx.sig[m_stateId]];

        rdCostA += m_gtxFracBitsArray[state.ctx.cff[m_stateId]].bits[1];

        if( spt == SCAN_ISCSBB )
        {
          rdCostA += sigBits.intBits[ 1 ];
          rdCostZ += sigBits.intBits[ 0 ];
        }
        else if( spt == SCAN_SOCSBB )
        {
          rdCostA += state.sbbBits1[m_stateId] + sigBits.intBits[ 1 ];
          rdCostZ += state.sbbBits1[m_stateId] + sigBits.intBits[ 0 ];
        }
        else if( state.numSig[m_stateId] )
        {
          rdCostA += sigBits.intBits[ 1 ];
          rdCostZ += sigBits.intBits[ 0 ];
        }
        else
        {
          rdCostZ = rdCostInit;
        }
      }
      else
      {
        const int32_t*  goRiceTab = g_goRiceBits[m_goRicePar];

        rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[0];
        rdCostZ += goRiceTab[m_goRiceZero];
      }

      if( rdCostA < decisions.rdCost[idxA] )
      {
        decisions.rdCost  [idxA] = rdCostA;
        decisions.absLevel[idxA] = pqDataA.absLevel;
        decisions.prevId  [idxA] = m_stateId;
      }

      if( rdCostZ < decisions.rdCost[idxZ] )
      {
        decisions.rdCost  [idxZ] = rdCostZ;
        decisions.absLevel[idxZ] = 0;
        decisions.prevId  [idxZ] = m_stateId;
      }
    }

    // has to be called as a first check, assumes no decision has been made yet!!!
    static void checkAllRdCostsOdd1( const ScanPosType spt, State* states, const PQData* pqData, Decisions& decisions, const StateMem& state )
    {
      // State mapping
      // decision 0: either 1 from 1 (pqData[2]), or 0 from 0
      // decision 1: either 1 from 3 (pqData[1]), or 0 from 2
      // decision 2: either 1 from 0 (pqData[2]), or 0 from 1
      // decision 3: either 1 from 2 (pqData[1]), or 0 from 3

      __m128i mrd01 = _mm_loadu_si128( ( const __m128i* ) &state.rdCost[0] );
      __m128i mrd23 = _mm_loadu_si128( ( const __m128i* ) &state.rdCost[2] );

      //int64_t         rdCostA   = state.rdCost[m_stateId] + pqDataA.deltaDist; // done
      //int64_t         rdCostZ   = state.rdCost[m_stateId]; // done
      __m128i rdCostZ01 = _mm_unpacklo_epi64( mrd01, mrd23 );
      __m128i rdCostZ23 = _mm_unpackhi_epi64( mrd01, mrd23 );
      __m128i deltaDist = _mm_unpacklo_epi64( _mm_loadu_si64( &pqData[2].deltaDist ), _mm_loadu_si64( &pqData[1].deltaDist ) );
      __m128i rdCostA01 = _mm_add_epi64( rdCostZ23, deltaDist );
      __m128i rdCostA23 = _mm_add_epi64( rdCostZ01, deltaDist );

      //const BinFracBits sigBits = m_sigFracBitsArray[state.ctx.sig[m_stateId]];
      //
      //rdCostA += m_gtxFracBitsArray[state.ctx.cff[m_stateId]].bits[1]; // done
      //
      __m128i sgbts02   = _mm_unpacklo_epi64( _mm_loadu_si64( &states[0].m_sigFracBitsArray[state.ctx.sig[0]] ),
                                              _mm_loadu_si64( &states[2].m_sigFracBitsArray[state.ctx.sig[2]] ) );
      __m128i sgbts13   = _mm_unpacklo_epi64( _mm_loadu_si64( &states[1].m_sigFracBitsArray[state.ctx.sig[1]] ),
                                              _mm_loadu_si64( &states[3].m_sigFracBitsArray[state.ctx.sig[3]] ) );

      {
        __m128i sgbts02_0 = _mm_shuffle_epi32( sgbts02, 0 + ( 2 << 2 ) + ( 0 << 4 ) + ( 2 << 6 ) );
        __m128i sgbts02_1 = _mm_shuffle_epi32( sgbts02, 1 + ( 3 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );
        __m128i sgbts13_0 = _mm_shuffle_epi32( sgbts13, 0 + ( 2 << 2 ) + ( 0 << 4 ) + ( 2 << 6 ) );
        __m128i sgbts13_1 = _mm_shuffle_epi32( sgbts13, 1 + ( 3 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

        sgbts02 = _mm_unpacklo_epi64( sgbts02_0, sgbts02_1 );
        sgbts13 = _mm_unpacklo_epi64( sgbts13_0, sgbts13_1 );
      }

      {
#if USE_AVX2
        __m128i cffidx = _mm_cvtepi8_epi32( _mm_loadu_si32( &state.ctx.cff ) );
        cffidx = _mm_shuffle_epi32( cffidx, ( 1 << 0 ) + ( 3 << 2 ) + ( 0 << 4 ) + ( 2 << 6 ) );
        cffidx = _mm_sub_epi8( cffidx, _mm_set1_epi32( state.cffBitsCtxOffset ) );
        __m256i cffBits256 = _mm256_loadu_si256( ( const __m256i* ) &state.cffBits1[state.cffBitsCtxOffset] );
        cffBits256 = _mm256_permutevar8x32_epi32( cffBits256, _mm256_castsi128_si256( cffidx ) );
        __m128i cffBits = _mm256_castsi256_si128( cffBits256 );
#else
        __m128i cffBits;
        __m128i bits0123 = _mm_loadu_si128( ( const __m128i* ) &state.cffBits1[state.cffBitsCtxOffset + 0] );
        __m128i bits4    = _mm_loadu_si32 (                    &state.cffBits1[state.cffBitsCtxOffset + 4] );
        __m128i cfCtxIdx = _mm_loadu_si32 (                    &state.ctx.cff );
        cfCtxIdx = _mm_cvtepi8_epi32( cfCtxIdx );
        cfCtxIdx = _mm_sub_epi8( cfCtxIdx, _mm_set1_epi32(      state.cffBitsCtxOffset ) );
        cfCtxIdx = _mm_or_si128( cfCtxIdx, _mm_slli_si128( cfCtxIdx, 1 ) );
        cfCtxIdx = _mm_or_si128( cfCtxIdx, _mm_slli_si128( cfCtxIdx, 2 ) );
        cfCtxIdx = _mm_slli_epi32( cfCtxIdx, 2 );
        cfCtxIdx = _mm_add_epi8( cfCtxIdx, _mm_setr_epi8( 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3 ) );
        cffBits  = _mm_shuffle_epi8( bits4, _mm_sub_epi8( cfCtxIdx, _mm_set1_epi8( 16 ) ) );
        cfCtxIdx = _mm_or_si128( cfCtxIdx, _mm_cmpgt_epi8( cfCtxIdx, _mm_set1_epi8( 15 ) ) );
        cffBits  = _mm_or_si128( cffBits, _mm_shuffle_epi8( bits0123, cfCtxIdx ) );
        cffBits  = _mm_shuffle_epi32( cffBits, ( 1 << 0 ) + ( 3 << 2 ) +( 0 << 4 ) + ( 2 << 6 ) );
#endif
        rdCostA01 = _mm_add_epi64( rdCostA01, _mm_cvtepi32_epi64( cffBits ) );
        rdCostA23 = _mm_add_epi64( rdCostA23, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( cffBits, cffBits ) ) );
      }

      if( spt == SCAN_ISCSBB )
      {
        //  rdCostZ += sigBits.intBits[ 0 ]; // done
        rdCostZ01 = _mm_add_epi64( rdCostZ01, _mm_cvtepi32_epi64( sgbts02 ) );
        rdCostZ23 = _mm_add_epi64( rdCostZ23, _mm_cvtepi32_epi64( sgbts13 ) );

        sgbts02   = _mm_unpackhi_epi64( sgbts02, sgbts02 );
        sgbts13   = _mm_unpackhi_epi64( sgbts13, sgbts13 );

        //  rdCostA += sigBits.intBits[ 1 ]; // done
        rdCostA01 = _mm_add_epi64( rdCostA01, _mm_cvtepi32_epi64( sgbts13 ) );
        rdCostA23 = _mm_add_epi64( rdCostA23, _mm_cvtepi32_epi64( sgbts02 ) );
      }
      else if( spt == SCAN_SOCSBB )
      {
        //  rdCostZ += m_sbbFracBits.intBits[ 1 ] + sigBits.intBits[ 0 ]; // done
        //  rdCostA += m_sbbFracBits.intBits[ 1 ] + sigBits.intBits[ 1 ]; // dome
        __m128i sbbBits = _mm_loadu_si128( ( const __m128i* ) state.sbbBits1 );
        sbbBits = _mm_shuffle_epi32( sbbBits, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

        rdCostZ01 = _mm_add_epi64( rdCostZ01, _mm_cvtepi32_epi64( sgbts02 ) );
        rdCostZ23 = _mm_add_epi64( rdCostZ23, _mm_cvtepi32_epi64( sgbts13 ) );
        
        __m128i add = _mm_cvtepi32_epi64( sbbBits );
        rdCostA23 = _mm_add_epi64( rdCostA23, add );
        rdCostZ01 = _mm_add_epi64( rdCostZ01, add );
        add = _mm_cvtepi32_epi64( _mm_unpackhi_epi64( sbbBits, sbbBits ) );
        rdCostA01 = _mm_add_epi64( rdCostA01, add );
        rdCostZ23 = _mm_add_epi64( rdCostZ23, add );

        sgbts02   = _mm_unpackhi_epi64( sgbts02, sgbts02 );
        sgbts13   = _mm_unpackhi_epi64( sgbts13, sgbts13 );
        rdCostA01 = _mm_add_epi64( rdCostA01, _mm_cvtepi32_epi64( sgbts13 ) );
        rdCostA23 = _mm_add_epi64( rdCostA23, _mm_cvtepi32_epi64( sgbts02 ) );
      }
      else
      {
        //else if( m_numSigSbb )
        //{
        //  rdCostA += sigBits.intBits[ 1 ]; // done
        //  rdCostZ += sigBits.intBits[ 0 ]; // done
        //}
        //else
        //{
        //  rdCostZ = decisionZ.rdCost; // done
        //}

        __m128i numSig = _mm_loadu_si32( state.numSig );

        rdCostZ01 = _mm_add_epi64(  rdCostZ01, _mm_cvtepi32_epi64( sgbts02 ) );
        rdCostZ23 = _mm_add_epi64( rdCostZ23, _mm_cvtepi32_epi64( sgbts13 ) );

        __m128i mask01 = _mm_shuffle_epi8( numSig, _mm_setr_epi8( 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3 ) );
        mask01    = _mm_cmpgt_epi8( mask01, _mm_setzero_si128() );
        __m128i mask23 = _mm_shuffle_epi8( numSig, _mm_setr_epi8( 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2 ) );
        mask23    = _mm_cmpgt_epi8( mask23, _mm_setzero_si128() );
        sgbts02   = _mm_unpackhi_epi64( sgbts02, sgbts02 );
        sgbts13   = _mm_unpackhi_epi64( sgbts13, sgbts13 );
        rdCostA01 = _mm_add_epi64( rdCostA01, _mm_and_si128( mask01, _mm_cvtepi32_epi64( sgbts13 ) ) );
        rdCostA23 = _mm_add_epi64( rdCostA23, _mm_and_si128( mask23, _mm_cvtepi32_epi64( sgbts02 ) ) );

        __m128i rdMax = _mm_loadu_si64( &rdCostInit );
        rdMax = _mm_unpacklo_epi64( rdMax, rdMax );

        rdCostZ01 = _mm_blendv_epi8( rdMax, rdCostZ01, mask23 );
        rdCostZ23 = _mm_blendv_epi8( rdMax, rdCostZ23, mask01 );
      }

      //// decision 0: either 1 from 1 (pqData[2]), or 0 from 0
      //// decision 1: either 1 from 3 (pqData[1]), or 0 from 2
      //// decision 2: either 1 from 0 (pqData[2]), or 0 from 1
      //// decision 3: either 1 from 2 (pqData[1]), or 0 from 3

      // d0: Z0, or A0
      // d1: Z1, or A1
      // d2: A2, or Z2
      // d3: A3, or Z3

      __m128i rdBest01 = rdCostZ01;
      __m128i rdBest23 = rdCostA23;

      __m128i valBest = _mm_setr_epi32( 0, 0, 1, 1 );
      __m128i valCand = _mm_setr_epi32( 1, 1, 0, 0 );

      __m128i idxBest = _mm_setr_epi32( 0, 2, 0, 2 );
      __m128i idxCand = _mm_setr_epi32( 1, 3, 1, 3 );

      __m128i chng01 = _my_cmpgt_epi64( rdBest01, rdCostA01 );
      __m128i chng23 = _my_cmpgt_epi64( rdBest23, rdCostZ23 );
      __m128i chng   = _mm_blend_epi16( chng01, chng23, ( 3 << 2 ) + ( 3 << 6 ) ); // 00110011
              chng   = _mm_shuffle_epi32( chng, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

      rdBest01 = _mm_blendv_epi8( rdBest01, rdCostA01, chng01 );
      rdBest23 = _mm_blendv_epi8( rdBest23, rdCostZ23, chng23 );

      _mm_storeu_si128( ( __m128i* ) &decisions.rdCost[0], rdBest01 );
      _mm_storeu_si128( ( __m128i* ) &decisions.rdCost[2], rdBest23 );

      valBest = _mm_packs_epi32( _mm_blendv_epi8( valBest, valCand, chng ), _mm_setzero_si128() );
      idxBest = _mm_packs_epi32( _mm_blendv_epi8( idxBest, idxCand, chng ), _mm_setzero_si128() );
      idxBest = _mm_packs_epi16( idxBest, _mm_setzero_si128() );

      _mm_storeu_si64( decisions.absLevel, valBest );
      _mm_storeu_si32( decisions.prevId,   idxBest );
    }

    inline void checkRdCostStart(int32_t lastOffset, const PQData &pqData, Decisions &decisions, int idx ) const
    {
      const CoeffFracBits &cffBits = m_gtxFracBitsArray[0];

      int64_t rdCost = pqData.deltaDist + lastOffset;
      if (pqData.absLevel < 4)
      {
        rdCost += cffBits.bits[pqData.absLevel];
      }
      else
      {
        const unsigned value = (pqData.absLevel - 4) >> 1;
        rdCost += cffBits.bits[pqData.absLevel - (value << 1)] + g_goRiceBits[0][value < RICEMAX ? value : RICEMAX-1];
      }

      if( rdCost < decisions.rdCost[idx] )
      {
        decisions.rdCost  [idx] = rdCost;
        decisions.absLevel[idx] = pqData.absLevel;
        decisions.prevId  [idx] = -1;
      }
    }

    inline void checkRdCostSkipSbb(Decisions &decisions, int idx, const StateMem& state) const
    {
      int64_t rdCost = state.rdCost[m_stateId] + state.sbbBits0[m_stateId];
      if( rdCost < decisions.rdCost[idx] )
      {
        decisions.rdCost  [idx] = rdCost;
        decisions.absLevel[idx] = 0;
        decisions.prevId  [idx] = 4 | m_stateId;
      }
    }

    inline void checkRdCostSkipSbbZeroOut(Decisions &decisions, int idx, const StateMem& state) const
    {
      int64_t rdCost          = state.rdCost[m_stateId] + state.sbbBits0[m_stateId];
      decisions.rdCost  [idx] = rdCost;
      decisions.absLevel[idx] = 0;
      decisions.prevId  [idx] = 4 | m_stateId;
    }

    inline void setRiceParam( const ScanInfo& scanInfo, const StateMem& state, bool ge4 )
    {
      if( state.remRegBins[m_stateId] < 4 || ge4 )
      {
        const int addr  = ( scanInfo.insidePos << 2 ) + m_stateId;
        TCoeff  sumAbs  = state.sum[addr];
        int sumSub      = state.remRegBins[m_stateId] < 4 ? 0 : 4 * 5;
        int sumAll      = std::max( std::min( 31, ( int ) sumAbs - sumSub ), 0 );
        m_goRicePar     = g_auiGoRiceParsCoeff[sumAll];

        if( state.remRegBins[m_stateId] < 4 )
        {
          m_goRiceZero  = g_auiGoRicePosCoeff0( m_stateId, m_goRicePar );
        }
      }
    }

  private:

    int8_t                    m_goRicePar;
    int8_t                    m_goRiceZero;
    const int8_t              m_stateId;
    const BinFracBits*const   m_sigFracBitsArray;
    const CoeffFracBits*const m_gtxFracBitsArray;
    CommonCtx<vext>&          m_commonCtx;
  };

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q                                                              =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/
  template<X86_VEXT vext>
  class DepQuantSimd : private RateEstimator, public DepQuantImpl
  {
  public:
    const Decisions startDec[2] =
    { 
      Decisions
      {
        { rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2 },
        { -1, -1, -1, -1 },
        { -2, -2, -2, -2 },
      }, 
      Decisions
      {
        { rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2 },
        { 0, 0, 0, 0 },
        { 4, 5, 6, 7 },
      }
    };

#define TINIT(x) {*this,m_commonCtx,x}
    DepQuantSimd()
      : RateEstimator ()
      , m_commonCtx   ()
      , m_allStates   {TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3)}
      , m_currStates  (  m_allStates      )
      , m_prevStates  (  m_currStates + 4 )
      , m_skipStates  (  m_prevStates + 4 )
    {
      m_scansRom.init();

      for( int t = 0; t < ( MAX_TB_SIZEY * MAX_TB_SIZEY ); t++ )
      {
        memcpy( m_trellis[t], startDec, sizeof( startDec ) );
      }
    }
#undef TINIT

    ~DepQuantSimd()
    {
    }

    void init( int dqTrVal )
    {
      m_quant.init( dqTrVal );
    }

    void quant( TransformUnit &tu, const CCoeffBuf &srcCoeff, const ComponentID compID, const QpParam &cQP, const double lambda, const Ctx &ctx, TCoeff &absSum, bool enableScalingLists, int *quantCoeff )
    {
      //===== reset / pre-init =====
      const TUParameters& tuPars  = *m_scansRom.getTUPars( tu.blocks[compID], compID );
      m_quant.initQuantBlock    ( tu, compID, cQP, lambda );
      TCoeffSig*    qCoeff      = tu.getCoeffs( compID ).buf;
      const TCoeff* tCoeff      = srcCoeff.buf;
      const int     numCoeff    = tu.blocks[compID].area();
      ::memset( qCoeff, 0x00, numCoeff * sizeof( TCoeffSig ) );
      absSum                    = 0;

      const CompArea& area      = tu.blocks[ compID ];
      const uint32_t  width     = area.width;
      const uint32_t  height    = area.height;
      const uint32_t  lfnstIdx  = tu.cu->lfnstIdx;
      //===== scaling matrix ====
      //const int         qpDQ = cQP.Qp + 1;
      //const int         qpPer = qpDQ / 6;
      //const int         qpRem = qpDQ - 6 * qpPer;

      //TCoeff thresTmp = thres;
      bool zeroOut = false;
      bool zeroOutforThres = false;
      int effWidth = tuPars.m_width, effHeight = tuPars.m_height;
      if( ( tu.mtsIdx[compID] > MTS_SKIP || ( tu.cs->sps->MTS && tu.cu->sbtInfo != 0 && tuPars.m_height <= 32 && tuPars.m_width <= 32 ) ) && compID == COMP_Y )
      {
        effHeight = ( tuPars.m_height == 32 ) ? 16 : tuPars.m_height;
        effWidth  = ( tuPars.m_width  == 32 ) ? 16 : tuPars.m_width;
        zeroOut   = ( effHeight < tuPars.m_height || effWidth < tuPars.m_width );
      }
      zeroOutforThres = zeroOut || ( 32 < tuPars.m_height || 32 < tuPars.m_width );
      //===== find first test position =====
      int firstTestPos = std::min<int>( tuPars.m_width, JVET_C0024_ZERO_OUT_TH ) * std::min<int>( tuPars.m_height, JVET_C0024_ZERO_OUT_TH ) - 1;
      if( lfnstIdx > 0 && tu.mtsIdx[compID] != MTS_SKIP && width >= 4 && height >= 4 )
      {
        firstTestPos = ( ( width == 4 && height == 4 ) || ( width == 8 && height == 8 ) )  ? 7 : 15 ;
      }

      const TCoeff defaultQuantisationCoefficient = (TCoeff)m_quant.getQScale();
      const TCoeff thres = m_quant.getLastThreshold();
      const int zeroOutWidth  = ( tuPars.m_width  == 32 && zeroOut ) ? 16 : 32;
      const int zeroOutHeight = ( tuPars.m_height == 32 && zeroOut ) ? 16 : 32;

      if( enableScalingLists )
      {
        for( ; firstTestPos >= 0; firstTestPos-- )
        {
          if( zeroOutforThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth || tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) ) continue;

          const TCoeff thresTmp = TCoeff( thres / ( 4 * quantCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) );

          if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > thresTmp ) break;
        }
      }
      else
      {
        const TCoeff defaultTh = TCoeff( thres / ( defaultQuantisationCoefficient << 2 ) );

#if ENABLE_SIMD_OPT_QUANT && defined( TARGET_SIMD_X86 )
        // if more than one 4x4 coding subblock is available, use SIMD to find first subblock with coefficient larger than threshold
        if( firstTestPos >= 16 && tuPars.m_log2SbbWidth == 2 && tuPars.m_log2SbbHeight == 2 && read_x86_extension_flags() > x86_simd::SCALAR )
        {
          const int sbbSize = tuPars.m_sbbSize;
          // move the pointer to the beginning of the current subblock
          firstTestPos -= ( sbbSize - 1 );

          const __m128i xdfTh = _mm_set1_epi32( defaultTh );

          // for each subblock
          for( ; firstTestPos >= 0; firstTestPos -= sbbSize )
          {
            // skip zeroed out blocks
            // for 64-point transformation the coding order takes care of that
            if( zeroOutforThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth || tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) )
            {
              continue;
            }

            // read first line of the subblock and check for coefficients larger than the threshold
            // assumming the subblocks are dense 4x4 blocks in raster scan order with the stride of tuPars.m_width
            int pos = tuPars.m_scanId2BlkPos[firstTestPos].idx;
            __m128i xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &tCoeff[pos] ) );
            __m128i xdf = _mm_cmpgt_epi32( xl0, xdfTh );

            // same for the next line in the subblock
            pos += tuPars.m_width;
            xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &tCoeff[pos] ) );
            xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

            // and the third line
            pos += tuPars.m_width;
            xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &tCoeff[pos] ) );
            xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

            // and the last line
            pos += tuPars.m_width;
            xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &tCoeff[pos] ) );
            xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

            // if any of the 16 comparisons were true, break, because this subblock contains a coefficient larger than threshold
            if( !_mm_testz_si128( xdf, xdf ) ) break;
          }

          if( firstTestPos >= 0 )
          {
            // if a coefficient was found, advance the pointer to the end of the current subblock
            // for the subsequent coefficient-wise refinement (C-impl after endif)
            firstTestPos += sbbSize - 1;
          }
        }

#endif
        for( ; firstTestPos >= 0; firstTestPos-- )
        {
          if( zeroOutforThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth || tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) ) continue;
          if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > defaultTh ) break;
        }
      }

      if( firstTestPos < 0 )
      {
        tu.lastPos[compID] = -1;
        return;
      }

      //===== real init =====
      RateEstimator::initCtx( tuPars, tu, compID, ctx.getFracBitsAcess() );
      m_commonCtx.reset( tuPars, *this );
      for( int k = 0; k < 12; k++ )
      {
        m_allStates[k].init( m_state_mem[k>>2] );
      }

      const int numCtx = isLuma( compID ) ? 21 : 11;
      const CoeffFracBits* const cffBits = gtxFracBits();
      for( int i = 0; i < numCtx; i++ )
      {
        m_state_mem[0].cffBits1[i] = cffBits[i].bits[1];
        m_state_mem[1].cffBits1[i] = cffBits[i].bits[1];
        m_state_mem[2].cffBits1[i] = cffBits[i].bits[1];
      }

      int effectWidth  = std::min( 32, effWidth );
      int effectHeight = std::min( 32, effHeight );
      for (int k = 0; k < 3; k++)
      {
        m_state_mem[k].effWidth         = effectWidth;
        m_state_mem[k].effHeight        = effectHeight;
        m_state_mem[k].initRemRegBins   = ( effectWidth * effectHeight * MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT ) / 16;
        m_state_mem[k].anyRemRegBinsLt4 = true; // for the first coeff use scalar impl., because it check against the init state, which
                                                // prohibits some paths
      }

      //===== populate trellis =====
      for( int scanIdx = firstTestPos; scanIdx >= 0; scanIdx-- )
      {
        const ScanInfo& scanInfo = tuPars.m_scanInfo[ scanIdx ];
        if( enableScalingLists )
        {
          m_quant.initQuantBlock( tu, compID, cQP, lambda, quantCoeff[scanInfo.rasterPos] );
          xDecideAndUpdate( abs( tCoeff[scanInfo.rasterPos] ), scanInfo, zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ), quantCoeff[scanInfo.rasterPos] );
        }
        else
          xDecideAndUpdate( abs( tCoeff[scanInfo.rasterPos] ), scanInfo, zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ), defaultQuantisationCoefficient );
      }

      //===== find best path =====
      int       prevId      = -1;
      int64_t   minPathCost =  0;
      for( int8_t stateId = 0; stateId < 4; stateId++ )
      {
        int64_t pathCost = m_trellis[0][0].rdCost[stateId];
        if( pathCost < minPathCost )
        {
          prevId      = stateId;
          minPathCost = pathCost;
        }
      }

      //===== backward scanning =====
      int scanIdx = 0;
      for( ; prevId >= 0; scanIdx++ )
      {
        TCoeffSig absLevel = m_trellis[scanIdx][prevId >> 2].absLevel[prevId & 3];
        int32_t blkpos     = tuPars.m_scanId2BlkPos[scanIdx].idx;
        qCoeff[ blkpos ]   = TCoeffSig( tCoeff[blkpos] < 0 ? -absLevel : absLevel );
        absSum            += absLevel;
        prevId             = m_trellis[scanIdx][prevId >> 2].prevId[prevId & 3];
      }

      tu.lastPos[compID] = scanIdx - 1;
    }

  private:

    void xDecideAndUpdate( const TCoeff absCoeff, const ScanInfo &scanInfo, bool zeroOut, int quantCoeff )
    {
      Decisions *decisions = &m_trellis[scanInfo.scanIdx][0];

      std::swap( m_prevStates, m_currStates );
      std::swap( m_prevStateI, m_currStateI );

      xDecide( scanInfo, absCoeff, lastOffset(scanInfo.scanIdx), *decisions, zeroOut, quantCoeff );

      if( scanInfo.scanIdx )
      {
        if( scanInfo.insidePos == 0 )
        {
          m_commonCtx.swap();
          State<vext>::updateStatesEOS( scanInfo, *decisions, m_state_mem[m_prevStateI], m_state_mem[m_skipStateI], m_state_mem[m_currStateI], m_commonCtx );
          ::memcpy( decisions + 1, decisions, sizeof( Decisions ) );
        }
        else if( !zeroOut )
        {
          State<vext>::updateStates( scanInfo, *decisions, m_state_mem[m_prevStateI], m_state_mem[m_currStateI] );
        }

        if( scanInfo.spt == SCAN_SOCSBB )
        {
          std::swap( m_prevStates, m_skipStates );
          std::swap( m_prevStateI, m_skipStateI );
        }
      }
    }

    void xDecide( const ScanInfo &scanInfo, const TCoeff absCoeff, const int lastOffset, Decisions &decisions, bool zeroOut, int quantCoeff )
    {
      ::memcpy( &decisions, startDec, sizeof( Decisions ) );

      StateMem& prev = m_state_mem[m_prevStateI];
      StateMem& skip = m_state_mem[m_skipStateI];

      if( zeroOut )
      {
        if( scanInfo.spt==SCAN_EOCSBB )
        {
          m_skipStates[0].checkRdCostSkipSbbZeroOut( decisions, 0, skip );
          m_skipStates[1].checkRdCostSkipSbbZeroOut( decisions, 1, skip );
          m_skipStates[2].checkRdCostSkipSbbZeroOut( decisions, 2, skip );
          m_skipStates[3].checkRdCostSkipSbbZeroOut( decisions, 3, skip );
        }
        return;
      }

      PQData  pqData[4];
      //bool near0 = m_quant.preQuantCoeff( absCoeff, pqData, quantCoeff );

      /// start inline prequant
      int64_t scaledOrg = int64_t( absCoeff ) * quantCoeff;
      TCoeff  qIdx      = TCoeff( ( scaledOrg + m_quant.m_QAdd ) >> m_quant.m_QShift );

      if( qIdx < 0 )
      {
        int64_t scaledAdd = m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;
        PQData& pq_a      = pqData[1];
        PQData& pq_b      = pqData[2];

        pq_a.deltaDist    = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * 1 + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
        pq_a.absLevel     = 1;

        pq_b.deltaDist    = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * 2 + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
        pq_b.absLevel     = 1;
        /// stop inline prequant

        if( prev.anyRemRegBinsLt4 )
        {
          m_prevStates[0].setRiceParam( scanInfo, prev, false );
          m_prevStates[0].checkRdCostsOdd1( scanInfo.spt, pqData[2], decisions, 2, 0, prev );

          m_prevStates[1].setRiceParam( scanInfo, prev, false );
          m_prevStates[1].checkRdCostsOdd1( scanInfo.spt, pqData[2], decisions, 0, 2, prev );

          m_prevStates[2].setRiceParam( scanInfo, prev, false );
          m_prevStates[2].checkRdCostsOdd1( scanInfo.spt, pqData[1], decisions, 3, 1, prev );

          m_prevStates[3].setRiceParam( scanInfo, prev, false ); 
          m_prevStates[3].checkRdCostsOdd1( scanInfo.spt, pqData[1], decisions, 1, 3, prev );
        }
        else
        {
          // has to be called as a first check, assumes no decision has been made yet
          State<vext>::checkAllRdCostsOdd1( scanInfo.spt, m_prevStates, pqData, decisions, prev );
        }

        m_prevStates->checkRdCostStart( lastOffset, pqData[2], decisions, 2 );
      }
      else
      {
        /// start inline prequant
        qIdx              = std::max<TCoeff>( 1, std::min<TCoeff>( m_quant.m_maxQIdx, qIdx ) );
        int64_t scaledAdd = qIdx * m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;

        PQData& pq_a      = pqData[( qIdx + 0 ) & 3];
        PQData& pq_b      = pqData[( qIdx + 1 ) & 3];
        PQData& pq_c      = pqData[( qIdx + 2 ) & 3];
        PQData& pq_d      = pqData[( qIdx + 3 ) & 3];

        pq_a.deltaDist    = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * ( qIdx + 0 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
        pq_a.absLevel     = ( qIdx + 1 ) >> 1;

        pq_b.deltaDist    = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * ( qIdx + 1 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
        pq_b.absLevel     = ( qIdx + 2 ) >> 1;

        pq_c.deltaDist    = ( ( scaledAdd + 2 * m_quant.m_DistStepAdd ) * ( qIdx + 2 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
        pq_c.absLevel     = ( qIdx + 3 ) >> 1;

        pq_d.deltaDist    = ( ( scaledAdd + 3 * m_quant.m_DistStepAdd ) * ( qIdx + 3 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
        pq_d.absLevel     = ( qIdx + 4 ) >> 1;
        /// stop inline prequant

        bool cff02ge4 = pqData[0].absLevel >= 4/* || pqData[2].absLevel >= 4 */;
        bool cff13ge4 = /* pqData[1].absLevel >= 4 || */ pqData[3].absLevel >= 4;

        if( cff02ge4 || cff13ge4 || prev.anyRemRegBinsLt4 )
        {
          if( prev.anyRemRegBinsLt4 || cff02ge4 )
          {
            m_prevStates[0].setRiceParam( scanInfo, prev, cff02ge4 );
            m_prevStates[1].setRiceParam( scanInfo, prev, cff02ge4 );
          }

          if( prev.anyRemRegBinsLt4 || cff13ge4 )
          {
            m_prevStates[2].setRiceParam( scanInfo, prev, cff13ge4 );
            m_prevStates[3].setRiceParam( scanInfo, prev, cff13ge4 ); 
          }

          m_prevStates[0].checkRdCosts( scanInfo.spt, pqData[0], pqData[2], decisions, 0, 2, prev );
          m_prevStates[1].checkRdCosts( scanInfo.spt, pqData[0], pqData[2], decisions, 2, 0, prev );
          m_prevStates[2].checkRdCosts( scanInfo.spt, pqData[3], pqData[1], decisions, 1, 3, prev );
          m_prevStates[3].checkRdCosts( scanInfo.spt, pqData[3], pqData[1], decisions, 3, 1, prev );
        }
        else
        {
          // has to be called as a first check, assumes no decision has been made yet
          State<vext>::checkAllRdCosts( scanInfo.spt, m_prevStates, pqData, decisions, prev );
        }

        m_prevStates->checkRdCostStart( lastOffset, pqData[0], decisions, 0 );
        m_prevStates->checkRdCostStart( lastOffset, pqData[2], decisions, 2 );
      }

      if( scanInfo.spt==SCAN_EOCSBB )
      {
        m_skipStates[0].checkRdCostSkipSbb( decisions, 0, skip );
        m_skipStates[1].checkRdCostSkipSbb( decisions, 1, skip );
        m_skipStates[2].checkRdCostSkipSbb( decisions, 2, skip );
        m_skipStates[3].checkRdCostSkipSbb( decisions, 3, skip );
      }
    }

  private:
    CommonCtx<vext> m_commonCtx;
    State<vext>     m_allStates[ 12 ];
    State<vext>*    m_currStates;
    State<vext>*    m_prevStates;
    State<vext>*    m_skipStates;
    Quantizer       m_quant;
    Decisions       m_trellis[MAX_TB_SIZEY * MAX_TB_SIZEY][2];
    Rom             m_scansRom;

    StateMem        m_state_mem[3];

    int m_currStateI = 0;
    int m_prevStateI = 1;
    int m_skipStateI = 2;
  };
}; // namespace DQIntern

template<X86_VEXT vext>
void DepQuant::_initDepQuantX86()
{
  p = new DQIntern::DepQuantSimd<vext>();
}
template void DepQuant::_initDepQuantX86<SIMDX86>();

} // namespace vvenc

//! \}

;
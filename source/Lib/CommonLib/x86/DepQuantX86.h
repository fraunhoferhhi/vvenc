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

#include "DepQuant.h"

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT_QUANT

#  include "x86/CommonDefX86.h"
#  include <simde/x86/sse4.1.h>
#if defined( USE_SSE41 ) || !defined( REAL_TARGET_X86 )
#  include <simde/x86/sse4.2.h>
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

  namespace DQInternSimd
  {
    template<X86_VEXT vext>
    static inline void updateStates( const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, DQIntern::StateMem& curr )
    {
      int8_t s[4] = { 0 }, t[4] = { 0 }, l[4] = { 0 };

      __m128i v126_4 = _mm_setr_epi16( 126, 126, 126, 126, 4, 4, 4, 4 );
      __m128i v01 = _mm_setr_epi16( 1, 1, 1, 1, 1, 1, 1, 1 );
      __m128i v032 = _mm_setr_epi8( 0, 0, 0, 0, 32, 32, 32, 32, 0, 0, 0, 0, 0, 0, 0, 0 );
      __m128i vn1 = _mm_set1_epi8( -1 );

      static_assert( sizeof( curr.rdCost ) == sizeof( decisions.rdCost ), "Non-matching array size" );
      memcpy( curr.rdCost, decisions.rdCost, sizeof( decisions.rdCost ) );

      // in signalling, the coeffs are always max 16 bit!
      __m128i v = _mm_loadu_si64( decisions.absLevel );
      v = _mm_unpacklo_epi64( v, v );
      __m128i p = _mm_loadu_si32( decisions.prevId );
      _mm_storeu_si32( s, p ); // store previous state indexes
      p = _mm_shuffle_epi32( p, 0 );
      __m128i n2 = _mm_cmplt_epi8( p, vn1 );
      __m128i a_1 = _mm_and_si128( v, v01 );
      __m128i a_m = _mm_min_epi16( v, _mm_add_epi16( v126_4, a_1 ) );
      a_m = _mm_packs_epi16( a_m, vn1 );
      a_m = _mm_or_si128( a_m, _mm_sign_epi8( v032, a_m ) );
      a_m = _mm_andnot_si128( n2, a_m );
      _mm_storeu_si32( l, a_m ); // store abs value
      a_m = _mm_shuffle_epi32( a_m, 1 );
      _mm_storeu_si32( t, a_m ); // store store capped abs value

      {
        const int ctxSize = 16 * 4;
        const int regSize = 16;

        __m128i vshuf = _mm_loadu_si32( s );
        vshuf = _mm_shuffle_epi32( vshuf, 0 );
        __m128i vshufmask = _mm_cmplt_epi8( vshuf, _mm_setzero_si128() );
        vshuf = _mm_add_epi8( vshuf, _mm_setr_epi8( 0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12 ) );
        vshuf = _mm_blendv_epi8( vshuf, _mm_set1_epi8( -1 ), vshufmask );

        auto* tplAcc = &curr.tplAcc[0][0];
        auto* absVal = &curr.absVal[0][0];
        auto* sum1st = &curr.sum1st[0][0];

        for( int i = 0; i < ctxSize; i += regSize )
        {
          __m128i vtpl = _mm_loadu_si128( ( const __m128i* ) &tplAcc[i]);
          vtpl = _mm_shuffle_epi8( vtpl, vshuf );
          _mm_storeu_si128( ( __m128i* ) &tplAcc[i], vtpl );

          __m128i vval = _mm_loadu_si128( ( const __m128i* ) &absVal[i] );
          vval = _mm_shuffle_epi8( vval, vshuf );
          _mm_storeu_si128( ( __m128i* ) &absVal[i], vval );

          __m128i vsum = _mm_loadu_si128( ( const __m128i* ) &sum1st[i] );
          vsum = _mm_shuffle_epi8( vsum, vshuf );
          _mm_storeu_si128( ( __m128i* ) &sum1st[i], vsum );
        }

        __m128i numSig = _mm_loadu_si32( curr.numSig );
        numSig = _mm_shuffle_epi8( numSig, vshuf );
        __m128i lvls = _mm_loadu_si32( l );
        lvls = _mm_cmpgt_epi8( lvls, _mm_setzero_si128() );
        numSig = _mm_subs_epi8( numSig, lvls );
        _mm_storeu_si32( curr.numSig, numSig );

        __m128i rsc = _mm_loadu_si32( curr.refSbbCtxId );
        rsc = _mm_shuffle_epi8( rsc, vshuf );
        rsc = _mm_blendv_epi8( rsc, vshuf, vshuf );
        _mm_storeu_si32( curr.refSbbCtxId, rsc );

        vshuf = _mm_shuffle_epi8( vshuf, _mm_setr_epi8( 0, 0, 1, 1, 2, 2, 3, 3, -1, -1, -1, -1, -1, -1, -1, -1 ) );
        vshuf = _mm_slli_epi16( vshuf, 1 );
        vshuf = _mm_add_epi8( vshuf,
                              _mm_blendv_epi8( _mm_setr_epi8( 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 ),
                              _mm_setzero_si128(),
                              vshuf ) );

        __m128i rrb = _mm_loadu_si64( ( const __m128i* ) curr.remRegBins );
        rrb = _mm_shuffle_epi8( rrb, vshuf );
        rrb = _mm_sub_epi16( rrb, v01 );
        rrb = _mm_blendv_epi8( rrb, _mm_set1_epi16( curr.initRemRegBins ), vshuf );
        __m128i mlvl = _mm_loadu_si32( l );
        __m128i mbins = _mm_min_epi8( mlvl, _mm_set1_epi8( 2 ) );
        __m128i mlutb = _mm_setr_epi8( 0, 1, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
        rrb = _mm_sub_epi16( rrb, _mm_cvtepi8_epi16( _mm_shuffle_epi8( mlutb, mbins ) ) );
        _mm_storeu_si64( ( __m128i* ) curr.remRegBins, rrb );
        rrb = _mm_cmplt_epi16( rrb, _mm_set1_epi16( 4 ) );

        curr.anyRemRegBinsLt4 = !!_mm_cvtsi128_si64( rrb );

        __m128i lvl1 = _mm_loadu_si32( l );
        __m128i tpl1 = _mm_loadu_si32( t );

        auto update_deps_vec = [&]( int k )
        {
          int addr = scanInfo.currNbInfoSbb.invInPos[k];

          __m128i msum = _mm_loadu_si32( &curr.sum1st[addr][0] );
          msum = _mm_adds_epu8( msum, mlvl );
          _mm_storeu_si32( &curr.sum1st[addr][0], msum);

          __m128i tpl = _mm_loadu_si32( &curr.tplAcc[addr][0] );
          tpl = _mm_add_epi8( tpl, tpl1 );
          _mm_storeu_si32( &curr.tplAcc[addr][0], tpl);
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
        case 0:
          ;
        }

        _mm_storeu_si32( &curr.absVal[scanInfo.insidePos][0], lvl1);
      }

      {
        __m128i tplAcc = _mm_loadu_si32( &curr.tplAcc[scanInfo.nextInsidePos][0]);

        __m128i sumAbs1 = _mm_and_si128( tplAcc, _mm_set1_epi8( 31 ) );
        __m128i sumNum = _mm_and_si128( _mm_srli_epi32( tplAcc, 5 ), _mm_set1_epi8( 7 ) );
        __m128i sumGt1 = _mm_sub_epi8( sumAbs1, sumNum );
        sumGt1 = _mm_min_epi8( sumGt1, _mm_set1_epi8( 4 ) );
        sumGt1 = _mm_add_epi8( _mm_set1_epi8( scanInfo.gtxCtxOffsetNext ), sumGt1 );
        _mm_storeu_si32( curr.ctx.cff, sumGt1 );

        sumAbs1 = _mm_add_epi8( sumAbs1, _mm_set1_epi8( 1 ) );
        sumAbs1 = _mm_srli_epi32( sumAbs1, 1 );
        sumAbs1 = _mm_and_si128( sumAbs1, _mm_set1_epi8( 127 ) );
        sumAbs1 = _mm_min_epi8( sumAbs1, _mm_set1_epi8( 3 ) );
        sumAbs1 = _mm_add_epi8( _mm_set1_epi8( scanInfo.sigCtxOffsetNext ), sumAbs1 );
        _mm_storeu_si32( curr.ctx.sig, sumAbs1 );

        curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
      }
    }

    template<X86_VEXT vext>
    static inline void updateStatesEOS( const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, const DQIntern::StateMem& skip, DQIntern::StateMem& curr, DQIntern::CommonCtx& commonCtx )
    {
      int8_t s[4] = { 0 }, l[4] = { 0 }, z[4] = { 0 };
      for( int i = 0; i < 4; ++i )
      {
        s[i] = decisions.prevId[i] >= 4 ? -2 : decisions.prevId[i];
        l[i] = s[i] > -2 ? std::min<int>( decisions.absLevel[i], 126 + ( decisions.absLevel[i] & 1 ) ) : 0;
        z[i] = 3 - decisions.prevId[i];
        curr.rdCost[i] = decisions.rdCost[i];
      }
      {
        const int ctxSize = 16 * 4;
        const int regSize = 16;

        __m128i vshuf = _mm_loadu_si32( s );
        vshuf = _mm_shuffle_epi32( vshuf, 0 );
        __m128i vshufmask = _mm_cmplt_epi8( vshuf, _mm_setzero_si128() );
        vshuf = _mm_add_epi8( vshuf, _mm_setr_epi8( 0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12 ) );
        vshuf = _mm_blendv_epi8( vshuf, _mm_set1_epi8( -1 ), vshufmask );

        auto* absVal = &curr.absVal[0][0];

        for( int i = 0; i < ctxSize; i += regSize )
        {
          __m128i vval = _mm_loadu_si128( ( const __m128i* ) &absVal[i] );
          vval = _mm_shuffle_epi8( vval, vshuf );
          _mm_storeu_si128( ( __m128i* ) &absVal[i], vval );
        }

        __m128i numSig = _mm_loadu_si32( curr.numSig );
        numSig = _mm_shuffle_epi8( numSig, vshuf );
        __m128i lvls = _mm_loadu_si32( l );
        _mm_storeu_si32( &curr.absVal[scanInfo.insidePos][0], lvls);
        lvls = _mm_cmpgt_epi8( lvls, _mm_setzero_si128() );
        numSig = _mm_subs_epi8( numSig, lvls );
        _mm_storeu_si32( curr.numSig, numSig );

        __m128i rsc = _mm_loadu_si32( curr.refSbbCtxId );
        rsc = _mm_shuffle_epi8( rsc, vshuf );
        rsc = _mm_blendv_epi8( rsc, vshuf, vshuf );
        _mm_storeu_si32( curr.refSbbCtxId, rsc );

        vshuf = _mm_shuffle_epi8( vshuf, _mm_setr_epi8( 0, 0, 1, 1, 2, 2, 3, 3, -1, -1, -1, -1, -1, -1, -1, -1 ) );
        vshuf = _mm_slli_epi16( vshuf, 1 );
        vshuf = _mm_add_epi8( vshuf,
                              _mm_blendv_epi8( _mm_setr_epi8( 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 ),
                              _mm_setzero_si128(),
                              vshuf ) );

        __m128i rrb = _mm_loadu_si64( ( const __m128i* ) curr.remRegBins );
        rrb = _mm_shuffle_epi8( rrb, vshuf );
        rrb = _mm_sub_epi16( rrb, _mm_set1_epi16( 1 ) );
        rrb = _mm_blendv_epi8( rrb, _mm_set1_epi16( curr.initRemRegBins ), vshuf );

        __m128i vskip = _mm_cvtepi8_epi16( _mm_loadu_si32( z ) );
        rrb = _mm_blendv_epi8( rrb, _mm_loadu_si64( ( const __m128i* ) skip.remRegBins ), vskip );

        __m128i mlvl = _mm_loadu_si32( l );
        __m128i mbins = _mm_min_epi8( mlvl, _mm_set1_epi8( 2 ) );
        __m128i mlutb = _mm_setr_epi8( 0, 1, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
        rrb = _mm_sub_epi16( rrb, _mm_cvtepi8_epi16( _mm_shuffle_epi8( mlutb, mbins ) ) );
        _mm_storeu_si64( ( __m128i* ) curr.remRegBins, rrb );
        rrb = _mm_cmplt_epi16( rrb, _mm_set1_epi16( 4 ) );

        curr.anyRemRegBinsLt4 = !!_mm_cvtsi128_si64( rrb );
      }

      {
        uint8_t* levels0;
        uint8_t* levels1;
        uint8_t* levels2;
        uint8_t* levels3;

        commonCtx.getLevelPtrs( scanInfo, levels0, levels1, levels2, levels3 );

        const int regSize = 16;
        const int ctxSize = scanInfo.sbbSize << 2;

        const __m128i vshuf0 = _mm_setr_epi8( 0, 4, 8, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
        const __m128i vshuf1 = _mm_setr_epi8( 1, 5, 9, 13, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
        const __m128i vshuf2 = _mm_setr_epi8( 2, 6, 10, 14, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );
        const __m128i vshuf3 = _mm_setr_epi8( 3, 7, 11, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 );

        auto* absVal = &curr.absVal[0][0];

        for( int i = 0, j = 0; i < ctxSize; i += regSize, j += 4 )
        {
          __m128i in = _mm_loadu_si128( ( const __m128i* ) &absVal[i] );

          _mm_storeu_si32( &levels0[j], _mm_shuffle_epi8( in, vshuf0 ) );
          _mm_storeu_si32( &levels1[j], _mm_shuffle_epi8( in, vshuf1 ) );
          _mm_storeu_si32( &levels2[j], _mm_shuffle_epi8( in, vshuf2 ) );
          _mm_storeu_si32( &levels3[j], _mm_shuffle_epi8( in, vshuf3 ) );
        }
      }

      memset( curr.absVal, 0, sizeof( curr.absVal ) );
      memset( curr.tplAcc, 0, sizeof( curr.tplAcc ) );
      memset( curr.sum1st, 0, sizeof( curr.sum1st ) );

      for( int i = 0; i < 4; i++ )
      {
        int prevId = decisions.prevId[i];

        if( prevId > -2 )
        {
          const int refId = prevId < 0 ? -1 : ( prevId < 4 ? curr.refSbbCtxId[i] : prevId - 4 );
          commonCtx.update( scanInfo, refId, i, curr );
        }
      }

      memset( curr.numSig, 0, sizeof( curr.numSig ) );

      {
        __m128i tplAcc = _mm_loadu_si32( &curr.tplAcc[scanInfo.nextInsidePos][0]);

        __m128i sumAbs1 = _mm_and_si128( tplAcc, _mm_set1_epi8( 31 ) );
        __m128i sumNum = _mm_and_si128( _mm_srli_epi32( tplAcc, 5 ), _mm_set1_epi8( 7 ) );
        __m128i sumGt1 = _mm_sub_epi8( sumAbs1, sumNum );
        sumGt1 = _mm_min_epi8( sumGt1, _mm_set1_epi8( 4 ) );
        sumGt1 = _mm_add_epi8( _mm_set1_epi8( scanInfo.gtxCtxOffsetNext ), sumGt1 );
        _mm_storeu_si32( curr.ctx.cff, sumGt1 );

        sumAbs1 = _mm_add_epi8( sumAbs1, _mm_set1_epi8( 1 ) );
        sumAbs1 = _mm_srli_epi32( sumAbs1, 1 );
        sumAbs1 = _mm_and_si128( sumAbs1, _mm_set1_epi8( 127 ) );
        sumAbs1 = _mm_min_epi8( sumAbs1, _mm_set1_epi8( 3 ) );
        sumAbs1 = _mm_add_epi8( _mm_set1_epi8( scanInfo.sigCtxOffsetNext ), sumAbs1 );
        _mm_storeu_si32( curr.ctx.sig, sumAbs1 );

        curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
      }
    }

    // has to be called as a first check, assumes no decision has been made yet
    template<X86_VEXT vext>
    static void checkAllRdCosts( const DQIntern::ScanPosType spt, const DQIntern::PQData* pqData, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
    {
      // State mapping
      // decision 0: either A from 0 (pq0), or B from 1 (pq2), or 0 from 0
      // decision 1: either A from 2 (pq3), or B from 3 (pq1), or 0 from 2
      // decision 2: either A from 1 (pq0), or B from 0 (pq2), or 0 from 1
      // decision 3: either A from 3 (pq3), or B from 2 (pq1), or 0 from 3

      __m128i mrd01 = _mm_loadu_si128( ( const __m128i* ) & state.rdCost[0] );
      __m128i mrd23 = _mm_loadu_si128( ( const __m128i* ) & state.rdCost[2] );

      __m128i rdCostZ01 = mrd01;
      __m128i rdCostZ23 = mrd23;
      __m128i rdCostB01 = _mm_add_epi64( rdCostZ01, _mm_set1_epi64x( pqData[2].deltaDist ) );
      __m128i rdCostB23 = _mm_add_epi64( rdCostZ23, _mm_set1_epi64x( pqData[1].deltaDist )  );
      __m128i rdCostA01 = _mm_add_epi64( rdCostZ01, _mm_set1_epi64x( pqData[0].deltaDist )  );
      __m128i rdCostA23 = _mm_add_epi64( rdCostZ23, _mm_set1_epi64x( pqData[3].deltaDist )  );

      //const CoeffFracBits &cffBits = m_gtxFracBitsArray[state.ctx.cff[m_stateId]];
      //const BinFracBits    sigBits = m_sigFracBitsArray[state.ctx.sig[m_stateId]];
      //
      //rdCostA += cffBits.bits[ pqDataA.absLevel ];
      //rdCostB += cffBits.bits[ pqDataB.absLevel ];
      __m128i sgbts01 = _mm_unpacklo_epi64( _mm_loadu_si64( &state.m_sigFracBitsArray[0][state.ctx.sig[0]] ),
                                            _mm_loadu_si64( &state.m_sigFracBitsArray[1][state.ctx.sig[1]] ) );
      __m128i sgbts23 = _mm_unpacklo_epi64( _mm_loadu_si64( &state.m_sigFracBitsArray[2][state.ctx.sig[2]] ),
                                            _mm_loadu_si64( &state.m_sigFracBitsArray[3][state.ctx.sig[3]] ) );

      sgbts01 = _mm_shuffle_epi32( sgbts01, 0 + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );
      sgbts23 = _mm_shuffle_epi32( sgbts23, 0 + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );

      __m128i addZ = _mm_unpacklo_epi64( sgbts01, sgbts23 );
      __m128i addA, addB;
      __m128i sgbts1 = _mm_unpackhi_epi64( sgbts01, sgbts23 );

      const TCoeff lvl0 = pqData[0].absLevel, lvl1 = pqData[1].absLevel, lvl2 = pqData[2].absLevel, lvl3 = pqData[3].absLevel;

#if USE_AVX2
      static_assert( sizeof( state.m_gtxFracBitsArray[0] ) == 24, "CoeffFracBits has to be 6 x int32_t" );
      static_assert( sizeof( state.m_gtxFracBitsArray[0].bits[0] ) == 4, "bits has to be int32_t" );

      __m128i absLvl = _mm_setr_epi32( lvl0, lvl1, lvl2, lvl3 );
      __m128i cffs   = _mm_loadu_si32( ( const void* ) state.ctx.cff );
      cffs           = _mm_cvtepi8_epi32( cffs );
      cffs           = _mm_slli_epi32( cffs, 1 );
      cffs           = _mm_add_epi32 ( cffs, _mm_slli_epi32( cffs, 1 ) ); // 6 x cff, is the CoeffFracBits offset in multiples of int32_t

      __m128i idxA   = _mm_shuffle_epi32( absLvl, 0 + ( 0 << 2 ) + ( 3 << 4 ) + ( 3 << 6 ) );
      __m128i idxB   = _mm_shuffle_epi32( absLvl, 2 + ( 2 << 2 ) + ( 1 << 4 ) + ( 1 << 6 ) );
      idxA           = _mm_add_epi32( idxA, cffs );
      idxB           = _mm_add_epi32( idxB, cffs );

      addA           = _mm_i32gather_epi32( ( const int32_t* ) state.m_gtxFracBitsArray, idxA, 4 );
      addB           = _mm_i32gather_epi32( ( const int32_t* ) state.m_gtxFracBitsArray, idxB, 4 );
#else
      {
        // coeff context is indepndent of state
        auto& base = state.m_gtxFracBitsArray;

        int32_t cffBitsArr[4] =
        {
          base[state.ctx.cff[0]].bits[lvl2],
          base[state.ctx.cff[1]].bits[lvl2],
          base[state.ctx.cff[2]].bits[lvl1],
          base[state.ctx.cff[3]].bits[lvl1],
        };

        addB = _mm_loadu_si128( ( const __m128i* ) cffBitsArr );
      }

      {
        // coeff context is indepndent of state
        auto& base = state.m_gtxFracBitsArray;

        int32_t cffBitsArr[4] =
        {
          base[state.ctx.cff[0]].bits[lvl0],
          base[state.ctx.cff[1]].bits[lvl0],
          base[state.ctx.cff[2]].bits[lvl3],
          base[state.ctx.cff[3]].bits[lvl3],
        };

        addA  = _mm_loadu_si128( ( const __m128i* ) cffBitsArr );
      }
#endif

      if( spt == DQIntern::SCAN_ISCSBB )
      {
      }
      else if( spt == DQIntern::SCAN_SOCSBB )
      {
        __m128i sbbBits = _mm_loadu_si128( ( const __m128i* ) state.sbbBits1 );
        addA = _mm_add_epi32( sbbBits, addA );
        addB = _mm_add_epi32( sbbBits, addB );
        addZ = _mm_add_epi32( sbbBits, addZ );
      }
      else if( spt == DQIntern::SCAN_EOCSBB )
      {
        __m128i numSig = _mm_loadu_si32( state.numSig );

        __m128i mask = _mm_shuffle_epi8( numSig, _mm_setr_epi8( 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3 ) );
        mask         = _mm_cmpeq_epi8( mask, _mm_setzero_si128() );

        sgbts1 = _mm_andnot_si128( mask, sgbts1 );

        __m128i rdMax = _mm_loadu_si64( &DQIntern::rdCostInit );
        rdMax = _mm_unpacklo_epi64( rdMax, rdMax );

        rdCostZ01 = _mm_blendv_epi8( rdCostZ01, rdMax, _mm_cvtepi32_epi64( mask ) );
        rdCostZ23 = _mm_blendv_epi8( rdCostZ23, rdMax, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( mask, mask ) ) );
      }

      addA = _mm_add_epi32( addA, sgbts1 );
      addB = _mm_add_epi32( addB, sgbts1 );

      rdCostZ01 = _mm_add_epi64( rdCostZ01, _mm_cvtepi32_epi64( addZ ) );
      rdCostZ23 = _mm_add_epi64( rdCostZ23, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( addZ, addZ ) ) );
      rdCostA01 = _mm_add_epi64( rdCostA01, _mm_cvtepi32_epi64( addA ) );
      rdCostA23 = _mm_add_epi64( rdCostA23, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( addA, addA ) ) );
      rdCostB01 = _mm_add_epi64( rdCostB01, _mm_cvtepi32_epi64( addB ) );
      rdCostB23 = _mm_add_epi64( rdCostB23, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( addB, addB ) ) );

      {
        __m128i tmp1 = rdCostZ01, tmp2 = rdCostZ23;
        rdCostZ01 = _mm_unpacklo_epi64( tmp1, tmp2 );
        rdCostZ23 = _mm_unpackhi_epi64( tmp1, tmp2 );

        tmp1 = rdCostA01; tmp2 = rdCostA23;
        rdCostA01 = _mm_unpacklo_epi64( tmp1, tmp2 );
        rdCostA23 = _mm_unpackhi_epi64( tmp1, tmp2 );

        tmp1 = rdCostB01; tmp2 = rdCostB23;
        rdCostB01 = _mm_unpackhi_epi64( tmp1, tmp2 );
        rdCostB23 = _mm_unpacklo_epi64( tmp1, tmp2 );
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

      __m128i valIdxBest = _mm_setr_epi16( 0, 0, lvl2, lvl1, 0, 2, 0, 2 );
      __m128i valIdxCand = _mm_setr_epi16( lvl0, lvl3, 0, 0, 0, 2, 1, 3 );

      __m128i chng01 = _my_cmpgt_epi64( rdBest01, rdCostA01 );
      __m128i chng23 = _my_cmpgt_epi64( rdBest23, rdCostZ23 );
      __m128i chng = _mm_packs_epi32( chng01, chng23 );
      chng = _mm_packs_epi32( chng, chng );

      rdBest01 = _mm_blendv_epi8( rdBest01, rdCostA01, chng01 );
      rdBest23 = _mm_blendv_epi8( rdBest23, rdCostZ23, chng23 );

      valIdxBest = _mm_blendv_epi8( valIdxBest, valIdxCand, chng );
      valIdxCand = _mm_setr_epi16( lvl2, lvl1, lvl0, lvl3, 1, 3, 1, 3 );

      chng01 = _my_cmpgt_epi64( rdBest01, rdCostB01 );
      chng23 = _my_cmpgt_epi64( rdBest23, rdCostA23 );
      chng = _mm_packs_epi32( chng01, chng23 );
      chng = _mm_packs_epi32( chng, chng );

      rdBest01 = _mm_blendv_epi8( rdBest01, rdCostB01, chng01 );
      rdBest23 = _mm_blendv_epi8( rdBest23, rdCostA23, chng23 );

      valIdxBest = _mm_blendv_epi8( valIdxBest, valIdxCand, chng );

      _mm_storeu_si128( ( __m128i* )& decisions.rdCost[0], rdBest01 );
      _mm_storeu_si128( ( __m128i* )& decisions.rdCost[2], rdBest23 );

      _mm_storeu_si64( decisions.absLevel, valIdxBest );
      valIdxBest = _mm_packs_epi16( _mm_unpackhi_epi64( valIdxBest, valIdxBest ), _mm_setzero_si128() );
      _mm_storeu_si32( decisions.prevId, valIdxBest );
    }

    // has to be called as a first check, assumes no decision has been made yet!!!
    template<X86_VEXT vext>
    static inline void checkAllRdCostsOdd1( const DQIntern::ScanPosType spt, const int64_t pq_a_dist, const int64_t pq_b_dist, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
    {
      // State mapping
      // decision 0: either 1 from 1 (pqData[2]), or 0 from 0
      // decision 1: either 1 from 3 (pqData[1]), or 0 from 2
      // decision 2: either 1 from 0 (pqData[2]), or 0 from 1
      // decision 3: either 1 from 2 (pqData[1]), or 0 from 3

      __m128i mrd01 = _mm_loadu_si128( ( const __m128i* ) & state.rdCost[0] );
      __m128i mrd23 = _mm_loadu_si128( ( const __m128i* ) & state.rdCost[2] );

      __m128i rdCostZ01 = mrd01;
      __m128i rdCostZ23 = mrd23;

      __m128i deltaDist  = _mm_unpacklo_epi64( _mm_cvtsi64_si128( pq_b_dist ), _mm_cvtsi64_si128( pq_a_dist ) );
      __m128i deltaDistB = _mm_unpacklo_epi64( deltaDist, deltaDist );
      __m128i deltaDistA = _mm_unpackhi_epi64( deltaDist, deltaDist );

      __m128i rdCostA01 = _mm_add_epi64( rdCostZ01, deltaDistB );
      __m128i rdCostA23 = _mm_add_epi64( rdCostZ23, deltaDistA );
      __m128i sgbts1, addZ, addA = _mm_setzero_si128();

      {
        __m128i sgbts01 = _mm_unpacklo_epi64( _mm_loadu_si64( &state.m_sigFracBitsArray[0][state.ctx.sig[0]] ),
                                              _mm_loadu_si64( &state.m_sigFracBitsArray[1][state.ctx.sig[1]] ) );
        __m128i sgbts23 = _mm_unpacklo_epi64( _mm_loadu_si64( &state.m_sigFracBitsArray[2][state.ctx.sig[2]] ),
                                              _mm_loadu_si64( &state.m_sigFracBitsArray[3][state.ctx.sig[3]] ) );

        sgbts01 = _mm_shuffle_epi32( sgbts01, 0 + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );
        sgbts23 = _mm_shuffle_epi32( sgbts23, 0 + ( 2 << 2 ) + ( 1 << 4 ) + ( 3 << 6 ) );
        sgbts1  = _mm_unpackhi_epi64( sgbts01, sgbts23 );
        addZ    = _mm_unpacklo_epi64( sgbts01, sgbts23 );
      }

      {
#if USE_AVX2
        __m128i cffidx = _mm_cvtepi8_epi32( _mm_loadu_si32( &state.ctx.cff ) );
        cffidx = _mm_sub_epi8( cffidx, _mm_set1_epi32( state.cffBitsCtxOffset ) );
        __m256i cffBits256 = _mm256_loadu_si256( ( const __m256i* ) & state.cffBits1[state.cffBitsCtxOffset] );
        cffBits256 = _mm256_permutevar8x32_epi32( cffBits256, _mm256_castsi128_si256( cffidx ) );
        __m128i cffBits = _mm256_castsi256_si128( cffBits256 );
#else
        __m128i cffBits;
        __m128i bits0123 = _mm_loadu_si128( ( const __m128i* ) & state.cffBits1[state.cffBitsCtxOffset + 0] );
        __m128i bits4 = _mm_loadu_si32( &state.cffBits1[state.cffBitsCtxOffset + 4] );
        __m128i cfCtxIdx = _mm_loadu_si32( &state.ctx.cff );
        cfCtxIdx = _mm_cvtepi8_epi32( cfCtxIdx );
        cfCtxIdx = _mm_sub_epi8( cfCtxIdx, _mm_set1_epi32( state.cffBitsCtxOffset ) );
        cfCtxIdx = _mm_or_si128( cfCtxIdx, _mm_slli_si128( cfCtxIdx, 1 ) );
        cfCtxIdx = _mm_or_si128( cfCtxIdx, _mm_slli_si128( cfCtxIdx, 2 ) );
        cfCtxIdx = _mm_slli_epi32( cfCtxIdx, 2 );
        cfCtxIdx = _mm_add_epi8( cfCtxIdx, _mm_setr_epi8( 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3 ) );
        cffBits = _mm_shuffle_epi8( bits4, _mm_sub_epi8( cfCtxIdx, _mm_set1_epi8( 16 ) ) );
        cfCtxIdx = _mm_or_si128( cfCtxIdx, _mm_cmpgt_epi8( cfCtxIdx, _mm_set1_epi8( 15 ) ) );
        cffBits = _mm_or_si128( cffBits, _mm_shuffle_epi8( bits0123, cfCtxIdx ) );
#endif
        addA = _mm_add_epi32( addA, cffBits );
      }

      if( spt == DQIntern::SCAN_ISCSBB )
      {
      }
      else if( spt == DQIntern::SCAN_SOCSBB )
      {
        __m128i sbbBits = _mm_loadu_si128( ( const __m128i* ) state.sbbBits1 );
        addA = _mm_add_epi32( addA, sbbBits );
        addZ = _mm_add_epi32( addZ, sbbBits );
      }
      else if( spt == DQIntern::SCAN_EOCSBB )
      {
        __m128i numSig = _mm_loadu_si32( state.numSig );

        __m128i mask = _mm_shuffle_epi8( numSig, _mm_setr_epi8( 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3 ) );
        mask         = _mm_cmpeq_epi8( mask, _mm_setzero_si128() );

        // if last coeff and first significant coeff, no sig-flag signalled
        sgbts1 = _mm_andnot_si128( mask, sgbts1 );

        __m128i rdMax = _mm_loadu_si64( &DQIntern::rdCostInit );
        rdMax = _mm_unpacklo_epi64( rdMax, rdMax );

        // if last coeff in the so-far empty sbb, and sbb-sig-flag is 1, cannot be 0!
        rdCostZ01 = _mm_blendv_epi8( rdCostZ01, rdMax, _mm_cvtepi32_epi64( mask ) );
        rdCostZ23 = _mm_blendv_epi8( rdCostZ23, rdMax, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( mask, mask ) ) );
      }

      addA = _mm_add_epi32( addA, sgbts1 );

      rdCostZ01 = _mm_add_epi64( rdCostZ01, _mm_cvtepi32_epi64( addZ ) );
      rdCostZ23 = _mm_add_epi64( rdCostZ23, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( addZ, addZ ) ) );
      rdCostA01 = _mm_add_epi64( rdCostA01, _mm_cvtepi32_epi64( addA ) );
      rdCostA23 = _mm_add_epi64( rdCostA23, _mm_cvtepi32_epi64( _mm_unpackhi_epi64( addA, addA ) ) );

      {
        __m128i tmp1 = rdCostZ01, tmp2 = rdCostZ23;
        rdCostZ01 = _mm_unpacklo_epi64( tmp1, tmp2 );
        rdCostZ23 = _mm_unpackhi_epi64( tmp1, tmp2 );

        tmp1 = rdCostA01; tmp2 = rdCostA23;
        rdCostA01 = _mm_unpackhi_epi64( tmp1, tmp2 );
        rdCostA23 = _mm_unpacklo_epi64( tmp1, tmp2 );
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

      __m128i valIdxBest = _mm_setr_epi16( 0, 0, 1, 1, 0, 2, 0, 2 );
      __m128i valIdxCand = _mm_setr_epi16( 1, 1, 0, 0, 1, 3, 1, 3 );

      __m128i chng01 = _my_cmpgt_epi64( rdBest01, rdCostA01 );
      __m128i chng23 = _my_cmpgt_epi64( rdBest23, rdCostZ23 );
      __m128i chng = _mm_packs_epi32( chng01, chng23 );
      chng = _mm_packs_epi32( chng, chng );

      rdBest01 = _mm_blendv_epi8( rdBest01, rdCostA01, chng01 );
      rdBest23 = _mm_blendv_epi8( rdBest23, rdCostZ23, chng23 );

      _mm_storeu_si128( ( __m128i* ) & decisions.rdCost[0], rdBest01 );
      _mm_storeu_si128( ( __m128i* ) & decisions.rdCost[2], rdBest23 );

      valIdxBest = _mm_blendv_epi8( valIdxBest, valIdxCand, chng );
      _mm_storeu_si64( decisions.absLevel, valIdxBest );
      valIdxBest = _mm_packs_epi16( _mm_unpackhi_epi64( valIdxBest, valIdxBest ), _mm_setzero_si128() );
      _mm_storeu_si32( decisions.prevId, valIdxBest );
    }

    template<X86_VEXT vext>
    void findFirstPos( int& firstTestPos, const TCoeff* tCoeff, const DQIntern::TUParameters& tuPars, int defaultTh, bool zeroOutForThres, int zeroOutWidth, int zeroOutHeight )
    {
      if( firstTestPos >= 16 && tuPars.m_log2SbbWidth == 2 && tuPars.m_log2SbbHeight == 2 )
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
          if( zeroOutForThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth || tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) )
          {
            continue;
          }

          // read first line of the subblock and check for coefficients larger than the threshold
          // assumming the subblocks are dense 4x4 blocks in raster scan order with the stride of tuPars.m_width
          int pos = tuPars.m_scanId2BlkPos[firstTestPos].idx;
          __m128i xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) & tCoeff[pos] ) );
          __m128i xdf = _mm_cmpgt_epi32( xl0, xdfTh );

          // same for the next line in the subblock
          pos += tuPars.m_width;
          xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) & tCoeff[pos] ) );
          xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

          // and the third line
          pos += tuPars.m_width;
          xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) & tCoeff[pos] ) );
          xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

          // and the last line
          pos += tuPars.m_width;
          xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) & tCoeff[pos] ) );
          xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

          // if any of the 16 comparisons were true, break, because this subblock contains a coefficient larger than threshold
          if( !_mm_testz_si128( xdf, xdf ) ) break;
        }

        if( firstTestPos >= 0 )
        {
          // if a coefficient was found, advance the pointer to the end of the current subblock
          // for the subsequent coefficient-wise refinement.
          firstTestPos += sbbSize - 1;
        }
      }

      for( ; firstTestPos >= 0; firstTestPos-- )
      {
        if( zeroOutForThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth ||
                                tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) )
        {
          continue;
        }
        if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > defaultTh )
        {
          break;
        }
      }
    }
  };

template<X86_VEXT vext>
void DepQuant::_initDepQuantX86()
{
  m_checkAllRdCosts     = DQInternSimd::checkAllRdCosts<vext>;
  m_checkAllRdCostsOdd1 = DQInternSimd::checkAllRdCostsOdd1<vext>;
  m_updateStatesEOS     = DQInternSimd::updateStatesEOS<vext>;
  m_updateStates        = DQInternSimd::updateStates<vext>;
  m_findFirstPos        = DQInternSimd::findFirstPos<vext>;
}
template void DepQuant::_initDepQuantX86<SIMDX86>();

}; // namespace vvenc

//! \}

#endif //ENABLE_SIMD_OPT_QUANT && defined( TARGET_SIMD_X86 )


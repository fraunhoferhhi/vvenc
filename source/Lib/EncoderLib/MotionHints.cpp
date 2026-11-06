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
/** \file     MotionHints.cpp
\brief    Optional motion hints supplied by the application
*/

#include "MotionHints.h"

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <vector>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

namespace
{
  const int MH_BLOCK_SIZE = 16;

  struct EncoderHints
  {
    std::unordered_map<int64_t, std::vector<int16_t>> fields;   // empty vector: the callback declined
  };

  std::mutex                                             g_hintMutex;
  std::unordered_map<const VVEncCfg*, EncoderHints>      g_hints;
}

bool MotionHints::get( const VVEncCfg& cfg, const Picture& cur, const Picture& ref, int cx, int cy, int& dx, int& dy )
{
  // with inter-frame parallelism a reference may still be reconstructing: no hints then
  if( !cfg.m_motionHintFnc || cfg.m_ifp )
  {
    return false;
  }

  const CPelBuf c  = cur.getOrigBuf( COMP_Y );
  const CPelBuf r  = ref.getRecoBuf( COMP_Y );
  const int     gw = ( c.width  + MH_BLOCK_SIZE - 1 ) / MH_BLOCK_SIZE;
  const int     gh = ( c.height + MH_BLOCK_SIZE - 1 ) / MH_BLOCK_SIZE;
  const int64_t key = ( int64_t( cur.poc ) << 32 ) | uint32_t( ref.poc );

  std::lock_guard<std::mutex> lock( g_hintMutex );
  EncoderHints& eh = g_hints[&cfg];
  auto it = eh.fields.find( key );
  if( it == eh.fields.end() )
  {
    if( eh.fields.size() > 2048 )
    {
      // pictures retire in coding order: drop pairs well behind the current one
      for( auto e = eh.fields.begin(); e != eh.fields.end(); )
      {
        const int a = int( e->first >> 32 ), b = int( int32_t( e->first & 0xffffffff ) );
        e = std::max( a, b ) < cur.poc - 128 ? eh.fields.erase( e ) : std::next( e );
      }
    }
    std::vector<int16_t> f( size_t( gw ) * gh * 2 );
    const int rc = cfg.m_motionHintFnc( cfg.m_motionHintCtx, cur.poc, ref.poc, c.buf, c.stride, r.buf, r.stride,
                                        c.width, c.height, cfg.m_internalBitDepth[CH_L], MH_BLOCK_SIZE, f.data() );
    if( rc )
    {
      f.clear();
    }
    it = eh.fields.emplace( key, std::move( f ) ).first;
  }

  if( it->second.empty() )
  {
    return false;
  }
  const int gx = std::min( std::max( cx / MH_BLOCK_SIZE, 0 ), gw - 1 );
  const int gy = std::min( std::max( cy / MH_BLOCK_SIZE, 0 ), gh - 1 );
  dx = it->second[2 * ( size_t( gy ) * gw + gx )];
  dy = it->second[2 * ( size_t( gy ) * gw + gx ) + 1];
  return true;
}

void MotionHints::release( const VVEncCfg& cfg )
{
  std::lock_guard<std::mutex> lock( g_hintMutex );
  g_hints.erase( &cfg );
}

} // namespace vvenc

//! \}

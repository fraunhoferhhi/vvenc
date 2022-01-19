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

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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


/** \file     BinDecoder.cpp
 *  \brief    Low level binary symbol writer
 */


#include "BinDecoder.h"
#include "CommonLib/Rom.h"
#include "CommonLib/dtrace_next.h"

//! \ingroup DecoderLib
//! \{

namespace vvenc {

#define CNT_OFFSET 0


BinDecoderBase::BinDecoderBase( const BinProbModel* dummy )
  : Ctx         ( dummy )
  , m_Bitstream ( 0 )
  , m_Range     ( 0 )
  , m_Value     ( 0 )
  , m_bitsNeeded( 0 )
{}


void BinDecoderBase::init( InputBitstream* bitstream )
{
  m_Bitstream = bitstream;
}


void BinDecoderBase::uninit()
{
  m_Bitstream = 0;
}


void BinDecoderBase::start()
{
  CHECK( m_Bitstream->getNumBitsUntilByteAligned(), "Bitstream is not byte aligned." );
  m_Range       = 510;
  m_Value       = ( m_Bitstream->readByte() << 8 ) + m_Bitstream->readByte();
  m_bitsNeeded  = -8;
}


void BinDecoderBase::finish()
{
  unsigned lastByte;
  m_Bitstream->peekPreviousByte( lastByte );
  CHECK( ( ( lastByte << ( 8 + m_bitsNeeded ) ) & 0xff ) != 0x80,
        "No proper stop/alignment pattern at end of CABAC stream." );
}


void BinDecoderBase::reset( int qp, int initId )
{
  Ctx::init( qp, initId );
  start();
}


unsigned BinDecoderBase::decodeBinEP()
{
  m_Value            += m_Value;
  if( ++m_bitsNeeded >= 0 )
  {
    m_Value          += m_Bitstream->readByte();
    m_bitsNeeded      = -8;
  }

  unsigned bin = 0;
  unsigned SR  = m_Range << 7;
  if( m_Value >= SR )
  {
    m_Value   -= SR;
    bin        = 1;
  }
  DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n",  DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, bin );
  return bin;
}


unsigned BinDecoderBase::decodeBinsEP( unsigned numBins )
{
#if ENABLE_TRACING
  int numBinsOrig = numBins;
#endif

  if( m_Range == 256 )
  {
    return decodeAlignedBinsEP( numBins );
  }
  unsigned remBins = numBins;
  unsigned bins    = 0;
  while(   remBins > 8 )
  {
    m_Value     = ( m_Value << 8 ) + ( m_Bitstream->readByte() << ( 8 + m_bitsNeeded ) );
    unsigned SR =   m_Range << 15;
    for( int i = 0; i < 8; i++ )
    {
      bins += bins;
      SR  >>= 1;
      if( m_Value >= SR )
      {
        bins    ++;
        m_Value -= SR;
      }
    }
    remBins -= 8;
  }
  m_bitsNeeded   += remBins;
  m_Value       <<= remBins;
  if( m_bitsNeeded >= 0 )
  {
    m_Value      += m_Bitstream->readByte() << m_bitsNeeded;
    m_bitsNeeded -= 8;
  }
  unsigned SR = m_Range << ( remBins + 7 );
  for ( int i = 0; i < remBins; i++ )
  {
    bins += bins;
    SR  >>= 1;
    if( m_Value >= SR )
    {
      bins    ++;
      m_Value -= SR;
    }
  }
#if ENABLE_TRACING
  for( int i = 0; i < numBinsOrig; i++ )
  {
    DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, ( bins >> ( numBinsOrig - 1 - i ) ) & 1 );
  }
#endif
  return bins;
}

unsigned BinDecoderBase::decodeRemAbsEP(unsigned goRicePar, unsigned cutoff, int maxLog2TrDynamicRange)
{
  unsigned prefix = 0;
  {
    const unsigned  maxPrefix = 32 - maxLog2TrDynamicRange;
    unsigned        codeWord = 0;
    do
    {
      prefix++;
      codeWord = decodeBinEP();
    } while (codeWord && prefix < maxPrefix);
    prefix -= 1 - codeWord;
  }

  unsigned length = goRicePar, offset;
  if (prefix < cutoff)
  {
    offset = prefix << goRicePar;
  }
  else
  {
    offset = (((1 << (prefix - cutoff)) + cutoff - 1) << goRicePar);
    {
      length += (prefix == (32 - maxLog2TrDynamicRange) ? maxLog2TrDynamicRange - goRicePar : prefix - cutoff);
    }
  }
  return offset + decodeBinsEP(length);
}


unsigned BinDecoderBase::decodeBinTrm()
{
  m_Range    -= 2;
  unsigned SR = m_Range << 7;
  if( m_Value >= SR )
  {
    return 1;
  }
  else
  {
    if( m_Range < 256 )
    {
      m_Range += m_Range;
      m_Value += m_Value;
      if( ++m_bitsNeeded == 0 )
      {
        m_Value      += m_Bitstream->readByte();
        m_bitsNeeded  = -8;
      }
    }
    return 0;
  }
}


void BinDecoderBase::align()
{
  m_Range = 256;
}


unsigned BinDecoderBase::decodeAlignedBinsEP( unsigned numBins )
{
#if ENABLE_TRACING
  int numBinsOrig = numBins;
#endif
  unsigned remBins = numBins;
  unsigned bins    = 0;
  while(   remBins > 0 )
  {
    // The MSB of m_Value is known to be 0 because range is 256. Therefore:
    //   > The comparison against the symbol range of 128 is simply a test on the next-most-significant bit
    //   > "Subtracting" the symbol range if the decoded bin is 1 simply involves clearing that bit.
    //  As a result, the required bins are simply the <binsToRead> next-most-significant bits of m_Value
    //  (m_Value is stored MSB-aligned in a 16-bit buffer - hence the shift of 15)
    //
    //    m_Value = |0|V|V|V|V|V|V|V|V|B|B|B|B|B|B|B|        (V = usable bit, B = potential buffered bit (buffer refills when m_bitsNeeded >= 0))
    //
    unsigned binsToRead = std::min<unsigned>( remBins, 8 ); //read bytes if able to take advantage of the system's byte-read function
    unsigned binMask    = ( 1 << binsToRead ) - 1;
    unsigned newBins    = ( m_Value >> (15 - binsToRead) ) & binMask;
    bins                = ( bins    << binsToRead) | newBins;
    m_Value             = ( m_Value << binsToRead) & 0x7FFF;
    remBins            -= binsToRead;
    m_bitsNeeded       += binsToRead;
    if( m_bitsNeeded >= 0 )
    {
      m_Value          |= m_Bitstream->readByte() << m_bitsNeeded;
      m_bitsNeeded     -= 8;
    }
  }
#if ENABLE_TRACING
  for( int i = 0; i < numBinsOrig; i++ )
  {
    DTRACE( g_trace_ctx, D_CABAC, "%d" "  " "%d" "  " "EP=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, ( bins >> ( numBinsOrig - 1 - i ) ) & 1 );
  }
#endif
  return bins;
}




BinDecoder::BinDecoder()
  : BinDecoderBase( static_cast<const BinProbModel*>    ( nullptr ) )
  , m_Ctx         ( static_cast<CtxStore&>              ( *this   ) )
{}


unsigned BinDecoder::decodeBin( unsigned ctxId )
{
  BinProbModel& rcProbModel = m_Ctx[ctxId];
  unsigned      bin         = rcProbModel.mps();
  uint32_t      LPS         = rcProbModel.getLPS( m_Range );

 // DTRACE( g_trace_ctx, D_CABAC, "%d" " %d " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  " , DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), ctxId, m_Range, m_Range-LPS, LPS, ( unsigned int )( rcProbModel.state() ), m_Value < ( ( m_Range - LPS ) << 7 ) );
  DTRACE( g_trace_ctx, D_CABAC, " %d " "%d" "  " "[%d:%d]" "  " "%2d(MPS=%d)"  "  ", DTRACE_GET_COUNTER( g_trace_ctx, D_CABAC ), m_Range, m_Range - LPS, LPS, (unsigned int)( rcProbModel.state() ), m_Value < ( ( m_Range - LPS ) << 7 ) );

  m_Range   -=  LPS;
  uint32_t      SR          = m_Range << 7;
  if( m_Value < SR )
  {
    // MPS path
    if( m_Range < 256 )
    {
      int numBits   = rcProbModel.getRenormBitsRange( m_Range );
      m_Range     <<= numBits;
      m_Value     <<= numBits;
      m_bitsNeeded += numBits;
      if( m_bitsNeeded >= 0 )
      {
        m_Value      += m_Bitstream->readByte() << m_bitsNeeded;
        m_bitsNeeded -= 8;
      }
    }
  }
  else
  {
    bin = 1 - bin;
    // LPS path
    int numBits   = rcProbModel.getRenormBitsLPS( LPS );
    m_Value      -= SR;
    m_Value       = m_Value << numBits;
    m_Range       = LPS     << numBits;
    m_bitsNeeded += numBits;
    if( m_bitsNeeded >= 0 )
    {
      m_Value      += m_Bitstream->readByte() << m_bitsNeeded;
      m_bitsNeeded -= 8;
    }
  }
  rcProbModel.update( bin );
  //DTRACE_DECR_COUNTER( g_trace_ctx, D_CABAC );
  DTRACE_WITHOUT_COUNT( g_trace_ctx, D_CABAC, "  -  " "%d" "\n", bin );
  return  bin;
}

} // namespace vvenc

//! \}


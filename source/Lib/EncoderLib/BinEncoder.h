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
#pragma once

#include "CommonLib/Contexts.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/dtrace_next.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class BinStore
{
public:
  BinStore () : m_inUse(false), m_allocated(false)  {}
  ~BinStore()                                       {}

  void  reset   ()
  {
    if( m_inUse )
    {
      for( unsigned n = 0; n < Ctx::NumberOfContexts; n++ )
      {
        m_binBuffer[n].clear();
      }
    }
  }
  void  addBin  ( unsigned bin, unsigned ctxId )
  {
    if( m_inUse )
    {
      std::vector<bool>& binBuffer = m_binBuffer[ctxId];
      if( binBuffer.size() < m_maxNumBins )
      {
        binBuffer.push_back( bin == 1 );
      }
    }
  }

  void                      setUse      ( bool useStore )         { m_inUse = useStore; if(m_inUse){xCheckAlloc();} }
  bool                      inUse       ()                  const { return m_inUse; }
  const std::vector<bool>&  getBinVector( unsigned ctxId )  const { return m_binBuffer[ctxId]; }

private:
  void  xCheckAlloc()
  {
    if( !m_allocated )
    {
      m_binBuffer.resize( Ctx::NumberOfContexts );
      for( unsigned n = 0; n < Ctx::NumberOfContexts; n++ )
      {
        m_binBuffer[n].reserve( m_maxNumBins );
      }
      m_allocated = true;
    }
  }

private:
  static const std::size_t          m_maxNumBins = 100000;
  bool                              m_inUse;
  bool                              m_allocated;
  std::vector< std::vector<bool> >  m_binBuffer;
};


class BinEncIf : public Ctx
{
protected:
  template <class BinProbModel>
  BinEncIf( const BinProbModel* dummy ) : Ctx( dummy ) {}
public:
  virtual ~BinEncIf() {}
public:
  virtual void      init              ( OutputBitstream* bitstream )        = 0;
  virtual void      uninit            ()                                    = 0;
  virtual void      start             ()                                    = 0;
  virtual void      finish            ()                                    = 0;
  virtual void      restart           ()                                    = 0;
  virtual void      reset             ( int qp, int initId )                = 0;
public:
  virtual void      resetBits         ()                                    = 0;
  virtual uint64_t  getEstFracBits    ()                              const = 0;
  virtual unsigned  getNumBins        ( unsigned    ctxId )           const = 0;
public:
  virtual void      encodeBin         ( unsigned bin,   unsigned ctxId    ) = 0;
  virtual void      encodeBinEP       ( unsigned bin                      ) = 0;
  virtual void      encodeBinsEP      ( unsigned bins,  unsigned numBins  ) = 0;
  virtual void      encodeRemAbsEP    ( unsigned bins,
                                        unsigned goRicePar,
                                        unsigned cutoff,
                                        int      maxLog2TrDynamicRange    ) = 0;
  virtual void      encodeBinTrm      ( unsigned bin                      ) = 0;
  virtual void      align             ()                                    = 0;
public:
  virtual uint32_t  getNumBins        ()                                    = 0;
  virtual bool      isEncoding        ()                                    = 0;
  virtual unsigned  getNumWrittenBits ()                                    = 0;
public:
  virtual void            setBinStorage     ( bool b )                      = 0;
  virtual const BinStore* getBinStore       ()                        const = 0;
  virtual BinEncIf*       getTestBinEncoder ()                        const = 0;
};



class BinCounter
{
public:
  BinCounter();
  ~BinCounter() {}
public:
  void      reset   ();
  void      addCtx  ( unsigned ctxId )          { m_NumBinsCtx[ctxId]++; }
  void      addEP   ( unsigned num   )          { m_NumBinsEP+=num; }
  void      addEP   ()                          { m_NumBinsEP++; }
  void      addTrm  ()                          { m_NumBinsTrm++; }
  uint32_t  getAll  ()                  const;
  uint32_t  getCtx  ( unsigned ctxId )  const   { return m_NumBinsCtx[ctxId]; }
  uint32_t  getEP   ()                  const   { return m_NumBinsEP; }
  uint32_t  getTrm  ()                  const   { return m_NumBinsTrm; }
private:
  std::vector<uint32_t> m_CtxBinsCodedBuffer;
  uint32_t*             m_NumBinsCtx;
  uint32_t              m_NumBinsEP;
  uint32_t              m_NumBinsTrm;
};



class BinEncoderBase : public BinEncIf, public BinCounter
{
protected:
  BinEncoderBase ( const BinProbModel* dummy );
public:
  ~BinEncoderBase() {}
public:
  void      init    ( OutputBitstream* bitstream );
  void      uninit  ();
  void      start   ();
  void      finish  ();
  void      restart ();
  void      reset   ( int qp, int initId );

  void      resetBits           ();
  uint64_t  getEstFracBits      ()                    const { THROW( "not supported" ); return 0; }
  unsigned  getNumBins          ( unsigned ctxId )    const { return BinCounter::getCtx(ctxId); }

  void      encodeBinEP         ( unsigned bin                      );
  void      encodeBinsEP        ( unsigned bins,  unsigned numBins  );
  void      encodeRemAbsEP      ( unsigned bins,
                                  unsigned goRicePar,
                                  unsigned cutoff,
                                  int      maxLog2TrDynamicRange    );
  void      encodeBinTrm        ( unsigned bin                      );
  void      align               ();
  unsigned  getNumWrittenBits   () { return ( m_Bitstream->getNumberOfWrittenBits() + 8 * m_numBufferedBytes + 23 - m_bitsLeft ); }

  uint32_t  getNumBins          ()                          { return BinCounter::getAll(); }
  bool      isEncoding          ()                          { return true; }
protected:
  void      encodeAlignedBinsEP ( unsigned bins,  unsigned numBins  );
  void      writeOut            ();
protected:
  OutputBitstream* m_Bitstream;
  uint32_t         m_Low;
  uint32_t         m_Range;
  uint32_t         m_bufferedByte;
  int32_t          m_numBufferedBytes;
  int32_t          m_bitsLeft;
  BinStore         m_BinStore;
};



class BinEncoder : public BinEncoderBase
{
public:
  BinEncoder ();
  ~BinEncoder() {}
  void  encodeBin   ( unsigned bin, unsigned ctxId );
public:
  void            setBinStorage     ( bool b )          { m_BinStore.setUse(b); }
  const BinStore* getBinStore       ()          const   { return &m_BinStore; }
  BinEncIf*       getTestBinEncoder ()          const;
private:
  CtxStore& m_Ctx;
};





class BitEstimatorBase : public BinEncIf
{
protected:
  BitEstimatorBase ( const BinProbModel* dummy );
public:
  ~BitEstimatorBase() {}
public:
  void      init                ( OutputBitstream* bitstream )        {}
  void      uninit              ()                                    {}
  void      start               ()                                    { m_EstFracBits = 0; }
  void      finish              ()                                    {}
  void      restart             ()                                    { m_EstFracBits = (m_EstFracBits >> SCALE_BITS) << SCALE_BITS; }
  void      reset               ( int qp, int initId )                { Ctx::init( qp, initId ); m_EstFracBits = 0;}
public:
  void      resetBits           ()                                    { m_EstFracBits = 0; }

  uint64_t  getEstFracBits      ()                              const { return m_EstFracBits; }
  unsigned  getNumBins          ( unsigned ctxId )              const { THROW( "not supported for BitEstimator" ); return 0; }
public:
  void      encodeBinEP         ( unsigned bin                      ) { m_EstFracBits += BinProbModelBase::estFracBitsEP (); }
  void      encodeBinsEP        ( unsigned bins,  unsigned numBins  ) { m_EstFracBits += BinProbModelBase::estFracBitsEP ( numBins ); }
  void      encodeRemAbsEP      ( unsigned bins,
                                  unsigned goRicePar,
                                  unsigned cutoff,
                                  int      maxLog2TrDynamicRange    );
  void      align               ();
public:
  uint32_t  getNumBins          ()                                      { THROW("Not supported"); return 0; }
  bool      isEncoding          ()                                      { return false; }
  unsigned  getNumWrittenBits   ()                                      { /*THROW( "Not supported" );*/ return (uint32_t)( 0/*m_EstFracBits*//* >> SCALE_BITS*/ ); }

protected:
  uint64_t                m_EstFracBits;
};



class BitEstimator : public BitEstimatorBase
{
public:
  BitEstimator ();
  ~BitEstimator() {}
  void encodeBin    ( unsigned bin, unsigned ctxId )  { m_Ctx[ctxId].estFracBitsUpdate( bin, m_EstFracBits ); }
  void encodeBinTrm ( unsigned bin )                  { m_EstFracBits += BinProbModel::estFracBitsTrm( bin ); }
  void            setBinStorage     ( bool b )        {}
  const BinStore* getBinStore       ()          const { return 0; }
  BinEncIf*       getTestBinEncoder ()          const { return 0; }
private:
  CtxStore& m_Ctx;
};

} // namespace vvenc

//! \}


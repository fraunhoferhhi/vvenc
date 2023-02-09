/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     CABACReader.h
 *  \brief    Reader for low level syntax
 */

#pragma once

#include "BinDecoder.h"
#include "CommonLib/ContextModelling.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/UnitPartitioner.h"

//! \ingroup DecoderLib
//! \{

namespace vvenc {

class CABACReader : public DeriveCtx
{
public:
  CABACReader(BinDecoderBase& binDecoder) : m_BinDecoder(binDecoder), m_Bitstream(nullptr)
  { 
  }
  virtual ~CABACReader() {}

public:
  void        initCtxModels             ( Slice&                        slice );
  void        initBitstream             ( InputBitstream*               bitstream )           { m_Bitstream = bitstream; m_BinDecoder.init( m_Bitstream ); }
  const Ctx&  getCtx                    ()                                            const   { return m_BinDecoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinDecoder.getCtx();  }

public:
  // slice segment data (clause 7.3.8.1)
  bool        terminating_bit           ();
  void        remaining_bytes           ( bool                          noTrailingBytesExpected );

  // coding tree unit (clause 7.3.8.2)
  void        coding_tree_unit          ( CodingStructure&     cs,     const UnitArea& area,     int (&qps)[2],   unsigned  ctuRsAddr );

  // sao (clause 7.3.8.3)
  void        sao                       ( CodingStructure&     cs,     unsigned        ctuRsAddr );
  void        readAlfCtuFilterIndex     ( CodingStructure&     cs,     unsigned        ctuRsAddr);
  void        ccAlfFilterControlIdc     ( CodingStructure&     cs,     const ComponentID compID, const int curIdx, uint8_t *filterControlIdc, Position lumaPos, int filterCount);

  // coding (quad)tree (clause 7.3.8.4)
  void        coding_tree               ( CodingStructure&     cs,     Partitioner&    pm,       CUCtx& cuCtx, Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
  PartSplit   split_cu_mode             ( CodingStructure&     cs,     Partitioner&    pm );
  ModeType    mode_constraint           ( CodingStructure&     cs,     Partitioner&    pm,       const PartSplit splitMode );

  // coding unit (clause 7.3.8.5)
  void        coding_unit               ( CodingUnit&          cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        cu_skip_flag              ( CodingUnit&          cu );
  void        pred_mode                 ( CodingUnit&          cu );
  void        bdpcm_mode                ( CodingUnit&          cu,     const ComponentID compID );
  void        cu_pred_data              ( CodingUnit&          cu );
  void        cu_bcw_flag               ( CodingUnit&          cu );
  void        extend_ref_line           ( CodingUnit&          cu);
  void        intra_luma_pred_modes     ( CodingUnit&          cu );
  void        intra_chroma_pred_modes   ( CodingUnit&          cu );
  bool        intra_chroma_lmc_mode     ( CodingUnit&          cu );
  void        intra_chroma_pred_mode    ( CodingUnit&          cu );
  void        cu_residual               ( CodingUnit&          cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        rqt_root_cbf              ( CodingUnit&          cu );
  void        adaptive_color_transform  ( CodingUnit&          cu);
  void        sbt_mode                  ( CodingUnit&          cu );
  void        end_of_ctu                ( CodingUnit&          cu,     CUCtx&          cuCtx );
  void        mip_flag                  ( CodingUnit&          cu );
  void        mip_pred_modes            ( CodingUnit&          cu );
  void        mip_pred_mode             ( CodingUnit&          cu );
  void        cu_palette_info           ( CodingUnit&          cu,     ComponentID     compBegin, uint32_t numComp, CUCtx& cuCtx );
  void        cuPaletteSubblockInfo     ( CodingUnit&          cu,     ComponentID     compBegin, uint32_t numComp, int subSetId, uint32_t& prevRunPos, unsigned& prevRunType );

  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( CodingUnit&          cu,     MergeCtx&       mrgCtx );
  void        merge_flag                ( CodingUnit&          cu );
  void        merge_data                ( CodingUnit&          cu );
  void        affine_flag               ( CodingUnit&          cu );
  void        subblock_merge_flag       ( CodingUnit&          cu );
  void        merge_idx                 ( CodingUnit&          cu );
  void        mmvd_merge_idx            ( CodingUnit&          cu);
  void        imv_mode                  ( CodingUnit&          cu,     MergeCtx&       mrgCtx );
  void        affine_amvr_mode          ( CodingUnit&          cu,     MergeCtx&       mrgCtx );
  void        inter_pred_idc            ( CodingUnit&          cu );
  void        ref_idx                   ( CodingUnit&          cu,     RefPicList      eRefList );
  void        mvp_flag                  ( CodingUnit&          cu,     RefPicList      eRefList );
  void        Ciip_flag                 ( CodingUnit&          cu );
  void        smvd_mode                 ( CodingUnit&          cu );

  // transform tree (clause 7.3.8.8)
  void        transform_tree            ( CodingStructure& cs, Partitioner& pm, CUCtx& cuCtx, CodingUnit& cu, const PartSplit ispType = TU_NO_ISP, const int subTuIdx = -1 );
  bool        cbf_comp                  ( CodingUnit& cu,     const CompArea& area,     unsigned depth, const bool prevCbf = false, const bool useISP = false );

  // mvd coding (clause 7.3.8.9)
  void        mvd_coding                ( Mv &rMvd );

  // transform unit (clause 7.3.8.10)
  void        transform_unit            ( TransformUnit&       tu,     CUCtx&          cuCtx, Partitioner& pm,        const int subTuCounter = -1 );
  void        cu_qp_delta               ( CodingUnit&          cu,     int             predQP, int8_t& qp );
  void        cu_chroma_qp_offset       ( CodingUnit&          cu );

  // residual coding (clause 7.3.8.11)
  void        residual_coding           ( TransformUnit&       tu,     ComponentID     compID, CUCtx& cuCtx );
  void        ts_flag                   ( TransformUnit&       tu,     ComponentID     compID );
  void        mts_idx                   ( CodingUnit&          cu,     CUCtx&          cuCtx  );
  void        residual_lfnst_mode       ( CodingUnit&          cu,     CUCtx&          cuCtx  );
  void        isp_mode                  ( CodingUnit&          cu );
  int         last_sig_coeff            ( CoeffCodingContext&  cctx,   TransformUnit& tu, ComponentID   compID );
  void        residual_coding_subblock  ( CoeffCodingContext&  cctx,   TCoeffSig*      coeff, const int stateTransTable, int& state );
  void        residual_codingTS         ( TransformUnit&       tu,     ComponentID     compID );
  void        residual_coding_subblockTS( CoeffCodingContext&  cctx,   TCoeffSig*      coeff  );
  void        joint_cb_cr               ( TransformUnit&       tu,     const int cbfMask );


private:
  unsigned    unary_max_symbol          ( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  unsigned    unary_max_eqprob          (                                   unsigned maxSymbol );
  unsigned    exp_golomb_eqprob         ( unsigned count );
  unsigned    get_num_bits_read         () { return m_BinDecoder.getNumBitsRead(); }

  void        xReadTruncBinCode(uint32_t& symbol, uint32_t maxSymbol);
private:
  BinDecoderBase&    m_BinDecoder;
  InputBitstream*    m_Bitstream;
  Partitioner        m_partitioner[2];
};


class CABACDecoder
{
public:
  CABACDecoder() : m_CABACReader( m_BinDecoderStd ) {}
  CABACReader*   getCABACReader() { return &m_CABACReader; }
private:
  BinDecoder     m_BinDecoderStd;
  CABACReader    m_CABACReader;
};

} // namespace vvenc

//! \}


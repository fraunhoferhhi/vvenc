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
/** \file     CABACWriter.h
 *  \brief    Writer for low level syntax
 */

#pragma once

#include "CommonLib/BitStream.h"
#include "CommonLib/ContextModelling.h"
#include "BinEncoder.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class EncCu;
class CABACWriter : public DeriveCtx
{
public:
  CABACWriter(BinEncIf& binEncoder)   : m_BinEncoder(binEncoder), m_Bitstream(0)   { m_TestCtx = m_BinEncoder.getCtx(); }  
  virtual ~CABACWriter() {}

public:
  DeriveCtx&  getDeriveCtx              ()                                                    { return *this; }
  void        initCtxModels             ( const Slice&                  slice );
  SliceType   getCtxInitId              ( const Slice&                  slice );
  void        initBitstream             ( OutputBitstream*              bitstream )           { m_Bitstream = bitstream; m_BinEncoder.init( m_Bitstream ); }

  const Ctx&  getCtx                    ()                                            const   { return m_BinEncoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinEncoder.getCtx();  }

  void        start                     ()                                                    { m_BinEncoder.start(); }
  void        resetBits                 ()                                                    { m_BinEncoder.resetBits(); }
  uint64_t    getEstFracBits            ()                                            const   { return m_BinEncoder.getEstFracBits(); }
  uint32_t    getNumBins                ()                                                    { return m_BinEncoder.getNumBins(); }
  bool        isEncoding                ()                                                    { return m_BinEncoder.isEncoding(); }

public:
  // slice segment data (clause 7.3.8.1)
  void        end_of_slice              ();

  // coding tree unit (clause 7.3.8.2)
  void        coding_tree_unit          ( CodingStructure&              cs,       const UnitArea&   area,       int (&qps)[2],  unsigned ctuRsAddr,  bool skipSao = false, bool skipAlf = false );

  // sao (clause 7.3.8.3)
  void        sao                       ( const Slice&                  slice,    unsigned          ctuRsAddr );
  void        sao_block_pars            ( const SAOBlkParam&            saoPars,  const BitDepths&  bitDepths,  const bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo );
  void        sao_offset_pars           ( const SAOOffset&              ctbPars,  ComponentID       compID,     bool sliceEnabled,  int bitDepth );
  // coding (quad)tree (clause 7.3.8.4)
  void        coding_tree               ( const CodingStructure&        cs,       Partitioner&      pm,         CUCtx& cuCtx, Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
  void        split_cu_mode             ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm );
  void        mode_constraint           ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm,    const ModeType modeType );

  // coding unit (clause 7.3.8.5)
  void        coding_unit               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        cu_skip_flag              ( const CodingUnit&             cu );
  void        pred_mode                 ( const CodingUnit&             cu );
  void        bdpcm_mode                ( const CodingUnit&             cu,       const ComponentID compID );

  void        cu_pred_data              ( const CodingUnit&             cu );
  void        cu_bcw_flag               ( const CodingUnit&             cu );
  void        extend_ref_line           ( const CodingUnit&             cu );
  void        intra_luma_pred_modes     ( const CodingUnit&             cu );
  void        intra_luma_pred_mode      ( const CodingUnit&             cu,       const unsigned *mpmLst = nullptr );
  void        intra_chroma_pred_modes   ( const CodingUnit&             cu );
  void        intra_chroma_lmc_mode     ( const CodingUnit&             cu );
  void        intra_chroma_pred_mode    ( const CodingUnit&             cu );
  void        cu_residual               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        rqt_root_cbf              ( const CodingUnit&             cu );
  void        adaptive_color_transform  ( const CodingUnit&             cu);
  void        sbt_mode                  ( const CodingUnit&             cu );
  void        end_of_ctu                ( const CodingUnit&             cu,       CUCtx&            cuCtx );
  void        mip_flag                  ( const CodingUnit&             cu );
  void        mip_pred_modes            ( const CodingUnit&             cu );
  void        mip_pred_mode             ( const CodingUnit&             cu );
  void        cu_palette_info           ( const CodingUnit&             cu,       ComponentID       compBegin,     uint32_t numComp,          CUCtx&       cuCtx);
  void        cuPaletteSubblockInfo     ( const CodingUnit&             cu,       ComponentID       compBegin,     uint32_t numComp,          int subSetId,               uint32_t& prevRunPos,        unsigned& prevRunType );
  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( const CodingUnit&             cu );
  void        merge_flag                ( const CodingUnit&             cu );
  void        merge_data                ( const CodingUnit&             cu );
  void        affine_flag               ( const CodingUnit&             cu );
  void        subblock_merge_flag       ( const CodingUnit&             cu );
  void        merge_idx                 ( const CodingUnit&             cu );
  void        mmvd_merge_idx            ( const CodingUnit&             cu);
  void        imv_mode                  ( const CodingUnit&             cu );
  void        affine_amvr_mode          ( const CodingUnit&             cu );
  void        inter_pred_idc            ( const CodingUnit&             cu );
  void        ref_idx                   ( const CodingUnit&             cu,       RefPicList        eRefList );
  void        mvp_flag                  ( const CodingUnit&             cu,       RefPicList        eRefList );

  void        ciip_flag                 ( const CodingUnit&             cu );
  void        smvd_mode                 ( const CodingUnit&             cu );

  // transform tree (clause 7.3.8.8)
  void        transform_tree            ( const CodingStructure&        cs,       Partitioner&      pm,     CUCtx& cuCtx,                         const PartSplit ispType = TU_NO_ISP, const int subTuIdx = -1 );
  void        cbf_comp                  ( const CodingUnit&             cu,       bool              cbf,    const CompArea& area, unsigned depth, const bool prevCbf = false, const bool useISP = false );

  // mvd coding (clause 7.3.8.9)
  void        mvd_coding                ( const Mv &rMvd, int8_t imv );
  // transform unit (clause 7.3.8.10)
  void        transform_unit            ( const TransformUnit&          tu,       CUCtx&            cuCtx,  Partitioner& pm,       const int subTuCounter = -1 );
  void        cu_qp_delta               ( const CodingUnit&             cu,       int               predQP, const int8_t qp );
  void        cu_chroma_qp_offset       ( const CodingUnit&             cu );

  // residual coding (clause 7.3.8.11)
  void        residual_coding           ( const TransformUnit&          tu,       ComponentID       compID, CUCtx* cuCtx = nullptr );
  void        ts_flag                   ( const TransformUnit&          tu,       ComponentID       compID );
  void        mts_idx                   ( const CodingUnit&             cu,       CUCtx*            cuCtx  );
  void        residual_lfnst_mode       ( const CodingUnit&             cu,       CUCtx&            cuCtx );
  void        isp_mode                  ( const CodingUnit&             cu );
  void        last_sig_coeff            ( CoeffCodingContext&           cctx,     const TransformUnit& tu, ComponentID       compID );
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,     const TCoeffSig*  coeff, const int stateTransTable, int& state );
  void        residual_codingTS         ( const TransformUnit&          tu,       ComponentID       compID );
  void        residual_coding_subblockTS( CoeffCodingContext&           cctx,     const TCoeffSig*  coeff  );
  void        joint_cb_cr               ( const TransformUnit&          tu,       const int cbfMask );


  void        codeAlfCtuEnabled          ( CodingStructure& cs, ChannelType channel, AlfParam* alfParam, const int numCtus );
  void        codeAlfCtuEnabled          ( CodingStructure& cs, ComponentID compID, AlfParam* alfParam, const int numCtus );
  void        codeAlfCtuEnabledFlag      ( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx );
  void        codeAlfCtuFilterIndex      ( CodingStructure& cs, uint32_t ctuRsAddr );

  void        codeAlfCtuAlternatives     ( CodingStructure& cs, ChannelType channel, AlfParam* alfParam, const int numCtus );
  void        codeAlfCtuAlternatives     ( CodingStructure& cs, ComponentID compID, AlfParam* alfParam, const int numCtus );
  void        codeAlfCtuAlternative      ( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, const AlfParam* alfParam = NULL );
  void        codeCcAlfFilterControlIdc  ( uint8_t idcVal, CodingStructure &cs, const ComponentID compID, const int curIdx,
                                           const uint8_t *filterControlIdc, Position lumaPos, const int filterCount);

private:
  void        unary_max_symbol           ( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  void        unary_max_eqprob           ( unsigned symbol,                                   unsigned maxSymbol );
  void        exp_golomb_eqprob          ( unsigned symbol, unsigned count );

  void        xWriteTruncBinCode         ( uint32_t uiSymbol, uint32_t uiMaxSymbol);
private:
  BinEncIf&          m_BinEncoder;
  OutputBitstream*   m_Bitstream;
  Ctx                m_TestCtx;
  const ScanElement* m_scanOrder;

  Partitioner        m_partitioner[2];
  CtxTpl             m_tplBuf[MAX_TB_SIZEY * MAX_TB_SIZEY];
 };


} // namespace vvenc

//! \}


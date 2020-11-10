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
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,     const TCoeff*     coeff, const int stateTransTable, int& state );
  void        residual_codingTS         ( const TransformUnit&          tu,       ComponentID       compID );
  void        residual_coding_subblockTS( CoeffCodingContext&           cctx,     const TCoeff*     coeff  );
  void        joint_cb_cr               ( const TransformUnit&          tu,       const int cbfMask );


  void        codeAlfCtuEnabled          ( CodingStructure& cs, ChannelType channel, AlfParam* alfParam );
  void        codeAlfCtuEnabled          ( CodingStructure& cs, ComponentID compID, AlfParam* alfParam );
  void        codeAlfCtuEnabledFlag      ( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfParam* alfParam );
  void        codeAlfCtuFilterIndex      ( CodingStructure& cs, uint32_t ctuRsAddr, bool alfEnableLuma );

  void        codeAlfCtuAlternatives     ( CodingStructure& cs, ChannelType channel, AlfParam* alfParam );
  void        codeAlfCtuAlternatives     ( CodingStructure& cs, ComponentID compID, AlfParam* alfParam );
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
 };


} // namespace vvenc

//! \}


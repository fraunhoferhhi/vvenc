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
  bool        intra_chroma_lmc_mode     ( PredictionUnit&      pu );
  void        intra_chroma_pred_mode    ( PredictionUnit&      pu );
  void        cu_residual               ( CodingUnit&          cu,     Partitioner&    pm,       CUCtx& cuCtx );
  void        rqt_root_cbf              ( CodingUnit&          cu );
  void        adaptive_color_transform  ( CodingUnit&          cu);
  void        sbt_mode                  ( CodingUnit&          cu );
  void        end_of_ctu                ( CodingUnit&          cu,     CUCtx&          cuCtx );
  void        mip_flag                  ( CodingUnit&          cu );
  void        mip_pred_modes            ( CodingUnit&          cu );
  void        mip_pred_mode             ( PredictionUnit&      pu );
  void        cu_palette_info           ( CodingUnit&          cu,     ComponentID     compBegin, uint32_t numComp, CUCtx& cuCtx );
  void        cuPaletteSubblockInfo     ( CodingUnit&          cu,     ComponentID     compBegin, uint32_t numComp, int subSetId, uint32_t& prevRunPos, unsigned& prevRunType );

  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( PredictionUnit&      pu,     MergeCtx&       mrgCtx );
  void        merge_flag                ( PredictionUnit&      pu );
  void        merge_data                ( PredictionUnit&      pu );
  void        affine_flag               ( CodingUnit&          cu );
  void        subblock_merge_flag       ( CodingUnit&          cu );
  void        merge_idx                 ( PredictionUnit&      pu );
  void        mmvd_merge_idx            ( PredictionUnit&      pu);
  void        imv_mode                  ( CodingUnit&          cu,     MergeCtx&       mrgCtx );
  void        affine_amvr_mode          ( CodingUnit&          cu,     MergeCtx&       mrgCtx );
  void        inter_pred_idc            ( PredictionUnit&      pu );
  void        ref_idx                   ( PredictionUnit&      pu,     RefPicList      eRefList );
  void        mvp_flag                  ( PredictionUnit&      pu,     RefPicList      eRefList );
  void        Ciip_flag                 ( PredictionUnit&      pu );
  void        smvd_mode                 ( PredictionUnit&      pu );

  // transform tree (clause 7.3.8.8)
#if ISP_VVC
  void        transform_tree            ( CodingStructure& cs, Partitioner& pm, CUCtx& cuCtx, CodingUnit& cu, const PartSplit ispType = TU_NO_ISP, const int subTuIdx = -1 );
#else
  void        transform_tree            ( CodingStructure&     cs, Partitioner&    pm, CUCtx& cuCtx, const PartSplit ispType = TU_NO_ISP, const int subTuIdx = -1 );
#endif
  bool        cbf_comp                  ( CodingStructure&     cs,     const CompArea& area,     unsigned depth, const bool prevCbf = false, const bool useISP = false );

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
  void        residual_coding_subblock  ( CoeffCodingContext&  cctx,   TCoeff*         coeff, const int stateTransTable, int& state );
  void        residual_codingTS         ( TransformUnit&       tu,     ComponentID     compID );
  void        residual_coding_subblockTS( CoeffCodingContext&  cctx,   TCoeff*         coeff  );
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


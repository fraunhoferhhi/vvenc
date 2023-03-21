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


/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

static const int prefix_ctx[7]  = { 0, 0, 0, 3, 6, 10, 15 };

CoeffCodingContext::CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide, bool bdpcm, CtxTpl* tplBuf )
  : m_compID                    (component)
  , m_chType                    (toChannelType(m_compID))
  , m_width                     (tu.block(m_compID).width)
  , m_height                    (tu.block(m_compID).height)
  , m_log2CGWidth               ( g_log2SbbSize[ Log2(m_width) ][ Log2(m_height) ][0] )
  , m_log2CGHeight              ( g_log2SbbSize[ Log2(m_width) ][ Log2(m_height) ][1] )
  , m_log2CGSize                (m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups             (std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width) >> m_log2CGWidth)
  , m_heightInGroups            (std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) >> m_log2CGHeight)
  , m_log2WidthInGroups         (Log2(m_widthInGroups))
  , m_log2BlockWidth            (Log2(m_width))
  , m_log2BlockHeight           (Log2(m_height))
  , m_maxNumCoeff               (m_width * m_height)
  , m_signHiding                (signHide)
  , m_maxLog2TrDynamicRange     (tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
  , m_scan                      (getScanOrder( SCAN_GROUPED_4x4, m_log2BlockWidth, m_log2BlockHeight ))
  , m_scanCG                    (getScanOrder( SCAN_UNGROUPED  , Log2(m_widthInGroups), Log2(m_heightInGroups)))
  , m_CtxSetLastX               (Ctx::LastX[m_chType])
  , m_CtxSetLastY               (Ctx::LastY[m_chType])
  , m_maxLastPosX               (g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width) - 1])
  , m_maxLastPosY               (g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) - 1])
  , m_lastOffsetX               ((m_chType == CH_C) ? 0 :prefix_ctx[ m_log2BlockWidth ])
  , m_lastOffsetY               ((m_chType == CH_C) ? 0 :prefix_ctx[ m_log2BlockHeight ])
  , m_lastShiftX                ((m_chType == CH_C) ? Clip3( 0, 2, int( m_width >> 3) )  : (m_log2BlockWidth + 1) >> 2)
  , m_lastShiftY                ((m_chType == CH_C) ? Clip3( 0, 2, int( m_height >> 3) ) : (m_log2BlockHeight + 1) >> 2)
  , m_scanPosLast               (-1)
  , m_subSetId                  (-1)
  , m_subSetPos                 (-1)
  , m_subSetPosX                (-1)
  , m_subSetPosY                (-1)
  , m_minSubPos                 (-1)
  , m_maxSubPos                 (-1)
  , m_sigGroupCtxId             (-1)
  , m_tmplCpSum1                (-1)
  , m_tmplCpDiag                (-1)
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType+4] }
  , m_parFlagCtxSet             ( Ctx::ParFlag[m_chType] )
  , m_gtxFlagCtxSet             { Ctx::GtxFlag[m_chType], Ctx::GtxFlag[m_chType+2] }
  , m_sigGroupCtxIdTS           (-1)
  , m_tsSigFlagCtxSet           ( Ctx::TsSigFlag )
  , m_tsParFlagCtxSet           ( Ctx::TsParFlag )
  , m_tsGtxFlagCtxSet           ( Ctx::TsGtxFlag )
  , m_tsLrg1FlagCtxSet          (Ctx::TsLrg1Flag)
  , m_tsSignFlagCtxSet          (Ctx::TsResidualSign)
  , m_sigCoeffGroupFlag         ()
  , m_bdpcm                     (bdpcm)
  , m_tplBuf                    (tplBuf + m_width * m_height - 1)
{
  if( tplBuf && ( tu.mtsIdx[ component ] != MTS_SKIP || tu.cu->slice->tsResidualCodingDisabled ) )
    memset( tplBuf, 0, m_width * m_height * sizeof( CtxTpl ) );
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[m_subSetId].idx;
  m_subSetPosY              = m_subSetPos >> m_log2WidthInGroups;
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY << m_log2WidthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
  unsigned  CGPosY    = m_subSetPosY;
  unsigned  CGPosX    = m_subSetPosX;
  unsigned  sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
  unsigned  sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  m_sigGroupCtxId     = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );
  unsigned  sigLeft   = unsigned( CGPosX > 0 ? m_sigCoeffGroupFlag[m_subSetPos - 1              ] : false );
  unsigned  sigAbove  = unsigned( CGPosY > 0 ? m_sigCoeffGroupFlag[m_subSetPos - m_widthInGroups] : false );
  m_sigGroupCtxIdTS   = Ctx::TsSigCoeffGroup( sigLeft  + sigAbove );
}


void DeriveCtx::determineNeighborCus( const CodingStructure& cs, const UnitArea& ua, const ChannelType ch, const TreeType _treeType )
{
  const Position& posLuma    = ua.lumaPos();
  const Position& pos        = ch == CH_L ? posLuma : ua.chromaPos();
  const uint32_t curSliceIdx = cs.slice->independentSliceIdx;
  const uint32_t curTileIdx  = cs.pps->getTileIdx( posLuma );

  cuRestrictedLeft[ch]  = cs.getCURestricted( pos.offset(-1, 0), pos, curSliceIdx, curTileIdx, ch, _treeType );
  cuRestrictedAbove[ch] = cs.getCURestricted( pos.offset(0, -1), pos, curSliceIdx, curTileIdx, ch, _treeType );
}

void DeriveCtx::CtxSplit( const Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, const bool canSplit[6] ) const
{
  const ChannelType chType  = partitioner.chType;
  const CodingUnit* cuLeft  = cuRestrictedLeft[chType];
  const CodingUnit* cuAbove = cuRestrictedAbove[chType];

  ///////////////////////
  // CTX do split (0-8)
  ///////////////////////
  const unsigned widthCurr  = partitioner.currArea().blocks[chType].width;
  const unsigned heightCurr = partitioner.currArea().blocks[chType].height;

  ctxSpl = 0;

  if( cuLeft )
  {
    const unsigned heightLeft = cuLeft->blocks[chType].height;
    ctxSpl += ( heightLeft < heightCurr ? 1 : 0 );
  }
  if( cuAbove )
  {
    const unsigned widthAbove = cuAbove->blocks[chType].width;
    ctxSpl += ( widthAbove < widthCurr ? 1 : 0 );
  }

  unsigned numSplit = 0;
  if( canSplit[1] ) numSplit += 2;
  if( canSplit[2] ) numSplit += 1;
  if( canSplit[3] ) numSplit += 1;
  if( canSplit[4] ) numSplit += 1;
  if( canSplit[5] ) numSplit += 1;

  if( numSplit > 0 ) numSplit--;

  ctxSpl += 3 * ( numSplit >> 1 );

  //////////////////////////
  // CTX is qt split (0-5)
  //////////////////////////
  ctxQt =  ( cuLeft  && cuLeft->qtDepth  > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += partitioner.currQtDepth < 2 ? 0 : 3;

  ////////////////////////////
  // CTX is ver split (0-4)
  ////////////////////////////
  ctxHv = 0;

  const unsigned numHor = ( canSplit[2] ? 1 : 0 ) + ( canSplit[4] ? 1 : 0 );
  const unsigned numVer = ( canSplit[3] ? 1 : 0 ) + ( canSplit[5] ? 1 : 0 );

  if( numVer == numHor )
  {
    const Area& area = partitioner.currArea().blocks[chType];

    const unsigned wAbove       = cuAbove ? cuAbove->blocks[chType].width  : 1;
    const unsigned hLeft        = cuLeft  ? cuLeft ->blocks[chType].height : 1;

    const unsigned depAbove     = area.width / wAbove;
    const unsigned depLeft      = area.height / hLeft;

    if( depAbove == depLeft || !cuLeft || !cuAbove ) ctxHv = 0;
    else if( depAbove < depLeft ) ctxHv = 1;
    else ctxHv = 2;
  }
  else if( numVer < numHor )
  {
    ctxHv = 3;
  }
  else
  {
    ctxHv = 4;
  }

  //////////////////////////
  // CTX is h/v bt (0-3)
  //////////////////////////
  ctxHorBt = ( partitioner.currMtDepth <= 1 ? 1 : 0 );
  ctxVerBt = ( partitioner.currMtDepth <= 1 ? 3 : 2 );
}



void MergeCtx::setMergeInfo( CodingUnit& cu, int candIdx ) const
{
  CHECK( candIdx >= numValidMergeCand, "Merge candidate does not exist" );
  cu.regularMergeFlag           = !(cu.ciip || cu.geo);
  cu.mergeFlag                  = true;
  cu.mmvdMergeFlag              = false;
  cu.interDir                   = interDirNeighbours[candIdx];
  cu.imv                        = (!cu.geo && useAltHpelIf[candIdx]) ? IMV_HPEL : 0;
  cu.mergeIdx                   = candIdx;
  cu.mergeType                  = mrgTypeNeighbours[candIdx];
  cu.mv     [REF_PIC_LIST_0][0] = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  cu.mv     [REF_PIC_LIST_1][0] = mvFieldNeighbours[(candIdx << 1) + 1].mv;
  cu.mvd    [REF_PIC_LIST_0][0] = Mv();
  cu.mvd    [REF_PIC_LIST_1][0] = Mv();
  cu.refIdx [REF_PIC_LIST_0]    = mvFieldNeighbours[( candIdx << 1 ) + 0].refIdx;
  cu.refIdx [REF_PIC_LIST_1]    = mvFieldNeighbours[( candIdx << 1 ) + 1].refIdx;
  cu.mvpIdx [REF_PIC_LIST_0]    = NOT_VALID;
  cu.mvpIdx [REF_PIC_LIST_1]    = NOT_VALID;
  cu.mvpNum [REF_PIC_LIST_0]    = NOT_VALID;
  cu.mvpNum [REF_PIC_LIST_1]    = NOT_VALID;
  if( CU::isIBC( cu ) )
  {
    cu.imv  = cu.imv == IMV_HPEL ? 0 : cu.imv;
  }
  cu.BcwIdx = ( interDirNeighbours[candIdx] == 3 ) ? BcwIdx[candIdx] : BCW_DEFAULT;

  CU::restrictBiPredMergeCandsOne(cu);
  cu.mcControl = 0;
}

void MergeCtx::setMmvdMergeCandiInfo(CodingUnit& cu, int candIdx) const
{
  const Slice &slice = *cu.cs->slice;
  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
  const int refMvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
  int fPosGroup = 0;
  int fPosBaseIdx = 0;
  int fPosStep = 0;
  int tempIdx = 0;
  int fPosPosition = 0;
  Mv tempMv[2];

  tempIdx = candIdx;
  fPosGroup = tempIdx / (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  tempIdx = tempIdx - fPosGroup * (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  fPosBaseIdx = tempIdx / MMVD_MAX_REFINE_NUM;
  tempIdx = tempIdx - fPosBaseIdx * (MMVD_MAX_REFINE_NUM);
  fPosStep = tempIdx / 4;
  fPosPosition = tempIdx - fPosStep * (4);
  int offset = refMvdCands[fPosStep];
  if ( cu.slice->picHeader->disFracMMVD )
  {
    offset <<= 2;
  }
  const int refList0 = mmvdBaseMv[fPosBaseIdx][0].refIdx;
  const int refList1 = mmvdBaseMv[fPosBaseIdx][1].refIdx;

  if ((refList0 != -1) && (refList1 != -1))
  {
    const int poc0 = slice.getRefPOC(REF_PIC_LIST_0, refList0);
    const int poc1 = slice.getRefPOC(REF_PIC_LIST_1, refList1);
    const int currPoc = slice.poc;
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
    if ((poc0 - currPoc) == (poc1 - currPoc))
    {
      tempMv[1] = tempMv[0];
    }
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
    {
      const int scale = CU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
      tempMv[1] = tempMv[0];
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->isLongTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->isLongTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          tempMv[0] = tempMv[1];
        }
        else
        {
          tempMv[0].set(-1 * tempMv[1].hor, -1 * tempMv[1].ver);
        }
      }
      else
      tempMv[0] = tempMv[1].scaleMv(scale);
    }
    else
    {
      const int scale = CU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->isLongTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->isLongTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          tempMv[1] = tempMv[0];
        }
        else
        {
          tempMv[1].set(-1 * tempMv[0].hor, -1 * tempMv[0].ver);
        }
      }
      else
      tempMv[1] = tempMv[0].scaleMv(scale);
    }

    cu.interDir = 3;
    cu.mv[REF_PIC_LIST_0][0]  = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    cu.refIdx[REF_PIC_LIST_0] = refList0;
    cu.mv[REF_PIC_LIST_1][0]  = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    cu.refIdx[REF_PIC_LIST_1] = refList1;
  }
  else if (refList0 != -1)
  {
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
    cu.interDir = 1;
    cu.mv[REF_PIC_LIST_0][0]  = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    cu.refIdx[REF_PIC_LIST_0] = refList0;
    cu.mv[REF_PIC_LIST_1][0]  = Mv(0, 0);
    cu.refIdx[REF_PIC_LIST_1] = -1;
  }
  else if (refList1 != -1)
  {
    if (fPosPosition == 0)
    {
      tempMv[1] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[1] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[1] = Mv(0, offset);
    }
    else
    {
      tempMv[1] = Mv(0, -offset);
    }
    cu.interDir = 2;
    cu.mv[REF_PIC_LIST_0][0]  = Mv(0, 0);
    cu.refIdx[REF_PIC_LIST_0] = -1;
    cu.mv[REF_PIC_LIST_1][0]  = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    cu.refIdx[REF_PIC_LIST_1] = refList1;
  }

  cu.mmvdMergeFlag          = true;
  cu.mmvdMergeIdx           = candIdx;
  cu.mergeFlag              = true;
  cu.regularMergeFlag       = true;
  cu.mergeIdx               = candIdx;
  cu.mergeType              = MRG_TYPE_DEFAULT_N;
  cu.mvd[REF_PIC_LIST_0][0] = Mv();
  cu.mvd[REF_PIC_LIST_1][0] = Mv();
  cu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  cu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  cu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  cu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  cu.imv                    = mmvdUseAltHpelIf[fPosBaseIdx] ? IMV_HPEL : 0;

  cu.BcwIdx = (interDirNeighbours[fPosBaseIdx] == 3) ? BcwIdx[fPosBaseIdx] : BCW_DEFAULT;

  for (int refList = 0; refList < 2; refList++)
  {
    if (cu.refIdx[refList] >= 0)
    {
      cu.mv[refList][0].clipToStorageBitDepth();
    }
  }


  CU::restrictBiPredMergeCandsOne(cu);
}

unsigned DeriveCtx::CtxMipFlag( const CodingUnit& cu ) const
{
  unsigned ctxId = 0;
  const CodingUnit *cuLeft = cuRestrictedLeft[CH_L];
  ctxId = (cuLeft && cuLeft->mipFlag) ? 1 : 0;

  const CodingUnit *cuAbove = cuRestrictedAbove[CH_L];
  ctxId += (cuAbove && cuAbove->mipFlag) ? 1 : 0;

  ctxId  = (cu.lwidth() > 2*cu.lheight() || cu.lheight() > 2*cu.lwidth()) ? 3 : ctxId;

  return ctxId;
}

} // namespace vvenc

//! \}


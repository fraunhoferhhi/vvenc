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


/** \file     Slice.cpp
    \brief    slice header and SPS class
*/

#include "CommonDef.h"
#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "UnitTools.h"
#include "dtrace_next.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

Slice::Slice()
  : ppsId                               ( -1 )
//  , picOutputFlag                       ( true )
  , poc                                 ( 0 )
  , lastIDR                             ( 0 )
  , prevGDRInSameLayerPOC               ( 0 )
  , associatedIRAP                      ( 0 )
  , associatedIRAPType                  ( VVENC_NAL_UNIT_INVALID )
  , enableDRAPSEI                       ( false )
  , useLTforDRAP                        ( false )
  , isDRAP                              ( false )
  , latestDRAPPOC                       ( 0 )
  , colourPlaneId                       ( 0 )
  , pictureHeaderInSliceHeader          ( true )
  , nuhLayerId                          ( 0 )
  , nalUnitType                         ( VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL )
  , sliceType                           ( VVENC_I_SLICE )
  , sliceQp                             ( 0 )
  , chromaQpAdjEnabled                  ( false )
  , lmcsEnabled                         ( 0 )
  , explicitScalingListUsed             ( 0 )
  , deblockingFilterDisable             ( false )
  , deblockingFilterOverride            ( false )
  , deblockingFilterBetaOffsetDiv2      { 0 }
  , deblockingFilterTcOffsetDiv2        { 0 }
  , depQuantEnabled                     ( false )
  , signDataHidingEnabled               ( false )
  , tsResidualCodingDisabled            ( false )
  , pendingRasInit                      ( false )
  , checkLDC                            ( false )
  , biDirPred                           ( false )
  , lmChromaCheckDisable                { false }
  , symRefIdx                           { -1, -1 }
  , vps                                 ( nullptr )
  , dci                                 ( nullptr )
  , sps                                 ( nullptr )
  , pps                                 ( nullptr )
  , pic                                 ( nullptr )
  , picHeader                           ( nullptr )
  , colFromL0Flag                       ( true )
  , colRefIdx                           ( 0 )
  , TLayer                              ( 0 )
  , TLayerSwitchingFlag                 ( false )
  , independentSliceIdx                 ( 0 )
  , cabacInitFlag                       ( false )
  , sliceSubPicId                       ( 0 )
  , encCABACTableIdx                    ( VVENC_I_SLICE )
  , numAps                     ( 0 )
  , chromaApsId                ( -1 )
  , ccAlfCbEnabled             ( false )
  , ccAlfCrEnabled             ( false )
  , ccAlfCbApsId               ( -1 )
  , ccAlfCrApsId               ( -1 )
  , isLossless                          ( false )
{
  ::memset( saoEnabled,              0, sizeof( saoEnabled ) );
  ::memset( numRefIdx,               0, sizeof( numRefIdx ) );
  ::memset( sliceChromaQpDelta,      0, sizeof( sliceChromaQpDelta ) );
  ::memset( lambdas,                 0, sizeof( lambdas ) );
  ::memset( alfEnabled,              0, sizeof( alfEnabled ) );
  ::memset( alfAps,                  0, sizeof( alfAps ) );
  ::memset( refPicList,              0, sizeof( refPicList ) );
  ::memset( refPOCList,              0, sizeof( refPOCList ) );
  ::memset( isUsedAsLongTerm,        0, sizeof( isUsedAsLongTerm ) );
  ::memset( ccAlfFilterControl,      0, sizeof( ccAlfFilterControl ) );

  for ( int idx = 0; idx < MAX_NUM_REF; idx++ )
  {
    list1IdxToList0Idx[idx] = -1;
  }

  for ( int idx = 0; idx < NUM_REF_PIC_LIST_01; idx++ )
  {
    rpl[idx]                = nullptr;
    rplIdx[idx]             = -1;
  }
  
  resetWpScaling();
}

Slice::~Slice()
{
}


void Slice::resetSlicePart()
{
  colFromL0Flag        = true;
  colRefIdx            = 0;
  checkLDC             = false;
  biDirPred            = false;
  symRefIdx[0]         = -1;
  symRefIdx[1]         = -1;
  cabacInitFlag        = false;

  substreamSizes.clear();

  ::memset( numRefIdx,           0, sizeof( numRefIdx ) );
  ::memset( sliceChromaQpDelta,  0, sizeof( sliceChromaQpDelta ) );
  ::memset( lambdas,             0, sizeof( lambdas ) );
  ::memset( alfEnabled,          0, sizeof( alfEnabled ) );

  ccAlfFilterParam.reset();
  ccAlfCbEnabled = false;
  ccAlfCrEnabled = false;

  sliceMap = SliceMap();
}

void Slice::setDefaultClpRng( const SPS& sps )
{
  CHECK( sps.bitDepths[CH_L] != sps.bitDepths[CH_C], "Different luma/chroma bitdepths not supported!" );

  clpRngs.bd = sps.bitDepths[CH_L];
}


bool Slice::getRapPicFlag() const
{
  return nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP
      || nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA;
}


void  Slice::sortPicList        (PicList& rcListPic)
{
  Picture*    picExtract;
  Picture*    picInsert;

  PicList::iterator    iterPicExtract;
  PicList::iterator    iterPicExtract_1;
  PicList::iterator    iterPicInsert;

  for (int i = 1; i < (int)(rcListPic.size()); i++)
  {
    iterPicExtract = rcListPic.begin();
    for (int j = 0; j < i; j++)
    {
      iterPicExtract++;
    }
    picExtract = *(iterPicExtract);

    iterPicInsert = rcListPic.begin();
    while (iterPicInsert != iterPicExtract)
    {
      picInsert = *(iterPicInsert);
      if (picInsert->getPOC() >= picExtract->getPOC())
      {
        break;
      }

      iterPicInsert++;
    }

    iterPicExtract_1 = iterPicExtract;    iterPicExtract_1++;

    //  swap iterPicExtract and iterPicInsert, iterPicExtract = curr. / iterPicInsert = insertion position
    rcListPic.insert( iterPicInsert, iterPicExtract, iterPicExtract_1 );
    rcListPic.erase( iterPicExtract );
  }
}


Picture* Slice::xGetLongTermRefPic( const PicList& rcListPic, int poc, bool pocHasMsb)
{
  PicList::const_iterator  iterPic = rcListPic.begin();
  Picture*           picCand = *(iterPic);
  Picture*           pcStPic = picCand;

  int pocCycle = 1 << sps->bitsForPOC;
  if (!pocHasMsb)
  {
    poc = poc & (pocCycle - 1);
  }

  while ( iterPic != rcListPic.end() )
  {
    picCand = *(iterPic);
    if (picCand && picCand->poc != this->poc && picCand->isReferenced)
    {
      int picPoc = picCand->poc;
      if (!pocHasMsb)
      {
        picPoc = picPoc & (pocCycle - 1);
      }

      if (poc == picPoc)
      {
        if(picCand->isLongTerm)
        {
          return picCand;
        }

        pcStPic = picCand;
        break;
      }
    }

    iterPic++;
  }

  return pcStPic;
}

void Slice::setRefPOCList       ()
{
  for (int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (int iNumRefIdx = 0; iNumRefIdx < numRefIdx[iDir]; iNumRefIdx++)
    {
      refPOCList[iDir][iNumRefIdx] = refPicList[iDir][iNumRefIdx]->getPOC();
    }
  }

}

void Slice::setSMVDParam()
{
  if ( sps->SMVD && checkLDC == false && picHeader->mvdL1Zero == false )
  {
    int currPOC  = poc;
    int forwardPOC = poc;
    int backwardPOC = poc;
    int ref = 0;
    int refIdx0 = -1;
    int refIdx1 = -1;

    // search nearest forward POC in List 0
    for (ref = 0; ref < numRefIdx[REF_PIC_LIST_0]; ref++)
    {
      const int refPoc = getRefPic(REF_PIC_LIST_0, ref)->getPOC();
      const bool isRefLongTerm = getRefPic(REF_PIC_LIST_0, ref)->isLongTerm;
      if (refPoc < currPOC && (refPoc > forwardPOC || refIdx0 == -1) && !isRefLongTerm)
      {
        forwardPOC = refPoc;
        refIdx0 = ref;
      }
    }

    // search nearest backward POC in List 1
    for (ref = 0; ref < numRefIdx[REF_PIC_LIST_1]; ref++)
    {
      const int refPoc = getRefPic(REF_PIC_LIST_1, ref)->getPOC();
      const bool isRefLongTerm = getRefPic(REF_PIC_LIST_1, ref)->isLongTerm;
      if (refPoc > currPOC && (refPoc < backwardPOC || refIdx1 == -1) && !isRefLongTerm)
      {
        backwardPOC = refPoc;
        refIdx1 = ref;
      }
    }

    if (!(forwardPOC < currPOC && backwardPOC > currPOC))
    {
      forwardPOC = currPOC;
      backwardPOC = currPOC;
      refIdx0 = -1;
      refIdx1 = -1;

      // search nearest backward POC in List 0
      for (ref = 0; ref < numRefIdx[REF_PIC_LIST_0]; ref++)
      {
        const int refPoc = getRefPic(REF_PIC_LIST_0, ref)->getPOC();
        const bool isRefLongTerm = getRefPic(REF_PIC_LIST_0, ref)->isLongTerm;
        if (refPoc > currPOC && (refPoc < backwardPOC || refIdx0 == -1) && !isRefLongTerm)
        {
          backwardPOC = refPoc;
          refIdx0 = ref;
        }
      }

      // search nearest forward POC in List 1
      for (ref = 0; ref < numRefIdx[REF_PIC_LIST_1]; ref++)
      {
        const int refPoc = getRefPic(REF_PIC_LIST_1, ref)->getPOC();
        const bool isRefLongTerm = getRefPic(REF_PIC_LIST_1, ref)->isLongTerm;
        if (refPoc < currPOC && (refPoc > forwardPOC || refIdx1 == -1) && !isRefLongTerm)
        {
          forwardPOC = refPoc;
          refIdx1 = ref;
        }
      }
    }

    if (forwardPOC < currPOC && backwardPOC > currPOC)
    {
      biDirPred    = true;
      symRefIdx[0] = refIdx0;
      symRefIdx[1] = refIdx1;
      return;
    }
  }

  biDirPred    = false;
  symRefIdx[0] = -1;
  symRefIdx[1] = -1;
}

void Slice::setList1IdxToList0Idx()
{
  int idxL0, idxL1;
  for ( idxL1 = 0; idxL1 < numRefIdx[ REF_PIC_LIST_1 ]; idxL1++ )
  {
    list1IdxToList0Idx[idxL1] = -1;
    for ( idxL0 = 0; idxL0 < numRefIdx[ REF_PIC_LIST_0 ]; idxL0++ )
    {
      if ( refPicList[REF_PIC_LIST_0][idxL0]->getPOC() == refPicList[REF_PIC_LIST_1][idxL1]->getPOC() )
      {
        list1IdxToList0Idx[idxL1] = idxL0;
        break;
      }
    }
  }
}

void Slice::constructRefPicList(const PicList& rcListPic, bool extBorder, const bool usingLongTerm)
{
  ::memset(isUsedAsLongTerm, 0, sizeof(isUsedAsLongTerm));
  if (sliceType == VVENC_I_SLICE)
  {
    ::memset(refPicList, 0, sizeof(refPicList));
    ::memset(numRefIdx, 0, sizeof(numRefIdx));
    return;
  }

  Picture*  pcRefPic = NULL;
  uint32_t numOfActiveRef = 0;
  //construct L0
  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    RefPicList eRefList = RefPicList(refList);
    numOfActiveRef = numRefIdx[ eRefList ];
    for (int ii = 0; ii < numOfActiveRef; ii++)
    {
      if (!rpl[eRefList]->isLongtermRefPic[ii])
      {
        int poc_ = poc + rpl[eRefList]->refPicIdentifier[ii];

        PicList::const_iterator  iterPic = rcListPic.begin();
        pcRefPic   = *(iterPic);

        while ( iterPic != rcListPic.end() )
        {
          if(pcRefPic->getPOC() == poc_)
          {
            break;
          }
          iterPic++;
          pcRefPic = *(iterPic);
        }

        if(usingLongTerm)
          pcRefPic->isLongTerm = false;
      }
      else
      {
        CHECK(!usingLongTerm, "Wrong state: using long term when it's not supported by the encoder configuration");
        int pocBits = sps->bitsForPOC;
        int pocMask = (1 << pocBits) - 1;
        int ltrpPoc = rpl[eRefList]->refPicIdentifier[ii] & pocMask;
        ltrpPoc += rpl[eRefList]->deltaPocMSBPresent[ii] ? (pocMask + 1) * rpl[eRefList]->deltaPocMSBCycleLT[ii] : 0;
        pcRefPic = xGetLongTermRefPic(rcListPic, ltrpPoc, rpl[eRefList]->deltaPocMSBPresent[ii]);
        pcRefPic->isLongTerm = true;
      }
      if ( extBorder )
      {
        pcRefPic->extendPicBorder();
      }
      refPicList[eRefList][ii] = pcRefPic;
      isUsedAsLongTerm[eRefList][ii] = usingLongTerm ? pcRefPic->isLongTerm: false;
    }
  }
}

void Slice::updateRefPicCounter( int step )
{
  for ( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    int numOfActiveRef = numRefIdx[ refList ];
    for ( int i = 0; i < numOfActiveRef; i++ )
    {
      refPicList[ refList ][ i ]->refCounter += step;
    }
  }
}

bool Slice::checkAllRefPicsReconstructed() const
{
  for ( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    int numOfActiveRef = numRefIdx[ refList ];
    for ( int i = 0; i < numOfActiveRef; i++ )
    {
      if ( ! refPicList[ refList ][ i ]->isReconstructed )
      {
        return false;
      }
    }
  }

  return true;
}

bool Slice::checkAllRefPicsAccessible() const
{
  for ( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    int numOfActiveRef = numRefIdx[ refList ];
    for ( int i = 0; i < numOfActiveRef; i++ )
    {
      if ( ! refPicList[ refList ][ i ]->isInProcessList )
      {
        return false;
      }
    }
  }

  return true;
}

void Slice::checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic) const
{
  Slice* curSlice   = pic->slices[ curSliceSegmentIdx ];
  int currColRefPOC =  curSlice->getRefPOC( RefPicList( 1 - curSlice->colFromL0Flag ), curSlice->colRefIdx );

  for( int i = curSliceSegmentIdx - 1; i >= 0; i-- )
  {
    const Slice* preSlice = pic->slices[i];
    if( preSlice->sliceType != VVENC_I_SLICE )
    {
      const int preColRefPOC  = preSlice->getRefPOC( RefPicList( 1 - preSlice->colFromL0Flag ), preSlice->colRefIdx );
      if( currColRefPOC != preColRefPOC )
      {
        THROW( "Collocated_ref_idx shall always be the same for all slices of a coded picture!" );
      }
      else
      {
        break;
      }
    }
  }
}

void Slice::checkCRA(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, int& pocCRA, vvencNalUnitType& associatedIRAPType, PicList& rcListPic)
{
  if (pocCRA < MAX_UINT && poc > pocCRA)
  {
    uint32_t numRefPic = pRPL0->numberOfShorttermPictures + pRPL0->numberOfLongtermPictures;
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL0->isLongtermRefPic[i])
      {
        CHECK(poc + pRPL0->refPicIdentifier[i] < pocCRA, "Invalid state");
      }
      else
      {
        CHECK(xGetLongTermRefPic(rcListPic, pRPL0->refPicIdentifier[i], pRPL0->deltaPocMSBPresent[i])->getPOC() < pocCRA, "Invalid state");
      }
    }
    numRefPic = pRPL1->numberOfShorttermPictures + pRPL1->numberOfLongtermPictures;
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL1->isLongtermRefPic[i])
      {
        CHECK(poc + pRPL1->refPicIdentifier[i] < pocCRA, "Invalid state");
      }
      else
      {
        CHECK(xGetLongTermRefPic(rcListPic, pRPL1->refPicIdentifier[i], pRPL1->deltaPocMSBPresent[i])->getPOC() < pocCRA, "Invalid state");
      }
    }
  }
  if (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL || nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP) // IDR picture found
  {
    pocCRA = poc;
    associatedIRAPType = nalUnitType;
  }
  else if (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA) // CRA picture found
  {
    pocCRA = poc;
    associatedIRAPType = nalUnitType;
  }
}

/** Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
 * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
 * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
 * \param rcListPic reference to the reference picture list
 * This function marks the reference pictures as "unused for reference" in the following conditions.
 * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
 * are marked as "unused for reference"
 *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
 * Otherwise
 *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
 *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
 *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
 *    the bRefreshPending flag to false.
 *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
 *    reference of the current picture.
 * Note that the current picture is already placed in the reference list and its marking is not changed.
 * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
 */
void Slice::setDecodingRefreshMarking( int& pocCRA, bool& bRefreshPending, const PicList& rcListPic )
{
  const bool bEfficientFieldIRAPEnabled = true;
  Picture* rpcPic;
  int      pocCurr = poc;

  if ( nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL
    || nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP)  // IDR picture
  {
    // mark all pictures as not used for reference
    PicList::const_iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic);
      if (rpcPic->getPOC() != pocCurr)
      {
        rpcPic->isReferenced = false;
      }
      iterPic++;
    }
    if (bEfficientFieldIRAPEnabled)
    {
      bRefreshPending = true;
    }
  }
  else // CRA or No DR
  {
    if(bEfficientFieldIRAPEnabled && (associatedIRAPType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP || associatedIRAPType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL))
    {
      if (bRefreshPending==true && pocCurr > lastIDR) // IDR reference marking pending
      {
        PicList::const_iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != lastIDR)
          {
            rpcPic->isReferenced = false;
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    else
    {
      if (bRefreshPending==true && pocCurr > pocCRA) // CRA reference marking pending
      {
        PicList::const_iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != pocCRA)
          {
            rpcPic->isReferenced = false;
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    if ( nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
    {
      bRefreshPending = true;
      pocCRA = pocCurr;
    }
  }
}

void Slice::copySliceInfo( const Slice* slice, bool cpyAlmostAll)
{
  CHECK(!slice, "Source is NULL");

  int i, j;

  poc                             = slice->poc;
  nalUnitType                     = slice->nalUnitType;
  sliceType                       = slice->sliceType;
  sliceQp                         = slice->sliceQp;
  chromaQpAdjEnabled              = slice->chromaQpAdjEnabled;
  deblockingFilterDisable         = slice->deblockingFilterDisable;
  deblockingFilterOverride        = slice->deblockingFilterOverride;
  for( int comp = 0; comp < MAX_NUM_COMP; comp++ )
  {
    deblockingFilterBetaOffsetDiv2[comp]  = slice->deblockingFilterBetaOffsetDiv2[comp];
    deblockingFilterTcOffsetDiv2[comp]    = slice->deblockingFilterTcOffsetDiv2[comp];
  }

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    numRefIdx[i] = slice->numRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)
  {
    list1IdxToList0Idx[i] = slice->list1IdxToList0Idx[i];
  }

  checkLDC      = slice->checkLDC;

  biDirPred     = slice->biDirPred;
  symRefIdx[0]  = slice->symRefIdx[0];
  symRefIdx[1]  = slice->symRefIdx[1];

  for (uint32_t component = 0; component < MAX_NUM_COMP; component++)
  {
    sliceChromaQpDelta[component] = slice->sliceChromaQpDelta[component];
  }
  sliceChromaQpDelta[COMP_JOINT_CbCr] = slice->sliceChromaQpDelta[COMP_JOINT_CbCr];
  if( cpyAlmostAll )
  {
    for( i = 0; i < NUM_REF_PIC_LIST_01; i++ )
    {
      for( j = 0; j < MAX_NUM_REF; j++ )
      {
        refPicList[i][j] = slice->refPicList[i][j];
        refPOCList[i][j] = slice->refPOCList[i][j];
        isUsedAsLongTerm[i][j] = slice->isUsedAsLongTerm[i][j];
      }
      isUsedAsLongTerm[i][MAX_NUM_REF] = slice->isUsedAsLongTerm[i][MAX_NUM_REF];
    }
  }

  // access channel
  if (cpyAlmostAll) rpl[0] = slice->rpl[0];
  if (cpyAlmostAll) rpl[1] = slice->rpl[1];
  lastIDR             = slice->lastIDR;

  if( cpyAlmostAll ) pic  = slice->pic;

  colFromL0Flag        = slice->colFromL0Flag;
  colRefIdx            = slice->colRefIdx;

  if( cpyAlmostAll ) setLambdas(slice->getLambdas());

  TLayer                        = slice->TLayer;
  TLayerSwitchingFlag           = slice->TLayerSwitchingFlag;
  independentSliceIdx           = slice->independentSliceIdx;
  clpRngs                       = slice->clpRngs;
  lmcsEnabled                   = slice->lmcsEnabled;
  explicitScalingListUsed       = slice->explicitScalingListUsed;
  pendingRasInit                = slice->pendingRasInit;

  for( uint32_t ch = 0 ; ch < MAX_NUM_CH; ch++)
  {
    saoEnabled[ch] = slice->saoEnabled[ch];
  }

  cabacInitFlag                 = slice->cabacInitFlag;
  memcpy( alfAps, slice->alfAps, sizeof(alfAps)); // this might be quite unsafe
  memcpy( alfEnabled, slice->alfEnabled, sizeof(alfEnabled));
  numAps               = slice->numAps;
  lumaApsId            = slice->lumaApsId;
  chromaApsId          = slice->chromaApsId;
  isLossless                    = slice->isLossless;

  sliceMap                      = slice->sliceMap;

  ccAlfFilterParam              = slice->ccAlfFilterParam;
  ccAlfFilterControl[0]         = slice->ccAlfFilterControl[0];
  ccAlfFilterControl[1]         = slice->ccAlfFilterControl[1];
  ccAlfCbEnabled       = slice->ccAlfCbEnabled;
  ccAlfCrEnabled       = slice->ccAlfCrEnabled;
  ccAlfCbApsId         = slice->ccAlfCbApsId;
  ccAlfCrApsId         = slice->ccAlfCrApsId;

  if( cpyAlmostAll ) encCABACTableIdx  = slice->encCABACTableIdx;
}

/** Function for checking if this is a STSA candidate
 */
bool Slice::isStepwiseTemporalLayerSwitchingPointCandidate(const PicList& rcListPic) const
{
  PicList::const_iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    const Picture* pic = *(iterPic++);
    if( pic->isInitDone && pic->isReferenced && pic->poc != poc)
    {
      if( pic->TLayer >= TLayer)
      {
        return false;
      }
    }
  }
  return true;
}


void Slice::checkLeadingPictureRestrictions(const PicList& rcListPic) const
{
  // When a picture is a leading picture, it shall be a RADL or RASL picture.
  if(associatedIRAP > poc && !pps->mixedNaluTypesInPic)
  {
    // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
    if (nalUnitType < VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
        nalUnitType > VVENC_NAL_UNIT_CODED_SLICE_CRA)
    {
      CHECK(nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_RASL &&
            nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_RADL, "Invalid NAL unit type");
    }
  }

  // When a picture is a trailing picture, it shall not be a RADL or RASL picture.
  if(associatedIRAP < poc && !pps->mixedNaluTypesInPic)
  {
    CHECK(nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL ||
          nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RADL, "Invalid NAL unit type");
  }


  // No RASL pictures shall be present in the bitstream that are associated with
  // an IDR picture.
  if (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL)
  {
    CHECK( associatedIRAPType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
           associatedIRAPType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL, "Invalid NAL unit type");
  }

  // No RADL pictures shall be present in the bitstream that are associated with
  // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
  // with an IDR picture having nal_unit_type equal to IDR_N_LP.
  if (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RADL)
  {
    CHECK (associatedIRAPType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP, "Invalid NAL unit type");
  }

  // loop through all pictures in the reference picture buffer
  PicList::const_iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    Picture* pic = *(iterPic++);
    if( ! pic->isReconstructed )
    {
      continue;
    }
    if( pic->poc == poc)
    {
      continue;
    }
    const Slice* slice = pic->slices[0];

    if (slice->picHeader->picOutputFlag == 1 && !picHeader->noOutputOfPriorPics && pic->layerId == nuhLayerId)
    {
      if (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA || nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP || nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL)
      {
        CHECK(pic->poc >= poc, "Any picture, with nuh_layer_id equal to a particular value layerId, that precedes an IRAP picture with nuh_layer_id "
              "equal to layerId in decoding order shall precede the IRAP picture in output order.");
      }
    }

    if (slice->picHeader->picOutputFlag == 1 && !picHeader->noOutputBeforeRecovery && pic->layerId == nuhLayerId)
    {
      if (poc == picHeader->recoveryPocCnt + prevGDRInSameLayerPOC)
      {
        CHECK(pic->poc >= poc, "Any picture, with nuh_layer_id equal to a particular value layerId, that precedes a recovery point picture with "
              "nuh_layer_id equal to layerId in decoding order shall precede the recovery point picture in output order.");
      }
    }

    if (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL)
    {
      if ((associatedIRAPType == VVENC_NAL_UNIT_CODED_SLICE_CRA) &&
          associatedIRAP == slice->associatedIRAP)
      {
        if (slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RADL)
        {
          CHECK(pic->poc <= poc, "Any RASL picture associated with a CRA picture shall precede any RADL picture associated with the CRA picture in output order.");
        }
      }
    }

    if (nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL)
    {
      if(associatedIRAPType == VVENC_NAL_UNIT_CODED_SLICE_CRA)
      {
        if(slice->poc < associatedIRAP &&
          (
            slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
            slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA ||
            slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_GDR) &&
            pic->layerId == nuhLayerId)
        {
          CHECK(poc <= slice->poc, "Any RASL picture, with nuh_layer_id equal to a particular value layerId, associated with a CRA picture shall follow, "
               "in output order, any IRAP or GDR picture with nuh_layer_id equal to layerId that precedes the CRA picture in decoding order.");
        }
      }
    }
  }
}


//Function for applying picture marking based on the Reference Picture List
void Slice::applyReferencePictureListBasedMarking(const PicList& rcListPic, const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int layerId, const PPS& pps, const bool usingLongTerm ) const
{
  int i, isReference;
  checkLeadingPictureRestrictions(rcListPic);

  bool isNeedToCheck = ( nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP || nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL ) ? false : true;

  // loop through all pictures in the reference picture buffer
  PicList::const_iterator iterPic = rcListPic.begin();
  while( iterPic != rcListPic.end() )
  {
    Picture* pic = *( iterPic++ );

    if( !pic->isReferenced )
      continue;

    isReference = 0;
    // loop through all pictures in the Reference Picture Set
    // to see if the picture should be kept as reference picture
    for( i = 0; isNeedToCheck && !isReference && i < pRPL0->numberOfShorttermPictures + pRPL0->numberOfLongtermPictures + pRPL0->numberOfInterLayerPictures; i++ )
    {
      if( pRPL0->isInterLayerRefPic[ i ] )
      {
        // Diagonal inter-layer prediction is not allowed
        CHECK( pRPL0->refPicIdentifier[i], "ILRP identifier should be 0" );

        if( pic->poc == poc )
        {
          isReference = 1;
          if( usingLongTerm && !pic->isLongTerm ) pic->isLongTerm = true;
        }
      }
      else if( pic->layerId == layerId )
      {
        if( !pRPL0->isLongtermRefPic[i] )
        {
          if( pic->poc == poc + pRPL0->refPicIdentifier[i] )
          {
            isReference = 1;
            if( usingLongTerm && pic->isLongTerm ) pic->isLongTerm = false;
          }
        }
        else
        {
          int pocCycle = 1 << (pic->cs->sps->bitsForPOC);
          int curPoc = pic->poc & (pocCycle - 1);
          if( usingLongTerm && pic->isLongTerm && curPoc == pRPL0->refPicIdentifier[i] )
          {
            isReference = 1;
          }
        }
      }
    }
    for( i = 0; isNeedToCheck && !isReference && i < pRPL1->numberOfShorttermPictures + pRPL1->numberOfLongtermPictures + pRPL1->numberOfInterLayerPictures; i++ )
    {
      if( pRPL1->isInterLayerRefPic[i] )
      {
        // Diagonal inter-layer prediction is not allowed
        CHECK( pRPL1->refPicIdentifier[i], "ILRP identifier should be 0" );

        if( pic->poc == poc )
        {
          isReference = 1;
          if( usingLongTerm && !pic->isLongTerm ) pic->isLongTerm = true;
        }
      }
      else if( pic->layerId == layerId )
      {
        if( !pRPL1->isLongtermRefPic[i] )
        {
          if( pic->poc == poc + pRPL1->refPicIdentifier[i] )
          {
            isReference = 1;
            if( usingLongTerm && pic->isLongTerm ) pic->isLongTerm = false;
          }
        }
        else
        {
          int pocCycle = 1 << ( pic->cs->sps->bitsForPOC );
          int curPoc = pic->poc & ( pocCycle - 1 );
          if( usingLongTerm && pic->isLongTerm && curPoc == pRPL1->refPicIdentifier[i] )
          {
            isReference = 1;
          }
        }
      }
    }
    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture List
    if ( pic->layerId == layerId && pic->isInitDone && pic->poc != poc && isReference == 0 )
    {
      pic->isReferenced = false;
      if( usingLongTerm )
        pic->isLongTerm   = false;
    }
  }
}

// int Slice::checkThatAllRefPicsAreAvailable( const PicList& rcListPic, const ReferencePictureList *pRPL, int rplIdx ) const
bool Slice::isRplPicMissing( const PicList& rcListPic, const RefPicList refList, int& missingPoc, int ip ) const
{
  if( isIDRorBLA() ) return false; // assume that all pic in the DPB will be flushed anyway so no need to check.

  const ReferencePictureList* pRPL = rpl[ refList ];
  int numberOfPictures             = pRPL->numberOfLongtermPictures + pRPL->numberOfShorttermPictures + pRPL->numberOfInterLayerPictures;

  // check long term ref pics
  if( pRPL->numberOfLongtermPictures > 0 )
  {
    for( int ii = 0; ii < numberOfPictures; ii++ )
    {
      if( ! pRPL->isLongtermRefPic[ii] || pRPL->isInterLayerRefPic[ii] )
        continue;

      bool isAvailable = false;
      int  checkPoc    = pRPL->refPicIdentifier[ii];

      for( auto& pic : rcListPic )
      {
        int pocCycle = 1 << (pic->cs->sps->bitsForPOC);
        int curPoc   = pic->getPOC() & (pocCycle - 1);
        int refPoc   = pRPL->refPicIdentifier[ii] & (pocCycle - 1);
        if( pRPL->deltaPocMSBPresent[ii] )
        {
          refPoc += poc - pRPL->deltaPocMSBCycleLT[ii] * pocCycle - (poc & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if( pic->isLongTerm && curPoc == refPoc && pic->isReferenced )
        {
          isAvailable = true;
          break;
        }
      }

      // if there was no such long-term check the short terms
      if( ! isAvailable )
      {
        for( auto& pic : rcListPic )
        {
          int pocCycle = 1 << (pic->cs->sps->bitsForPOC);
          int curPoc   = pic->getPOC() & (pocCycle - 1);
          int refPoc   = pRPL->refPicIdentifier[ii] & (pocCycle - 1);
          if( pRPL->deltaPocMSBPresent[ii] )
          {
            refPoc += poc - pRPL->deltaPocMSBCycleLT[ii] * pocCycle - (poc & (pocCycle - 1));
          }
          else
          {
            curPoc = curPoc & (pocCycle - 1);
          }
          if( ! pic->isLongTerm && curPoc == refPoc && pic->isReferenced )
          {
            isAvailable     = true;
            pic->isLongTerm = true;
            break;
          }
        }
      }

      if( ! isAvailable )
      {
        missingPoc = checkPoc;
        return true;
      }
    }
  }

  // check short term ref pics
  for( int ii = 0; ii < numberOfPictures; ii++ )
  {
    if( pRPL->isLongtermRefPic[ii] )
      continue;

    bool isAvailable = false;
    int  checkPoc    = poc + pRPL->refPicIdentifier[ii];

    for( auto& pic : rcListPic )
    {
      if( ! pic->isLongTerm && pic->getPOC() == poc + pRPL->refPicIdentifier[ii] && pic->isReferenced && !refPicIsFutureIDRnoLP( pic->getPOC(), ip ) )
      {
        isAvailable = true;
        break;
      }
    }
    if( ! isAvailable && pRPL->numberOfShorttermPictures > 0 )
    {
      missingPoc = checkPoc;
      return true;
    }
  }

  return false;
}

void Slice::createExplicitReferencePictureSetFromReference(const PicList& rcListPic, const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, int ip)
{
  Picture* picCand;;
  int pocCycle = 0;

  if ( isIDRorBLA() ) return; //Assume that all pic in the DPB will be flushed anyway so no need to check.

  ReferencePictureList rplSrc0 = *pRPL0;
  ReferencePictureList rplSrc1 = *pRPL1;

  ReferencePictureList* pLocalRPL0 = &rplLocal[0];
  (*pLocalRPL0) = ReferencePictureList();

  uint32_t numOfSTRPL0 = 0;
  uint32_t numOfLTRPL0 = 0;
  uint32_t numOfILRPL0 = 0;
  uint32_t numOfRefPic = rplSrc0.numberOfShorttermPictures + rplSrc0.numberOfLongtermPictures;
  uint32_t refPicIdxL0 = 0;
  for (int ii = 0; ii < numOfRefPic; ii++)
  {
    // loop through all pictures in the reference picture buffer
    PicList::const_iterator iterPic = rcListPic.begin();
    bool isAvailable = false;

    pocCycle = 1 << (sps->bitsForPOC);
    while (iterPic != rcListPic.end())
    {
      picCand = *(iterPic++);
      if( pic->layerId == picCand->layerId && picCand->isReferenced)
      {
        if (!rplSrc0.isLongtermRefPic[ii] && picCand->poc == poc + rplSrc0.refPicIdentifier[ii] && !isPocRestrictedByDRAP(picCand->poc, picCand->precedingDRAP) && !refPicIsFutureIDRnoLP(picCand->poc, ip))
        {
          isAvailable = true;
          break;
        }
        else if (rplSrc0.isLongtermRefPic[ii] && (picCand->poc & (pocCycle - 1)) == rplSrc0.refPicIdentifier[ii] && !isPocRestrictedByDRAP(picCand->poc, picCand->precedingDRAP))
        {
          isAvailable = true;
          break;
        }
      }
    }
    if (isAvailable)
    {
      pLocalRPL0->setRefPicIdentifier(refPicIdxL0, rplSrc0.refPicIdentifier[ii], rplSrc0.isLongtermRefPic[ii], rplSrc0.isInterLayerRefPic[ii], rplSrc0.interLayerRefPicIdx[ii]);
      refPicIdxL0++;
      numOfSTRPL0 = numOfSTRPL0 + ((rplSrc0.isLongtermRefPic[ii]) ? 0 : 1);
      numOfLTRPL0 = numOfLTRPL0 + ((rplSrc0.isLongtermRefPic[ii]) ? 1 : 0);
      isAvailable = false;
    }
  }

  if( enableDRAPSEI)
  {
    pLocalRPL0->numberOfShorttermPictures = (numOfSTRPL0);
    pLocalRPL0->numberOfLongtermPictures = (numOfLTRPL0);
    if( !isIRAP() && !pLocalRPL0->isPOCInRefPicList(associatedIRAP, poc))
    {
      if (useLTforDRAP && !rplSrc1.isPOCInRefPicList(associatedIRAP, poc))
      {
        // Adding associated IRAP as longterm picture
        pLocalRPL0->setRefPicIdentifier(refPicIdxL0, associatedIRAP, true, false, 0);
        refPicIdxL0++;
        numOfLTRPL0++;
      }
      else
      {
        // Adding associated IRAP as shortterm picture
        pLocalRPL0->setRefPicIdentifier(refPicIdxL0, associatedIRAP - poc, false, false, 0);
        refPicIdxL0++;
        numOfSTRPL0++;
      }
    }
  }

  ReferencePictureList* pLocalRPL1 = &rplLocal[1];
  (*pLocalRPL1) = ReferencePictureList();

  uint32_t numOfSTRPL1 = 0;
  uint32_t numOfLTRPL1 = 0;
  uint32_t numOfILRPL1 = 0;
  numOfRefPic = rplSrc1.numberOfShorttermPictures + rplSrc1.numberOfLongtermPictures;
  uint32_t refPicIdxL1 = 0;
  for (int ii = 0; ii < numOfRefPic; ii++)
  {
    // loop through all pictures in the reference picture buffer
    PicList::const_iterator iterPic = rcListPic.begin();
    bool isAvailable = false;
    pocCycle = 1 << sps->bitsForPOC;
    while (iterPic != rcListPic.end())
    {
      picCand = *(iterPic++);
      if( pic->layerId == picCand->layerId && picCand->isReferenced )
      {
        if (!rplSrc1.isLongtermRefPic[ii] && picCand->poc == poc + rplSrc1.refPicIdentifier[ii] && !isPocRestrictedByDRAP(picCand->poc, picCand->precedingDRAP) && !refPicIsFutureIDRnoLP(picCand->poc, ip))
        {
          isAvailable = true;
          break;
        }
        else if (rplSrc1.isLongtermRefPic[ii] && (picCand->poc & (pocCycle - 1)) == rplSrc1.refPicIdentifier[ii] && !isPocRestrictedByDRAP(picCand->poc, picCand->precedingDRAP))
        {
          isAvailable = true;
          break;
        }
      }
    }
    if (isAvailable)
    {
      pLocalRPL1->setRefPicIdentifier(refPicIdxL1, rplSrc1.refPicIdentifier[ii], rplSrc1.isLongtermRefPic[ii], rplSrc1.isInterLayerRefPic[ii], rplSrc1.interLayerRefPicIdx[ii]);
      refPicIdxL1++;
      numOfSTRPL1 = numOfSTRPL1 + ((rplSrc1.isLongtermRefPic[ii]) ? 0 : 1);
      numOfLTRPL1 = numOfLTRPL1 + ((rplSrc1.isLongtermRefPic[ii]) ? 1 : 0);
      isAvailable = false;
    }
  }

  //Copy from L1 if we have less than active ref pic
  int numOfNeedToFill = rplSrc0.numberOfActivePictures - (numOfLTRPL0 + numOfSTRPL0);
  bool isDisallowMixedRefPic = sps->allRplEntriesHasSameSign;
  int originalL0StrpNum = numOfSTRPL0;
  int originalL0LtrpNum = numOfLTRPL0;
  int originalL0IlrpNum = numOfILRPL0;

  for (int ii = 0; numOfNeedToFill > 0 && ii < (numOfLTRPL1 + numOfSTRPL1 + numOfILRPL1); ii++)
  {
    if (ii <= (numOfLTRPL1 + numOfSTRPL1 + numOfILRPL1 - 1))
    {
      //Make sure this copy is not already in L0
      bool canIncludeThis = true;
      for (int jj = 0; jj < refPicIdxL0; jj++)
      {
        if ((pLocalRPL1->refPicIdentifier[ii] == pLocalRPL0->refPicIdentifier[jj])
         && (pLocalRPL1->isLongtermRefPic[ii] == pLocalRPL0->isLongtermRefPic[jj])
         && (pLocalRPL1->isInterLayerRefPic[ii] == pLocalRPL0->isInterLayerRefPic[jj]) )
        {
          canIncludeThis = false;
        }
        bool sameSign = (pLocalRPL1->refPicIdentifier[ii] > 0) == (pLocalRPL0->refPicIdentifier[0] > 0);
        if (isDisallowMixedRefPic && canIncludeThis && !pLocalRPL1->isLongtermRefPic[ii] && !sameSign)
        {
          canIncludeThis = false;
        }
      }
      if (canIncludeThis)
      {
        pLocalRPL0->setRefPicIdentifier(refPicIdxL0, pLocalRPL1->refPicIdentifier[ii], pLocalRPL1->isLongtermRefPic[ii], pLocalRPL1->isInterLayerRefPic[ii], pLocalRPL1->interLayerRefPicIdx[ii]);
        refPicIdxL0++;
        numOfSTRPL0 +=  rplSrc1.isLongtermRefPic[ii] ? 0 : 1;
        numOfLTRPL0 += (rplSrc1.isLongtermRefPic[ii] && !rplSrc1.isInterLayerRefPic[ii]) ? 1 : 0;
        numOfILRPL0 +=  rplSrc1.isInterLayerRefPic[ ii] ? 1 : 0;

        numOfNeedToFill--;
      }
    }
  }
  pLocalRPL0->numberOfLongtermPictures  = numOfLTRPL0;
  pLocalRPL0->numberOfShorttermPictures = numOfSTRPL0;
  pLocalRPL0->numberOfInterLayerPictures = numOfILRPL0;

  int numPics = numOfLTRPL0 + numOfSTRPL0;
  pLocalRPL0->numberOfActivePictures  = ( numPics < rpl[0]->numberOfActivePictures ? numPics : rpl[0]->numberOfActivePictures ) + numOfILRPL0;
  pLocalRPL0->ltrpInSliceHeader = rpl[0]->ltrpInSliceHeader;

  pLocalRPL0->numberOfActivePictures    = (numOfLTRPL0 + numOfSTRPL0 < rplSrc0.numberOfActivePictures) ? numOfLTRPL0 + numOfSTRPL0 : rplSrc0.numberOfActivePictures;
  pLocalRPL0->ltrpInSliceHeader = rplSrc0.ltrpInSliceHeader;
  rplIdx[0] = -1;
  rpl[0]    = pLocalRPL0;

  //Copy from L0 if we have less than active ref pic
  numOfNeedToFill = pLocalRPL0->numberOfActivePictures - (numOfLTRPL1 + numOfSTRPL1);
  for (int ii = 0; numOfNeedToFill > 0 && ii < (pLocalRPL0->numberOfLongtermPictures + pLocalRPL0->numberOfShorttermPictures + pLocalRPL0->numberOfInterLayerPictures ); ii++)
  {
    if (ii <= (originalL0StrpNum + originalL0LtrpNum + originalL0IlrpNum - 1))
    {
      //Make sure this copy is not already in L0
      bool canIncludeThis = true;
      for (int jj = 0; jj < refPicIdxL1; jj++)
      {
        if ((pLocalRPL0->refPicIdentifier[ii] == pLocalRPL1->refPicIdentifier[jj])
          && (pLocalRPL0->isLongtermRefPic[ii] == pLocalRPL1->isLongtermRefPic[jj])
          && (pLocalRPL0->isInterLayerRefPic[ii] == pLocalRPL1->isInterLayerRefPic[jj]))
        {
          canIncludeThis = false;
        }
        bool sameSign = (pLocalRPL0->refPicIdentifier[ii] > 0) == (pLocalRPL1->refPicIdentifier[0] > 0);
        if (isDisallowMixedRefPic && canIncludeThis && !pLocalRPL0->isLongtermRefPic[ii] && !sameSign)
        {
          canIncludeThis = false;
        }
      }
      if (canIncludeThis)
      {
        pLocalRPL1->setRefPicIdentifier(refPicIdxL1, pLocalRPL0->refPicIdentifier[ii], pLocalRPL0->isLongtermRefPic[ii], pLocalRPL0->isInterLayerRefPic[ii], pLocalRPL0->interLayerRefPicIdx[ii]);
        refPicIdxL1++;
        numOfSTRPL1 += pLocalRPL0->isLongtermRefPic[ii] ? 0 : 1;
        numOfLTRPL1 += (pLocalRPL0->isLongtermRefPic[ii] && !pLocalRPL0->isInterLayerRefPic[ii]) ? 1 : 0;
        numOfLTRPL1 += pLocalRPL0->isInterLayerRefPic[ii] ? 1 : 0;

        numOfNeedToFill--;
      }
    }
  }
  pLocalRPL1->numberOfLongtermPictures  = numOfLTRPL1;
  pLocalRPL1->numberOfShorttermPictures = numOfSTRPL1;
  pLocalRPL1->numberOfInterLayerPictures = numOfILRPL1;
  numPics = numOfLTRPL1 + numOfSTRPL1;

  pLocalRPL1->numberOfActivePictures    = (isDisallowMixedRefPic) ? numPics : ((numPics < rplSrc1.numberOfActivePictures) ? numPics : rplSrc1.numberOfActivePictures);
  pLocalRPL1->ltrpInSliceHeader         = rplSrc1.ltrpInSliceHeader;
  rplIdx[1] = -1;
  rpl[1]    = pLocalRPL1;
}

//! get tables for weighted prediction
void  Slice::getWpScaling( RefPicList e, int iRefIdx, WPScalingParam *&wp ) const
{
 CHECK(e>=NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  wp = (WPScalingParam*) weightPredTable[e][iRefIdx];
}

void PicHeader::getWpScaling(RefPicList e, int iRefIdx, WPScalingParam *&wp) const
{
  CHECK(e >= NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  wp = (WPScalingParam *) weightPredTable[e][iRefIdx];
}

//! reset Default WP tables settings : no weight.
void  Slice::resetWpScaling()
{
  for ( int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( int yuv=0 ; yuv<MAX_NUM_COMP ; yuv++ )
      {
        WPScalingParam  *pwp = &(weightPredTable[e][i][yuv]);
        pwp->presentFlag     = false;
        pwp->log2WeightDenom = 0;
        pwp->iWeight         = 1;
        pwp->iOffset         = 0;
      }
    }
  }
}

unsigned Slice::getMinPictureDistance() const
{
  int minPicDist = MAX_INT;
  if (sps->IBC)
  {
    minPicDist = 0;
  }
  else
  if( ! isIntra() )
  {
    const int currPOC  = poc;
    for (int refIdx = 0; refIdx < numRefIdx[ REF_PIC_LIST_0 ]; refIdx++)
    {
      minPicDist = std::min( minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_0, refIdx)->getPOC()));
    }
    if( sliceType == VVENC_B_SLICE )
    {
      for (int refIdx = 0; refIdx < numRefIdx[ REF_PIC_LIST_1 ]; refIdx++)
      {
        minPicDist = std::min(minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_1, refIdx)->getPOC()));
      }
    }
  }
  return (unsigned) minPicDist;
}

bool Slice::isPocRestrictedByDRAP( int poc, bool precedingDRAPInDecodingOrder ) const
{
  if (!enableDRAPSEI)
  {
    return false;
  }
  return ( isDRAP && poc != associatedIRAP ) || ( latestDRAPPOC != MAX_INT && poc > latestDRAPPOC && (precedingDRAPInDecodingOrder || poc < latestDRAPPOC) );
}

bool Slice::refPicIsFutureIDRnoLP( int candPoc, int ipc ) const
{
  //check if we are not trying to reference a future IDR picture
  if( ipc != 0 && associatedIRAP + ipc == candPoc )
  {
    return true;
  }
  return false;
}

void Slice::setAlfApsIds( const std::vector<int>& ApsIDs)
{
  lumaApsId.resize(numAps);
  for (int i = 0; i < numAps; i++)
  {
    lumaApsId[i] = ApsIDs[i];
  }
}


void PicHeader::copyPicInfo( const PicHeader* other, bool cpyAll)
{
  pocLsb                                  = other->pocLsb;                                
  nonRefPic                               = other->nonRefPic;                             
  gdrOrIrapPic                            = other->gdrOrIrapPic;                          
  gdrPic                                  = other->gdrPic;                                
  noOutputOfPriorPics                     = other->noOutputOfPriorPics;                   
  recoveryPocCnt                          = other->recoveryPocCnt;                        
  noOutputBeforeRecovery                  = other->noOutputBeforeRecovery;                
  handleCraAsCvsStart                     = other->handleCraAsCvsStart;                   
  handleGdrAsCvsStart                     = other->handleGdrAsCvsStart;                   
  spsId                                   = other->spsId;                                 
  ppsId                                   = other->ppsId;                                 
  pocMsbPresent                           = other->pocMsbPresent;                         
  pocMsbVal                               = other->pocMsbVal;                             
  virtualBoundariesEnabled                = other->virtualBoundariesEnabled;              
  virtualBoundariesPresent                = other->virtualBoundariesPresent;              
  numVerVirtualBoundaries                 = other->numVerVirtualBoundaries;               
  numHorVirtualBoundaries                 = other->numHorVirtualBoundaries;               
//  virtualBoundariesPosX[3];               = other->virtualBoundariesPosX[3];              
//  virtualBoundariesPosY[3];               = other->virtualBoundariesPosY[3];              
  picOutputFlag                           = other->picOutputFlag;                         
//pic                                     = other->pic;                                   
//  pRPL[NUM_REF_PIC_LIST_01]              = other->pRPL[NUM_REF_PIC_LIST_01];             
  localRPL[L0]                            = other->localRPL[L0];         
  localRPL[L1]                            = other->localRPL[L1];         
  rplIdx[L0]                              = other->rplIdx[L0];           
  rplIdx[L1]                              = other->rplIdx[L1];           
  picInterSliceAllowed                    = other->picInterSliceAllowed;                  
  picIntraSliceAllowed                    = other->picIntraSliceAllowed;                  
  splitConsOverride                       = other->splitConsOverride;                     
  cuQpDeltaSubdivIntra                    = other->cuQpDeltaSubdivIntra;                  
  cuQpDeltaSubdivInter                    = other->cuQpDeltaSubdivInter;                  
  cuChromaQpOffsetSubdivIntra             = other->cuChromaQpOffsetSubdivIntra;           
  cuChromaQpOffsetSubdivInter             = other->cuChromaQpOffsetSubdivInter;           
  enableTMVP                              = other->enableTMVP;                            
  picColFromL0                            = other->picColFromL0;                          
  colRefIdx                               = other->colRefIdx;
  mvdL1Zero                               = other->mvdL1Zero;                             
  maxNumAffineMergeCand                   = other->maxNumAffineMergeCand;                 
  disFracMMVD                             = other->disFracMMVD;                           
  disBdofFlag                             = other->disBdofFlag;                           
  disDmvrFlag                             = other->disDmvrFlag;                           
  disProfFlag                             = other->disProfFlag;                           
  jointCbCrSign                           = other->jointCbCrSign;                         
  qpDelta                                 = other->qpDelta;                               
  memcpy(saoEnabled, other->saoEnabled, sizeof(saoEnabled));                
  memcpy(alfEnabled, other->alfEnabled, sizeof(alfEnabled));              
  numAlfAps                               = other->numAlfAps;                             
  alfApsId                                = other->alfApsId;                              
  alfChromaApsId                          = other->alfChromaApsId;                        
  memcpy(ccalfEnabled, other->ccalfEnabled, sizeof(ccalfEnabled));                
  ccalfCbApsId                            = other->ccalfCbApsId;
  ccalfCrApsId                            = other->ccalfCrApsId;
  deblockingFilterOverride                = other->deblockingFilterOverride;                
  deblockingFilterDisable                 = other->deblockingFilterDisable;                
  memcpy(deblockingFilterBetaOffsetDiv2,  other->deblockingFilterBetaOffsetDiv2,  sizeof(deblockingFilterBetaOffsetDiv2));                
  memcpy(deblockingFilterTcOffsetDiv2,    other->deblockingFilterTcOffsetDiv2,    sizeof(deblockingFilterTcOffsetDiv2));                
  lmcsEnabled                             = other->lmcsEnabled;                           
  lmcsApsId                               = other->lmcsApsId;                             
//lmcsAps;                                = other->lmcsAps;                               
  lmcsChromaResidualScale                 = other->lmcsChromaResidualScale;               
  explicitScalingListEnabled              = other->explicitScalingListEnabled;            
  scalingListApsId                        = other->scalingListApsId;                      
//scalingListAps;                         = other->scalingListAps;                        
  memcpy(minQTSize,   other->minQTSize, sizeof(minQTSize));                
  memcpy(maxMTTDepth, other->maxMTTDepth, sizeof(maxMTTDepth));                
  memcpy(maxBTSize,   other->maxBTSize, sizeof(maxBTSize));                
  memcpy(maxTTSize,   other->maxTTSize, sizeof(maxTTSize));                

//  memcpy(weightPredTable,   other->weightPredTable, sizeof(weightPredTable));                
  numL0Weights                            = other->numL0Weights;                          
  numL1Weights                            = other->numL1Weights;                          

}

// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)
// ------------------------------------------------------------------------------------------------

SPS::SPS()
: spsId                           (  0 )
, dciId                           (  0 )
, vpsId                           (  0 )
, layerId                         ( 0 )
, AffineAmvr                      ( false )
, DMVR                            ( false )
, MMVD                            ( false )
, SBT                             ( false )
, ISP                             ( false )
, chromaFormatIdc                 ( CHROMA_420 )
, separateColourPlane             ( false )
, maxTLayers                      ( 1 )
, ptlDpbHrdParamsPresent          ( true )
, subLayerDpbParams               ( false )
, maxPicWidthInLumaSamples        ( 352 )
, maxPicHeightInLumaSamples       ( 288 )
, subPicInfoPresent               ( false )
, numSubPics                      ( 0 )
, independentSubPicsFlag          ( false )
, subPicIdMappingExplicitlySignalled ( false )
, log2MinCodingBlockSize          ( 0 )
, CTUSize                         ( 0 )
, partitionOverrideEnabled        ( 1 )
, minQTSize                       { 0, 0, 0 }
, maxMTTDepth                     { MAX_BT_DEPTH, MAX_BT_DEPTH_INTER, 0 }
, maxBTSize                       { 0,  0,  0 }
, maxTTSize                       { 0,  0,  0 }
, idrRefParamList                 ( false )
, dualITree                       ( 0 )
, rpl1CopyFromRpl0                ( false )
, rpl1IdxPresent                  ( false )
, allRplEntriesHasSameSign        ( true )
, longTermRefsPresent             ( false )
, temporalMVPEnabled              ( 0 )
, transformSkip                   ( false )
, log2MaxTransformSkipBlockSize   ( 0 )
, BDPCM                           ( false )
, jointCbCr                       ( false )
, entropyCodingSyncEnabled        ( false )
, entryPointsPresent              ( false )
, qpBDOffset                      { 0,0 }
, internalMinusInputBitDepth      { 0,0 }
, SbtMvp                          ( false)
, BDOF                            ( false)
, fpelMmvd                        ( false )
, BdofPresent                     ( false )
, DmvrPresent                     ( false )
, ProfPresent                     ( false )
, bitsForPOC                      ( 8 )
, pocMsbFlag                      ( false )
, pocMsbLen                       ( 0 )
, numExtraPHBitsBytes             ( 0 )
, numExtraSHBitsBytes             ( 0 )
, numLongTermRefPicSPS            ( 0 )
, log2MaxTbSize                   ( 6 )
, weightPred                      ( false )
, weightedBiPred                  ( false )
, saoEnabled                      ( false )
, temporalIdNesting               ( false )
, scalingListEnabled              ( false )
, depQuantEnabled                 ( false )
, signDataHidingEnabled           ( false )
, virtualBoundariesEnabled        ( false )
, virtualBoundariesPresent        ( false )
, numVerVirtualBoundaries         ( 0 )
, numHorVirtualBoundaries         ( 0 )
, virtualBoundariesPosX           { 0,  0,  0 }
, virtualBoundariesPosY           { 0,  0,  0 }
, hrdParametersPresent            ( false )
, subLayerParametersPresent       ( false )
, fieldSeqFlag                    ( false )
, vuiParametersPresent            ( false )
, vuiPayloadSize                  ( 0 )
, vuiParameters                   ()
, alfEnabled                      ( false )
, ccalfEnabled                    ( false )
, wrapAroundEnabled               ( false )
, IBC                             ( false )
, useColorTrans                   ( false )
, PLT                             ( false )
, lumaReshapeEnable               ( false )
, AMVR                            ( false )
, LMChroma                        ( false )
, horCollocatedChroma             ( false )
, verCollocatedChroma             ( false )
, MTS                             ( false )
, MTSIntra                        ( false )
, MTSInter                        ( false )
, LFNST                           ( false )
, SMVD                            ( false )
, Affine                          ( false )
, AffineType                      ( false )
, PROF                            ( false )
, BCW                             ( false )
, CIIP                            ( false )
, GEO                             ( false )
, LADF                            ( false )
, MRL                             ( false )
, MIP                             ( false )
, GDR                             ( true )
, subLayerCbpParametersPresent    ( true)
, rprEnabled                      ( false )
, resChangeInClvsEnabled          ( false )
, interLayerPresent               ( false )
, log2ParallelMergeLevelMinus2    ( 0 )
, maxNumMergeCand                 ( 0 )
, maxNumAffineMergeCand           ( 0 )
, maxNumIBCMergeCand              ( 0 )
, maxNumGeoCand                   ( 0 )
, scalingMatrixAlternativeColourSpaceDisabled ( false )
, scalingMatrixDesignatedColourSpace          ( false )
, disableScalingMatrixForLfnstBlks            ( false )

{
  for(int ch=0; ch<MAX_NUM_CH; ch++)
  {
    bitDepths.recon[ch] = 8;
    qpBDOffset   [ch] = 0;
  }

  for ( int i = 0; i < VVENC_MAX_TLAYER; i++ )
  {
    maxLatencyIncreasePlus1[i] = 0;
    maxDecPicBuffering[i] = 1;
    numReorderPics[i]       = 0;
  }

  ::memset(ltRefPicPocLsbSps, 0, sizeof(ltRefPicPocLsbSps));
  ::memset(usedByCurrPicLtSPS, 0, sizeof(usedByCurrPicLtSPS));

  for( int i = 0; i < MAX_NUM_SUB_PICS; i++ )
  {
    subPicCtuTopLeftX[i] = 0;
    subPicCtuTopLeftY[i] = 0;
    subPicWidth[i] = 0;
    subPicHeight[i] = 0;
    subPicTreatedAsPic[i] = false;
    loopFilterAcrossSubpicEnabled[i] = false;
    subPicId[i] = 0;
  }
}

void ChromaQpMappingTable::setParams(const vvencChromaQpMappingTableParams &params, const int qpBdOffset)
{
  m_qpBdOffset = qpBdOffset;
  m_sameCQPTableForAllChromaFlag = params.m_sameCQPTableForAllChromaFlag;

  for (int i = 0; i < VVENC_MAX_NUM_CQP_MAPPING_TABLES; i++)
  {
    m_qpTableStartMinus26[i] = params.m_qpTableStartMinus26[i];
    m_numPtsInCQPTableMinus1[i] = params.m_numPtsInCQPTableMinus1[i];
    memcpy(m_deltaQpInValMinus1[i], params.m_deltaQpInValMinus1[i], sizeof m_deltaQpInValMinus1[i]);
    memcpy(m_deltaQpOutVal[i], params.m_deltaQpOutVal[i], sizeof m_deltaQpOutVal[i]);
    m_chromaQpMappingTables[i].resize( MAX_QP + qpBdOffset + 1 );
  }
}

void ChromaQpMappingTable::derivedChromaQPMappingTables()
{
  for (int i = 0; i < m_numQpTables; i++)
  {
    const int qpBdOffsetC = m_qpBdOffset;
    const int numPtsInCQPTableMinus1 = m_numPtsInCQPTableMinus1[i];
    std::vector<int> qpInVal(numPtsInCQPTableMinus1 + 2), qpOutVal(numPtsInCQPTableMinus1 + 2);

    qpInVal[0] = m_qpTableStartMinus26[i] + 26;
    qpOutVal[0] = qpInVal[0];
    for (int j = 0; j <= m_numPtsInCQPTableMinus1[i]; j++)
    {
      qpInVal[j+1] = qpInVal[j] + m_deltaQpInValMinus1[i][j] + 1;
      qpOutVal[j+1] = qpOutVal[j] + m_deltaQpOutVal[i][j];
    }

    for (int j = 0; j <= m_numPtsInCQPTableMinus1[i]; j++)
    {
      CHECK(qpInVal[j]  < -qpBdOffsetC || qpInVal[j]  > MAX_QP, "qpInVal out of range");
      CHECK(qpOutVal[j] < -qpBdOffsetC || qpOutVal[j] > MAX_QP, "qpOutVal out of range");
    }

    m_chromaQpMappingTables[i][qpInVal[0] + qpBdOffsetC] = qpOutVal[0];
    for (int k = qpInVal[0] - 1; k >= -qpBdOffsetC; k--)
    {
      m_chromaQpMappingTables[i][k + qpBdOffsetC] = Clip3(-qpBdOffsetC, MAX_QP, m_chromaQpMappingTables[i][k + 1 + qpBdOffsetC] - 1);
    }
    for (int j = 0; j <= numPtsInCQPTableMinus1; j++)
    {
      int sh = (m_deltaQpInValMinus1[i][j] + 1) >> 1;
      for (int k = qpInVal[j] + 1, m = 1; k <= qpInVal[j + 1]; k++, m++)
      {
        m_chromaQpMappingTables[i][k + qpBdOffsetC] = m_chromaQpMappingTables[i][qpInVal[j] + qpBdOffsetC]
          + ((qpOutVal[j + 1] - qpOutVal[j]) * m + sh) / (m_deltaQpInValMinus1[i][j]+ 1);
      }
    }
    for (int k = qpInVal[numPtsInCQPTableMinus1+1]+1; k <= MAX_QP; k++)
    {
      m_chromaQpMappingTables[i][k + qpBdOffsetC] = Clip3(-qpBdOffsetC, MAX_QP, m_chromaQpMappingTables[i][k - 1 + qpBdOffsetC] + 1);
    }
  }
}


PPS::PPS()
: ppsId                              (0)
, spsId                              (0)
, picInitQPMinus26                   (0)
, useDQP                             (false)
, usePPSChromaTool                   (false)
, sliceChromaQpFlag                  (false)
, layerId                            (0)
, temporalId                         (0)
, chromaQpOffset                     { 0 }
, jointCbCrQpOffsetPresent           (false)
, chromaQpOffsetListLen              (0)
, numRefIdxL0DefaultActive           (1)
, numRefIdxL1DefaultActive           (1)
, rpl1IdxPresent                     (false)
, weightPred                         (false)
, weightedBiPred                     (0)
, outputFlagPresent                  (false)
, numSubPics                         (0)
, subPicIdMappingInPps               (false)
, subPicIdLen                        (0)
//,   subPicId[MAX_NUM_SUB_PICS];        //!< sub-picture ID for each sub-picture in the sequence
, noPicPartition                     (false)
, log2CtuSize                        (0)
, ctuSize                            (0)
, picWidthInCtu                      (0)
, picHeightInCtu                     (0)
, numExpTileCols                     (1)
, numExpTileRows                     (1)
, numTileCols                        (1)
, numTileRows                        (1)
, rectSlice                          (true)
, singleSlicePerSubPic               (false)
, numSlicesInPic                     (1)
, tileIdxDeltaPresent                (false)
, loopFilterAcrossTilesEnabled       (false)
, loopFilterAcrossSlicesEnabled      (false)
, cabacInitPresent                   (false)
, pictureHeaderExtensionPresent      (false)
, sliceHeaderExtensionPresent        (false)
, deblockingFilterControlPresent     (true)
, deblockingFilterOverrideEnabled    (0)
, deblockingFilterDisabled           (true)
, deblockingFilterBetaOffsetDiv2     {0}
, deblockingFilterTcOffsetDiv2       {0}
, listsModificationPresent           (false)
, rplInfoInPh                        (false)
, dbfInfoInPh                        (false)
, saoInfoInPh                        (false)
, alfInfoInPh                        (false)
, wpInfoInPh                         (false)
, qpDeltaInfoInPh                    (false)
, mixedNaluTypesInPic                (false)
, picWidthInLumaSamples              (0)
, picHeightInLumaSamples             (0)
, wrapAroundEnabled                  (false)
, picWidthMinusWrapAroundOffset      (0)
, wrapAroundOffset                   (0)
, pcv                                (NULL)
{
  chromaQpAdjTableIncludingNullEntry[0].u.comp.CbOffset = 0; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here and never subsequently changed.
  chromaQpAdjTableIncludingNullEntry[0].u.comp.CrOffset = 0;
  chromaQpAdjTableIncludingNullEntry[0].u.comp.JointCbCrOffset = 0;
}

PPS::~PPS()
{
  delete pcv;
}

/**
 - initialize tile row/column sizes and boundaries
 */
void PPS::initTiles()
{
  int colIdx, rowIdx;
  int ctuX, ctuY;

  // check explicit tile column sizes
  uint32_t  remainingWidthInCtu  = picWidthInCtu;
  for( colIdx = 0; colIdx < numExpTileCols; colIdx++ )
  {
    CHECK(tileColWidth[colIdx] > remainingWidthInCtu,    "Tile column width exceeds picture width");
    remainingWidthInCtu -= tileColWidth[colIdx];
  }

  // divide remaining picture width into uniform tile columns
  uint32_t  uniformTileColWidth = tileColWidth[colIdx-1];
  while( remainingWidthInCtu > 0 )
  {
    CHECK(colIdx >= MAX_TILE_COLS, "Number of tile columns exceeds valid range");
    uniformTileColWidth = std::min(remainingWidthInCtu, uniformTileColWidth);
    tileColWidth.push_back( uniformTileColWidth );
    remainingWidthInCtu -= uniformTileColWidth;
    colIdx++;
  }
  numTileCols = colIdx;

  // check explicit tile row sizes
  uint32_t  remainingHeightInCtu  = picHeightInCtu;
  for( rowIdx = 0; rowIdx < numExpTileRows; rowIdx++ )
  {
    CHECK(tileRowHeight[rowIdx] > remainingHeightInCtu,     "Tile row height exceeds picture height");
    remainingHeightInCtu -= tileRowHeight[rowIdx];
  }

  // divide remaining picture height into uniform tile rows
  uint32_t  uniformTileRowHeight = tileRowHeight[rowIdx - 1];
  while( remainingHeightInCtu > 0 )
  {
    uniformTileRowHeight = std::min(remainingHeightInCtu, uniformTileRowHeight);
    tileRowHeight.push_back( uniformTileRowHeight );
    remainingHeightInCtu -= uniformTileRowHeight;
    rowIdx++;
  }
  numTileRows = rowIdx;

  // set left column bounaries
  tileColBd.push_back( 0 );
  for( colIdx = 0; colIdx < numTileCols; colIdx++ )
  {
    tileColBd.push_back( tileColBd[ colIdx ] + tileColWidth[ colIdx ] );
  }

  // set top row bounaries
  tileRowBd.push_back( 0 );
  for( rowIdx = 0; rowIdx < numTileRows; rowIdx++ )
  {
    tileRowBd.push_back( tileRowBd[ rowIdx ] + tileRowHeight[ rowIdx ] );
  }

  // set right column bounaries
  for( colIdx = 0; colIdx < numTileCols; colIdx++ )
  {
    tileColBdRgt.push_back( std::min( ( tileColBd[ colIdx ] + tileColWidth[ colIdx ] ) << log2CtuSize, picWidthInLumaSamples ) );
  }

  // set bottom row bounaries
  for( rowIdx = 0; rowIdx < numTileRows; rowIdx++ )
  {
    tileRowBdBot.push_back( std::min( ( tileRowBd[ rowIdx ] + tileRowHeight[ rowIdx ] ) << log2CtuSize, picHeightInLumaSamples ) );
  }

  // set mapping between horizontal CTU address and tile column index
  colIdx = 0;
  for( ctuX = 0; ctuX <= picWidthInCtu; ctuX++ )
  {
    if( ctuX == tileColBd[ colIdx + 1 ] )
    {
      colIdx++;
    }
    ctuToTileCol.push_back( colIdx );
  }

  // set mapping between vertical CTU address and tile row index
  rowIdx = 0;
  for( ctuY = 0; ctuY <= picHeightInCtu; ctuY++ )
  {
    if( ctuY == tileRowBd[ rowIdx + 1 ] )
    {
      rowIdx++;
    }
    ctuToTileRow.push_back( rowIdx );
  }
}

/**
 - initialize memory for rectangular slice parameters
 */
void PPS::initRectSliceMap( const SPS* sps )
{
  //currently only one slice is allowed
  if( sps )
  {
    CHECK( sps->numSubPics > 1, "SubPic encoding not yet supported" );
  }
  
  CHECK( numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range" );
  sliceMap.resize( numSlicesInPic );

  sliceMap[0].initSliceMap();
  
  uint32_t tileX = 0, tileY = 0;  
  for( uint32_t j = 0; j < numTileRows; j++ )
  {
    for( uint32_t k = 0; k < numTileCols; k++ )
    {
      sliceMap[0].addCtusToSlice( tileColBd[tileX + k], tileColBd[tileX + k +1],
                                  tileRowBd[tileY + j], tileRowBd[tileY + j +1], picWidthInCtu );
    }
  }

  checkSliceMap();
}

void PPS::checkSliceMap()
{
  uint32_t i;
  std::vector<int>  ctuList, sliceList;
  uint32_t picSizeInCtu = picWidthInCtu * picHeightInCtu;
  for( i = 0; i < numSlicesInPic; i++ )
  {
    sliceList = sliceMap[ i ].ctuAddrInSlice;
    ctuList.insert( ctuList.end(), sliceList.begin(), sliceList.end() );
  }
  CHECK( ctuList.size() < picSizeInCtu, "Slice map contains too few CTUs");
  CHECK( ctuList.size() > picSizeInCtu, "Slice map contains too many CTUs");
  std::sort( ctuList.begin(), ctuList.end() );
  for( i = 1; i < ctuList.size(); i++ )
  {
    CHECK( ctuList[i] > ctuList[i-1]+1, "CTU missing in slice map");
    CHECK( ctuList[i] == ctuList[i-1],  "CTU duplicated in slice map");
  }
}

int Slice::getNumEntryPoints( const SPS& sps, const PPS& pps ) const
{
  if (!sps.entryPointsPresent )
  {
    return 0;
  }

  uint32_t ctuAddr, ctuX, ctuY, prevCtuX = 0, prevCtuY = 0;
  int numEntryPoints = 0;

  // count the number of CTUs that align with either the start of a tile, or with an entropy coding sync point
  // ignore the first CTU since it doesn't count as an entry point
  for( uint32_t i = 0; i < sliceMap.numCtuInSlice; i++ )
  {
    ctuAddr = sliceMap.ctuAddrInSlice[i];
    ctuX = ( ctuAddr % pps.picWidthInCtu );
    ctuY = ( ctuAddr / pps.picWidthInCtu );

    if( i != 0 && ( pps.tileRowBd[pps.ctuToTileRow[ctuY]] != pps.tileRowBd[pps.ctuToTileRow[prevCtuY]] || pps.tileColBd[pps.ctuToTileCol[ctuX]] != pps.tileColBd[pps.ctuToTileCol[prevCtuX]] || ( ctuY != prevCtuY && sps.entropyCodingSyncEnabled ) ) )
    {
      numEntryPoints++;
    }
    
    prevCtuX    = ctuX;
    prevCtuY    = ctuY;
  }
  return numEntryPoints;
}


uint32_t PPS::getSubPicIdxFromSubPicId( uint32_t subPicId ) const
{
  for (int i = 0; i < numSubPics; i++)
  {
    if(subPics[i].subPicID == subPicId)
    {
      return i;
    }
  }
  return 0;
}


const SubPic& PPS::getSubPicFromPos(const Position& pos)  const
{
  for (int i = 0; i< numSubPics; i++)
  {
    if (subPics[i].isContainingPos(pos))
    {
      return subPics[i];
    }
  }
  return subPics[0];
}


const SubPic& PPS::getSubPicFromCU(const CodingUnit& cu) const
{
  const Position lumaPos = cu.Y().valid() ? cu.Y().pos() : recalcPosition(cu.chromaFormat, cu.chType, CH_L, cu.blocks[cu.chType].pos());
  return getSubPicFromPos(lumaPos);
}


ReferencePictureList::ReferencePictureList()
  : numberOfShorttermPictures (0)
  , numberOfLongtermPictures  (0)
  , numberOfActivePictures    (0)
  , ltrpInSliceHeader         (0)
  , interLayerPresent     (false)
  , numberOfInterLayerPictures(0)

{
  ::memset(isLongtermRefPic,    0, sizeof(isLongtermRefPic));
  ::memset(refPicIdentifier,    0, sizeof(refPicIdentifier));
  ::memset(deltaPocMSBCycleLT,  0, sizeof(deltaPocMSBCycleLT));
  ::memset(deltaPocMSBPresent,  0, sizeof(deltaPocMSBPresent));
  ::memset(POC,                 0, sizeof(POC));
  ::memset(isInterLayerRefPic,  0, sizeof(isInterLayerRefPic));
  ::memset(interLayerRefPicIdx, 0, sizeof(interLayerRefPicIdx));
}

void ReferencePictureList::initFromGopEntry( const GOPEntry& gopEntry, int l )
{
  *this = ReferencePictureList();
  numberOfShorttermPictures = gopEntry.m_numRefPics[ l ];
  numberOfLongtermPictures  = 0;
  numberOfActivePictures    = gopEntry.m_numRefPicsActive[ l ];
  for( int j = 0; j < gopEntry.m_numRefPics[ l ]; j++ )
  {
    setRefPicIdentifier( j, -gopEntry.m_deltaRefPics[ l ][ j ], 0, false, 0 );
  }
}

void ReferencePictureList::setRefPicIdentifier(int idx, int identifier, bool isLongterm, bool _isInterLayerRefPic, int interLayerIdx)
{
  refPicIdentifier[idx] = identifier;
  isLongtermRefPic[idx] = isLongterm;

  deltaPocMSBPresent[idx] = false;
  deltaPocMSBCycleLT[idx] = 0;

  isInterLayerRefPic[idx] = _isInterLayerRefPic;
  interLayerRefPicIdx[idx] = interLayerIdx;
}

bool ReferencePictureList::isPOCInRefPicList( const int poc, const int currPoc ) const
{
  for (int i = 0; i < numberOfLongtermPictures + numberOfShorttermPictures; i++)
  {
    if (isLongtermRefPic[i] ? (poc == refPicIdentifier[i]) : (poc == currPoc - refPicIdentifier[i]) )
    {
      return true;
    }
  }
  return false;
}


ParameterSetManager::ParameterSetManager()
: m_spsMap      (MAX_NUM_SPS)
, m_ppsMap      (MAX_NUM_PPS)
, m_apsMap      (MAX_NUM_APS * MAX_NUM_APS_TYPE)
, m_dciMap      (MAX_NUM_DCI)
, m_vpsMap      (MAX_NUM_VPS)
, m_activeDCIId (-1)
, m_activeSPSId (-1)
, m_activeVPSId (-1)
{
}


ParameterSetManager::~ParameterSetManager()
{
}


//! activate a PPS and depending on isIDR parameter also SPS
//! \returns true, if activation is successful
ParameterSetManager::PPSErrCodes ParameterSetManager::activatePPS(int ppsId, bool isIRAP)
{
  PPSErrCodes ret=PPS_OK;

  PPS *pps = m_ppsMap.getPS(ppsId);
  if (pps)
  {
    int spsId = pps->spsId;
    if (!isIRAP && (spsId != m_activeSPSId ))
    {
      ret=PPS_ERR_INACTIVE_SPS;
    }
    else
    {
      SPS *sps = m_spsMap.getPS(spsId);
      if (sps)
      {
        int dciId = sps->dciId;
        if ((m_activeDCIId!=-1) && (dciId != m_activeDCIId ))
        {
          ret=PPS_WARN_DCI_ID;
        }
        else
        {
          if (dciId != 0)
          {
            DCI *dci =m_dciMap.getPS(dciId);
            if (dci)
            {
              m_activeDCIId = dciId;
              m_dciMap.setActive(dciId);
            }
            else
            {
              ret=PPS_WARN_NO_DCI;
            }
          }
          else
          {
            // set zero as active DCI ID (special reserved value, no actual DCI)
            m_activeDCIId = dciId;
            m_dciMap.setActive(dciId);
          }
        }

        m_spsMap.clearActive();
        m_spsMap.setActive(spsId);
        m_activeSPSId = spsId;
        m_ppsMap.clearActive();
        m_ppsMap.setActive(ppsId);
        return ret;
      }
      else
      {
        ret=PPS_ERR_NO_SPS;
      }
    }
  }
  else
  {
    ret=PPS_ERR_NO_PPS;
  }

  // Failed to activate if reach here.
  m_activeSPSId=-1;
  m_activeDCIId=-1;
  return ret;
}

bool ParameterSetManager::activateAPS(int apsId, int apsType)
{
  APS *aps = m_apsMap.getPS((apsId << NUM_APS_TYPE_LEN) + apsType);
  if (aps)
  {
    m_apsMap.setActive((apsId << NUM_APS_TYPE_LEN) + apsType);
    return true;
  }

  return false;
}

template <>
void ParameterSetMap<APS>::setID(APS* parameterSet, const int psId)
{
  parameterSet->apsId = psId;
}
template <>
void ParameterSetMap<PPS>::setID(PPS* parameterSet, const int psId)
{
  parameterSet->ppsId = psId;
}

template <>
void ParameterSetMap<SPS>::setID(SPS* parameterSet, const int psId)
{
  parameterSet->spsId = psId;
}

void calculateParameterSetChangedFlag(bool& bChanged, const std::vector<uint8_t>* pOldData, const std::vector<uint8_t>* pNewData)
{
  if (!bChanged)
  {
    if ((pOldData==0 && pNewData!=0) || (pOldData!=0 && pNewData==0))
    {
      bChanged=true;
    }
    else if (pOldData!=0 && pNewData!=0)
    {
      // compare the two
      if (pOldData->size() != pNewData->size())
      {
        bChanged=true;
      }
      else
      {
        const uint8_t *pNewDataArray=&(*pNewData)[0];
        const uint8_t *pOldDataArray=&(*pOldData)[0];
        if (memcmp(pOldDataArray, pNewDataArray, pOldData->size()))
        {
          bChanged=true;
        }
      }
    }
  }
}

//! \}

uint32_t PreCalcValues::getValIdx( const Slice &slice, const ChannelType chType ) const
{
  return slice.isIntra() ? ( ISingleTree ? 0 : ( chType << 1 ) ) : 1;
}

uint32_t PreCalcValues::getMaxMTTDepth( const Slice &slice, const ChannelType chType ) const
{
  if ( slice.picHeader->splitConsOverride )
    { return slice.sliceType == VVENC_I_SLICE ? ((ISingleTree || CH_L == chType) ? slice.picHeader->maxMTTDepth[0] : slice.picHeader->maxMTTDepth[2]) : slice.picHeader->maxMTTDepth[1]; }
  else
    return maxMTTDepth[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMinTSize( const Slice &slice, const ChannelType chType ) const
{
  return minTSize[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMaxBtSize( const Slice &slice, const ChannelType chType ) const
{
  if (slice.picHeader->splitConsOverride)
    return slice.picHeader->maxBTSize[getValIdx(slice, ISingleTree ? CH_L : chType)];
  else
    return maxBtSize[getValIdx(slice, chType)];
}

uint32_t PreCalcValues::getMaxTtSize( const Slice &slice, const ChannelType chType ) const
{
  if ( slice.picHeader->splitConsOverride )
    return slice.picHeader->maxTTSize[getValIdx(slice, ISingleTree ? CH_L : chType)];
  else
    return maxTtSize[getValIdx( slice, chType )];
}

uint32_t PreCalcValues::getMinQtSize( const Slice &slice, const ChannelType chType ) const
{
  if ( slice.picHeader->splitConsOverride )
    return slice.picHeader->minQTSize[getValIdx(slice, ISingleTree ? CH_L : chType)];
  else
    return minQtSize[getValIdx( slice, chType )];
}

Area PreCalcValues::getCtuArea( const int ctuPosX, const int ctuPosY ) const
{
  CHECKD( ctuPosX >= widthInCtus || ctuPosY >= heightInCtus, "CTU idx overflow" );
  const int x = ctuPosX << maxCUSizeLog2;
  const int y = ctuPosY << maxCUSizeLog2;
  const int width = std::min( maxCUSize, lumaWidth - x );
  const int height = std::min( maxCUSize, lumaHeight - y );
  return Area( x, y, width, height );
}


void VPS::deriveOutputLayerSets()
{
  if( maxLayers == 1 )
  {
    totalNumOLSs = 1;
  }
  else if( eachLayerIsAnOls || olsModeIdc < 2 )
  {
    totalNumOLSs = maxLayers;
  }
  else if( olsModeIdc == 2 )
  {
    totalNumOLSs = numOutputLayerSets;
  }

  olsDpbParamsIdx.resize( totalNumOLSs );
  olsDpbPicSize.resize( totalNumOLSs, Size(0, 0) );
  numOutputLayersInOls.resize( totalNumOLSs );
  numLayersInOls.resize( totalNumOLSs );
  outputLayerIdInOls.resize( totalNumOLSs, std::vector<int>( maxLayers, NOT_VALID ) );
  layerIdInOls.resize( totalNumOLSs, std::vector<int>( maxLayers, NOT_VALID ) );

  std::vector<int> numRefLayers( maxLayers );
  std::vector<std::vector<int>> outputLayerIdx( totalNumOLSs, std::vector<int>( maxLayers, NOT_VALID ) );
  std::vector<std::vector<int>> layerIncludedInOlsFlag( totalNumOLSs, std::vector<int>( maxLayers, 0 ) );
  std::vector<std::vector<int>> dependencyFlag( maxLayers, std::vector<int>( maxLayers, NOT_VALID ) );
  std::vector<std::vector<int>> refLayerIdx( maxLayers, std::vector<int>( maxLayers, NOT_VALID ) );

  for( int i = 0; i < maxLayers; i++ )
  {
    int r = 0;

    for( int j = 0; j < maxLayers; j++ )
    {
      dependencyFlag[i][j] = directRefLayer[i][j];

      for( int k = 0; k < i; k++ )
      {
        if( directRefLayer[i][k] && dependencyFlag[k][j] )
        {
          dependencyFlag[i][j] = 1;
        }
      }

      if( dependencyFlag[i][j] )
      {
        refLayerIdx[i][r++] = j;
      }
    }

    numRefLayers[i] = r;
  }

  numOutputLayersInOls[0] = 1;
  outputLayerIdInOls[0][0] = layerId[0];

  for( int i = 1; i < totalNumOLSs; i++ )
  {
    if( eachLayerIsAnOls || olsModeIdc == 0 )
    {
      numOutputLayersInOls[i] = 1;
      outputLayerIdInOls[i][0] = layerId[i];
    }
    else if( olsModeIdc == 1 )
    {
      numOutputLayersInOls[i] = i + 1;

      for( int j = 0; j < numOutputLayersInOls[i]; j++ )
      {
        outputLayerIdInOls[i][j] = layerId[j];
      }
    }
    else if( olsModeIdc == 2 )
    {
      int j = 0;
      for( int k = 0; k < maxLayers; k++ )
      {
        if( olsOutputLayer[i][k] )
        {
          layerIncludedInOlsFlag[i][k] = 1;
          outputLayerIdx[i][j] = k;
          outputLayerIdInOls[i][j++] = layerId[k];
        }
      }
      numOutputLayersInOls[i] = j;

      for( j = 0; j < numOutputLayersInOls[i]; j++ )
      {
        int idx = outputLayerIdx[i][j];
        for( int k = 0; k < numRefLayers[idx]; k++ )
        {
          layerIncludedInOlsFlag[i][refLayerIdx[idx][k]] = 1;
        }
      }
    }
  }

  numLayersInOls[0] = 1;
  layerIdInOls[0][0] = layerId[0];

  for( int i = 1; i < totalNumOLSs; i++ )
  {
    if( eachLayerIsAnOls )
    {
      numLayersInOls[i] = 1;
      layerIdInOls[i][0] = layerId[i];
    }
    else if( olsModeIdc == 0 || olsModeIdc == 1 )
    {
      numLayersInOls[i] = i + 1;
      for( int j = 0; j < numLayersInOls[i]; j++ )
      {
        layerIdInOls[i][j] = layerId[j];
      }
    }
    else if( olsModeIdc == 2 )
    {
      int j = 0;
      for( int k = 0; k < maxLayers; k++ )
      {
        if( layerIncludedInOlsFlag[i][k] )
        {
          layerIdInOls[i][j++] = layerId[k];
        }
      }

      numLayersInOls[i] = j;
    }
  }
}

} // namespace vvenc

//! \}


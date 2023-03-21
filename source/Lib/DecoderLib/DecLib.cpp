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


/** \file     DecLib.cpp
    \brief    decoder class
*/

#include "NALread.h"
#include "DecLib.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitTools.h"
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_TRAFO
#include "CommonLib/TrQuant_EMT.h"
#endif

#include "EncoderLib/EncGOP.h"

#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include <unordered_map>
#include "AnnexBread.h"
#include "NALread.h"

//! \ingroup DecoderLib
//! \{

namespace vvenc {

bool tryDecodePicture( Picture* pcEncPic, const int expectedPoc, const std::string& bitstreamFileName, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>* apsMap, MsgLog& msg, bool bDecodeUntilPocFound /* = false */, int debugPOC /* = -1*/, bool copyToEnc /* = true */ )
{
  PicList* pcListPic = NULL;
  bool     bRet      = false;

  if( pcEncPic )
  {
    if( ! ffwdDecoder.pcDecLib )
    {
      ffwdDecoder.bitstreamFile = new std::ifstream( bitstreamFileName.c_str(), std::ifstream::in | std::ifstream::binary );
      ffwdDecoder.bytestream    = new InputByteStream( *ffwdDecoder.bitstreamFile );

      CHECK( !*ffwdDecoder.bitstreamFile, "failed to open bitstream file " << bitstreamFileName.c_str() << " for reading" ) ;
      // create decoder class
      ffwdDecoder.pcDecLib = new DecLib(msg);
      ffwdDecoder.pcDecLib->create();

      // initialize decoder class
      ffwdDecoder.pcDecLib->init();

      ffwdDecoder.pcDecLib->setDecoderInEncoderMode        ( true );
      ffwdDecoder.pcDecLib->setDebugPOC                    ( debugPOC );
      ffwdDecoder.pcDecLib->setDecodedPictureHashSEIEnabled( true );

      msg.log( VVENC_INFO, "start to decode %s \n", bitstreamFileName.c_str() );
    }
    if(apsMap) 
      ffwdDecoder.pcDecLib->setAPSMapEnc        ( apsMap );

    bool goOn = true;
    DecLib *pcDecLib = ffwdDecoder.pcDecLib;
    CHECK( pcDecLib == nullptr, "error in setup of decoder lib" );

    // main decoder loop
    while( !!*ffwdDecoder.bitstreamFile && goOn )
    {
      InputNALUnit nalu;
      nalu.m_nalUnitType = VVENC_NAL_UNIT_INVALID;

      // determine if next NAL unit will be the first one from a new picture
      bool bNewPicture = pcDecLib->isNewPicture( ffwdDecoder.bitstreamFile,  ffwdDecoder.bytestream );
      bool bNewAccessUnit = bNewPicture && pcDecLib->isNewAccessUnit( bNewPicture, ffwdDecoder.bitstreamFile,  ffwdDecoder.bytestream );
      bNewPicture = bNewPicture && bNewAccessUnit;

      if( !bNewPicture )
      {
        AnnexBStats stats = AnnexBStats();

        byteStreamNALUnit( *ffwdDecoder.bytestream, nalu.getBitstream().getFifo(), stats );

        // call actual decoding function
        if( nalu.getBitstream().getFifo().empty() )
        {
          /* this can happen if the following occur:
           *  - empty input file
           *  - two back-to-back start_code_prefixes
           *  - start_code_prefix immediately followed by EOF
           */
          msg.log( VVENC_WARNING, "Warning: Attempt to decode an empty NAL unit\n" );
        }
        else
        {
          read( nalu, msg );
          int iSkipFrame = 0;
          pcDecLib->decode( nalu, iSkipFrame, ffwdDecoder.iPOCLastDisplay, 0 );
        }
      }

      if( ( bNewPicture || !*ffwdDecoder.bitstreamFile || nalu.m_nalUnitType == VVENC_NAL_UNIT_EOS ) && !pcDecLib->getFirstSliceInSequence() )
      {
        if( ! ffwdDecoder.loopFiltered || *ffwdDecoder.bitstreamFile )
        {
          int poc;
          pcDecLib->finishPictureLight( poc, pcListPic );

          if( pcListPic )
          {
            for( auto & pic : *pcListPic )
            {
              if( copyToEnc )
              {
                if( pic->poc == poc && (!bDecodeUntilPocFound || expectedPoc == poc ) )
                {
                  pcEncPic->createTempBuffers( pic->cs->pcv->maxCUSize );
                  pcEncPic->cs->createCoeffs();
                  pcEncPic->cs->createTempBuffers( true );
                  pcEncPic->cs->initStructData( MAX_INT, false, nullptr );

                  CHECK( pcEncPic->slices.size() == 0, "at least one slice should be available" );

                  CHECK( expectedPoc != poc, "mismatch in POC - check encoder configuration" );

                  if( poc != debugPOC )
                  {
                    for( int i = 0; i < pic->slices.size(); i++ )
                    {
                      if( pcEncPic->slices.size() <= i )
                      {
                        pcEncPic->slices.push_back( new Slice );
                        pcEncPic->slices.back()->pps = pcEncPic->slices[0]->pps;
                        pcEncPic->slices.back()->sps = pcEncPic->slices[0]->sps;
                        pcEncPic->slices.back()->vps = pcEncPic->slices[0]->vps;
                        pcEncPic->slices.back()->pic = pcEncPic->slices[0]->pic;
                      }
                      pcEncPic->slices[i]->copySliceInfo( pic->slices[i], false );
                    }
                  }

                  pcEncPic->cs->slice = pcEncPic->slices.back();

                  {
                    if ( pic->cs->sps->saoEnabled )
                    {
                      pcEncPic->copySAO( *pic, 0 );
                    }

                    if( pic->cs->sps->alfEnabled )
                    {
                      pcEncPic->resizeAlfCtuBuffers( pic->cs->pcv->sizeInCtus );
                      for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
                      {
                        std::copy( pic->m_alfCtuEnabled[ compIdx ].begin(), pic->m_alfCtuEnabled[ compIdx ].end(), pcEncPic->m_alfCtuEnabled[ compIdx ].begin() );
                      }
                      std::copy( pic->m_alfCtbFilterIndex.begin(), pic->m_alfCtbFilterIndex.end(), pcEncPic->m_alfCtbFilterIndex.begin() );

                      std::copy( pic->m_alfCtuAlternative[COMP_Cb].begin(), pic->m_alfCtuAlternative[COMP_Cb].end(), pcEncPic->m_alfCtuAlternative[COMP_Cb].begin() );
                      std::copy( pic->m_alfCtuAlternative[COMP_Cr].begin(), pic->m_alfCtuAlternative[COMP_Cr].end(), pcEncPic->m_alfCtuAlternative[COMP_Cr].begin() );

                      for( int i = 0; i < pic->slices.size(); i++ )
                      {
                        pcEncPic->slices[i]->numAps = (pic->slices[i]->numAps);
                        pcEncPic->slices[i]->setAlfAPSs(pic->slices[i]->alfAps);
                        pcEncPic->slices[i]-> chromaApsId        = pic->slices[i]->chromaApsId;
                        pcEncPic->slices[i]->alfEnabled[COMP_Y]  = pic->slices[i]->alfEnabled[COMP_Y];
                        pcEncPic->slices[i]->alfEnabled[COMP_Cb] = pic->slices[i]->alfEnabled[COMP_Cb];
                        pcEncPic->slices[i]->alfEnabled[COMP_Cr] = pic->slices[i]->alfEnabled[COMP_Cr];
                        pcEncPic->slices[i]->ccAlfCbApsId   = pic->slices[i]->ccAlfCbApsId;
                        pcEncPic->slices[i]->ccAlfCbEnabled = pic->slices[i]->ccAlfCbEnabled;
                        pcEncPic->slices[i]->ccAlfCrApsId   = pic->slices[i]->ccAlfCrApsId;
                        pcEncPic->slices[i]->ccAlfCrEnabled = pic->slices[i]->ccAlfCrEnabled;
                      }
                    }

                    pcDecLib->executeLoopFilters();
                    if ( pic->cs->sps->saoEnabled )
                    {
                      pcEncPic->copySAO( *pic, 1 );
                    }

                    pcEncPic->cs->copyStructure( *pic->cs, CH_L, TREE_D, true, true );

                    if( CS::isDualITree( *pcEncPic->cs ) )
                    {
                      pcEncPic->cs->copyStructure( *pic->cs, CH_C, TREE_D, true, true );
                    }
                  }

                  for( const auto* slice : pcEncPic->slices )
                  {
                    for( int ctu : slice->sliceMap.ctuAddrInSlice )
                    {
                      pcEncPic->ctuSlice[ctu] = slice;
                    }
                  }

                  pcEncPic->cs->slice = pcEncPic->slices[ 0 ];
                  pcEncPic->cs->picHeader->copyPicInfo( pic->cs->picHeader, false );
                  for( auto& cu: pcEncPic->cs->cus)
                  {
                    cu->slice = pcEncPic->cs->slice;
                  }
                  goOn = false; // exit the loop return
                  bRet = true;
                  break;
                }
              }
            }
          }
          // postpone loop filters
          if (!bRet)
          {
            pcDecLib->executeLoopFilters();
          }

          pcDecLib->finishPicture( poc, pcListPic, copyToEnc ? VVENC_DETAILS : VVENC_INFO );

          // write output
          if( ! pcListPic->empty())
          {
            PicList::iterator iterPic   = pcListPic->begin();
            int numPicsNotYetDisplayed = 0;
            int dpbFullness = 0;
            const SPS* activeSPS = (pcListPic->front()->cs->sps);
            uint32_t maxNrSublayers = activeSPS->maxTLayers;
            uint32_t numReorderPicsHighestTid = activeSPS->numReorderPics[maxNrSublayers-1];
            uint32_t maxDecPicBufferingHighestTid =  activeSPS->maxDecPicBuffering[maxNrSublayers-1];
            const VPS* referredVPS = pcListPic->front()->cs->vps;

            if( referredVPS != nullptr && referredVPS->numLayersInOls[referredVPS->targetOlsIdx] > 1 )
            {
              numReorderPicsHighestTid = referredVPS->getNumReorderPics( maxNrSublayers - 1 );
              maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering( maxNrSublayers - 1 );
            }

            while (iterPic != pcListPic->end())
            {
              Picture* pcCurPic = *(iterPic);
              if(pcCurPic->isNeededForOutput && pcCurPic->getPOC() > ffwdDecoder.iPOCLastDisplay)
              {
                numPicsNotYetDisplayed++;
                dpbFullness++;
              }
              else if(pcCurPic->isReferenced)
              {
                dpbFullness++;
              }
              iterPic++;
            }

            iterPic = pcListPic->begin();

            if (numPicsNotYetDisplayed>2)
            {
              iterPic++;
            }

            Picture* pcCurPic = *(iterPic);
            iterPic = pcListPic->begin();

            while (iterPic != pcListPic->end())
            {
              pcCurPic = *(iterPic);

              if(pcCurPic->isNeededForOutput && pcCurPic->getPOC() > ffwdDecoder.iPOCLastDisplay &&
                  (numPicsNotYetDisplayed >  numReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid))
              {
                numPicsNotYetDisplayed--;
                if( ! pcCurPic->isReferenced )
                {
                  dpbFullness--;
                }
                // update POC of display order
                ffwdDecoder.iPOCLastDisplay = pcCurPic->getPOC();

                // erase non-referenced picture in the reference picture list after display
                if( ! pcCurPic->isReferenced && pcCurPic->isReconstructed )
                {
                  pcCurPic->isReconstructed = false;
                }
                pcCurPic->isNeededForOutput = false;
              }

              iterPic++;
            }
          }


          if( ffwdDecoder.bitstreamFile )
          {
            pcDecLib->resetAccessUnitNals();
            pcDecLib->resetAccessUnitApsNals();
          }
        }
        ffwdDecoder.loopFiltered = ( nalu.m_nalUnitType == VVENC_NAL_UNIT_EOS );
        if( nalu.m_nalUnitType == VVENC_NAL_UNIT_EOS )
        {
          pcDecLib->setFirstSliceInSequence( true );
        }

      }
      else if( ( bNewPicture || !*ffwdDecoder.bitstreamFile || nalu.m_nalUnitType == VVENC_NAL_UNIT_EOS ) && pcDecLib->getFirstSliceInSequence() )
      {
        pcDecLib->setFirstSliceInPicture( true );
      }
    }
  }

  if( !bRet )
  {
    CHECK( bDecodeUntilPocFound, " decoding failed - check decodeBitstream2 parameter File: " << bitstreamFileName.c_str() );
    if( ffwdDecoder.pcDecLib )
    {
      ffwdDecoder.pcDecLib->destroy();
      ffwdDecoder.pcDecLib->deletePicBuffer();
      delete ffwdDecoder.pcDecLib;
      ffwdDecoder.pcDecLib = nullptr;
    }
    ffwdDecoder.loopFiltered    = false;
    ffwdDecoder.iPOCLastDisplay = -MAX_INT;

    if( ffwdDecoder.bytestream )
    {
      delete ffwdDecoder.bytestream;
      ffwdDecoder.bytestream = nullptr;
    }

    if( ffwdDecoder.bitstreamFile )
    {
      delete ffwdDecoder.bitstreamFile;
      ffwdDecoder.bitstreamFile = nullptr;
    }
  }

  return bRet;
}

//! \ingroup DecoderLib
//! \{

DecLib::DecLib( MsgLog& logger )
  : msg ( logger )
  , m_iMaxRefPicNum(0)
  , m_associatedIRAPType(VVENC_NAL_UNIT_INVALID)
  , m_pocCRA(0)
  , m_pocRandomAccess(MAX_INT)
  , m_lastRasPoc(MAX_INT)
  , m_cListPic()
  , m_parameterSetManager()
  , m_apcSlicePilot(NULL)
  , m_SEIs()
  , m_cIntraPred()
  , m_cInterPred()
  , m_cTrQuant()
  , m_cSliceDecoder()
  , m_cCuDecoder()
  , m_HLSReader(logger)
  , m_seiReader(logger)
  , m_cLoopFilter()
  , m_cSAO()
  , m_cReshaper()
  , m_pic(NULL)
  , m_prevLayerID(MAX_INT)
  , m_prevPOC(MAX_INT)
  , m_prevTid0POC(0)
  , m_bFirstSliceInPicture(true)
  , m_bFirstSliceInSequence(true)
  , m_prevSliceSkipped(false)
  , m_skippedPOC(0)
  , m_bFirstSliceInBitstream(true)
  , m_lastPOCNoOutputPriorPics(-1)
  , m_isNoOutputPriorPics(false)
  , m_lastNoIncorrectPicOutputFlag(false)
  , m_pDecodedSEIOutputStream(NULL)
  , m_decodedPictureHashSEIEnabled(false)
  , m_numberOfChecksumErrorsDetected(0)
  , m_warningMessageSkipPicture(false)
  , m_prefixSEINALUs()
  , m_debugPOC( -1 )
  , m_isDecoderInEncoder( false )
  , m_vps( nullptr )
  , m_scalingListUpdateFlag(true)
  , m_PreScalingListAPSId(-1)
  , m_maxDecSubPicIdx(0)
  , m_maxDecSliceAddrInSubPic(-1)
  , m_apsMapEnc( nullptr )
{
#if ENABLE_SIMD_OPT_BUFFER && defined( TARGET_SIMD_X86 )
  g_pelBufOP.initPelBufOpsX86();
#endif
#if ENABLE_SIMD_TRAFO  && defined( TARGET_SIMD_X86 )
  g_tCoeffOps.initTCoeffOpsX86();
#endif
}

DecLib::~DecLib()
{
  while (!m_prefixSEINALUs.empty())
  {
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
}

void DecLib::create()
{
  m_apcSlicePilot = new Slice;
  m_uiSliceSegmentIdx = 0;
  m_cRdCost.create();
}

void DecLib::destroy()
{
  delete m_apcSlicePilot;
  m_apcSlicePilot = NULL;

  m_cSliceDecoder.destroy();
}

void DecLib::init()
{
  m_cSliceDecoder.init( &m_CABACDecoder, &m_cCuDecoder );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
}

void DecLib::deletePicBuffer ( )
{
  PicList::iterator  iterPic   = m_cListPic.begin();
  int iSize = int( m_cListPic.size() );

  for (int i = 0; i < iSize; i++ )
  {
    Picture* pic = *(iterPic++);
    pic->destroy( false );

    delete pic;
    pic = NULL;
  }
}

Picture* DecLib::xGetNewPicBuffer ( const SPS &sps, const PPS &pps, const uint32_t temporalLayer )
{
  Picture * pic = nullptr;
  m_iMaxRefPicNum = ( m_vps == nullptr || m_vps->numLayersInOls[m_vps->targetOlsIdx] == 1 ) ? sps.maxDecPicBuffering[temporalLayer] : m_vps->getMaxDecPicBuffering( temporalLayer );     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded uiMaxDecPicBuffering has the space for the picture currently being decoded
  if (m_cListPic.size() < (uint32_t)m_iMaxRefPicNum)
  {
    pic = new Picture();

    pic->create( sps.chromaFormatIdc, Size( pps.picWidthInLumaSamples, pps.picHeightInLumaSamples ), sps.CTUSize, sps.CTUSize + 16, true );

    m_cListPic.push_back( pic );

    return pic;
  }

  bool bBufferIsAvailable = false;
  for(auto * p: m_cListPic)
  {
    pic = p;  // workaround because range-based for-loops don't work with existing variables
    if ( pic->isReconstructed == false && ! pic->isNeededForOutput )
    {
      pic->isNeededForOutput = false;
      bBufferIsAvailable     = true;
      break;
    }

    if( ! pic->isReferenced  && ! pic->isNeededForOutput )
    {
      pic->isNeededForOutput = false;
      pic->isReconstructed   = false;
      bBufferIsAvailable     = true;
      break;
    }
  }

  if( ! bBufferIsAvailable )
  {
    //There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    m_iMaxRefPicNum++;

    pic = new Picture();

    m_cListPic.push_back( pic );

    pic->create( sps.chromaFormatIdc, Size( pps.picWidthInLumaSamples, pps.picHeightInLumaSamples ), sps.CTUSize, sps.CTUSize + 16, true );
  }
  else
  {
    if( !pic->Y().Size::operator==( Size( pps.picWidthInLumaSamples, pps.picHeightInLumaSamples ) ) || pic->cs->pcv->maxCUSize != sps.CTUSize || pic->cs->pcv->maxCUSize != sps.CTUSize )
    {
      pic->destroy( false );
      pic->create( sps.chromaFormatIdc, Size( pps.picWidthInLumaSamples, pps.picHeightInLumaSamples ), sps.CTUSize, sps.CTUSize + 16, true );
    }
  }

  pic->isBorderExtended  = false;
  pic->isNeededForOutput = false;
  pic->isReconstructed   = false;

  return pic;
}


void DecLib::executeLoopFilters()
{
  if( !m_pic )
  {
    return; // nothing to deblock
  }
  CodingStructure& cs = *m_pic->cs;
  DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", cs.slice->poc ) ) );
  DTRACE(g_trace_ctx, D_CRC, "reconstruction");
  DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());

  ReshapeData& reshapeData = m_pic->reshapeData;
  if (cs.sps->lumaReshapeEnable && reshapeData.getSliceReshaperInfo().sliceReshaperEnabled)
  {
    m_pic->getRecoBuf(COMP_Y).rspSignal( reshapeData.getInvLUT());

    DTRACE(g_trace_ctx, D_CRC, "LMCS");
    DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
  }


  // deblocking filter
  m_cLoopFilter.loopFilterPic( cs, false );
  CS::setRefinedMotionField(cs);
  DTRACE(g_trace_ctx, D_CRC, "LoopFilter");
  DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());

  if( cs.sps->saoEnabled )
  {
    m_cSAO.SAOProcess( cs, cs.picture->getSAO() );
    DTRACE(g_trace_ctx, D_CRC, "SAO");
    DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
  }

  if( cs.sps->alfEnabled )
  {
    m_cALF.m_ccAlfFilterParam = cs.slice->ccAlfFilterParam;

     // ALF decodes the differentially coded coefficients and stores them in the parameters structure.
      // Code could be restructured to do directly after parsing. So far we just pass a fresh non-const
      // copy in case the APS gets used more than once.
    m_cALF.ALFProcess( cs );

    DTRACE( g_trace_ctx, D_CRC, "ALF" );
    DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
    DTRACE_PIC_COMP( D_REC_CB_LUMA_ALF,   cs, cs.getRecoBuf(), COMP_Y  );
    DTRACE_PIC_COMP( D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cb );
    DTRACE_PIC_COMP( D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cr );
  }

  // TODO (VG): It looks like a piece of debugging code here, should be removed somehow
  for (int i = 0; i < cs.pps->numSubPics && m_targetSubPicIdx; i++)
  {
    // keep target subpic samples untouched, for other subpics mask their output sample value to 0
    if (i != m_targetSubPicIdx - 1)
    {
      SubPic SubPicNoUse = cs.pps->subPics[i];
      uint32_t left  = SubPicNoUse.subPicLeft;
      uint32_t right = SubPicNoUse.subPicRight;
      uint32_t top   = SubPicNoUse.subPicTop;
      uint32_t bottom= SubPicNoUse.subPicBottom;
      for (uint32_t row = top; row <= bottom; row++)
      {
        for (uint32_t col = left; col <= right; col++)
        {
          cs.getRecoBuf().Y().at(col, row) = 0;
          // for test only, hard coding using 4:2:0 chroma format
          cs.getRecoBuf().Cb().at(col>>1, row>>1) = 0;
          cs.getRecoBuf().Cr().at(col>>1, row>>1) = 0;
        }
      }
    }
  }

}

void DecLib::finishPictureLight(int& poc, PicList*& rpcListPic )
{
  Slice*  slice = m_pic->cs->slice;

  m_pic->isNeededForOutput = slice->picHeader->picOutputFlag;
  m_pic->isReconstructed   = true;

  Slice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = slice->poc;
  rpcListPic          = &m_cListPic;
}

void DecLib::finishPicture(int& poc, PicList*& rpcListPic, vvencMsgLevel msgl )
{
  Slice*  slice = m_pic->cs->slice;

  char c = (slice->isIntra() ? 'I' : slice->isInterP() ? 'P' : 'B');
  if (!m_pic->isReferenced)
  {
    c += 32;  // tolower
  }

  if (slice->isDRAP) c = 'D';

  //-- For time output for each slice
  msg.log( msgl, "POC %4d TId: %1d ( %c-SLICE, QP%3d ) ", slice->poc,
         slice->TLayer,
         c,
         slice->sliceQp );

  for (int iRefList = 0; iRefList < 2; iRefList++)
  {
    msg.log( msgl, "[L%d ", iRefList);
    for (int iRefIndex = 0; iRefIndex < slice->numRefIdx[ iRefList ]; iRefIndex++)
    {
      msg.log( msgl, "%d ", slice->getRefPOC(RefPicList(iRefList), iRefIndex));
    }
    msg.log( msgl, "] ");
  }

  if (m_decodedPictureHashSEIEnabled)
  {
    SEIMessages pictureHashes = getSeisByType(m_pic->SEIs, SEI::DECODED_PICTURE_HASH );
    const SEIDecodedPictureHash *hash = ( pictureHashes.size() > 0 ) ? (SEIDecodedPictureHash*) *(pictureHashes.begin()) : NULL;
    if (pictureHashes.size() > 1)
    {
      msg.log( VVENC_WARNING, "Warning: Got multiple decoded picture hash SEI messages. Using first.");
    }
    m_numberOfChecksumErrorsDetected += calcAndPrintHashStatus(((const Picture*) m_pic)->getRecoBuf(), hash, slice->sps->bitDepths, msgl, msg);
  }

  msg.log( msgl, "\n");

  m_pic->isNeededForOutput = slice->picHeader->picOutputFlag;
  m_pic->isReconstructed   = true;

  Slice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = slice->poc;
  rpcListPic          = &m_cListPic;
  m_bFirstSliceInPicture  = true; // TODO: immer true? hier ist irgendwas faul
  m_maxDecSubPicIdx = 0;
  m_maxDecSliceAddrInSubPic = -1;

  m_pic->cs->releaseIntermediateData();
  m_pic->cs->destroyTempBuffers();
  m_pic->cs->destroyCoeffs();
  m_pic->destroyTempBuffers();
  m_pic->cs->picHeader->initPicHeader();
}

void DecLib::checkNoOutputPriorPics (PicList* pcListPic)
{
  if (!pcListPic || !m_isNoOutputPriorPics)
  {
    return;
  }

  PicList::iterator  iterPic   = pcListPic->begin();

  while (iterPic != pcListPic->end())
  {
    Picture* picTmp = *(iterPic++);
    if (m_lastPOCNoOutputPriorPics != picTmp->getPOC())
    {
      picTmp->isNeededForOutput = false;
    }
  }
}

void DecLib::xUpdateRasInit(Slice* slice)
{
  slice->pendingRasInit = false;
  if ( slice->poc > m_lastRasPoc )
  {
    m_lastRasPoc = MAX_INT;
    slice->pendingRasInit = true;
  }
  if ( slice->isIRAP() )
  {
    m_lastRasPoc = slice->poc;
  }
}

void DecLib::xCreateLostPicture( int iLostPoc )
{
  msg.log( VVENC_INFO, "\ninserting lost poc : %d\n",iLostPoc);
  Picture *cFillPic = xGetNewPicBuffer(*(m_parameterSetManager.getFirstSPS()), *(m_parameterSetManager.getFirstPPS()), 0);

  CHECK( !cFillPic->slices.size(), "No slices in picture" );

  cFillPic->slices[0]->resetSlicePart();

  PicList::iterator iterPic = m_cListPic.begin();
  int closestPoc = 1000000;
  while ( iterPic != m_cListPic.end())
  {
    Picture * pic = *(iterPic++);
    if(abs(pic->getPOC() -iLostPoc)<closestPoc&&abs(pic->getPOC() -iLostPoc)!=0&&pic->getPOC()!=m_apcSlicePilot->poc)
    {
      closestPoc=abs(pic->getPOC() -iLostPoc);
    }
  }
  iterPic = m_cListPic.begin();
  while ( iterPic != m_cListPic.end())
  {
    Picture *pic = *(iterPic++);
    if(abs(pic->getPOC() -iLostPoc)==closestPoc&&pic->getPOC()!=m_apcSlicePilot->poc)
    {
      msg.log( VVENC_INFO, "copying picture %d to %d (%d)\n",pic->getPOC() ,iLostPoc,m_apcSlicePilot->poc);
      cFillPic->getRecoBuf().copyFrom( pic->getRecoBuf() );
      break;
    }
  }

  cFillPic->isReferenced = true;
  cFillPic->slices[0]->poc = iLostPoc;
  xUpdatePreviousTid0POC(cFillPic->slices[0]);
  cFillPic->isReconstructed   = true;
  cFillPic->isNeededForOutput = true;
  if(m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iLostPoc;
  }

}

/**
 - Determine if the first VCL NAL unit of a picture is also the first VCL NAL of an Access Unit
 */
bool DecLib::isSliceNaluFirstInAU( bool newPicture, InputNALUnit &nalu )
{
  // can only be the start of an AU if this is the start of a new picture
  if( newPicture == false )
  {
    return false;
  }

  // should only be called for slice NALU types
  if( nalu.m_nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_TRAIL &&
      nalu.m_nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_STSA &&
      nalu.m_nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_RASL &&
      nalu.m_nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_RADL &&
      nalu.m_nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL &&
      nalu.m_nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP &&
      nalu.m_nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_CRA &&
      nalu.m_nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_GDR )
  {
    return false;
  }

  // check for layer ID less than or equal to previous picture's layer ID
  if( nalu.m_nuhLayerId <= m_prevLayerID )
  {
    return true;
  }

  // get slice POC
  m_apcSlicePilot->picHeader = ( &m_picHeader );
//  m_apcSlicePilot->>initSlice();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.getSlicePoc( m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC );

  // check for different POC
  return (m_apcSlicePilot->poc != m_prevPOC);
}


void activateAPS(PicHeader* picHeader, Slice* pSlice, ParameterSetManager& parameterSetManager, APS** apss, APS* lmcsAPS, APS* scalingListAPS)
{
  //luma APSs
  if (pSlice->alfEnabled[COMP_Y])
  {
    for (int i = 0; i < pSlice->lumaApsId.size(); i++)
    {
      int apsId = pSlice->lumaApsId[i];
      APS* aps = parameterSetManager.getAPS(apsId, ALF_APS);

      if (aps)
      {
        apss[apsId] = aps;
        if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
        {
          THROW("APS activation failed!");
        }

        CHECK( aps->temporalId > pSlice->TLayer, "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
        //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
      }
    }
  }
  if (pSlice->alfEnabled[COMP_Cb]||pSlice->alfEnabled[COMP_Cr])
  {
    //chroma APS
    int apsId = pSlice->chromaApsId;
    APS* aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if (aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK( aps->temporalId > pSlice->TLayer, "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
    }
  }

  CcAlfFilterParam &filterParam = pSlice->ccAlfFilterParam;
  // cleanup before copying
  for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
  {
    memset( filterParam.ccAlfCoeff[COMP_Cb - 1][filterIdx], 0, sizeof(filterParam.ccAlfCoeff[COMP_Cb - 1][filterIdx]) );
    memset( filterParam.ccAlfCoeff[COMP_Cr - 1][filterIdx], 0, sizeof(filterParam.ccAlfCoeff[COMP_Cr - 1][filterIdx]) );
  }
  memset( filterParam.ccAlfFilterIdxEnabled[COMP_Cb - 1], false, sizeof(filterParam.ccAlfFilterIdxEnabled[COMP_Cb - 1]) );
  memset( filterParam.ccAlfFilterIdxEnabled[COMP_Cr - 1], false, sizeof(filterParam.ccAlfFilterIdxEnabled[COMP_Cr - 1]) );

  if(pSlice->ccAlfCbEnabled)
  {
    int apsId = pSlice->ccAlfCbApsId;
    APS *aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if(aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK( aps->temporalId > pSlice->TLayer, "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.

      filterParam.ccAlfFilterEnabled[COMP_Cb - 1] = true;
      filterParam.ccAlfFilterCount[COMP_Cb - 1] = aps->ccAlfParam.ccAlfFilterCount[COMP_Cb - 1];
      for (int filterIdx=0; filterIdx < filterParam.ccAlfFilterCount[COMP_Cb - 1]; filterIdx++ )
      {
        filterParam.ccAlfFilterIdxEnabled[COMP_Cb - 1][filterIdx] = aps->ccAlfParam.ccAlfFilterIdxEnabled[COMP_Cb - 1][filterIdx];
        memcpy(filterParam.ccAlfCoeff[COMP_Cb - 1][filterIdx], aps->ccAlfParam.ccAlfCoeff[COMP_Cb - 1][filterIdx], sizeof(aps->ccAlfParam.ccAlfCoeff[COMP_Cb - 1][filterIdx]));
      }
    }
  }

  if(pSlice->ccAlfCrEnabled)
  {
    int apsId = pSlice->ccAlfCrApsId;
    APS *aps = parameterSetManager.getAPS(apsId, ALF_APS);
    if(aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ALF_APS))
      {
        THROW("APS activation failed!");
      }

      CHECK( aps->temporalId > pSlice->TLayer, "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.

      filterParam.ccAlfFilterEnabled[COMP_Cr - 1] = true;
      filterParam.ccAlfFilterCount[COMP_Cr - 1] = aps->ccAlfParam.ccAlfFilterCount[COMP_Cr - 1];
      for (int filterIdx=0; filterIdx < filterParam.ccAlfFilterCount[COMP_Cr - 1]; filterIdx++ )
      {
        filterParam.ccAlfFilterIdxEnabled[COMP_Cr - 1][filterIdx] = aps->ccAlfParam.ccAlfFilterIdxEnabled[COMP_Cr - 1][filterIdx];
        memcpy(filterParam.ccAlfCoeff[COMP_Cr - 1][filterIdx], aps->ccAlfParam.ccAlfCoeff[COMP_Cr - 1][filterIdx], sizeof(aps->ccAlfParam.ccAlfCoeff[COMP_Cr - 1][filterIdx]));
      }
    }
  }

  if (picHeader->lmcsEnabled && lmcsAPS == nullptr)
  {
    lmcsAPS = parameterSetManager.getAPS(picHeader->lmcsApsId, LMCS_APS);
    CHECK(lmcsAPS == nullptr, "No LMCS APS present");
    if (lmcsAPS)
    {
      parameterSetManager.clearAPSChangedFlag(picHeader->lmcsApsId, LMCS_APS);
      if (false == parameterSetManager.activateAPS(picHeader->lmcsApsId, LMCS_APS))
      {
        THROW("LMCS APS activation failed!");
      }

      CHECK( lmcsAPS->temporalId > pSlice->TLayer, "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
    }
  }
  picHeader->lmcsAps = (lmcsAPS);

  if( picHeader->explicitScalingListEnabled && scalingListAPS == nullptr)
  {
    scalingListAPS = parameterSetManager.getAPS( picHeader->scalingListApsId, SCALING_LIST_APS );
    CHECK( scalingListAPS == nullptr, "No SCALING LIST APS present" );
    if( scalingListAPS )
    {
      parameterSetManager.clearAPSChangedFlag( picHeader->scalingListApsId, SCALING_LIST_APS );
      if( false == parameterSetManager.activateAPS( picHeader->scalingListApsId, SCALING_LIST_APS ) )
      {
        THROW( "SCALING LIST APS activation failed!" );
      }

      CHECK( scalingListAPS->temporalId > pSlice->TLayer, "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit" );
      //ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing a coded slice NAL unit that referrs it.
    }
  }
  picHeader->scalingListAps = (scalingListAPS);
}


void DecLib::xActivateParameterSets( const int layerId)
{
  if (m_bFirstSliceInPicture)
  {
    APS** apss = m_parameterSetManager.getAPSs();
    memset(apss, 0, sizeof(*apss) * ALF_CTB_MAX_NUM_APS);
    const PPS *pps = m_parameterSetManager.getPPS(m_picHeader.ppsId); // this is a temporary PPS object. Do not store this value
    CHECK(pps == 0, "No PPS present");

    const SPS *sps = m_parameterSetManager.getSPS(pps->spsId);             // this is a temporary SPS object. Do not store this value
    CHECK(sps == 0, "No SPS present");

    const VPS *vps = sps->vpsId ? m_parameterSetManager.getVPS( sps->vpsId ) : nullptr;

    if (NULL == pps->pcv)
    {
      const unsigned _maxQtSize[3] = { sps->CTUSize, sps->CTUSize, sps->CTUSize };
      m_parameterSetManager.getPPS( m_picHeader.ppsId )->pcv = new PreCalcValues( *sps, *pps, _maxQtSize, false );
    }
    m_parameterSetManager.clearSPSChangedFlag(sps->spsId);
    m_parameterSetManager.clearPPSChangedFlag(pps->ppsId);

    ParameterSetManager::PPSErrCodes retPPS = m_parameterSetManager.activatePPS(m_picHeader.ppsId,m_apcSlicePilot->isIRAP() );
    if( (int)retPPS != ParameterSetManager::PPS_OK )
    {
      switch(retPPS)
      {
        case ParameterSetManager::PPS_ERR_INACTIVE_SPS: msg.log( VVENC_WARNING, "Warning: tried to activate PPS referring to a inactive SPS at non-IDR."); break;
        case ParameterSetManager::PPS_ERR_NO_SPS:       msg.log( VVENC_WARNING, "Warning: tried to activate a PPS that refers to a non-existing SPS."); break;
        case ParameterSetManager::PPS_ERR_NO_PPS:       msg.log( VVENC_WARNING, "Warning: tried to activate non-existing PPS."); break;
        case ParameterSetManager::PPS_WARN_DCI_ID:      msg.log( VVENC_WARNING, "Warning: tried to activate DCI with different ID than the currently active DCI. This should not happen within the same bitstream!"); break;
        case ParameterSetManager::PPS_WARN_NO_DCI:      msg.log( VVENC_WARNING, "Warning: tried to activate PPS that refers to a non-existing DCI."); break;
        default: break;
      }

      if( (int)retPPS < 0 )
      {
        THROW("Parameter set activation failed!");
      }
    }

    m_parameterSetManager.getApsMap()->clearActive();
    //luma APSs
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      APS* aps = m_parameterSetManager.getAPS(i, ALF_APS);
      if (aps)
      {
        m_parameterSetManager.clearAPSChangedFlag(i, ALF_APS);
        apss[i] = aps;
        if (false == m_parameterSetManager.activateAPS(i, ALF_APS))
        {
          THROW("APS activation failed!");
        }
      }
    }

    APS* lmcsAPS = NULL;
    if (m_picHeader.lmcsEnabled)
    {
      lmcsAPS = m_parameterSetManager.getAPS(m_picHeader.lmcsApsId, LMCS_APS);
      CHECK(lmcsAPS == 0, "No LMCS APS present");
    }

    if (lmcsAPS)
    {
      m_picHeader.lmcsAps = lmcsAPS; //need for transcoding
      m_parameterSetManager.clearAPSChangedFlag(m_picHeader.lmcsApsId, LMCS_APS);
      if (false == m_parameterSetManager.activateAPS(m_picHeader.lmcsApsId, LMCS_APS))
      {
        THROW("LMCS APS activation failed!");
      }
    }

    xParsePrefixSEImessages();

    if( sps->bitDepths[CH_L] > 10 || sps->bitDepths[CH_C] > 10 )
    {
      THROW( "VVenC does not support high bitdepth stream\n" );
    }

    //  Get a new picture buffer. This will also set up m_pic, and therefore give us a SPS and PPS pointer that we can use.
    m_pic = xGetNewPicBuffer (*sps, *pps, m_apcSlicePilot->TLayer);

    m_apcSlicePilot->pps = pps;
    m_apcSlicePilot->picHeader = &m_picHeader;
    m_apcSlicePilot->applyReferencePictureListBasedMarking(m_cListPic, m_apcSlicePilot->rpl[0], m_apcSlicePilot->rpl[1], layerId, *pps);
    m_pic->finalInit(*vps, *sps, *pps, &m_picHeader, m_unitCache, nullptr, apss, lmcsAPS);

    m_pic->createTempBuffers( m_pic->cs->pps->pcv->maxCUSize );
    m_pic->cs->createCoeffs();
    m_pic->cs->createTempBuffers( true );
    m_pic->cs->initStructData( MAX_INT, false, nullptr );

    m_pic->allocateNewSlice();
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    CHECK(m_pic->slices.size() != (m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    // we now have a real slice:
    Slice* pSlice = m_pic->slices[m_uiSliceSegmentIdx];

    // Update the PPS and SPS pointers with the ones of the picture.
    pps=pSlice->pps;
    sps=pSlice->sps;

    // fix Parameter Sets, now that we have the real slice
    m_pic->cs->slice = pSlice;
    m_pic->cs->sps   = sps;
    m_pic->cs->pps   = pps;
    m_pic->cs->vps   = vps;
    memcpy(m_pic->cs->alfAps, apss, sizeof(m_pic->cs->alfAps));
    m_pic->cs->lmcsAps = lmcsAPS;

    m_pic->cs->pcv   = pps->pcv;

    APS *scalinglistAPS = m_picHeader.scalingListAps;
    activateAPS(&m_picHeader, pSlice, m_parameterSetManager, apss, lmcsAPS, scalinglistAPS);

    if( sps->lumaReshapeEnable )
    {
      m_cReshaper.createDec(sps->bitDepths[CH_L]);
    }
    const uint32_t  log2SaoOffsetScaleLuma   = (uint32_t) std::max(0, sps->bitDepths[CH_L] - MAX_SAO_TRUNCATED_BITDEPTH);
    const uint32_t  log2SaoOffsetScaleChroma = (uint32_t) std::max(0, sps->bitDepths[CH_C] - MAX_SAO_TRUNCATED_BITDEPTH);
    m_cSAO.init( sps->chromaFormatIdc, sps->CTUSize, sps->CTUSize, log2SaoOffsetScaleLuma, log2SaoOffsetScaleChroma );
    m_cIntraPred.init( sps->chromaFormatIdc, sps->bitDepths[ CH_L ] );
    m_cInterPred.init( &m_cRdCost, sps->chromaFormatIdc, sps->CTUSize );
    // transfer any SEI messages that have been received to the picture
    m_pic->SEIs = m_SEIs;
    m_SEIs.clear();

    // Recursive structure
    m_cCuDecoder.init( &m_cTrQuant, &m_cIntraPred, &m_cInterPred, sps->chromaFormatIdc );

    m_cTrQuant.init( nullptr, 0, false, false, false, false );
    // RdCost
    m_cRdCost.setCostMode ( VVENC_COST_STANDARD_LOSSY ); // not used in decoder side RdCost stuff -> set to default
    // RdCost
    m_cRdCost.setCostMode ( VVENC_COST_STANDARD_LOSSY ); // not used in decoder side RdCost stuff -> set to default

    m_cSliceDecoder.create();
    if( sps->alfEnabled )
    {
      m_cALF.create( sps->maxPicWidthInLumaSamples, sps->maxPicHeightInLumaSamples, sps->chromaFormatIdc, sps->CTUSize, sps->CTUSize, sps->bitDepths.recon );
    }
    pSlice->ccAlfFilterControl[0] = m_cALF.m_ccAlfFilterControl[COMP_Cb-1];
    pSlice->ccAlfFilterControl[1] = m_cALF.m_ccAlfFilterControl[COMP_Cr-1];
  }
  else
  {
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    m_pic->allocateNewSlice();
    CHECK(m_pic->slices.size() != (size_t)(m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    Slice* pSlice = m_pic->slices[m_uiSliceSegmentIdx]; // we now have a real slice.

    const SPS *sps = pSlice->sps;
    const PPS *pps = pSlice->pps;
    APS** apss = pSlice->alfAps;
    APS *lmcsAPS = m_picHeader.lmcsAps;
    APS *scalinglistAPS = m_picHeader.scalingListAps;

    // fix Parameter Sets, now that we have the real slice
    m_pic->cs->slice = pSlice;
    m_pic->cs->sps   = sps;
    m_pic->cs->pps   = pps;
    memcpy(m_pic->cs->alfAps, apss, sizeof(m_pic->cs->alfAps));
    m_pic->cs->lmcsAps = lmcsAPS;
//    m_pic->cs->scalinglistAps = scalinglistAPS;

    m_pic->cs->pcv   = pps->pcv;

    // check that the current active PPS has not changed...
    if (m_parameterSetManager.getSPSChangedFlag(sps->spsId) )
    {
      THROW("a new SPS has been decoded while processing a picture");
    }
    if (m_parameterSetManager.getPPSChangedFlag(pps->ppsId) )
    {
      THROW("a new PPS has been decoded while processing a picture");
    }
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      APS* aps = m_parameterSetManager.getAPS(i, ALF_APS);
      if (aps && m_parameterSetManager.getAPSChangedFlag(i, ALF_APS))
      {
        THROW("a new APS has been decoded while processing a picture");
      }
    }

    if (lmcsAPS && m_parameterSetManager.getAPSChangedFlag(lmcsAPS->apsId, LMCS_APS) )
    {
      THROW("a new LMCS APS has been decoded while processing a picture");
    }
    if( scalinglistAPS && m_parameterSetManager.getAPSChangedFlag( scalinglistAPS->apsId, SCALING_LIST_APS ) )
    {
      THROW( "a new SCALING LIST APS has been decoded while processing a picture" );
    }

    activateAPS(&m_picHeader, pSlice, m_parameterSetManager, apss, lmcsAPS, scalinglistAPS);

    m_pic->cs->lmcsAps = lmcsAPS;
    m_pic->cs->scalinglistAps = scalinglistAPS;

    xParsePrefixSEImessages();

    // Check if any new SEI has arrived
     if(!m_SEIs.empty())
     {
       // Currently only decoding Unit SEI message occurring between VCL NALUs copied
       SEIMessages& picSEI = m_pic->SEIs;
       SEIMessages decodingUnitInfos = extractSeisByType( picSEI, SEI::DECODING_UNIT_INFO);
       picSEI.insert(picSEI.end(), decodingUnitInfos.begin(), decodingUnitInfos.end());
       deleteSEIs(m_SEIs);
     }
  }
  xCheckParameterSetConstraints(layerId);
}

void DecLib::xCheckParameterSetConstraints(const int layerId)
{
  // Conformance checks
  Slice* slice = m_pic->slices[m_uiSliceSegmentIdx];
  const SPS *sps = slice->sps;
  const PPS *pps = slice->pps;
  const VPS *vps = slice->vps;

  static std::unordered_map<int, int> m_clvssSPSid;
  bool isClvssPu =  slice->nalUnitType >= VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL && slice->nalUnitType <= VVENC_NAL_UNIT_CODED_SLICE_GDR && !pps->mixedNaluTypesInPic;

  if( isClvssPu && m_bFirstSliceInPicture )
  {
    m_clvssSPSid[layerId] = pps->spsId;
  }

//  CHECK( clvssSPSid[layerId] != pps->spsId, "The value of pps_seq_parameter_set_id shall be the same in all PPSs that are referred to by coded pictures in a CLVS" );

  CHECK(sps->GDR == false && m_picHeader.gdrPic, "When gdr_enabled_flag is equal to 0, the value of gdr_pic_flag shall be equal to 0 ");
  if( !sps->weightPred )
  {
    CHECK( pps->weightPred, "When sps_weighted_pred_flag is equal to 0, the value of pps_weighted_pred_flag shall be equal to 0." );
  }

  if( !sps->weightedBiPred )
  {
    CHECK( pps->weightedBiPred, "When sps_weighted_bipred_flag is equal to 0, the value of pps_weighted_bipred_flag shall be equal to 0." );
  }

  const int minCuSize = 1 << sps->log2MinCodingBlockSize;
  CHECK( ( pps->picWidthInLumaSamples % ( std::max( 8, minCuSize) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK( ( pps->picHeightInLumaSamples % ( std::max( 8, minCuSize) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );
  if( !sps->resChangeInClvsEnabled )
  {
    CHECK( pps->picWidthInLumaSamples != sps->maxPicWidthInLumaSamples, "When res_change_in_clvs_allowed_flag equal to 0, the value of pic_width_in_luma_samples shall be equal to pic_width_max_in_luma_samples." );
    CHECK( pps->picHeightInLumaSamples != sps->maxPicHeightInLumaSamples, "When res_change_in_clvs_allowed_flag equal to 0, the value of pic_height_in_luma_samples shall be equal to pic_height_max_in_luma_samples." );
  }
  if( sps->resChangeInClvsEnabled )
  {
    CHECK( sps->subPicInfoPresent != 0, "When res_change_in_clvs_allowed_flag is equal to 1, the value of subpic_info_present_flag shall be equal to 0." );
  }

  CHECK(sps->resChangeInClvsEnabled && sps->virtualBoundariesEnabled, "when the value of res_change_in_clvs_allowed_flag is equal to 1, the value of sps_virtual_boundaries_present_flag shall be equal to 0");

  if( sps->CTUSize + 2 * ( 1 << sps->log2MinCodingBlockSize ) > pps->picWidthInLumaSamples )
  {
    CHECK( sps->wrapAroundEnabled, "Wraparound shall be disabled when the value of ( CtbSizeY / MinCbSizeY + 1) is less than or equal to ( pic_width_in_luma_samples / MinCbSizeY - 1 )" );
  }

  if( vps != nullptr && vps->numOutputLayersInOls[vps->targetOlsIdx] > 1 )
  {
    CHECK( sps->maxPicWidthInLumaSamples > vps->olsDpbPicSize[ vps->targetOlsIdx ].width, "pic_width_max_in_luma_samples shall be less than or equal to the value of ols_dpb_pic_width[ i ]" );
    CHECK( sps->maxPicHeightInLumaSamples > vps->olsDpbPicSize[ vps->targetOlsIdx ].height, "pic_height_max_in_luma_samples shall be less than or equal to the value of ols_dpb_pic_height[ i ]" );
  }

  static std::unordered_map<int, int> m_layerChromaFormat;
  static std::unordered_map<int, int> m_layerBitDepth;

  if (vps != nullptr && vps->maxLayers > 1)
  {
    int curLayerIdx = vps->generalLayerIdx[layerId];
    int curLayerChromaFormat = sps->chromaFormatIdc;
    int curLayerBitDepth = sps->bitDepths[CH_L];

    if (isClvssPu && m_bFirstSliceInPicture)
    {
      m_layerChromaFormat[curLayerIdx] = curLayerChromaFormat;
      m_layerBitDepth[curLayerIdx] = curLayerBitDepth;
    }
    else
    {
      CHECK(m_layerChromaFormat[curLayerIdx] != curLayerChromaFormat, "Different chroma format in the same layer.");
      CHECK(m_layerBitDepth[curLayerIdx] != curLayerBitDepth, "Different bit-depth in the same layer.");
    }

    for (int i = 0; i < curLayerIdx; i++)
    {
      if (vps->directRefLayer[curLayerIdx][i])
      {
        int refLayerChromaFormat = m_layerChromaFormat[i];
        CHECK(curLayerChromaFormat != refLayerChromaFormat, "The chroma formats of the current layer and the reference layer are different");
        int refLayerBitDepth = m_layerBitDepth[i];
        CHECK(curLayerBitDepth != refLayerBitDepth, "The bit-depth of the current layer and the reference layer are different");
      }
    }
  }

  if (sps->profileTierLevel.constraintInfo.oneTilePerPicConstraintFlag)
  {
   // CHECK( pps->numTiles != 1, "When one_tile_per_pic_constraint_flag is equal to 1, each picture shall contain only one tile");
    CHECK( pps->numSlicesInPic != 0, "When one_slice_per_pic_constraint_flag is equal to 1, each picture shall contain only one slice");
  }
}


void DecLib::xParsePrefixSEIsForUnknownVCLNal()
{
  while (!m_prefixSEINALUs.empty())
  {
    // do nothing?
    msg.log( VVENC_NOTICE, "Discarding Prefix SEI associated with unknown VCL NAL unit.\n");
    delete m_prefixSEINALUs.front();
  }
  // TODO: discard following suffix SEIs as well?
}


void DecLib::xParsePrefixSEImessages()
{
  while (!m_prefixSEINALUs.empty())
  {
    InputNALUnit &nalu=*m_prefixSEINALUs.front();
    m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_SEIs, nalu.m_nalUnitType, nalu.m_nuhLayerId, nalu.m_temporalId, m_parameterSetManager.getActiveVPS(), m_parameterSetManager.getActiveSPS(), m_HRD, m_pDecodedSEIOutputStream );
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
}

void DecLib::xDecodePicHeader( InputNALUnit& nalu )
{
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parsePictureHeader( &m_picHeader, &m_parameterSetManager, true );
}

bool DecLib::xDecodeSlice(InputNALUnit &nalu, int& iSkipFrame, int iPOCLastDisplay )
{
  m_apcSlicePilot->resetSlicePart(); // the slice pilot is an object to prepare for a new slice
                                     // it is not associated with picture, sps or pps structures.

  if (m_bFirstSliceInPicture)
  {
    m_uiSliceSegmentIdx = 0;
  }
  else
  {
    m_apcSlicePilot->copySliceInfo( m_pic->slices[m_uiSliceSegmentIdx-1] );
  }

  m_apcSlicePilot->nalUnitType = nalu.m_nalUnitType;
  m_apcSlicePilot->TLayer = nalu.m_temporalId;

  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_apcSlicePilot->ccAlfFilterParam = m_cALF.m_ccAlfFilterParam;

  m_HLSReader.parseSliceHeader( m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC );

  PPS *pps = m_parameterSetManager.getPPS(m_picHeader.ppsId);
  CHECK(pps == 0, "No PPS present");
  SPS *sps = m_parameterSetManager.getSPS(pps->spsId);
  CHECK(sps == 0, "No SPS present");
  VPS *vps = m_parameterSetManager.getVPS(sps->vpsId);

  int currSubPicIdx = pps->getSubPicIdxFromSubPicId( m_apcSlicePilot->sliceSubPicId );
  int currSliceAddr = m_apcSlicePilot->sliceSubPicId;
  for(int sp = 0; sp < currSubPicIdx; sp++)
  {
    currSliceAddr -= pps->subPics[(sp)].numSlicesInSubPic;
  }
  if( currSubPicIdx == m_maxDecSubPicIdx )
  {
    m_maxDecSliceAddrInSubPic = currSliceAddr;
  }
  if( currSubPicIdx > m_maxDecSubPicIdx )
  {
    m_maxDecSubPicIdx = currSubPicIdx;
    m_maxDecSliceAddrInSubPic = currSliceAddr;
  }
  if ((sps->vpsId== 0) && (m_prevLayerID != MAX_INT))
  {
    CHECK(m_prevLayerID != nalu.m_nuhLayerId, "All VCL NAL unit in the CVS shall have the same value of nuh_layer_id "
                                              "when sps_video_parameter_set_id is equal to 0");
  }
  CHECK((sps->vpsId > 0) && (vps == 0), "Invalid VPS");
  if (vps != nullptr && (vps->independentLayer[nalu.m_nuhLayerId] == 0))
  {
    bool pocIsSet = false;
    for(auto auNALit = m_accessUnitPicInfo.begin(); auNALit != m_accessUnitPicInfo.end();auNALit++)
    {
      for (int iRefIdx = 0; iRefIdx < m_apcSlicePilot->numRefIdx[REF_PIC_LIST_0] && !pocIsSet; iRefIdx++)
      {
        if (m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx) && m_apcSlicePilot->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() == (*auNALit).m_POC)
        {
          m_apcSlicePilot->poc = m_apcSlicePilot->refPicList[REF_PIC_LIST_0][ iRefIdx]->poc;
          pocIsSet = true;
        }
      }
      for (int iRefIdx = 0; iRefIdx < m_apcSlicePilot->numRefIdx[REF_PIC_LIST_1] && !pocIsSet; iRefIdx++)
      {
        if (m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx) && m_apcSlicePilot->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() == (*auNALit).m_POC)
        {
          m_apcSlicePilot->poc = m_apcSlicePilot->refPicList[REF_PIC_LIST_1][iRefIdx]->poc;
          pocIsSet = true;
        }
      }
    }
  }

  // update independent slice index
  uint32_t uiIndependentSliceIdx = 0;
  if (!m_bFirstSliceInPicture)
  {
    uiIndependentSliceIdx = m_pic->slices[m_uiSliceSegmentIdx-1]->independentSliceIdx;
      uiIndependentSliceIdx++;
  }
  m_apcSlicePilot->independentSliceIdx = uiIndependentSliceIdx;

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "poc", m_apcSlicePilot->poc ) );

  xUpdatePreviousTid0POC(m_apcSlicePilot);

  m_apcSlicePilot->associatedIRAP = (m_pocCRA);
  m_apcSlicePilot->associatedIRAPType = (m_associatedIRAPType);

  // Notice, we can also run into these part from encoder due to DebugBitstream mode, then the changes of pic.header should be avoided.
  if (m_apcSlicePilot->isIRAP() && !m_isDecoderInEncoder )
  {
    //the inference for NoOutputPriorPicsFlag
    // KJS: This cannot happen at the encoder
    if (!m_bFirstSliceInBitstream && (m_apcSlicePilot->isIRAP() || m_apcSlicePilot->nalUnitType >= VVENC_NAL_UNIT_CODED_SLICE_GDR)
      )
    {
      if (m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA || m_apcSlicePilot->nalUnitType >= VVENC_NAL_UNIT_CODED_SLICE_GDR)
      {
        m_picHeader.noOutputOfPriorPics = (true);
      }
    }
  }

  if ((m_apcSlicePilot->getRapPicFlag()  || m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_GDR) && m_picHeader.noOutputOfPriorPics)
  {
    m_lastPOCNoOutputPriorPics = m_apcSlicePilot->poc;
    m_isNoOutputPriorPics = true;
  }
  else
  {
    m_isNoOutputPriorPics = false;
  }

  //For inference of PicOutputFlag
  if (m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL)
  {
    if ( m_lastNoIncorrectPicOutputFlag )
    {
      m_picHeader.picOutputFlag = (false);
    }
  }

  if (sps->vpsId > 0)
  {
    VPS *vps = m_parameterSetManager.getVPS(sps->vpsId);
    CHECK(vps == 0, "No VPS present");
    if ((vps->olsModeIdc == 0 && vps->generalLayerIdx[nalu.m_nuhLayerId] < (vps->maxLayers - 1) && vps->olsOutputLayer[vps->targetOlsIdx][ vps->maxLayers - 1]) || (vps->olsModeIdc == 2 && !vps->olsOutputLayer[vps->targetOlsIdx][vps->generalLayerIdx[nalu.m_nuhLayerId]]))
    {
      m_picHeader.picOutputFlag = (false);
    }
  }

  if ((m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA || m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_GDR) &&
      m_lastNoIncorrectPicOutputFlag)                     //Reset POC MSB when CRA or GDR has NoIncorrectPicOutputFlag equal to 1
  {
    int iMaxPOClsb = 1 << sps->bitsForPOC;
    m_apcSlicePilot->poc = ( m_apcSlicePilot->poc & (iMaxPOClsb - 1) );
    xUpdatePreviousTid0POC(m_apcSlicePilot);
  }

  AccessUnitPicInfo picInfo;
  picInfo.m_nalUnitType = nalu.m_nalUnitType;
  picInfo.m_nuhLayerId  = nalu.m_nuhLayerId;
  picInfo.m_temporalId  = nalu.m_temporalId;
  picInfo.m_POC         = m_apcSlicePilot->poc;
  m_accessUnitPicInfo.push_back(picInfo);

  // Skip pictures due to random access

  if (isRandomAccessSkipPicture(iSkipFrame, iPOCLastDisplay))
  {
    m_prevSliceSkipped = true;
    m_skippedPOC = m_apcSlicePilot->poc;
    return false;
  }
  // Skip TFD pictures associated with BLA/BLANT pictures

  // clear previous slice skipped flag
  m_prevSliceSkipped = false;

  //we should only get a different poc for a new picture (with CTU address==0)
  if(m_apcSlicePilot->poc != m_prevPOC && !m_bFirstSliceInSequence && (m_apcSlicePilot->sliceMap.ctuAddrInSlice[0] != 0))
  {
    msg.log( VVENC_WARNING, "Warning, the first slice of a picture might have been lost!\n");
  }
  m_prevLayerID = nalu.m_nuhLayerId;

  // leave when a new picture is found
  if(m_apcSlicePilot->sliceMap.ctuAddrInSlice[0] == 0 && !m_bFirstSliceInPicture)
  {
    if (m_prevPOC >= m_pocRandomAccess)
    {
      DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 0 ) );
      m_prevPOC = m_apcSlicePilot->poc;
      return true;
    }
    m_prevPOC = m_apcSlicePilot->poc;
  }
  else
  {
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
  }


  if (sps->ccalfEnabled && m_apcSlicePilot->alfEnabled[COMP_Y])
  {
  }


  //detect lost reference picture and insert copy of earlier frame.
  {
    int lostPoc;
    while( m_apcSlicePilot->isRplPicMissing( m_cListPic, REF_PIC_LIST_0, lostPoc ) )
    {
      msg.log( VVENC_WARNING, "\nCurrent picture: %d reference picture with POC = %3d seems to have been removed or not correctly decoded.", m_apcSlicePilot->poc, lostPoc );
      xCreateLostPicture( lostPoc - 1 );
    }
    while( m_apcSlicePilot->isRplPicMissing( m_cListPic, REF_PIC_LIST_1, lostPoc ) )
    {
      msg.log( VVENC_WARNING, "\nCurrent picture: %d reference picture with POC = %3d seems to have been removed or not correctly decoded.", m_apcSlicePilot->poc, lostPoc );
      xCreateLostPicture( lostPoc - 1 );
    }
  }

  m_prevPOC = m_apcSlicePilot->poc;

  if (m_bFirstSliceInPicture)
  {
    xUpdateRasInit(m_apcSlicePilot);
  }

  // actual decoding starts here
  xActivateParameterSets( nalu.m_nuhLayerId );

  m_bFirstSliceInSequence = false;
  m_bFirstSliceInBitstream  = false;

  Slice* slice = m_pic->slices[ m_uiSliceSegmentIdx ];
  slice->picHeader    = &m_picHeader;
  slice->pic          = m_pic;
  m_pic->poc          = slice->poc;
  m_pic->TLayer       = slice->TLayer;
  m_pic->isInitDone   = true;
  m_pic->isReferenced = true;
  m_pic->TLayer       = nalu.m_temporalId;
  m_pic->layerId      = nalu.m_nuhLayerId;

  if( m_bFirstSliceInPicture )
  {
    m_pic->cs->getLoopFilterParamBuf( EDGE_VER ).memset( 0 );
    m_pic->cs->getLoopFilterParamBuf( EDGE_HOR ).memset( 0 );
  }


  slice->checkCRA(slice->rpl[0], slice->rpl[1], m_pocCRA, m_associatedIRAPType, m_cListPic);
  slice->constructRefPicList(m_cListPic, true);
//  slice->scaleRefPicList( scaledRefPic, m_pic->cs->picHeader, m_parameterSetManager.getAPSs(), m_picHeader.lmcsAps, m_picHeader.scalingListAps, true );

  if (!slice->isIntra())
  {
    bool bLowDelay = true;
    int  iCurrPOC  = slice->poc;
    int iRefIdx = 0;

    for (iRefIdx = 0; iRefIdx < slice->numRefIdx[ REF_PIC_LIST_0 ] && bLowDelay; iRefIdx++)
    {
      if ( slice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
      {
        bLowDelay = false;
      }
    }
    if (slice->isInterB())
    {
      for (iRefIdx = 0; iRefIdx < slice->numRefIdx[ REF_PIC_LIST_1 ] && bLowDelay; iRefIdx++)
      {
        if ( slice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
    }

    slice->checkLDC = (bLowDelay);
  }

  slice->setSMVDParam();

  //---------------
  slice->setRefPOCList();

  if (slice->sps->lumaReshapeEnable )
  {
    if (slice->picHeader->lmcsEnabled )
    {
      APS* lmcsAPS = slice->picHeader->lmcsAps;
      LmcsParam& sInfo = lmcsAPS->lmcsParam;
      LmcsParam& tInfo = m_cReshaper.getSliceReshaperInfo();
      tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
      tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
      memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int)*(PIC_CODE_CW_BINS));
      tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
      tInfo.chrResScalingOffset = sInfo.chrResScalingOffset;
      tInfo.sliceReshaperEnabled = slice->picHeader->lmcsEnabled;
      tInfo.enableChromaAdj = slice->picHeader->lmcsChromaResidualScale;
      tInfo.sliceReshaperModelPresent = true;
    }
    else
    {
      LmcsParam& tInfo = m_cReshaper.getSliceReshaperInfo();
      tInfo.sliceReshaperEnabled = false;
      tInfo.enableChromaAdj = false;
      tInfo.sliceReshaperModelPresent = false;
    }
    if (slice->picHeader->lmcsEnabled )
    {
      m_cReshaper.constructReshaper();
    }
    else
    {
      m_cReshaper.setReshapeFlag(false);
    }
    if( slice->sliceType == VVENC_I_SLICE )
    {
      m_cReshaper.setCTUFlag(false);
    }
    else
    {
      m_cReshaper.setCTUFlag( m_cReshaper.getSliceReshaperInfo().sliceReshaperEnabled );
    }
    m_cReshaper.setVPDULoc(-1, -1);
    m_pic->reshapeData.copyReshapeData( m_cReshaper );
  }
  else
  {
    m_cReshaper.setCTUFlag         ( false );
    m_pic->reshapeData.setCTUFlag( false );
  }


  //  Decode a picture
  m_cSliceDecoder.decompressSlice( slice, &( nalu.getBitstream() ) );

  m_bFirstSliceInPicture = false;
  m_uiSliceSegmentIdx++;

  return false;
}

void DecLib::xDecodeVPS( InputNALUnit& nalu )
{
  VPS* vps = new VPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parseVPS( vps );
  m_parameterSetManager.storeVPS( m_vps, nalu.getBitstream().getFifo());
}

void DecLib::xDecodeDCI( InputNALUnit& nalu )
{
  DCI* dci = new DCI();
  m_HLSReader.setBitstream( &nalu.getBitstream() );

  CHECK( nalu.m_temporalId, "The value of TemporalId of DCI NAL units shall be equal to 0" );

  m_HLSReader.parseDCI( dci );
  m_parameterSetManager.storeDCI( dci, nalu.getBitstream().getFifo() );
}

void DecLib::xDecodeSPS( InputNALUnit& nalu )
{
  SPS* sps = new SPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );

  CHECK( nalu.m_temporalId, "The value of TemporalId of SPS NAL units shall be equal to 0" );

  m_HLSReader.parseSPS( sps );

  DTRACE( g_trace_ctx, D_QP_PER_CTU, "CTU Size: %dx%d", sps->CTUSize, sps->CTUSize );

  m_parameterSetManager.storeSPS( sps, nalu.getBitstream().getFifo() );
}

void DecLib::xDecodePPS( InputNALUnit& nalu )
{
  PPS* pps = new PPS();
  m_HLSReader.setBitstream( &nalu.getBitstream() );
  m_HLSReader.parsePPS( pps, &m_parameterSetManager );
  m_parameterSetManager.storePPS( pps, nalu.getBitstream().getFifo() );
}

void DecLib::xDecodeAPS(InputNALUnit& nalu)
{
  APS* aps = new APS();
  m_HLSReader.setBitstream(&nalu.getBitstream());
  m_HLSReader.parseAPS(aps);
  aps->temporalId = nalu.m_temporalId;
  if( m_apsMapEnc )
  {
    APS* apsEnc = new APS();
    *apsEnc = *aps;
    m_apsMapEnc->storePS( ( apsEnc->apsId << NUM_APS_TYPE_LEN ) + apsEnc->apsType, apsEnc );
  }
  m_parameterSetManager.storeAPS(aps, nalu.getBitstream().getFifo());
}
bool DecLib::decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay, int iTargetOlsIdx)
{
  bool ret;
  // ignore all NAL units of layers > 0

  m_accessUnitNals.push_back( std::pair<vvencNalUnitType, int>( nalu.m_nalUnitType, nalu.m_temporalId ) );

  switch (nalu.m_nalUnitType)
  {
    case VVENC_NAL_UNIT_VPS:
      xDecodeVPS( nalu );
      m_vps->targetOlsIdx = iTargetOlsIdx;
      return false;

    case VVENC_NAL_UNIT_DCI:
      xDecodeDCI( nalu );
      return false;

    case VVENC_NAL_UNIT_SPS:
      xDecodeSPS( nalu );
      return false;

    case VVENC_NAL_UNIT_PPS:
      xDecodePPS( nalu );
      return false;
    case VVENC_NAL_UNIT_PH:
      xDecodePicHeader(nalu);
      return !m_bFirstSliceInPicture;

    case VVENC_NAL_UNIT_PREFIX_APS:
    case VVENC_NAL_UNIT_SUFFIX_APS:
      xDecodeAPS(nalu);
      return false;

    case VVENC_NAL_UNIT_PREFIX_SEI:
      // Buffer up prefix SEI messages until SPS of associated VCL is known.
      m_prefixSEINALUs.push_back(new InputNALUnit(nalu));
      return false;

    case VVENC_NAL_UNIT_SUFFIX_SEI:
      if (m_pic)
      {
        m_seiReader.parseSEImessage( &(nalu.getBitstream()), m_pic->SEIs, nalu.m_nalUnitType, nalu.m_nuhLayerId, nalu.m_temporalId, m_parameterSetManager.getActiveVPS(), m_parameterSetManager.getActiveSPS(), m_HRD, m_pDecodedSEIOutputStream );
      }
      else
      {
        msg.log( VVENC_NOTICE, "Note: received suffix SEI but no picture currently active.\n");
      }
      return false;

    case VVENC_NAL_UNIT_CODED_SLICE_TRAIL:
    case VVENC_NAL_UNIT_CODED_SLICE_STSA:
    case VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case VVENC_NAL_UNIT_CODED_SLICE_CRA:
    case VVENC_NAL_UNIT_CODED_SLICE_GDR:
    case VVENC_NAL_UNIT_CODED_SLICE_RADL:
    case VVENC_NAL_UNIT_CODED_SLICE_RASL:
      ret = xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay);
      return ret;

    case VVENC_NAL_UNIT_EOS:
      m_associatedIRAPType = VVENC_NAL_UNIT_INVALID;
      m_pocCRA = 0;
      m_pocRandomAccess = MAX_INT;
      m_prevLayerID = MAX_INT;
      m_prevPOC = MAX_INT;
      m_prevSliceSkipped = false;
      m_skippedPOC = 0;
      return false;

    case VVENC_NAL_UNIT_ACCESS_UNIT_DELIMITER:
      {
        AUDReader audReader(msg);
        uint32_t picType;
        uint32_t audIrapOrGdrAuFlag;
        audReader.parseAccessUnitDelimiter(&(nalu.getBitstream()),audIrapOrGdrAuFlag,picType);
        return !m_bFirstSliceInPicture;
      }

    case VVENC_NAL_UNIT_EOB:
      return false;

    case VVENC_NAL_UNIT_RESERVED_IRAP_VCL_11:
    case VVENC_NAL_UNIT_RESERVED_IRAP_VCL_12:
      msg.log( VVENC_NOTICE, "Note: found reserved VCL NAL unit.\n");
      xParsePrefixSEIsForUnknownVCLNal();
      return false;
    case VVENC_NAL_UNIT_RESERVED_VCL_4:
    case VVENC_NAL_UNIT_RESERVED_VCL_5:
    case VVENC_NAL_UNIT_RESERVED_VCL_6:
    case VVENC_NAL_UNIT_RESERVED_NVCL_26:
    case VVENC_NAL_UNIT_RESERVED_NVCL_27:
      msg.log( VVENC_NOTICE, "Note: found reserved NAL unit.\n");
      return false;
    case VVENC_NAL_UNIT_UNSPECIFIED_28:
    case VVENC_NAL_UNIT_UNSPECIFIED_29:
    case VVENC_NAL_UNIT_UNSPECIFIED_30:
    case VVENC_NAL_UNIT_UNSPECIFIED_31:
      msg.log( VVENC_NOTICE, "Note: found unspecified NAL unit.\n");
      return false;
    default:
      THROW( "Invalid NAL unit type" );
      break;
  }

  return false;
}


/** Function for checking if picture should be skipped because of random access. This function checks the skipping of pictures in the case of -s option random access.
 *  All pictures prior to the random access point indicated by the counter iSkipFrame are skipped.
 *  It also checks the type of Nal unit type at the random access point.
 *  If the random access point is CRA/CRANT/BLA/BLANT, TFD pictures with POC less than the POC of the random access point are skipped.
 *  If the random access point is IDR all pictures after the random access point are decoded.
 *  If the random access point is none of the above, a warning is issues, and decoding of pictures with POC
 *  equal to or greater than the random access point POC is attempted. For non IDR/CRA/BLA random
 *  access point there is no guarantee that the decoder will not crash.
 */
bool DecLib::isRandomAccessSkipPicture( int& iSkipFrame, int& iPOCLastDisplay )
{
  if (iSkipFrame)
  {
    iSkipFrame--;   // decrement the counter
    return true;
  }
  else if ( m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP )
  {
    m_pocRandomAccess = -MAX_INT; // no need to skip the reordered pictures in IDR, they are decodable.
  }
  else if (m_pocRandomAccess == MAX_INT) // start of random access point, m_pocRandomAccess has not been set yet.
  {
    if (m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA )
    {
      // set the POC random access since we need to skip the reordered pictures in the case of CRA/CRANT/BLA/BLANT.
      m_pocRandomAccess = m_apcSlicePilot->poc;
    }
    else
    {
      if(!m_warningMessageSkipPicture)
      {
        msg.log( VVENC_WARNING, "\nWarning: this is not a valid random access point and the data is discarded until the first CRA picture");
        m_warningMessageSkipPicture = true;
      }
      return true;
    }
  }
  // skip the reordered pictures, if necessary
  else if (m_apcSlicePilot->poc < m_pocRandomAccess && (m_apcSlicePilot->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL))
  {
    iPOCLastDisplay++;
    return true;
  }
  // if we reach here, then the picture is not skipped.
  return false;
}

void DecLib::checkNalUnitConstraints( uint32_t naluType )
{
  if (m_parameterSetManager.getActiveSPS() != NULL )
  {
    const ConstraintInfo *cInfo = &m_parameterSetManager.getActiveSPS()->profileTierLevel.constraintInfo;
    xCheckNalUnitConstraintFlags( cInfo, naluType );
  }
  if (m_parameterSetManager.getActiveDCI() != NULL)
  {
    const DCI *dci = m_parameterSetManager.getActiveDCI();
    for (int i=0; i< (int)dci->profileTierLevel.size(); i++)
    {
      ProfileTierLevel ptl = dci->profileTierLevel[i];
      const ConstraintInfo *cInfo = &ptl.constraintInfo;
      xCheckNalUnitConstraintFlags( cInfo, naluType );
    }
  }
}
void DecLib::xCheckNalUnitConstraintFlags( const ConstraintInfo *cInfo, uint32_t naluType )
{
  if (cInfo != NULL)
  {
    CHECK(cInfo->noTrailConstraintFlag && naluType == VVENC_NAL_UNIT_CODED_SLICE_TRAIL,
      "Non-conforming bitstream. no_trail_constraint_flag is equal to 1 but bitstream contains NAL unit of type TRAIL_NUT.");
    CHECK(cInfo->noStsaConstraintFlag && naluType == VVENC_NAL_UNIT_CODED_SLICE_STSA,
      "Non-conforming bitstream. no_stsa_constraint_flag is equal to 1 but bitstream contains NAL unit of type STSA_NUT.");
    CHECK(cInfo->noRaslConstraintFlag && naluType == VVENC_NAL_UNIT_CODED_SLICE_RASL,
      "Non-conforming bitstream. no_rasl_constraint_flag is equal to 1 but bitstream contains NAL unit of type RASL_NUT.");
    CHECK(cInfo->noRadlConstraintFlag && naluType == VVENC_NAL_UNIT_CODED_SLICE_RADL,
      "Non-conforming bitstream. no_radl_constraint_flag is equal to 1 but bitstream contains NAL unit of type RADL_NUT.");
    CHECK(cInfo->noIdrConstraintFlag && (naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL),
      "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type IDR_W_RADL.");
    CHECK(cInfo->noIdrConstraintFlag && (naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP),
      "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type IDR_N_LP.");
    CHECK(cInfo->noCraConstraintFlag && naluType == VVENC_NAL_UNIT_CODED_SLICE_CRA,
      "Non-conforming bitstream. no_cra_constraint_flag is equal to 1 but bitstream contains NAL unit of type CRA_NUT.");
    CHECK(cInfo->noGdrConstraintFlag && naluType == VVENC_NAL_UNIT_CODED_SLICE_GDR,
      "Non-conforming bitstream. no_gdr_constraint_flag is equal to 1 but bitstream contains NAL unit of type GDR_NUT.");
    CHECK(cInfo->noApsConstraintFlag && naluType == VVENC_NAL_UNIT_PREFIX_APS,
      "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type APS_PREFIX_NUT.");
    CHECK(cInfo->noApsConstraintFlag && naluType == VVENC_NAL_UNIT_SUFFIX_APS,
      "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type APS_SUFFIX_NUT.");
  }
}

/**
- lookahead through next NAL units to determine if current NAL unit is the first NAL unit in a new picture
*/
bool DecLib::isNewPicture(std::ifstream *bitstreamFile, class InputByteStream *bytestream)
{
  bool ret = false;
  bool finished = false;

  // cannot be a new picture if there haven't been any slices yet
  if(getFirstSliceInPicture())
  {
    return false;
  }

  // save stream position for backup
  std::streampos location = bitstreamFile->tellg();

  // look ahead until picture start location is determined
  while (!finished && !!(*bitstreamFile))
  {
    AnnexBStats stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);
    if (nalu.getBitstream().getFifo().empty())
    {
      msg.log( VVENC_WARNING, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      // get next NAL unit type
      read(nalu, msg);
      switch( nalu.m_nalUnitType ) {

        // NUT that indicate the start of a new picture
      case VVENC_NAL_UNIT_ACCESS_UNIT_DELIMITER:
      case VVENC_NAL_UNIT_DCI:
      case VVENC_NAL_UNIT_VPS:
      case VVENC_NAL_UNIT_SPS:
      case VVENC_NAL_UNIT_PPS:
      case VVENC_NAL_UNIT_PH:
        ret = true;
        finished = true;
        break;

      // NUT that may be the start of a new picture - check first bit in slice header
      case VVENC_NAL_UNIT_CODED_SLICE_TRAIL:
      case VVENC_NAL_UNIT_CODED_SLICE_STSA:
      case VVENC_NAL_UNIT_CODED_SLICE_RASL:
      case VVENC_NAL_UNIT_CODED_SLICE_RADL:
      case VVENC_NAL_UNIT_RESERVED_VCL_4:
      case VVENC_NAL_UNIT_RESERVED_VCL_5:
      case VVENC_NAL_UNIT_RESERVED_VCL_6:
      case VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL:
      case VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP:
      case VVENC_NAL_UNIT_CODED_SLICE_CRA:
      case VVENC_NAL_UNIT_CODED_SLICE_GDR:
      case VVENC_NAL_UNIT_RESERVED_IRAP_VCL_11:
      case VVENC_NAL_UNIT_RESERVED_IRAP_VCL_12:
        ret = checkPictureHeaderInSliceHeaderFlag(nalu);
        finished = true;
        break;

      // NUT that are not the start of a new picture
      case VVENC_NAL_UNIT_EOS:
      case VVENC_NAL_UNIT_EOB:
      case VVENC_NAL_UNIT_SUFFIX_APS:
      case VVENC_NAL_UNIT_SUFFIX_SEI:
      case VVENC_NAL_UNIT_FD:
        ret = false;
        finished = true;
        break;

        // NUT that might indicate the start of a new picture - keep looking
      case VVENC_NAL_UNIT_PREFIX_APS:
      case VVENC_NAL_UNIT_PREFIX_SEI:
      case VVENC_NAL_UNIT_RESERVED_NVCL_26:
      case VVENC_NAL_UNIT_RESERVED_NVCL_27:
      case VVENC_NAL_UNIT_UNSPECIFIED_28:
      case VVENC_NAL_UNIT_UNSPECIFIED_29:
      case VVENC_NAL_UNIT_UNSPECIFIED_30:
      case VVENC_NAL_UNIT_UNSPECIFIED_31:
      default:
        break;
      }
    }
  }

  // restore previous stream location - minus 3 due to the need for the annexB parser to read three extra bytes
  bitstreamFile->clear();
  bitstreamFile->seekg(location-std::streamoff(3));
  bytestream->reset();

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}

/**
- lookahead through next NAL units to determine if current NAL unit is the first NAL unit in a new access unit
*/
bool DecLib::isNewAccessUnit( bool newPicture, std::ifstream *bitstreamFile, class InputByteStream *bytestream )
{
  bool ret = false;
  bool finished = false;

  // can only be the start of an AU if this is the start of a new picture
  if( newPicture == false )
  {
    return false;
  }

  // save stream position for backup
  std::streampos location = bitstreamFile->tellg();

  // look ahead until access unit start location is determined
  while (!finished && !!(*bitstreamFile))
  {
    AnnexBStats stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);
    if (nalu.getBitstream().getFifo().empty())
    {
      msg.log( VVENC_WARNING, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      // get next NAL unit type
      read(nalu,msg);
      switch( nalu.m_nalUnitType ) {

        // AUD always indicates the start of a new access unit
      case VVENC_NAL_UNIT_ACCESS_UNIT_DELIMITER:
        ret = true;
        finished = true;
        break;

        // slice types - check layer ID and POC
      case VVENC_NAL_UNIT_CODED_SLICE_TRAIL:
      case VVENC_NAL_UNIT_CODED_SLICE_STSA:
      case VVENC_NAL_UNIT_CODED_SLICE_RASL:
      case VVENC_NAL_UNIT_CODED_SLICE_RADL:
      case VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL:
      case VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP:
      case VVENC_NAL_UNIT_CODED_SLICE_CRA:
      case VVENC_NAL_UNIT_CODED_SLICE_GDR:
        ret = isSliceNaluFirstInAU( newPicture, nalu );
        finished = true;
        break;

        // NUT that are not the start of a new access unit
      case VVENC_NAL_UNIT_EOS:
      case VVENC_NAL_UNIT_EOB:
      case VVENC_NAL_UNIT_SUFFIX_APS:
      case VVENC_NAL_UNIT_SUFFIX_SEI:
      case VVENC_NAL_UNIT_FD:
        ret = false;
        finished = true;
        break;

        // all other NUT - keep looking to find first VCL
      default:
        break;
      }
    }
  }

  // restore previous stream location
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}

} // namespace vvenc

//! \}


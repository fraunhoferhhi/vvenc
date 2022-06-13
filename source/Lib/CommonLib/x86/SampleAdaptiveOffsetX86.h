/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     SampleAdaptiveOffsetX86.h
    \brief    SAO filter class
*/

#pragma once

#include "CommonDefX86.h"
#include "SampleAdaptiveOffset.h"

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {

#define SAO_NUM_OFFSETS                     4 /* number of SAO offset values */
#define SAO_EO_NUM_CATEGORIES               (SAO_NUM_OFFSETS + 1) /* number of different eo categories */
#define TH_SAO 0

template <X86_VEXT vext>
void offsetBlock_SIMD( const int     channelBitDepth,
                                        const ClpRng& clpRng,
                                        int           typeIdx,
                                        int*          offset,
                                        int           startIdx,
                                        const Pel*    srcBlk,
                                        Pel*          resBlk,
                                        ptrdiff_t     srcStride,
                                        ptrdiff_t     resStride,
                                        int           width,
                                        int           height,
                                        uint8_t       availMask,
//                                        bool          isLeftAvail,
//                                        bool          isRightAvail,
//                                        bool          isAboveAvail,
//                                        bool          isBelowAvail,
//                                        bool          isAboveLeftAvail,
//                                        bool          isAboveRightAvail,
//                                        bool          isBelowLeftAvail,
//                                        bool          isBelowRightAvail,
                                        std::vector<int8_t> &signLineBuf1,
                                        std::vector<int8_t> &signLineBuf2)
{

  int x,y, startX, startY, endX, endY, edgeType;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signLeft, signRight, signDown;

  const Pel* srcLine = srcBlk;
  Pel* resLine = resBlk;

  switch(typeIdx)
  {
  case SAO_TYPE_EO_0:
  {
    if (availMask&LeftAvail && availMask&RightAvail)
    {

      int8_t p_eo_offsets[16] = {0,};
      for (int i = 0; i < SAO_EO_NUM_CATEGORIES; i++)
      {
        p_eo_offsets[i] = offset[i];
      }

#ifdef USE_AVX2
      // AVX2
      if ((width>8) && (vext >= AVX2))
      {
        __m256i vsrca,vsrcal,vsrcar;
        __m256i vbaseoffset = _mm256_set1_epi16(2) ;
        __m256i vplusone = _mm256_set1_epi16(1);
        __m256i vzero = _mm256_set1_epi8(0);
        __m256i vibdimax = _mm256_set1_epi16((1<<channelBitDepth) -1 );
        __m256i voffsettbl =  _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i*)p_eo_offsets));

        for (y=0; y< height; y++)
        {
          for (x=0; x< width; x+=16)
          {
            vsrca = _mm256_load_si256((__m256i*)&srcLine[x]);
            vsrcal = _mm256_loadu_si256((__m256i*)&srcLine[x-1]);
            vsrcar = _mm256_loadu_si256((__m256i*)&srcLine[x+1]);
            vsrcal = _mm256_sub_epi16(vsrca, vsrcal);
            vsrcar = _mm256_sub_epi16(vsrca, vsrcar);
            __m256i vsignl = _mm256_sign_epi16(vplusone, vsrcal);
            __m256i vsignr = _mm256_sign_epi16(vplusone, vsrcar);
            __m256i vsign = _mm256_add_epi16(_mm256_add_epi16(vsignl, vsignr), vbaseoffset);
            __m256i veoffsets = _mm256_shuffle_epi8(voffsettbl, vsign);
            veoffsets = _mm256_slli_epi16 (veoffsets,8);
            veoffsets = _mm256_srai_epi16 (veoffsets,8);

            vsrca = _mm256_add_epi16(vsrca, veoffsets);
            vsrca    = _mm256_min_epi16(_mm256_max_epi16(vsrca, vzero), vibdimax);
            _mm256_storeu_si256((__m256i*)&resLine[x], vsrca);
          }
          srcLine  += srcStride;
          resLine += resStride;
        }
      }
      else
#endif
      {
        __m128i vsrca,vsrcal,vsrcar;
        __m128i vbaseoffset = _mm_set1_epi16(2) ;
        __m128i vplusone = _mm_set1_epi16(1);
        __m128i vzero = _mm_set1_epi8(0);
        __m128i vibdimax = _mm_set1_epi16((1<<channelBitDepth) -1 );
        __m128i voffsettbl = _mm_loadu_si128((__m128i*)p_eo_offsets);


        for (y=0; y< height; y++)
        {

          for (x=0; x< width; x+=8)
          {
            vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
            vsrcal = _mm_loadu_si128((__m128i*)&srcLine[x-1]);
            vsrcar = _mm_loadu_si128((__m128i*)&srcLine[x+1]);
            vsrcal = _mm_sub_epi16(vsrca, vsrcal);
            vsrcar = _mm_sub_epi16(vsrca, vsrcar);
            __m128i vsignl = _mm_sign_epi16(vplusone, vsrcal);
            __m128i vsignr = _mm_sign_epi16(vplusone, vsrcar);
            __m128i vsign = _mm_add_epi16(_mm_add_epi16(vsignl, vsignr), vbaseoffset);
            __m128i veoffsets = _mm_shuffle_epi8(voffsettbl, vsign);
            veoffsets = _mm_slli_epi16 (veoffsets,8);
            veoffsets = _mm_srai_epi16 (veoffsets,8);

            vsrca = _mm_add_epi16(vsrca, veoffsets);
            vsrca    = _mm_min_epi16(_mm_max_epi16(vsrca, vzero), vibdimax);
            _mm_store_si128((__m128i*)&resLine[x], vsrca);
          }
          srcLine  += srcStride;
          resLine += resStride;
        }
      }
    }
    else
    {
      offset += 2;
      startX = availMask&LeftAvail ? 0 : 1;
      endX   = availMask&RightAvail ? width : (width -1);
      for (y=0; y< height; y++)
      {
        signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX-1]);
        for (x=startX; x< endX; x++)
        {
          signRight = (int8_t)sgn(srcLine[x] - srcLine[x+1]);
          edgeType =  signRight + signLeft;
          signLeft  = -signRight;

          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
        }
        srcLine  += srcStride;
        resLine += resStride;
      }

    }
  }
  break;
  case SAO_TYPE_EO_90:
  {

    int8_t p_eo_offsets[16] = {0,};
    for (int i = 0; i < SAO_EO_NUM_CATEGORIES; i++)
    {
      p_eo_offsets[i] = offset[i];
    }
    const Pel* srcLineAbove= srcLine- srcStride;
    const Pel* srcLineBelow= srcLine+ srcStride;
    startY=0;
    if (!(availMask&AboveAvail))
    {
      startY=1;
      srcLineAbove= srcLine;
      srcLine  += srcStride;
      resLine += resStride;
      srcLineBelow= srcLine+ srcStride;
    }
    endY=height;
    if (!(availMask&BelowAvail))
    {
      endY=height-1;
    }
#ifdef USE_AVX2
    // AVX2
    if ((width>8) && (vext >= AVX2))
    {
      __m256i vsrca,vsrcat,vsrcab;

      __m256i vbaseoffset = _mm256_set1_epi16(2) ;
      __m256i vplusone = _mm256_set1_epi16(1);
      __m256i vzero = _mm256_set1_epi8(0);
      __m256i vibdimax = _mm256_set1_epi16((1<<channelBitDepth) -1 );
      __m256i voffsettbl =  _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i*)p_eo_offsets));
      const Pel* srcLineBelow= srcLine+ srcStride;

      for (y=startY; y< endY; y++)
      {
        for (x=0; x< width; x+=16)
        {
          vsrca = _mm256_load_si256((__m256i*)&srcLine[x]);
          vsrcat = _mm256_loadu_si256((__m256i*)&srcLineAbove[x]);
          vsrcab = _mm256_loadu_si256((__m256i*)&srcLineBelow[x]);
          vsrcat = _mm256_sub_epi16(vsrca, vsrcat);
          vsrcab = _mm256_sub_epi16(vsrca, vsrcab);
          __m256i vsignt = _mm256_sign_epi16(vplusone, vsrcat);
          __m256i vsignb = _mm256_sign_epi16(vplusone, vsrcab);
          __m256i vsign = _mm256_add_epi16(_mm256_add_epi16(vsignt, vsignb), vbaseoffset);
          __m256i veoffsets = _mm256_shuffle_epi8(voffsettbl, vsign);
          veoffsets = _mm256_slli_epi16 (veoffsets,8);
          veoffsets = _mm256_srai_epi16 (veoffsets,8);

          vsrca = _mm256_add_epi16(vsrca, veoffsets);
          vsrca    = _mm256_min_epi16(_mm256_max_epi16(vsrca, vzero), vibdimax);
          _mm256_storeu_si256((__m256i*)&resLine[x], vsrca);
        }
        srcLine  += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
    else
#endif
    {
      __m128i vsrca,vsrcat,vsrcab;
      __m128i vbaseoffset = _mm_set1_epi16(2) ;
      __m128i vplusone = _mm_set1_epi16(1);
      __m128i vzero = _mm_set1_epi8(0);
      __m128i vibdimax = _mm_set1_epi16((1<<channelBitDepth) -1 );
      __m128i voffsettbl = _mm_loadu_si128((__m128i*)p_eo_offsets);


      for (y=startY; y< endY; y++)
      {
        for (x=0; x< width; x+=8)
        {
          vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
          vsrcat = _mm_loadu_si128((__m128i*)&srcLineAbove[x]);
          vsrcab = _mm_loadu_si128((__m128i*)&srcLineBelow[x]);
          vsrcat = _mm_sub_epi16(vsrca, vsrcat);
          vsrcab = _mm_sub_epi16(vsrca, vsrcab);
          __m128i vsignt = _mm_sign_epi16(vplusone, vsrcat);
          __m128i vsignb = _mm_sign_epi16(vplusone, vsrcab);
          __m128i vsign = _mm_add_epi16(_mm_add_epi16(vsignt, vsignb), vbaseoffset);
          __m128i veoffsets = _mm_shuffle_epi8(voffsettbl, vsign);
          veoffsets = _mm_slli_epi16 (veoffsets,8);
          veoffsets = _mm_srai_epi16 (veoffsets,8);

          vsrca = _mm_add_epi16(vsrca, veoffsets);
          vsrca    = _mm_min_epi16(_mm_max_epi16(vsrca, vzero), vibdimax);
          _mm_store_si128((__m128i*)&resLine[x], vsrca);
        }
        srcLine  += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
  }
  break;
  case SAO_TYPE_EO_135:
  {
//    if (isLeftAvail && isRightAvail && isAboveLeftAvail && isBelowRightAvail )
    if((LeftAvail|RightAvail|AboveLeftAvail|BelowRightAvail) == (int)(availMask&(LeftAvail|RightAvail|AboveLeftAvail|BelowRightAvail)))
    {

      int8_t p_eo_offsets[16] = {0,};
      for (int i = 0; i < SAO_EO_NUM_CATEGORIES; i++)
      {
        p_eo_offsets[i] = offset[i];
      }
      const Pel* srcLineAbove= srcLine- srcStride;
      const Pel* srcLineBelow= srcLine+ srcStride;
      startY=0;
      if (!(availMask&AboveAvail))
      {
        startY=1;
        srcLineAbove= srcLine;
        srcLine  += srcStride;
        resLine += resStride;
        srcLineBelow= srcLine+ srcStride;
      }
      endY=height;
      if (!(availMask&BelowAvail))
      {
        endY=height-1;
      }
#ifdef USE_AVX2
      // AVX2
      if ((width>8) && (vext >= AVX2))
      {
        __m256i vsrca,vsrcat,vsrcab;

        __m256i vbaseoffset = _mm256_set1_epi16(2) ;
        __m256i vplusone = _mm256_set1_epi16(1);
        __m256i vzero = _mm256_set1_epi8(0);
        __m256i vibdimax = _mm256_set1_epi16((1<<channelBitDepth) -1 );
        __m256i voffsettbl =  _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i*)p_eo_offsets));
        const Pel* srcLineBelow= srcLine+ srcStride;

        for (y=startY; y< endY; y++)
        {
          for (x=0; x< width; x+=16)
          {
            vsrca = _mm256_load_si256((__m256i*)&srcLine[x]);
            vsrcat = _mm256_loadu_si256((__m256i*)&srcLineAbove[x-1]);
            vsrcab = _mm256_loadu_si256((__m256i*)&srcLineBelow[x+1]);
            vsrcat = _mm256_sub_epi16(vsrca, vsrcat);
            vsrcab = _mm256_sub_epi16(vsrca, vsrcab);
            __m256i vsignt = _mm256_sign_epi16(vplusone, vsrcat);
            __m256i vsignb = _mm256_sign_epi16(vplusone, vsrcab);
            __m256i vsign = _mm256_add_epi16(_mm256_add_epi16(vsignt, vsignb), vbaseoffset);
            __m256i veoffsets = _mm256_shuffle_epi8(voffsettbl, vsign);
            veoffsets = _mm256_slli_epi16 (veoffsets,8);
            veoffsets = _mm256_srai_epi16 (veoffsets,8);

            vsrca = _mm256_add_epi16(vsrca, veoffsets);
            vsrca    = _mm256_min_epi16(_mm256_max_epi16(vsrca, vzero), vibdimax);
            _mm256_storeu_si256((__m256i*)&resLine[x], vsrca);
          }
          srcLine  += srcStride;
          srcLineBelow += srcStride;
          srcLineAbove += srcStride;
          resLine += resStride;
        }
      }
      else
#endif
      {
        __m128i vsrca,vsrcat,vsrcab;
        __m128i vbaseoffset = _mm_set1_epi16(2) ;
        __m128i vplusone = _mm_set1_epi16(1);
        __m128i vzero = _mm_set1_epi8(0);
        __m128i vibdimax = _mm_set1_epi16((1<<channelBitDepth) -1 );
        __m128i voffsettbl = _mm_loadu_si128((__m128i*)p_eo_offsets);


        for (y=startY; y< endY; y++)
        {
          for (x=0; x< width; x+=8)
          {
            vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
            vsrcat = _mm_loadu_si128((__m128i*)&srcLineAbove[x-1]);
            vsrcab = _mm_loadu_si128((__m128i*)&srcLineBelow[x+1]);
            vsrcat = _mm_sub_epi16(vsrca, vsrcat);
            vsrcab = _mm_sub_epi16(vsrca, vsrcab);
            __m128i vsignt = _mm_sign_epi16(vplusone, vsrcat);
            __m128i vsignb = _mm_sign_epi16(vplusone, vsrcab);
            __m128i vsign = _mm_add_epi16(_mm_add_epi16(vsignt, vsignb), vbaseoffset);
            __m128i veoffsets = _mm_shuffle_epi8(voffsettbl, vsign);
            veoffsets = _mm_slli_epi16 (veoffsets,8);
            veoffsets = _mm_srai_epi16 (veoffsets,8);

            vsrca = _mm_add_epi16(vsrca, veoffsets);
            vsrca    = _mm_min_epi16(_mm_max_epi16(vsrca, vzero), vibdimax);
            _mm_store_si128((__m128i*)&resLine[x], vsrca);
          }
          srcLine  += srcStride;
          srcLineBelow += srcStride;
          srcLineAbove += srcStride;
          resLine += resStride;
        }
      }


    }
    else
    {
      offset += 2;
      int8_t *signUpLine, *signDownLine, *signTmpLine;

      signUpLine  = &signLineBuf1[0];
      signDownLine= &signLineBuf2[0];

      startX = availMask&LeftAvail ? 0 : 1 ;
      endX   = availMask&RightAvail ? width : (width-1);

      //prepare 2nd line's upper sign
      const Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX; x< endX+1; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x- 1]);
      }

      //1st line
      const Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = availMask&AboveLeftAvail ? 0 : 1;
      firstLineEndX   = availMask&AboveAvail? endX: 1;
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType  =  sgn(srcLine[x] - srcLineAbove[x- 1]) - signUpLine[x+1];

        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine  += srcStride;
      resLine  += resStride;


      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=startX; x<endX; x++)
        {
          signDown =  (int8_t)sgn(srcLine[x] - srcLineBelow[x+ 1]);
          edgeType =  signDown + signUpLine[x];
          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);

          signDownLine[x+1] = -signDown;
        }
        signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX-1]);

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;
        signDownLine = signTmpLine;

        srcLine += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = availMask&BelowAvail ? startX : (width -1);
      lastLineEndX   = availMask&BelowRightAvail ? width : (width -1);
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType =  sgn(srcLine[x] - srcLineBelow[x+ 1]) + signUpLine[x];
        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);

      }

    }
  }
  break;
  case SAO_TYPE_EO_45:
  {
//    if (isLeftAvail && isRightAvail && isAboveLeftAvail && isBelowRightAvail )
    if((LeftAvail|RightAvail|AboveLeftAvail|BelowRightAvail) == ((int)availMask&(LeftAvail|RightAvail|AboveLeftAvail|BelowRightAvail)))
    {

      int8_t p_eo_offsets[16] = {0,};
      for (int i = 0; i < SAO_EO_NUM_CATEGORIES; i++)
      {
        p_eo_offsets[i] = offset[i];
      }
      const Pel* srcLineAbove= srcLine- srcStride;
      const Pel* srcLineBelow= srcLine+ srcStride;
      startY=0;
      if (!(availMask&AboveAvail))
      {
        startY=1;
        srcLineAbove= srcLine;
        srcLine  += srcStride;
        resLine += resStride;
        srcLineBelow= srcLine+ srcStride;
      }
      endY=height;
      if (!(availMask&BelowAvail))
      {
        endY=height-1;
      }
#ifdef USE_AVX2
      // AVX2
      if ((width>8) && (vext >= AVX2))
      {
        __m256i vsrca,vsrcat,vsrcab;
        __m256i vbaseoffset = _mm256_set1_epi16(2) ;
        __m256i vplusone = _mm256_set1_epi16(1);
        __m256i vzero = _mm256_set1_epi8(0);
        __m256i vibdimax = _mm256_set1_epi16((1<<channelBitDepth) -1 );
        __m256i voffsettbl =  _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i*)p_eo_offsets));
        const Pel* srcLineBelow= srcLine+ srcStride;

        for (y=startY; y< endY; y++)
        {
          for (x=0; x< width; x+=16)
          {
            vsrca = _mm256_load_si256((__m256i*)&srcLine[x]);
            vsrcat = _mm256_loadu_si256((__m256i*)&srcLineAbove[x+1]);
            vsrcab = _mm256_loadu_si256((__m256i*)&srcLineBelow[x-1]);
            vsrcat = _mm256_sub_epi16(vsrca, vsrcat);
            vsrcab = _mm256_sub_epi16(vsrca, vsrcab);
            __m256i vsignt = _mm256_sign_epi16(vplusone, vsrcat);
            __m256i vsignb = _mm256_sign_epi16(vplusone, vsrcab);
            __m256i vsign = _mm256_add_epi16(_mm256_add_epi16(vsignt, vsignb), vbaseoffset);
            __m256i veoffsets = _mm256_shuffle_epi8(voffsettbl, vsign);
            veoffsets = _mm256_slli_epi16 (veoffsets,8);
            veoffsets = _mm256_srai_epi16 (veoffsets,8);

            vsrca = _mm256_add_epi16(vsrca, veoffsets);
            vsrca    = _mm256_min_epi16(_mm256_max_epi16(vsrca, vzero), vibdimax);
            _mm256_storeu_si256((__m256i*)&resLine[x], vsrca);
          }
          srcLine  += srcStride;
          srcLineBelow += srcStride;
          srcLineAbove += srcStride;
          resLine += resStride;
        }
      }
      else
#endif
      {
        __m128i vsrca,vsrcat,vsrcab;
        __m128i vbaseoffset = _mm_set1_epi16(2) ;
        __m128i vplusone = _mm_set1_epi16(1);
        __m128i vzero = _mm_set1_epi8(0);
        __m128i vibdimax = _mm_set1_epi16((1<<channelBitDepth) -1 );
        __m128i voffsettbl = _mm_loadu_si128((__m128i*)p_eo_offsets);

        for (y=startY; y< endY; y++)
        {
          for (x=0; x< width; x+=8)
          {
            vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
            vsrcat = _mm_loadu_si128((__m128i*)&srcLineAbove[x+1]);
            vsrcab = _mm_loadu_si128((__m128i*)&srcLineBelow[x-1]);
            vsrcat = _mm_sub_epi16(vsrca, vsrcat);
            vsrcab = _mm_sub_epi16(vsrca, vsrcab);
            __m128i vsignt = _mm_sign_epi16(vplusone, vsrcat);
            __m128i vsignb = _mm_sign_epi16(vplusone, vsrcab);
            __m128i vsign = _mm_add_epi16(_mm_add_epi16(vsignt, vsignb), vbaseoffset);
            __m128i veoffsets = _mm_shuffle_epi8(voffsettbl, vsign);
            veoffsets = _mm_slli_epi16 (veoffsets,8);
            veoffsets = _mm_srai_epi16 (veoffsets,8);

            vsrca = _mm_add_epi16(vsrca, veoffsets);
            vsrca    = _mm_min_epi16(_mm_max_epi16(vsrca, vzero), vibdimax);
            _mm_store_si128((__m128i*)&resLine[x], vsrca);
          }
          srcLine  += srcStride;
          srcLineBelow += srcStride;
          srcLineAbove += srcStride;
          resLine += resStride;
        }
      }
    }
    else
    {
      offset += 2;
      int8_t *signUpLine = &signLineBuf1[1];

      startX = availMask&LeftAvail ? 0 : 1;
      endX   = availMask&RightAvail ? width : (width -1);

      //prepare 2nd line upper sign
      const Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX-1; x< endX; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x+1]);
      }
      //first line
      const Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = availMask&AboveAvail ? startX : (width -1 );
      firstLineEndX   = availMask&AboveRightAvail ? width : (width-1);
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) -signUpLine[x-1];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;

      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for(x= startX; x< endX; x++)
        {
          signDown =  (int8_t)sgn(srcLine[x] - srcLineBelow[x-1]);
          edgeType =  signDown + signUpLine[x];
          resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
          signUpLine[x-1] = -signDown;
        }
        signUpLine[endX-1] = (int8_t)sgn(srcLineBelow[endX-1] - srcLine[endX]);
        srcLine  += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = availMask&BelowLeftAvail ? 0 : 1;
      lastLineEndX   = availMask&BelowAvail ? endX : 1;
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + signUpLine[x];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);

      }

    }
  }
  break;
  case SAO_TYPE_BO:
  {
    const int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
    int8_t p_eo_offsets[16] = {0,};
    for (int i = 0; i < 4; i++)
    {
      p_eo_offsets[i] = offset[startIdx+i];
    }
#ifdef USE_AVX2
    // AVX2
    if ((width>8) && (vext >= AVX2))
    {
      __m256i vsrc;
      __m256i vbaseoffset = _mm256_set1_epi16(startIdx) ;
      __m256i vminus = _mm256_set1_epi8(-1);
      __m256i vzero = _mm256_set1_epi8(0);

      __m256i vfour = _mm256_set1_epi16(4);
      __m256i vibdimax = _mm256_set1_epi16((1<<channelBitDepth) -1 );
      __m256i voffsettbl =  _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i*)p_eo_offsets));

      for (y=0; y< height; y++)
      {
        for (x=0; x< width; x+=16)
        {
          vsrc = _mm256_load_si256((__m256i*)&srcLine[x]);
          __m256i bands = _mm256_srai_epi16(vsrc, shiftBits);
          bands = _mm256_sub_epi16(bands, vbaseoffset);
          __m256i mask1 = _mm256_cmpgt_epi16(bands,vminus);
          __m256i mask2 = _mm256_cmpgt_epi16(vfour,bands);

          __m256i veoffsets = _mm256_shuffle_epi8(voffsettbl, bands);
          veoffsets = _mm256_slli_epi16 (veoffsets,8);
          veoffsets = _mm256_srai_epi16 (veoffsets,8);

          veoffsets = _mm256_and_si256(veoffsets,mask1);
          veoffsets = _mm256_and_si256(veoffsets,mask2);

          vsrc = _mm256_add_epi16(vsrc, veoffsets);
          vsrc    = _mm256_min_epi16(_mm256_max_epi16(vsrc, vzero), vibdimax);
          _mm256_storeu_si256((__m256i*)&resLine[x], vsrc);
        }
        srcLine  += srcStride;
        resLine += resStride;
      }

    }
    else
#endif
    {
      __m128i vsrc;
      __m128i vbaseoffset = _mm_set1_epi16(startIdx) ;
      __m128i vminus = _mm_set1_epi8(-1);
      __m128i vzero = _mm_set1_epi8(0);

      __m128i vfour = _mm_set1_epi16(4);
      __m128i vibdimax = _mm_set1_epi16((1<<channelBitDepth) -1 );
      __m128i voffsettbl = _mm_loadu_si128((__m128i*)p_eo_offsets);
      for (y=0; y< height; y++)
      {
        for (x=0; x< width; x+=8)
        {
          vsrc = _mm_load_si128((__m128i*)&srcLine[x]);
          __m128i bands = _mm_srai_epi16(vsrc, shiftBits);
          bands = _mm_sub_epi16(bands, vbaseoffset);
          __m128i mask1 = _mm_cmpgt_epi16(bands,vminus);
          __m128i mask2 = _mm_cmplt_epi16(bands,vfour);

          __m128i veoffsets = _mm_shuffle_epi8(voffsettbl, bands);
          veoffsets = _mm_slli_epi16 (veoffsets,8);
          veoffsets = _mm_srai_epi16 (veoffsets,8);

          veoffsets = _mm_and_si128(veoffsets,mask1);
          veoffsets = _mm_and_si128(veoffsets,mask2);

          vsrc = _mm_add_epi16(vsrc, veoffsets);
          vsrc    = _mm_min_epi16(_mm_max_epi16(vsrc, vzero), vibdimax);
          _mm_store_si128((__m128i*)&resLine[x], vsrc);
        }
        srcLine  += srcStride;
        resLine += resStride;
      }
    }
  }
  break;
  default:
  {
    THROW("Not a supported SAO types\n");
  }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template <X86_VEXT vext>
void calcSaoStatisticsBo_SIMD(Pel*  srcLine,Pel*  orgLine,int width,int endX,int endY,int srcStride,int orgStride,int channelBitDepth, int64_t *count,int64_t  *diff)
{
  if ( width % 16 == 0 )
  {
    int iNaRight=width-endX;
    int x;
    int i_bo_range_shift = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
    __m128i vzero = _mm_setzero_si128();
    for (int y=0; y<endY; y++)
    {
      for (x=0; x<endX-16; x+=16)
      {
        __m128i vsrca, vsrcb;
        __m128i vdiffa,vdiffb;
        if (sizeof(Pel) == 1){
          __m128i vsrc = _mm_load_si128((__m128i*)&srcLine[x]);
          vsrca = _mm_unpacklo_epi8(vsrc, vzero);
          vsrcb = _mm_unpackhi_epi8(vsrc, vzero);
          __m128i vorg  = _mm_load_si128((__m128i*)&orgLine[x]);
          __m128i vorga = _mm_unpacklo_epi8(vorg, vzero);
          __m128i vorgb = _mm_unpackhi_epi8(vorg, vzero);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        else
        {
          vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
          vsrcb = _mm_load_si128((__m128i*)&srcLine[x+8]);
          __m128i vorga = _mm_load_si128((__m128i*)&orgLine[x]);
          __m128i vorgb = _mm_load_si128((__m128i*)&orgLine[x+8]);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        __m128i vbanda = _mm_srai_epi16(vsrca, i_bo_range_shift);
        __m128i vbandb = _mm_srai_epi16(vsrcb, i_bo_range_shift);
        int iBand;
        // since gcc 4.6 synopsis of _mm_extract_epi16 has changed to (int)(unsigned short)_mm_extract_epi16()
        // therefore cast result to short to have signed values
        short iDiff;
        iBand = _mm_extract_epi16(vbanda, 0);
        iDiff = (short)_mm_extract_epi16(vdiffa, 0);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbanda, 1);
        iDiff = (short)_mm_extract_epi16(vdiffa, 1);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbanda, 2);
        iDiff = (short)_mm_extract_epi16(vdiffa, 2);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbanda, 3);
        iDiff = (short)_mm_extract_epi16(vdiffa, 3);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbanda, 4);
        iDiff = (short)_mm_extract_epi16(vdiffa, 4);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbanda, 5);
        iDiff = (short)_mm_extract_epi16(vdiffa, 5);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbanda, 6);
        iDiff = (short)_mm_extract_epi16(vdiffa, 6);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbanda, 7);
        iDiff = (short)_mm_extract_epi16(vdiffa, 7);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbandb, 0);
        iDiff = (short)_mm_extract_epi16(vdiffb, 0);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbandb, 1);
        iDiff = (short)_mm_extract_epi16(vdiffb, 1);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbandb, 2);
        iDiff = (short)_mm_extract_epi16(vdiffb, 2);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbandb, 3);
        iDiff = (short)_mm_extract_epi16(vdiffb, 3);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbandb, 4);
        iDiff = (short)_mm_extract_epi16(vdiffb, 4);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbandb, 5);
        iDiff = (short)_mm_extract_epi16(vdiffb, 5);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbandb, 6);
        iDiff = (short)_mm_extract_epi16(vdiffb, 6);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        iBand = _mm_extract_epi16(vbandb, 7);
        iDiff = (short)_mm_extract_epi16(vdiffb, 7);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
      }
      //last colum
      {
        __m128i vsrca, vsrcb;
        __m128i vdiffa,vdiffb;
        if (sizeof(Pel) == 1){
          __m128i vsrc = _mm_load_si128((__m128i*)&srcLine[x]);
          vsrca = _mm_unpacklo_epi8(vsrc, vzero);
          vsrcb = _mm_unpackhi_epi8(vsrc, vzero);
          __m128i vorg  = _mm_load_si128((__m128i*)&orgLine[x]);
          __m128i vorga = _mm_unpacklo_epi8(vorg, vzero);
          __m128i vorgb = _mm_unpackhi_epi8(vorg, vzero);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        else
        {
          vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
          vsrcb = _mm_load_si128((__m128i*)&srcLine[x+8]);
          __m128i vorga = _mm_load_si128((__m128i*)&orgLine[x]);
          __m128i vorgb = _mm_load_si128((__m128i*)&orgLine[x+8]);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        __m128i vbanda = _mm_srai_epi16(vsrca, i_bo_range_shift);
        __m128i vbandb = _mm_srai_epi16(vsrcb, i_bo_range_shift);
        int iBand;
        // since gcc 4.6 synopsis of _mm_extract_epi16 has changed to (int)(unsigned short)_mm_extract_epi16()
        // therefore cast result to short to have signed values
        short iDiff;
        iBand = _mm_extract_epi16(vbanda, 0);
        iDiff = (short)_mm_extract_epi16(vdiffa, 0);
        diff[iBand]  += iDiff;
        count[iBand] += 1;
        if (iNaRight<15)
        {
          iBand = _mm_extract_epi16(vbanda, 1);
          iDiff = (short)_mm_extract_epi16(vdiffa, 1);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<14)
        {
          iBand = _mm_extract_epi16(vbanda, 2);
          iDiff = (short)_mm_extract_epi16(vdiffa, 2);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<13)
        {
          iBand = _mm_extract_epi16(vbanda, 3);
          iDiff = (short)_mm_extract_epi16(vdiffa, 3);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<12)
        {
          iBand = _mm_extract_epi16(vbanda, 4);
          iDiff = (short)_mm_extract_epi16(vdiffa, 4);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<11)
        {
          iBand = _mm_extract_epi16(vbanda, 5);
          iDiff = (short)_mm_extract_epi16(vdiffa, 5);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<10)
        {
          iBand = _mm_extract_epi16(vbanda, 6);
          iDiff = (short)_mm_extract_epi16(vdiffa, 6);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<9)
        {
          iBand = _mm_extract_epi16(vbanda, 7);
          iDiff = (short)_mm_extract_epi16(vdiffa, 7);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<8)
        {
          iBand = _mm_extract_epi16(vbandb, 0);
          iDiff = (short)_mm_extract_epi16(vdiffb, 0);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<7)
        {
          iBand = _mm_extract_epi16(vbandb, 1);
          iDiff = (short)_mm_extract_epi16(vdiffb, 1);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<6)
        {
          iBand = _mm_extract_epi16(vbandb, 2);
          iDiff = (short)_mm_extract_epi16(vdiffb, 2);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<5)
        {
          iBand = _mm_extract_epi16(vbandb, 3);
          iDiff = (short)_mm_extract_epi16(vdiffb, 3);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<=4)
        {
          iBand = _mm_extract_epi16(vbandb, 4);
          iDiff = (short)_mm_extract_epi16(vdiffb, 4);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<3)
        {
          iBand = _mm_extract_epi16(vbandb, 5);
          iDiff = (short)_mm_extract_epi16(vdiffb, 5);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<2)
        {
          iBand = _mm_extract_epi16(vbandb, 6);
          iDiff = (short)_mm_extract_epi16(vdiffb, 6);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
        if (iNaRight<1)
        {
          iBand = _mm_extract_epi16(vbandb, 7);
          iDiff = (short)_mm_extract_epi16(vdiffb, 7);
          diff[iBand]  += iDiff;
          count[iBand] += 1;
        }
      }
      srcLine += srcStride;
      orgLine += orgStride;
    }
  }
  else
  {
    int i,j;
    int iBoRangeShift = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
    for ( i = 0; i < endY; i++ )
    {
      for ( j = 0; j < endX; j++, srcLine++, orgLine++ )
      {
        int iBand            = *srcLine >> iBoRangeShift;
        diff[iBand]  += (*orgLine - *srcLine);
        count[iBand] += 1;
      }
      srcLine += srcStride - endX;
      orgLine += orgStride - endX;
    }
  }
}
template <X86_VEXT vext>
void calcSaoStatisticsEo0_SIMD(int width,int startX,int endX,int endY,Pel*  srcLine,Pel*  orgLine,int srcStride,int orgStride,int64_t  *count, int64_t *diff)
{
  int iNaRight=width-endX;

  int iNaWidth = startX + iNaRight;
  int i,j;
  if ( width % 16 == 0 )
  {
    __m128i vzero       = _mm_set1_epi8(0);
    __m128i vplusone    = _mm_set1_epi8(1);
    __m128i vbaseoffset = _mm_set1_epi8(2);
    // store intermediate results in 32bit partial sums for each EO type
    __m128i vdiffsum[NUM_SAO_EO_CLASSES];
    __m128i vcountsum[NUM_SAO_EO_CLASSES];
    __m128i vconst[NUM_SAO_EO_CLASSES];
    for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      vdiffsum[i]  = _mm_set1_epi32(0);
      vcountsum[i] = _mm_set1_epi32(0);
      vconst[i]    = _mm_set1_epi16(i);
    }
    // create masks for first and last pixel row
    __m128i vmaskgs = _mm_set1_epi16(0);
    __m128i vmaskge = _mm_set1_epi16(0);
    if ( startX )
    {
      switch (startX)
      {
      case 1:
        vmaskgs = _mm_insert_epi16( vmaskgs, 0xffff, 0);
        break;
      case 2:
        vmaskgs = _mm_set_epi16(0,0,0,0,0,0,0xffff,0xffff);
        break;
      case 3:
        vmaskgs = _mm_set_epi16(0,0,0,0,0,0xffff,0xffff,0xffff);
        break;
      case 4:
        vmaskgs = _mm_set_epi16(0,0,0,0,0xffff,0xffff,0xffff,0xffff);
        break;
      case 5:
        vmaskgs = _mm_set_epi16(0,0,0,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      case 6:
        vmaskgs = _mm_set_epi16(0,0,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      case 7:
        vmaskgs = _mm_set_epi16(0,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      }
    }
    if ( iNaRight )
    {
      vmaskge = _mm_insert_epi16( vmaskge, 0xffff, 7);
      switch (iNaRight)
      {
      case 1:
        vmaskge = _mm_insert_epi16( vmaskge, 0xffff, 7);
        break;
      case 2:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0,0,0,0,0,0);
        break;
      case 3:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0,0,0,0,0);
        break;
      case 4:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0,0,0,0);
        break;
      case 5:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0);
        break;
      case 6:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0);
        break;
      case 7:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0);
        break;
      }
    }
    for ( int y = 0; y < endY; y++)
    {
      __m128i vmaskga = vmaskgs;
      __m128i vmaskgb = vzero;
      for ( int  x= 0; x < width; x+=16 )
      {
        __m128i vsrcal,vsrcar;
        __m128i vsrcbl,vsrcbr;
        __m128i vdiffa,vdiffb;
        // set mask for last pixel
        if ( x >= width - 16 )
        {
          vmaskgb = vmaskge;
        }
        // load reconstruction and compute difference between original signal and reconstruction
        if (sizeof(Pel) ==1)
        {
          __m128i vsrc = _mm_load_si128((__m128i*)&srcLine[x]);
          __m128i vsrcl = _mm_loadu_si128((__m128i*)&srcLine[x-1]);
          __m128i vsrcr = _mm_loadu_si128((__m128i*)&srcLine[x+1]);
          __m128i vsrca = _mm_unpacklo_epi8(vsrc, vzero);
          __m128i vsrcb = _mm_unpackhi_epi8(vsrc, vzero);
          vsrcal = _mm_unpacklo_epi8(vsrcl, vzero);
          vsrcbl = _mm_unpackhi_epi8(vsrcl, vzero);
          vsrcar = _mm_unpacklo_epi8(vsrcr, vzero);
          vsrcbr = _mm_unpackhi_epi8(vsrcr, vzero);
          vsrcal = _mm_sub_epi16(vsrca, vsrcal);
          vsrcar = _mm_sub_epi16(vsrca, vsrcar);
          vsrcbl = _mm_sub_epi16(vsrcb, vsrcbl);
          vsrcbr = _mm_sub_epi16(vsrcb, vsrcbr);
          __m128i vorg  = _mm_loadu_si128((__m128i*)&orgLine[x]);
          __m128i vorga = _mm_unpacklo_epi8(vorg, vzero);
          __m128i vorgb = _mm_unpackhi_epi8(vorg, vzero);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        else
        {
          __m128i vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
          vsrcal = _mm_loadu_si128((__m128i*)&srcLine[x-1]);
          vsrcar = _mm_loadu_si128((__m128i*)&srcLine[x+1]);
          vsrcal = _mm_sub_epi16(vsrca, vsrcal);
          vsrcar = _mm_sub_epi16(vsrca, vsrcar);
          __m128i vsrcb = _mm_loadu_si128((__m128i*)&srcLine[x+8]);
          vsrcbl = _mm_loadu_si128((__m128i*)&srcLine[x+8-1]);
          vsrcbr = _mm_loadu_si128((__m128i*)&srcLine[x+8+1]);
          vsrcbl = _mm_sub_epi16(vsrcb, vsrcbl);
          vsrcbr = _mm_sub_epi16(vsrcb, vsrcbr);
          __m128i vorga = _mm_loadu_si128((__m128i*)&orgLine[x]);
          __m128i vorgb = _mm_loadu_si128((__m128i*)&orgLine[x+8]);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        // compute sign and type for 16 pixels
        __m128i vsignl = _mm_packs_epi16(vsrcal, vsrcbl);
        __m128i vsignr = _mm_packs_epi16(vsrcar, vsrcbr);
        vsignl = _mm_sign_epi8(vplusone, vsignl);
        vsignr = _mm_sign_epi8(vplusone, vsignr);
        __m128i vtype  = _mm_add_epi8(_mm_add_epi8(vsignl, vsignr), vbaseoffset);
        __m128i vtypea = _mm_unpacklo_epi8(vtype, vzero);
        __m128i vtypeb = _mm_unpackhi_epi8(vtype, vzero);
        vtypea = _mm_or_si128(vtypea, vmaskga);
        vtypeb = _mm_or_si128(vtypeb, vmaskgb);
        // count occurence of each type and accumulate partial sums for each type
        for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
        {
          __m128i vmaska = _mm_cmpeq_epi16(vtypea, vconst[i]);
          __m128i vmaskb = _mm_cmpeq_epi16(vtypeb, vconst[i]);
          __m128i vdiffma = _mm_and_si128(vmaska, vdiffa);
          __m128i vdiffmb = _mm_and_si128(vmaskb, vdiffb);
          vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_madd_epi16(vdiffma, vconst[1]));
          vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_madd_epi16(vdiffmb, vconst[1]));
          __m128i vcountma = _mm_srli_epi16(vmaska,15);
          __m128i vcountmb = _mm_srli_epi16(vmaskb,15);
          vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_madd_epi16(vcountma, vconst[1]));
          vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_madd_epi16(vcountmb, vconst[1]));
        }
        // clear mask for first pixel
        vmaskga = vzero;
      }
      // next pixel line
      srcLine += srcStride;
      orgLine += orgStride;
    }
    // horizontal add of four 32 bit partial sums
    for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_srli_si128(vdiffsum[i], 8));
      vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_srli_si128(vdiffsum[i], 4));
      diff[i] = _mm_cvtsi128_si32(vdiffsum[i]);
      vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_srli_si128(vcountsum[i], 8));
      vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_srli_si128(vcountsum[i], 4));
      count[i] = _mm_cvtsi128_si32(vcountsum[i]);
    }
  }
  else
  {
    srcLine      = srcLine + startX;
    orgLine      =orgLine + startX;
    diff +=2;
    count+=2;

    for ( i = 0; i < endY; i++ )
    {
      int iSignLeft = sgn( *srcLine - *(srcLine - 1) );
      for ( j = 0; j < width - iNaWidth; j++, srcLine++, orgLine++ )
      {
        int iSignRight       = sgn( *srcLine - *(srcLine + 1) );
        //printf("%d ",*srcLine);
        int iType            = iSignLeft + iSignRight;
        iSignLeft            = -1 * iSignRight;
        diff[iType]  += (*orgLine - *srcLine);
        count[iType] += 1;
      }
      srcLine += srcStride - ( width - iNaWidth );
      orgLine += orgStride - ( width - iNaWidth );
    }
  }
}
template <X86_VEXT vext>
void calcSaoStatisticsEo90_SIMD(int width,int endX,int startY,int endY,Pel*  srcLine,Pel*  orgLine,int srcStride,int orgStride,int64_t  *count, int64_t *diff,int8_t *signUpLine)
{
  if ( width % 16 == 0 )
  {
    int iNaRight=width-endX;
    __m128i vzero       = _mm_set1_epi8(0);
    __m128i vplusone    = _mm_set1_epi8(1);
    __m128i vbaseoffset = _mm_set1_epi8(2);
    // store intermediate results in 32bit partial sums for each EO type
    __m128i vdiffsum[NUM_SAO_EO_CLASSES];
    __m128i vcountsum[NUM_SAO_EO_CLASSES];
    __m128i vconst[NUM_SAO_EO_CLASSES];
    for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      vdiffsum[i]  = _mm_set1_epi32(0);
      vcountsum[i] = _mm_set1_epi32(0);
      vconst[i]    = _mm_set1_epi16(i);
    }
    // create masks for first and last pixel row
    __m128i vmaskge = _mm_set1_epi16(0);
    if ( iNaRight )
    {
      vmaskge = _mm_insert_epi16( vmaskge, 0xffff, 7);
      switch (iNaRight)
      {
      case 1:
        vmaskge = _mm_insert_epi16( vmaskge, 0xffff, 7);
        break;
      case 2:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0,0,0,0,0,0);
        break;
      case 3:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0,0,0,0,0);
        break;
      case 4:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0,0,0,0);
        break;
      case 5:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0);
        break;
      case 6:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0);
        break;
      case 7:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0);
        break;
      }
    }

    __m128i vsigns[MAX_CU_SIZE/16 +1];  //+1 to avoid MSVC error
    for (int x=0; x< endX; x+=16)
    {
      __m128i vsrca,vsrcb;
      __m128i vsrcat,vsrcbt;
      if (sizeof(Pel) == 1)
      {
        __m128i vsrc = _mm_load_si128((__m128i*)&srcLine[x]);
        __m128i vsrct = _mm_load_si128((__m128i*)&srcLine[x-srcStride]);
        vsrca = _mm_unpacklo_epi8(vsrc, vzero);
        vsrcb = _mm_unpackhi_epi8(vsrc, vzero);
        vsrcat = _mm_unpacklo_epi8(vsrct, vzero);
        vsrcbt = _mm_unpackhi_epi8(vsrct, vzero);
      }
      else
      {
        vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
        vsrcb = _mm_load_si128((__m128i*)&srcLine[x+8]);
        vsrcat = _mm_load_si128((__m128i*)&srcLine[x   - srcStride]);
        vsrcbt = _mm_load_si128((__m128i*)&srcLine[x+8 - srcStride]);
      }
      vsrcat = _mm_sub_epi16(vsrcat, vsrca);
      vsrcbt = _mm_sub_epi16(vsrcbt, vsrcb);
      vsigns[x/16] = _mm_sign_epi8(vplusone, _mm_packs_epi16(vsrcat, vsrcbt));
    }
    /* filter all lines */
    for (int j = startY; j < endY ; j++)
    {
      __m128i vmaskgb = vzero;

      /* start with first pixel */
      /* filter all pixels of this line */
      for (int x = 0; x < endX; x+=16)
      {
        __m128i vsrca,vsrcb;
        __m128i vsrcad, vsrcbd;
        __m128i vdiffa,vdiffb;
        // set mask for last pixel
        if ( x >= width - 16 )
        {
          vmaskgb = vmaskge;
        }
        // load reconstruction and compute difference between original signal and reconstruction
        if (sizeof(Pel) == 1)
        {
          __m128i vsrc = _mm_load_si128((__m128i*)&srcLine[x]);
          __m128i vsrcd = _mm_load_si128((__m128i*)&srcLine[x+srcStride]);
          vsrca = _mm_unpacklo_epi8(vsrc, vzero);
          vsrcb = _mm_unpackhi_epi8(vsrc, vzero);
          vsrcad = _mm_unpacklo_epi8(vsrcd, vzero);
          vsrcbd = _mm_unpackhi_epi8(vsrcd, vzero);

          __m128i vorg  = _mm_load_si128((__m128i*)&orgLine[x]);
          __m128i vorga = _mm_unpacklo_epi8(vorg, vzero);
          __m128i vorgb = _mm_unpackhi_epi8(vorg, vzero);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        else
        {
          vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
          vsrcb = _mm_load_si128((__m128i*)&srcLine[x+8]);
          vsrcad = _mm_load_si128((__m128i*)&srcLine[x   + srcStride]);
          vsrcbd = _mm_load_si128((__m128i*)&srcLine[x+8 + srcStride]);
          __m128i vorga = _mm_load_si128((__m128i*)&orgLine[x]);
          __m128i vorgb = _mm_load_si128((__m128i*)&orgLine[x+8]);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        // compute sign and type for 16 pixels
        vsrcad = _mm_sub_epi16(vsrca, vsrcad);
        vsrcbd = _mm_sub_epi16(vsrcb, vsrcbd);
        __m128i vsignd = _mm_sign_epi8(vplusone, _mm_packs_epi16(vsrcad, vsrcbd));
        __m128i vsignt = vsigns[x/16];
        vsigns[x/16] = vsignd;
        __m128i vtype  = _mm_add_epi8(_mm_sub_epi8(vsignd, vsignt), vbaseoffset);
        __m128i vtypea = _mm_unpacklo_epi8(vtype, vzero);
        __m128i vtypeb = _mm_unpackhi_epi8(vtype, vzero);
        vtypeb = _mm_or_si128(vtypeb, vmaskgb);

        // count occurence of each type and accumulate partial sums for each type
        for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
        {
          __m128i vmaska = _mm_cmpeq_epi16(vtypea, vconst[i]);
          __m128i vmaskb = _mm_cmpeq_epi16(vtypeb, vconst[i]);
          __m128i vdiffma = _mm_and_si128(vmaska, vdiffa);
          __m128i vdiffmb = _mm_and_si128(vmaskb, vdiffb);
          vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_madd_epi16(vdiffma, vconst[1]));
          vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_madd_epi16(vdiffmb, vconst[1]));
          __m128i vcountma = _mm_srli_epi16(vmaska,15);
          __m128i vcountmb = _mm_srli_epi16(vmaskb,15);
          vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_madd_epi16(vcountma, vconst[1]));
          vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_madd_epi16(vcountmb, vconst[1]));
        }
      }
      // next pixel line
      srcLine += srcStride;
      orgLine += orgStride;
    }
    // horizontal add of four 32 bit partial sums
    for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_srli_si128(vdiffsum[i], 8));
      vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_srli_si128(vdiffsum[i], 4));
      diff[i] = _mm_cvtsi128_si32(vdiffsum[i]);
      vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_srli_si128(vcountsum[i], 8));
      vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_srli_si128(vcountsum[i], 4));
      count[i] = _mm_cvtsi128_si32(vcountsum[i]);
    }
  }
  else
  {
    diff +=2;
    count+=2;
    int x,y,edgeType;
    Pel* srcLineAbove = srcLine - srcStride;
    int8_t signDown;
    for (x=0; x<endX; x++)
    {
      signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
    }
    Pel* srcLineBelow;
    for (y=startY; y<endY; y++)
    {
      srcLineBelow = srcLine + srcStride;

      for (x=0; x<endX; x++)
      {
        signDown  = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
        edgeType  = signDown + signUpLine[x];
        signUpLine[x]= -signDown;

        diff [edgeType] += (orgLine[x] - srcLine[x]);
        count[edgeType] ++;
      }
      srcLine += srcStride;
      orgLine += orgStride;
    }
  }
}
template <X86_VEXT vext>
void calcSaoStatisticsEo135_SIMD(int width,int startX,int endX,int endY,Pel*  srcLine,Pel*  orgLine,int srcStride,int orgStride,int64_t  *count, int64_t *diff,int8_t *signUpLine,int8_t *signDownLine)
{
  if ( width % 16 == 0 )
  {
    int iNaRight=width-endX;
    int iNaWidth = startX + iNaRight;
    diff -=2;
    count-=2;
    __m128i vzero       = _mm_set1_epi8(0);
    __m128i vplusone    = _mm_set1_epi8(1);
    __m128i vbaseoffset = _mm_set1_epi8(2);
    // store intermediate results in 32bit partial sums for each EO type
    __m128i vdiffsum[NUM_SAO_EO_CLASSES];
    __m128i vcountsum[NUM_SAO_EO_CLASSES];
    __m128i vconst[NUM_SAO_EO_CLASSES];
    for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      vdiffsum[i]  = _mm_set1_epi32(0);
      vcountsum[i] = _mm_set1_epi32(0);
      vconst[i]    = _mm_set1_epi16(i);
    }
    // create masks for first and last pixel row
    __m128i vmaskgs = _mm_set1_epi16(0);
    __m128i vmaskge = _mm_set1_epi16(0);
    if ( startX )
    {
      switch (startX)
      {
      case 1:
        vmaskgs = _mm_insert_epi16( vmaskgs, 0xffff, 0);
        break;
      case 2:
        vmaskgs = _mm_set_epi16(0,0,0,0,0,0,0xffff,0xffff);
        break;
      case 3:
        vmaskgs = _mm_set_epi16(0,0,0,0,0,0xffff,0xffff,0xffff);
        break;
      case 4:
        vmaskgs = _mm_set_epi16(0,0,0,0,0xffff,0xffff,0xffff,0xffff);
        break;
      case 5:
        vmaskgs = _mm_set_epi16(0,0,0,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      case 6:
        vmaskgs = _mm_set_epi16(0,0,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      case 7:
        vmaskgs = _mm_set_epi16(0,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      }
    }
    if ( iNaRight )
    {
      vmaskge = _mm_insert_epi16( vmaskge, 0xffff, 7);
      switch (iNaRight)
      {
      case 1:
        vmaskge = _mm_insert_epi16( vmaskge, 0xffff, 7);
        break;
      case 2:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0,0,0,0,0,0);
        break;
      case 3:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0,0,0,0,0);
        break;
      case 4:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0,0,0,0);
        break;
      case 5:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0);
        break;
      case 6:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0);
        break;
      case 7:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0);
        break;
      }
    }
    /* filter all lines */
    for (int j = 1; j < endY; j++)
    {
      __m128i vmaskga = vmaskgs;
      __m128i vmaskgb = vconst[0];
      /* start with first pixel */
      /* filter all pixels of this line */
      for (int x = 0; x < width; x+=16)
      {
        __m128i vsrca,vsrcb;
        __m128i vsrcad,vsrcbd;
        __m128i vsrcat,vsrcbt;
        __m128i vdiffa,vdiffb;
        // set mask for last pixel
        if ( x >= width - 16 )
        {
          vmaskgb = vmaskge;
        }
        if (sizeof(Pel) == 1)
        {
          __m128i vsrct = _mm_loadu_si128((__m128i*)&srcLine[ x-srcStride-1 ]);
          __m128i vsrc  = _mm_load_si128((__m128i*)&srcLine[ x ]);
          __m128i vsrcd = _mm_loadu_si128((__m128i*)&srcLine[ x+srcStride+1 ]);
          vsrcat = _mm_unpacklo_epi8(vsrct, vzero);
          vsrcbt = _mm_unpackhi_epi8(vsrct, vzero);
          vsrca = _mm_unpacklo_epi8(vsrc, vzero);
          vsrcb = _mm_unpackhi_epi8(vsrc, vzero);
          vsrcad = _mm_unpacklo_epi8(vsrcd, vzero);
          vsrcbd = _mm_unpackhi_epi8(vsrcd, vzero);
          __m128i vorg  = _mm_load_si128((__m128i*)&orgLine[x]);
          __m128i vorga = _mm_unpacklo_epi8(vorg, vzero);
          __m128i vorgb = _mm_unpackhi_epi8(vorg, vzero);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        else
        {
          vsrcat = _mm_loadu_si128((__m128i*)&srcLine[x - 1 - srcStride ]);
          vsrcbt = _mm_loadu_si128((__m128i*)&srcLine[x - 1 + 8 - srcStride]);
          vsrca = _mm_load_si128((__m128i*)&srcLine[x]);
          vsrcb = _mm_load_si128((__m128i*)&srcLine[x+8]);
          vsrcad = _mm_loadu_si128((__m128i*)&srcLine[x + 1 + srcStride ]);
          vsrcbd = _mm_loadu_si128((__m128i*)&srcLine[x + 1 + 8 + srcStride]);
          __m128i vorga = _mm_load_si128((__m128i*)&orgLine[x]);
          __m128i vorgb = _mm_load_si128((__m128i*)&orgLine[x+8]);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        // compute sign and type for 16 pixels
        vsrcat = _mm_sub_epi16(vsrca, vsrcat);
        vsrcbt = _mm_sub_epi16(vsrcb, vsrcbt);
        vsrcad = _mm_sub_epi16(vsrca, vsrcad);
        vsrcbd = _mm_sub_epi16(vsrcb, vsrcbd);
        __m128i vsignt = _mm_sign_epi8(vplusone, _mm_packs_epi16(vsrcat, vsrcbt));
        __m128i vsignd = _mm_sign_epi8(vplusone, _mm_packs_epi16(vsrcad, vsrcbd));
        __m128i vtype = _mm_add_epi8(_mm_add_epi8(vsignd, vsignt), vbaseoffset);
        __m128i vtypea = _mm_unpacklo_epi8(vtype, vzero);
        __m128i vtypeb = _mm_unpackhi_epi8(vtype, vzero);
        vtypea = _mm_or_si128(vtypea, vmaskga);
        vtypeb = _mm_or_si128(vtypeb, vmaskgb);
        // count occurence of each type and accumulate partial sums for each type
        for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
        {
          __m128i vmaska = _mm_cmpeq_epi16(vtypea, vconst[i]);
          __m128i vmaskb = _mm_cmpeq_epi16(vtypeb, vconst[i]);
          __m128i vdiffma = _mm_and_si128(vmaska, vdiffa);
          __m128i vdiffmb = _mm_and_si128(vmaskb, vdiffb);
          vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_madd_epi16(vdiffma, vconst[1]));
          vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_madd_epi16(vdiffmb, vconst[1]));
          __m128i vcountma = _mm_srli_epi16(vmaska,15);
          __m128i vcountmb = _mm_srli_epi16(vmaskb,15);
          vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_madd_epi16(vcountma, vconst[1]));
          vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_madd_epi16(vcountmb, vconst[1]));
        }
        // clear mask for first pixel
        vmaskga = vconst[0];
      }
      // next pixel line
      srcLine += srcStride;
      orgLine += orgStride;
    }
    // horizontal add of four 32 bit partial sums
    for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_srli_si128(vdiffsum[i], 8));
      vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_srli_si128(vdiffsum[i], 4));
      diff[i] += _mm_cvtsi128_si32(vdiffsum[i]);
      vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_srli_si128(vcountsum[i], 8));
      vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_srli_si128(vcountsum[i], 4));
      count[i] += _mm_cvtsi128_si32(vcountsum[i]);
    }
  }
  else
  {
    int x,y,edgeType;
    int8_t signDown;
    int8_t *signTmpLine;
    //middle lines
     for (y=1; y<endY; y++)
     {
       int8_t* pTopSign = NULL;
       Pel* srcLineBelow = srcLine + srcStride;
       int8_t iTmpSign =  (int8_t)sgn( srcLineBelow[startX]   - srcLine[startX-1] );
       for ( x=startX,pTopSign = &signUpLine[startX]; x<endX; x++ , pTopSign++ )
       {
         signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x+1]);
         edgeType = signDown + *pTopSign;
         *pTopSign            = iTmpSign;
         iTmpSign             = -signDown;
         diff [edgeType] += (orgLine[x] - srcLine[x]);
         count[edgeType] ++;
       }
       srcLine += srcStride;
       orgLine += orgStride;
     }
  }
}
template <X86_VEXT vext>
void calcSaoStatisticsEo45_SIMD(int width,int startX,int endX,int endY,Pel*  srcLine,Pel*  orgLine,int srcStride,int orgStride,int64_t  *count, int64_t *diff,int8_t *signUpLine)
{
  Pel* pRec = srcLine;
  Pel* pOrg = orgLine;
  Pel* srcLineBelow = srcLine + srcStride;
  if (width % 16 == 0 )
  {
    int iNaRight=width-endX;
    int iNaWidth = startX + iNaRight;
    diff -=2;
    count-=2;
    __m128i vzero       = _mm_set1_epi8(0);
    __m128i vplusone    = _mm_set1_epi8(1);
    __m128i vbaseoffset = _mm_set1_epi8(2);
    // store intermediate results in 32bit partial sums for each EO type
    __m128i vdiffsum[NUM_SAO_EO_CLASSES];
    __m128i vcountsum[NUM_SAO_EO_CLASSES];
    __m128i vconst[NUM_SAO_EO_CLASSES];
    for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      vdiffsum[i]  = _mm_set1_epi32(0);
      vcountsum[i] = _mm_set1_epi32(0);
      vconst[i]    = _mm_set1_epi16(i);
    }
    // create masks for first and last pixel row
    __m128i vmaskgs = _mm_set1_epi16(0);
    __m128i vmaskge = _mm_set1_epi16(0);
    if ( startX )
    {
      switch (startX)
      {
      case 1:
        vmaskgs = _mm_insert_epi16( vmaskgs, 0xffff, 0);
        break;
      case 2:
        vmaskgs = _mm_set_epi16(0,0,0,0,0,0,0xffff,0xffff);
        break;
      case 3:
        vmaskgs = _mm_set_epi16(0,0,0,0,0,0xffff,0xffff,0xffff);
        break;
      case 4:
        vmaskgs = _mm_set_epi16(0,0,0,0,0xffff,0xffff,0xffff,0xffff);
        break;
      case 5:
        vmaskgs = _mm_set_epi16(0,0,0,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      case 6:
        vmaskgs = _mm_set_epi16(0,0,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      case 7:
        vmaskgs = _mm_set_epi16(0,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff);
        break;
      }
    }
    if ( iNaRight )
    {
      vmaskge = _mm_insert_epi16( vmaskge, 0xffff, 7);
      switch (iNaRight)
      {
      case 1:
        vmaskge = _mm_insert_epi16( vmaskge, 0xffff, 7);
        break;
      case 2:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0,0,0,0,0,0);
        break;
      case 3:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0,0,0,0,0);
        break;
      case 4:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0,0,0,0);
        break;
      case 5:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0);
        break;
      case 6:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0);
        break;
      case 7:
        vmaskge = _mm_set_epi16(0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0);
        break;
      }
    }
    /* filter all lines */
    for (int j = 1; j < endY; j++)
    {
      __m128i vmaskga = vmaskgs;
      __m128i vmaskgb = vconst[0];
      /* start with first pixel */
      /* filter all pixels of this line */
      for (int x = 0; x < width; x+=16)
      {
        __m128i vsrca,vsrcb;
        __m128i vsrcad,vsrcbd;
        __m128i vsrcat,vsrcbt;
        __m128i vdiffa,vdiffb;

        // set mask for last pixel
        if ( x >= width - 16 )
        {
          vmaskgb = vmaskge;
        }

        if (sizeof(Pel) == 1)
        {
          __m128i vsrct = _mm_loadu_si128((__m128i*)&pRec[ x-srcStride+1 ]);
          __m128i vsrc  = _mm_load_si128((__m128i*)&pRec[ x ]);
          __m128i vsrcd = _mm_loadu_si128((__m128i*)&pRec[ x+srcStride-1 ]);
          vsrcat = _mm_unpacklo_epi8(vsrct, vzero);
          vsrcbt = _mm_unpackhi_epi8(vsrct, vzero);
          vsrca = _mm_unpacklo_epi8(vsrc, vzero);
          vsrcb = _mm_unpackhi_epi8(vsrc, vzero);
          vsrcad = _mm_unpacklo_epi8(vsrcd, vzero);
          vsrcbd = _mm_unpackhi_epi8(vsrcd, vzero);
          __m128i vorg  = _mm_load_si128((__m128i*)&pOrg[x]);
          __m128i vorga = _mm_unpacklo_epi8(vorg, vzero);
          __m128i vorgb = _mm_unpackhi_epi8(vorg, vzero);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        else
        {
          vsrcat = _mm_loadu_si128((__m128i*)&pRec[x + 1 - srcStride ]);
          vsrcbt = _mm_loadu_si128((__m128i*)&pRec[x + 1 + 8 - srcStride]);
          vsrca = _mm_load_si128((__m128i*)&pRec[x]);
          vsrcb = _mm_load_si128((__m128i*)&pRec[x+8]);
          vsrcad = _mm_loadu_si128((__m128i*)&pRec[x - 1 + srcStride ]);
          vsrcbd = _mm_loadu_si128((__m128i*)&pRec[x - 1 + 8 + srcStride]);
          __m128i vorga = _mm_load_si128((__m128i*)&pOrg[x]);
          __m128i vorgb = _mm_load_si128((__m128i*)&pOrg[x+8]);
          vdiffa = _mm_sub_epi16(vorga, vsrca);
          vdiffb = _mm_sub_epi16(vorgb, vsrcb);
        }
        // compute sign and type for 16 pixels
        vsrcat = _mm_sub_epi16(vsrca, vsrcat);
        vsrcbt = _mm_sub_epi16(vsrcb, vsrcbt);
        vsrcad = _mm_sub_epi16(vsrca, vsrcad);
        vsrcbd = _mm_sub_epi16(vsrcb, vsrcbd);
        __m128i vsignt = _mm_sign_epi8(vplusone, _mm_packs_epi16(vsrcat, vsrcbt));
        __m128i vsignd = _mm_sign_epi8(vplusone, _mm_packs_epi16(vsrcad, vsrcbd));
        __m128i vtype = _mm_add_epi8(_mm_add_epi8(vsignd, vsignt), vbaseoffset);
        __m128i vtypea = _mm_unpacklo_epi8(vtype, vzero);
        __m128i vtypeb = _mm_unpackhi_epi8(vtype, vzero);
        vtypea = _mm_or_si128(vtypea, vmaskga);
        vtypeb = _mm_or_si128(vtypeb, vmaskgb);
        // count occurence of each type and accumulate partial sums for each type
        for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
        {
          __m128i vmaska = _mm_cmpeq_epi16(vtypea, vconst[i]);
          __m128i vmaskb = _mm_cmpeq_epi16(vtypeb, vconst[i]);
          __m128i vdiffma = _mm_and_si128(vmaska, vdiffa);
          __m128i vdiffmb = _mm_and_si128(vmaskb, vdiffb);
          vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_madd_epi16(vdiffma, vconst[1]));
          vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_madd_epi16(vdiffmb, vconst[1]));
          __m128i vcountma = _mm_srli_epi16(vmaska,15);
          __m128i vcountmb = _mm_srli_epi16(vmaskb,15);
          vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_madd_epi16(vcountma, vconst[1]));
          vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_madd_epi16(vcountmb, vconst[1]));
        }
        // clear mask for first pixel
        vmaskga = vconst[0];
      }
      // next pixel line
      pRec += srcStride;
      pOrg += orgStride;
    }

    // horizontal add of four 32 bit partial sums
    for ( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_srli_si128(vdiffsum[i], 8));
      vdiffsum[i] = _mm_add_epi32(vdiffsum[i], _mm_srli_si128(vdiffsum[i], 4));
      diff[i] += _mm_cvtsi128_si32(vdiffsum[i]);
      vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_srli_si128(vcountsum[i], 8));
      vcountsum[i] = _mm_add_epi32(vcountsum[i], _mm_srli_si128(vcountsum[i], 4));
      count[i] += _mm_cvtsi128_si32(vcountsum[i]);
    }
  }
  else
  {
    int x,y,edgeType;
    int8_t signDown;
    //middle lines
    for (y=1; y<endY; y++)
    {
      srcLineBelow = srcLine + srcStride;
      for(x=startX; x<endX; x++)
      {
        signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x-1]);
        edgeType = signDown + signUpLine[x];
        diff [edgeType] += (orgLine[x] - srcLine[x]);
        count[edgeType] ++;
        signUpLine[x-1] = -signDown;
      }
      signUpLine[endX-1] = (int8_t)sgn(srcLineBelow[endX-1] - srcLine[endX]);
      srcLine  += srcStride;
      orgLine  += orgStride;
    }
  }
}
template <X86_VEXT vext>
void SampleAdaptiveOffset::_initSampleAdaptiveOffsetX86()
{
  offsetBlock= offsetBlock_SIMD<vext>;
  calcSaoStatisticsEo0 =  calcSaoStatisticsEo0_SIMD<vext>;
  calcSaoStatisticsEo90 =  calcSaoStatisticsEo90_SIMD<vext>;
  calcSaoStatisticsEo135 =  calcSaoStatisticsEo135_SIMD<vext>;
  calcSaoStatisticsEo45 =  calcSaoStatisticsEo45_SIMD<vext>;
  calcSaoStatisticsBo =  calcSaoStatisticsBo_SIMD<vext>;

}

template void SampleAdaptiveOffset::_initSampleAdaptiveOffsetX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


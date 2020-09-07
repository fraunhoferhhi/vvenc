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
void SampleAdaptiveOffset::_initSampleAdaptiveOffsetX86()
{
  offsetBlock= offsetBlock_SIMD<vext>;
}

template void SampleAdaptiveOffset::_initSampleAdaptiveOffsetX86<SIMDX86>();

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


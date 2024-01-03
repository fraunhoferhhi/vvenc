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
#include "CommonDefX86.h"

#include "../AdaptiveLoopFilter.h"

#ifdef TARGET_SIMD_X86
#if ENABLE_SIMD_OPT_ALF

namespace vvenc
{

using namespace x86_simd;

template<X86_VEXT vext>
void simdDeriveClassificationBlk(AlfClassifier *classifier, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const int shift, const int vbCTUHeight, int vbPos)
{
  CHECK((blk.height & 7) != 0, "Block height must be a multiple of 8");
  CHECK((blk.width & 7) != 0, "Block width must be a multiple of 8");
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  const size_t imgStride = srcLuma.stride;
  const Pel*   srcExt    = srcLuma.buf;

  const int imgHExtended = blk.height + 4;
  const int imgWExtended = blk.width + 4;

  const int posX = blk.pos().x;
  const int posY = blk.pos().y;

  // 18x40 array
  uint16_t colSums[(AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4) >> 1]
                  [AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 8];

  for (int i = 0; i < imgHExtended; i += 2)
  {
    const size_t offset = (i + posY - 3) * imgStride + posX - 3;

    const Pel* imgY0 = &srcExt[offset];
    const Pel* imgY1 = &srcExt[offset + imgStride];
    const Pel* imgY2 = &srcExt[offset + imgStride * 2];
    const Pel* imgY3 = &srcExt[offset + imgStride * 3];

    // pixel padding for gradient calculation
    int pos      = blkDst.pos().y - 2 + i;
    int posInCTU = pos & (vbCTUHeight - 1);
    if (pos > 0 && posInCTU == vbPos - 2)
    {
      imgY3 = imgY2;
    }
    else if (pos > 0 && posInCTU == vbPos)
    {
      imgY0 = imgY1;
    }

    __m128i prev = _mm_setzero_si128();

    for (int j = 0; j < imgWExtended; j += 8)
    {
      const __m128i x0 = _mm_loadu_si128((const __m128i *) (imgY0 + j));
      const __m128i x1 = _mm_loadu_si128((const __m128i *) (imgY1 + j));
      const __m128i x2 = _mm_loadu_si128((const __m128i *) (imgY2 + j));
      const __m128i x3 = _mm_loadu_si128((const __m128i *) (imgY3 + j));

      const __m128i x4 = _mm_loadu_si128((const __m128i *) (imgY0 + j + 2));
      const __m128i x5 = _mm_loadu_si128((const __m128i *) (imgY1 + j + 2));
      const __m128i x6 = _mm_loadu_si128((const __m128i *) (imgY2 + j + 2));
      const __m128i x7 = _mm_loadu_si128((const __m128i *) (imgY3 + j + 2));

      const __m128i nw = _mm_blend_epi16(x0, x1, 0xaa);
      const __m128i n  = _mm_blend_epi16(x0, x5, 0x55);
      const __m128i ne = _mm_blend_epi16(x4, x5, 0xaa);
      const __m128i w  = _mm_blend_epi16(x1, x2, 0xaa);
      const __m128i e  = _mm_blend_epi16(x5, x6, 0xaa);
      const __m128i sw = _mm_blend_epi16(x2, x3, 0xaa);
      const __m128i s  = _mm_blend_epi16(x2, x7, 0x55);
      const __m128i se = _mm_blend_epi16(x6, x7, 0xaa);

      __m128i c = _mm_blend_epi16(x1, x6, 0x55);
      c         = _mm_add_epi16(c, c);
      __m128i d = _mm_shuffle_epi8(c, _mm_setr_epi8(2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13));

      const __m128i ver = _mm_abs_epi16(_mm_sub_epi16(c, _mm_add_epi16(n, s)));
      const __m128i hor = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(w, e)));
      const __m128i di0 = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(nw, se)));
      const __m128i di1 = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(ne, sw)));
      const __m128i hv  = _mm_hadd_epi16(ver, hor);
      const __m128i di  = _mm_hadd_epi16(di0, di1);
      const __m128i all = _mm_hadd_epi16(hv, di);

      const __m128i t = _mm_blend_epi16(all, prev, 0xaa);
      _mm_storeu_si128((__m128i *) &colSums[i >> 1][j], _mm_hadd_epi16(t, all));
      prev = all;
    }
  }

  for (int i = 0; i < (blk.height >> 1); i += 4)
  {
    const int yOffset1 = ( i << 1 );
    const int yOffset2 = yOffset1 + 4;
    AlfClassifier* clPtr1 = &classifier[( yOffset1 / 4 ) * ( MAX_CU_SIZE / 4 )];
    AlfClassifier* clPtr2 = &classifier[( yOffset2 / 4 ) * ( MAX_CU_SIZE / 4 )];
    for (int j = 0; j < blk.width; j += 8)
    {
      __m128i x0, x1, x2, x3, x4, x5, x6, x7;

      const uint32_t z  = (2 * i + blkDst.pos().y) & (vbCTUHeight - 1);
      const uint32_t z2 = (2 * i + 4 + blkDst.pos().y) & (vbCTUHeight - 1);

      x0 = ( z  == vbPos     ) ? _mm_setzero_si128() : _mm_loadu_si128( ( __m128i * ) &colSums[i + 0][j + 4] );
      x1 = _mm_loadu_si128((__m128i *) &colSums[i + 1][j + 4]);
      x2 = _mm_loadu_si128((__m128i *) &colSums[i + 2][j + 4]);
      x3 = ( z  == vbPos - 4 ) ? _mm_setzero_si128() : _mm_loadu_si128( ( __m128i * ) &colSums[i + 3][j + 4] );

      x4 = ( z2 == vbPos     ) ? _mm_setzero_si128() : _mm_loadu_si128( ( __m128i * ) &colSums[i + 2][j + 4] );
      x5 = _mm_loadu_si128((__m128i *) &colSums[i + 3][j + 4]);
      x6 = _mm_loadu_si128((__m128i *) &colSums[i + 4][j + 4]);
      x7 = ( z2 == vbPos - 4 ) ? _mm_setzero_si128() : _mm_loadu_si128( ( __m128i * ) &colSums[i + 5][j + 4] );

      __m128i x0l = _mm_cvtepu16_epi32(x0);
      __m128i x0h = _mm_unpackhi_epi16(x0, _mm_setzero_si128());
      __m128i x1l = _mm_cvtepu16_epi32(x1);
      __m128i x1h = _mm_unpackhi_epi16(x1, _mm_setzero_si128());
      __m128i x2l = _mm_cvtepu16_epi32(x2);
      __m128i x2h = _mm_unpackhi_epi16(x2, _mm_setzero_si128());
      __m128i x3l = _mm_cvtepu16_epi32(x3);
      __m128i x3h = _mm_unpackhi_epi16(x3, _mm_setzero_si128());
      __m128i x4l = _mm_cvtepu16_epi32(x4);
      __m128i x4h = _mm_unpackhi_epi16(x4, _mm_setzero_si128());
      __m128i x5l = _mm_cvtepu16_epi32(x5);
      __m128i x5h = _mm_unpackhi_epi16(x5, _mm_setzero_si128());
      __m128i x6l = _mm_cvtepu16_epi32(x6);
      __m128i x6h = _mm_unpackhi_epi16(x6, _mm_setzero_si128());
      __m128i x7l = _mm_cvtepu16_epi32(x7);
      __m128i x7h = _mm_unpackhi_epi16(x7, _mm_setzero_si128());

      x0l = _mm_add_epi32(x0l, x1l);
      x2l = _mm_add_epi32(x2l, x3l);
      x4l = _mm_add_epi32(x4l, x5l);
      x6l = _mm_add_epi32(x6l, x7l);
      x0h = _mm_add_epi32(x0h, x1h);
      x2h = _mm_add_epi32(x2h, x3h);
      x4h = _mm_add_epi32(x4h, x5h);
      x6h = _mm_add_epi32(x6h, x7h);

      x0l = _mm_add_epi32(x0l, x2l);
      x4l = _mm_add_epi32(x4l, x6l);
      x0h = _mm_add_epi32(x0h, x2h);
      x4h = _mm_add_epi32(x4h, x6h);

      x2l = _mm_unpacklo_epi32(x0l, x4l);
      x2h = _mm_unpackhi_epi32(x0l, x4l);
      x6l = _mm_unpacklo_epi32(x0h, x4h);
      x6h = _mm_unpackhi_epi32(x0h, x4h);

      __m128i sumV  = _mm_unpacklo_epi32(x2l, x6l);
      __m128i sumH  = _mm_unpackhi_epi32(x2l, x6l);
      __m128i sumD0 = _mm_unpacklo_epi32(x2h, x6h);
      __m128i sumD1 = _mm_unpackhi_epi32(x2h, x6h);

      //      uint32_t tempAct = sumV + sumH;
      __m128i tempAct = _mm_add_epi32(sumV, sumH);

      //      const uint32_t activity = std::min<uint32_t>(15, tempAct * scale >> shift);
      //      static const uint8_t th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
      //      uint8_t classIdx = th[activity];
      const uint32_t scale  = ( z  == vbPos - 4 || z  == vbPos ) ? 96 : 64;
      const uint32_t scale2 = ( z2 == vbPos - 4 || z2 == vbPos ) ? 96 : 64;
      __m128i activity = _mm_mullo_epi32(tempAct, _mm_unpacklo_epi64(_mm_set1_epi32(scale), _mm_set1_epi32(scale2)));
      activity         = _mm_srl_epi32(activity, _mm_cvtsi32_si128(shift));
      activity         = _mm_min_epi32(activity, _mm_set1_epi32(15));
      __m128i classIdx = _mm_shuffle_epi8(_mm_setr_epi8(0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4), activity);

      //      if (sumV > sumH)
      //      {
      //        hv1       = sumV;
      //        hv0       = sumH;
      //        dirTempHV = 0;
      //      }
      //      else
      //      {
      //        hv1       = sumH;
      //        hv0       = sumV;
      //        dirTempHV = 1;
      //      }
      __m128i dirTempHVMinus1 = _mm_cmpgt_epi32(sumV, sumH);
      __m128i hv1             = _mm_max_epi32(sumV, sumH);
      __m128i hv0             = _mm_min_epi32(sumV, sumH);

      //      if (sumD0 > sumD1)
      //      {
      //        d1       = sumD0;
      //        d0       = sumD1;
      //        dirTempD = 0;
      //      }
      //      else
      //      {
      //        d1       = sumD1;
      //        d0       = sumD0;
      //        dirTempD = 1;
      //      }
      __m128i dirTempDMinus1 = _mm_cmpgt_epi32(sumD0, sumD1);
      __m128i d1             = _mm_max_epi32(sumD0, sumD1);
      __m128i d0             = _mm_min_epi32(sumD0, sumD1);

      //      int dirIdx;
      //      if (d1 * hv0 > hv1 * d0)
      //      {
      //        hvd1   = d1;
      //        hvd0   = d0;
      //        dirIdx = 0;
      //      }
      //      else
      //      {
      //        hvd1   = hv1;
      //        hvd0   = hv0;
      //        dirIdx = 2;
      //      }
      __m128i a      = _mm_xor_si128(_mm_mullo_epi32(d1, hv0), _mm_set1_epi32(0x80000000));
      __m128i b      = _mm_xor_si128(_mm_mullo_epi32(hv1, d0), _mm_set1_epi32(0x80000000));
      __m128i dirIdx = _mm_cmpgt_epi32(a, b);
      __m128i hvd1   = _mm_blendv_epi8(hv1, d1, dirIdx);
      __m128i hvd0   = _mm_blendv_epi8(hv0, d0, dirIdx);

      //      if (hvd1 * 2 > 9 * hvd0)
      //      {
      //        classIdx += (dirIdx + 2) * 5;
      //      }
      //      else if (hvd1 > 2 * hvd0)
      //      {
      //        classIdx += (dirIdx + 1) * 5;
      //      }
      __m128i strength1 = _mm_cmpgt_epi32(hvd1, _mm_add_epi32(hvd0, hvd0));
      __m128i strength2 = _mm_cmpgt_epi32(_mm_add_epi32(hvd1, hvd1), _mm_add_epi32(hvd0, _mm_slli_epi32(hvd0, 3)));
      __m128i offset    = _mm_and_si128(strength1, _mm_set1_epi32(5));
      classIdx          = _mm_add_epi32(classIdx, offset);
      classIdx          = _mm_add_epi32(classIdx, _mm_and_si128(strength2, _mm_set1_epi32(5)));
      offset            = _mm_andnot_si128(dirIdx, offset);
      offset            = _mm_add_epi32(offset, offset);
      classIdx          = _mm_add_epi32(classIdx, offset);

      //      uint8_t transposeIdx = 2 * dirTempD + dirTempHV;
      __m128i transposeIdx = _mm_set1_epi32(3);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempHVMinus1);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempDMinus1);


      static_assert(sizeof(AlfClassifier) == 2, "ALFClassifier type must be 16 bits wide");
      __m128i v;
      v = _mm_unpacklo_epi8(classIdx, transposeIdx);
      v = _mm_shuffle_epi8(v, _mm_setr_epi8(0, 1, 8, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
      *((int32_t *) (clPtr1++)) = _mm_extract_epi32(v, 0); clPtr1++;
      v = _mm_unpackhi_epi8(classIdx, transposeIdx);
      v = _mm_shuffle_epi8(v, _mm_setr_epi8(0, 1, 8, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
      *((int32_t *) (clPtr2++)) = _mm_extract_epi32(v, 0); clPtr2++;
    }
  }
}

template<X86_VEXT vext, bool isNonLinear>
void simdFilter5x5Blk( const AlfClassifier *classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc,
  const Area& blkDst, const Area& blk, const ComponentID compId, const short *filterSet,
  const short *fClipSet, const ClpRng &clpRng, const CodingStructure &cs, const int vbCTUHeight,
  int vbPos)

{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");
  CHECK(!isChroma(compId), "ALF 5x5 filter is for chroma only");

  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;

  constexpr int SHIFT    = AdaptiveLoopFilter::m_NUM_BITS - 1;
  constexpr int SHIFT_P3 = SHIFT + 3;
  constexpr int ROUND    = 1 << (SHIFT - 1);
  const __m128i mmOffset1 = _mm_set1_epi32((1 << ((SHIFT_P3) - 1)) - ROUND);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 4;

  CHECK(blk.y % STEP_Y, "Wrong startHeight in filtering");
  CHECK(blk.x % STEP_X, "Wrong startWidth in filtering");
  CHECK(height % STEP_Y, "Wrong endHeight in filtering");
  CHECK(width % 4, "Wrong endWidth in filtering");

  const Pel* src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel*       dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;


  const __m128i mmOffset = _mm_set1_epi32(ROUND);
  const __m128i mmMin = _mm_set1_epi16( clpRng.min() );
  const __m128i mmMax = _mm_set1_epi16( clpRng.max() );

  __m128i params[2][3];
  __m128i fs   = _mm_loadu_si128((__m128i *) filterSet);
  params[0][0] = _mm_shuffle_epi32(fs, 0x00);
  params[0][1] = _mm_shuffle_epi32(fs, 0x55);
  params[0][2] = _mm_shuffle_epi32(fs, 0xaa);
  if( isNonLinear )
  {
    __m128i fc = _mm_loadu_si128( ( __m128i * ) fClipSet );
    params[1][0] = _mm_shuffle_epi32( fc, 0x00 );
    params[1][1] = _mm_shuffle_epi32( fc, 0x55 );
    params[1][2] = _mm_shuffle_epi32( fc, 0xaa );
  }

  for (size_t i = 0; i < height; i += STEP_Y)
  {
    for (size_t j = 0; j < width; j += STEP_X)
    {

      for (size_t ii = 0; ii < STEP_Y; ii++)
      {
        const Pel* pImg0, *pImg1, *pImg2, *pImg3, *pImg4;

        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;

        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - 2))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + 1))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
        }
        bool isNearVBabove = yVb < vbPos && (yVb >= vbPos - 1);
        bool isNearVBbelow = yVb >= vbPos && (yVb <= vbPos);

        __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel* ptr0, const Pel* ptr1, const Pel* ptr2, const Pel* ptr3) {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

          __m128i val01A = _mm_unpacklo_epi16(val00, val10);
          __m128i val01B = _mm_unpackhi_epi16(val00, val10);
          __m128i val01C = _mm_unpacklo_epi16(val01, val11);
          __m128i val01D = _mm_unpackhi_epi16(val01, val11);

          if( isNonLinear )
          {
            __m128i limit01A = params[1][i];

            val01A = _mm_min_epi16( val01A, limit01A );
            val01B = _mm_min_epi16( val01B, limit01A );
            val01C = _mm_min_epi16( val01C, limit01A );
            val01D = _mm_min_epi16( val01D, limit01A );

            limit01A = _mm_sub_epi16( _mm_setzero_si128(), limit01A );

            val01A = _mm_max_epi16( val01A, limit01A );
            val01B = _mm_max_epi16( val01B, limit01A );
            val01C = _mm_max_epi16( val01C, limit01A );
            val01D = _mm_max_epi16( val01D, limit01A );
          }

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          __m128i coeff01A = params[0][i];

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01A));
        };

        process2coeffs(0, pImg3 + 0, pImg4 + 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(1, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(2, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

        if (!(isNearVBabove || isNearVBbelow))
        {
          accumA = _mm_srai_epi32(accumA, SHIFT);
          accumB = _mm_srai_epi32(accumB, SHIFT);
        }
        else
        {
          accumA = _mm_srai_epi32(_mm_add_epi32(accumA, mmOffset1), SHIFT_P3);
          accumB = _mm_srai_epi32(_mm_add_epi32(accumB, mmOffset1), SHIFT_P3);
        }

        accumA = _mm_packs_epi32(accumA, accumB);
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        if (j + STEP_X <= width)
        {
          _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j), accumA);
        }
        else
        {
          _mm_storel_epi64((__m128i *) (dst + ii * dstStride + j), accumA);
        }
      }

    }

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
  }
}

#if USE_AVX2
template<bool isNonLinear>
void simdFilter5x5Blk_AVX2( const AlfClassifier *, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, const short *filterSet, const short* fClipSet, const ClpRng &clpRng, const CodingStructure &cs, const int vbCTUHeight, int vbPos )
{
  CHECK( !isChroma( compId ), "ALF 5x5 filter is for chroma only" );

  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;

  constexpr int SHIFT    = AdaptiveLoopFilter::m_NUM_BITS - 1;
  constexpr int SHIFT_P3 = SHIFT + 3;
  constexpr int ROUND    = 1 << (SHIFT - 1);
  const __m256i mmOffset1 = _mm256_set1_epi32((1 << ((SHIFT_P3) - 1)) - ROUND);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t STEP_X = 16;
  constexpr size_t STEP_Y =  4;

  const Pel* src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel*       dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;

  const __m256i mmOffset = _mm256_set1_epi32( ROUND );
  const __m256i mmMin    = _mm256_set1_epi16( clpRng.min() );
  const __m256i mmMax    = _mm256_set1_epi16( clpRng.max() );

  __m256i params[2][3];
  __m256i fs   = _mm256_castsi128_si256( _mm_loadu_si128( ( __m128i* ) filterSet ) );
  fs = _mm256_inserti128_si256( fs, _mm256_extracti128_si256( fs, 0 ), 1 );
  params[0][0] = _mm256_shuffle_epi32(fs, 0x00);
  params[0][1] = _mm256_shuffle_epi32(fs, 0x55);
  params[0][2] = _mm256_shuffle_epi32(fs, 0xaa);
  if( isNonLinear )
  {
    __m256i fc = _mm256_castsi128_si256( _mm_loadu_si128( ( __m128i* ) fClipSet ) );
    fc = _mm256_inserti128_si256( fc, _mm256_extracti128_si256( fc, 0 ), 1 );
    params[1][0] = _mm256_shuffle_epi32( fc, 0x00 );
    params[1][1] = _mm256_shuffle_epi32( fc, 0x55 );
    params[1][2] = _mm256_shuffle_epi32( fc, 0xaa );
  }

  for (int i = 0; i < height; i += STEP_Y)
  {
    for (int j = 0; j < width; j += STEP_X)
    {
      for (size_t ii = 0; ii < STEP_Y; ii++)
      {
        const Pel* pImg0, *pImg1, *pImg2, *pImg3, *pImg4;

        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        
        const int yVb = ( blkDst.y + i + ii ) & ( vbCTUHeight - 1 );

        if( yVb < vbPos && ( yVb >= vbPos - 2 ) )   // above
        {
          pImg1 = ( yVb == vbPos - 1 ) ? pImg0 : pImg1;
          pImg3 = ( yVb >= vbPos - 2 ) ? pImg1 : pImg3;

          pImg2 = ( yVb == vbPos - 1 ) ? pImg0 : pImg2;
          pImg4 = ( yVb >= vbPos - 2 ) ? pImg2 : pImg4;
        }
        else if( yVb >= vbPos && ( yVb <= vbPos + 1 ) )   // bottom
        {
          pImg2 = ( yVb == vbPos     ) ? pImg0 : pImg2;
          pImg4 = ( yVb <= vbPos + 1 ) ? pImg2 : pImg4;

          pImg1 = ( yVb == vbPos     ) ? pImg0 : pImg1;
          pImg3 = ( yVb <= vbPos + 1 ) ? pImg1 : pImg3;
        }

        bool isNearVBabove = yVb < vbPos && (yVb >= vbPos - 1);
        bool isNearVBbelow = yVb >= vbPos && (yVb <= vbPos);

        __m256i cur = _mm256_loadu_si256((const __m256i *) pImg0);

        __m256i accumA = mmOffset;
        __m256i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel* ptr0, const Pel* ptr1, const Pel* ptr2, const Pel* ptr3) {
          const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
          const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
          const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
          const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

          __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
          __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
          __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
          __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

          if( isNonLinear )
          {
            __m256i limit01A = params[1][i];

            val01A = _mm256_min_epi16( val01A, limit01A );
            val01B = _mm256_min_epi16( val01B, limit01A );
            val01C = _mm256_min_epi16( val01C, limit01A );
            val01D = _mm256_min_epi16( val01D, limit01A );

            limit01A = _mm256_sub_epi16( _mm256_setzero_si256(), limit01A );

            val01A = _mm256_max_epi16( val01A, limit01A );
            val01B = _mm256_max_epi16( val01B, limit01A );
            val01C = _mm256_max_epi16( val01C, limit01A );
            val01D = _mm256_max_epi16( val01D, limit01A );
          }

          val01A = _mm256_add_epi16( val01A, val01C );
          val01B = _mm256_add_epi16( val01B, val01D );

          __m256i coeff01A = params[0][i];

          accumA = _mm256_add_epi32( accumA, _mm256_madd_epi16( val01A, coeff01A ) );
          accumB = _mm256_add_epi32( accumB, _mm256_madd_epi16( val01B, coeff01A ) );
        };

        process2coeffs(0, pImg3 + 0, pImg4 + 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(1, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(2, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

        if (!(isNearVBabove || isNearVBbelow))
        {
          accumA = _mm256_srai_epi32(accumA, SHIFT);
          accumB = _mm256_srai_epi32(accumB, SHIFT);
        }
        else
        {
          accumA = _mm256_srai_epi32(_mm256_add_epi32(accumA, mmOffset1), SHIFT_P3);
          accumB = _mm256_srai_epi32(_mm256_add_epi32(accumB, mmOffset1), SHIFT_P3);
        }

        accumA = _mm256_packs_epi32(accumA, accumB);
        accumA = _mm256_add_epi16(accumA, cur);
        accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

        if (j + STEP_X <= width)
        {
          _mm256_storeu_si256((__m256i *) (dst + ii * dstStride + j), accumA);
        }
        else if(j + 12 <= width)
        {
          _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j    ), _mm256_castsi256_si128(accumA));
          _mm_storel_epi64((__m128i *) (dst + ii * dstStride + j + 8), _mm256_extracti128_si256(accumA, 1));
        }
        else if(j + 8 <= width)
        {
          _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j), _mm256_castsi256_si128(accumA));
        }
        else
        {
          _mm_storel_epi64((__m128i *) (dst + ii * dstStride + j), _mm256_castsi256_si128( accumA ) );
        }
      }
    }

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
  }
}

#endif

constexpr uint16_t sh(int x)
{
  return 0x0202 * (x & 7) + 0x0100 + 0x1010 * (x & 8);
}

static const uint16_t shuffleTab[4][2][8] = {
  {
    { sh(0), sh(1), sh(2), sh(3), sh(4), sh(5), sh(6), sh(7) },
    { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
  },
  {
    { sh(9), sh(4), sh(10), sh(8), sh(1), sh(5), sh(11), sh(7) },
    { sh(3), sh(0), sh(2), sh(6), sh(12), sh(13), sh(14), sh(15) },
  },
  {
    { sh(0), sh(3), sh(2), sh(1), sh(8), sh(7), sh(6), sh(5) },
    { sh(4), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
  },
  {
    { sh(9), sh(8), sh(10), sh(4), sh(3), sh(7), sh(11), sh(5) },
    { sh(1), sh(0), sh(2), sh(6), sh(12), sh(13), sh(14), sh(15) },
  },
};

template<X86_VEXT vext, bool isNonLinear>
void simdFilter7x7Blk( const AlfClassifier *classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc,
  const Area& blkDst, const Area& blk, const ComponentID compId, const short *filterSet,
  const short *fClipSet, const ClpRng &clpRng, const CodingStructure &cs, const int vbCTUHeight,
  int vbPos)
{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");
  CHECK(isChroma(compId), "7x7 ALF filter is meant for luma only");


  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;

  constexpr int SHIFT    = AdaptiveLoopFilter::m_NUM_BITS - 1;
  constexpr int SHIFT_P3 = SHIFT + 3;
  constexpr int ROUND    = 1 << (SHIFT - 1);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 4;

  CHECK(blk.y % STEP_Y, "Wrong startHeight in filtering");
  CHECK(blk.x % STEP_X, "Wrong startWidth in filtering");
  CHECK(height % STEP_Y, "Wrong endHeight in filtering");
  CHECK(width % STEP_X, "Wrong endWidth in filtering");

  const Pel* src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel*       dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;

  const __m128i mmOffset = _mm_set1_epi32(ROUND);
  const __m128i mmOffset1 = _mm_set1_epi32((1 << ((SHIFT_P3) - 1)) - ROUND);
  const __m128i mmMin = _mm_set1_epi16( clpRng.min() );
  const __m128i mmMax = _mm_set1_epi16( clpRng.max() );

  for (size_t i = 0; i < height; i += STEP_Y)
  {

    for (size_t j = 0; j < width; j += STEP_X)
    {
      __m128i params[2][2][6];

      for (int k = 0; k < 2; ++k)
      {
        const AlfClassifier &cl = classifier[( i / 4 ) * ( MAX_CU_SIZE / 4 ) + ( j / 4 ) + k];

        const int transposeIdx = cl.transposeIdx;
        const int classIdx     = cl.classIdx;

        static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
        static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

        __m128i rawCoeff0, rawCoeff1;

        rawCoeff0 = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
        rawCoeff1 = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));

        const __m128i s0 = _mm_loadu_si128((const __m128i *) shuffleTab[transposeIdx][0]);
        const __m128i s1 = _mm_xor_si128(s0, _mm_set1_epi8((char) 0x80));
        const __m128i s2 = _mm_loadu_si128((const __m128i *) shuffleTab[transposeIdx][1]);
        const __m128i s3 = _mm_xor_si128(s2, _mm_set1_epi8((char) 0x80));

        const __m128i rawCoeffLo = _mm_or_si128(_mm_shuffle_epi8(rawCoeff0, s0), _mm_shuffle_epi8(rawCoeff1, s1));
        const __m128i rawCoeffHi = _mm_or_si128(_mm_shuffle_epi8(rawCoeff0, s2), _mm_shuffle_epi8(rawCoeff1, s3));

        params[k][0][0] = _mm_shuffle_epi32(rawCoeffLo, 0x00);
        params[k][0][1] = _mm_shuffle_epi32(rawCoeffLo, 0x55);
        params[k][0][2] = _mm_shuffle_epi32(rawCoeffLo, 0xaa);
        params[k][0][3] = _mm_shuffle_epi32(rawCoeffLo, 0xff);
        params[k][0][4] = _mm_shuffle_epi32(rawCoeffHi, 0x00);
        params[k][0][5] = _mm_shuffle_epi32(rawCoeffHi, 0x55);

        if( isNonLinear )
        {
          __m128i rawClip0, rawClip1;
          rawClip0 = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip1 = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          const __m128i rawClipLo  = _mm_or_si128(_mm_shuffle_epi8(rawClip0, s0), _mm_shuffle_epi8(rawClip1, s1));
          const __m128i rawClipHi  = _mm_or_si128(_mm_shuffle_epi8(rawClip0, s2), _mm_shuffle_epi8(rawClip1, s3));
          params[k][1][0] = _mm_shuffle_epi32( rawClipLo, 0x00 );
          params[k][1][1] = _mm_shuffle_epi32( rawClipLo, 0x55 );
          params[k][1][2] = _mm_shuffle_epi32( rawClipLo, 0xaa );
          params[k][1][3] = _mm_shuffle_epi32( rawClipLo, 0xff );
          params[k][1][4] = _mm_shuffle_epi32( rawClipHi, 0x00 );
          params[k][1][5] = _mm_shuffle_epi32( rawClipHi, 0x55 );
        }
      }

      for (size_t ii = 0; ii < STEP_Y; ii++)
      {
        const Pel* pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;

        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - 4))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;
          pImg5 = (yVb >= vbPos - 3) ? pImg3 : pImg5;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
          pImg6 = (yVb >= vbPos - 3) ? pImg4 : pImg6;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + 3))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;
          pImg6 = (yVb <= vbPos + 2) ? pImg4 : pImg6;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
          pImg5 = (yVb <= vbPos + 2) ? pImg3 : pImg5;
        }

        bool isNearVBabove = yVb < vbPos && (yVb >= vbPos - 1);
        bool isNearVBbelow = yVb >= vbPos && (yVb <= vbPos);

        __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);

        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel* ptr0, const Pel* ptr1, const Pel* ptr2, const Pel* ptr3) {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

          __m128i val01A = _mm_unpacklo_epi16(val00, val10);
          __m128i val01B = _mm_unpackhi_epi16(val00, val10);
          __m128i val01C = _mm_unpacklo_epi16(val01, val11);
          __m128i val01D = _mm_unpackhi_epi16(val01, val11);

          if( isNonLinear )
          {
            __m128i limit01A = params[0][1][i];
            __m128i limit01B = params[1][1][i];

            val01A = _mm_min_epi16( val01A, limit01A );
            val01B = _mm_min_epi16( val01B, limit01B );
            val01C = _mm_min_epi16( val01C, limit01A );
            val01D = _mm_min_epi16( val01D, limit01B );

            limit01A = _mm_sub_epi16( _mm_setzero_si128(), limit01A );
            limit01B = _mm_sub_epi16( _mm_setzero_si128(), limit01B );

            val01A = _mm_max_epi16( val01A, limit01A );
            val01B = _mm_max_epi16( val01B, limit01B );
            val01C = _mm_max_epi16( val01C, limit01A );
            val01D = _mm_max_epi16( val01D, limit01B );
          }

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff01A = params[0][0][i];
          const __m128i coeff01B = params[1][0][i];

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
        };

        process2coeffs(0, pImg5 + 0, pImg6 + 0, pImg3 + 1, pImg4 - 1);
        process2coeffs(1, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
        process2coeffs(2, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
        process2coeffs(3, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(4, pImg1 - 2, pImg2 + 2, pImg0 + 3, pImg0 - 3);
        process2coeffs(5, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

        if (!(isNearVBabove || isNearVBbelow))
        {
          accumA = _mm_srai_epi32(accumA, SHIFT);
          accumB = _mm_srai_epi32(accumB, SHIFT);
        }
        else
        {
          accumA = _mm_srai_epi32(_mm_add_epi32(accumA, mmOffset1), SHIFT_P3);
          accumB = _mm_srai_epi32(_mm_add_epi32(accumB, mmOffset1), SHIFT_P3);
        }

        accumA = _mm_packs_epi32(accumA, accumB);
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j), accumA);
      }
    }

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
  }
}


#if USE_AVX2
template<bool isNonLinear>
void simdFilter7x7Blk_AVX2( const AlfClassifier *classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, const CodingStructure &cs, const int vbCTUHeight, int vbPos)
{

  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;

  constexpr int SHIFT    = AdaptiveLoopFilter::m_NUM_BITS - 1;
  constexpr int SHIFT_P3 = SHIFT + 3;
  constexpr int ROUND     = 1 << (SHIFT - 1);
  const __m256i mmOffset1 = _mm256_set1_epi32((1 << ((SHIFT_P3) - 1)) - ROUND);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t STEP_X = 16;
  constexpr size_t STEP_Y =  4;

  const Pel* src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel*       dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;

  const __m256i mmOffset = _mm256_set1_epi32( ROUND );
  const __m256i mmMin    = _mm256_set1_epi16( clpRng.min() );
  const __m256i mmMax    = _mm256_set1_epi16( clpRng.max() );

  for (size_t i = 0; i < height; i += STEP_Y)
  {

    for (size_t j = 0; j < width; j += STEP_X)
    {
      __m256i params[2][2][6];

      for (int k = 0; k < 2; ++k)
      { 
        const AlfClassifier &cl = classifier[( i / 4 ) * ( MAX_CU_SIZE / 4 ) + ( j / 4 ) + k];

        const int transposeIdx = cl.transposeIdx;
        const int classIdx     = cl.classIdx;

        const short *coef = filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF;
        
        const __m128i s0 = _mm_loadu_si128( ( const __m128i * ) shuffleTab[transposeIdx][0] );
        const __m128i s1 = _mm_xor_si128  ( s0, _mm_set1_epi8( ( char ) 0x80 ) );
        const __m128i s2 = _mm_loadu_si128( ( const __m128i * ) shuffleTab[transposeIdx][1] );
        const __m128i s3 = _mm_xor_si128  ( s2, _mm_set1_epi8( ( char ) 0x80 ) );
        
        __m128i rawCoeff0, rawCoeff1;
          rawCoeff0 = _mm_loadu_si128( ( const __m128i * ) ( coef ) );
          rawCoeff1 = _mm_loadl_epi64( ( const __m128i * ) ( coef + 8 ) );
        
        const __m128i rawCoeffLo = _mm_or_si128( _mm_shuffle_epi8( rawCoeff0, s0 ), _mm_shuffle_epi8( rawCoeff1, s1 ) );
        const __m128i rawCoeffHi = _mm_or_si128( _mm_shuffle_epi8( rawCoeff0, s2 ), _mm_shuffle_epi8( rawCoeff1, s3 ) );
        params[k][0][0] = _mm256_inserti128_si256(_mm256_setzero_si256(), _mm_shuffle_epi32(rawCoeffLo, 0x00), 0);
        params[k][0][1] = _mm256_inserti128_si256(_mm256_setzero_si256(), _mm_shuffle_epi32(rawCoeffLo, 0x55), 0);
        params[k][0][2] = _mm256_inserti128_si256(_mm256_setzero_si256(), _mm_shuffle_epi32(rawCoeffLo, 0xaa), 0);
        params[k][0][3] = _mm256_inserti128_si256(_mm256_setzero_si256(), _mm_shuffle_epi32(rawCoeffLo, 0xff), 0);
        params[k][0][4] = _mm256_inserti128_si256(_mm256_setzero_si256(), _mm_shuffle_epi32(rawCoeffHi, 0x00), 0);
        params[k][0][5] = _mm256_inserti128_si256(_mm256_setzero_si256(), _mm_shuffle_epi32(rawCoeffHi, 0x55), 0);

        if( isNonLinear )
        {
          const short *clip = fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF;
          __m128i rawClip0 = _mm_loadu_si128( ( const __m128i * ) ( clip ) );
          __m128i rawClip1 = _mm_loadl_epi64( ( const __m128i * ) ( clip + 8 ) );
          const __m128i rawClipLo = _mm_or_si128( _mm_shuffle_epi8( rawClip0, s0 ), _mm_shuffle_epi8( rawClip1, s1 ) );
          const __m128i rawClipHi = _mm_or_si128( _mm_shuffle_epi8( rawClip0, s2 ), _mm_shuffle_epi8( rawClip1, s3 ) );

          params[k][1][0] = _mm256_inserti128_si256( _mm256_setzero_si256(), _mm_shuffle_epi32( rawClipLo, 0x00 ), 0 );
          params[k][1][1] = _mm256_inserti128_si256( _mm256_setzero_si256(), _mm_shuffle_epi32( rawClipLo, 0x55 ), 0 );
          params[k][1][2] = _mm256_inserti128_si256( _mm256_setzero_si256(), _mm_shuffle_epi32( rawClipLo, 0xaa ), 0 );
          params[k][1][3] = _mm256_inserti128_si256( _mm256_setzero_si256(), _mm_shuffle_epi32( rawClipLo, 0xff ), 0 );
          params[k][1][4] = _mm256_inserti128_si256( _mm256_setzero_si256(), _mm_shuffle_epi32( rawClipHi, 0x00 ), 0 );
          params[k][1][5] = _mm256_inserti128_si256( _mm256_setzero_si256(), _mm_shuffle_epi32( rawClipHi, 0x55 ), 0 );
        }
      }

      for( int k = 2; k < 4; ++k )
      {
        if( j + ( k << 2 ) >= width ) break;
        
        const AlfClassifier &cl = classifier[( i / 4 ) * ( MAX_CU_SIZE / 4 ) + ( j / 4 ) + k];
        
        const int transposeIdx = cl.transposeIdx;
        const int classIdx     = cl.classIdx;

        const short *coef = filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF;
        
        const __m128i s0 = _mm_loadu_si128( ( const __m128i * ) shuffleTab[transposeIdx][0] );
        const __m128i s1 = _mm_xor_si128  ( s0, _mm_set1_epi8( ( char ) 0x80 ) );
        const __m128i s2 = _mm_loadu_si128( ( const __m128i * ) shuffleTab[transposeIdx][1] );
        const __m128i s3 = _mm_xor_si128  ( s2, _mm_set1_epi8( ( char ) 0x80 ) );
        
        __m128i rawCoeff0, rawCoeff1;

          rawCoeff0 = _mm_loadu_si128( ( const __m128i * ) ( coef ) );
          rawCoeff1 = _mm_loadl_epi64( ( const __m128i * ) ( coef + 8 ) );
        
        const __m128i rawCoeffLo = _mm_or_si128( _mm_shuffle_epi8( rawCoeff0, s0 ), _mm_shuffle_epi8( rawCoeff1, s1 ) );
        const __m128i rawCoeffHi = _mm_or_si128( _mm_shuffle_epi8( rawCoeff0, s2 ), _mm_shuffle_epi8( rawCoeff1, s3 ) );

        params[k - 2][0][0] = _mm256_inserti128_si256( params[k - 2][0][0], _mm_shuffle_epi32( rawCoeffLo, 0x00 ), 1 );
        params[k - 2][0][1] = _mm256_inserti128_si256( params[k - 2][0][1], _mm_shuffle_epi32( rawCoeffLo, 0x55 ), 1 );
        params[k - 2][0][2] = _mm256_inserti128_si256( params[k - 2][0][2], _mm_shuffle_epi32( rawCoeffLo, 0xaa ), 1 );
        params[k - 2][0][3] = _mm256_inserti128_si256( params[k - 2][0][3], _mm_shuffle_epi32( rawCoeffLo, 0xff ), 1 );
        params[k - 2][0][4] = _mm256_inserti128_si256( params[k - 2][0][4], _mm_shuffle_epi32( rawCoeffHi, 0x00 ), 1 );
        params[k - 2][0][5] = _mm256_inserti128_si256( params[k - 2][0][5], _mm_shuffle_epi32( rawCoeffHi, 0x55 ), 1 );

        if( isNonLinear )
        {
          const short *clip = fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF;
          __m128i rawClip0, rawClip1;
          rawClip0 = _mm_loadu_si128( ( const __m128i * ) ( clip ) );
          rawClip1 = _mm_loadl_epi64( ( const __m128i * ) ( clip + 8 ) );
          const __m128i rawClipLo = _mm_or_si128( _mm_shuffle_epi8( rawClip0, s0 ), _mm_shuffle_epi8( rawClip1, s1 ) );
          const __m128i rawClipHi = _mm_or_si128( _mm_shuffle_epi8( rawClip0, s2 ), _mm_shuffle_epi8( rawClip1, s3 ) );
          params[k - 2][1][0] = _mm256_inserti128_si256( params[k - 2][1][0], _mm_shuffle_epi32( rawClipLo, 0x00 ), 1 );
          params[k - 2][1][1] = _mm256_inserti128_si256( params[k - 2][1][1], _mm_shuffle_epi32( rawClipLo, 0x55 ), 1 );
          params[k - 2][1][2] = _mm256_inserti128_si256( params[k - 2][1][2], _mm_shuffle_epi32( rawClipLo, 0xaa ), 1 );
          params[k - 2][1][3] = _mm256_inserti128_si256( params[k - 2][1][3], _mm_shuffle_epi32( rawClipLo, 0xff ), 1 );
          params[k - 2][1][4] = _mm256_inserti128_si256( params[k - 2][1][4], _mm_shuffle_epi32( rawClipHi, 0x00 ), 1 );
          params[k - 2][1][5] = _mm256_inserti128_si256( params[k - 2][1][5], _mm_shuffle_epi32( rawClipHi, 0x55 ), 1 );
        }
      }

      for (size_t ii = 0; ii < STEP_Y; ii++)
      {
        const Pel* pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
        
        const int yVb = ( blkDst.y + i + ii ) & ( vbCTUHeight - 1 );

        if( yVb < vbPos && ( yVb >= vbPos - 4 ) )   // above
        {
          pImg1 = ( yVb == vbPos - 1 ) ? pImg0 : pImg1;
          pImg3 = ( yVb >= vbPos - 2 ) ? pImg1 : pImg3;
          pImg5 = ( yVb >= vbPos - 3 ) ? pImg3 : pImg5;

          pImg2 = ( yVb == vbPos - 1 ) ? pImg0 : pImg2;
          pImg4 = ( yVb >= vbPos - 2 ) ? pImg2 : pImg4;
          pImg6 = ( yVb >= vbPos - 3 ) ? pImg4 : pImg6;
        }
        else if( yVb >= vbPos && ( yVb <= vbPos + 3 ) )   // bottom
        {
          pImg2 = ( yVb == vbPos     ) ? pImg0 : pImg2;
          pImg4 = ( yVb <= vbPos + 1 ) ? pImg2 : pImg4;
          pImg6 = ( yVb <= vbPos + 2 ) ? pImg4 : pImg6;

          pImg1 = ( yVb == vbPos     ) ? pImg0 : pImg1;
          pImg3 = ( yVb <= vbPos + 1 ) ? pImg1 : pImg3;
          pImg5 = ( yVb <= vbPos + 2 ) ? pImg3 : pImg5;
        }

        bool isNearVBabove = yVb < vbPos && (yVb >= vbPos - 1);
        bool isNearVBbelow = yVb >= vbPos && (yVb <= vbPos);


        __m256i cur = _mm256_loadu_si256((const __m256i *) pImg0);

        __m256i accumA = mmOffset;
        __m256i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel* ptr0, const Pel* ptr1, const Pel* ptr2, const Pel* ptr3) {
          const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
          const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
          const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
          const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

          __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
          __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
          __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
          __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

          if( isNonLinear )
          {
            __m256i limit01A = params[0][1][i];
            __m256i limit01B = params[1][1][i];

            val01A = _mm256_min_epi16( val01A, limit01A );
            val01B = _mm256_min_epi16( val01B, limit01B );
            val01C = _mm256_min_epi16( val01C, limit01A );
            val01D = _mm256_min_epi16( val01D, limit01B );

            limit01A = _mm256_sub_epi16( _mm256_setzero_si256(), limit01A );
            limit01B = _mm256_sub_epi16( _mm256_setzero_si256(), limit01B );

            val01A = _mm256_max_epi16( val01A, limit01A );
            val01B = _mm256_max_epi16( val01B, limit01B );
            val01C = _mm256_max_epi16( val01C, limit01A );
            val01D = _mm256_max_epi16( val01D, limit01B );
          }
          val01A = _mm256_add_epi16( val01A, val01C );
          val01B = _mm256_add_epi16( val01B, val01D );

          const __m256i coeff01A = params[0][0][i];
          const __m256i coeff01B = params[1][0][i];

          accumA = _mm256_add_epi32( accumA, _mm256_madd_epi16( val01A, coeff01A ) );
          accumB = _mm256_add_epi32( accumB, _mm256_madd_epi16( val01B, coeff01B ) );
        };

        process2coeffs(0, pImg5 + 0, pImg6 + 0, pImg3 + 1, pImg4 - 1);
        process2coeffs(1, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
        process2coeffs(2, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
        process2coeffs(3, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(4, pImg1 - 2, pImg2 + 2, pImg0 + 3, pImg0 - 3);
        process2coeffs(5, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

        if (!(isNearVBabove || isNearVBbelow))
        {
          accumA = _mm256_srai_epi32(accumA, SHIFT);
          accumB = _mm256_srai_epi32(accumB, SHIFT);
        }
        else
        {
          accumA = _mm256_srai_epi32(_mm256_add_epi32(accumA, mmOffset1), SHIFT_P3);
          accumB = _mm256_srai_epi32(_mm256_add_epi32(accumB, mmOffset1), SHIFT_P3);
        }

        accumA = _mm256_packs_epi32(accumA, accumB);
        accumA = _mm256_add_epi16(accumA, cur);
        accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

        if (j + STEP_X <= width)
        {
          _mm256_storeu_si256((__m256i *) (dst + ii * dstStride + j), accumA);
        }
        else if(j + 8 <= width)
        {
          _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j), _mm256_castsi256_si128(accumA));
        }
        else
        {
          _mm_storel_epi64((__m128i *) (dst + ii * dstStride + j), _mm256_castsi256_si128( accumA ) );
        }
      }
    }

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
  }
}

#endif

template<X86_VEXT vext>
void simdFilterBlkCcAlf( const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst,
                          const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff,
                          const ClpRngs &clpRngs, CodingStructure &cs, int vbCTUHeight, int vbPos )
{
  CHECK( 1 << floorLog2( vbCTUHeight ) != vbCTUHeight, "Not a power of 2" );

  CHECK( !isChroma( compId ), "Must be chroma" );

  static constexpr int scaleBits = 7; // 8-bits
  static constexpr int clsSizeY  = 4;
  static constexpr int clsSizeX  = 4;

  const SPS* sps        = cs.slice->sps;
  ChromaFormat nChromaFormat = sps->chromaFormatIdc;
  const int startHeight = blkDst.y;
  const int endHeight   = blkDst.y + blkDst.height;
  const int startWidth  = blkDst.x;
  const int endWidth    = blkDst.x + blkDst.width;
  const int scaleX      = getComponentScaleX( compId, nChromaFormat );
  const int scaleY      = getComponentScaleY( compId, nChromaFormat );

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  const CPelBuf& srcBuf  = recSrc.get( COMP_Y );
  const int   lumaStride = srcBuf.stride;
  const Pel * lumaPtr    = srcBuf.buf + blkSrc.y * lumaStride + blkSrc.x;

  const int   chromaStride = dstBuf.stride;
  Pel *       chromaPtr = dstBuf.buf + blkDst.y * chromaStride + blkDst.x;
  
  if( getChannelTypeScaleX( CH_C, nChromaFormat ) == 1 )
  {
    __m128i xfilterCoeff[4];
    xfilterCoeff[0] = _mm_set1_epi32( ( filterCoeff[1] & 0xffff ) | ( filterCoeff[2] * (1<< 16 )));
    xfilterCoeff[1] = _mm_set1_epi32( ( filterCoeff[0] & 0xffff ) | ( filterCoeff[3] * (1<< 16 )));
    xfilterCoeff[2] = _mm_set1_epi32( ( filterCoeff[4] & 0xffff ) | ( filterCoeff[5] * (1<< 16 )));
    xfilterCoeff[3] = _mm_set1_epi32( ( filterCoeff[6] & 0xffff ) );

    for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
    {
      for( int j = 0; j < endWidth - startWidth; j += clsSizeX )
      {
        for( int ii = 0; ii < clsSizeY; ii++ )
        {
          int row = ii;
          int col = j;
          Pel *srcSelf = chromaPtr + col + row * chromaStride;

          int offset1 = lumaStride;
          int offset2 = -lumaStride;
          int offset3 = 2 * lumaStride;
          row <<= scaleY;
          col <<= scaleX;
          const Pel *srcCross = lumaPtr + col + row * lumaStride;

          int pos = ( ( startHeight + i + ii ) << scaleY ) & ( vbCTUHeight - 1 );
          if( pos == ( vbPos - 2 ) || pos == ( vbPos + 1 ) )
          {
            offset3 = offset1;
          }
          else if( pos == ( vbPos - 1 ) || pos == vbPos )
          {
            offset1 = 0;
            offset2 = 0;
            offset3 = 0;
          }
  
          const int offset0 = 0;
  
          __m128i xin0      = _mm_loadu_si128( ( const __m128i* ) &srcCross[offset0 - 1] );
          __m128i xsrcCross = _mm_loadu_si128( ( const __m128i* ) &srcCross[offset0    ] );
          __m128i xsum      = _mm_set1_epi32( 1 << ( scaleBits - 1 ) );
          
          xin0      = _mm_blend_epi16( xin0, xsrcCross, 0 + 2 + 8 + 32 + 128 );
          
          xsrcCross = _mm_shufflelo_epi16( xsrcCross, ( 0 << 0 ) + ( 0 << 2 ) + ( 2 << 4 ) + ( 2 << 6 ) );
          xsrcCross = _mm_shufflehi_epi16( xsrcCross, ( 0 << 0 ) + ( 0 << 2 ) + ( 2 << 4 ) + ( 2 << 6 ) );
          
          xin0      = _mm_sub_epi16( xin0, xsrcCross );
          xin0      = _mm_madd_epi16( xin0, xfilterCoeff[0] );
          
          xsum      = _mm_add_epi32( xsum, xin0 );
          
          xin0      = _mm_loadu_si128( ( const __m128i* ) &srcCross[offset2] );
          __m128i
          xin1      = _mm_loadu_si128( ( const __m128i* ) &srcCross[offset1 - 2] );
          xin0      = _mm_blend_epi16( xin0, xin1, 0 + 2 + 8 + 32 + 128 );
          
          xin0      = _mm_sub_epi16( xin0, xsrcCross );
          xin0      = _mm_madd_epi16( xin0, xfilterCoeff[1] );
          
          xsum      = _mm_add_epi32( xsum, xin0 );
          
          xin0      = _mm_loadu_si128( ( const __m128i* ) &srcCross[offset1] );
          xin0      = _mm_sub_epi16( xin0, xsrcCross );
          xin0      = _mm_madd_epi16( xin0, xfilterCoeff[2] );
          
          xsum      = _mm_add_epi32( xsum, xin0 );
          
          xin0      = _mm_loadu_si128( ( const __m128i* ) &srcCross[offset3] );
          xin0      = _mm_sub_epi16( xin0, xsrcCross );
          xin0      = _mm_madd_epi16( xin0, xfilterCoeff[3] );
          
          xsum      = _mm_add_epi32( xsum, xin0 );
          xsum      = _mm_srai_epi32( xsum, scaleBits );
          
          xin0      = _mm_set1_epi32( 1 << clpRngs[compId].bd >> 1 );
          xsum      = _mm_add_epi32( xsum, xin0 );
          
          xsum      = _mm_max_epi32( _mm_setzero_si128(), xsum );
          xsum      = _mm_min_epi32( _mm_set1_epi32( clpRngs[compId].max() ), xsum );
          xsum      = _mm_sub_epi32( xsum, xin0 );
          xsum      = _mm_packs_epi32( xsum, xsum );
          
          //int16_t vsum[4];
          //_mm_storel_epi64( ( __m128i* ) vsum, xsum );
          //
          //for( int jj = 0; jj < clsSizeX; jj++ )
          //{
          //  const int jj2 = ( jj << scaleX );
          //  //const int offset0 = 0;
          //
          //  int sum = 0;
          //  const Pel currSrcCross = srcCross[offset0 + jj2];
          //  sum += filterCoeff[0] * ( srcCross[offset2 + jj2] - currSrcCross );
          //  sum += filterCoeff[1] * ( srcCross[offset0 + jj2 - 1] - currSrcCross );
          //  sum += filterCoeff[2] * ( srcCross[offset0 + jj2 + 1] - currSrcCross );
          //  sum += filterCoeff[3] * ( srcCross[offset1 + jj2 - 1] - currSrcCross );
          //  sum += filterCoeff[4] * ( srcCross[offset1 + jj2] - currSrcCross );
          //  sum += filterCoeff[5] * ( srcCross[offset1 + jj2 + 1] - currSrcCross );
          //  sum += filterCoeff[6] * ( srcCross[offset3 + jj2] - currSrcCross );
          //
          //  sum = ( sum + ( ( 1 << scaleBits ) >> 1 ) ) >> scaleBits;
          //  const int offset = 1 << clpRngs.comp[compId].bd >> 1;
          //  sum = ClipPel( sum + offset, clpRngs.comp[compId] ) - offset;
          //  sum += srcSelf[jj];
          //
          //  CHECK( vsum[jj] != sum, "" );
          //}
  
          xin1 = _mm_loadl_epi64( ( const __m128i* ) &srcSelf[0] );
          xin0 = _mm_add_epi16( xsum, xin1 );
          
          xin0 = _mm_max_epi16( _mm_setzero_si128(), xin0 );
          xin0 = _mm_min_epi16( _mm_set1_epi16( clpRngs[compId].max() ), xin0 );
          
          _mm_storel_epi64( ( __m128i* ) &srcSelf[0], xin0 );
        }
      }

      chromaPtr += chromaStride * clsSizeY;

      lumaPtr += lumaStride * clsSizeY << getComponentScaleY( compId, nChromaFormat );
    }
  }
  else
  {
    // TODO: implement for 444 subsampling
    AdaptiveLoopFilter::filterBlkCcAlf<CC_ALF>( dstBuf, recSrc, blkDst, blkSrc, compId, filterCoeff, clpRngs, cs, vbCTUHeight, vbPos );
  }
}

#if USE_AVX2

template<>
void simdFilterBlkCcAlf<AVX2>( const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst,
                               const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff,
                               const ClpRngs &clpRngs, CodingStructure &cs, int vbCTUHeight, int vbPos )
{
  if( blkDst.width & 7 )
  {
    simdFilterBlkCcAlf<SSE42>( dstBuf, recSrc, blkDst, blkSrc, compId, filterCoeff, clpRngs, cs, vbCTUHeight, vbPos );
    return;
  }

  CHECK( 1 << floorLog2( vbCTUHeight ) != vbCTUHeight, "Not a power of 2" );

  CHECK( !isChroma( compId ), "Must be chroma" );

  static constexpr int scaleBits = 7; // 8-bits
  static constexpr int clsSizeY = 4;
  static constexpr int clsSizeX = 4;

  const SPS* sps = cs.slice->sps;
  ChromaFormat nChromaFormat = sps->chromaFormatIdc;
  const int startHeight = blkDst.y;
  const int endHeight = blkDst.y + blkDst.height;
  const int startWidth = blkDst.x;
  const int endWidth = blkDst.x + blkDst.width;
  const int scaleX = getComponentScaleX( compId, nChromaFormat );
  const int scaleY = getComponentScaleY( compId, nChromaFormat );

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  const CPelBuf& srcBuf = recSrc.get( COMP_Y );
  const int   lumaStride = srcBuf.stride;
  const Pel * lumaPtr = srcBuf.buf + blkSrc.y * lumaStride + blkSrc.x;

  const int   chromaStride = dstBuf.stride;
  Pel *       chromaPtr = dstBuf.buf + blkDst.y * chromaStride + blkDst.x;

  if( getChannelTypeScaleX( CH_C, nChromaFormat ) == 1 )
  {
    __m256i vfilterCoeff[4];
    vfilterCoeff[0] = _mm256_set1_epi32( ( filterCoeff[1] & 0xffff ) | ( filterCoeff[2] * (1<< 16 )) );
    vfilterCoeff[1] = _mm256_set1_epi32( ( filterCoeff[0] & 0xffff ) | ( filterCoeff[3] * (1<< 16 )) );
    vfilterCoeff[2] = _mm256_set1_epi32( ( filterCoeff[4] & 0xffff ) | ( filterCoeff[5] * (1<< 16 )));
    vfilterCoeff[3] = _mm256_set1_epi32( ( filterCoeff[6] & 0xffff ) );

    for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
    {
      for( int j = 0; j < endWidth - startWidth; j += 8 )
      {
        for( int ii = 0; ii < clsSizeY; ii++ )
        {
          int row = ii;
          int col = j;
          Pel *srcSelf = chromaPtr + col + row * chromaStride;

          int offset1 = lumaStride;
          int offset2 = -lumaStride;
          int offset3 = 2 * lumaStride;
          row <<= scaleY;
          col <<= scaleX;
          const Pel *srcCross = lumaPtr + col + row * lumaStride;

          int pos = ( ( startHeight + i + ii ) << scaleY ) & ( vbCTUHeight - 1 );
          if( pos == ( vbPos - 2 ) || pos == ( vbPos + 1 ) )
          {
            offset3 = offset1;
          }
          else if( pos == ( vbPos - 1 ) || pos == vbPos )
          {
            offset1 = 0;
            offset2 = 0;
            offset3 = 0;
          }

          const int offset0 = 0;

          __m256i vin0      = _mm256_loadu_si256( ( const __m256i* ) &srcCross[offset0 - 1] );
          __m256i vsrcCross = _mm256_loadu_si256( ( const __m256i* ) &srcCross[offset0] );
          __m256i vsum      = _mm256_set1_epi32( 1 << ( scaleBits - 1 ) );

          vin0 = _mm256_blend_epi16( vin0, vsrcCross, 0 + 2 + 8 + 32 + 128 );

          vsrcCross = _mm256_shufflelo_epi16( vsrcCross, ( 0 << 0 ) + ( 0 << 2 ) + ( 2 << 4 ) + ( 2 << 6 ) );
          vsrcCross = _mm256_shufflehi_epi16( vsrcCross, ( 0 << 0 ) + ( 0 << 2 ) + ( 2 << 4 ) + ( 2 << 6 ) );

          vin0 = _mm256_sub_epi16( vin0, vsrcCross );
          vin0 = _mm256_madd_epi16( vin0, vfilterCoeff[0] );

          vsum = _mm256_add_epi32( vsum, vin0 );

          vin0 = _mm256_loadu_si256( ( const __m256i* ) &srcCross[offset2] );
          __m256i
          vin1 = _mm256_loadu_si256( ( const __m256i* ) &srcCross[offset1 - 2] );
          vin0 = _mm256_blend_epi16( vin0, vin1, 0 + 2 + 8 + 32 + 128 );

          vin0 = _mm256_sub_epi16( vin0, vsrcCross );
          vin0 = _mm256_madd_epi16( vin0, vfilterCoeff[1] );

          vsum = _mm256_add_epi32( vsum, vin0 );

          vin0 = _mm256_loadu_si256( ( const __m256i* ) &srcCross[offset1] );
          vin0 = _mm256_sub_epi16( vin0, vsrcCross );
          vin0 = _mm256_madd_epi16( vin0, vfilterCoeff[2] );

          vsum = _mm256_add_epi32( vsum, vin0 );

          vin0 = _mm256_loadu_si256( ( const __m256i* ) &srcCross[offset3] );
          vin0 = _mm256_sub_epi16( vin0, vsrcCross );
          vin0 = _mm256_madd_epi16( vin0, vfilterCoeff[3] );

          vsum = _mm256_add_epi32( vsum, vin0 );
          vsum = _mm256_srai_epi32( vsum, scaleBits );

          vin0 = _mm256_set1_epi32( 1 << clpRngs[compId].bd >> 1 );
          vsum = _mm256_add_epi32( vsum, vin0 );

          vsum = _mm256_max_epi32( _mm256_setzero_si256(), vsum );
          vsum = _mm256_min_epi32( _mm256_set1_epi32( clpRngs[compId].max() ), vsum );
          vsum = _mm256_sub_epi32( vsum, vin0 );
          vsum = _mm256_packs_epi32( vsum, vsum );
          vsum = _mm256_permute4x64_epi64( vsum, ( 0 << 0 ) + ( 2 << 2 ) + ( 2 << 4 ) + ( 3 << 6 ) );

          __m128i
          xin0 = _mm256_castsi256_si128( vsum );
          __m128i
          xin1 = _mm_loadu_si128( ( const __m128i* ) &srcSelf[0] );
          xin0 = _mm_add_epi16( xin0, xin1 );

          xin0 = _mm_max_epi16( _mm_setzero_si128(), xin0 );
          xin0 = _mm_min_epi16( _mm_set1_epi16( clpRngs[compId].max() ), xin0 );

          _mm_storeu_si128( ( __m128i* ) &srcSelf[0], xin0 );
        }
      }

      chromaPtr += chromaStride * clsSizeY;

      lumaPtr += lumaStride * clsSizeY << getComponentScaleY( compId, nChromaFormat );
    }
  }
  else
  {
    // TODO: implement for 444 subsampling
    AdaptiveLoopFilter::filterBlkCcAlf<CC_ALF>( dstBuf, recSrc, blkDst, blkSrc, compId, filterCoeff, clpRngs, cs, vbCTUHeight, vbPos );
  }
}

#endif

template <X86_VEXT vext>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86()
{
  m_deriveClassificationBlk = simdDeriveClassificationBlk<vext>;
#if USE_AVX2
  if( vext == AVX2 )
  {
    m_filter5x5Blk[0] = simdFilter5x5Blk_AVX2<false>; // NonLin is Off
    m_filter5x5Blk[1] = simdFilter5x5Blk_AVX2<true>;  // NonLin is On
    m_filter7x7Blk[0] = simdFilter7x7Blk_AVX2<false>; // NonLin is Off
    m_filter7x7Blk[1] = simdFilter7x7Blk_AVX2<true>;  // NonLin is On
  }
  else
#endif
  {
    m_filter5x5Blk[0] = simdFilter5x5Blk<vext, false>; // NonLin is Off
    m_filter5x5Blk[1] = simdFilter5x5Blk<vext, true>;  // NonLin is On
    m_filter7x7Blk[0] = simdFilter7x7Blk<vext, false>; // NonLin is Off
    m_filter7x7Blk[1] = simdFilter7x7Blk<vext, true>;  // NonLin is On
  }
  m_filterCcAlf  = simdFilterBlkCcAlf<vext>;
}

template void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86<SIMDX86>();

} // namespace vvenc

//! \}
#endif // ENABLE_SIMD_OPT_ALF
#endif // TARGET_SIMD_X86

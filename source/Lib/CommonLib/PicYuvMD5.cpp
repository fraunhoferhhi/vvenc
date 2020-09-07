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


#include "PicYuvMD5.h"
#include "Picture.h"
#include "SEI.h"
#include "libmd5/MD5.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

/**
 * Update md5 using n samples from plane, each sample is adjusted to
 * OUTBIT_BITDEPTH_DIV8.
 */
template<uint32_t OUTPUT_BITDEPTH_DIV8>
static void md5_block(MD5& md5, const Pel* plane, uint32_t n)
{
  /* create a 64 byte buffer for packing Pel's into */
  uint8_t buf[64/OUTPUT_BITDEPTH_DIV8][OUTPUT_BITDEPTH_DIV8];
  for (uint32_t i = 0; i < n; i++)
  {
    Pel pel = plane[i];
    /* perform bitdepth and endian conversion */
    for (uint32_t d = 0; d < OUTPUT_BITDEPTH_DIV8; d++)
    {
      buf[i][d] = pel >> (d*8);
    }
  }
  md5.update((uint8_t*)buf, n * OUTPUT_BITDEPTH_DIV8);
}

/**
 * Update md5 with all samples in plane in raster order, each sample
 * is adjusted to OUTBIT_BITDEPTH_DIV8.
 */
template<uint32_t OUTPUT_BITDEPTH_DIV8>
static void md5_plane(MD5& md5, const Pel* plane, uint32_t width, uint32_t height, uint32_t stride)
{
  /* N is the number of samples to process per md5 update.
   * All N samples must fit in buf */
  uint32_t N = 32;
  uint32_t width_modN = width % N;
  uint32_t width_less_modN = width - width_modN;

  for (uint32_t y = 0; y < height; y++)
  {
    /* convert pels into unsigned chars in little endian byte order.
     * NB, for 8bit data, data is truncated to 8bits. */
    for (uint32_t x = 0; x < width_less_modN; x += N)
    {
      md5_block<OUTPUT_BITDEPTH_DIV8>(md5, &plane[y*stride + x], N);
    }

    /* mop up any of the remaining line */
    md5_block<OUTPUT_BITDEPTH_DIV8>(md5, &plane[y*stride + width_less_modN], width_modN);
  }
}


uint32_t compCRC(int bitdepth, const Pel* plane, uint32_t width, uint32_t height, uint32_t stride, PictureHash &digest)
{
  uint32_t crcMsb;
  uint32_t bitVal;
  uint32_t crcVal = 0xffff;
  uint32_t bitIdx;
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      // take CRC of first pictureData byte
      for(bitIdx=0; bitIdx<8; bitIdx++)
      {
        crcMsb = (crcVal >> 15) & 1;
        bitVal = (plane[y*stride+x] >> (7 - bitIdx)) & 1;
        crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
      }
      // take CRC of second pictureData byte if bit depth is greater than 8-bits
      if(bitdepth > 8)
      {
        for(bitIdx=0; bitIdx<8; bitIdx++)
        {
          crcMsb = (crcVal >> 15) & 1;
          bitVal = (plane[y*stride+x] >> (15 - bitIdx)) & 1;
          crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
        }
      }
    }
  }
  for(bitIdx=0; bitIdx<16; bitIdx++)
  {
    crcMsb = (crcVal >> 15) & 1;
    crcVal = ((crcVal << 1) & 0xffff) ^ (crcMsb * 0x1021);
  }

  digest.hash.push_back((crcVal>>8)  & 0xff);
  digest.hash.push_back( crcVal      & 0xff);
  return 2;
}

uint32_t calcCRC(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths)
{
  uint32_t digestLen=0;
  digest.hash.clear();
  for (uint32_t chan = 0; chan< (uint32_t)pic.bufs.size(); chan++)
  {
    const ComponentID compID = ComponentID(chan);
    const CPelBuf area = pic.get(compID);
    digestLen = compCRC(bitDepths[toChannelType(compID)], area.bufAt(0, 0), area.width, area.height, area.stride, digest );
  }
  return digestLen;
}

uint32_t compChecksum(int bitdepth, const Pel* plane, uint32_t width, uint32_t height, uint32_t stride, PictureHash &digest, const BitDepths &/*bitDepths*/)
{
  uint32_t checksum = 0;
  uint8_t xor_mask;

  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      xor_mask = (x & 0xff) ^ (y & 0xff) ^ (x >> 8) ^ (y >> 8);
      checksum = (checksum + ((plane[y*stride+x] & 0xff) ^ xor_mask)) & 0xffffffff;

      if(bitdepth > 8)
      {
        checksum = (checksum + ((plane[y*stride+x]>>8) ^ xor_mask)) & 0xffffffff;
      }
    }
  }

  digest.hash.push_back((checksum>>24) & 0xff);
  digest.hash.push_back((checksum>>16) & 0xff);
  digest.hash.push_back((checksum>>8)  & 0xff);
  digest.hash.push_back( checksum      & 0xff);
  return 4;
}

uint32_t calcChecksum(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths)
{
  uint32_t digestLen=0;
  digest.hash.clear();
  for(uint32_t chan=0; chan< (uint32_t)pic.bufs.size(); chan++)
  {
    const ComponentID compID=ComponentID(chan);
    const CPelBuf area = pic.get(compID);
    digestLen=compChecksum(bitDepths[toChannelType(compID)], area.bufAt(0,0), area.width, area.height, area.stride, digest, bitDepths);
  }
  return digestLen;
}
/**
 * Calculate the MD5sum of pic, storing the result in digest.
 * MD5 calculation is performed on Y' then Cb, then Cr; each in raster order.
 * Pel data is inserted into the MD5 function in little-endian byte order,
 * using sufficient bytes to represent the picture bitdepth.  Eg, 10bit data
 * uses little-endian two byte words; 8bit data uses single byte words.
 */
uint32_t calcMD5(const CPelUnitBuf& pic, PictureHash &digest, const BitDepths &bitDepths)
{
  /* choose an md5_plane packing function based on the system bitdepth */
  typedef void (*MD5PlaneFunc)(MD5&, const Pel*, uint32_t, uint32_t, uint32_t);
  MD5PlaneFunc md5_plane_func;

  MD5 md5[MAX_NUM_COMP];

  digest.hash.clear();

  for (uint32_t chan = 0; chan< (uint32_t)pic.bufs.size(); chan++)
  {
    const ComponentID compID=ComponentID(chan);
    const CPelBuf area = pic.get(compID);
    md5_plane_func = bitDepths[toChannelType(compID)] <= 8 ? (MD5PlaneFunc)md5_plane<1> : (MD5PlaneFunc)md5_plane<2>;
    uint8_t tmp_digest[MD5_DIGEST_STRING_LENGTH];
    md5_plane_func(md5[compID], area.bufAt(0, 0), area.width, area.height, area.stride );
    md5[compID].finalize(tmp_digest);
    for(uint32_t i=0; i<MD5_DIGEST_STRING_LENGTH; i++)
    {
      digest.hash.push_back(tmp_digest[i]);
    }
  }
  return 16;
}

std::string hashToString(const PictureHash &digest, int numChar)
{
  static const char* hex = "0123456789abcdef";
  std::string result;

  for(int pos=0; pos<int(digest.hash.size()); pos++)
  {
    if ((pos % numChar) == 0 && pos!=0 )
    {
      result += ',';
    }
    result += hex[digest.hash[pos] >> 4];
    result += hex[digest.hash[pos] & 0xf];
  }

  return result;
}

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl)
{
  /* calculate MD5sum for entire reconstructed picture */
  PictureHash recon_digest;
  int numChar=0;
  const char* hashType = "\0";

  if (pictureHashSEI)
  {
    switch (pictureHashSEI->method)
    {
      case HASHTYPE_MD5:
        {
          hashType = "MD5";
          numChar = calcMD5(pic, recon_digest, bitDepths);
          break;
        }
      case HASHTYPE_CRC:
        {
          hashType = "CRC";
          numChar = calcCRC(pic, recon_digest, bitDepths);
          break;
        }
      case HASHTYPE_CHECKSUM:
        {
          hashType = "Checksum";
          numChar = calcChecksum(pic, recon_digest, bitDepths);
          break;
        }
      default:
        {
          THROW("Unknown hash type");
          break;
        }
    }
  }

  /* compare digest against received version */
  const char* ok = "(unk)";
  bool mismatch = false;

  if (pictureHashSEI)
  {
    ok = "(OK)";
    if (recon_digest != pictureHashSEI->m_pictureHash)
    {
      ok = "(***ERROR***)";
      mismatch = true;
    }
  }

  msg( msgl, "[%s:%s,%s] ", hashType, hashToString(recon_digest, numChar).c_str(), ok);

  if (mismatch)
  {
    msg( msgl, "[rx%s:%s] ", hashType, hashToString(pictureHashSEI->m_pictureHash, numChar).c_str());
  }
  return mismatch;
}

} // namespace vvenc

//! \}


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
/** \file     Analyze.h
    \brief    encoder analyzer class (header)
*/

#pragma once

#include "CommonLib/CommonDef.h"

#include <stdio.h>
#include <memory.h>
#include <assert.h>
#include <cinttypes>
#include "math.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder analyzer class
class Analyze
{
private:
  double    m_dPSNRSum[MAX_NUM_COMP];
  double    m_dAddBits;
  uint32_t  m_uiNumPic;
  double    m_dFrmRate; //--CFG_KDY
  double    m_MSEyuvframe[MAX_NUM_COMP]; // sum of MSEs

public:
  virtual ~Analyze()  {}
  Analyze() { clear(); }

  void  addResult( double psnr[MAX_NUM_COMP], double bits, const double MSEyuvframe[MAX_NUM_COMP]
    , bool isEncodeLtRef
  )
  {
    m_dAddBits  += bits;
    if (isEncodeLtRef)
      return;
    for(uint32_t i=0; i<MAX_NUM_COMP; i++)
    {
      m_dPSNRSum[i] += psnr[i];
      m_MSEyuvframe[i] += MSEyuvframe[i];
    }

    m_uiNumPic++;
  }
  double  getPsnr(ComponentID compID) const { return  m_dPSNRSum[compID];  }
  double  getBits()                   const { return  m_dAddBits;   }
  void    setBits(double numBits)     { m_dAddBits = numBits; }
  uint32_t    getNumPic()                 const { return  m_uiNumPic;   }

  void    setFrmRate  (double dFrameRate) { m_dFrmRate = dFrameRate; } //--CFG_KDY
  void    clear()
  {
    m_dAddBits = 0;
    for(uint32_t i=0; i<MAX_NUM_COMP; i++)
    {
      m_dPSNRSum[i] = 0;
      m_MSEyuvframe[i] = 0;
    }
    m_uiNumPic = 0;
  }


  void calculateCombinedValues(const ChromaFormat chFmt, double &PSNRyuv, double &MSEyuv, const BitDepths &bitDepths)
  {
    MSEyuv    = 0;
    int scale = 0;

    int maximumBitDepth = bitDepths[CH_L];
    for (uint32_t ch = 1; ch < MAX_NUM_CH; ch++)
    {
      if (bitDepths[ChannelType(ch)] > maximumBitDepth)
      {
        maximumBitDepth = bitDepths[ChannelType(ch)];
      }
    }

    const uint32_t maxval                = 255 << (maximumBitDepth - 8);
    const uint32_t numberValidComponents = getNumberValidComponents(chFmt);

    for (uint32_t comp=0; comp<numberValidComponents; comp++)
    {
      const ComponentID compID        = ComponentID(comp);
      const uint32_t        csx           = getComponentScaleX(compID, chFmt);
      const uint32_t        csy           = getComponentScaleY(compID, chFmt);
      const int         scaleChan     = (4>>(csx+csy));
      const uint32_t        bitDepthShift = 2 * (maximumBitDepth - bitDepths[toChannelType(compID)]); //*2 because this is a squared number

      const double      channelMSE    = (m_MSEyuvframe[compID] * double(1 << bitDepthShift)) / double(getNumPic());

      scale  += scaleChan;
      MSEyuv += scaleChan * channelMSE;
    }

    MSEyuv /= double(scale);  // i.e. divide by 6 for 4:2:0, 8 for 4:2:2 etc.
    PSNRyuv = (MSEyuv == 0) ? 999.99 : 10.0 * log10((maxval * maxval) / MSEyuv);
  }

  std::string printOut ( char cDelim, const ChromaFormat chFmt, const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr, const BitDepths &bitDepths )
  {
    double dFps     =   m_dFrmRate; //--CFG_KDY
    double dScale   = dFps / 1000 / (double)m_uiNumPic;
    std::string info;

    double MSEBasedSNR[MAX_NUM_COMP];
    if (printMSEBasedSNR)
    {
      for (uint32_t componentIndex = 0; componentIndex < MAX_NUM_COMP; componentIndex++)
      {
        const ComponentID compID = ComponentID(componentIndex);

        if (getNumPic() == 0)
        {
          MSEBasedSNR[compID] = 0 * dScale; // this is the same calculation that will be evaluated for any other statistic when there are no frames (it should result in NaN). We use it here so all the output is consistent.
        }
        else
        {
          const uint32_t maxval = 255 << (bitDepths[toChannelType(compID)] - 8); // fix with WPSNR: 1023 (4095) instead of 1020 (4080) for bit depth 10 (12)
          const double MSE  = m_MSEyuvframe[compID];

          MSEBasedSNR[compID] = (MSE == 0) ? 999.99 : 10.0 * log10((maxval * maxval) / (MSE / (double)getNumPic()));
        }
      }
    }

    switch (chFmt)
    {
      case CHROMA_400:
        if (printMSEBasedSNR)
        {
          info.append(prnt("         \tTotal Frames |   "   "Bitrate     "  "Y-PSNR" ) );
          
          if (printHexPsnr)
          {
            info.append(prnt("xY-PSNR           " ));
          }

          if (printSequenceMSE)
          {
            info.append(prnt("    Y-MSE\n" ));
          }
          else
          {
            info.append(prnt("\n" ));
          }

          //info.append(prnt("\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" ));
           info.append(prnt("Average: \t %8d    %c "          "%12.4lf  "    "%8.4lf",
                 getNumPic(), cDelim,
                 getBits() * dScale,
                 getPsnr(COMP_Y) / (double)getNumPic() ));

          if (printHexPsnr)
          {
            double dPsnr;
            uint64_t xPsnr;
            dPsnr = getPsnr(COMP_Y) / (double)getNumPic();

            std::copy(reinterpret_cast<uint8_t *>(&dPsnr),
              reinterpret_cast<uint8_t *>(&dPsnr) + sizeof(dPsnr),
              reinterpret_cast<uint8_t *>(&xPsnr));

            info.append(prnt( "   %16" PRIx64 " ", xPsnr ));
          }

          if (printSequenceMSE)
          {
            info.append(prnt("  %8.4lf\n", m_MSEyuvframe[COMP_Y] / (double)getNumPic() ));
          }
          else
          {
            info.append(prnt("\n"));
          }

          info.append(prnt("From MSE:\t %8d    %c "          "%12.4lf  "    "%8.4lf\n",
                 getNumPic(), cDelim,
                 getBits() * dScale,
                 MSEBasedSNR[COMP_Y] ));
        }
        else
        {
          info.append(prnt("\tTotal Frames |   "   "Bitrate     "  "Y-PSNR" ));

          if (printHexPsnr)
          {
            info.append(prnt("xY-PSNR           "));
          }

          if (printSequenceMSE)
          {
            info.append(prnt("    Y-MSE\n" ));
          }
          else
          {
            info.append(prnt("\n"));
          }

          //info.append(prnt("\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" ));
          info.append(prnt("\t %8d    %c "          "%12.4lf  "    "%8.4lf",
                 getNumPic(), cDelim,
                 getBits() * dScale,
                 getPsnr(COMP_Y) / (double)getNumPic() ));

          if (printHexPsnr)
          {
            double dPsnr;
            uint64_t xPsnr;
            dPsnr = getPsnr(COMP_Y) / (double)getNumPic();

            std::copy(reinterpret_cast<uint8_t *>(&dPsnr),
              reinterpret_cast<uint8_t *>(&dPsnr) + sizeof(dPsnr),
              reinterpret_cast<uint8_t *>(&xPsnr));

            info.append(prnt("   %16" PRIx64 " ", xPsnr));
          }

          if (printSequenceMSE)
          {
            info.append(prnt("  %8.4lf\n", m_MSEyuvframe[COMP_Y] / (double)getNumPic() ));
          }
          else
          {
            info.append(prnt("\n"));
          }
        }
        break;
      case CHROMA_420:
      case CHROMA_422:
      case CHROMA_444:
        {
          double PSNRyuv = MAX_DOUBLE;
          double MSEyuv  = MAX_DOUBLE;

          calculateCombinedValues(chFmt, PSNRyuv, MSEyuv, bitDepths);

          if (printMSEBasedSNR)
          {
            info.append(prnt("         \tTotal Frames |   "   "Bitrate     "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR    "  "YUV-PSNR " ));

            if (printHexPsnr)
            {
              info.append(prnt("xY-PSNR           "  "xU-PSNR           "  "xV-PSNR           "));
            }

            if (printSequenceMSE)
            {
              info.append(prnt(" Y-MSE     "  "U-MSE     "  "V-MSE    "  "YUV-MSE \n" ));
            }
            else
            {
              info.append(prnt("\n"));
            }

            //info.append(prnt("\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" ));
            info.append(prnt("Average: \t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf",
                   getNumPic(), cDelim,
                   getBits() * dScale,
                   getPsnr(COMP_Y ) / (double)getNumPic(),
                   getPsnr(COMP_Cb) / (double)getNumPic(),
                   getPsnr(COMP_Cr) / (double)getNumPic(),
                   PSNRyuv ));

            if (printHexPsnr)
            {
              double dPsnr[MAX_NUM_COMP];
              uint64_t xPsnr[MAX_NUM_COMP];
              for (int i = 0; i < MAX_NUM_COMP; i++)
              {
                dPsnr[i] = getPsnr((ComponentID)i) / (double)getNumPic();

                std::copy(reinterpret_cast<uint8_t *>(&dPsnr[i]),
                  reinterpret_cast<uint8_t *>(&dPsnr[i]) + sizeof(dPsnr[i]),
                  reinterpret_cast<uint8_t *>(&xPsnr[i]));
              }
              info.append(prnt("   %16" PRIx64 "  %16" PRIx64 "  %16" PRIx64, xPsnr[COMP_Y], xPsnr[COMP_Cb], xPsnr[COMP_Cr]));
            }

            if (printSequenceMSE)
            {
              info.append(prnt("  %8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                     m_MSEyuvframe[COMP_Y ] / (double)getNumPic(),
                     m_MSEyuvframe[COMP_Cb] / (double)getNumPic(),
                     m_MSEyuvframe[COMP_Cr] / (double)getNumPic(),
                     MSEyuv ));
            }
            else
            {
              info.append(prnt("\n"));
            }

            info.append(prnt("From MSE:\t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                   getNumPic(), cDelim,
                   getBits() * dScale,
                   MSEBasedSNR[COMP_Y ],
                   MSEBasedSNR[COMP_Cb],
                   MSEBasedSNR[COMP_Cr],
                   PSNRyuv ));
          }
          else
          {
            info.append(prnt("\tTotal Frames |   "   "Bitrate     "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR    "  "YUV-PSNR   " ));

            if (printHexPsnr)
            {
              info.append(prnt("xY-PSNR           "  "xU-PSNR           "  "xV-PSNR           "));
            }
            if (printSequenceMSE)
            {
              info.append(prnt(" Y-MSE     "  "U-MSE     "  "V-MSE    "  "YUV-MSE \n" ));
            }
            else
            {
              info.append(prnt("\n"));
            }

            //info.append(prnt("\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" ));
            info.append(prnt("\t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf",
                   getNumPic(), cDelim,
                   getBits() * dScale,
                   getPsnr(COMP_Y ) / (double)getNumPic(),
                   getPsnr(COMP_Cb) / (double)getNumPic(),
                   getPsnr(COMP_Cr) / (double)getNumPic(),
                   PSNRyuv ));


            if (printHexPsnr)
            {
              double dPsnr[MAX_NUM_COMP];
              uint64_t xPsnr[MAX_NUM_COMP];
              for (int i = 0; i < MAX_NUM_COMP; i++)
              {
                dPsnr[i] = getPsnr((ComponentID)i) / (double)getNumPic();

                std::copy(reinterpret_cast<uint8_t *>(&dPsnr[i]),
                  reinterpret_cast<uint8_t *>(&dPsnr[i]) + sizeof(dPsnr[i]),
                  reinterpret_cast<uint8_t *>(&xPsnr[i]));
              }
              info.append(prnt("   %16" PRIx64 "  %16" PRIx64 "  %16" PRIx64 , xPsnr[COMP_Y], xPsnr[COMP_Cb], xPsnr[COMP_Cr]));
            }
            if (printSequenceMSE)
            {
              info.append(prnt("  %8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                     m_MSEyuvframe[COMP_Y ] / (double)getNumPic(),
                     m_MSEyuvframe[COMP_Cb] / (double)getNumPic(),
                     m_MSEyuvframe[COMP_Cr] / (double)getNumPic(),
                     MSEyuv ));
            }
            else
            {
              info.append(prnt("\n"));
            }
          }
        }
        break;
      default:
        info.append(prnt("Unknown format during print out\n"));
        break;
    }
    return info;
  }


  void    printSummary(const ChromaFormat chFmt, const bool printSequenceMSE, const bool printHexPsnr, const BitDepths &bitDepths, const std::string &sFilename)
  {
    FILE* pFile = fopen (sFilename.c_str(), "at");

    double dFps     =   m_dFrmRate; //--CFG_KDY
    double dScale   = dFps / 1000 / (double)m_uiNumPic;
    switch (chFmt)
    {
      case CHROMA_400:
        fprintf(pFile, "%f\t %f\n",
            getBits() * dScale,
            getPsnr(COMP_Y) / (double)getNumPic() );
        break;
      case CHROMA_420:
      case CHROMA_422:
      case CHROMA_444:
        {
          double PSNRyuv = MAX_DOUBLE;
          double MSEyuv  = MAX_DOUBLE;

          calculateCombinedValues(chFmt, PSNRyuv, MSEyuv, bitDepths);

          fprintf(pFile, "%f\t %f\t %f\t %f\t %f",
              getBits() * dScale,
              getPsnr(COMP_Y ) / (double)getNumPic(),
              getPsnr(COMP_Cb) / (double)getNumPic(),
              getPsnr(COMP_Cr) / (double)getNumPic(),
              PSNRyuv );

          if (printSequenceMSE)
          {
            fprintf(pFile, "\t %f\t %f\t %f\t %f\n",
                m_MSEyuvframe[COMP_Y ] / (double)getNumPic(),
                m_MSEyuvframe[COMP_Cb] / (double)getNumPic(),
                m_MSEyuvframe[COMP_Cr] / (double)getNumPic(),
                MSEyuv );
          }
          else
          {
            fprintf(pFile, "\n");
          }

          break;
        }

      default:
          fprintf(pFile, "Unknown format during print out\n");
          break;
    }

    fclose(pFile);
  }
};

} // namespace vvenc

//! \}


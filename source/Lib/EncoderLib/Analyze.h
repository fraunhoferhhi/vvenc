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

  void    printOut ( char cDelim, const ChromaFormat chFmt, const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr, const BitDepths &bitDepths )
  {
    MsgLevel e_msg_level = cDelim == 'a' ? INFO: DETAILS;
    double dFps     =   m_dFrmRate; //--CFG_KDY
    double dScale   = dFps / 1000 / (double)m_uiNumPic;

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
          msg( e_msg_level, "         \tTotal Frames |   "   "Bitrate     "  "Y-PSNR" );

          if (printHexPsnr)
          {
            msg(e_msg_level, "xY-PSNR           ");
          }

          if (printSequenceMSE)
          {
            msg( e_msg_level, "    Y-MSE\n" );
          }
          else
          {
            msg( e_msg_level, "\n");
          }

          //msg( e_msg_level, "\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" );
          msg( e_msg_level, "Average: \t %8d    %c "          "%12.4lf  "    "%8.4lf",
                 getNumPic(), cDelim,
                 getBits() * dScale,
                 getPsnr(COMP_Y) / (double)getNumPic() );

          if (printHexPsnr)
          {
            double dPsnr;
            uint64_t xPsnr;
            dPsnr = getPsnr(COMP_Y) / (double)getNumPic();

            std::copy(reinterpret_cast<uint8_t *>(&dPsnr),
              reinterpret_cast<uint8_t *>(&dPsnr) + sizeof(dPsnr),
              reinterpret_cast<uint8_t *>(&xPsnr));

            msg(e_msg_level, "   %16" PRIx64 " ", xPsnr);
          }

          if (printSequenceMSE)
          {
            msg( e_msg_level, "  %8.4lf\n", m_MSEyuvframe[COMP_Y] / (double)getNumPic() );
          }
          else
          {
            msg( e_msg_level, "\n");
          }

          msg( e_msg_level, "From MSE:\t %8d    %c "          "%12.4lf  "    "%8.4lf\n",
                 getNumPic(), cDelim,
                 getBits() * dScale,
                 MSEBasedSNR[COMP_Y] );
        }
        else
        {
          msg( e_msg_level, "\tTotal Frames |   "   "Bitrate     "  "Y-PSNR" );

          if (printHexPsnr)
          {
            msg(e_msg_level, "xY-PSNR           ");
          }

          if (printSequenceMSE)
          {
            msg( e_msg_level, "    Y-MSE\n" );
          }
          else
          {
            msg( e_msg_level, "\n");
          }

          //msg( e_msg_level, "\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" );
          msg( e_msg_level, "\t %8d    %c "          "%12.4lf  "    "%8.4lf",
                 getNumPic(), cDelim,
                 getBits() * dScale,
                 getPsnr(COMP_Y) / (double)getNumPic() );

          if (printHexPsnr)
          {
            double dPsnr;
            uint64_t xPsnr;
            dPsnr = getPsnr(COMP_Y) / (double)getNumPic();

            std::copy(reinterpret_cast<uint8_t *>(&dPsnr),
              reinterpret_cast<uint8_t *>(&dPsnr) + sizeof(dPsnr),
              reinterpret_cast<uint8_t *>(&xPsnr));

            msg(e_msg_level, "   %16" PRIx64 " ", xPsnr);
          }

          if (printSequenceMSE)
          {
            msg( e_msg_level, "  %8.4lf\n", m_MSEyuvframe[COMP_Y] / (double)getNumPic() );
          }
          else
          {
            msg( e_msg_level, "\n");
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
            msg( e_msg_level, "         \tTotal Frames |   "   "Bitrate     "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR    "  "YUV-PSNR " );

            if (printHexPsnr)
            {
              msg(e_msg_level, "xY-PSNR           "  "xU-PSNR           "  "xV-PSNR           ");
            }

            if (printSequenceMSE)
            {
              msg( e_msg_level, " Y-MSE     "  "U-MSE     "  "V-MSE    "  "YUV-MSE \n" );
            }
            else
            {
              msg( e_msg_level, "\n");
            }

            //msg( e_msg_level, "\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" );
            msg( e_msg_level, "Average: \t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf",
                   getNumPic(), cDelim,
                   getBits() * dScale,
                   getPsnr(COMP_Y ) / (double)getNumPic(),
                   getPsnr(COMP_Cb) / (double)getNumPic(),
                   getPsnr(COMP_Cr) / (double)getNumPic(),
                   PSNRyuv );

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
              msg(e_msg_level, "   %16" PRIx64 "  %16" PRIx64 "  %16" PRIx64, xPsnr[COMP_Y], xPsnr[COMP_Cb], xPsnr[COMP_Cr]);
            }

            if (printSequenceMSE)
            {
              msg( e_msg_level, "  %8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                     m_MSEyuvframe[COMP_Y ] / (double)getNumPic(),
                     m_MSEyuvframe[COMP_Cb] / (double)getNumPic(),
                     m_MSEyuvframe[COMP_Cr] / (double)getNumPic(),
                     MSEyuv );
            }
            else
            {
              msg( e_msg_level, "\n");
            }

            msg( e_msg_level, "From MSE:\t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                   getNumPic(), cDelim,
                   getBits() * dScale,
                   MSEBasedSNR[COMP_Y ],
                   MSEBasedSNR[COMP_Cb],
                   MSEBasedSNR[COMP_Cr],
                   PSNRyuv );
          }
          else
          {
            msg( e_msg_level, "\tTotal Frames |   "   "Bitrate     "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR    "  "YUV-PSNR   " );

            if (printHexPsnr)
            {
              msg(e_msg_level, "xY-PSNR           "  "xU-PSNR           "  "xV-PSNR           ");
            }
            if (printSequenceMSE)
            {
              msg( e_msg_level, " Y-MSE     "  "U-MSE     "  "V-MSE    "  "YUV-MSE \n" );
            }
            else
            {
              msg( e_msg_level, "\n");
            }

            //msg( e_msg_level, "\t------------ "  " ----------"   " -------- "  " -------- "  " --------\n" );
            msg( e_msg_level, "\t %8d    %c "          "%12.4lf  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf",
                   getNumPic(), cDelim,
                   getBits() * dScale,
                   getPsnr(COMP_Y ) / (double)getNumPic(),
                   getPsnr(COMP_Cb) / (double)getNumPic(),
                   getPsnr(COMP_Cr) / (double)getNumPic(),
                   PSNRyuv );


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
              msg(e_msg_level, "   %16" PRIx64 "  %16" PRIx64 "  %16" PRIx64 , xPsnr[COMP_Y], xPsnr[COMP_Cb], xPsnr[COMP_Cr]);
            }
            if (printSequenceMSE)
            {
              msg( e_msg_level, "  %8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
                     m_MSEyuvframe[COMP_Y ] / (double)getNumPic(),
                     m_MSEyuvframe[COMP_Cb] / (double)getNumPic(),
                     m_MSEyuvframe[COMP_Cr] / (double)getNumPic(),
                     MSEyuv );
            }
            else
            {
              msg( e_msg_level, "\n");
            }
          }
        }
        break;
      default:
        msg( ERROR, "Unknown format during print out\n");
        exit(1);
        break;
    }
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
          msg( ERROR, "Unknown format during print out\n");
          exit(1);
          break;
    }

    fclose(pFile);
  }
};

} // namespace vvenc

//! \}


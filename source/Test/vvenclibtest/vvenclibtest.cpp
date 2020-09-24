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

/**
  \ingroup vvenclibtest
  \file    vvenclibtest.cpp
  \brief   This vvenclibtest.cpp file contains the main entry point of the test application.
*/

#include <iostream>

#include <stdio.h>
#include <string>
#include <fstream>
#include <cstring>

#include "vvenc/version.h"
#include "vvenc/vvenc.h"

#define TEST(x)   { int res = x; g_numTests++; g_numFails += res;  if( res && g_verbose ) std::cerr << "\nFail: In function \""  << __FUNCTION__ << "\" ln " <<  __LINE__; }
#define ERROR(s)  { g_numTests++; g_numFails ++;  if( g_verbose ) std::cerr << "\n" << s << " Fail: In function \""  << __FUNCTION__ << "\" ln " <<  __LINE__; }
int g_numTests = 0; 
int g_numFails = 0;
int g_verbose = 0;

void fillEncoderParameters( vvenc::VVEncParameter& cVVEncParameter );

int testLibCallingOrder();     // check invalid caling order
int testLibParameterRanges();  // single parameter rang 
int testInvalidInputParams();  //  input Buffer does not match
int testInvalidOutputParams(); //  AUBuffer to small


void fillEncoderParameters( vvenc::VVEncParameter& cVVEncParameter )
{
  cVVEncParameter.m_iQp             = 32;                         // quantization parameter 0-51
  cVVEncParameter.m_iWidth          = 1920;                       // luminance width of input picture
  cVVEncParameter.m_iHeight         = 1080;                       // luminance height of input picture
  cVVEncParameter.m_iGopSize        = 16;                         //  gop size (1: intra only, 16: hierarchical b frames)
  cVVEncParameter.m_eDecodingRefreshType = vvenc::VVC_DRT_CRA;    // intra period refresh type
  cVVEncParameter.m_iIDRPeriod      = 32;                         // intra period for IDR/CDR intra refresh/RAP flag (should be a factor of m_iGopSize)
  cVVEncParameter.m_eLogLevel       = vvenc::LL_SILENT;          // log level > 4 (VERBOSE) enables psnr/rate output
  cVVEncParameter.m_iTemporalRate   = 60;                         // temporal rate (fps)
  cVVEncParameter.m_iTemporalScale  = 1;                          // temporal scale (fps)
  cVVEncParameter.m_iTicksPerSecond = 90000;                      // ticks per second e.g. 90000 for dts generation
  cVVEncParameter.m_iThreadCount    = 4;                          // number of worker threads (should not exceed the number of physical cpu's)
  cVVEncParameter.m_iQuality        = 2;                          // encoding quality (vs speed) 0: faster, 1: fast, 2: medium, 3: slow
  cVVEncParameter.m_iPerceptualQPA  = 2;                          // percepual qpa adaption, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  cVVEncParameter.m_eProfile        = vvenc::VVC_PROFILE_MAIN_10; // profile: use main_10 or main_10_still_picture
  cVVEncParameter.m_eLevel          = vvenc::VVC_LEVEL_4_1;       // level
  cVVEncParameter.m_eTier           = vvenc::VVC_TIER_MAIN;       // tier
}

template< typename T >
int testParameter( vvenc::VVEncParameter& vvencParams, T& testPara, const std::vector<std::pair<T,bool> >& expValues)
{
  vvenc::VVEnc cVVEnc;
  fillEncoderParameters( vvencParams );
  const int numFails = g_numFails;

  for( auto &test : expValues )
  {
    testPara = test.first;
    try
    {
  //    vvenc::VVEnc cVVEnc;
      // initialize the encoder
      TEST( (test.second == (0 == cVVEnc.checkConfig( vvencParams ))) ? 0 : 1);
    }
    catch(...)
    {
      ERROR("Caught Exception"); //fail due to exception 
    }
  }
  return numFails == g_numFails ? 0 : 1;
}

typedef std::vector<std::pair<int,bool> > intpar;
typedef std::vector<std::pair<int,bool> > boolpar;

int testLibParameterRanges()
{
  vvenc::VVEncParameter vvencParams;
  TEST( testParameter( vvencParams, (int&)vvencParams.m_eDecodingRefreshType, intpar{ {-1,0},{0,1},{1,0},{2,0},{3,0},{4,0} } ));
  TEST( testParameter( vvencParams, (int&)vvencParams.m_eLevel,               intpar{ {-1,0},{0,0},{15,0},{16,1},{17,0},{31,0},{32,1},{35,1},{48,1},{51,1},{64,1},{67,1},{80,1},{83,1},{86,1},{96,1},{99,1},{102,1},{255,1},{256,0} } ));
  TEST( testParameter( vvencParams, (int&)vvencParams.m_eLogLevel,            intpar{ {-1,0},{0,1},{1,1},{6,1} } ));
  return 0;
}


int main( /*int argc, char* argv[]*/ )
{
  g_numTests = 0; 
  g_numFails = 0;
  g_verbose = 1;

  TEST( testLibParameterRanges());
  
  if( g_numTests == 0 )
  {
    std::cerr << "\n\n no test available";
    return -1;
  }

  std::cerr << "\n\n" << g_numFails << " out of " << g_numTests << " tests failed"; 
  return g_numFails;
}

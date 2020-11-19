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

#define TEST(x)     { int res = x; g_numTests++; g_numFails += res;  if( g_verbose ) if(res) { std::cerr << "\n test failed: In function "  << __FUNCTION__ << "\" ln " <<  __LINE__;} }
#define TESTT(x,w)  { int res = x; g_numTests++; g_numFails += res;  if( g_verbose ) if(res) { std::cerr << "\n" << w << "\n test failed: In function "  << __FUNCTION__ << "\" ln " <<  __LINE__;} }
#define ERROR(w)    { g_numTests++; g_numFails ++;  if( g_verbose ) std::cerr << "\n" << w << " test failed: In function "  << __FUNCTION__ << "\" ln " <<  __LINE__; }

int g_numTests = 0; 
int g_numFails = 0;
int g_verbose = 0;

int testLibCallingOrder();     // check invalid caling order
int testLibParameterRanges();  // single parameter rangewew checks 
int testInvalidInputParams();  // input Buffer does not match
int testInvalidOutputParams(); // AUBuffer to small


int main( int argc, char* argv[] )
{
  int testId = -1;
  if( argc > 1 )
  {
    bool printHelp = false;
    if( 0 == strcmp( argv[1] ,"-h") || 0  == strcmp( argv[1], "--help"))
    {
      printHelp = true;
    }
    else
    {
      testId = atoi(argv[1]);
      printHelp = ( testId < 1 || testId > 4 );
    }

    if( printHelp )
    {
      printf( "venclibtest <test> [1..4]\n");
      return -1;
    }
  }

  g_numTests = 0; 
  g_numFails = 0;
  g_verbose = 1;

  switch( testId )
  {
  case 1:
  {
    testLibParameterRanges(); 
    break;
  }
  case 2: 
  {
    testLibCallingOrder(); 
    break;
  }
  case 3: 
  {
    testInvalidInputParams(); 
    break;
  }
  case 4: 
  {
    testInvalidOutputParams(); 
    break;
  }
  default:
    testLibParameterRanges();
    testLibCallingOrder();
    testInvalidInputParams();
    testInvalidOutputParams();
    break;
  }

  if( g_numTests == 0 )
  {
    std::cerr << "\n\n no test available";
    return -1;
  }

  if( g_numFails == 0 )
  {
    std::cerr << "\n\n all of " << g_numTests << " tests succeeded"; 
  }
  else
  {
    std::cerr << "\n\n" << g_numFails << " out of " << g_numTests << " tests failed"; 
  }
  return g_numFails;
}


void fillEncoderParameters( vvenc::VVEncParameter& cVVEncParameter )
{
  cVVEncParameter.m_iQp               = 32;                         // quantization parameter 0-51
  cVVEncParameter.m_iWidth            = 176;                        // luminance width of input picture
  cVVEncParameter.m_iHeight           = 144;                        // luminance height of input picture
  cVVEncParameter.m_iGopSize          = 16;                         // gop size (1: intra only, 16, 32: hierarchical b frames)
  cVVEncParameter.m_eDecodingRefreshType = vvenc::VVC_DRT_CRA;      // intra period refresh type
  cVVEncParameter.m_iIDRPeriod        = 32;                         // intra period for IDR/CDR intra refresh/RAP flag (should be a factor of m_iGopSize)
  cVVEncParameter.m_eLogLevel         = vvenc::LL_SILENT;           // log level > 4 (VERBOSE) enables psnr/rate output
  cVVEncParameter.m_iTemporalRate     = 60;                         // temporal rate (fps)
  cVVEncParameter.m_iTemporalScale    = 1;                          // temporal scale (fps)
  cVVEncParameter.m_iTicksPerSecond   = 90000;                      // ticks per second e.g. 90000 for dts generation
  cVVEncParameter.m_iThreadCount      = 0;                          // number of worker threads (should not exceed the number of physical cpu's)
  cVVEncParameter.m_iQuality          = 0;                          // encoding quality (vs speed) 0: faster, 1: fast, 2: medium, 3: slow
  cVVEncParameter.m_iPerceptualQPA    = 2;                          // percepual qpa adaption, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  cVVEncParameter.m_iInputBitDepth    = 8;                          // 8bit input
  cVVEncParameter.m_iInternalBitDepth = 10;                         // 10bit internal
  cVVEncParameter.m_eProfile          = vvenc::VVC_PROFILE_MAIN_10; // profile: use main_10 or main_10_still_picture
  cVVEncParameter.m_eLevel            = vvenc::VVC_LEVEL_4_1;       // level
  cVVEncParameter.m_eTier             = vvenc::VVC_TIER_MAIN;       // tier
  cVVEncParameter.m_bAccessUnitDelimiter       = false;
  cVVEncParameter.m_bHrdParametersPresent      = false;
  cVVEncParameter.m_bBufferingPeriodSEIEnabled = false;
  cVVEncParameter.m_bPictureTimingSEIEnabled   = false;
}

void fillInputPic( vvenc::InputPicture& cInputPic )
{
  cInputPic.m_pcPicAttributes = nullptr;
  const short val = 512;
  int lumaSize   = cInputPic.m_cPicBuffer.m_iHeight   * cInputPic.m_cPicBuffer.m_iStride;
  int chromaSize = ( cInputPic.m_cPicBuffer.m_iCStride ) ? (cInputPic.m_cPicBuffer.m_iHeight/2 * cInputPic.m_cPicBuffer.m_iCStride) : (lumaSize / 4);
  std::fill_n( static_cast<short*> (cInputPic.m_cPicBuffer.m_pvY), lumaSize, val );
  std::fill_n( static_cast<short*> (cInputPic.m_cPicBuffer.m_pvU), chromaSize, val );
  std::fill_n( static_cast<short*> (cInputPic.m_cPicBuffer.m_pvV), chromaSize, val );
}

template< typename T >
int testParamList( const std::string& w, T& testParam, vvenc::VVEncParameter& vvencParams, const std::vector<int>& testValues, const bool expectedFail = false )
{
  vvenc::VVEnc cVVEnc;
  const int numFails = g_numFails;
  const T savedTestParam = testParam;

  for( auto &testVal : testValues )
  {
    testParam = (T)testVal;
    try
    {
      // initialize the encoder
      TESTT( expectedFail == (0 == cVVEnc.checkConfig( vvencParams )),  "\n" << w << "==" << testVal << " expected " << (expectedFail ? "failure" : "success"));
    }
    catch(...)
    {
      ERROR("\nCaught Exception " << w << "==" << testVal << " expected " << (expectedFail ? "failure" : "success")); //fail due to exception 
    }
  }

  //restore original test param
  testParam = savedTestParam; 
  return numFails == g_numFails ? 0 : 1;
}

int testLibParameterRanges()
{
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );

  testParamList( "DecodingRefreshType", vvencParams.m_eDecodingRefreshType,       vvencParams, { 0 } );
  testParamList( "DecodingRefreshType", vvencParams.m_eDecodingRefreshType,       vvencParams, { -1,1,2,3,4}, true );

  testParamList( "Level",               vvencParams.m_eLevel,                     vvencParams, { 16,32,35,48,51,64,67,80,83,86,96,99,102,255} );
  testParamList( "Level",               vvencParams.m_eLevel,                     vvencParams, {-1,0,15,31,256,}, true );

//  testParamList( "LogLevel",            vvencParams.m_eLogLevel,                  vvencParams, { 0,1,2,3,4,5,6,7} );
//  testParamList( "LogLevel",            vvencParams.m_eLogLevel,                  vvencParams, {-1,8,9}, true );

  testParamList( "Profile",             vvencParams.m_eProfile,                   vvencParams, { 1,3,9} );
  testParamList( "Profile",             vvencParams.m_eProfile,                   vvencParams, {-1,0,2,4,5,6,7,8,10}, true );

  testParamList( "Tier",                vvencParams.m_eTier,                      vvencParams, { 0,1} );
  testParamList( "Tier",                vvencParams.m_eTier,                      vvencParams, { -1,2}, true );

  testParamList( "GOPSize",             vvencParams.m_iGopSize,                   vvencParams, { 16,32} );
  testParamList( "GOPSize",             vvencParams.m_iGopSize,                   vvencParams, { 1,8, -1,0,2,3,4,17,33,64,128}, true ); //th is this intended

  testParamList( "Width",               vvencParams.m_iWidth,                     vvencParams, { 320,1920,3840 } );
  testParamList( "Width",               vvencParams.m_iWidth,                     vvencParams, { -1,0 }, true );

  testParamList( "Height",              vvencParams.m_iHeight,                    vvencParams, { 16,32,1080,1088} );
  testParamList( "Height",              vvencParams.m_iHeight,                    vvencParams, { -1,0 }, true );

  testParamList( "IDRPeriod",           vvencParams.m_iIDRPeriod,                 vvencParams, { 16,32,48, 0} );
  testParamList( "IDRPeriod",           vvencParams.m_iIDRPeriod,                 vvencParams, { 1,-1,17,24 }, true );

  testParamList( "PerceptualQPA",       vvencParams.m_iPerceptualQPA,             vvencParams, { 0,1,2,3,4,5} );
  testParamList( "PerceptualQPA",       vvencParams.m_iPerceptualQPA,             vvencParams, { -1,6}, true );

  testParamList( "Qp",                  vvencParams.m_iQp,                        vvencParams, { 0,1,2,3,4,51} );
  testParamList( "Qp",                  vvencParams.m_iQp,                        vvencParams, { -1,52}, true );

  testParamList( "Quality",             vvencParams.m_iQuality,                   vvencParams, { 0,1,2,3} );
  testParamList( "Quality",             vvencParams.m_iQuality,                   vvencParams, { -1,4}, true );

  testParamList( "TargetBitRate",       vvencParams.m_iTargetBitRate,             vvencParams, { 0,1000000,20000000} );
  testParamList( "TargetBitRate",       vvencParams.m_iTargetBitRate,             vvencParams, { -1,100000001}, true );

  testParamList( "InputBitDepth",       vvencParams.m_iInputBitDepth,             vvencParams, { 8,10} );
  testParamList( "InputBitDepth",       vvencParams.m_iInputBitDepth,             vvencParams, { 0,1,7,9,11}, true );

  testParamList( "InternalBitDepth",    vvencParams.m_iInternalBitDepth,          vvencParams, { 8,10} );
  testParamList( "InternalBitDepth",    vvencParams.m_iInternalBitDepth,          vvencParams, { 0,1,7,9,11}, true );

//  testParamList( "TemporalScale",       vvencParams.m_iTemporalScale,             vvencParams, { 1,2,4,1001} );
//  testParamList( "TemporalScale",       vvencParams.m_iTemporalScale,             vvencParams, { -1,0,3,1000 }, true );

  vvencParams.m_iTemporalScale = 1;
  testParamList( "TemporalRate",        vvencParams.m_iTemporalRate,              vvencParams, { 1,25,30,50,60,100,120} );
  testParamList( "TemporalRate",        vvencParams.m_iTemporalRate,              vvencParams, { -1,0/*,24*/}, true );    //th is this intended

  vvencParams.m_iTemporalScale = 1001;
  testParamList( "TemporalRate",        vvencParams.m_iTemporalRate,              vvencParams, { 24000,30000,60000 /*,1200000*/} );
  testParamList( "TemporalRate",        vvencParams.m_iTemporalRate,              vvencParams, { -1,1,0,24}, true );

  fillEncoderParameters( vvencParams );

  testParamList( "ThreadCount",         vvencParams.m_iThreadCount,               vvencParams, { 0,1,2,64} );
  testParamList( "ThreadCount",         vvencParams.m_iThreadCount,               vvencParams, { -1,65 }, true );

  testParamList( "TicksPerSecond",      vvencParams.m_iTicksPerSecond,            vvencParams, { 90000,27000000,60,120 } );
  testParamList( "TicksPerSecond",      vvencParams.m_iTicksPerSecond,            vvencParams, { -1,0, 50, 27000001 }, true );
  
  vvencParams.m_iTargetBitRate = 0;
  testParamList( "HrdParametersPresent",      vvencParams.m_bHrdParametersPresent,          vvencParams, { true }, true );
  testParamList( "BufferingPeriodSEIEnabled", vvencParams.m_bBufferingPeriodSEIEnabled,     vvencParams, { true }, true );
  testParamList( "PictureTimingSEIEnabled",   vvencParams.m_bPictureTimingSEIEnabled,       vvencParams, { true }, true );

  return 0;
}

int testfunc( const std::string& w, int (*funcCallingOrder)(void), const bool expectedFail = false )
{
  const int numFails = g_numFails;

  try
  {
    // initialize the encoder
    TESTT( expectedFail == (0 == funcCallingOrder()),  "\n" << w << " expected " << (expectedFail ? "failure" : "success"));
  }
  catch(...)
  {
    ERROR("\nCaught Exception " << w << " expected " << (expectedFail ? "failure" : "success")); //fail due to exception 
  }

  return numFails == g_numFails ? 0 : 1;
}

int callingOrderInvalidUninit()
{
  vvenc::VVEnc cVVEnc;
  if( 0 != cVVEnc.uninit())
  {
    return -1;
  }
  return 0;
}

int callingOrderInitNoUninit()
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }
  return 0;
}

int callingOrderInitTwice()
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams; // 
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }
  return 0;
}

int callingOrderNoInit()
{
  vvenc::VVEnc cVVEnc;
  vvenc::VvcAccessUnit cAU;
  vvenc::InputPicture cInputPic;
  if( 0 != cVVEnc.encode( &cInputPic, cAU))
  {
    return -1;
  }
  return 0;
}

int callingOrderRegular()
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }
  vvenc::VvcAccessUnit cAU;
  cAU.m_iBufSize  = vvencParams.m_iWidth * vvencParams.m_iHeight;   cAU.m_pucBuffer = new unsigned char [ cAU.m_iBufSize ];

  vvenc::InputPicture cInputPic;
  if( 0 != cVVEnc.getPreferredBuffer( cInputPic.m_cPicBuffer ))
  {
    return -1;
  }
  fillInputPic( cInputPic );
  if( 0 != cVVEnc.encode( &cInputPic, cAU))
  {
    return -1;
  }
  if( 0 != cVVEnc.uninit())
  {
    return -1;
  }
  return 0;
}

int testLibCallingOrder()
{
  testfunc( "callingOrderInvalidUninit",  &callingOrderInvalidUninit, true );
  testfunc( "callingOrderInitNoUninit",   &callingOrderInitNoUninit ); // not calling uninit seems to be valid
  testfunc( "callingOrderInitTwice",      &callingOrderInitTwice,     true );
  testfunc( "callingOrderNoInit",         &callingOrderNoInit,        true );
  testfunc( "callingOrderRegular",        &callingOrderRegular,       false );
  return 0;
}


int inputBufTest( vvenc::InputPicture& cInputPic )
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }
  vvenc::VvcAccessUnit cAU;
  cAU.m_iBufSize  = vvencParams.m_iWidth * vvencParams.m_iHeight;   cAU.m_pucBuffer = new unsigned char [ cAU.m_iBufSize ];

  if( 0 != cVVEnc.encode( &cInputPic, cAU))
  {
    return -1;
  }
  if( 0 != cVVEnc.uninit())
  {
    return -1;
  }
  return 0;
}


int invaildInputUninitialzedInputPic( )
{
  vvenc::InputPicture cInputPic;

  if( 0 != inputBufTest( cInputPic ))
  {
    return -1;
  }

  return 0;
}

int invaildInputInvalidPicSize( )
{
  vvenc::InputPicture cInputPic;
  cInputPic.m_cPicBuffer.m_pvY = &cInputPic;
  cInputPic.m_cPicBuffer.m_pvU = &cInputPic;
  cInputPic.m_cPicBuffer.m_pvV = &cInputPic;

  if( 0 != inputBufTest( cInputPic ))
  {
    return -1;
  }

  return 0;
}

int invaildInputInvalidLumaStride( )
{
  vvenc::InputPicture cInputPic;
  cInputPic.m_cPicBuffer.m_pvY = &cInputPic;
  cInputPic.m_cPicBuffer.m_pvU = &cInputPic;
  cInputPic.m_cPicBuffer.m_pvV = &cInputPic;
  cInputPic.m_cPicBuffer.m_iWidth = 176;
  cInputPic.m_cPicBuffer.m_iHeight = 144;
  cInputPic.m_cPicBuffer.m_iStride = 100;

  if( 0 != inputBufTest( cInputPic ))
  {
    return -1;
  }

  return 0;
}


int invaildInputInvalidChromaStride( )
{
  vvenc::InputPicture cInputPic;
  cInputPic.m_cPicBuffer.m_pvY = &cInputPic;
  cInputPic.m_cPicBuffer.m_pvU = &cInputPic;
  cInputPic.m_cPicBuffer.m_pvV = &cInputPic;
  cInputPic.m_cPicBuffer.m_iWidth = 176;
  cInputPic.m_cPicBuffer.m_iHeight = 144;
  cInputPic.m_cPicBuffer.m_iStride = 176;
  cInputPic.m_cPicBuffer.m_iCStride = 50;

  if( 0 != inputBufTest( cInputPic ))
  {
    return -1;
  }

  return 0;
}


int invaildInputBuf( )
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }

  vvenc::InputPicture cInputPic;
  if( 0 != cVVEnc.getPreferredBuffer( cInputPic.m_cPicBuffer ))
  {
    return -1;
  }
  fillInputPic( cInputPic );

  if( 0 != inputBufTest( cInputPic ))
  {
    return -1;
  }
  return 0;
}


int testInvalidInputParams()
{
  testfunc( "invaildInputUninitialzedInputPic",              &invaildInputUninitialzedInputPic,         true );
  testfunc( "invaildInputInvalidPicSize",                    &invaildInputInvalidPicSize,               true );

  testfunc( "invaildInputInvalidPicSize",                    &invaildInputInvalidPicSize,               true );
  testfunc( "invaildInputInvalidLumaStride",                 &invaildInputInvalidLumaStride,            true );
  testfunc( "invaildInputInvalidChromaStride",               &invaildInputInvalidChromaStride,          true );
 
  return 0;
}

int outputBufSizeTest( vvenc::VvcAccessUnit& cAU, int numPics)
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }

  vvenc::InputPicture cInputPic;
  if( 0 != cVVEnc.getPreferredBuffer( cInputPic.m_cPicBuffer ))
  {
    return -1;
  }
  fillInputPic( cInputPic );
  for(int i = 0; i < numPics; i++ )
  {
    if( 0 != cVVEnc.encode( &cInputPic, cAU))
    {
      return -1;
    }
  }
  if( 0 != cVVEnc.uninit())
  {
    return -1;
  }
  return 0;
}

int outputBufNull()
{
  vvenc::VvcAccessUnit cAU;
  cAU.m_pucBuffer = NULL;

  if( 0 != outputBufSizeTest( cAU, 1 ))
  {
    return -1;
  }
  return 0;
}

int outputBufSizeZero()
{
  vvenc::VvcAccessUnit cAU;
  cAU.m_pucBuffer = new unsigned char [20000];
  cAU.m_iBufSize = 0;

  if( 0 != outputBufSizeTest( cAU, 1 ))
  {
    return -1;
  }
  return 0;
}

int outputBufSizeToSmall()
{
  vvenc::VvcAccessUnit cAU;
  cAU.m_iBufSize = 10;
  cAU.m_pucBuffer = new unsigned char [ cAU.m_iBufSize ];

  if( 0 != outputBufSizeTest( cAU, 17 ))
  {
    return -1;
  }
  return 0;
}

int testInvalidOutputParams()
{
  testfunc( "outputBufNull",              &outputBufNull,         true );
  testfunc( "outputBufSizeZero",          &outputBufSizeZero,     true );
  testfunc( "outputBufSizeToSmall",       &outputBufSizeToSmall,  true );
  return 0;
}

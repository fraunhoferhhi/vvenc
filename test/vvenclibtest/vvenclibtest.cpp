/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

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
#include <vector>

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


template< class PicBufferLocal >
int allocPicBuffer( PicBufferLocal& rcPicBuffer, unsigned int uiWidth,  unsigned int uiHeight )
{
  const int iSizeFactor     = 2;
  const int iAlignmentGuard =16;
  rcPicBuffer.bitDepth = 10;
  rcPicBuffer.width    = uiWidth;
  rcPicBuffer.height   = uiHeight;
  rcPicBuffer.stride   = uiWidth;
  int iLumaSize   = uiWidth * uiHeight;
  const int iBufSize = iSizeFactor * iLumaSize * 3 / 2 + 3*iAlignmentGuard;

  rcPicBuffer.deletePicBuffer = new (std::nothrow) unsigned char[ iBufSize ];
  if( NULL == rcPicBuffer.deletePicBuffer )
  {
    return vvenc::VVENC_NOT_ENOUGH_MEM;
  }

  unsigned char* pY = rcPicBuffer.deletePicBuffer + iSizeFactor * ( 0 );
  unsigned char* pU = rcPicBuffer.deletePicBuffer + iSizeFactor * ( iLumaSize);
  unsigned char* pV = rcPicBuffer.deletePicBuffer + iSizeFactor * ( 5*iLumaSize/4);

  rcPicBuffer.y = (pY +   iAlignmentGuard) - (((size_t)pY) & (iAlignmentGuard-1));
  rcPicBuffer.u = (pU + 2*iAlignmentGuard) - (((size_t)pU) & (iAlignmentGuard-1));
  rcPicBuffer.v = (pV + 3*iAlignmentGuard) - (((size_t)pV) & (iAlignmentGuard-1));

  return 0;
}

void fillEncoderParameters( vvenc::VVEncParameter& cVVEncParameter )
{
  cVVEncParameter.qp               = 32;                         // quantization parameter 0-51
  cVVEncParameter.width            = 176;                        // luminance width of input picture
  cVVEncParameter.height           = 144;                        // luminance height of input picture
  cVVEncParameter.gopSize          = 16;                         // gop size (1: intra only, 16, 32: hierarchical b frames)
  cVVEncParameter.decodingRefreshType = vvenc::DRT_CRA;          // intra period refresh type
  cVVEncParameter.idrPeriod           = 32;                         // intra period for IDR/CDR intra refresh/RAP flag (should be a factor of m_iGopSize)
  cVVEncParameter.msgLevel         = vvenc::SILENT;              // log level > 4 (VERBOSE) enables psnr/rate output
  cVVEncParameter.temporalRate     = 60;                         // temporal rate (fps)
  cVVEncParameter.temporalScale    = 1;                          // temporal scale (fps)
  cVVEncParameter.ticksPerSecond   = 90000;                      // ticks per second e.g. 90000 for dts generation
  cVVEncParameter.threadCount      = 0;                          // number of worker threads (should not exceed the number of physical cpu's)
  cVVEncParameter.quality          = 0;                          // encoding quality (vs speed) 0: faster, 1: fast, 2: medium, 3: slow, 4: slower
  cVVEncParameter.perceptualQPA    = 2;                          // percepual qpa adaption, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  cVVEncParameter.inputBitDepth    = 8;                          // 8bit input
  cVVEncParameter.internalBitDepth = 10;                         // 10bit internal
  cVVEncParameter.profile          = vvenc::Profile::MAIN_10;    // profile: use main_10 or main_10_still_picture
  cVVEncParameter.level            = vvenc::Level::LEVEL4_1;     // level
  cVVEncParameter.tier             = vvenc::Tier::TIER_MAIN;     // tier
  cVVEncParameter.useAccessUnitDelimiter       = false;
  cVVEncParameter.useHrdParametersPresent      = false;
  cVVEncParameter.useBufferingPeriodSEIEnabled = false;
  cVVEncParameter.usePictureTimingSEIEnabled   = false;
}

void fillInputPic( vvenc::YuvPicture& cYuvPicture )
{
  const short val = 512;
  int lumaSize   = cYuvPicture.height   * cYuvPicture.stride;
  int chromaSize = ( cYuvPicture.cStride ) ? (cYuvPicture.height/2 * cYuvPicture.cStride) : (lumaSize / 4);
  std::fill_n( static_cast<short*> (cYuvPicture.y), lumaSize, val );
  std::fill_n( static_cast<short*> (cYuvPicture.u), chromaSize, val );
  std::fill_n( static_cast<short*> (cYuvPicture.v), chromaSize, val );
}

template< typename T, typename V = int>
int testParamList( const std::string& w, T& testParam, vvenc::VVEncParameter& vvencParams, const std::vector<V>& testValues, const bool expectedFail = false )
{
  vvenc::VVEnc cVVEnc;
  const int numFails = g_numFails;
  const T savedTestParam = testParam;

  for( auto testVal : testValues )
  {
    testParam = (T)testVal;
    try
    {
      // initialize the encoder
      TESTT( expectedFail == ( 0 == cVVEnc.checkConfig( vvencParams ) ), "\n" << w << "==" << testVal << " expected " << ( expectedFail ? "failure" : "success" ) );
    }
    catch ( ... )
    {
      ERROR( "\nCaught Exception " << w << "==" << testVal << " expected " << ( expectedFail ? "failure" : "success" ) ); //fail due to exception 
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

  testParamList( "DecodingRefreshType",                    vvencParams.decodingRefreshType,        vvencParams, { 0, 1 } );
  testParamList( "DecodingRefreshType",                    vvencParams.decodingRefreshType,        vvencParams, { -1,2,3,4 }, true );

  testParamList( "Level",                                  vvencParams.level,                      vvencParams, { 16,32,35,48,51,64,67,80,83,86,96,99,102,255 } );
  testParamList( "Level",                                  vvencParams.level,                      vvencParams, { -1,0,15,31,256, }, true );

  //  testParamList( "LogLevel",                               vvencParams.msgLevel,                   vvencParams, { 0,1,2,3,4,5,6} );
  //  testParamList( "LogLevel",                               vvencParams.msgLevel,                   vvencParams, {-1,7,8}, true );

  testParamList( "Profile",                                vvencParams.profile,                    vvencParams, { 1,3,9 } );
  testParamList( "Profile",                                vvencParams.profile,                    vvencParams, { -1,0,2,4,5,6,7,8,10 }, true );

  testParamList( "Tier",                                   vvencParams.tier,                       vvencParams, { 0,1 } );
  testParamList( "Tier",                                   vvencParams.tier,                       vvencParams, { -1,2 }, true );

  testParamList( "GOPSize",                                vvencParams.gopSize,                    vvencParams, { 16,32 } );
  testParamList( "GOPSize",                                vvencParams.gopSize,                    vvencParams, { 1,8, -1,0,2,3,4,17,33,64,128 }, true ); //th is this intended

  testParamList( "Width",                                  vvencParams.width,                      vvencParams, { 320,1920,3840 } );
  testParamList( "Width",                                  vvencParams.width,                      vvencParams, { -1,0 }, true );

  testParamList( "Height",                                 vvencParams.height,                     vvencParams, { 16,32,1080,1088 } );
  testParamList( "Height",                                 vvencParams.height,                     vvencParams, { -1,0 }, true );

  testParamList( "IDRPeriod",                              vvencParams.idrPeriod,                  vvencParams, { 16,32,48, 0 } );
  testParamList( "IDRPeriod",                              vvencParams.idrPeriod,                  vvencParams, { 1,-1,17,24 }, true );

  testParamList( "PerceptualQPA",                          vvencParams.perceptualQPA,              vvencParams, { 0,1,2,3,4,5 } );
  testParamList( "PerceptualQPA",                          vvencParams.perceptualQPA,              vvencParams, { -1,6 }, true );

  testParamList( "Qp",                                     vvencParams.qp,                         vvencParams, { 0,1,2,3,4,51 } );
  testParamList( "Qp",                                     vvencParams.qp,                         vvencParams, { -1,52 }, true );

  testParamList( "Quality",                                vvencParams.quality,                    vvencParams, { 0,1,2,3,4 } );
  testParamList( "Quality",                                vvencParams.quality,                    vvencParams, { -1,5 }, true );

  testParamList( "TargetBitRate",                          vvencParams.targetBitRate,              vvencParams, { 0,1000000,20000000 } );
  testParamList( "TargetBitRate",                          vvencParams.targetBitRate,              vvencParams, { -1,100000001 }, true );

  vvencParams.targetBitRate = 1;
  testParamList( "NumPasses",                              vvencParams.numPasses,                  vvencParams, { 1,2 } );
  testParamList( "NumPasses",                              vvencParams.numPasses,                  vvencParams, { -1,0,3 }, true );
  vvencParams.targetBitRate = 0;
  testParamList( "NumPasses",                              vvencParams.numPasses,                  vvencParams, { 1 } );
  testParamList( "NumPasses",                              vvencParams.numPasses,                  vvencParams, { 0,2 }, true );

  testParamList( "InputBitDepth",                          vvencParams.inputBitDepth,              vvencParams, { 8,10 } );
  testParamList( "InputBitDepth",                          vvencParams.inputBitDepth,              vvencParams, { 0,1,7,9,11 }, true );

  testParamList( "InternalBitDepth",                       vvencParams.internalBitDepth,           vvencParams, { 8,10 } );
  testParamList( "InternalBitDepth",                       vvencParams.internalBitDepth,           vvencParams, { 0,1,7,9,11 }, true );

  //  testParamList( "TemporalScale",                          vvencParams.temporalScale,              vvencParams, { 1,2,4,1001} );
  //  testParamList( "TemporalScale",                          vvencParams.temporalScale,              vvencParams, { -1,0,3,1000 }, true );

  vvencParams.temporalScale = 1;
  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { 1,25,30,50,60,100,120 } );
  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { -1,0/*,24*/ }, true );    //th is this intended

  vvencParams.temporalScale = 1001;
  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { 24000,30000,60000 /*,1200000*/ } );
  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { -1,1,0,24 }, true );

  fillEncoderParameters( vvencParams );

  testParamList( "ThreadCount",                            vvencParams.threadCount,                vvencParams, { 0,1,2,64 } );
  testParamList( "ThreadCount",                            vvencParams.threadCount,                vvencParams, { -1,65 }, true );

  testParamList( "TicksPerSecond",                         vvencParams.ticksPerSecond,             vvencParams, { 90000,27000000,60,120 } );
  testParamList( "TicksPerSecond",                         vvencParams.ticksPerSecond,             vvencParams, { -1,0, 50, 27000001 }, true );

  vvencParams.targetBitRate = 0;
  testParamList<bool, bool>( "useHrdParametersPresent",       vvencParams.useHrdParametersPresent,       vvencParams, { true }, true );
  testParamList<bool, bool>( "useBufferingPeriodSEIEnabled",  vvencParams.useBufferingPeriodSEIEnabled,  vvencParams, { true }, true );
  testParamList<bool, bool>( "usePictureTimingSEIEnabled",    vvencParams.usePictureTimingSEIEnabled,    vvencParams, { true }, true );

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
  if( 0 != cVVEnc.init( vvencParams ) )
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
  vvenc::AccessUnit cAU;
  vvenc::YuvPicture cYuvPicture;
  bool encodeDone = false;
  if( 0 != cVVEnc.encode( &cYuvPicture, cAU, encodeDone))
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
  if( 0 != cVVEnc.init( vvencParams ) )
  {
    return -1;
  }
  vvenc::AccessUnit cAU;
  vvenc::YuvPicture cYuvPicture;
  if( 0 != allocPicBuffer( cYuvPicture, vvencParams.width, vvencParams.height ))
  {
    return -1;
  }
  fillInputPic( cYuvPicture );

  bool encodeDone = false;
  if( 0 != cVVEnc.encode( &cYuvPicture, cAU, encodeDone ))
  {
    return -1;
  }
  if( 0 != cVVEnc.uninit())
  {
    return -1;
  }
  return 0;
}

int callingOrderRegularInitPass()
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ) )
  {
    return -1;
  }
  vvenc::AccessUnit cAU;
  vvenc::YuvPicture cYuvPicture;

  if( 0 != allocPicBuffer( cYuvPicture, vvencParams.width, vvencParams.height ))
  {
    return -1;
  }
  fillInputPic( cYuvPicture );
  if( 0 != cVVEnc.initPass( 0 ) )
  {
    return -1;
  }
  bool encodeDone = false;
  if( 0 != cVVEnc.encode( &cYuvPicture, cAU, encodeDone ))
  {
    return -1;
  }
  if( 0 != cVVEnc.uninit())
  {
    return -1;
  }
  return 0;
}

int callingOrderRegularInit2Pass()
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;
  fillEncoderParameters( vvencParams );

  vvencParams.numPasses = 2;
  vvencParams.targetBitRate = 500000;

  if( 0 != cVVEnc.init( vvencParams ) )
  {
    return -1;
  }
  vvenc::AccessUnit cAU;
  vvenc::YuvPicture cYuvPicture;
  if( 0 != allocPicBuffer( cYuvPicture, vvencParams.width, vvencParams.height ))
  {
    return -1;
  }
  fillInputPic( cYuvPicture );
  if( 0 != cVVEnc.initPass( 0 ) )
  {
    return -1;
  }

  bool encodeDone = false;
  if( 0 != cVVEnc.encode( &cYuvPicture, cAU, encodeDone ))
  {
    return -1;
  }

  vvenc::YuvPicture* flushPicture = nullptr;
  if( 0 != cVVEnc.encode( flushPicture, cAU, encodeDone ))
  {
    return -1;
  }

  if( 0 != cVVEnc.initPass( 1 ) )
  {
    return -1;
  }

  if( 0 != cVVEnc.encode( &cYuvPicture, cAU, encodeDone))
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
  testfunc( "callingOrderInvalidUninit",   &callingOrderInvalidUninit,   true  );
  testfunc( "callingOrderInitNoUninit",    &callingOrderInitNoUninit           ); // not calling uninit seems to be valid
  testfunc( "callingOrderInitTwice",       &callingOrderInitTwice,       true  );
  testfunc( "callingOrderNoInit",          &callingOrderNoInit,          true  );
  testfunc( "callingOrderRegular",         &callingOrderRegular,         false );
  testfunc( "callingOrderRegularInitPass", &callingOrderRegularInitPass, false );
  testfunc( "callingOrderRegularInit2Pass", &callingOrderRegularInit2Pass, false );

  return 0;
}


int inputBufTest( vvenc::YuvPicture& cYuvPicture )
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }
  vvenc::AccessUnit cAU;
  bool encodeDone = false;
  if( 0 != cVVEnc.encode( &cYuvPicture, cAU, encodeDone))
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
  vvenc::YuvPicture cYuvPicture;

  if( 0 != inputBufTest( cYuvPicture ))
  {
    return -1;
  }

  return 0;
}

int invaildInputInvalidPicSize( )
{
  vvenc::YuvPicture cYuvPicture;
  cYuvPicture.y = &cYuvPicture;
  cYuvPicture.u = &cYuvPicture;
  cYuvPicture.v = &cYuvPicture;

  if( 0 != inputBufTest( cYuvPicture ))
  {
    return -1;
  }

  return 0;
}

int invaildInputInvalidLumaStride( )
{
  vvenc::YuvPicture cYuvPicture;
  cYuvPicture.y = &cYuvPicture;
  cYuvPicture.u = &cYuvPicture;
  cYuvPicture.v = &cYuvPicture;
  cYuvPicture.width = 176;
  cYuvPicture.height = 144;
  cYuvPicture.stride = 100;

  if( 0 != inputBufTest( cYuvPicture ))
  {
    return -1;
  }

  return 0;
}


int invaildInputInvalidChromaStride( )
{
  vvenc::YuvPicture cYuvPicture;
  cYuvPicture.y = &cYuvPicture;
  cYuvPicture.u = &cYuvPicture;
  cYuvPicture.v = &cYuvPicture;
  cYuvPicture.width = 176;
  cYuvPicture.height = 144;
  cYuvPicture.stride = 176;
  cYuvPicture.cStride = 50;

  if( 0 != inputBufTest( cYuvPicture ))
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

  vvenc::YuvPicture cYuvPicture;
  if( 0 != allocPicBuffer( cYuvPicture, vvencParams.width, vvencParams.height ))
  {
    return -1;
  }
  fillInputPic( cYuvPicture );

  if( 0 != inputBufTest( cYuvPicture ))
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

int outputBufSizeTest( vvenc::AccessUnit& cAU, int numPics)
{
  vvenc::VVEnc cVVEnc;
  vvenc::VVEncParameter vvencParams;  
  fillEncoderParameters( vvencParams );
  if( 0 != cVVEnc.init( vvencParams ))
  {
    return -1;
  }

  vvenc::YuvPicture cYuvPicture;
  if( 0 != allocPicBuffer( cYuvPicture, vvencParams.width, vvencParams.height ))
  {
    return -1;
  }
  fillInputPic( cYuvPicture );
  bool encodeDone = false;
  for(int i = 0; i < numPics; i++ )
  {
    if( 0 != cVVEnc.encode( &cYuvPicture, cAU, encodeDone ))
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
  vvenc::AccessUnit cAU;
  if( 0 != outputBufSizeTest( cAU, 1 ))
  {
    return -1;
  }
  return 0;
}

int outputBufSizeZero()
{
  vvenc::AccessUnit cAU;
  if( 0 != outputBufSizeTest( cAU, 1 ))
  {
    return -1;
  }
  return 0;
}

int outputBufSizeToSmall()
{
  vvenc::AccessUnit cAU;
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

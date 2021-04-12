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

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
#define ERROR(w)    { g_numTests++; g_numFails ++;                   if( g_verbose ) std::cerr << "\n" << w << " test failed: In function "  << __FUNCTION__ << "\" ln " <<  __LINE__; }

int g_numTests = 0; 
int g_numFails = 0;
int g_verbose = 0;

int testLibCallingOrder();     // check invalid caling order
int testLibParameterRanges();  // single parameter rangewew checks 
int testInvalidInputParams();  // input Buffer does not match
int testSDKDefaultBehaviour(); // check default behaviour when using in sdk

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
    testSDKDefaultBehaviour();
    break;
  }
  default:
    testLibParameterRanges();
    testLibCallingOrder();
    testInvalidInputParams();
    testSDKDefaultBehaviour();
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


void fillEncoderParameters( VVEncCfg& rcEncCfg, bool callInitCfgParameter = true )
{
  rcEncCfg.m_SourceWidth                = 176;                 // luminance width of input picture
  rcEncCfg.m_SourceHeight               = 144;                 // luminance height of input picture
  rcEncCfg.m_GOPSize                    = 16;                  // gop size (1: intra only, 16, 32: hierarchical b frames)
  rcEncCfg.m_DecodingRefreshType        = VVENC_DRT_CRA;             // intra period refresh type
  rcEncCfg.m_IntraPeriod                = 32;                  // intra period for IDR/CDR intra refresh/RAP flag (should be a factor of m_iGopSize)
  rcEncCfg.m_verbosity                  = VVENC_SILENT;              // log level > 4 (VERBOSE) enables psnr/rate output
  rcEncCfg.m_FrameRate                  = 60;                  // temporal rate (fps)
//rcEncCfg.temporalScale                = 1;                   // temporal scale (fps)
  rcEncCfg.m_numThreads                 = 0;                   // number of worker threads (should not exceed the number of physical cpu's)
  rcEncCfg.m_usePerceptQPA              = true;                // perceptual QP adaptation (false: off, true: on)
  rcEncCfg.m_inputBitDepth[0]           = 8;                   // 8bit input
  rcEncCfg.m_internalBitDepth[0]        = 10;                  // 10bit internal

  rcEncCfg.m_internChromaFormat         =  VVENC_CHROMA_420;

  vvenc_init_preset(  &rcEncCfg, vvencPresetMode::VVENC_FASTER );
  if( callInitCfgParameter )
  {
    vvenc_init_cfg_parameter( &rcEncCfg );
  }
}

void defaultSDKInit( VVEncCfg& rcEncCfg, int targetBitrate, bool callInitCfgParameter = false )
{
  vvenc_init_default( &rcEncCfg, 176,144,60, targetBitrate, 32,  vvencPresetMode::VVENC_MEDIUM );

  if( callInitCfgParameter )
  {
    vvenc_init_cfg_parameter( &rcEncCfg );
  }
}

void fillInputPic( vvencYUVBuffer* pcYuvBuffer, const short val = 512 )
{
  for( int n = 0; n < VVENC_MAX_NUM_COMP; n++)
  {
    const int size = pcYuvBuffer->planes[n].stride * pcYuvBuffer->planes[n].height;
    std::fill_n( static_cast<short*> (pcYuvBuffer->planes[n].ptr), size, val );
  }
}

template< typename T, typename V = int>
int testParamList( const std::string& w, T& testParam, VVEncCfg& vvencParams, const std::vector<V>& testValues, const bool expectedFail = false )
{
  vvencEncoder *enc = vvenc_encoder_create();

  const int numFails = g_numFails;
  const T savedTestParam = testParam;

  for( auto testVal : testValues )
  {
    testParam = (T)testVal;
    try
    {
      // initialize the encoder
      TESTT( expectedFail == ( 0 == vvenc_check_config( enc, &vvencParams ) ), "\n" << w << "==" << testVal << " expected " << ( expectedFail ? "failure" : "success" ) );
    }
    catch ( ... )
    {
      ERROR( "\nCaught Exception " << w << "==" << testVal << " expected " << ( expectedFail ? "failure" : "success" ) ); //fail due to exception 
    }
  }

  vvenc_encoder_close( enc );

  //restore original test param
  testParam = savedTestParam;
  return numFails == g_numFails ? 0 : 1;
}

int testLibParameterRanges()
{
  VVEncCfg vvencParams;
  fillEncoderParameters( vvencParams, false );

  testParamList( "DecodingRefreshType",                    vvencParams.m_DecodingRefreshType,        vvencParams, { 1, 2 } );
  testParamList( "DecodingRefreshType",                    vvencParams.m_DecodingRefreshType,        vvencParams, { -1,0,3,4 }, true );

  testParamList( "Level",                                  vvencParams.m_level,                      vvencParams, { 16,32,35,48,51,64,67,80,83,86,96,99,102,255 } );
  testParamList( "Level",                                  vvencParams.m_level,                      vvencParams, { 15,31,256, }, true );

  //  testParamList( "LogLevel",                               vvencParams.msgLevel,                   vvencParams, { 0,1,2,3,4,5,6} );
  //  testParamList( "LogLevel",                               vvencParams.msgLevel,                   vvencParams, {-1,7,8}, true );

  testParamList( "Profile",                                vvencParams.m_profile,                    vvencParams, { 0,1,2 } );
  testParamList( "Profile",                                vvencParams.m_profile,                    vvencParams, { 3,4,5,6,7,8,9,10 }, true );
//  testParamList( "Profile",                                vvencParams.profile,                    vvencParams, { 1,3,9 } );
//  testParamList( "Profile",                                vvencParams.profile,                    vvencParams, { -1,0,2,4,5,6,7,8,10 }, true );

  testParamList( "Tier",                                   vvencParams.m_levelTier,                       vvencParams, { 0,1 } );
  testParamList( "Tier",                                   vvencParams.m_levelTier,                       vvencParams, { -1,2 }, true );

  testParamList( "GOPSize",                                vvencParams.m_GOPSize,                    vvencParams, { 16,32 } );
  vvencParams.m_IntraPeriod = 1;
  testParamList( "GOPSize",                                vvencParams.m_GOPSize,                    vvencParams, { 1 } );
  vvencParams.m_IntraPeriod = 32;
  testParamList( "GOPSize",                                vvencParams.m_GOPSize,                    vvencParams, { 1,8, -1,0,2,3,4,17,33,64,128 }, true ); //th is this intended

  testParamList( "Width",                                  vvencParams.m_SourceWidth,                      vvencParams, { 320,1920,3840 } );
  testParamList( "Width",                                  vvencParams.m_SourceWidth,                      vvencParams, { -1,0 }, true );

  testParamList( "Height",                                 vvencParams.m_SourceHeight,                     vvencParams, { 16,32,1080,1088 } );
  testParamList( "Height",                                 vvencParams.m_SourceHeight,                     vvencParams, { -1,0 }, true );

  testParamList( "IDRPeriod",                              vvencParams.m_IntraPeriod,                  vvencParams, { 16,32,48, 0 } );
  testParamList( "IDRPeriod",                              vvencParams.m_IntraPeriod,                  vvencParams, { 1,-1,17,24 }, true );

  testParamList( "Qp",                                     vvencParams.m_QP,                         vvencParams, { 0,1,2,3,4,51 } );
  testParamList( "Qp",                                     vvencParams.m_QP,                         vvencParams, { -1,64 }, true );

//  testParamList( "Quality",                                vvencParams.quality,                    vvencParams, { 0,1,2,3,4 } );
//  testParamList( "Quality",                                vvencParams.quality,                    vvencParams, { -1,5 }, true );

  testParamList( "TargetBitRate",                          vvencParams.m_RCTargetBitrate,              vvencParams, { 0,1000000,20000000 } );
  testParamList( "TargetBitRate",                          vvencParams.m_RCTargetBitrate,              vvencParams, { -1,800000001 }, true );

  vvencParams.m_RCTargetBitrate = 1;
  testParamList( "NumPasses",                              vvencParams.m_RCNumPasses,                  vvencParams, { 1,2 } );
  testParamList( "NumPasses",                              vvencParams.m_RCNumPasses,                  vvencParams, { 0,3 }, true );
  vvencParams.m_RCTargetBitrate = 0;

  testParamList( "NumPasses",                              vvencParams.m_RCNumPasses,                  vvencParams, { 1 } );
  testParamList( "NumPasses",                              vvencParams.m_RCNumPasses,                  vvencParams, { 0,2 }, true );

  testParamList( "InputBitDepth",                          vvencParams.m_inputBitDepth[0],              vvencParams, { 8,10 } );
  testParamList( "InputBitDepth",                          vvencParams.m_inputBitDepth[0],              vvencParams, { 0,1,7,9,11 }, true );

  testParamList( "InternalBitDepth",                       vvencParams.m_internalBitDepth[0],           vvencParams, { 8,10 } );
  testParamList( "InternalBitDepth",                       vvencParams.m_internalBitDepth[0],           vvencParams, { 0,1,7,9,11 }, true );

//  vvencParams.temporalScale = 1;
//  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { 1,25,30,50,60,100,120 } );
//  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { -1,0/*,24*/ }, true );    //th is this intended
//
//  vvencParams.temporalScale = 1001;
//  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { 24000,30000,60000 /*,1200000*/ } );
//  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { -1,1,0,24 }, true );

  fillEncoderParameters( vvencParams, false );

//  testParamList( "ThreadCount",                            vvencParams.m_ThreadCount,                vvencParams, { 0,1,2,64 } );
//  testParamList( "ThreadCount",                            vvencParams.m_ThreadCount,                vvencParams, { -1,65 }, true );

  testParamList( "TicksPerSecond",                         vvencParams.m_TicksPerSecond,             vvencParams, { 90000,27000000,60,120 } );
  testParamList( "TicksPerSecond",                         vvencParams.m_TicksPerSecond,             vvencParams, { -1,0, 50, 27000001 }, true );

  vvencParams.m_RCTargetBitrate = 0;
  testParamList( "useHrdParametersPresent",                   vvencParams.m_hrdParametersPresent,       vvencParams, { 1 }, true );
  testParamList<bool, bool>( "useBufferingPeriodSEIEnabled",  vvencParams.m_bufferingPeriodSEIEnabled,  vvencParams, { true }, true );
  testParamList<bool, bool>( "usePictureTimingSEIEnabled",    vvencParams.m_pictureTimingSEIEnabled,    vvencParams, { true }, true );

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

  return (numFails == g_numFails) ? 0 : 1;
}

int callingOrderInvalidUninit()
{
  vvencEncoder *enc = nullptr;
  if( 0 != vvenc_encoder_close( enc ))
  {
    return -1;
  }
  return 0;
}

int callingOrderInitNoUninit()
{
  VVEncCfg vvencParams;
  fillEncoderParameters( vvencParams, true );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  return 0;
}

int callingOrderInitTwice()
{
  VVEncCfg vvencParams; //
  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvenc_encoder_close( enc );

  return 0;
}

int callingOrderNoInit()
{
  vvencAccessUnit* AU;
  VVEncCfg vvencParams;
  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  vvencYUVBuffer* pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  vvenc_accessUnit_free( AU, true );

  return 0;
}

int callingOrderRegular()
{
  VVEncCfg vvencParams;
  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU;
  vvencYUVBuffer* pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );

  fillInputPic( pcYuvPicture );

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  vvencYUVBuffer* pcYUVBufferFlush = nullptr;
  if( 0 != vvenc_encode( enc, pcYUVBufferFlush, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( !encodeDone )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( 0 != vvenc_encoder_close( enc ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  vvenc_accessUnit_free( AU, true );

  return 0;
}

int callingOrderNotRegular()
{
  VVEncCfg vvencParams;
  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU;
  vvencYUVBuffer* pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );
  fillInputPic( pcYuvPicture );

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  vvencYUVBuffer* pcYUVBufferFlush = nullptr;
  if( 0 != vvenc_encode( enc, pcYUVBufferFlush, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( 0 != vvenc_encoder_close( enc ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  vvenc_accessUnit_free( AU, true );

  return 0;
}

int callingOrderRegularInitPass()
{
  VVEncCfg vvencParams;
  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU;

  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );

  fillInputPic( pcYuvPicture );
  if( 0 != vvenc_init_pass( enc, 0 ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_encoder_close( enc );
    return -1;
  }
  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    vvenc_encoder_close( enc );
    return -1;
  }
  if( 0 != vvenc_encoder_close( enc ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );

    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  vvenc_accessUnit_free( AU, true );
  return 0;
}

int callingOrderRegularInit2Pass()
{
  VVEncCfg vvencParams;
  vvencParams.m_RCNumPasses = 2;
  vvencParams.m_RCTargetBitrate = 500000;

  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit *AU;
  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );
  fillInputPic( pcYuvPicture );

  if( 0 != vvenc_init_pass( enc, 0 ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    return -1;
  }

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( 0 != vvenc_encode( enc, nullptr, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( 0 != vvenc_init_pass( enc, 1 ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }
  if( 0 != vvenc_encoder_close( enc ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  vvenc_accessUnit_free( AU, true );

  return 0;
}


int checkSDKDefaultBehaviourRC()
{
  VVEncCfg vvencParams;
  defaultSDKInit( vvencParams,  500000 );
  vvencParams.m_internChromaFormat = VVENC_CHROMA_420;

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit *AU;
  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );
  fillInputPic( pcYuvPicture );

  int validAUs = 0;

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( AU && AU->payloadUsedSize > 0 )
  {
    validAUs++;
  }

  while ( !encodeDone )
  {
    if( 0 != vvenc_encode( enc, nullptr, &AU, &encodeDone ))
    {
      vvenc_YUVBuffer_free( pcYuvPicture, true );
      vvenc_accessUnit_free( AU, true );
      return -1;
    }

    if( AU && AU->payloadUsedSize > 0 )
    {
      validAUs++;
    }
  }

  if( 0 != vvenc_encoder_close( enc ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }


  if( validAUs != 1 )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  vvenc_accessUnit_free( AU, true );

  return 0;
}


int testLibCallingOrder()
{
  testfunc( "callingOrderInvalidUninit",    &callingOrderInvalidUninit,    true  );
  testfunc( "callingOrderInitNoUninit",     &callingOrderInitNoUninit            ); // not calling uninit seems to be valid
  testfunc( "callingOrderInitTwice",        &callingOrderInitTwice,        true  );
  testfunc( "callingOrderNoInit",           &callingOrderNoInit,           true  );
  testfunc( "callingOrderRegular",          &callingOrderRegular,          false );
  testfunc( "callingOrderRegularInitPass",  &callingOrderRegularInitPass,  false );
  testfunc( "callingOrderRegularInit2Pass", &callingOrderRegularInit2Pass, false );

  testfunc( "callingOrderNotRegular",       &callingOrderNotRegular,       true );

  return 0;
}

int testSDKDefaultBehaviour()
{
  testfunc( "checkSDKDefaultBehaviourRC", &checkSDKDefaultBehaviourRC, false );
  return 0;
}

int inputBufTest( vvencYUVBuffer* pcYuvPicture )
{
  VVEncCfg vvencParams;
  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU;
  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, &AU, &encodeDone))
  {
    return -1;
  }

  if( 0 != vvenc_encoder_close( enc ))
  {
    return -1;
  }

  vvenc_accessUnit_free( AU, true );

  return 0;
}


int invaildInputUninitialzedInputPic( )
{
  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  if( 0 != inputBufTest( pcYuvPicture ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );

  return 0;
}

int invaildInputInvalidPicSize( )
{
  int16_t dummy = 0;

  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  pcYuvPicture->planes[0].ptr = &dummy;
  pcYuvPicture->planes[1].ptr = &dummy;
  pcYuvPicture->planes[2].ptr = &dummy;

  if( 0 != inputBufTest( pcYuvPicture ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );


  return 0;
}

int invaildInputInvalidLumaStride( )
{
  int16_t dummy = 0;
  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();

  pcYuvPicture->planes[0].ptr = &dummy;
  pcYuvPicture->planes[1].ptr = &dummy;
  pcYuvPicture->planes[2].ptr = &dummy;
  pcYuvPicture->planes[0].width  = 176;
  pcYuvPicture->planes[0].height = 144;
  pcYuvPicture->planes[0].stride = 100;

  if( 0 != inputBufTest( pcYuvPicture ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  return 0;
}


int invaildInputInvalidChromaStride( )
{
  int16_t dummy = 0;
  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();

  pcYuvPicture->planes[0].ptr = &dummy;
  pcYuvPicture->planes[1].ptr = &dummy;
  pcYuvPicture->planes[2].ptr = &dummy;
  pcYuvPicture->planes[0].width  = 176;
  pcYuvPicture->planes[0].height = 144;
  pcYuvPicture->planes[0].stride = 100;
  pcYuvPicture->planes[1].width  = 88;
  pcYuvPicture->planes[1].height = 72;
  pcYuvPicture->planes[1].stride = 50;

  if( 0 != inputBufTest( pcYuvPicture ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  return 0;
}


int invaildInputBuf( )
{
  VVEncCfg vvencParams;
  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams, NULL ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );
  fillInputPic( pcYuvPicture );

  if( 0 != inputBufTest( pcYuvPicture ))
  {
    return -1;
  }

  if( 0 != vvenc_encoder_close( enc ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );

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


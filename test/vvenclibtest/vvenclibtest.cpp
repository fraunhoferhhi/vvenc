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
#include <tuple>

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
int testStringApiInterface();  // check behaviour when using in sdk by using string api
int testTimestamps();          // check behaviour when using in sdk by using string api

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
      printHelp = ( testId < 1 || testId > 6 );
    }

    if( printHelp )
    {
      printf( "venclibtest <test> [1..5]\n");
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
  case 5:
  {
    testStringApiInterface();
    break;
  }
  case 6:
  {
    testTimestamps();
    break;
  }
  default:
    testLibParameterRanges();
    testLibCallingOrder();
    testInvalidInputParams();
    testSDKDefaultBehaviour();
    testStringApiInterface();
    testTimestamps();
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


void fillEncoderParameters( vvenc_config& rcEncCfg, bool callInitCfgParameter = true )
{
  rcEncCfg.m_SourceWidth                = 176;                 // luminance width of input picture
  rcEncCfg.m_SourceHeight               = 144;                 // luminance height of input picture
  rcEncCfg.m_GOPSize                    = 16;                  // gop size (1: intra only, 16, 32: hierarchical b frames)
  rcEncCfg.m_DecodingRefreshType        = VVENC_DRT_CRA;             // intra period refresh type
  rcEncCfg.m_IntraPeriod                = 32;                  // intra period for IDR/CDR intra refresh/RAP flag (should be a factor of m_iGopSize)
  rcEncCfg.m_verbosity                  = VVENC_SILENT;              // log level > 4 (VERBOSE) enables psnr/rate output
  rcEncCfg.m_FrameRate                  = 60;                  // temporal rate (fps)
  rcEncCfg.m_FrameScale                 = 1;                   // temporal scale (fps)
  rcEncCfg.m_numThreads                 = 0;                   // number of worker threads (should not exceed the number of physical cpu's)
  rcEncCfg.m_usePerceptQPA              = true;                // perceptual QP adaptation (false: off, true: on)
  rcEncCfg.m_inputBitDepth[0]           = 8;                   // 8bit input
  rcEncCfg.m_internalBitDepth[0]        = 10;                  // 10bit internal

  rcEncCfg.m_internChromaFormat         =  VVENC_CHROMA_420;

  vvenc_init_preset(  &rcEncCfg, vvencPresetMode::VVENC_FASTER );
  if( callInitCfgParameter )
  {
    vvenc_init_config_parameter( &rcEncCfg );
  }
}

void defaultSDKInit( vvenc_config& rcEncCfg, int targetBitrate, bool callInitCfgParameter = false )
{
  vvenc_init_default( &rcEncCfg, 176,144,60, targetBitrate, 32, vvencPresetMode::VVENC_MEDIUM );

  if( callInitCfgParameter )
  {
    vvenc_init_config_parameter( &rcEncCfg );
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
int testParamList( const std::string& w, T& testParam, vvenc_config& vvencParams, const std::vector<V>& testValues, const bool expectedFail = false )
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
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams, false );

  testParamList( "DecodingRefreshType",                    vvencParams.m_DecodingRefreshType,        vvencParams, { 1, 2, 4, 5 } );
  testParamList( "DecodingRefreshType",                    vvencParams.m_DecodingRefreshType,        vvencParams, { -1,0,3,6 }, true );

  testParamList( "Level",                                  vvencParams.m_level,                      vvencParams, { 16,32,35,48,51,64,67,80,83,86,96,99,102,255 } );
  testParamList( "Level",                                  vvencParams.m_level,                      vvencParams, { 15,31,256, }, true );

  //  testParamList( "LogLevel",                               vvencParams.msgLevel,                   vvencParams, { 0,1,2,3,4,5,6} );
  //  testParamList( "LogLevel",                               vvencParams.msgLevel,                   vvencParams, {-1,7,8}, true );

  testParamList( "Profile",                                vvencParams.m_profile,                    vvencParams, { 0,1,17,33,49,65,81,97,113 } );
  testParamList( "Profile",                                vvencParams.m_profile,                    vvencParams, { 2,3,5,6,7,8,9,114 }, true );
//  testParamList( "Profile",                                vvencParams.profile,                    vvencParams, { 1,3,9 } );
//  testParamList( "Profile",                                vvencParams.profile,                    vvencParams, { -1,0,2,4,5,6,7,8,10 }, true );

  testParamList( "Tier",                                   vvencParams.m_levelTier,                  vvencParams, { 0,1 } );
  testParamList( "Tier",                                   vvencParams.m_levelTier,                  vvencParams, { -1,2 }, true );

  testParamList( "GOPSize",                                vvencParams.m_GOPSize,                    vvencParams, { 16,32 } );
  vvencParams.m_IntraPeriod = 1;
  testParamList( "GOPSize",                                vvencParams.m_GOPSize,                    vvencParams, { 1 } );
  vvencParams.m_IntraPeriod = 32;
  testParamList( "GOPSize",                                vvencParams.m_GOPSize,                    vvencParams, { -1,0,33,64,128 }, true ); //th is this intended

  testParamList( "Width",                                  vvencParams.m_SourceWidth,                vvencParams, { 320,1920,3840 } );
  testParamList( "Width",                                  vvencParams.m_SourceWidth,                vvencParams, { -1,0 }, true );

  testParamList( "Height",                                 vvencParams.m_SourceHeight,               vvencParams, { 16,32,1080,1088 } );
  testParamList( "Height",                                 vvencParams.m_SourceHeight,               vvencParams, { -1,0 }, true );

  testParamList( "IDRPeriod",                              vvencParams.m_IntraPeriod,                vvencParams, { -1,1,16,17,24,25,32,48,50,60, 0 } );
  testParamList( "IDRPeriod",                              vvencParams.m_IntraPeriod,                vvencParams, { -2 }, true );

  testParamList( "Qp",                                     vvencParams.m_QP,                         vvencParams, { 0,1,2,3,4,51 } );
  testParamList( "Qp",                                     vvencParams.m_QP,                         vvencParams, { -1,64 }, true );

//  testParamList( "Quality",                                vvencParams.quality,                    vvencParams, { 0,1,2,3,4 } );
//  testParamList( "Quality",                                vvencParams.quality,                    vvencParams, { -1,5 }, true );

  testParamList( "TargetBitRate",                          vvencParams.m_RCTargetBitrate,            vvencParams, { 0,1000000,20000000 } );
  testParamList( "TargetBitRate",                          vvencParams.m_RCTargetBitrate,            vvencParams, { -1,800000001 }, true );

  vvencParams.m_RCTargetBitrate = 1;
  testParamList( "NumPasses",                              vvencParams.m_RCNumPasses,                vvencParams, { 1,2 } );
  testParamList( "NumPasses",                              vvencParams.m_RCNumPasses,                vvencParams, { 0,3 }, true );
  vvencParams.m_RCTargetBitrate = 0;

  testParamList( "NumPasses",                              vvencParams.m_RCNumPasses,                vvencParams, { 1 } );
  testParamList( "NumPasses",                              vvencParams.m_RCNumPasses,                vvencParams, { 0,2 }, true );

  testParamList( "InputBitDepth",                          vvencParams.m_inputBitDepth[0],           vvencParams, { 8,10 } );
  testParamList( "InputBitDepth",                          vvencParams.m_inputBitDepth[0],           vvencParams, { 0,1,7,9,11 }, true );

  testParamList( "InternalBitDepth",                       vvencParams.m_internalBitDepth[0],        vvencParams, { 8,10 } );
  testParamList( "InternalBitDepth",                       vvencParams.m_internalBitDepth[0],        vvencParams, { 0,1,7,9,11 }, true );

//  vvencParams.temporalScale = 1;
//  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { 1,25,30,50,60,100,120 } );
//  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { -1,0/*,24*/ }, true );    //th is this intended
//
//  vvencParams.temporalScale = 1001;
//  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { 24000,30000,60000 /*,1200000*/ } );
//  testParamList( "TemporalRate",                           vvencParams.temporalRate,               vvencParams, { -1,1,0,24 }, true );
  
  vvencParams.m_SourceWidth        = 832;
  vvencParams.m_SourceHeight       = 480;
  testParamList<bool, bool>( "PicPartition",               vvencParams.m_picPartitionFlag,           vvencParams, { 1 } );
  vvencParams.m_tileColumnWidth[0] = 5;
  vvencParams.m_tileRowHeight[0]   = 3;
  testParamList<bool, bool>( "PicPartition",               vvencParams.m_picPartitionFlag,           vvencParams, { 1 } );
  vvencParams.m_tileColumnWidth[0] = 4;
  vvencParams.m_tileRowHeight[0]   = 2;
  testParamList<bool, bool>( "PicPartition",               vvencParams.m_picPartitionFlag,           vvencParams, { 1 }, true );

  fillEncoderParameters( vvencParams, false );

  testParamList( "RPR",                                    vvencParams.m_rprEnabledFlag,             vvencParams, { -1, 0, 1 } );
  testParamList( "RPR",                                    vvencParams.m_rprEnabledFlag,             vvencParams, { -2, 2, 3 }, true );

//  testParamList( "ThreadCount",                            vvencParams.m_ThreadCount,                vvencParams, { 0,1,2,64 } );
//  testParamList( "ThreadCount",                            vvencParams.m_ThreadCount,                vvencParams, { -1,65 }, true );

  testParamList( "TicksPerSecond",                         vvencParams.m_TicksPerSecond,             vvencParams, { 90000,27000000,60,120,-1 } );
  testParamList( "TicksPerSecond",                         vvencParams.m_TicksPerSecond,             vvencParams, { -2,0, 50, 27000001 }, true );

  vvencParams.m_RCTargetBitrate = 0;
  testParamList( "useHrdParametersPresent",                   vvencParams.m_hrdParametersPresent,       vvencParams, { 0, 1 } );
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
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams, true );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  return 0;
}

int callingOrderInitTwice()
{
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvenc_encoder_close( enc );

  return 0;
}

int callingOrderNoInit()
{
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams );

  vvencAccessUnit* AU = vvenc_accessUnit_alloc();
  vvenc_accessUnit_alloc_payload( AU, vvencParams.m_SourceWidth*vvencParams.m_SourceHeight );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  vvencYUVBuffer* pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone))
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
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU = vvenc_accessUnit_alloc();
  vvenc_accessUnit_alloc_payload( AU, vvencParams.m_SourceWidth*vvencParams.m_SourceHeight );

  vvencYUVBuffer* pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );

  fillInputPic( pcYuvPicture );

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  while( ! encodeDone )
  {
    vvencYUVBuffer* pcYUVBufferFlush = nullptr;
    if( 0 != vvenc_encode( enc, pcYUVBufferFlush, AU, &encodeDone ))
    {
      vvenc_YUVBuffer_free( pcYuvPicture, true );
      vvenc_accessUnit_free( AU, true );
      return -1;
    }
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
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU = vvenc_accessUnit_alloc();
  vvenc_accessUnit_alloc_payload( AU, vvencParams.m_SourceWidth*vvencParams.m_SourceHeight );

  vvencYUVBuffer* pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );
  fillInputPic( pcYuvPicture );

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  vvencYUVBuffer* pcYUVBufferFlush = nullptr;
  if( 0 != vvenc_encode( enc, pcYUVBufferFlush, AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone ))
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
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU = vvenc_accessUnit_alloc();
  vvenc_accessUnit_alloc_payload( AU, vvencParams.m_SourceWidth*vvencParams.m_SourceHeight );

  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );

  fillInputPic( pcYuvPicture );
  if( 0 != vvenc_init_pass( enc, 0, nullptr ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_encoder_close( enc );
    return -1;
  }
  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone ))
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
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  vvencParams.m_RCNumPasses = 2;
  vvencParams.m_RCTargetBitrate = 500000;

  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU = vvenc_accessUnit_alloc();
  vvenc_accessUnit_alloc_payload( AU, vvencParams.m_SourceWidth*vvencParams.m_SourceHeight );

  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );
  fillInputPic( pcYuvPicture );

  if( 0 != vvenc_init_pass( enc, 0, nullptr ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    return -1;
  }

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  while( ! encodeDone )
  {
    if( 0 != vvenc_encode( enc, nullptr, AU, &encodeDone ))
    {
      vvenc_YUVBuffer_free( pcYuvPicture, true );
      vvenc_accessUnit_free( AU, true );
      return -1;
    }
  }

  if( 0 != vvenc_init_pass( enc, 1, nullptr ) )
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
  }

  while( ! encodeDone )
  {
    if( 0 != vvenc_encode( enc, nullptr, AU, &encodeDone ))
    {
      vvenc_YUVBuffer_free( pcYuvPicture, true );
      vvenc_accessUnit_free( AU, true );
      return -1;
    }
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
  vvenc_config vvencParams;
  defaultSDKInit( vvencParams,  500000 );
  vvencParams.m_internChromaFormat = VVENC_CHROMA_420;
  vvencParams.m_RCNumPasses = 1;

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU = vvenc_accessUnit_alloc();
  vvenc_accessUnit_alloc_payload( AU, vvencParams.m_SourceWidth*vvencParams.m_SourceHeight );

  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, vvencParams.m_internChromaFormat, vvencParams.m_SourceWidth, vvencParams.m_SourceHeight );
  fillInputPic( pcYuvPicture );

  int validAUs = 0;

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone ))
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
    if( 0 != vvenc_encode( enc, nullptr, AU, &encodeDone ))
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

int checkSDKStringApiDefault()
{
  vvenc_config c;
  vvenc_init_default( &c, 176,144,60, 500000, 32, vvencPresetMode::VVENC_MEDIUM );

  std::vector <std::tuple<std::string, std::string>> settings;
  settings.push_back(std::make_tuple( VVENC_OPT_SIZE,         "176x144") );
  settings.push_back(std::make_tuple( VVENC_OPT_WIDTH,        "176") );
  settings.push_back(std::make_tuple( VVENC_OPT_HEIGHT,       "144") );
  settings.push_back(std::make_tuple( VVENC_OPT_FRAMERATE,    "60") );
  settings.push_back(std::make_tuple( VVENC_OPT_FRAMESCALE,   "1") );
  settings.push_back(std::make_tuple( VVENC_OPT_FPS,          "60/1") );
  settings.push_back(std::make_tuple( VVENC_OPT_TICKSPERSEC,  "900000") );
  settings.push_back(std::make_tuple( VVENC_OPT_INPUTBITDEPTH,"10") );
  settings.push_back(std::make_tuple( VVENC_OPT_FRAMES,       "2") );
  settings.push_back(std::make_tuple( VVENC_OPT_PRESET,       "medium") );
  settings.push_back(std::make_tuple( VVENC_OPT_THREADS,      "1") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1000000") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1M") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1.5Mbps") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1000k") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1500.5kbps") );

  settings.push_back(std::make_tuple( VVENC_OPT_QP,           "32") );
  settings.push_back(std::make_tuple( VVENC_OPT_TILES,        "1x0") );
  settings.push_back(std::make_tuple( VVENC_OPT_VERBOSITY,    "verbose") );

  settings.push_back(std::make_tuple( VVENC_OPT_PROFILE,      "auto") );
  settings.push_back(std::make_tuple( VVENC_OPT_LEVEL,        "auto") );
  settings.push_back(std::make_tuple( VVENC_OPT_TIER,         "main") );

  settings.push_back(std::make_tuple( VVENC_OPT_INTRAPERIOD,         "0") );
  settings.push_back(std::make_tuple( VVENC_OPT_REFRESHDSEC,         "1") );
  settings.push_back(std::make_tuple( VVENC_OPT_DECODINGREFRESHTYPE, "cra") );
  settings.push_back(std::make_tuple( VVENC_OPT_GOPSIZE,              "32") );

  settings.push_back(std::make_tuple( VVENC_OPT_QPA,              "off") );
  settings.push_back(std::make_tuple( VVENC_OPT_RCPASSES,         "2") );
  settings.push_back(std::make_tuple( VVENC_OPT_RCPASS,           "1") );
  settings.push_back(std::make_tuple( VVENC_OPT_INTERNALBITDEPTH, "10") );
  settings.push_back(std::make_tuple( VVENC_OPT_HDR,              "off") );
  settings.push_back(std::make_tuple( VVENC_OPT_SEGMENT,          "off") );

  for( auto & d : settings )
  {
    std::string key = std::get<0>(d);
    std::string value = std::get<1>(d);
    int parse_ret = vvenc_set_param( &c, key.c_str(), value.c_str() );
    switch (parse_ret)
    {
      case VVENC_PARAM_BAD_NAME:
        return -1;
      case VVENC_PARAM_BAD_VALUE:
        return -1;
      default:
        break;
    }
  }

  if( vvenc_init_config_parameter( &c ) )
  {
    return -1;
  }

  return 0;
}

int checkSDKStringApiInvalid()
{
  vvenc_config c;
  vvenc_init_default( &c, 176,144,60, 500000, 32, vvencPresetMode::VVENC_MEDIUM );

  std::vector <std::tuple<std::string, std::string>> settings;
  settings.push_back(std::make_tuple( VVENC_OPT_SIZE,         "176t144") );
  settings.push_back(std::make_tuple( VVENC_OPT_PRESET,       "MED") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1apple") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1;5M") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1m") );
  settings.push_back(std::make_tuple( VVENC_OPT_BITRATE,      "1K") );

  int ret=0;

  for( auto & d : settings )
  {
    std::string key = std::get<0>(d);
    std::string value = std::get<1>(d);
    int parse_ret = vvenc_set_param( &c, key.c_str(), value.c_str() );
    switch (parse_ret)
    {
      case VVENC_PARAM_BAD_NAME:
        ret = -1;
        break;
      case VVENC_PARAM_BAD_VALUE:
        ret = -1;
        break;
      default:
        return 0; // expecting an error - if no error something went wrong
        break;
    }
  }

  if( vvenc_init_config_parameter( &c ) )
  {
    return -1;
  }

  return ret;
}

static int runEncoder( vvenc_config& c, uint64_t framesToEncode ) 
{
  uint64_t ctsDiff   = (c.m_TicksPerSecond > 0) ? (uint64_t)c.m_TicksPerSecond * (uint64_t)c.m_FrameScale / (uint64_t)c.m_FrameRate : 1;  // expected cts diff between frames
  uint64_t ctsOffset = (c.m_TicksPerSecond > 0) ? (uint64_t)c.m_TicksPerSecond : (uint64_t)c.m_FrameRate/(uint64_t)c.m_FrameScale;        // start with offset 1sec, to generate  cts/dts > 0
  //std::cout << "test framerate " << c.m_FrameRate << "/" << c.m_FrameScale << " TicksPerSecond  " << c.m_TicksPerSecond << " ctsDiff " << ctsDiff << " framesToEncode " << framesToEncode  << std::endl;
  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
    return -1;

  if( 0 != vvenc_encoder_open( enc, &c ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU = vvenc_accessUnit_alloc();
  vvenc_accessUnit_alloc_payload( AU, c.m_SourceWidth*c.m_SourceHeight );

  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  vvenc_YUVBuffer_alloc_buffer( pcYuvPicture, c.m_internChromaFormat, c.m_SourceWidth, c.m_SourceHeight );
  fillInputPic( pcYuvPicture );
  
  uint64_t lastDts=0;
  uint64_t auCount=0;
  bool encodeDone = false;
  for( uint64_t f = 0; f < framesToEncode; f++ )
  {
    pcYuvPicture->cts      = (c.m_TicksPerSecond > 0) ? (ctsOffset + (f * (uint64_t)c.m_TicksPerSecond * (uint64_t)c.m_FrameScale / (uint64_t)c.m_FrameRate)) : (ctsOffset + f);
    pcYuvPicture->ctsValid = true;

    if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone ))
    {
      goto fail;
    }

    if( AU && AU->payloadUsedSize > 0 )
    {
      auCount++;
      if ( !AU->ctsValid || !AU->dtsValid )
      {
        goto fail;
      } 
      //std::cout << " AU dts " << AU->dts << " lastDts " << lastDts  << " diff " << AU->dts - lastDts << std::endl;
      if ( lastDts > 0 && (AU->dts != lastDts + ctsDiff || AU->dts <= lastDts) )
      {
        if ( AU->dts <= lastDts ){
          //std::cout << " AU dts " << AU->dts << " <= " << lastDts << " - dts must always increase" << std::endl;
        }else{
          //std::cout << " AU dts " << AU->dts << " but expecting " << lastDts + ctsDiff << " lastDts " << lastDts << std::endl;
        }
        goto fail;
      }
      lastDts = AU->dts;
    }

    if ( auCount > 0 && (!AU || ( AU &&  AU->payloadUsedSize == 0 )) )
    {
      //std::cout << " no valid AU received. encoder must always return an AU" << std::endl;
      goto fail;
    }
  }

  while ( !encodeDone )
  {
    if( 0 != vvenc_encode( enc, nullptr, AU, &encodeDone ))
    {
      goto fail;
    }

    if( AU && AU->payloadUsedSize > 0 )
    {
      auCount++;
      if ( !AU->ctsValid || !AU->dtsValid )
      {
        goto fail;
      } 
      //std::cout << " AU dts " << AU->dts << " lastDts " << lastDts  << " diff " << AU->dts - lastDts << std::endl;
      if ( lastDts > 0 && (AU->dts != lastDts + ctsDiff || AU->dts <= lastDts) )
      {
        if ( AU->dts <= lastDts ){
          //std::cout << " AU dts " << AU->dts << " <= " << lastDts << " - dts must always increase" << std::endl;
        }else{
          //std::cout << " AU dts " << AU->dts << " but expecting " << lastDts + ctsDiff << " lastDts " << lastDts << std::endl;
        }
        goto fail;
      }
      lastDts = AU->dts;
    }
    if ( !encodeDone && auCount > 0 && (!AU || ( AU &&  AU->payloadUsedSize == 0 )) )
    {
      //std::cout << " no valid AU received. encoder must always return an AU" << std::endl;
      goto fail;
    }
  }

  if( auCount != framesToEncode )
  {
    //std::cout << "expecting " << frames << " au, but only encoded " << auCount << std::endl;
    goto fail;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, true );
  vvenc_accessUnit_free( AU, true );
  return 0;

  fail:
    vvenc_YUVBuffer_free( pcYuvPicture, true );
    vvenc_accessUnit_free( AU, true );
    return -1;
}

int checkTimestampsDefault()
{
  std::vector <std::tuple<int,int>> framerates;
  framerates.push_back(std::make_tuple( 25,1) );
  framerates.push_back(std::make_tuple( 30,1) );
  framerates.push_back(std::make_tuple( 50,1) );
  framerates.push_back(std::make_tuple( 60,1) );
  framerates.push_back(std::make_tuple( 120,1) );

  std::vector <int> tickspersecVec;
  tickspersecVec.push_back(90000);
  tickspersecVec.push_back(-1);

  for( auto & tickspersec : tickspersecVec )
  {
    for( auto & fps : framerates )
    {
      vvenc_config c;
      vvenc_init_default( &c, 176,144, 60, 0, 55, vvencPresetMode::VVENC_FASTER );
      c.m_internChromaFormat = VVENC_CHROMA_420;

      c.m_FrameRate  = std::get<0>(fps);
      c.m_FrameScale = std::get<1>(fps);
      c.m_TicksPerSecond = tickspersec;
      uint64_t frames= c.m_FrameRate/c.m_FrameScale * 2;

      if( 0 != runEncoder(c,frames) )
      {
        return -1;
      }
    }
  }

  framerates.clear();
  tickspersecVec.clear();
  framerates.push_back(std::make_tuple( 25000,1001) );
  framerates.push_back(std::make_tuple( 30000,1001) );
  framerates.push_back(std::make_tuple( 60000,1001) );
  framerates.push_back(std::make_tuple( 120000,1001) );

  tickspersecVec.push_back(27000000);
  tickspersecVec.push_back(-1);

  for( auto & tickspersec : tickspersecVec )
  {
    for( auto & fps : framerates )
    {
      vvenc_config c;
      vvenc_init_default( &c, 176,144, 60, 0, 55, vvencPresetMode::VVENC_FASTER );
      c.m_internChromaFormat = VVENC_CHROMA_420;
      c.m_FrameRate  = std::get<0>(fps);
      c.m_FrameScale = std::get<1>(fps);
      c.m_TicksPerSecond = tickspersec;
      uint64_t frames= c.m_FrameRate/c.m_FrameScale * 2;

      if( 0 != runEncoder(c,frames) )
      {
        return -1;
      }
    }
  }

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


int testStringApiInterface()
{
  testfunc( "checkSDKStringApiDefault", &checkSDKStringApiDefault, false );
  testfunc( "checkSDKStringApiInvalid", &checkSDKStringApiInvalid, true );

  return 0;
}

int testTimestamps()
{
  testfunc( "checkTimestampsDefault", &checkTimestampsDefault, false );
  //testfunc( "checkSDKStringApiInvalid", &checkSDKStringApiInvalid, true );

  return 0;
}

int inputBufTest( vvencYUVBuffer* pcYuvPicture )
{
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
  {
    vvenc_encoder_close( enc );
    return -1;
  }

  vvencAccessUnit* AU = vvenc_accessUnit_alloc();
  vvenc_accessUnit_alloc_payload( AU, vvencParams.m_SourceWidth*vvencParams.m_SourceHeight );

  bool encodeDone = false;
  if( 0 != vvenc_encode( enc, pcYuvPicture, AU, &encodeDone))
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


int invalidInputUninitialzedInputPic( )
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

int invalidInputInvalidPicSize( )
{
  int16_t dummy = 0;

  vvencYUVBuffer *pcYuvPicture = vvenc_YUVBuffer_alloc();
  pcYuvPicture->planes[0].ptr = &dummy;
  pcYuvPicture->planes[1].ptr = &dummy;
  pcYuvPicture->planes[2].ptr = &dummy;

  if( 0 != inputBufTest( pcYuvPicture ))
  {
    vvenc_YUVBuffer_free( pcYuvPicture, false );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, false );

  return 0;
}

int invalidInputInvalidLumaStride( )
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
    vvenc_YUVBuffer_free( pcYuvPicture, false );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, false );
  return 0;
}


int invalidInputInvalidChromaStride( )
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
    vvenc_YUVBuffer_free( pcYuvPicture, false );
    return -1;
  }

  vvenc_YUVBuffer_free( pcYuvPicture, false );
  return 0;
}


int invalidldInputBuf( )
{
  vvenc_config vvencParams;
  vvenc_config_default( &vvencParams );

  fillEncoderParameters( vvencParams );

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  if( 0 != vvenc_encoder_open( enc, &vvencParams ) )
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
  testfunc( "invalidInputUninitialzedInputPic",              &invalidInputUninitialzedInputPic,         true );
  testfunc( "invalidInputInvalidPicSize",                    &invalidInputInvalidPicSize,               true );

  testfunc( "invalidInputInvalidPicSize",                    &invalidInputInvalidPicSize,               true );
  testfunc( "invalidInputInvalidLumaStride",                 &invalidInputInvalidLumaStride,            true );
  testfunc( "invalidInputInvalidChromaStride",               &invalidInputInvalidChromaStride,          true );
 
  return 0;
}


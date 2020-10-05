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


/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include <chrono>
#include <ctime>

#include "vvenc/version.h"
#include "../vvencFFapp/EncApp.h"
#include "../vvencFFapp/ParseArg.h"

//! \ingroup EncoderApp
//! \{

#if _DEBUG
#define HANDLE_EXCEPTION 0
#else
#define HANDLE_EXCEPTION 1
#endif

// ====================================================================================================================
// Main function
// ====================================================================================================================


int main(int argc, char* argv[])
{
  vvenc::setMsgFnc( &msgFnc );

  std::string simdOpt;
  VVCEncoderFFApp::df::program_options_lite::Options opts;
  opts.addOptions()
    ( "c",           VVCEncoderFFApp::df::program_options_lite::parseConfigFile, "" )
    ( "Verbosity,v", g_verbosity,                               "" )
    ( "SIMD",        simdOpt,                                   "" );
  VVCEncoderFFApp::df::program_options_lite::SilentReporter err;
  VVCEncoderFFApp::df::program_options_lite::scanArgv( opts, argc, ( const char** ) argv, err );

  simdOpt = vvenc::setSIMDExtension( simdOpt );

  // print information
  msgApp( INFO, "\n");
  msgApp( INFO, "vvencFFapp: Encoder Version %s ", VVENC_VERSION );
  msgApp( INFO, "%s", getCompileInfoString().c_str() );
  msgApp( INFO, "[SIMD=%s]", simdOpt.c_str() );
  if ( vvenc::isTracingEnabled() )
  {
    msgApp( INFO, "[ENABLE_TRACING]" );
  }
  msgApp( INFO, "\n" );

  EncApp* pcEncApp = new EncApp;

  // parse configuration
  if ( ! pcEncApp->parseCfg( argc, argv ) )
  {
    return 1;
  }

  // starting time
  auto startTime  = std::chrono::steady_clock::now();
  std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  msgApp( INFO, " started @ %s", std::ctime(&startTime2) );
  clock_t startClock = clock();

  // call encoding function
#if HANDLE_EXCEPTION
  try
  {
#endif
    pcEncApp->encode();
#if HANDLE_EXCEPTION
  }
  catch( std::exception &e )
  {
    msgApp( ERROR, "%s\n", e.what() );
    return 1;
  }
  catch( ... )
  {
    msgApp( ERROR, "Unspecified error occurred\n" );
    return 1;
  }
#endif
  // ending time
  clock_t endClock = clock();
  auto endTime = std::chrono::steady_clock::now();
  std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  auto encTime = std::chrono::duration_cast<std::chrono::milliseconds>( endTime - startTime).count();

  delete pcEncApp;

  msgApp( INFO, "\n finished @ %s", std::ctime(&endTime2) );
  msgApp( INFO, " Total Time: %12.3f sec. [user] %12.3f sec. [elapsed]\n", (endClock - startClock) * 1.0 / CLOCKS_PER_SEC, encTime / 1000.0);

  return 0;
}

//! \}

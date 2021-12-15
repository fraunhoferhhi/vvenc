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


/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include <chrono>
#include <ctime>

#include "../vvencFFapp/EncApp.h"
#include "apputils/ParseArg.h"

#include "vvenc/vvenc.h"

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Main function
// ====================================================================================================================


int main(int argc, char* argv[])
{
  vvenc_set_logging_callback( nullptr, msgFnc ); // register global log callback ( deprecated, will be removed)

  std::string simdOpt;
  std::string verbosity = std::to_string(VVENC_VERBOSE);
  bool bShowVersion = false;
  apputils::df::program_options_lite::Options opts;
  opts.addOptions()
    ( "c",           apputils::df::program_options_lite::parseConfigFile, "" )
    ( "SIMD",        simdOpt,         "" )
    ( "version",     bShowVersion,    "show version ")
    ( "Verbosity,v", verbosity,       "Specifies the level of the verboseness (0: silent, 1: error, 2: warning, 3: info, 4: notice, 5: verbose, 6: debug)");

  apputils::df::program_options_lite::SilentReporter err;
  apputils::df::program_options_lite::scanArgv( opts, argc, ( const char** ) argv, err );

  if( bShowVersion )
  {
    msgApp( VVENC_INFO,"vvencFFapp version %s\n", vvenc_get_version() );
    return 0;
  }

  if( std::isdigit(verbosity[0]))
  {
    g_verbosity = (vvencMsgLevel)std::atoi( verbosity.c_str() );
  }
  else
  {
    if     ( verbosity == "silent")  g_verbosity = VVENC_SILENT;
    else if( verbosity == "error")   g_verbosity = VVENC_ERROR;
    else if( verbosity == "warning") g_verbosity = VVENC_WARNING;
    else if( verbosity == "info")    g_verbosity = VVENC_INFO;
    else if( verbosity == "notice")  g_verbosity = VVENC_NOTICE;
    else if( verbosity == "verbose") g_verbosity = VVENC_VERBOSE;
    else if( verbosity == "debug")   g_verbosity = VVENC_DETAILS;
  }

  vvenc_set_SIMD_extension( simdOpt.c_str() );
  msgApp( VVENC_INFO, "vvencFFapp: %s\n", vvenc_get_enc_information( nullptr) );

  EncApp* pcEncApp = new EncApp;
  //g_vvencEncApp = (vvencEncApp*)pcEncApp;

  // parse configuration
  if ( ! pcEncApp->parseCfg( argc, argv ) )
  {
    return 1;
  }

  // starting time
  auto startTime  = std::chrono::steady_clock::now();
  std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  msgApp( VVENC_INFO, " started @ %s", std::ctime(&startTime2) );
  clock_t startClock = clock();

  // call encoding function
  int ret = pcEncApp->encode();

  // ending time
  clock_t endClock = clock();
  auto endTime = std::chrono::steady_clock::now();
  std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  auto encTime = std::chrono::duration_cast<std::chrono::milliseconds>( endTime - startTime).count();

  delete pcEncApp;

  msgApp( VVENC_INFO, "\n finished @ %s", std::ctime(&endTime2) );
  msgApp( VVENC_INFO, " Total Time: %12.3f sec. [user] %12.3f sec. [elapsed]\n", (endClock - startClock) * 1.0 / CLOCKS_PER_SEC, encTime / 1000.0);

  return ret;
}

//! \}

/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

/** \file     CommonDefARM.cpp
 */

#include "CommonDefARM.h"

#if defined( __linux__ )
#include <sys/auxv.h>  // getauxval
#endif

namespace vvenc
{
using namespace arm_simd;

const static std::vector<std::pair<ARM_VEXT, std::string>> vext_names{
  { UNDEFINED, ""       },
  { SCALAR,    "SCALAR" },
  { NEON,      "NEON"   },
  { SVE,       "SVE"    },
  { SVE2,      "SVE2"   },
};

const std::string& arm_vext_to_string( ARM_VEXT vext )
{
  for( auto& it : vext_names )
  {
    if( it.first == vext )
    {
      return it.second;
    }
  }
  THROW( "Invalid SIMD extension value " << vext );
}

ARM_VEXT string_to_arm_vext( const std::string& ext_name )
{
  if( ext_name.empty() )
  {
    return UNDEFINED;
  }

  for( auto& it : vext_names )
  {
    if( it.second == ext_name )
    {
      return it.first;
    }
  }

  THROW( "Invalid SIMD Mode string: \"" << ext_name << "\"" );
}

#if defined( __linux__ )

// Define hwcap values ourselves: building with an old auxv header where these
// hwcap values are not defined should not prevent features from being enabled.
#define AARCH64_HWCAP_SVE ( 1 << 22 )
#define AARCH64_HWCAP2_SVE2 ( 1 << 1 )

static ARM_VEXT _get_arm_extensions()
{
  // We assume Neon is always supported for relevant Arm processors.
  ARM_VEXT ext = NEON;

  unsigned long hwcap  = getauxval( AT_HWCAP );
  unsigned long hwcap2 = getauxval( AT_HWCAP2 );

  if( hwcap & AARCH64_HWCAP_SVE )
  {
    ext = SVE;
    if( hwcap2 & AARCH64_HWCAP2_SVE2 )
    {
      ext = SVE2;
    }
  }

  return ext;
}

#else

static ARM_VEXT _get_arm_extensions()
{
  // We assume Neon is always supported for relevant Arm processors.
  // No other extensions supported on non-Linux platforms for now.
  return NEON;
}

#endif

ARM_VEXT read_arm_extension_flags( ARM_VEXT request )
{
  static ARM_VEXT max_supported = _get_arm_extensions();
  static ARM_VEXT ext_flags     = max_supported;

  if( request != UNDEFINED )
  {
    if( request > max_supported )
    {
      THROW( "requested SIMD level (" << request << ") not supported by current CPU (max " << max_supported << ")." );
    }
    ext_flags = request;
  }

  return ext_flags;
};

const std::string& read_arm_extension_name()
{
  return arm_vext_to_string( read_arm_extension_flags() );
}

}   // namespace

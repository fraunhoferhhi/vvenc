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
/** \file     dtrace.h
 *  \brief    Implementation of trace messages support for debugging
 */

#pragma once

#include "CommonDef.h"

#include <stdio.h>
#include <string>
#include <list>
#include <map>
#include <set>
#include <vector>
#include <cstdarg>

//! \ingroup CommonLib
//! \{

namespace vvenc {

class CDTrace;

typedef std::string CType;

struct dtrace_channel
{
  int channel_number;
  std::string channel_name;
};

typedef std::vector< dtrace_channel > dtrace_channels_t;

class Condition
{
public:
    CType type;
    bool ( *eval )( int, int );
    int rval;

    Condition( CType t, bool ( *efunc )( int,int ), int refval )
    : type(t), eval(efunc), rval(refval)
    {}
};

class Channel
{
    typedef std::vector<Condition> Rule;
public:
    Channel() : rule_list(), _active(false), _counter(0) {}
    void update( std::map< CType, int > state );
    bool active() { return _active; }
    void add( Rule rule );
    void incrementCounter() { _counter++; }
    void decrementCounter() { _counter--  ; }
    int64_t getCounter() { return _counter; }
private:
    std::list< Rule > rule_list;
    bool _active;
    int64_t _counter;
};

class CDTrace
{
  typedef std::pair< CType, int > state_type;
  //friend class Rules;
private:
    bool          copy;
    FILE         *m_trace_file;
    int           m_error_code;

    typedef std::string Key;
    typedef std::vector<std::string> vstring;
    typedef std::map< Key, int > channel_map_t;
    std::vector< Channel > chanRules;
    std::set< CType > condition_types;
    std::map< CType, int > state;
    std::map< Key, int > deserializationTable;

public:
    CDTrace() : copy(false), m_trace_file(NULL) {}
    CDTrace( const char *filename, vstring channel_names );
    CDTrace( const char *filename, const dtrace_channels_t& channels );
    CDTrace( const std::string& sTracingFile, const std::string& sTracingRule, const dtrace_channels_t& channels );
    CDTrace( const CDTrace& other );
    CDTrace& operator=( const CDTrace& other );
    ~CDTrace();
    void swap         ( CDTrace& other );
    int  addRule      ( std::string rulestring );
    template<bool bCount>
    void dtrace       ( int, const char *format, /*va_list args*/... );
    void dtrace_repeat( int, int i_times, const char *format, /*va_list args*/... );
    bool update       ( state_type stateval );
    int  init( vstring channel_names );
    int  getLastError() { return m_error_code;  }
    const char*  getChannelName( int channel_number );
    void getChannelsList( std::string& sChannels );
    std::string getErrMessage();
    int64_t getChannelCounter( int channel ) { return chanRules[channel].getCounter(); }
    void    decrementChannelCounter( int channel ) { chanRules[channel].decrementCounter(); }
};

} // namespace vvenc

//! \}


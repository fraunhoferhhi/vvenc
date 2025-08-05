/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     dtrace.cpp
 *  \brief    Implementation of trace messages support for debugging
 */

#include "dtrace.h"
#include "dtrace_next.h"
#include "CommonDef.h"

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdlib>

//! \ingroup CommonLib
//! \{

namespace vvenc {

bool evaluateRules( std::list<Rule>& rule_list, state_type& stateval )
{
    bool _active = false;
    for( std::list<Rule>::iterator rules_iter = rule_list.begin();
            rules_iter != rule_list.end();
            ++rules_iter ) {
        /* iterate over conditions, get the state of the condition type
         * and check if condition is met:
         *     if not -> go to next rule
         *        yes -> go to next condition
         * if all conditions are met: set channel active and return */
        bool probe = true;

        for( Rule::iterator cond_iter = rules_iter->begin();
                cond_iter != rules_iter->end();
                ++cond_iter ) 
        {
            if( cond_iter->type == stateval.first )
            {
              int sVal = stateval.second;
              if(!cond_iter->eval( cond_iter->rval, sVal )) {
                probe = false;
                break;
              }
            }
        }
        if( probe ) {
            _active = true;
            break;
        }
    }
    return _active;
}

bool Channel::evaluate( state_type stateval )
{
  return evaluateRules( rule_list, stateval );
}

void Channel::update( state_type& stateval, bool localState )
{
  bool val = evaluateRules( rule_list, stateval );
  if( localState )
    _activeLocal = val;
  else
    _active = val;
}

void Channel::update( std::map< CType, int > state )
{

    for( std::list<Rule>::iterator rules_iter = rule_list.begin();
            rules_iter != rule_list.end();
            ++rules_iter ) {
        /* iterate over conditions, get the state of the condition type
         * and check if contion is met:
         *     if not -> go to next rule
         *        yes -> go to next condition
         * if all conditions are met: set channel active and return */
        bool probe = true;

        for( Rule::iterator cond_iter = rules_iter->begin();
                cond_iter != rules_iter->end();
                ++cond_iter ) {
            int sVal = state[cond_iter->type];
            if( !cond_iter->eval( cond_iter->rval, sVal ) ) {
                probe = false;
                break;
            }
        }
        if( probe ) {
            _active = true;
            return;
        }
    }

    _active = false;
}

void Channel::add( std::vector<Condition> rule )
{
    rule_list.push_back( rule );
}

static inline
std::vector<std::string> &split( const std::string &s, char delim, std::vector<std::string> &elems )
{
    std::stringstream ss( s );
    std::string item;
    while ( std::getline( ss, item, delim ) ) {
        elems.push_back( item );
    }
    return elems;
}

static inline
std::vector<std::string> split( const std::string &s, char delim )
{
    std::vector<std::string> elems;
    split( s, delim, elems );
    return elems;
}

CDTrace::CDTrace( const char *filename, vstring channel_names )
    : copy(false), m_trace_file(NULL), m_error_code( 0 )
{
    if( filename )
        m_trace_file = fopen( filename, "w" );

    int i = 0;
    for( vstring::iterator ci = channel_names.begin(); ci != channel_names.end(); ++ci ) {
        deserializationTable[*ci] = i++;
        chanRules.push_back( Channel() );
    }
}

CDTrace::CDTrace( const char *filename, const dtrace_channels_t& channels )
  : copy( false ), m_trace_file( NULL ), m_error_code( 0 )
{
  if( filename )
    m_trace_file = fopen( filename, "w" );

  //int i = 0;
  for( dtrace_channels_t::const_iterator ci = channels.begin(); ci != channels.end(); ++ci ) {
    deserializationTable[ci->channel_name] = ci->channel_number/*i++*/;
    chanRules.push_back( Channel() );
  }
}

CDTrace::CDTrace( const CDTrace& other )
{
    copy = true;
    m_trace_file         = other.m_trace_file;
    chanRules            = other.chanRules;
    condition_types      = other.condition_types;
    state                = other.state;
    deserializationTable = other.deserializationTable;
    m_error_code         = other.m_error_code;
}

CDTrace::CDTrace( const std::string& sTracingFile, const std::string& sTracingRule, const dtrace_channels_t& channels )
  : CDTrace( sTracingFile.c_str(), channels )
{
  //CDTrace::CDTrace( sTracingFile.c_str(), channels );
  if( !sTracingRule.empty() )
  {
    m_error_code = addRule( sTracingRule );
  }
}

void CDTrace::swap( CDTrace& other )
{
    using std::swap;
    CDTrace& first = *this;
    CDTrace& second = other;
    swap(first.copy,second.copy);
    swap(first.m_trace_file,second.m_trace_file);
    swap(first.chanRules,second.chanRules);
    swap(first.condition_types,second.condition_types);
    swap(first.state,second.state);
    swap(first.deserializationTable,second.deserializationTable);
}

CDTrace& CDTrace::operator=( const CDTrace& other )
{
    CDTrace tmp(other);
    swap( tmp );
    return *this;
}

CDTrace::~CDTrace()
{
    if( !copy && m_trace_file )
        fclose( m_trace_file );
}

bool _cf_eq ( int bound, int val ) { return ( val==bound ); }
bool _cf_neq( int bound, int val ) { return ( val!=bound ); }
bool _cf_le ( int bound, int val ) { return ( val<=bound ); }
bool _cf_ge ( int bound, int val ) { return ( val>=bound ); }

int CDTrace::addRule( std::string rulestring )
{
    vstring chans_conds = split( rulestring, ':' );
    vstring channels = split( chans_conds[0], ',' );
    vstring conditions = chans_conds.size()>1 ? split( chans_conds[1], ',' ) : vstring();

    /* parse the rules first */
    std::vector<Condition> rule;
    for( vstring::iterator ci = conditions.begin(); ci != conditions.end(); ++ci ) {
        /* find one of "==", "!=", "<=", ">=" */
        const char *ops_[] = { "==", "!=", "<=", ">=" };
        vstring operators( ops_,&ops_[sizeof( ops_ )/sizeof( ops_[0] )] );
        vstring::iterator oi = operators.begin();
        std::size_t pos = std::string::npos;
        do {
            if( ( pos = ci->find( *oi ) ) != std::string::npos ) break;
        } while( ++oi != operators.end() );

        /* No operator found, malformed rules string -> abort */
        if( pos == std::string::npos ) return -2;

        CType ctype( *ci,0,pos );
        int value = std::atoi( ci->substr( pos+2, ci->length()-( pos+2 ) ).c_str() );
        //if( condition_types.find( ctype ) == condition_types.end() ) return 0;

        /* partially apply the condition value to the associated
         * condtion function and append it to the rule */
        bool ( *cfunc )( int,int );
        if( "==" == *oi ) cfunc = _cf_eq;
        else if( "!=" == *oi ) cfunc = _cf_neq;
        else if( "<=" == *oi ) cfunc = _cf_le;
        else if( ">=" == *oi ) cfunc = _cf_ge;
        else return 0; // this is already taken care of

        rule.push_back( Condition( ctype, cfunc, value ) );
    }

    /* add the rule to each channel */
    for( vstring::iterator chan_iter = channels.begin(); chan_iter != channels.end(); ++chan_iter ) {
        std::map< Key, int>::iterator ichan = deserializationTable.find(*chan_iter);
        if( ichan != deserializationTable.end() )
            chanRules[ichan->second].add( rule );
        else
            return -3;
    }

    //return (int)channels.size();
    return 0;
}

bool CDTrace::update( state_type stateval )
{
    state[stateval.first] = stateval.second;

    /* pass over all the channel rules */
    for( std::vector< Channel >::iterator citer = chanRules.begin(); citer != chanRules.end(); ++citer )
    {
        citer->update( state );
    }

    return true;
}

bool CDTrace::updateChannel( int channel, state_type stateval )
{
    chanRules[channel].update( stateval, true );
    return true;
}

void CDTrace::getChannelsList( std::string& sChannels )
{
  sChannels.clear();
  /* pass over all the channel rules */
  if( deserializationTable.size() > 0 )
  {
    for( channel_map_t::iterator it = deserializationTable.begin(); it != deserializationTable.end(); ++it )
      sChannels += it->first + "\n";
  }
}

const char* CDTrace::getChannelName( int channel_number )
{
  static const char not_found[] = "";
  if( deserializationTable.size() > 0 )
  {
    for( channel_map_t::iterator it = deserializationTable.begin(); it != deserializationTable.end(); ++it )
      if( it->second == channel_number )
        return it->first.c_str();
  }
  return not_found;
}

std::string CDTrace::getErrMessage()
{
  std::string str = "";
  if( m_error_code )
  {
    if( m_error_code == -2 )
      str = ( " - DTrace ERROR: Add tracing rule failed: DECERR_DTRACE_BAD_RULE" );
    else if( m_error_code == -3 )
      str = ( " - DTrace ERROR: Add tracing rule failed: DECERR_DTRACE_UNKNOWN_CHANNEL" );
    else
    {
      str = " - DTrace ERROR: Undefined error";
    }
  }

  return str;
}

template< bool bCount>
void CDTrace::dtrace( int k, const char *format, /*va_list args*/... )
{
  if( m_trace_file && chanRules[k].active() )
  {
    va_list args;
    va_start ( args, format );
    vfprintf ( m_trace_file, format, args );
    fflush( m_trace_file );
    va_end ( args );
    if( bCount )
      chanRules[k].incrementCounter();
  }
  return;
}

template void CDTrace::dtrace<true>( int k, const char *format, /*va_list args*/... );
template void CDTrace::dtrace<false>( int k, const char *format, /*va_list args*/... );

void CDTrace::dtrace_repeat( int k, int i_times, const char *format, /*va_list args*/... )
{
  if( m_trace_file && chanRules[k].active() )
  {
    va_list args;
    va_start( args, format );
    while( i_times > 0 )
    {
      i_times--;
      vfprintf( m_trace_file, format, args );
    }
    fflush( m_trace_file );
    va_end( args );
  }
  return;
}


//! \}

} // namespace vvenc


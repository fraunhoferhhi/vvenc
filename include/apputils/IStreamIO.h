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

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
#pragma once
#include "apputils/apputilsDecl.h"

#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <map>
#include <vector>

#include "apputils/VVEncAppCfg.h"
#include <vvenc/vvencCfg.h>

//! \ingroup Interface
//! \{

namespace apputils {

template<typename E>
struct APPUTILS_DECL SVPair
{
  const char* str;
  E           value;
};

template<typename T>
class APPUTILS_DECL IStreamToRefVec
{
  public:
    IStreamToRefVec( std::vector<T*> v, bool _allRequired, char _sep = 'x' )
      : valVec( v )
      , sep( _sep)
      , allRequired( _allRequired)
    {
    }

    ~IStreamToRefVec()
    {
    }

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToRefVec<F>& toVec );

    template<typename F>
    friend std::ostream& operator << ( std::ostream& os, const IStreamToRefVec<F>& toVec );

  private:
    std::vector<T*> valVec;
    char sep;
    bool allRequired;
};

template<typename T>
inline std::istream& operator >> ( std::istream& in, IStreamToRefVec<T>& toVec )
{
  const size_t maxSize = toVec.valVec.size();
  size_t idx = 0;
  bool fail = false;
  // split into multiple lines if any
  while ( ! in.eof() )
  {
    std::string line;
    std::getline( in, line );
    // treat all whitespaces and commas as valid separators
    if( toVec.sep == 'x')
      replace_if( line.begin(), line.end(), []( int c ){ return isspace( c ) || c == 'x'; }, ' ' );
    else if( toVec.sep == '/')
      replace_if( line.begin(), line.end(), []( int c ){ return isspace( c ) || c == '/'; }, ' ' );
    else
      replace_if( line.begin(), line.end(), []( int c ){ return isspace( c ) || c == ','; }, ' ' );
    std::stringstream tokenStream( line );
    std::string token;
    // split into multiple tokens if any
    while( std::getline( tokenStream, token, ' ' ) )
    {
      if ( ! token.length() )
        continue;
      // convert to value
      std::stringstream convStream( token );
      T val;
      convStream >> val;
      fail |= convStream.fail();
      if( idx >= maxSize )
      {
        fail = true;//try to write behind buffer
      }
      else
      {
        *toVec.valVec[idx++] =  val;
      }
    }
  }

  if ( fail || (toVec.allRequired && idx != maxSize) )
  {
    in.setstate( std::ios::failbit );
  }

  return in;
}

template<typename T>
inline std::ostream& operator << ( std::ostream& os, const IStreamToRefVec<T>& toVec )
{
  bool bfirst = true;
  for( auto& e: toVec.valVec )
  {
    if( bfirst )
    {
      bfirst = false;
    }
    else
    {
      os << toVec.sep;
    }
    os << *e;
  }
  return os;
}

template<typename E>
class APPUTILS_DECL IStreamToEnum
{
  public:
    IStreamToEnum( E* d, const std::vector<SVPair<E>>* m )
      : dstVal ( d )
        , toMap( m )
    {
    }

    ~IStreamToEnum()
    {
    }

    template<typename F>
    friend std::ostream& operator << ( std::ostream& os, const IStreamToEnum<F>& toEnum );

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToEnum<F>& toEnum );

    const char* to_string() const
    {
      for ( const auto& map : *toMap )
      {
        if ( *dstVal == map.value )
        {
          return map.str;
        }
      }
      //msgApp( ERROR, "Unknown enum \"%s\" in to_string", *dstVal );
      return "";
    }

  private:
    E*                            dstVal;
    const std::vector<SVPair<E>>* toMap;
};

template<typename E>
inline std::istream& operator >> ( std::istream& in, IStreamToEnum<E>& toEnum )
{
  std::string str;
  in >> str;

  for ( const auto& map : *toEnum.toMap )
  {
    if ( str == map.str )
    {
      *toEnum.dstVal = map.value;
      return in;
    }
  }

  /* not found */
  in.setstate( std::ios::failbit );
  return in;
}

template<typename E>
inline std::ostream& operator << ( std::ostream& os, const IStreamToEnum<E>& toEnum )
{
  for ( const auto& map : *toEnum.toMap )
  {
    if ( *toEnum.dstVal == map.value )
    {
      os << map.str;
      return os;
    }
  }

  /* not found */
  os.setstate( std::ios::failbit );
  return os;
}

// ====================================================================================================================
// string <-> enum
// ====================================================================================================================


typedef void (*setParamFunc) (VVEncAppCfg*, vvenc_config*, int);

template<typename E>
class APPUTILS_DECL IStreamToFunc
{
  public:
    IStreamToFunc( setParamFunc func, VVEncAppCfg* appCfg, vvenc_config* encCfg, const std::vector<SVPair<E>>* m, const E _default )
      : mfunc( func )
      , mappCfg ( appCfg )
      , mencCfg( encCfg )
      , toMap( m )
      , dstVal( _default )
    {
    }

    ~IStreamToFunc()
    {
    }

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToFunc<F>& toEnum );

    template<typename F>
    friend std::ostream& operator << ( std::ostream& in, const IStreamToFunc<F>& toEnum );

    const char* to_string() const
    {
      return "";
    }

  private:
    setParamFunc                  mfunc;
    VVEncAppCfg*                  mappCfg;
    vvenc_config*                 mencCfg;
    const std::vector<SVPair<E>>* toMap;
    E                             dstVal;
};

template<typename E>
inline std::istream& operator >> ( std::istream& in, IStreamToFunc<E>& toEnum )
{
  std::string str;
  in >> str;

  for ( const auto& map : *toEnum.toMap )
  {
    if ( str == map.str )
    {
      toEnum.dstVal = map.value;
      toEnum.mfunc( toEnum.mappCfg, toEnum.mencCfg, map.value);
      return in;
    }
  }

  /* not found */
  in.setstate( std::ios::failbit );
  return in;
}

template<typename F>
inline std::ostream& operator << ( std::ostream& os, const IStreamToFunc<F>& toEnum )
{
  for ( const auto& map : *toEnum.toMap )
  {
    if ( toEnum.dstVal == map.value )
    {
      os << map.str;
      return os;
    }
  }

  /* not found */
  os.setstate( std::ios::failbit );
  return os;
}

// ====================================================================================================================
// string -> list
// ====================================================================================================================

template<typename T>
class APPUTILS_DECL IStreamToVec
{
  public:
    IStreamToVec( std::vector<T>* v )
      : valVec( v )
    {
    }

    ~IStreamToVec()
    {
    }

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToVec<F>& toVec );

    template<typename F>
    friend std::ostream& operator << ( std::ostream& in, const IStreamToVec<F>& toVec );

  private:
    std::vector<T>* valVec;
};

template<typename T>
inline std::istream& operator >> ( std::istream& in, IStreamToVec<T>& toVec )
{
  std::vector<T>* valVec = toVec.valVec;
  valVec->clear();

  bool fail = false;
  // split into multiple lines if any
  while ( ! in.eof() )
  {
    std::string line;
    std::getline( in, line );

    if( line == "[]" || line == "empty"  )
    {
      return in;    // forcing empty entry
    }
    else
    {
      // treat all whitespaces and commas as valid separators
      replace_if( line.begin(), line.end(), []( int c ){ return isspace( c ) || c == ','; }, ' ' );
      std::stringstream tokenStream( line );
      std::string token;
      // split into multiple tokens if any
      while( std::getline( tokenStream, token, ' ' ) )
      {
        if ( ! token.length() )
          continue;
        // convert to value
        std::stringstream convStream( token );
        T val;
        convStream >> val;
        fail |= convStream.fail();
        valVec->push_back( val );
      }
    }
  }

  if ( fail || (! valVec->size() ) )
  {
    in.setstate( std::ios::failbit );
  }

  return in;
}

template<typename T>
inline std::ostream& operator << ( std::ostream& os, const IStreamToVec<T>& toVec )
{
  if( (*(toVec.valVec)).empty() )
  {
    os << "[]";
    return os;
  }

  bool bfirst = true;
  for( auto& e : (*(toVec.valVec)))
  {
    if( bfirst )
    {
      bfirst = false;
    }
    else
    {
      os << ",";
    }
    os << e;
  }

  return os;
}


// ====================================================================================================================
// string -> array
// ====================================================================================================================

template<typename T>
class APPUTILS_DECL IStreamToArr
{
  public:
    IStreamToArr( T* v, size_t maxSize )
    : _valVec        ( v )
    , _maxSize       ( maxSize )
    {
    }

    ~IStreamToArr()
    {
    }

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToArr<F>& toArr );

    template<typename F>
    friend std::ostream& operator << ( std::ostream& in, const IStreamToArr<F>& toArr );

  private:
    T*     _valVec;
    size_t _maxSize;
};

template<typename T>
inline std::istream& operator >> ( std::istream& in, IStreamToArr<T>& toArr )
{
  for( size_t i = 0; i < toArr._maxSize; i++ ) memset(&toArr._valVec[i],0, sizeof(T));

  bool fail = false;
  size_t pos = 0;
  // split into multiple lines if any
  while ( ! in.eof() )
  {
    std::string line;
    std::getline( in, line );

    if( line == "[]" || line == "empty"  )
    {
      return in;    // forcing empty entry
    }
    else
    {
      // treat all whitespaces and commas as valid separators
      replace_if( line.begin(), line.end(), []( int c ){ return isspace( c ) || c == ','; }, ' ' );
      std::stringstream tokenStream( line );
      std::string token;
      // split into multiple tokens if any
      while( std::getline( tokenStream, token, ' ' ) )
      {
        if ( ! token.length() )
          continue;
        // convert to value
        std::stringstream convStream( token );
        T val;
        convStream >> val;
        fail |= convStream.fail();

        if ( pos < toArr._maxSize )
        {
          toArr._valVec[pos] = val;
        }
        pos++;
      }
    }
  }

  if ( fail || ( 0 == pos ) || (pos > toArr._maxSize) )
  {
    in.setstate( std::ios::failbit );
  }

  return in;
}

template<>
inline std::istream& operator >> ( std::istream& in, IStreamToArr<char>& toArr )
{
  for( size_t i = 0; i < toArr._maxSize; i++ ) memset(&toArr._valVec[i],0, sizeof(char));

  bool fail = false;
  size_t size = 0;
  // split into multiple lines if any
  while ( ! in.eof() )
  {
    std::string line;
    std::getline( in, line );

    if( line == "" || line == "\"\"" || line == "[]" || line == "empty" || line == "''"  )
    {
      return in;    // forcing empty entry
    }
    else
    {
      if( line.size() >= toArr._maxSize )
      {
        line.copy( toArr._valVec , toArr._maxSize-1 );
        toArr._valVec[toArr._maxSize-1] = '\0';
        fail = true;
      }
      else
      {
        line.copy( toArr._valVec , line.size()+1 );
        toArr._valVec[line.size()] = '\0';
        size = line.size();
      }
    }
  }

  if ( fail || ( 0 == size ) )
  {
    in.setstate( std::ios::failbit );
  }

  return in;
}

template<typename T>
inline std::ostream& operator << ( std::ostream& os, const IStreamToArr<T>& toArr )
{
  int size=0;
  for ( size_t i = 0; i < toArr._maxSize; i++ )
  {
    if( toArr._valVec[i] != 0 ) size++;
    else break;
  }

  if( 0 == size )
  {
    os << "[]";
    return os;
  }

  bool bfirst = true;
  for ( int i = 0; i < size; i++ )
  {
    if( bfirst )
    {
      bfirst = false;
    }
    else
    {
      os << ",";
    }
    os << toArr._valVec[i];
  }

  return os;
}

template<>
inline std::ostream& operator << ( std::ostream& os, const IStreamToArr<char>& toArr )
{
  int size=0;
  for ( size_t i = 0; i < toArr._maxSize; i++ )
  {
    if( toArr._valVec[i] != '\0' ) size++;
    else break;
  }

  if( 0 == size )
  {
    os << "''";
    return os;
  }

  for ( int i = 0; i < size; i++ )
  {
    os << toArr._valVec[i];
  }

  return os;
}

// ====================================================================================================================
// vvencGOPEntry
// ====================================================================================================================


inline std::ostream& operator<< ( std::ostream& os, const vvencGOPEntry& entry )
{
  os << entry.m_sliceType;
  os << entry.m_POC;
  os << entry.m_QPOffset;
  os << entry.m_QPOffsetModelOffset;
  os << entry.m_QPOffsetModelScale;
  os << entry.m_CbQPoffset;
  os << entry.m_CrQPoffset;
  os << entry.m_QPFactor;
  os << entry.m_tcOffsetDiv2;
  os << entry.m_betaOffsetDiv2;
  os << entry.m_CbTcOffsetDiv2;
  os << entry.m_CbBetaOffsetDiv2;
  os << entry.m_CrTcOffsetDiv2;
  os << entry.m_CrBetaOffsetDiv2;
  os << entry.m_temporalId;

  for( int l = 0; l < 2; l++)
  {
    os <<  entry.m_numRefPicsActive[l];
    os <<  entry.m_numRefPics[l];
    for ( int i = 0; i < entry.m_numRefPics[l]; i++ )
    {
      os <<  entry.m_deltaRefPics[l][i];
    }
  }

  return os;
}

inline std::istream& operator>> ( std::istream& in, vvencGOPEntry& entry )
{
  in >> entry.m_sliceType;
  in >> entry.m_POC;
  in >> entry.m_QPOffset;
  in >> entry.m_QPOffsetModelOffset;
  in >> entry.m_QPOffsetModelScale;
  in >> entry.m_CbQPoffset;
  in >> entry.m_CrQPoffset;
  in >> entry.m_QPFactor;
  in >> entry.m_tcOffsetDiv2;
  in >> entry.m_betaOffsetDiv2;
  in >> entry.m_CbTcOffsetDiv2;
  in >> entry.m_CbBetaOffsetDiv2;
  in >> entry.m_CrTcOffsetDiv2;
  in >> entry.m_CrBetaOffsetDiv2;
  in >> entry.m_temporalId;

  for( int l = 0; l < 2; l++)
  {
    in >> entry.m_numRefPicsActive[l];
    in >> entry.m_numRefPics[l];
    for ( int i = 0; i < entry.m_numRefPics[l]; i++ )
    {
      in >> entry.m_deltaRefPics[l][i];
    }
  }

  return in;
}


} // namespace

//! \}


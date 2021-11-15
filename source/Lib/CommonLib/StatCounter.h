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
#pragma once


#include <string>
#include <ostream>
#include <vector>
#include <cstdarg>
#include <map>

#ifndef THROW
#define THROW(x)            throw( Exception( "ERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#endif
#ifndef CHECK
#define CHECK(c,x)          if(c){ THROW(x); }
#endif

namespace vvenc {

namespace StatCounters
{
////////////////////////////////////////////////////////////////////////////////////
// StatCounter

template<typename T>
class Counter
{
public:
  Counter()                                       : m_counter( 0 )        {}
  Counter( T init_val )                           : m_counter( init_val ) {}
  Counter( T init_val, int id )                   : m_counter( init_val ) {}
  ~Counter(){}

  Counter& operator+=( const Counter& other ) {
    m_counter += other.m_counter;
    return *this;
  }

  Counter& operator+=( const T& other ) {
    m_counter += other;
    return *this;
  }

  Counter& operator-=( const Counter& other ) {
    m_counter -= other.m_counter;
    return *this;
  }

  Counter& operator= ( const T& other )
  {
    m_counter = other;
    return *this;
  }

  Counter& operator/= ( const T& other )
  {
    m_counter /= other;
    return *this;
  }
  bool operator<  ( const T& other ) { return    m_counter < other; }
  bool operator>  ( const T& other ) { return    m_counter > other; }
  bool operator>= ( const T& other ) { return !( m_counter < other ); }
  bool operator<= ( const T& other ) { return !( m_counter > other ); }

  Counter& operator++()
  {
    ++m_counter;
    return *this;
  }

  Counter& operator++(int)
  {
    m_counter++;
    return *this;
  }

  friend std::ostream& operator<<( std::ostream& os, const Counter<T>& cnt )
  {
    os << cnt.m_counter;
    return os;
  }

  friend std::istream& operator>>( std::istream& os, Counter<T>& cnt )
  {
    os >> cnt.m_counter;
    return os;
  }

  void reset( T val )
  {
    m_counter = val;
  }

  T&                val()                       { return m_counter; }
  const T&          val() const                 { return m_counter; }

protected:
  T           m_counter;
};


template<typename T>
class StatCounter : public Counter<T>
{
public:
  StatCounter()                                       : Counter<T>()          , m_counter_name( "" )  , m_counter_id( 0 ) , m_dependenceIdx( -1 ), m_isPercentageOutput( false ) {}
  StatCounter( T init_val )                           : Counter<T>( init_val ), m_counter_name( "" )  , m_counter_id( 0 ) , m_dependenceIdx( -1 ), m_isPercentageOutput( false ) {}
  StatCounter( T init_val, const char* name )         : Counter<T>( init_val ), m_counter_name( name ), m_counter_id( 0 ) , m_dependenceIdx( -1 ), m_isPercentageOutput( false ) {}
  StatCounter( T init_val, const char* name, int id ) : Counter<T>( init_val ), m_counter_name( name ), m_counter_id( id ), m_dependenceIdx( -1 ), m_isPercentageOutput( false ) {}
  StatCounter( T init_val, int id )                   : Counter<T>( init_val ), m_counter_name( "" )  , m_counter_id( id ), m_dependenceIdx( -1 ), m_isPercentageOutput( false ) {}
  ~StatCounter(){}

  std::ostream& name( std::ostream& os )
  {
    os << Counter<T>::m_counter;
    return os;
  }

  void              setName( const char* name )        { m_counter_name = std::string( name ); }
  void              setName( const std::string& name ) { m_counter_name = name; }
  const std::string getName() const             { return m_counter_name; }
  int               id()                        { return m_counter_id; }
  const int&        id() const                  { return m_counter_id; }
  double            percentageFrom( const T& v ) const                         { return (double)Counter<T>::m_counter * 100.0 / (double)v; }
  double            percentageFrom( const StatCounter<T>& other ) const        { return (double)Counter<T>::m_counter * 100.0 / (double)other.m_counter; }
  double            diffAndPercentageFrom( const StatCounter<T>& other ) const { return (double)(Counter<T>::m_counter - other.m_counter) * 100.0 / (double)other.m_counter; }
  void              setPercentageDependence( size_t idx )                      { m_dependenceIdx = idx, m_isPercentageOutput = true; }
  bool              isPercentageOutput() const { return m_isPercentageOutput; }
  size_t            getDependenceIdx()   const { return m_dependenceIdx;  }

  std::ostream& streamOutValueInPercentage( std::ostream& os, T dep, size_t w ) const
  {
    double percentage = dep ? percentageFrom( dep ) : 0.0;
    os << std::fixed << std::setw( w - 1 ) << std::setprecision( 1 ) << percentage << "%";
 
    return os;
  }
  std::ostream& streamOutValueInPercentage( std::ostream& os, const StatCounter<T>& cntDep, size_t w ) const
  {
    streamOutValueInPercentage( os, cntDep.val(), w );
    return os;
  }

private:
  std::string m_counter_name;
  int         m_counter_id;
  size_t      m_dependenceIdx;
  bool        m_isPercentageOutput;
};


////////////////////////////////////////////////////////////////////////////////////
// StatCountersSet

template<typename T>
class StatCountersSet
{
public:
  StatCountersSet(){}
  StatCountersSet( size_t num_counters, T init_val = 0 ){ m_counters.resize( num_counters, init_val ); }
  StatCountersSet( std::vector<std::string> cntNames, T init_val = 0 )
  {
    for( int i = 0; i < cntNames.size(); i++ )
    {
      addCounter( i, cntNames[i], init_val );
    }
  }

  ~StatCountersSet(){}

  void addCounter( const int id, const std::string& name, T init_val = 0 )
  {
    m_counters.push_back( StatCounter<T>{ init_val, name.c_str(), id } );
  }

  void resize( size_t num_counters, T init_val )
  {
      m_counters.resize( num_counters, init_val );
      for( size_t i = 0; i < m_counters.size(); i++ )
        m_counters[i].setName( std::to_string( i ) );
  }

  StatCountersSet& operator+=( const StatCountersSet& other ) {
    auto i1 = m_counters.begin();
    auto i2 = other.m_counters.cbegin();
    for( ; i1 != m_counters.end() && i2 != other.m_counters.cend(); ++i1, ++i2 ) {
      *i1 += *i2;
    }
    return *this;
  }

  StatCountersSet& operator-=( const StatCountersSet& other ) {
    auto i1 = m_counters.begin();
    auto i2 = other.m_counters.cbegin();
    for( ; i1 != m_counters.end() && i2 != other.m_counters.cend(); ++i1, ++i2 ) {
      *i1 -= *i2;
    }
    return *this;
  }

  StatCountersSet& operator= ( const T& other )
  {
    m_counters[0] = other;
    return *this;
  }

  StatCountersSet& operator/= ( const T& other )
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
      m_counters[i] /= other;
    return *this;
  }

  StatCountersSet& operator= ( const StatCountersSet& other )
  {
    CHECK( m_counters.size() != other.m_counters.size(), "Accessing counters with differen number of elements!" );
    for( size_t i = 0; i < m_counters.size(); i++ ) m_counters[i] = other.m_counters[i];
    return *this;
  }

  StatCounter<T>&       operator[]( std::size_t idx )       { return m_counters[idx]; }
  const StatCounter<T>& operator[]( std::size_t idx ) const { return m_counters[idx]; }

  void inc( int cntIdx )
  {
    ++m_counters[cntIdx];
  }

  void add( int cntIdx, T val )
  {
    m_counters[cntIdx] += val;
  }

  void reset( T val )
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
      m_counters[i].reset( val );
  }

  void scale( T val )
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
      m_counters[i] /= val;
  }

  size_t size() const { return m_counters.size();  }

  T total( T accum = 0 )
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
      accum += m_counters[i].val();
    //accum = std::accumulate( m_counters.begin(), m_counters.end() - 1, accum, std::plus<StatCounter<T>>() );
    return accum;
  }

  T sumUp( size_t fromPos = 0, T accum = 0 )
  {
    if( fromPos >= 0 && fromPos < m_counters.size() )
    {
      for( size_t i = fromPos; i < m_counters.size(); i++ )
        accum += m_counters[i].val();
    }
    return accum;
  }

  std::ostream& streamOutNames( std::ostream& os, size_t w ) const
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
      os << std::setw( (m_counters[i].isPercentageOutput() ? w*2: w) ) << m_counters[i].getName();
    return os;
  }

  std::ostream& streamOutValuesInPercentage( std::ostream& os, T dep, int w ) const
  {
      for( size_t i = 0; i < m_counters.size(); i++ )
      {
          os << std::setw( w ) << m_counters[i];
          m_counters[i].streamOutValueInPercentage( os, dep, w );
      }
      return os;
  }

  std::ostream& streamOutValuesInPercentage( std::ostream& os, const StatCounter<T>& cntDep, int w ) const
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
    {
      os << std::setw( w ) << m_counters[i];
      m_counters[i].streamOutValueInPercentage( os, cntDep, w );
    }
    return os;
  }

  std::ostream& streamOutValues( std::ostream& os, size_t w ) const
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
    {
        os << std::setw( w ) << m_counters[i];
        if( m_counters[i].isPercentageOutput() )
          m_counters[i].streamOutValueInPercentage( os, m_counters[m_counters[i].getDependenceIdx()].val(), w );
    }
    return os;
  }

  std::ostream& streamOutResults( std::ostream& os, size_t w, bool isHorizontal = true )
  {
      if( isHorizontal )
      {
          //streamOutNames( os, w );
          streamOutValues( os, w );
      }
      else
      {
          size_t maxW = 0;
          for( size_t i = 0; i < m_counters.size(); i++ )
              if( m_counters[i].getName().length() > maxW )
                  maxW = m_counters[i].getName().size();

          for( size_t i = 0; i < m_counters.size(); i++ )
          {
              os << std::setw( maxW > w ? maxW: w ) << m_counters[i].getName() << ": ";
              os << std::setw( w ) << m_counters[i];
              if( m_counters[i].isPercentageOutput() )
                m_counters[i].streamOutValueInPercentage( os, m_counters[m_counters[i].getDependenceIdx()].val(), w );
              os << std::endl;
          }
      }
      return os;
  }

  std::ostream& streamOutValuesInPercentage( std::ostream& os )
  {
      T totalSum = total();
      for( size_t i = 0; i < m_counters.size(); i++ )
          os << std::fixed << std::setw( 6 ) << std::setprecision( 1 ) << m_counters[i].percentageFrom( totalSum );
      return os;
  }

  inline bool isEmpty() { for( size_t i = 0; i < m_counters.size(); i++ ){ if( m_counters[i].val() ) return false; } return true; }


  //template<typename T>
  friend std::ostream& operator<<( std::ostream& os, const StatCountersSet& cnt )
  {
    if( !cnt.m_counters.empty() )
    {
      size_t i = 0;
      for( ; i < cnt.m_counters.size() - 1; i++ )
        os << cnt.m_counters[i] << " ";

      os << cnt.m_counters[i];
    }
    return os;
  }

  friend std::istream& operator>>( std::istream& os, StatCountersSet& cnt )
  {
    if( !cnt.m_counters.empty() )
    {
      size_t i = 0;
      for( ; i < cnt.m_counters.size() - 1; i++ )
        os >> cnt.m_counters[i];

      os >> cnt.m_counters[i];
    }
    return os;
  }

protected:
  std::vector<StatCounter<T>> m_counters;
};

template<typename T>
class StatCounterSetMapped : public StatCountersSet<T>
{
public:
  StatCounterSetMapped() {}
  StatCounterSetMapped( std::vector<int> cntId, T init_val = 0 )
  {
    for( int i = 0; i < cntId.size(); i++ )
      addCounter( cntId[i], std::to_string( i ), init_val );
  }
  StatCounterSetMapped( std::vector<int> cntId, std::vector<std::string> cntNamesLUT, T init_val = 0 )
  {
    for( int i = 0; i < cntId.size(); i++ )
    {
      CHECK( cntId[i] >= cntNamesLUT.size(), "Name for id failed!" );
      addCounter( cntId[i], cntNamesLUT[cntId[i]], init_val );
    }
  }
  StatCounterSetMapped( std::vector<std::string> cntNamesLUT, T init_val = 0 )
  {
    for( int i = 0; i < cntNamesLUT.size(); i++ )
    {
      addCounter( i, cntNamesLUT[i], init_val );
    }
  }
  ~StatCounterSetMapped() {}

  void addCounter( const int id, const std::string& name, T init_val = 0 )
  {
    StatCountersSet<T>::addCounter( id, name, init_val );
    m_id_map.insert( std::make_pair( id, m_id_map.size() ) );
  }
  size_t cntPos( int cntIdx )
  {
    auto it = m_id_map.find( cntIdx );
    if( it != m_id_map.end() )
      return it->second;
    else
      THROW( "Not found" );
  }
  StatCounter<T>&       operator[]( int id )       { return StatCountersSet<T>::m_counters[cntPos( id )]; }
  const StatCounter<T>& operator[]( int id ) const { return StatCountersSet<T>::m_counters[cntPos( id )]; }
  StatCounter<T>&       cnt( int idx )       { return StatCountersSet<T>::m_counters[idx]; }
  const StatCounter<T>& cnt( int idx ) const { return StatCountersSet<T>::m_counters[idx]; }
  void inc( int id )
  {
    ++StatCountersSet<T>::m_counters[cntPos( id )];
  }
  void add( int id, T val )
  {
    StatCountersSet<T>::m_counters[cntPos( id )] += val;
  }
protected:
  std::map<int,size_t> m_id_map;
};


////////////////////////////////////////////////////////////////////////////////////
// Counter2D

template<typename T>
class Counter2D
{
public:
  Counter2D() {}
  Counter2D( size_t xDim, size_t yDim )
  {
    init( xDim, yDim );
  }
  ~Counter2D(){}

  void init( size_t xDim, size_t yDim )
  {
    m_counters.resize( yDim );
    for( size_t y = 0; y < yDim; y++ )
    {
      m_counters[y].resize( xDim );
    }
  }
public:
  T total() const
  {
    T accum = 0;
    for( size_t y = 0; y < m_counters.size(); y++ )
    {
      for( size_t x = 0; x < m_counters[y].size(); x++ )
      {
        accum += m_counters[y][x];
      }
    }
    return accum;
  }

  void reset( const T val )
  {
    for( size_t y = 0; y < m_counters.size(); y++ )
    {
      for( size_t x = 0; x < m_counters[y].size(); x++ )
      {
        m_counters[y][x] = val;
      }
    }
  }

  std::vector<T>&       operator[]( size_t y )       { return m_counters[y]; }
  const std::vector<T>& operator[]( size_t y ) const { return m_counters[y]; }

  Counter2D& operator+=( const Counter2D& other ) {
    for( size_t y = 0; y < m_counters.size(); y++ )
    {
      for( size_t x = 0; x < m_counters[y].size(); x++ )
      {
        m_counters[y][x] += other.m_counters[y][x];
      }
    }
    return *this;
  }

  Counter2D& operator/= ( const T& val )
  {
    for( size_t y = 0; y < m_counters.size(); y++ )
    {
      for( size_t x = 0; x < m_counters[y].size(); x++ )
      {
        m_counters[y][x] /= val;
      }
    }
    return *this;
  }

protected:
  std::vector<std::vector<T>> m_counters;
};

////////////////////////////////////////////////////////////////////////////////////
// StatCounter2D

template<typename T>
class StatCounter2D : public Counter2D<T>
{
public:
  StatCounter2D()                                                      : Counter2D<T>()            , m_counter_name( "" ),   m_counter_id( -1 ) {}
  StatCounter2D( size_t xDim, size_t yDim )                            : Counter2D<T>( xDim, yDim ), m_counter_name( "" ),   m_counter_id( -1 ) {}
  StatCounter2D( size_t xDim, size_t yDim , const char* name )         : Counter2D<T>( xDim, yDim ), m_counter_name( name ), m_counter_id( -1 ) {}
  StatCounter2D( size_t xDim, size_t yDim , const char* name, int id ) : Counter2D<T>( xDim, yDim ), m_counter_name( name ), m_counter_id( id ) {}
  StatCounter2D( size_t xDim, size_t yDim , int id )                   : Counter2D<T>( xDim, yDim ), m_counter_name( "" ),   m_counter_id( id ) {}
  ~StatCounter2D(){}

  std::ostream& name( std::ostream& os )
  {
    os << Counter2D<T>::m_counters;
    return os;
  }

  void              setName( const char* name )        { m_counter_name = std::string( name ); }
  void              setName( const std::string& name ) { m_counter_name = name; }
  const std::string getName() const             { return m_counter_name; }
  int               id()                        { return m_counter_id; }
  const int&        id() const                  { return m_counter_id; }
private:
  std::string m_counter_name;
  int         m_counter_id;
  size_t      m_dependenceIdx;
  bool        m_isPercentageOutput;
};

////////////////////////////////////////////////////////////////////////////////////
// StatCountersSet

template<typename T>
class StatCounter2DSet
{
public:
  StatCounter2DSet(){}
  StatCounter2DSet( size_t num_counters, size_t xDim, size_t yDim , T init_val = 0 )
  { 
    m_counters.resize( num_counters ); 
    reset( init_val );
    m_xDim = xDim;
    m_yDim = yDim;
  }
  StatCounter2DSet( const std::vector<std::string> cntNames, size_t xDim, size_t yDim, T init_val = 0 )
  {
    init( cntNames, xDim, yDim, init_val );
  }
  ~StatCounter2DSet(){}

  void init( const std::vector<std::string>& cntNames, size_t xDim, size_t yDim, T init_val = 0 )
  {
    for( int i = 0; i < cntNames.size(); i++ )
    {
      addCounter( i, cntNames[i], xDim, yDim, init_val );
    }
    m_xDim = xDim;
    m_yDim = yDim;
  }
  void init( unsigned numCounters, size_t xDim, size_t yDim, T init_val = 0 )
  {
    for( int i = 0; i < numCounters; i++ )
    {
      addCounter( i, "", xDim, yDim, init_val );
    }
    m_xDim = xDim;
    m_yDim = yDim;
  }
  void init( size_t xDim, size_t yDim )
  {
    m_xDim = xDim;
    m_yDim = yDim;
  }

  void addCounter( const int id, const std::string& name, size_t xDim, size_t yDim, T init_val = 0 )
  {
    m_counters.push_back( StatCounter2D<T>{ xDim, yDim, name.c_str(), id } );
    m_counters.back().reset( init_val );
  }

  void reset( T val = 0 )
  {
    for( int i = 0; i < m_counters.size(); i++ ) 
      m_counters[i].reset( val ); 
  }

  void scale( T val )
  {
    for( size_t j = 0; j < m_counters.size(); j++ )
      m_counters[j] /= val;
  }

  T total( size_t xDim, size_t yDim ) const
  {
    T accum = 0;
    for( int i = 0; i < m_counters.size(); i++ )
    {
      accum += m_counters[i][yDim][xDim];
    }
    return accum;
  }

  T total( size_t cntId ) const
  {
    return m_counters[cntId].total();
  }

  T total() const
  {
    T accum = 0;
    for( int i = 0; i < m_counters.size(); i++ )
    {
      accum += m_counters[i].total();
    }
    return accum;
  }

  void addOnXAxis( size_t xPosSrc1, size_t xPosSrc2, size_t xPosDst )
  {
    for( size_t i = 0; i < m_counters.size(); i++ )
    {
      for( size_t j = 0; j < m_yDim; j++ )
        m_counters[i][j][xPosDst] = m_counters[i][j][xPosSrc1] + m_counters[i][j][xPosSrc2];
    }
  }

  StatCounter2D<T>&       operator[]( size_t id )       { return m_counters[id]; }
  const StatCounter2D<T>& operator[]( size_t id ) const { return m_counters[id]; }

  StatCounter2DSet& operator+=( const StatCounter2DSet& other ) {
    auto i1 = m_counters.begin();
    auto i2 = other.m_counters.cbegin();
    for( ; i1 != m_counters.end() && i2 != other.m_counters.cend(); ++i1, ++i2 ) {
      *i1 += *i2;
    }
    return *this;
  }
  std::vector<StatCounter2D<T>>&       getCounters()       { return m_counters; }
  const std::vector<StatCounter2D<T>>& getCounters() const { return m_counters; }
  size_t getDimHor()      const { return m_xDim; }
  size_t getDimVer()      const { return m_yDim; }
  size_t getNumCntTypes() const { return m_counters.size(); }

protected:
  std::vector<StatCounter2D<T>> m_counters;
  size_t m_xDim;
  size_t m_yDim;
};

////////////////////////////////////////////////////////////////////////////////////
// StatCounter2DSetMapped

template<typename T>
class StatCounter2DSetMapped : public StatCounter2DSet<T>
{
public:
  StatCounter2DSetMapped() {}
  StatCounter2DSetMapped( const std::vector<int>& cntId, size_t xDim, size_t yDim, T init_val = 0 )
  {
    for( int i = 0; i < cntId.size(); i++ )
      addCounter( cntId[i], std::to_string( i ), xDim, yDim, init_val );
  }
  StatCounter2DSetMapped( const std::vector<int>& cntId, const std::vector<std::string>& cntNamesLUT, size_t xDim, size_t yDim, T init_val = 0 )
  {
    init( cntId, cntNamesLUT, xDim, yDim, init_val );
  }
  StatCounter2DSetMapped( const std::vector<std::string>& cntNamesLUT, size_t xDim, size_t yDim, T init_val = 0 )
  {
    for( int i = 0; i < cntNamesLUT.size(); i++ )
    {
      addCounter( i, cntNamesLUT[i], xDim, yDim, init_val );
    }
  }

  void init( const std::vector<int>& cntId, const std::vector<std::string>& cntNamesLUT, size_t xDim, size_t yDim, T init_val = 0 )
  {
    StatCounter2DSet<T>::init( xDim, yDim );
    for( int i = 0; i < cntId.size(); i++ )
    {
      CHECK( cntId[i] >= cntNamesLUT.size(), "Name for id failed!" );
      addCounter( cntId[i], cntNamesLUT[cntId[i]], xDim, yDim, init_val );
    }
  }
  ~StatCounter2DSetMapped() {}

  void addCounter( const int id, const std::string& name, size_t xDim, size_t yDim, T init_val = 0 )
  {
    StatCounter2DSet<T>::addCounter( id, name, xDim, yDim, init_val );
    m_id_map.insert( std::make_pair( id, m_id_map.size() ) );
  }
  size_t cntPos( int cntIdx ) const
  {
    auto it = m_id_map.find( cntIdx );
    if( it != m_id_map.end() )
      return it->second;
    else
      THROW( "Not found" );
  }
  StatCounter2D<T>&       operator[]( int id )       { return StatCounter2DSet<T>::m_counters[cntPos( id )]; }
  const StatCounter2D<T>& operator[]( int id ) const { return StatCounter2DSet<T>::m_counters[cntPos( id )]; }
  StatCounter2D<T>&       cnt( int idx )       { return StatCounter2DSet<T>::m_counters[idx]; }
  const StatCounter2D<T>& cnt( int idx ) const { return StatCounter2DSet<T>::m_counters[idx]; }

protected:
  std::map<int,size_t> m_id_map;
};

template<typename T>
std::ostream& report2D( std::ostream& os, const StatCounter2DSet<T>& counters, bool axisInBlockSizes = false, bool absoluteNumbers = true, bool weightedByArea = false, bool secondColumnInPercentage = false, bool ratiosWithinSingleElement = false, int refCntId = -1 );

}

} // namespace vvenc

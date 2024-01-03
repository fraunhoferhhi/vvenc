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
/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#pragma once

#ifndef __IN_COMMONDEF_H__
#error "Include CommonDef.h not TypeDef.h"
#endif

#include <vector>
#include <utility>
#include <sstream>
#include <cstddef>
#include <cstring>
#include <assert.h>
#include <cassert>

#include "EncoderLib/EncCfg.h"

typedef vvencChromaFormat ChromaFormat;
typedef vvencSliceType    SliceType;

//! \ingroup CommonLib
//! \{

namespace vvenc {

#define JVET_M0497_MATRIX_MULT                            1 // 0: Fast method; 1: Matrix multiplication

#define FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED         1 // Some compilers fail on particular code fragments, remove this when the compiler is fixed (or new version is used)

// ====================================================================================================================
// General settings
// ====================================================================================================================

#ifndef ENABLE_VALGRIND_CODE
#define ENABLE_VALGRIND_CODE                              0 // DISABLED by default (can be enabled by project configuration or make command)
#endif

#if ENABLE_VALGRIND_CODE
#define VALGRIND_MEMCLEAR(_ref,_size)                     memset(_ref,0,(_size))
#else
#define VALGRIND_MEMCLEAR(_ref,_size)
#endif

#ifndef ENABLE_TRACING
#define ENABLE_TRACING                                    0 // DISABLED by default (enable only when debugging, requires additional runtime)
#endif

#ifndef ENABLE_TIME_PROFILING
#define ENABLE_TIME_PROFILING                             0 // DISABLED by default (can be enabled by project configuration or make command)
#endif
#ifndef ENABLE_TIME_PROFILING_MT_MODE
#define ENABLE_TIME_PROFILING_MT_MODE                   ( 0 && ENABLE_TIME_PROFILING )
#endif
#ifndef ENABLE_TIME_PROFILING_TL
#define ENABLE_TIME_PROFILING_TL                          0 // DISABLED by default (can be enabled by project configuration or make command)
#endif
#ifndef ENABLE_TIME_PROFILING_PIC_TYPES
#define ENABLE_TIME_PROFILING_PIC_TYPES                   0 // DISABLED by default (can be enabled by project configuration or make command)
#endif
#ifndef ENABLE_TIME_PROFILING_CTUS_IN_PIC
#define ENABLE_TIME_PROFILING_CTUS_IN_PIC                 0 // DISABLED by default (can be enabled by project configuration or make command)
#endif
#ifndef ENABLE_TIME_PROFILING_CU_SHAPES
#define ENABLE_TIME_PROFILING_CU_SHAPES                   0 // DISABLED by default (can be enabled by project configuration or make command)
#endif  
#define ENABLE_TIME_PROFILING_EXTENDED                    ( ENABLE_TIME_PROFILING_PIC_TYPES || ENABLE_TIME_PROFILING_TL || ENABLE_TIME_PROFILING_CTUS_IN_PIC || ENABLE_TIME_PROFILING_CU_SHAPES )

#ifndef ENABLE_CU_MODE_COUNTERS
#define ENABLE_CU_MODE_COUNTERS                           0
#endif

#define ENABLE_MEASURE_SEARCH_SPACE                       0

// ====================================================================================================================
// Debugging
// ====================================================================================================================

#define INTRA_FULL_SEARCH                                 0 ///< enables full mode search for intra estimation
#define INTER_FULL_SEARCH                                 0 ///< enables full mode search for intra estimation

#define CLEAR_AND_CHECK_TUIDX                             0 ///< add additional checks to tu-map management (not accessing the map when dirty)

// SIMD optimizations
#define SIMD_ENABLE                                       1
#define ENABLE_SIMD_OPT                                 ( SIMD_ENABLE )                                     ///< SIMD optimizations, no impact on RD performance
#define ENABLE_SIMD_OPT_MCIF                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the interpolation filter, no impact on RD performance
#define ENABLE_SIMD_OPT_BUFFER                          ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the buffer operations, no impact on RD performance
#define ENABLE_SIMD_OPT_DIST                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the distortion calculations(SAD,SSE,HADAMARD), no impact on RD performance
#define ENABLE_SIMD_OPT_AFFINE_ME                       ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for affine ME, no impact on RD performance
#define ENABLE_SIMD_OPT_ALF                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for ALF
#define ENABLE_SIMD_OPT_SAO                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for SAO
#define ENABLE_SIMD_DBLF                                ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for DBLF
#define ENABLE_SIMD_OPT_BDOF                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for BDOF
#define ENABLE_SIMD_OPT_INTRAPRED                       ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for IntraPred
#define ENABLE_SIMD_OPT_MCTF                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for MCTF
#define ENABLE_SIMD_TRAFO                               ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for Transformation
#define ENABLE_SIMD_OPT_QUANT                           ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for Quantization
#define ENABLE_SIMD_LOG2                                ( 1 && ENABLE_SIMD_OPT )                            ///< use SIMD intrisic to calculate log2

#if ENABLE_SIMD_OPT_BUFFER
#define ENABLE_SIMD_OPT_BCW                               1                                                 ///< SIMD optimization for GBi
#endif


#if defined( TARGET_SIMD_X86 ) && !defined( REAL_TARGET_X86 )
#  define SIMD_EVERYWHERE_EXTENSION_LEVEL                 SSE42
#endif

// End of SIMD optimizations

// ====================================================================================================================
// Derived macros
// ====================================================================================================================

#define DISTORTION_PRECISION_ADJUSTMENT(x)                0

// ====================================================================================================================
// Error checks
// ====================================================================================================================

// ====================================================================================================================
// Named numerical types
// ====================================================================================================================

typedef       int16_t           Pel;               ///< pixel type
typedef       int32_t           TCoeff;            ///< transform coefficient
typedef       int16_t           TCoeffSig;         ///< transform coefficient as signalled
typedef       int16_t           TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int32_t           Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint32_t          Intermediate_UInt; ///< used as intermediate value in calculations

typedef       uint64_t          SplitSeries;       ///< used to encoded the splits that caused a particular CU size
typedef       uint64_t          ModeTypeSeries;    ///< used to encoded the ModeType at different split depth

typedef       uint64_t          Distortion;        ///< distortion measurement

// ====================================================================================================================
// Enumeration
// ====================================================================================================================

#define CHROMA_400 VVENC_CHROMA_400
#define CHROMA_420 VVENC_CHROMA_420
#define CHROMA_422 VVENC_CHROMA_422
#define CHROMA_444 VVENC_CHROMA_444
#define NUM_CHROMA_FORMAT VVENC_NUM_CHROMA_FORMAT

enum ChannelType : int8_t
{
  CH_L = 0,
  CH_C = 1,
  MAX_NUM_CH = 2
};

enum ComponentID : int8_t
{
  COMP_Y          = 0,
  COMP_Cb         = 1,
  COMP_Cr         = 2,
  MAX_NUM_COMP    = 3,
  COMP_JOINT_CbCr = MAX_NUM_COMP,
  MAX_NUM_TBLOCKS = MAX_NUM_COMP
};

enum ApsType : int8_t
{
  ALF_APS = 0,
  LMCS_APS = 1,
  SCALING_LIST_APS = 2,
};

enum QuantFlags : int8_t
{
  Q_INIT           = 0x0,
  Q_USE_RDOQ       = 0x1,
  Q_RDOQTS         = 0x2,
  Q_SELECTIVE_RDOQ = 0x4,
};

//EMT transform tags
enum TransType : int8_t
{
  DCT2 = 0,
  DCT8 = 1,
  DST7 = 2,
  NUM_TRANS_TYPE = 3
};

enum MTSIdx : int8_t
{
  MTS_DCT2_DCT2 = 0,
  MTS_SKIP = 1,
  MTS_DST7_DST7 = 2,
  MTS_DCT8_DST7 = 3,
  MTS_DST7_DCT8 = 4,
  MTS_DCT8_DCT8 = 5
};

enum ISPType : int8_t
{
  NOT_INTRA_SUBPARTITIONS       = 0,
  HOR_INTRA_SUBPARTITIONS       = 1,
  VER_INTRA_SUBPARTITIONS       = 2,
  NUM_INTRA_SUBPARTITIONS_MODES = 3,
  INTRA_SUBPARTITIONS_RESERVED  = 4
};

enum SbtIdx : int8_t
{
  SBT_OFF_DCT  = 0,
  SBT_VER_HALF = 1,
  SBT_HOR_HALF = 2,
  SBT_VER_QUAD = 3,
  SBT_HOR_QUAD = 4,
  NUMBER_SBT_IDX,
  SBT_OFF_MTS, //note: must be after all SBT modes, only used in fast algorithm to discern the best mode is inter EMT
};

enum SbtPos : int8_t
{
  SBT_POS0 = 0,
  SBT_POS1 = 1,
  NUMBER_SBT_POS
};

enum SbtMode : int8_t
{
  SBT_VER_H0 = 0,
  SBT_VER_H1 = 1,
  SBT_HOR_H0 = 2,
  SBT_HOR_H1 = 3,
  SBT_VER_Q0 = 4,
  SBT_VER_Q1 = 5,
  SBT_HOR_Q0 = 6,
  SBT_HOR_Q1 = 7,
  NUMBER_SBT_MODE
};

enum TreeType : int8_t
{
  TREE_D = 0, //default tree status (for single-tree slice, TREE_D means joint tree; for dual-tree I slice, TREE_D means TREE_L for luma and TREE_C for chroma)
  TREE_L = 1, //separate tree only contains luma (may split)
  TREE_C = 2, //separate tree only contains chroma (not split), to avoid small chroma block
};

enum ModeType : int8_t
{
  MODE_TYPE_ALL = 0, //all modes can try
  MODE_TYPE_INTER = 1, //can try inter
  MODE_TYPE_INTRA = 2, //can try intra, ibc, palette
};

enum DeblockEdgeDir : int8_t
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR = 2
};

/// supported prediction type
enum PredMode : int8_t
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
  MODE_IBC                   = 2,     ///< ibc-prediction mode
  MODE_PLT                   = 3,     ///< plt-prediction mode
  NUMBER_OF_PREDICTION_MODES = 4,
};

/// reference list index
enum RefPicList : int8_t
{
  REF_PIC_LIST_0               = 0,   ///< reference list 0
  REF_PIC_LIST_1               = 1,   ///< reference list 1
  NUM_REF_PIC_LIST_01          = 2,
  REF_PIC_LIST_X               = 100  ///< special mark
};

#define L0 REF_PIC_LIST_0
#define L1 REF_PIC_LIST_1

/// distortion function index
enum DFunc : uint8_t
{
  DF_SSE             = 0,             ///< general size SSE
  DF_SSE2            = DF_SSE+1,      ///<   2xM SSE
  DF_SSE4            = DF_SSE+2,      ///<   4xM SSE
  DF_SSE8            = DF_SSE+3,      ///<   8xM SSE
  DF_SSE16           = DF_SSE+4,      ///<  16xM SSE
  DF_SSE32           = DF_SSE+5,      ///<  32xM SSE
  DF_SSE64           = DF_SSE+6,      ///<  64xM SSE
  DF_SSE128          = DF_SSE+7,      ///< 16NxM SSE

  DF_SAD             = 8,             ///< general size SAD
  DF_SAD2            = DF_SAD+1,      ///<   2xM SAD
  DF_SAD4            = DF_SAD+2,      ///<   4xM SAD
  DF_SAD8            = DF_SAD+3,      ///<   8xM SAD
  DF_SAD16           = DF_SAD+4,      ///<  16xM SAD
  DF_SAD32           = DF_SAD+5,      ///<  32xM SAD
  DF_SAD64           = DF_SAD+6,      ///<  64xM SAD
  DF_SAD128          = DF_SAD+7,      ///< 16NxM SAD

  DF_HAD             = 16,            ///< general size Hadamard
  DF_HAD2            = DF_HAD+1,      ///<   2xM HAD
  DF_HAD4            = DF_HAD+2,      ///<   4xM HAD
  DF_HAD8            = DF_HAD+3,      ///<   8xM HAD
  DF_HAD16           = DF_HAD+4,      ///<  16xM HAD
  DF_HAD32           = DF_HAD+5,      ///<  32xM HAD
  DF_HAD64           = DF_HAD+6,      ///<  64xM HAD
  DF_HAD128          = DF_HAD+7,      ///< 16NxM HAD

  DF_HAD_2SAD        = 24,            //tbd th remove

  DF_SAD_WITH_MASK   = 25,
  
  DF_HAD_fast        = 26,            ///< general size Hadamard
  DF_HAD2_fast       = DF_HAD_fast+1,      ///<   2xM fast HAD
  DF_HAD4_fast       = DF_HAD_fast+2,      ///<   4xM fast HAD
  DF_HAD8_fast       = DF_HAD_fast+3,      ///<   8xM fast HAD
  DF_HAD16_fast      = DF_HAD_fast+4,      ///<  16xM fast HAD
  DF_HAD32_fast      = DF_HAD_fast+5,      ///<  32xM fast HAD
  DF_HAD64_fast      = DF_HAD_fast+6,      ///<  64xM fast HAD
  DF_HAD128_fast     = DF_HAD_fast+7,      ///< 16NxM fast HAD

  DF_TOTAL_FUNCTIONS = 34,

  DF_SSE_WTD         = 0xf2u          // out of func scope
};

/// motion vector predictor direction used in AMVP
enum MvpDir : int8_t
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

enum TransformDirection : int8_t
{
  TRANSFORM_FORWARD              = 0,
  TRANSFORM_INVERSE              = 0,
  TRANSFORM_NUMBER_OF_DIRECTIONS = 1
  //TRANSFORM_INVERSE              = 1,
  //TRANSFORM_NUMBER_OF_DIRECTIONS = 2
};

enum CoeffScanGroupType : int8_t
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum ScalingListSize : int8_t
{
  SCALING_LIST_1x1 = 0,
  SCALING_LIST_2x2,
  SCALING_LIST_4x4,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_64x64,
  SCALING_LIST_SIZE_NUM,
  //for user define matrix
  SCALING_LIST_FIRST_CODED = SCALING_LIST_2x2,
  SCALING_LIST_LAST_CODED = SCALING_LIST_64x64
};

enum SAOMode : int8_t //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes : int8_t
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes : int8_t
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,

  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses : int8_t
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};

#define NUM_SAO_BO_CLASSES_LOG2  5
#define NUM_SAO_BO_CLASSES       (1<<NUM_SAO_BO_CLASSES_LOG2)

enum SPSExtensionFlagIndex : int8_t
{
  NUM_SPS_EXTENSION_FLAGS = 8
};

enum PPSExtensionFlagIndex : int8_t
{
  PPS_EXT__REXT           = 0,
  NUM_PPS_EXTENSION_FLAGS = 8
};

enum MergeType : int8_t
{
  MRG_TYPE_DEFAULT_N        = 0, // 0
  MRG_TYPE_SUBPU_ATMVP,
  MRG_TYPE_IBC,
  NUM_MRG_TYPE
};

//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////

enum ImvMode : int8_t
{
 IMV_OFF = 0,
 IMV_FPEL,
 IMV_4PEL,
 IMV_HPEL,
 NUM_IMV_MODES
};


// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for adaptive loop filter
class PicSym;

#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  SAOMode modeIdc; // NEW, MERGE, OFF
  int typeIdc;     // union of SAOModeMergeTypes and SAOModeNewTypes, depending on modeIdc.
  int typeAuxInfo; // BO: starting band index
  int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset() { reset(); }
  void reset();
};

struct SAOBlkParam
{
  SAOBlkParam() { reset(); }
  void reset();
  SAOOffset&       operator[](int compIdx)       { return SAOOffsets[compIdx]; }
  const SAOOffset& operator[](int compIdx) const { return SAOOffsets[compIdx]; }
private:
  SAOOffset SAOOffsets[MAX_NUM_COMP];
};

struct BitDepths
{
  const int& operator[]( const ChannelType ch) const { return recon[ch]; }
  int recon[MAX_NUM_CH]; ///< the bit depth as indicated in the SPS
};

/// parameters for deblocking filter
struct LFCUParam
{
  bool internalEdge;                     ///< indicates internal edge
  bool leftEdge;                         ///< indicates left edge
  bool topEdge;                          ///< indicates top edge
};

struct LoopFilterParam
{
  int8_t   qp[3];
  uint8_t  bs;
  uint8_t  sideMaxFiltLength;
  uint8_t  flags;

  bool filterEdge( ChannelType chType ) const { return ( flags >> chType ) & 1; }
  // chroma max filter lenght
  bool filterCMFL()                     const { return ( flags >>      5 ) & 1; }

  void setFilterEdge( ChannelType chType, int f ) { flags = ( flags & ~( 1 << chType ) ) | ( f << chType ); }
  void setFilterCMFL(                     int f ) { flags = ( flags & ~( 1 <<      5 ) ) | ( f <<      5 ); }
};

struct PictureHash
{
  std::vector<uint8_t> hash;

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const
  {
    return !(*this == other);
  }
};

class ChromaCbfs
{
public:
  ChromaCbfs()
    : Cb(true), Cr(true)
  {}
  ChromaCbfs( bool _cbf )
    : Cb( _cbf ), Cr( _cbf )
  {}
public:
  bool sigChroma( ChromaFormat chromaFormat ) const
  {
    if( chromaFormat == CHROMA_400 )
    {
      return false;
    }
    return   ( Cb || Cr );
  }
  bool& cbf( ComponentID compID )
  {
    bool *cbfs[MAX_NUM_TBLOCKS] = { nullptr, &Cb, &Cr };

    return *cbfs[compID];
  }
public:
  bool Cb;
  bool Cr;
};


enum RESHAPE_SIGNAL_TYPE : int8_t
{
  RESHAPE_SIGNAL_SDR = 0,
  RESHAPE_SIGNAL_PQ  = 1,
  RESHAPE_SIGNAL_HLG = 2,
  RESHAPE_SIGNAL_NULL = 100,
};


// ---------------------------------------------------------------------------
// exception class
// ---------------------------------------------------------------------------

class Exception : public std::exception
{
public:
  Exception( const std::string& _s ) : m_str( _s ) { }
  Exception( const Exception& _e ) : std::exception( _e ), m_str( _e.m_str ) { }
  virtual ~Exception() noexcept { };
  virtual const char* what() const noexcept { return m_str.c_str(); }
  Exception& operator=( const Exception& _e ) { std::exception::operator=( _e ); m_str = _e.m_str; return *this; }
  template<typename T> Exception& operator<<( T t ) { std::ostringstream oss; oss << t; m_str += oss.str(); return *this; }
private:
  std::string m_str;
};

// if a check fails with THROW or CHECK, please check if ported correctly from assert in revision 1196)
#define THROW(x)            throw( Exception( "ERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#define CHECK(c,x)          if(c){ THROW(x); }

#if !NDEBUG  // for non MSVC compiler, define _DEBUG if in debug mode to have same behavior between MSVC and others in debug
#ifndef _DEBUG
#define _DEBUG 1
#endif
#endif

#if defined( _DEBUG )
#define CHECKD(c,x)         if(c){ THROW(x); }
#else
#define CHECKD(c,x)
#endif // _DEBUG

// ---------------------------------------------------------------------------
// static vector
// ---------------------------------------------------------------------------

template<typename T, size_t N>
class static_vector
{
  T _arr[ N ];
  size_t _size = 0;

public:

  typedef T         value_type;
  typedef size_t    size_type;
  typedef ptrdiff_t difference_type;
  typedef T&        reference;
  typedef T const&  const_reference;
  typedef T*        pointer;
  typedef T const*  const_pointer;
  typedef T*        iterator;
  typedef T const*  const_iterator;

  static const size_type max_num_elements = N;

  static_vector()                                  { }
  static_vector( size_t N_ ) : _size( N_ )         { }
  static_vector( size_t N_, const T& _val )        { resize( N_, _val ); }
  template<typename It>
  static_vector( It _it1, It _it2 )                { while( _it1 < _it2 ) _arr[ _size++ ] = *_it1++; }
  static_vector( std::initializer_list<T> _il )
  {
    auto _src1 = _il.begin();
    auto _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }
  static_vector( const static_vector<T, N>& other ) : _size( other._size )
  {
    static_assert( std::is_trivially_copyable<T>::value, "the type has to be trivially copyable!" );

    memcpy( _arr, other._arr, sizeof( T ) * _size );
  }
  static_vector& operator=( std::initializer_list<T> _il )
  {
    _size = 0;

    auto _src1 = _il.begin();
    auto _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }
  
  void resize_noinit( size_t N_ )               { CHECKD( N_ > N, "capacity exceeded" ); _size = N_; }
  void resize( size_t N_ )                      { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = T() ; _size = N_; }
  void resize( size_t N_, const T& _val )       { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = _val; _size = N_; }
  void reserve( size_t N_ )                     { CHECKD( N_ > N, "capacity exceeded" ); }
  void push_back( const T& _val )               { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = _val; }
  void push_back( T&& val )                     { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = std::forward<T>( val ); }
  void pop_back()                               { CHECKD( _size == 0, "calling pop_back on an empty vector" ); _size--; }
  void pop_front()                              { CHECKD( _size == 0, "calling pop_front on an empty vector" ); _size--; for( int i = 0; i < _size; i++ ) _arr[i] = _arr[i + 1]; }
  void clear()                                  { _size = 0; }
  reference       at( size_t _i )               { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference at( size_t _i ) const         { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       operator[]( size_t _i )       { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference operator[]( size_t _i ) const { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       front()                       { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  const_reference front() const                 { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  reference       back()                        { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  const_reference back() const                  { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  pointer         data()                        { return _arr; }
  const_pointer   data() const                  { return _arr; }
  iterator        begin()                       { return _arr; }
  const_iterator  begin() const                 { return _arr; }
  const_iterator  cbegin() const                { return _arr; }
  iterator        end()                         { return _arr + _size; }
  const_iterator  end() const                   { return _arr + _size; };
  const_iterator  cend() const                  { return _arr + _size; };
  size_type       size() const                  { return _size; };
  size_type       byte_size() const             { return _size * sizeof( T ); }
  bool            empty() const                 { return _size == 0; }

  size_type       capacity() const              { return N; }
  size_type       max_size() const              { return N; }
  size_type       byte_capacity() const         { return sizeof(_arr); }

  iterator        insert( const_iterator _pos, const T& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  iterator dst = _arr + ( _pos - _arr );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *dst = _val;
                                                  _size++;
                                                  return dst; }

  iterator        insert( const_iterator _pos, T&& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  iterator dst = _arr + ( _pos - _arr );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *dst = std::forward<T>( _val );
                                                  _size++;
                                                  return dst; }
  template<class InputIt>
  iterator        insert( const_iterator _pos, InputIt first, InputIt last )
                                                { const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl > N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = _arr + ( _pos - _arr ); _size += numEl; iterator ret = it;
                                                  while( first != last ) *it++ = *first++;
                                                  return ret; }

  iterator        insert( const_iterator _pos, size_t numEl, const T& val )
                                                { //const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl > N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = _arr + ( _pos - _arr ); _size += numEl; iterator ret = it;
                                                  for ( int k = 0; k < numEl; k++) *it++ = val;
                                                  return ret; }
  
  void            erase( const_iterator _pos )  { iterator it   = begin() + ( _pos - 1 - begin() );
                                                  iterator last = end() - 1;
                                                  while( ++it != last ) *it = *( it + 1 );
                                                  _size--; }
};


// ---------------------------------------------------------------------------
// dynamic cache
// ---------------------------------------------------------------------------


static constexpr size_t DYN_CACHE_CHUNK_SIZE = 512;

template<typename T>
class dynamic_cache
{
  std::vector<T*> m_cache;
  std::vector<T*> m_cacheChunks;

public:

  ~dynamic_cache()
  {
    deleteEntries();
  }

  void deleteEntries()
  {
    for( auto& chunk : m_cacheChunks )
    {
      delete[] chunk;
    }

    m_cache.clear();
    m_cacheChunks.clear();
  }

  T* get()
  {
    T* ret;

    if( !m_cache.empty() )
    {
      ret = m_cache.back();
      m_cache.pop_back();
    }
    else
    {
      T* chunk = new T[DYN_CACHE_CHUNK_SIZE];

      m_cacheChunks.push_back( chunk );
      m_cache.reserve( m_cache.size() + DYN_CACHE_CHUNK_SIZE );

      for( ptrdiff_t p = 0; p < DYN_CACHE_CHUNK_SIZE; p++ )
      {
        //m_cache.push_back( &chunk[DYN_CACHE_CHUNK_SIZE - p - 1] );
        m_cache.push_back( &chunk[p] );
      }

      ret = m_cache.back();
      m_cache.pop_back();
    }

    return ret;
  }

  void cache( T* el )
  {
    m_cache.push_back( el );
  }

  void cache( std::vector<T*>& vel )
  {
    m_cache.insert( m_cache.end(), vel.begin(), vel.end() );
    vel.clear();
  }
};

typedef dynamic_cache<struct CodingUnit    > CUCache;
typedef dynamic_cache<struct TransformUnit > TUCache;

struct XUCache
{
  CUCache cuCache;
  TUCache tuCache;
};


enum SceneType : int8_t
{
  SCT_NONE          = 0,
  SCT_TL0_SCENE_CUT = 1
};

typedef struct GOPEntry : vvencGOPEntry
{
  int       m_codingNum;
  int       m_gopNum;
  int       m_defaultRPLIdx;
  int       m_mctfIndex;
  bool      m_isSTSA;
  bool      m_useBckwdOnly;
  bool      m_isStartOfGop;
  bool      m_isStartOfIntra;
  bool      m_isValid;
  bool      m_skipFirstPass;
  SceneType m_scType;

  void setDefaultGOPEntry()
  {
    vvenc_GOPEntry_default( this );
    m_codingNum        = -1;
    m_gopNum           = -1;
    m_defaultRPLIdx    = -1;
    m_mctfIndex        = -1;
    m_isSTSA           = false;
    m_useBckwdOnly     = false;
    m_isStartOfGop     = false;
    m_isStartOfIntra   = false;
    m_isValid          = false;
    m_skipFirstPass    = false;
    m_scType           = SCT_NONE;
  }

  void copyFromGopCfg( const vvencGOPEntry& cfgEntry )
  {
    this->vvencGOPEntry::operator=( cfgEntry );
  }

  GOPEntry() = default;

  GOPEntry( char sliceType, int poc, int qpOffset, double qpOffsetModelOffset, double qpOffsetModelScale, double qpFactor, int temporalId, int numRefPicsActiveL0, const std::vector<int>& deltaRefPicsL0, int numRefPicsActiveL1, const std::vector<int>& deltaRefPicsL1 )
  {
    setDefaultGOPEntry();
    m_sliceType             = sliceType;
    m_POC                   = poc;
    m_QPOffset              = qpOffset;
    m_QPOffsetModelOffset   = qpOffsetModelOffset;
    m_QPOffsetModelScale    = qpOffsetModelScale;
    m_QPFactor              = qpFactor;
    m_temporalId            = temporalId;
    m_numRefPicsActive[ 0 ] = numRefPicsActiveL0;
    m_numRefPics[ 0 ]       = (int)deltaRefPicsL0.size();
    CHECK( m_numRefPicsActive[ 0 ] > m_numRefPics[ 0 ], "try to use more active reference pictures then are available" );
    CHECK( m_numRefPics[ 0 ] > VVENC_MAX_NUM_REF_PICS,  "array index out of bounds" );
    for( int i = 0; i < m_numRefPics[ 0 ]; i++ )
    {
      m_deltaRefPics[ 0 ][ i ] = deltaRefPicsL0[ i ];
    }
    m_numRefPicsActive[ 1 ] = numRefPicsActiveL1;
    m_numRefPics[ 1 ]       = (int)deltaRefPicsL1.size();
    CHECK( m_numRefPicsActive[ 1 ] > m_numRefPics[ 1 ], "try to use more active reference pictures then are available" );
    CHECK( m_numRefPics[ 1 ] > VVENC_MAX_NUM_REF_PICS,  "array index out of bounds" );
    for( int i = 0; i < m_numRefPics[ 1 ]; i++ )
    {
      m_deltaRefPics[ 1 ][ i ] = deltaRefPicsL1[ i ];
    }
  }

} GOPEntry;


} // namespace vvenc

//! \}


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
/** \file     CommonDef.h
    \brief    Defines version information, constants and small in-line functions
*/

#pragma once

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cstdarg>
#include <functional>
#include <mutex>

#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( __i386 ) || defined( _M_IX86 )
# define REAL_TARGET_X86 1
#elif defined( __aarch64__ ) || defined( _M_ARM64 ) || defined( __arm__ ) || defined( _M_ARM )
# if defined( __aarch64__ ) || defined( _M_ARM64 )
#  define REAL_TARGET_AARCH64 1
# endif
# define REAL_TARGET_ARM 1
#elif defined( __wasm__ ) || defined( __wasm32__ )
# define REAL_TARGET_WASM 1
#endif

#if defined( TARGET_SIMD_X86 )
# ifdef _WIN32
#  include <intrin.h>
//# elif defined( __GNUC__ )
//#  include <simde/x86/sse2.h>
# endif
#endif // TARGET_SIMD_X86

#if _MSC_VER > 1000
// disable "signed and unsigned mismatch"
#pragma warning( disable : 4018 )
// disable bool coercion "performance warning"
#pragma warning( disable : 4800 )
#endif // _MSC_VER > 1000

#define __IN_COMMONDEF_H__
#include "TypeDef.h"
#undef __IN_COMMONDEF_H__

#if ENABLE_CU_MODE_COUNTERS
#include "StatCounter.h"
#endif

// MS Visual Studio before 2014 does not support required C++11 features
#ifdef _MSC_VER
#if _MSC_VER < 1900
#error "MS Visual Studio version not supported. Please upgrade to Visual Studio 2015 or higher (or use other compilers)"
#endif
#endif


// macros to selectively disable some usually useful warnings
#if defined( __GNUC__ ) && __GNUC__ >= 8 && !defined( __clang__ )
# define GCC_WARNING_DISABLE_maybe_uninitialized _Pragma("GCC diagnostic push"); \
                                                 _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"");
# define GCC_WARNING_DISABLE_class_memaccess     _Pragma("GCC diagnostic push"); \
                                                 _Pragma("GCC diagnostic ignored \"-Wclass-memaccess\"");
# define GCC_WARNING_DISABLE_array_bounds        _Pragma("GCC diagnostic push"); \
                                                 _Pragma("GCC diagnostic ignored \"-Warray-bounds\"");
# define GCC_WARNING_RESET                       _Pragma("GCC diagnostic pop");
#else
# define GCC_WARNING_DISABLE_maybe_uninitialized
# define GCC_WARNING_DISABLE_class_memaccess
# define GCC_WARNING_DISABLE_array_bounds
# define GCC_WARNING_RESET
#endif

#ifdef TARGET_SIMD_X86
# if ENABLE_SIMD_OPT
#  define SIMD_PREFETCH_T0( _s ) _mm_prefetch( (char*) ( _s ), _MM_HINT_T0 )
# else
#  define SIMD_PREFETCH_T0( _s )
# endif   // ENABLE_SIMD_OPT
#endif    // TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Platform information
// ====================================================================================================================

#ifdef __clang__
#define NVM_COMPILEDBY  "[clang %d.%d.%d]", __clang_major__, __clang_minor__, __clang_patchlevel__
#ifdef __IA64__
#define NVM_ONARCH    "[on 64-bit] "
#else
#define NVM_ONARCH    "[on 32-bit] "
#endif
#elif __GNUC__
#define NVM_COMPILEDBY  "[GCC %d.%d.%d]", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__
#ifdef __IA64__
#define NVM_ONARCH    "[on 64-bit] "
#else
#define NVM_ONARCH    "[on 32-bit] "
#endif
#endif

#ifdef __INTEL_COMPILER
#define NVM_COMPILEDBY  "[ICC %d]", __INTEL_COMPILER
#elif  _MSC_VER
#define NVM_COMPILEDBY  "[VS %d]", _MSC_VER
#endif

#ifndef NVM_COMPILEDBY
#define NVM_COMPILEDBY  "[Unk-CXX]"
#endif

#ifdef _WIN32
#define NVM_ONOS        "[Windows]"
#elif  __linux
#define NVM_ONOS        "[Linux]"
#elif  __CYGWIN__
#define NVM_ONOS        "[Cygwin]"
#elif __APPLE__
#define NVM_ONOS        "[Mac OS X]"
#else
#define NVM_ONOS        "[Unk-OS]"
#endif

#define NVM_BITS          "[%d bit]", (sizeof(void*) == 8 ? 64 : 32) ///< used for checking 64-bit O/S

#ifndef NULL
#define NULL              0
#endif

enum EAffineModel : uint8_t
{
  AFFINEMODEL_4PARAM,
  AFFINEMODEL_6PARAM,
  AFFINE_MODEL_NUM
};

static constexpr int AFFINE_ME_LIST_SIZE =                             4;
static constexpr int AFFINE_ME_LIST_SIZE_LD =                          3;
static constexpr double AFFINE_ME_LIST_MVP_TH =                        1.0;

// ====================================================================================================================
// Common constants
// ====================================================================================================================
static constexpr uint64_t   MAX_UINT64 =                  0xFFFFFFFFFFFFFFFFU;
static constexpr uint32_t   MAX_UINT =                            0xFFFFFFFFU; ///< max. value of unsigned 32-bit integer
static constexpr int      MAX_INT =                             std::numeric_limits<int>::max(); ///< max. value of signed 32-bit integer
static constexpr int      MIN_INT =                             std::numeric_limits<int>::min(); ///< min. value of signed 32-bit integer
static constexpr uint8_t  MAX_UCHAR =                                   255;
static constexpr uint8_t  MAX_SCHAR =                                   127;
static constexpr double MAX_DOUBLE =                             1.7e+308; ///< max. value of double-type value
static constexpr Distortion MAX_DISTORTION =                     std::numeric_limits<Distortion>::max();

// ====================================================================================================================
// Coding tool configuration
// ====================================================================================================================
// Most of these should not be changed - they resolve the meaning of otherwise magic numbers.

static constexpr int MAX_NUM_REF =                                     16; ///< max. number of entries in picture reference list
static constexpr int MAX_QP =                                          63;
static constexpr int MAX_QP_PERCEPT_QPA =                              42; ///< max. base QP up to which CTU or sub-CTU QPA is used instead of frame QPA
static constexpr int NOT_VALID =                                       -1;
static constexpr int MI_NOT_VALID =                                    -1;
static constexpr int MH_NOT_VALID =                                    -1;

typedef enum
{
  PREV_FRAME_1    = 0,
  PREV_FRAME_2    = 1,
} PrevFrameType;

static constexpr int NUM_QPA_PREV_FRAMES =          (int)PREV_FRAME_2 + 1;

static constexpr int AMVP_MAX_NUM_CANDS =                               2; ///< AMVP: advanced motion vector prediction - max number of final candidates
static constexpr int AMVP_MAX_NUM_CANDS_MEM =                           3; ///< AMVP: advanced motion vector prediction - max number of candidates
static constexpr int AMVP_DECIMATION_FACTOR =                           2;
static constexpr int MRG_MAX_NUM_CANDS =                                6; ///< MERGE
static constexpr int AFFINE_MRG_MAX_NUM_CANDS =                         5; ///< AFFINE MERGE
static constexpr int IBC_MRG_MAX_NUM_CANDS =                            6; ///< IBC MERGE

static constexpr int ADAPT_SR_SCALE =                                   1; ///< division factor for adaptive search range

static constexpr int MIN_TB_LOG2_SIZEY = 2;
static constexpr int MAX_TB_LOG2_SIZEY = 6;

static constexpr int MIN_TB_SIZEY = 1 << MIN_TB_LOG2_SIZEY;
static constexpr int MAX_TB_SIZEY = 1 << MAX_TB_LOG2_SIZEY;

static constexpr int MAX_NESTING_NUM_LAYER =                           64;

static constexpr int MAX_VPS_LAYERS =                                  64;
static constexpr int MAX_VPS_SUBLAYERS =                                7;
static constexpr int MAX_NUM_OLSS =                                   256;
static constexpr int MAX_VPS_OLS_MODE_IDC =                             2;

static constexpr int MIP_MAX_WIDTH =                                   MAX_TB_SIZEY;
static constexpr int MIP_MAX_HEIGHT =                                  MAX_TB_SIZEY;

static constexpr int MAX_NUM_ALF_CLASSES         =                     25;
static constexpr int MAX_NUM_ALF_LUMA_COEFF      =                     13;
static constexpr int MAX_NUM_ALF_CHROMA_COEFF    =                      7;
static constexpr int MAX_ALF_FILTER_LENGTH       =                      7;
static constexpr int MAX_ALF_PADDING_SIZE        =                      4;

static constexpr int MAX_NUM_CC_ALF_FILTERS  =                          4;
static constexpr int MAX_NUM_CC_ALF_CHROMA_COEFF    =                   8;
static constexpr int CCALF_DYNAMIC_RANGE            =                   6;
static constexpr int CCALF_BITS_PER_COEFF_LEVEL     =                   3;

static constexpr int ALF_FIXED_FILTER_NUM        =                     64;
static constexpr int ALF_CTB_MAX_NUM_APS         =                      8;
static constexpr int NUM_FIXED_FILTER_SETS       =                     16;

static constexpr int MAX_BDOF_APPLICATION_REGION =                     16;

static constexpr int MAX_CPB_CNT =                                     32; ///< Upper bound of (cpb_cnt_minus1 + 1)
static constexpr int COEF_REMAIN_BIN_REDUCTION =                        5; ///< indicates the level at which the VLC transitions from Golomb-Rice to TU+EG(k)
static constexpr int CU_DQP_TU_CMAX =                                   5; ///< max number bins for truncated unary
static constexpr int CU_DQP_EG_k =                                      0; ///< expgolomb order

static constexpr int SBH_THRESHOLD =                                    4; ///< value of the fixed SBH controlling threshold

static constexpr int MAX_NUM_VPS =                                     16;
static constexpr int MAX_NUM_DCI =                                     16;
static constexpr int MAX_NUM_SPS =                                     16;
static constexpr int MAX_NUM_PPS =                                     64;
static constexpr int MAX_NUM_APS =                                     32;  //Currently APS ID has 5 bits
static constexpr int NUM_APS_TYPE_LEN =                                 3;  //Currently APS Type has 3 bits
static constexpr int MAX_NUM_APS_TYPE =                                 8;  //Currently APS Type has 3 bits so the max type is 8

static constexpr int MAX_TILE_COLS =                                   30;  ///< Maximum number of tile columns
static constexpr int MAX_TILES =                                      990;  ///< Maximum number of tiles
static constexpr int MAX_SLICES =                                     600;  ///< Maximum number of slices per picture

static constexpr int MLS_GRP_NUM =                                    256; ///< Max number of coefficient groups, max(16, 256)
static constexpr int MLS_CG_SIZE =                                      4; ///< Coefficient group size of 4x4; = MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT

static constexpr int MAX_REF_LINE_IDX =                                 3; //highest refLine offset in the list
static constexpr int MRL_NUM_REF_LINES =                                3; //number of candidates in the array
static constexpr int MULTI_REF_LINE_IDX[3] =                  { 0, 1, 2 };

static constexpr int PRED_REG_MIN_WIDTH =                               4;  // Minimum prediction region width for ISP subblocks

static constexpr int NUM_LUMA_MODE =                                   67; ///< Planar + DC + 65 directional mode (4*16 + 1)
static constexpr int NUM_LMC_MODE =                                    1 + 2; ///< LMC + MDLM_T + MDLM_L
static constexpr int NUM_INTRA_MODE = (NUM_LUMA_MODE + NUM_LMC_MODE);

static constexpr int NUM_EXT_LUMA_MODE =                               28;

static constexpr int NUM_DIR =           (((NUM_LUMA_MODE - 3) >> 2) + 1);
static constexpr int PLANAR_IDX =                                       0; ///< index for intra PLANAR mode
static constexpr int DC_IDX =                                           1; ///< index for intra DC     mode
static constexpr int HOR_IDX =                    (1 * (NUM_DIR - 1) + 2); ///< index for intra HORIZONTAL mode
static constexpr int DIA_IDX =                    (2 * (NUM_DIR - 1) + 2); ///< index for intra DIAGONAL   mode
static constexpr int VER_IDX =                    (3 * (NUM_DIR - 1) + 2); ///< index for intra VERTICAL   mode
static constexpr int VDIA_IDX =                   (4 * (NUM_DIR - 1) + 2); ///< index for intra VDIAGONAL  mode
static constexpr int BDPCM_IDX =                  (5 * (NUM_DIR - 1) + 2); ///< index for intra VDIAGONAL  mode

static constexpr int NUM_CHROMA_MODE =                 (5 + NUM_LMC_MODE); ///< total number of chroma modes
static constexpr int LM_CHROMA_IDX                        = NUM_LUMA_MODE; ///< chroma mode index for derived from LM mode
static constexpr int MDLM_L_IDX =                       LM_CHROMA_IDX + 1; ///< MDLM_L
static constexpr int MDLM_T_IDX =                       LM_CHROMA_IDX + 2; ///< MDLM_T
static constexpr int DM_CHROMA_IDX =                       NUM_INTRA_MODE; ///< chroma mode index for derived from luma intra mode
static constexpr int NUM_MOST_PROBABLE_MODES                          = 6;
static constexpr int LM_SYMBOL_NUM                   = (1 + NUM_LMC_MODE);

static constexpr uint32_t  NUM_TRAFO_MODES_MTS =                        6; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static constexpr uint32_t  MTS_INTRA_MAX_CU_SIZE =                     32; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static constexpr uint32_t  MTS_INTER_MAX_CU_SIZE =                     32; ///< Max Inter CU size applying EMT, supported values: 8, 16, 32, 64, 128

static constexpr int MAX_NUM_MIP_MODE =                                32; ///< maximum number of MIP modes
#if INTRA_FULL_SEARCH
static constexpr int FAST_UDI_MAX_RDMODE_NUM = (NUM_LUMA_MODE + MAX_NUM_MIP_MODE); ///< maximum number of RD comparison in fast-UDI estimation loop
#else
static constexpr int FAST_UDI_MAX_RDMODE_NUM = (16);                       ///< maximum number of RD comparison in fast-UDI estimation loop
#endif
static constexpr int MAX_LFNST_COEF_NUM =                              16;

static constexpr int LFNST_LAST_SIG_LUMA =                              1;
static constexpr int LFNST_LAST_SIG_CHROMA =                            1;

static constexpr int NUM_LFNST_NUM_PER_SET =                            3;

static constexpr int MV_FRACTIONAL_BITS_INTERNAL                      = 4;
static constexpr int MV_FRACTIONAL_BITS_SIGNAL                        = 2;
static constexpr int MV_FRACTIONAL_BITS_DIFF = MV_FRACTIONAL_BITS_INTERNAL - MV_FRACTIONAL_BITS_SIGNAL;
static constexpr int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL = 1 << MV_FRACTIONAL_BITS_SIGNAL;
static constexpr int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS = 1 << MV_FRACTIONAL_BITS_INTERNAL;
static constexpr int CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS = 1 << (MV_FRACTIONAL_BITS_INTERNAL + 1);

static constexpr int MAX_NUM_SUB_PICS =                         (1 << 16);
static constexpr int MAX_NUM_LONG_TERM_REF_PICS =                      33;
static constexpr int NUM_LONG_TERM_REF_PIC_SPS =                        0;
#if INTER_FULL_SEARCH
static constexpr int MAX_REF_PICS =                                    33;
#else
static constexpr int MAX_REF_PICS =                                    6;
#endif

static constexpr int MAX_QP_OFFSET_LIST_SIZE =                          6; ///< Maximum size of QP offset list is 6 entries

// Cost mode support
static constexpr int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP =      0; ///< QP to use for lossless coding.
static constexpr int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME =4; ///< QP' to use for mixed_lossy_lossless coding.

static constexpr int MAX_CU_DEPTH =                                     7; ///< log2(CTUSize)
static constexpr int MAX_CU_SIZE_IDX =                                  MAX_CU_DEPTH + 1; ///< 1+log2(CTUSize)
static constexpr int MAX_TU_SIZE_IDX =                                  MAX_TB_LOG2_SIZEY + 1; ///< 1+log2(MaxTuSize)
static constexpr int MAX_CU_SIZE =                                      1<<MAX_CU_DEPTH;
static constexpr int MIN_CU_LOG2 =                                      2;
static constexpr int MIN_PU_SIZE =                                      4;

static constexpr int JVET_C0024_ZERO_OUT_TH =                          32;

static constexpr int MAX_NUM_PART_IDXS_IN_CTU_WIDTH = MAX_CU_SIZE/MIN_PU_SIZE; ///< maximum number of partition indices across the width of a CTU (or height of a CTU)
static constexpr int SCALING_LIST_REM_NUM =                             6;

static constexpr int QUANT_SHIFT =                                     14; ///< Q(4) = 2^14
static constexpr int IQUANT_SHIFT =                                     6;

static constexpr int    SCALE_BITS      = 15;   // Precision for fractional bit estimates
static constexpr double FRAC_BITS_SCALE = 1.0 / (1 << SCALE_BITS);

static constexpr int SCALING_LIST_PRED_MODES = 2;
static constexpr int SCALING_LIST_NUM = MAX_NUM_COMP * SCALING_LIST_PRED_MODES; ///< list number for quantization matrix

static constexpr int SCALING_LIST_BITS =                                8; ///< bit depth of scaling list entries
static constexpr int LOG2_SCALING_LIST_NEUTRAL_VALUE =                  4; ///< log2 of the value that, when used in a scaling list, has no effect on quantisation

static constexpr int LAST_SIGNIFICANT_GROUPS =                         14;

static constexpr int AFFINE_MIN_BLOCK_SIZE =                            4; ///< Minimum affine MC block size

static constexpr int MMVD_REFINE_STEP =                                 8; ///< max number of distance step
static constexpr int MMVD_MAX_REFINE_NUM =                              (MMVD_REFINE_STEP * 4); ///< max number of candidate from a base candidate
static constexpr int MMVD_BASE_MV_NUM =                                 2; ///< max number of base candidate
static constexpr int MMVD_ADD_NUM =                                     (MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM);///< total number of mmvd candidate

union MmvdIdx
{
  using T = uint8_t;

  static constexpr int LOG_REFINE_STEP = 3;
  static constexpr int REFINE_STEP     = 1 << LOG_REFINE_STEP;
  static constexpr int LOG_BASE_MV_NUM = 1;
  static constexpr int BASE_MV_NUM     = 1 << LOG_BASE_MV_NUM;
  static constexpr int MAX_REFINE_NUM  = 4 * REFINE_STEP;
  static constexpr int ADD_NUM         = MAX_REFINE_NUM * BASE_MV_NUM;
  static constexpr int INVALID         = std::numeric_limits<T>::max();

  struct
  {
    T position : 2;
    T step     : LOG_REFINE_STEP;
    T baseIdx  : LOG_BASE_MV_NUM;
  } pos;
  T val;
};

static constexpr int MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT =            28;

static constexpr int BDOF_EXTEND_SIZE             =                     1;
static constexpr int BDOF_TEMP_BUFFER_SIZE        =                     (MAX_CU_SIZE + 2 * BDOF_EXTEND_SIZE) * (MAX_CU_SIZE + 2 * BDOF_EXTEND_SIZE);

static constexpr int PROF_BORDER_EXT_W            =                     1;
static constexpr int PROF_BORDER_EXT_H            =                     1;
static constexpr int BCW_NUM =                                          5; ///< the number of weight options
static constexpr int BCW_DEFAULT =                                      ((uint8_t)(BCW_NUM >> 1)); ///< Default weighting index representing for w=0.5
static constexpr int BCW_SIZE_CONSTRAINT =                            256; ///< disabling Bcw if cu size is smaller than 256
static constexpr int MAX_NUM_HMVP_CANDS =                              (MRG_MAX_NUM_CANDS-1); ///< maximum number of HMVP candidates to be stored and used in merge list
static constexpr int MAX_NUM_HMVP_AVMPCANDS =                          4; ///< maximum number of HMVP candidates to be used in AMVP list

static constexpr int ALF_VB_POS_ABOVE_CTUROW_LUMA = 4;
static constexpr int ALF_VB_POS_ABOVE_CTUROW_CHMA = 2;

static constexpr int DMVR_SUBCU_SIZE = 16;
static constexpr int DMVR_SUBCU_SIZE_LOG2 = 4;
static constexpr int MAX_NUM_SUBCU_DMVR = ((MAX_CU_SIZE * MAX_CU_SIZE) >> (DMVR_SUBCU_SIZE_LOG2 + DMVR_SUBCU_SIZE_LOG2));
static constexpr int DMVR_NUM_ITERATION = 2;

//QTBT high level parameters
//for I slice luma CTB configuration para.
static constexpr int    MAX_BT_DEPTH  =                                 4;      ///<  <=7
                                                                            //for P/B slice CTU config. para.
static constexpr int    MAX_BT_DEPTH_INTER =                            4;      ///< <=7
static constexpr int    MAX_BT_SIZE_INTER  =                          128;      ///< for initialization, [1<<MIN_BT_SIZE_INTER, 1<<CTU_LOG2]

                                                                           //for I slice chroma CTB configuration para. (in luma samples)
static constexpr int    MIN_DUALTREE_CHROMA_WIDTH  =                    4;
static constexpr int    MIN_DUALTREE_CHROMA_SIZE   =                   16;
static constexpr SplitSeries SPLIT_BITS         =                       5;
static constexpr SplitSeries SPLIT_DMULT        =                       5;
static constexpr SplitSeries SPLIT_MASK         =                      31;      ///< = (1 << SPLIT_BITS) - 1

static constexpr int    SKIP_DEPTH =                                    3;
static constexpr int    PICTURE_DISTANCE_TH =                           1;
static constexpr int    FAST_SKIP_DEPTH =                               2;

static constexpr double PBINTRA_RATIO     =                             1.1;
static constexpr int    NUM_MRG_SATD_CAND =                             4;
static constexpr double MRG_FAST_RATIO[2]    =                        { 1.25, 1.05 };
static constexpr double MRG_FAST_RATIOMYV[4] =                        { 1.15, 1.1, 1.1, 1.05 };
static constexpr int    NUM_AMAXBT_LAYER =                             10;
static constexpr double AMAXBT_TH32 =                                  15.0;
static constexpr double AMAXBT_TH64 =                                  30.0;
static constexpr int    NUM_AFF_MRG_SATD_CAND =                         2;

// Threshholds for Fast Chroma Block Partitoning. Only used in Intra Only Coding
static constexpr int    FCBP_TH1 =                                     18000;
static constexpr int    FCBP_TH2 =                                     105;

static constexpr int NTAPS_LUMA               =                         8; ///< Number of taps for luma
static constexpr int NTAPS_CHROMA             =                         4; ///< Number of taps for chroma
static constexpr int NTAPS_AFFINE             =                         6;
static constexpr int NTAPS_BILINEAR           =                         2; ///< Number of taps for bilinear filter

static constexpr int ATMVP_SUB_BLOCK_SIZE =                             3; ///< sub-block size for ATMVP

static constexpr int GEO_MAX_NUM_UNI_CANDS =                            6;
static constexpr int GEO_MAX_NUM_CANDS = GEO_MAX_NUM_UNI_CANDS * (GEO_MAX_NUM_UNI_CANDS - 1);
static constexpr int GEO_MIN_CU_LOG2 =                                  3;
static constexpr int GEO_MAX_CU_LOG2 =                                  6;
static constexpr int GEO_MIN_CU_SIZE =               1 << GEO_MIN_CU_LOG2;
static constexpr int GEO_MAX_CU_SIZE =               1 << GEO_MAX_CU_LOG2;
static constexpr int GEO_NUM_CU_SIZE = ( GEO_MAX_CU_LOG2 - GEO_MIN_CU_LOG2 ) + 1;
static constexpr int GEO_NUM_PARTITION_MODE =                          64;
static constexpr int GEO_NUM_ANGLES =                                  32;
static constexpr int GEO_NUM_DISTANCES =                                4;
static constexpr int GEO_NUM_PRESTORED_MASK =                           6;
static constexpr int GEO_WEIGHT_MASK_SIZE = 3 * (GEO_MAX_CU_SIZE >> 3) * 2 + GEO_MAX_CU_SIZE;
static constexpr int GEO_MV_MASK_SIZE =         GEO_WEIGHT_MASK_SIZE >> 2;
static constexpr int GEO_MAX_TRY_WEIGHTED_SAD = 60;
static constexpr int GEO_MAX_TRY_WEIGHTED_SATD = 8;

#if INTER_FULL_SEARCH
static constexpr int SBT_NUM_SL =                                      10; ///< maximum number of historical PU decision saved for a CU
#else
static constexpr int SBT_NUM_SL =                                       4; ///< maximum number of historical PU decision saved for a CU
#endif
static constexpr int SBT_NUM_RDO =                                      2; ///< maximum number of SBT mode tried for a PU
static constexpr unsigned SBT_FAST64_WIDTH_THRESHOLD =               1920;

static constexpr int NUM_INTER_CU_INFO_SAVE =                           8; ///< maximum number of inter cu information saved for fast algorithm
static constexpr int LDT_MODE_TYPE_INHERIT =                            0; ///< No need to signal mode_constraint_flag, and the modeType of the region is inherited from its parent node
static constexpr int LDT_MODE_TYPE_INFER =                              1; ///< No need to signal mode_constraint_flag, and the modeType of the region is inferred as MODE_TYPE_INTRA
static constexpr int LDT_MODE_TYPE_SIGNAL =                             2; ///< Need to signal mode_constraint_flag, and the modeType of the region is determined by the flag

static constexpr int IBC_NUM_CANDIDATES = 64; ///< Maximum number of candidates to store/test
static constexpr int CHROMA_REFINEMENT_CANDIDATES = 8; /// 8 candidates BV to choose from

static constexpr int MV_EXPONENT_BITCOUNT    = 4;
static constexpr int MV_MANTISSA_BITCOUNT    = 6;
static constexpr int MV_MANTISSA_UPPER_LIMIT = ((1 << (MV_MANTISSA_BITCOUNT - 1)) - 1);
static constexpr int MV_MANTISSA_LIMIT       = (1 << (MV_MANTISSA_BITCOUNT - 1));
static constexpr int MV_EXPONENT_MASK        = ((1 << MV_EXPONENT_BITCOUNT) - 1);

static constexpr int MV_BITS =                                   18;
static constexpr int MV_MAX =              (1 << (MV_BITS - 1)) - 1;
static constexpr int MV_MIN =                 -(1 << (MV_BITS - 1));

static constexpr int MVD_MAX =                            (1 << 17) - 1;
static constexpr int MVD_MIN =                               -(1 << 17);

static constexpr int PIC_ANALYZE_CW_BINS =                           32;
static constexpr int PIC_CODE_CW_BINS =                              16;
static constexpr int LMCS_SEG_NUM =                                  32;
static constexpr int FP_PREC =                                       11;
static constexpr int CSCALE_FP_PREC =                                11;

static constexpr int MCTF_PADDING         = 128;

static constexpr int SCALE_RATIO_BITS =                              14;
static constexpr int DELTA_QP_ACT[4] =                  { -5, 1, 3, 1 };

static constexpr uint32_t CCALF_CANDS_COEFF_NR = 8;
static constexpr int CCALF_SMALL_TAB[CCALF_CANDS_COEFF_NR] = { 0, 1, 2, 4, 8, 16, 32, 64 };

static constexpr uint8_t MIP_SHIFT_MATRIX  =  6;
static constexpr uint8_t MIP_OFFSET_MATRIX = 32;
static constexpr uint8_t SORTED_BUFS = 9;
static constexpr uint8_t MAX_TMP_BUFS = 2 + 2 * GEO_MAX_NUM_UNI_CANDS;

static constexpr int QPA_MAX_NOISE_LEVELS = 8;



// ====================================================================================================================
// Macro functions
// ====================================================================================================================

struct ClpRng
{
  static constexpr
  int min()       { return 0; }
  int max() const { return ( 1 << bd )-1; }
  int bd;
};

struct ClpRngs : public ClpRng
{
  // bitdepth has to be the same for all channels
  const ClpRng& operator[] ( const ComponentID /*compID*/ ) const { return *this; }
  const ClpRng& operator[] ( const int         /*compID*/ ) const { return *this; }
  //const ClpRng& operator[] ( const ComponentID compId ) const { return comp[compId]; }
  //ClpRng comp[MAX_NUM_COMP]; ///< the bit depth as indicated in the SPS
};

struct Position;
struct Size;
struct VPS;
struct DCI;
struct SPS;
struct PPS;
class Slice;
class PreCalcValues;

template <typename T> constexpr static inline T Clip3  ( const T minVal, const T maxVal, const T a) { return std::min<T> (std::max<T> (minVal, a) , maxVal); }  ///< general min/max clip
template <typename T> constexpr static inline T ClipBD ( const T x, const int bitDepth )            { return Clip3( T( 0 ), T( ( 1 << bitDepth ) - 1 ), x ); }
template <typename T> constexpr static inline T ClipPel( const T a, const ClpRng& clpRng )          { return std::min<T> (std::max<T> (clpRng.min(), a) , clpRng.max()); }  ///< clip reconstruction

template <typename T> inline void Check3( T minVal, T maxVal, T a)
{
  CHECK( ( a > maxVal ) || ( a < minVal ), "ERROR: Range check " << minVal << " >= " << a << " <= " << maxVal << " failed" );
}  ///< general min/max clip

// global logger message callback function - DEPRECATED - will be removed in next major version
extern std::function<void( void*, int, const char*, va_list )> g_msgFnc;
extern void * g_msgFncCtx;
// end global logger 


inline std::string prnt( const char* fmt, ...)
{
  va_list argptr;

  va_start(argptr,fmt);

  std::vector<char> cCharVector(256);
  size_t tRequiredSize = vsnprintf(&cCharVector[0], cCharVector.size(), fmt, argptr);

  if( tRequiredSize > cCharVector.size())
  {
    // try again
    cCharVector.resize( tRequiredSize );
    vsnprintf(&cCharVector[0], cCharVector.size(), fmt, argptr);
  }

  va_end(argptr);

  return std::string( &cCharVector[0] );
}

// template<typename T> bool isPowerOf2( const T val ) { return ( val & ( val - 1 ) ) == 0; }

#define MEMORY_ALIGN_DEF_SIZE       32  // for use with avx2 (256 bit)
#define CACHE_MEM_ALIGN_SIZE      1024

#define ALIGNED_MALLOC              1   ///< use 32-bit aligned malloc/free

#if ALIGNED_MALLOC

#if ( _WIN32 && ( _MSC_VER > 1300 ) ) || defined (__MINGW64_VERSION_MAJOR)
#define xMalloc( type, len )        _aligned_malloc( sizeof(type)*(len), MEMORY_ALIGN_DEF_SIZE )
#define xMalloc2( type, len, alg )  _aligned_malloc( sizeof(type)*(len), alg )
#define xFree( ptr )                _aligned_free  ( ptr )
#elif defined (__MINGW32__)
#define xMalloc( type, len )        __mingw_aligned_malloc( sizeof(type)*(len), MEMORY_ALIGN_DEF_SIZE )
#define xMalloc2( type, len, alg )  __mingw_aligned_malloc( sizeof(type)*(len), alg )
#define xFree( ptr )                __mingw_aligned_free( ptr )
#else
namespace detail {
template<typename T>
static inline T* aligned_malloc(size_t len, size_t alignement) {
  T* p = NULL;
  if( posix_memalign( (void**)&p, alignement, sizeof(T)*(len) ) )
  {
    THROW("posix_memalign failed");
  }
  return p;
}
}
#define xMalloc( type, len )        detail::aligned_malloc<type>( len, MEMORY_ALIGN_DEF_SIZE )
#define xMalloc2( type, len, alg )  detail::aligned_malloc<type>( len, alg )
#define xFree( ptr )                free( ptr )
#endif

#else
#define xMalloc( type, len )        malloc   ( sizeof(type)*(len) )
#define xMalloc2( type, len, alg )  malloc   ( sizeof(type)*(len) )
#define xFree( ptr )                free     ( ptr )
#endif //#if ALIGNED_MALLOC

#if defined _MSC_VER
#define ALIGN_DATA(nBytes,v) __declspec(align(nBytes)) v
#else
//#elif defined linux
#define ALIGN_DATA(nBytes,v) v __attribute__ ((aligned (nBytes)))
//#else
//#error unknown platform
#endif

#if defined(__GNUC__) && !defined(__clang__)
#    define GCC_VERSION_AT_LEAST(x,y) (__GNUC__ > x || __GNUC__ == x && __GNUC_MINOR__ >= y)
#else
#    define GCC_VERSION_AT_LEAST(x,y) 0
#endif

#ifdef __clang__
#    define CLANG_VERSION_AT_LEAST(x,y) (__clang_major__ > x || __clang_major__ == x && __clang_minor__ >= y)
#else
#    define CLANG_VERSION_AT_LEAST(x,y) 0
#endif

#ifdef __GNUC__
#    define ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined _MSC_VER
#    define ALWAYS_INLINE __forceinline
#else
#    define ALWAYS_INLINE
#endif

#if defined( _WIN32 ) && defined( TARGET_SIMD_X86 )
static inline unsigned int bit_scan_reverse( int a )
{
  unsigned long idx = 0;
  _BitScanReverse( &idx, a );
  return idx;
}
// disabled because it requires x86intrin.h which conflicts with simd-everywhere
// #elif defined( __GNUC__ ) && defined( TARGET_SIMD_X86 ) && !defined( REAL_TARGET_WASM )
// static inline unsigned int bit_scan_reverse( int a )
// {
//   return _bit_scan_reverse( a );
// }
#elif defined( __GNUC__ )
static inline unsigned int bit_scan_reverse( int a )
{
  return __builtin_clz( a ) ^ ( 8 * sizeof( a ) - 1 );
}
#endif

#if ENABLE_SIMD_LOG2
static inline int getLog2( int val )
{
  return bit_scan_reverse( val );
}
#else
extern int8_t g_aucLog2[MAX_CU_SIZE + 1];
static inline int getLog2( int val )
{
  CHECKD( g_aucLog2[2] != 1, "g_aucLog2[] has not been initialized yet." );
  if( val > 0 && val < (int) sizeof( g_aucLog2 ) )
  {
    return g_aucLog2[val];
  }
  return std::log2( val );
}
#endif

#if ENABLE_SIMD_OPT

//necessary to be able to compare with SIMD_EVERYWHERE_EXTENSION_LEVEL in RdCostArm during compile time.
#define X86_SIMD_UNDEFINED -1
#define X86_SIMD_SCALAR 0
#define X86_SIMD_SSE41 1
#define X86_SIMD_SSE42 2
#define X86_SIMD_AVX 3
#define X86_SIMD_AVX2 4
#define X86_SIMD_AVX512 5

namespace x86_simd
{
#ifdef TARGET_SIMD_X86
  typedef enum
  {
    UNDEFINED = X86_SIMD_UNDEFINED,
    SCALAR = X86_SIMD_SCALAR,
    SSE41 = X86_SIMD_SSE41,
    SSE42 = X86_SIMD_SSE42,
    AVX = X86_SIMD_AVX,
    AVX2 = X86_SIMD_AVX2,
    AVX512 = X86_SIMD_AVX512
  } X86_VEXT;
#endif
}

namespace arm_simd
{
#ifdef TARGET_SIMD_ARM
typedef enum
{
  UNDEFINED = -1,
  SCALAR    = 0,
  NEON,
  SVE,
  SVE2,
} ARM_VEXT;
#endif   // TARGET_SIMD_ARM
}   // namespace arm_simd

#endif //ENABLE_SIMD_OPT

template <typename ValueType> inline ValueType leftShiftU  (const ValueType value, const unsigned shift) { return value << shift; }
template <typename ValueType> inline ValueType rightShiftU (const ValueType value, const unsigned shift) { return value >> shift; }

#if ENABLE_SIMD_LOG2 && defined( TARGET_SIMD_X86 )
static inline int floorLog2( int val )
{
  CHECKD(val == 0, "invalid input value");
  return bit_scan_reverse( val );
}
#else
static inline int floorLog2(uint32_t x)
{
  CHECKD( x == 0, "invalid input value");
#ifdef __GNUC__
  return 31 - __builtin_clz(x);
#else
  int result = 0;
  if (x & 0xffff0000)
  {
    x >>= 16;
    result += 16;
  }
  if (x & 0xff00)
  {
    x >>= 8;
    result += 8;
  }
  if (x & 0xf0)
  {
    x >>= 4;
    result += 4;
  }
  if (x & 0xc)
  {
    x >>= 2;
    result += 2;
  }
  if (x & 0x2)
  {
    x >>= 1;
    result += 1;
  }
  return result;
#endif
}
#endif

static inline int ceilLog2(uint32_t x)
{
  return (x==0) ? -1 : floorLog2(x - 1) + 1;
}


//======================================================================================================================
//Chroma format utility functions  =====================================================================================
//======================================================================================================================


static inline bool        isLuma                    (const ComponentID id)                         { return (id==COMP_Y);                                  }
static inline bool        isLuma                    (const ChannelType id)                         { return (id==CH_L);                                    }
static inline bool        isChroma                  (const ComponentID id)                         { return (id!=COMP_Y);                                  }
static inline bool        isChroma                  (const ChannelType id)                         { return (id!=CH_L);                                    }
static inline ChannelType toChannelType             (const ComponentID id)                         { return (id==COMP_Y)? CH_L : CH_C;                     }
static inline uint32_t    getChannelTypeScaleX      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt==CHROMA_444)) ? 0 : 1;     }
static inline uint32_t    getChannelTypeScaleY      (const ChannelType id, const ChromaFormat fmt) { return (isLuma(id) || (fmt!=CHROMA_420)) ? 0 : 1;     }
static inline uint32_t    getComponentScaleX        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleX(toChannelType(id), fmt);  }
static inline uint32_t    getComponentScaleY        (const ComponentID id, const ChromaFormat fmt) { return getChannelTypeScaleY(toChannelType(id), fmt);  }
static inline uint32_t    getNumberValidComponents  (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_COMP;          }
static inline uint32_t    getNumberValidChannels    (const ChromaFormat fmt)                       { return (fmt==CHROMA_400) ? 1 : MAX_NUM_CH;            }
static inline bool        isChromaEnabled           (const ChromaFormat fmt)                       { return !(fmt==CHROMA_400);                            }
static inline ComponentID getFirstComponentOfChannel(const ChannelType id)                         { return (isLuma(id) ? COMP_Y : COMP_Cb);               }



//CASE-BREAK for breakpoints
#if defined ( _MSC_VER ) && defined ( _DEBUG )
#define _CASE(_x) if(_x)
#define _BREAK while(0);
#define _AREA_AT(_a,_x,_y,_w,_h)  (_a.x==_x && _a.y==_y && _a.width==_w && _a.height==_h)
#define _AREA_CONTAINS(_a,_x,_y)  (_a.contains( Position{ _x, _y} ))
#define _UNIT_AREA_AT(_a,_x,_y,_w,_h) (_a.Y().x==_x && _a.Y().y==_y && _a.Y().width==_w && _a.Y().height==_h)
#else
#define _CASE(...)
#define _BREAK
#define _AREA_AT(...)
#define _AREA_CONTAINS(_a,_x,_y)
#define _UNIT_AREA_AT(_a,_x,_y,_w,_h)
#endif

// Make enum and strings macros, used for TimeProfiler and DTrace
#define MAKE_ENUM(VAR) VAR,
#define MAKE_STRINGS(VAR) #VAR,
#define MAKE_ENUM_AND_STRINGS(source, enumName, enumStringName) \
enum enumName { \
    source(MAKE_ENUM) \
    };\
const char* const enumStringName[] = { \
    source(MAKE_STRINGS) \
    }; \

#ifdef TRACE_ENABLE_ITT

} // namespace vvenc

#include <ittnotify.h>

namespace vvenc {

# define ITT_TASKSTART( d, t ) __itt_task_begin( ( d ), __itt_null, __itt_null, ( (__itt_string_handle*)t ) )
# define ITT_TASKEND( d, t )   __itt_task_end  ( ( d ) )

# define ITT_SYNCPREP( p ) __itt_sync_prepare  ( & p )
# define ITT_SYNCACQ( p )  __itt_sync_acquired ( & p )
# define ITT_SYNCREL( p )  __itt_sync_releasing( & p )

# define ITT_COUNTSET( c, v ) __itt_counter_set_value( c, &v )
# define ITT_COUNTINC( c )    __itt_counter_inc( c )
# define ITT_COUNTDEC( c )    __itt_counter_dec( c )
# define ITT_COUNTADD( c, v ) __itt_counter_inc_delta( c, &v )
# define ITT_COUNTSUB( c, v ) __itt_counter_dec_delta( c, &v )
#else //!TRACE_ENABLE_ITT
# define ITT_TASKSTART( d, t )
# define ITT_TASKEND( d, t )

# define ITT_SYNCPREP( p )
# define ITT_SYNCACQ( p )
# define ITT_SYNCREL( p )

# define ITT_COUNTSET( c, v )
# define ITT_COUNTINC( c )
# define ITT_COUNTDEC( c )
# define ITT_COUNTADD( c, v )
# define ITT_COUNTSUB( c, v )
#endif //!TRACE_ENABLE_ITT

#define SIGN(x) ( (x) >= 0 ? 1 : -1 )

} // namespace vvenc

//! \}


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
/** \file     Rom.h
    \brief    global variables & functions (header)
*/

#pragma once

#include "CommonDef.h"
#include "Common.h"
#if ENABLE_TRACING
#include "dtrace.h"
#endif
#include "TimeProfiler.h"

#include <iostream>

//! \ingroup CommonLib
//! \{

namespace vvenc {

/*! Sophisticated Trace-logging */
#if ENABLE_TRACING
extern CDTrace* g_trace_ctx;
#endif

#if ENABLE_TIME_PROFILING
extern TimeProfiler *g_timeProfiler;
#elif ENABLE_TIME_PROFILING_EXTENDED
extern TimeProfiler2D *g_timeProfiler;
#endif

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================


// flexible conversion from relative to absolute index
struct ScanElement
{
  uint32_t idx;
  uint16_t x;
  uint16_t y;
};


class InitGeoRom
{
  public:
    InitGeoRom() { initGeoTemplate(); }

    ~InitGeoRom() { }

  private:
    void initGeoTemplate() const;

  private:
};

extern const ScanElement  m_scanOrderBuf[32258];
extern const ScanElement* m_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][MAX_TU_SIZE_IDX][MAX_TU_SIZE_IDX];

const ScanElement* const getScanOrder( int g, int w2, int h2 );

extern const int8_t g_BcwLog2WeightBase;
extern const int8_t g_BcwWeightBase;
extern const int8_t g_BcwWeights[BCW_NUM];
extern const int8_t g_BcwSearchOrder[BCW_NUM];
extern const int8_t g_BcwCodingOrder[BCW_NUM];
extern const int8_t g_BcwParsingOrder[BCW_NUM];

class CodingStructure;
int8_t getBcwWeight(uint8_t bcwIdx, uint8_t uhRefFrmList);
void resetBcwCodingOrder(bool bRunDecoding, const CodingStructure &cs);
uint32_t deriveWeightIdxBits(uint8_t bcwIdx);

extern const InitGeoRom g_scanOrderRom;

extern const uint32_t g_log2SbbSize[MAX_TU_SIZE_IDX][MAX_TU_SIZE_IDX][2];
extern const ScanElement g_coefTopLeftDiagScan8x8[MAX_TU_SIZE_IDX][64];

extern const int g_quantScales   [2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // Q(QP%6)
extern const int g_invQuantScales[2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // IQ(QP%6)

static const int g_numTransformMatrixSizes = 6;
static const int g_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = {  6, 6 };


// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================

extern const uint32_t   g_uiGroupIdx[ MAX_TB_SIZEY ];
extern const uint32_t   g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ];
extern const uint32_t   g_auiGoRiceParsCoeff     [ 32 ];
inline uint32_t g_auiGoRicePosCoeff0(int st, uint32_t ricePar)
{
  return (st < 2 ? 1 : 2) << ricePar;
}

// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const uint8_t  g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1];
extern const uint8_t  g_aucIntraModeNumFast_UseMPM   [MAX_CU_DEPTH];
extern const uint8_t  g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH];

extern const uint8_t  g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE];

// ====================================================================================================================
// Luma QP to Chroma QP mapping
// ====================================================================================================================
static const int chromaQPMappingTableSize = (MAX_QP + 7);

extern const uint8_t  g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize];

// ====================================================================================================================
// Mode-Dependent DST Matrices
// ====================================================================================================================


extern const TMatrixCoeff g_trCoreDCT2P2  [TRANSFORM_NUMBER_OF_DIRECTIONS][  2][  2];
extern const TMatrixCoeff g_trCoreDCT2P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDCT2P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDCT2P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDCT2P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];
extern const TMatrixCoeff g_trCoreDCT2P64 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 64][ 64];

extern const TMatrixCoeff g_trCoreDCT8P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDCT8P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDCT8P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDCT8P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];

extern const TMatrixCoeff g_trCoreDST7P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDST7P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDST7P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDST7P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];

extern const     int8_t   g_lfnst8x8[ 4 ][ 2 ][ 16 ][ 48 ];
extern const     int8_t   g_lfnst4x4[ 4 ][ 2 ][ 16 ][ 16 ];

extern const     uint8_t  g_lfnstLut[ NUM_INTRA_MODE + NUM_EXT_LUMA_MODE - 1 ];

// ====================================================================================================================
// Misc.
// ====================================================================================================================
inline int Log2(uint32_t x)
{
  return floorLog2( x );
}

extern const int       g_ictModes[2][4];

extern const UnitScale     g_miScaling; // scaling object for motion scaling

extern const char *MatrixType   [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];
extern const char *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern const int g_quantTSDefault4x4   [4*4];
extern const int g_quantIntraDefault8x8[8*8];
extern const int g_quantInterDefault8x8[8*8];

extern const uint32_t g_scalingListSize [SCALING_LIST_SIZE_NUM];
extern const uint32_t g_scalingListSizeX[SCALING_LIST_SIZE_NUM];

class CodingStructure;

const int g_IBCBufferSize = 256 * 128;

constexpr uint8_t g_tbMax[257] = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8 };

//! \}

extern int16_t   g_GeoParams[GEO_NUM_PARTITION_MODE][2];
extern int16_t   g_globalGeoWeights[GEO_NUM_PRESTORED_MASK]   [GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE];
extern int16_t   g_globalGeoEncSADmask[GEO_NUM_PRESTORED_MASK][GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE];
extern const int8_t    g_angle2mask[GEO_NUM_ANGLES];
extern const int8_t    g_Dis[GEO_NUM_ANGLES];
extern const int8_t    g_angle2mirror[GEO_NUM_ANGLES];
extern int16_t   g_weightOffset[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_NUM_PARTITION_MODE][2];

#if ENABLE_CU_MODE_COUNTERS
#define E_COUNTERS_CU(E_) \
    E_( CU_MODES_TRIED ) \
    E_( CU_MODES_TESTED ) \
    E_( CU_RD_TESTS ) \
    E_( CU_CODED_FINALLY )
MAKE_ENUM_AND_STRINGS( E_COUNTERS_CU, CUCounterId, g_cuCounterIdNames )

extern StatCounters::StatCounter2DSet<int64_t> g_cuCounters1D;
extern StatCounters::StatCounter2DSet<int64_t> g_cuCounters2D;
#define STAT_COUNT_CU_MODES(cond,...) if(cond) {(__VA_ARGS__)++;}
#else
#define STAT_COUNT_CU_MODES(cond,...)
#endif
} // namespace vvenc

//! \}


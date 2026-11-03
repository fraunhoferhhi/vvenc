// Experimental PPO observation extraction for the VVenC v1.14.0 fork.
//
// VVENC_RL_STATE_FILE=/path/observations.txt writes a line per final-pass
// picture after picture-level QPA, BEFORE any CTU is encoded.
// Format: STATE poc g0..g7 activity[12] temporal[12] qpa[12]
// The three maps are row-major and correspond to a 4x3 128px CTU grid.
//
// This is a pre-action observation; never read the current picture's coded bits,
// final motion decisions, partitions, or reconstructed frame into the state.

#pragma once

#include "CommonLib/Picture.h"
#include "RateCtrl.h"
#include "vvenc/vvencCfg.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>

namespace vvenc {
namespace rl {

class Observation
{
public:
  static bool enabled()
  {
    const char* f = std::getenv("VVENC_RL_STATE_FILE");
    return f != nullptr && f[0] != '\0';
  }

  static void emit( const Slice& slice, const VVEncCfg& cfg, const RateCtrl* rc )
  {
    if( !enabled() || slice.pic == nullptr || slice.pic->isPreAnalysis )
      return;

    const Picture& pic = *slice.pic;
    const PreCalcValues& pcv = *slice.pps->pcv;
    if( pcv.widthInCtus != 4 || pcv.heightInCtus != 3 || pcv.maxCUSize != 128 )
      throw std::runtime_error("VVenC-RL: PPO observation currently requires 4x3 CTUs at size 128");

    const CPelBuf current = pic.getOrigBuf( COMP_Y );
    const CPelBuf previous = pic.getOrigBufPrev( COMP_Y, (PrevFrameType) 0 );
    const bool hasPrevious = previous.buf != nullptr && previous.width == current.width &&
                             previous.height == current.height;

    const int depth = cfg.m_internalBitDepth[CH_L];
    const double peak = double((1 << std::min(12, std::max(1, depth))) - 1);

    std::array<double, 12> spatial = {};
    std::array<double, 12> temporal = {};
    std::array<double, 12> qpa = {};

    for( int row = 0; row < 3; ++row )
    {
      for( int col = 0; col < 4; ++col )
      {
        const int idx = row * 4 + col;
        const int x0 = col * (int) pcv.maxCUSize;
        const int y0 = row * (int) pcv.maxCUSize;
        const int x1 = std::min(x0 + (int)pcv.maxCUSize, (int)current.width);
        const int y1 = std::min(y0 + (int)pcv.maxCUSize, (int)current.height);
        const int qp = idx < (int) pic.ctuAdaptedQP.size()
                       ? pic.ctuAdaptedQP[idx] : slice.sliceQp;
        qpa[idx] = std::max(0.0, std::min(1.0, qp / 63.0));
        int samples = 0;
        double gradients = 0.0;
        double temporalDifference = 0.0;
        for( int y = y0; y < y1; y += 8 )
        {
          for( int x = x0; x < x1; x += 8 )
          {
            const int value = (int)current.at(x,y);
            if( x + 1 < x1 ) gradients += std::abs(value - (int)current.at(x+1,y));
            if( y + 1 < y1 ) gradients += std::abs(value - (int)current.at(x,y+1));
            if( hasPrevious )
              temporalDifference += std::abs(value - (int)previous.at(x,y));
            ++samples;
          }
        }
        if( samples > 0 )
        {
          spatial[idx] = std::min(1.0, gradients / (2.0 * samples * peak));
          temporal[idx] = hasPrevious
                          ? std::min(1.0, temporalDifference / (samples * peak))
                          : 0.0;
        }
      }
    }

    const double target = double(cfg.m_RCTargetBitrate);
    const double totalFrames = double(std::max(1, cfg.m_framesToBeEncoded));
    const double fps = double(cfg.m_FrameRate) / std::max(1, cfg.m_FrameScale);
    const double targetBits = (fps > 0.0 ? target * totalFrames / fps : 0.0);

    // Encoder-rate-control accumulators describe only previously coded frames.
    // On GOP reordering these are encoding-order budgets, not POC-order budgets.
    double usedFraction = 0.0;
    double lastQP = slice.sliceQp / 63.0;
    if( rc && rc->encRCSeq )
    {
      if( targetBits > 0.0 )
        usedFraction = std::max(0.0, std::min(2.0,
          double(rc->encRCSeq->bitsUsed) / targetBits));
      if( rc->encRCSeq->lastAverageQP >= 0 )
        lastQP = std::max(0.0, std::min(1.0,
          double(rc->encRCSeq->lastAverageQP) / 63.0));
    }

    // Global [8]: target rate, progress, used budget, remaining budget,
    // previous QP, picture QP, intra flag, temporal layer.
    const std::array<double,8> global = {{
      target / 200000.0,
      double(std::max(0, slice.poc)) / totalFrames,
      usedFraction,
      std::max(0.0, 1.0 - usedFraction),
      lastQP,
      slice.sliceQp / 63.0,
      slice.isIntra() ? 1.0 : 0.0,
      double(slice.TLayer) / 6.0
    }};

    static std::mutex outputMutex;
    std::lock_guard<std::mutex> lock(outputMutex);
    static std::ofstream output;
    if( !output.is_open() )
    {
      output.open(std::getenv("VVENC_RL_STATE_FILE"), std::ios::out | std::ios::trunc);
      if( !output )
        throw std::runtime_error("VVenC-RL: cannot open observation output file");
    }
    output << "STATE " << slice.poc << std::setprecision(9);
    for( double v : global ) output << ' ' << v;
    for( double v : spatial ) output << ' ' << v;
    for( double v : temporal ) output << ' ' << v;
    for( double v : qpa ) output << ' ' << v;
    output << '\n';
    output.flush(); // Python policy reader may be waiting on this POC
  }
};

} // namespace rl
} // namespace vvenc

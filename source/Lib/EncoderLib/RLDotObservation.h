// Pre-action RL-RC-DoT 16+4 observation prototype for VVenC v1.14.0.
// DOT_PROXY intentionally labels non-equivalent features; strict paper training
// must NOT use this contract until the missing x264-equivalent statistics exist.
// VVENC_DOT_STATE_FILE=/path/states.txt
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

namespace vvenc { namespace rl {

class DotObservation
{
public:
  static bool enabled()
  {
    const char* p = std::getenv("VVENC_DOT_STATE_FILE");
    return p != nullptr && p[0] != '\0';
  }
  static void emit( const Slice& slice, const VVEncCfg& cfg, const RateCtrl* rc )
  {
    if( !enabled() || !slice.pic || slice.pic->isPreAnalysis ) return;
    const Picture& pic = *slice.pic;
    const CPelBuf y = pic.getOrigBuf(COMP_Y);
    if( y.width != 480 || y.height != 320 )
      throw std::runtime_error("RL-RC-DoT: observation grid requires 480x320 Y plane");

    const CPelBuf prev = pic.getOrigBufPrev(COMP_Y, (PrevFrameType)0);
    const bool hasPrev = prev.buf && prev.width == y.width && prev.height == y.height;
    const double peak = double((1 << std::min(12,std::max(1,cfg.m_internalBitDepth[CH_L]))) - 1);
    std::array<double,600> energy = {}, intra = {}, temporal = {}, qscale = {};
    double complexitySum = 0.0;
    for( int by = 0; by < 20; ++by )
    for( int bx = 0; bx < 30; ++bx )
    {
      const int idx = by * 30 + bx;
      const int x0 = bx * 16, y0 = by * 16;
      double sum = 0, sumSq = 0, predErr = 0, tempErr = 0;
      for( int dy = 0; dy < 16; ++dy )
      for( int dx = 0; dx < 16; ++dx )
      {
        const int x = x0 + dx, yy = y0 + dy;
        const double v = y.at(x,yy);
        sum += v;
        sumSq += v * v;
        const double left = x > 0 ? y.at(x-1,yy) : v;
        const double top = yy > 0 ? y.at(x,yy-1) : v;
        predErr += std::abs(v - 0.5 * (left + top));
        if( hasPrev ) tempErr += std::abs(v - prev.at(x,yy));
      }
      const double variance = std::max(0.0, sumSq / 256.0 - (sum / 256.0) * (sum / 256.0));
      energy[idx] = variance / (peak * peak);
      // Intra spatial predictor error: explicit PROXY, not x264 lowres SATD.
      intra[idx] = predErr / (256.0 * peak);
      // Temporal SAD: explicit PROXY, not x264 propagated encoding cost.
      temporal[idx] = hasPrev ? tempErr / (256.0 * peak) : 0.0;
      const int ctuCol = x0 / (int)pic.cs->pcv->maxCUSize;
      const int ctuRow = y0 / (int)pic.cs->pcv->maxCUSize;
      const int ctuAddr = ctuRow * (int)pic.cs->pcv->widthInCtus + ctuCol;
      const int qp = ctuAddr < (int)pic.ctuAdaptedQP.size()
                      ? pic.ctuAdaptedQP[ctuAddr] : slice.sliceQp;
      const int offset = qp - slice.sliceQp;
      // AQ/QPA-derived scale proxy (NOT x264 i_inv_qscale_factor).
      qscale[idx] = std::max(1.0/16, std::min(16.0, std::pow(2.0, -double(offset)/6.0)));
      complexitySum += intra[idx];
    }
    const double frames = std::max(1, cfg.m_framesToBeEncoded);
    const double target = double(cfg.m_RCTargetBitrate);
    const double fps = double(cfg.m_FrameRate) / std::max(1,cfg.m_FrameScale);
    const double meanQp = rc && rc->encRCSeq && rc->encRCSeq->lastAverageQP >= 0
                        ? double(rc->encRCSeq->lastAverageQP) : double(slice.sliceQp);
    const double usedBits = rc && rc->encRCSeq ? double(rc->encRCSeq->bitsUsed) : 0.0;
    const double codedBps = slice.poc > 0 && fps > 0 ? usedBits * fps / double(slice.poc) : target;
    // G03,G06-G12 are NOT available as causal exact measurements in this
    // revision. They are zero-marked and tagged DOT_PROXY, never DOT_STATE.
    const std::array<double,16> g = {{
      double(slice.sliceQp)/63.0,
      double(slice.poc)/frames,
      0.0,
      meanQp/63.0,
      meanQp/63.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      double(slice.poc)/frames,
      target > 0 ? std::tanh(std::log(std::max(1.0,codedBps)/target)) : 0.0,
      slice.isIntra() ? 1.0 : 0.0,
      complexitySum/600.0
    }};
    static std::mutex outputMutex;
    std::lock_guard<std::mutex> lock(outputMutex);
    static std::ofstream out;
    if( !out.is_open() )
    {
      out.open(std::getenv("VVENC_DOT_STATE_FILE"), std::ios::out|std::ios::trunc);
      if( !out ) throw std::runtime_error("RL-RC-DoT: cannot open observation stream");
    }
    out << "DOT_PROXY " << slice.poc << std::setprecision(9);
    for(double v : g) out << ' ' << v;
    for(double v : energy) out << ' ' << v;
    for(double v : intra) out << ' ' << v;
    for(double v : temporal) out << ' ' << v;
    for(double v : qscale) out << ' ' << v;
    out << '\n';
    out.flush();
  }
};

}} // namespace vvenc::rl

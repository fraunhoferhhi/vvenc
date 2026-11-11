// RL-RC-DoT experimental action adapter for VVenC v1.14.0.
// Separate from the established 4x3 VVENC_RL_* pilot.
// Read a POC-keyed 30x20 delta-QP map and area-average it over the
// actual QG being encoded. This is NOT yet 16x16 QP signalling for Inter.
#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace vvenc { namespace rl {

class DotAction
{
public:
  static constexpr int width = 30;
  static constexpr int height = 20;
  static constexpr int size = width * height;
  static bool enabled()
  {
    const char* p = std::getenv("VVENC_DOT_ACTION_DIR");
    return p != nullptr && p[0] != '\0';
  }

  static int deltaFor( int poc, unsigned x, unsigned y, unsigned w, unsigned h )
  {
    if( !enabled() ) return 0;
    if( w == 0 || h == 0 || x >= 480 || y >= 320 )
      throw std::runtime_error("RL-RC-DoT: invalid action QG area");
    const auto& map = getMap( poc );
    const unsigned right = std::min(480u, x + w);
    const unsigned bottom = std::min(320u, y + h);
    if( right <= x || bottom <= y )
      throw std::runtime_error("RL-RC-DoT: QG outside source frame");

    int64_t weighted = 0;
    int64_t area = 0;
    const unsigned firstCol = x / 16, lastCol = (right - 1) / 16;
    const unsigned firstRow = y / 16, lastRow = (bottom - 1) / 16;
    for( unsigned row = firstRow; row <= lastRow; ++row )
    {
      const unsigned cellY0 = row * 16, cellY1 = cellY0 + 16;
      const unsigned overlapY = std::min(bottom,cellY1) - std::max(y,cellY0);
      for( unsigned col = firstCol; col <= lastCol; ++col )
      {
        const unsigned cellX0 = col * 16, cellX1 = cellX0 + 16;
        const unsigned overlapX = std::min(right,cellX1) - std::max(x,cellX0);
        const int64_t weight = int64_t(overlapX) * overlapY;
        weighted += weight * map[row * width + col];
        area += weight;
      }
    }
    if( !area ) throw std::runtime_error("RL-RC-DoT: empty action overlap");
    return int(std::llround(double(weighted) / double(area)));
  }

private:
  static const std::array<int,size>& getMap( int poc )
  {
    static std::mutex mutex;
    static std::map<int,std::array<int,size>> cache;
    static const std::string directory = []() {
      const char* p = std::getenv("VVENC_DOT_ACTION_DIR");
      return p ? std::string(p) : std::string();
    }();
    if( poc < 0 ) throw std::runtime_error("RL-RC-DoT: negative POC");
    // A stable reference is safe because std::map insertions do not invalidate it.
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto found = cache.find(poc);
      if( found != cache.end() ) return found->second;
    }
    const std::string path = directory + "/" + std::to_string(poc) + ".txt";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    for(;;)
    {
      std::ifstream input(path);
      if( input.is_open() )
      {
        int filePoc = -1;
        if( !(input >> filePoc) || filePoc != poc )
          throw std::runtime_error("RL-RC-DoT: action POC mismatch: " + path);
        std::array<int,size> values = {};
        for( int i = 0; i < size; ++i )
        {
          if( !(input >> values[i]) || values[i] < -12 || values[i] > 12 )
            throw std::runtime_error("RL-RC-DoT: malformed action grid: " + path);
        }
        std::string extra;
        if( input >> extra )
          throw std::runtime_error("RL-RC-DoT: unexpected extra action fields: " + path);
        std::lock_guard<std::mutex> lock(mutex);
        return cache.emplace(poc, values).first->second;
      }
      if( std::chrono::steady_clock::now() >= deadline )
        throw std::runtime_error("RL-RC-DoT: timed out for action: " + path);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
};

}} // namespace vvenc::rl

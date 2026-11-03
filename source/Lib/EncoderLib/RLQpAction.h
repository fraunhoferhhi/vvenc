// Experimental RL-QP action reader for VVenC v1.14.0.
//
// This file is part of the research fork. The original VVenC Clear BSD License
// applies to this repository. The reader is deliberately independent of the
// PPO implementation: first validate a fixed action map, then replace this
// input boundary with an online agent bridge.
//
// Format of VVENC_RL_QP_MAP (ASCII text, one POC per line):
//   POC d00 d01 d02 d03 d10 d11 d12 d13 d20 d21 d22 d23
// where each d is an integer [-2, +2], row-major on a 4 x 3 CTU grid.
//
// Environment variable not set: all actions are zero and no file is opened.
// Environment variable set: require an explicit row for every encoded POC.
// Thread safety: the immutable map is initialized once; subsequent reads are
// safe for VVenC's parallel CTU workers.
// VVENC_RL_ACTION_DIR adds an online mode: Python writes one atomic action
// file named <POC>.txt after receiving the causal pre-encode observation.

#pragma once

#include <array>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

namespace vvenc {
namespace rl {

class QpActionMap
{
public:
  static constexpr int kWidth = 4;
  static constexpr int kHeight = 3;
  static constexpr int kElements = kWidth * kHeight;

  static bool enabled()
  {
    return config().enabled;
  }

  static int deltaFor( int poc, unsigned x, unsigned y, unsigned ctuSize,
                       unsigned widthInCtus, unsigned heightInCtus )
  {
    const Config& cfg = config();
    if( !cfg.enabled ) return 0;

    if( ctuSize == 0 || widthInCtus != kWidth || heightInCtus != kHeight )
    {
      throw std::runtime_error( "VVenC-RL: fixed 4x3 action map requires a 4x3 CTU grid" );
    }

    std::array<int,kElements> actions = {};
    if( !cfg.actionDir.empty() )
    {
      actions = liveActionsFor( poc, cfg.actionDir );
    }
    else
    {
      const auto it = cfg.actions.find( poc );
      if( it == cfg.actions.end() )
        throw std::runtime_error( "VVenC-RL: missing action row for POC " + std::to_string( poc ) );
      actions = it->second;
    }

    const unsigned column = x / ctuSize;
    const unsigned row = y / ctuSize;
    if( column >= kWidth || row >= kHeight )
    {
      throw std::runtime_error( "VVenC-RL: action CTU coordinates are out of range" );
    }

    return actions[row * kWidth + column];
  }

  static bool traceEnabled()
  {
    const char* flag = std::getenv( "VVENC_RL_TRACE" );
    return flag != nullptr && flag[0] == '1';
  }

  // These are RDO candidate evaluations, NOT independent PPO actions.
  // Multiple calls at the same QG position are expected while evaluating
  // hypothetical CU partition trees.
  static void trace( int poc, unsigned x, unsigned y,
                     int baseQp, int requestedDelta, int appliedQp,
                     unsigned currSubdiv, unsigned currDepth,
                     unsigned cuWidth, unsigned cuHeight )
  {
    if( !traceEnabled() ) return;
    std::lock_guard<std::mutex> lock( traceMutex() );
    std::cerr << "[VVENC_RL_QP] poc=" << poc
              << " x=" << x << " y=" << y
              << " qp_base=" << baseQp
              << " delta_requested=" << requestedDelta
              << " qp_applied=" << appliedQp
              << " subdiv=" << currSubdiv
              << " depth=" << currDepth
              << " cu_w=" << cuWidth
              << " cu_h=" << cuHeight << '\n';
  }

  // Recorded after the winning CU tree for this CTU has been selected.
  // Unlike the candidate trace above, there must be exactly one final
  // record for each POC / CTU raster address in the final encode pass.
  static void traceFinalCtu( int poc, unsigned ctuAddr, unsigned x, unsigned y,
                             int requestedDelta, unsigned numLumaCus,
                             int minQp, int maxQp )
  {
    if( !traceEnabled() ) return;
    std::lock_guard<std::mutex> lock( traceMutex() );
    std::cerr << "[VVENC_RL_CTU_FINAL] poc=" << poc
              << " ctu=" << ctuAddr
              << " x=" << x << " y=" << y
              << " delta_requested=" << requestedDelta
              << " num_luma_cus=" << numLumaCus
              << " min_qp=" << minQp
              << " max_qp=" << maxQp << '\n';
  }

private:
  static std::mutex& traceMutex()
  {
    static std::mutex mutex;
    return mutex;
  }

  static std::array<int,kElements> liveActionsFor( int poc, const std::string& directory )
  {
    static std::mutex cacheMutex;
    static std::map<int,std::array<int,kElements>> cached;

    {
      std::lock_guard<std::mutex> lock( cacheMutex );
      const auto it = cached.find( poc );
      if( it != cached.end() ) return it->second;
    }

    const std::string path = directory + "/" + std::to_string(poc) + ".txt";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds( 60 );
    for( ;; )
    {
      std::ifstream file( path );
      if( file.is_open() )
      {
        int filePoc = -1;
        if( !(file >> filePoc) || filePoc != poc )
          throw std::runtime_error("VVenC-RL: action file POC mismatch: " + path);

        std::array<int,kElements> action = {};
        for( int i = 0; i < kElements; ++i )
        {
          if( !(file >> action[i]) || action[i] < -2 || action[i] > 2 )
            throw std::runtime_error("VVenC-RL: malformed live QP map: " + path);
        }
        std::string extra;
        if( file >> extra )
          throw std::runtime_error("VVenC-RL: extra data in live QP map: " + path);
        std::lock_guard<std::mutex> lock( cacheMutex );
        const auto inserted = cached.emplace( poc, action );
        return inserted.first->second;
      }

      if( std::chrono::steady_clock::now() >= deadline )
        throw std::runtime_error("VVenC-RL: timed out waiting for PPO action: " + path);

      std::this_thread::sleep_for( std::chrono::milliseconds( 20 ) );
    }
  }

  struct Config
  {
    bool enabled = false;
    std::string actionDir;
    std::map<int, std::array<int, kElements>> actions;
  };

  static const Config& config()
  {
    static const Config cfg = []() {
      Config result;
      const char* path = std::getenv( "VVENC_RL_QP_MAP" );
      const char* liveDir = std::getenv( "VVENC_RL_ACTION_DIR" );
      const bool hasMap = path != nullptr && path[0] != '\0';
      const bool hasLiveDir = liveDir != nullptr && liveDir[0] != '\0';
      if( hasMap && hasLiveDir )
        throw std::runtime_error( "VVenC-RL: choose only one of VVENC_RL_QP_MAP and VVENC_RL_ACTION_DIR" );
      if( hasLiveDir )
      {
        if( std::getenv("VVENC_RL_STATE_FILE") == nullptr )
          throw std::runtime_error("VVenC-RL: live actions require VVENC_RL_STATE_FILE");
        result.enabled = true;
        result.actionDir = liveDir;
        return result;
      }
      if( !hasMap ) return result;

      std::ifstream file( path );
      if( !file.is_open() )
        throw std::runtime_error( std::string( "VVenC-RL: cannot open action map: " ) + path );

      std::string line;
      int lineNumber = 0;
      while( std::getline( file, line ) )
      {
        ++lineNumber;
        const std::size_t first = line.find_first_not_of( " \t\r\n" );
        if( first == std::string::npos || line[first] == '#' ) continue;

        std::istringstream fields( line );
        int poc = 0;
        if( !( fields >> poc ) || poc < 0 )
          throw std::runtime_error( "VVenC-RL: invalid POC at line " + std::to_string( lineNumber ) );

        std::array<int, kElements> action = {};
        for( int i = 0; i < kElements; ++i )
        {
          if( !( fields >> action[i] ) || action[i] < -2 || action[i] > 2 )
            throw std::runtime_error( "VVenC-RL: invalid delta QP at line " + std::to_string( lineNumber ) );
        }

        std::string extra;
        if( fields >> extra )
          throw std::runtime_error( "VVenC-RL: extra fields at line " + std::to_string( lineNumber ) );

        if( !result.actions.emplace( poc, action ).second )
          throw std::runtime_error( "VVenC-RL: duplicated POC " + std::to_string( poc ) );
      }

      if( result.actions.empty() )
        throw std::runtime_error( "VVenC-RL: action file contains no POC rows" );

      result.enabled = true;
      return result;
    }();
    return cfg;
  }
};

} // namespace rl
} // namespace vvenc

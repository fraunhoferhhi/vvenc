/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2024-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

/**
  \ingroup vvenclibtest
  \file    test_ThreadPool.cpp
  \brief   Unit test of NoMallocThreadPool: tasks run to the end, in dependency order, and no wake-up is lost.
*/

#include <atomic>
#include <chrono>
#include <cstdio>
#include <vector>

#include "Utilities/NoMallocThreadPool.h"
#include "vvenc/vvenc.h"

using namespace vvenc;

// A wavefront of tasks like the encoder's CTU tasks: each cell runs NUM_STAGES stages, a stage only after the left and
// top neighbours have finished it (the ready check), and the task returns false after every stage but the last.
struct PoolGrid
{
  static constexpr int WIDTH = 8, HEIGHT = 4, NUM_STAGES = 3;
  std::atomic<int>  stage[HEIGHT][WIDTH];
  std::atomic<bool> orderBroken{ false };
};

struct PoolGridCell
{
  PoolGrid* grid;
  int       x, y;
};

static bool poolGridCellReady( int, void* param )
{
  const auto& cell = *static_cast<PoolGridCell*>( param );
  const int next   = cell.grid->stage[cell.y][cell.x] + 1;
  return ( cell.x == 0 || cell.grid->stage[cell.y][cell.x - 1] >= next ) && ( cell.y == 0 || cell.grid->stage[cell.y - 1][cell.x] >= next );
}

static bool poolGridCellTask( int threadId, void* param )
{
  auto& cell = *static_cast<PoolGridCell*>( param );
  if( !poolGridCellReady( threadId, param ) )
  {
    cell.grid->orderBroken = true;
  }
  const int next = ++cell.grid->stage[cell.y][cell.x];
  return next == PoolGrid::NUM_STAGES;
}

// A root task that makes two partner tasks ready when it finishes; each partner waits until the other has started too.
// The thread that ran the root takes one partner, so the other runs only if that thread woke the sleeping idle
// thread after the root finished.
struct PoolPair
{
  std::atomic<bool> rootDone{ false };
  std::atomic<int>  numStarted{ 0 };
  std::atomic<bool> partnerTimedOut{ false };
};

static bool poolPairRoot( int, void* param )
{
  // run long enough for the idle thread that looks for work to fall asleep (its busy-wait is at most 1 ms here)
  const auto runUntil = std::chrono::steady_clock::now() + std::chrono::milliseconds( 2 );
  while( std::chrono::steady_clock::now() < runUntil ) {}
  static_cast<PoolPair*>( param )->rootDone = true;
  return true;
}

static bool poolPairPartnerReady( int, void* param )
{
  return static_cast<PoolPair*>( param )->rootDone;
}

static bool poolPairPartner( int, void* param )
{
  auto& pair = *static_cast<PoolPair*>( param );
  pair.numStarted++;
  // longer than the pool's safety wait in the test (1 s), so a lost wake-up is counted rather than timed out
  const auto giveUpAt = std::chrono::steady_clock::now() + std::chrono::seconds( 3 );
  while( pair.numStarted < 2 )
  {
    if( std::chrono::steady_clock::now() > giveUpAt )
    {
      pair.partnerTimedOut = true;
      break;
    }
  }
  return true;
}

// Every task of the wavefront runs to the end, in dependency order, with 1, 2 and 8 threads, and no wake-up of a
// sleeping idle thread is lost (the pool's safety net would hide it, so it counts them; its wait is 1 s here, so a
// producer delayed by preemption is not counted). The idle thread's busy-wait is 0 (it sleeps after every failed scan)
// and the default; the second wavefront starts after the first finished (with busy-wait 0 the idle thread then sleeps).
// With 2 and 8 threads, a root task and two partners check the wake-up after a task ran.
bool test_ThreadPool()
{
  VVEncCfg cfg;
  vvenc_config_default( &cfg );
  bool passed = true;
  for( auto idleBusyWait : { std::chrono::microseconds( 0 ), std::chrono::duration_cast<std::chrono::microseconds>( BUSY_WAIT_TIME ) } )
  for( int numThreads : { 1, 2, 8 } )
  {
    NoMallocThreadPool pool( numThreads, "UnitTestPool", &cfg, idleBusyWait, std::chrono::milliseconds( 1000 ) );
    for( int round = 0; round < 2; round++ )
    {
      PoolGrid grid;
      std::vector<PoolGridCell> cells;
      for( int y = 0; y < PoolGrid::HEIGHT; y++ )
      {
        for( int x = 0; x < PoolGrid::WIDTH; x++ )
        {
          grid.stage[y][x] = 0;
          cells.push_back( { &grid, x, y } );
        }
      }
      WaitCounter done;
      // added last cell first, so that most tasks wait for their neighbours
      for( auto cell = cells.rbegin(); cell != cells.rend(); cell++ )
      {
        // rows of three priorities, like pictures started at different times: a thread may claim a ready cell and
        // give it back for a more urgent one
        pool.addBarrierTask( poolGridCellTask, &*cell, &done, nullptr, {}, poolGridCellReady, cell->y % 3 );
      }
      done.wait();

      bool complete = true;
      for( auto& cell : cells )
      {
        complete &= grid.stage[cell.y][cell.x] == PoolGrid::NUM_STAGES;
      }
      if( !complete || grid.orderBroken )
      {
        printf( "ThreadPool: busy-wait %d us, %d threads, round %d: %s\n", int( idleBusyWait.count() ), numThreads, round,
                complete ? "dependency order broken" : "tasks left unfinished" );
        passed = false;
      }
    }
    if( numThreads >= 2 )
    {
      PoolPair pair;
      WaitCounter done;
      pool.addBarrierTask( poolPairPartner, &pair, &done, nullptr, {}, poolPairPartnerReady );
      pool.addBarrierTask( poolPairPartner, &pair, &done, nullptr, {}, poolPairPartnerReady );
      pool.addBarrierTask( poolPairRoot, &pair, &done );
      done.wait();
      if( pair.partnerTimedOut )
      {
        printf( "ThreadPool: busy-wait %d us, %d threads: the partner tasks did not run together\n", int( idleBusyWait.count() ), numThreads );
        passed = false;
      }
    }
    if( pool.numWakeUpsMissed() != 0 )
    {
      printf( "ThreadPool: busy-wait %d us, %d threads: %u wake-ups lost\n", int( idleBusyWait.count() ), numThreads,
              pool.numWakeUpsMissed() );
      passed = false;
    }
  }
  return passed;
}

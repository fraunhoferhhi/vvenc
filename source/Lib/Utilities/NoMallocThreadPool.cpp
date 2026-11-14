/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     NoMallocThreadPool.cpp
    \brief    thread pool
*/

#include "NoMallocThreadPool.h"

#include <limits>

#ifdef HAVE_PTHREADS
#  include <pthread.h>
#  define THREAD_MIN_STACK_SIZE 1024 * 1024
#endif


//! \ingroup Utilities
//! \{

namespace vvenc {

#if ENABLE_TIME_PROFILING_MT_MODE
thread_local std::unique_ptr<TProfiler> ptls;
#endif

NoMallocThreadPool::NoMallocThreadPool( int numThreads, const char * threadPoolName, const VVEncCfg* encCfg )
  : NoMallocThreadPool( numThreads, threadPoolName, encCfg, BUSY_WAIT_TIME, std::chrono::milliseconds( 50 ) )
{
}

NoMallocThreadPool::NoMallocThreadPool( int numThreads, const char * threadPoolName, const VVEncCfg* encCfg,
                                        std::chrono::microseconds idleBusyWait, std::chrono::milliseconds safetyWakeInterval )
  : m_poolName( threadPoolName )
  , m_idleBusyWait( idleBusyWait )
  , m_safetyWakeInterval( safetyWakeInterval )
{
  if( numThreads < 0 )
  {
    numThreads = std::thread::hardware_concurrency();
  }

  for( int i = 0; i < numThreads; ++i )
  {
    m_threads.emplace_back( &NoMallocThreadPool::threadProc, this, i, *encCfg );
  }
}

NoMallocThreadPool::~NoMallocThreadPool()
{
  m_exitThreads = true;
  wakeIdleThread();

  waitForThreads();
}

bool NoMallocThreadPool::processTasksOnMainThread()
{
  CHECK( m_threads.size() != 0, "should not be used with multiple threads" );

  bool         progress      = false;
  TaskIterator firstFailedIt = m_tasks.end();
  for( auto taskIt = findNextTask( 0, m_tasks.begin() ); taskIt.isValid(); taskIt = findNextTask( 0, taskIt ) )
  {
    const bool success = processTask( 0, *taskIt );
    progress |= success;

    if( taskIt == firstFailedIt )
    {
      if( success )
      {
        // first failed was successful -> reset
        firstFailedIt = m_tasks.end();
      }
      else if( progress )
      {
        // reset progress, try another round
        progress = false;
      }
      else
      {
        // no progress -> exit
        break;
      }
    }
    else if( !success && !firstFailedIt.isValid() )
    {
      firstFailedIt = taskIt;
    }
  }

  // return true if all done (-> false if some tasks blocked due to barriers)
  return std::all_of( m_tasks.begin(), m_tasks.end(), []( Slot& t ) { return t.state == FREE; } );
}

void NoMallocThreadPool::shutdown( bool block )
{
  m_exitThreads = true;
  wakeIdleThread();
  if( block )
  {
    waitForThreads();
  }
}

void NoMallocThreadPool::waitForThreads()
{
  for( auto& t: m_threads )
  {
    if( t.joinable() )
      t.join();
  }
}

void NoMallocThreadPool::threadProc( int threadId, const VVEncCfg& encCfg )
{
#if __linux
  if( !m_poolName.empty() )
  {
    std::string threadName( m_poolName + std::to_string( threadId ) );
    pthread_setname_np( pthread_self(), threadName.c_str() );
  }
#endif
#if ENABLE_TIME_PROFILING_MT_MODE
  ptls.reset( timeProfilerCreate( encCfg ) );
  {
    std::unique_lock< std::mutex > lock( m_nextFillSlotMutex );
    TProfiler *tp = ptls.get();
    profilers.push_back( tp );
  }
#endif

  auto nextTaskIt = m_tasks.begin();
  while( !m_exitThreads )
  {
    bool rescheduled = false;
    auto taskIt      = findNextTask( threadId, nextTaskIt, &rescheduled );
    if( rescheduled )
    {
      // a slot this thread held for its ready check was invisible to the idle thread's last scan
      wakeIdleThread();
    }
    if( !taskIt.isValid() )
    {
      taskIt = waitForTask( threadId, nextTaskIt );
    }
    if( m_exitThreads || !taskIt.isValid() )
    {
      return;
    }

    processTask( threadId, *taskIt );
    // a task that ran, finished or not, may have made other tasks ready
    wakeIdleThread();

    nextTaskIt = taskIt;
    nextTaskIt.incWrap();
  }
}

// Called when a scan found nothing to run. One idle thread at a time looks for work: the owner of m_idleMutex; the
// others wait for the mutex. The owner busy-waits m_idleBusyWait (while the pool is busy, tasks become ready within
// microseconds), then sleeps until a task is added, runs or makes progress (wakeIdleThread), so the CPU stays free
// while the encoder waits, e.g. for input.
NoMallocThreadPool::TaskIterator NoMallocThreadPool::waitForTask( int threadId, TaskIterator startSearch )
{
  ITT_TASKSTART( itt_domain_thrd, itt_handle_TPblocked );
  std::unique_lock<std::mutex> idleLock( m_idleMutex );
  ITT_TASKEND( itt_domain_thrd, itt_handle_TPblocked );

  ITT_TASKSTART( itt_domain_thrd, itt_handle_TPspinWait );
  TaskIterator taskIt;
  bool afterTimeout = false;
  auto sleepAt      = std::chrono::steady_clock::now() + m_idleBusyWait;
  while( !m_exitThreads )
  {
    taskIt = findNextTask( threadId, startSearch );
    if( taskIt.isValid() )
    {
      if( afterTimeout )
      {
        m_wakeUpsMissed.fetch_add( 1, std::memory_order_relaxed );
      }
      break;
    }
    afterTimeout = false;
    if( std::chrono::steady_clock::now() < sleepAt )
    {
      std::this_thread::yield();
      continue;
    }

    // Sleep. The flag is set before the last scan and producers check it after publishing their change, both behind
    // a full fence: either the scan sees the change or the producer sees the flag, so no wake-up is lost.
    m_idleThreadSleeps.store( true );
    std::atomic_thread_fence( std::memory_order_seq_cst );
    taskIt = findNextTask( threadId, startSearch );
    if( taskIt.isValid() || m_exitThreads )
    {
      m_idleThreadSleeps.store( false );   // a notify that may still come finds no sleeper, which is harmless
      break;
    }
    {
      std::unique_lock<std::mutex> sleepLock( m_sleepMutex );
      // safety net: look again at least every m_safetyWakeInterval (50 ms)
      const bool woken = m_sleepCond.wait_for( sleepLock, m_safetyWakeInterval, [this] { return !m_idleThreadSleeps.load() || m_exitThreads; } );
      // false from the exchange: a producer took the flag just now, so it did not miss this sleep
      afterTimeout = !woken && m_idleThreadSleeps.exchange( false );
    }
    sleepAt = std::chrono::steady_clock::now() + m_idleBusyWait;
  }
  ITT_TASKEND( itt_domain_thrd, itt_handle_TPspinWait );
  return taskIt;   // unlocking m_idleMutex lets the next waiting thread look for work
}

// After a change that can make a task ready: wake the idle thread if it sleeps (one fence and one load otherwise).
void NoMallocThreadPool::wakeIdleThread()
{
  std::atomic_thread_fence( std::memory_order_seq_cst );
  if( m_idleThreadSleeps.load( std::memory_order_relaxed ) && m_idleThreadSleeps.exchange( false ) )
  {
    {
      // Empty on purpose. The sleeper checks the flag while it holds this mutex and gives the mutex up only inside
      // wait_for. So once this thread had the mutex, the sleeper either finds the flag cleared and does not wait, or
      // it is inside wait_for already and gets the notify. No wake-up is lost, and it need not wait for this mutex
      // when it wakes up, as it would if the notify came under the lock.
      std::lock_guard<std::mutex> sleepLock( m_sleepMutex );
    }
    m_sleepCond.notify_one();
  }
}

// Returns the most urgent ready task (lowest priority value), claimed (RUNNING); among equally urgent ones the first one
// from startSearch, so the threads spread over the queue. Only a task more urgent than the best one found so far gets its
// barriers and ready check tested; a ready task of priority 0 ends the search at once. A claimed task that a more urgent
// one replaces goes back to WAITING (reported as rescheduled, like a failed ready check).
NoMallocThreadPool::TaskIterator NoMallocThreadPool::findNextTask( int threadId, TaskIterator startSearch, bool* rescheduled )
{
  if( !startSearch.isValid() )
  {
    startSearch = m_tasks.begin();
  }
  TaskIterator best;
  int64_t      bestPriority = std::numeric_limits<int64_t>::max();
  bool first = true;
  for( auto it = startSearch; it != startSearch || first; it.incWrap() )
  {
#if ENABLE_VALGRIND_CODE
    MutexLock lock( m_extraMutex );
#endif

    first = false;

    Slot& t = *it;
    if( t.state.load( std::memory_order_relaxed ) != WAITING || t.priority.load( std::memory_order_relaxed ) >= bestPriority )
    {
      continue;
    }
    auto expected = WAITING;
    if( !t.state.compare_exchange_strong( expected, RUNNING ) )
    {
      continue;
    }
    // read again now that the slot is ours: it may have been refilled since the first read
    const int64_t priority = t.priority.load( std::memory_order_relaxed );
    bool          ready    = priority < bestPriority;
    if( ready && !t.barriers.empty() )
    {
      ready = std::none_of( t.barriers.cbegin(), t.barriers.cend(), []( const Barrier* b ) { return b && b->isBlocked(); } );
      if( ready )
      {
        t.barriers.clear();   // clear barriers, so we don't need to check them on the next try (we assume they won't get locked again)
      }
    }
    ready = ready && ( !t.readyCheck || t.readyCheck( threadId, t.param ) );
    if( !ready )
    {
      // give the slot back: not more urgent than the best (refilled since the first read), blocked by a barrier, or not
      // ready yet; it was invisible to other scans meanwhile, so the caller wakes the idle thread
      t.state.store( WAITING );
      if( rescheduled ) *rescheduled = true;
      continue;
    }

    if( best.isValid() )
    {
      ( *best ).state.store( WAITING );
      if( rescheduled ) *rescheduled = true;
    }
    best         = it;
    bestPriority = priority;
    if( priority == 0 )
    {
      break;   // nothing is more urgent
    }
  }
  return best;
}

bool NoMallocThreadPool::processTask( int threadId, NoMallocThreadPool::Slot& task )
{
  const bool success = task.func( threadId, task.param );
#if ENABLE_VALGRIND_CODE
  MutexLock lock( m_extraMutex );
#endif
  if( !success )
  {
    task.state = WAITING;
    return false;
  }

  if( task.done != nullptr )
  {
    task.done->unlock();
  }
  if( task.counter != nullptr )
  {
    --(*task.counter);
  }

  task.state = FREE;

  return true;
}

#ifdef HAVE_PTHREADS

template<class TFunc, class... TArgs>
NoMallocThreadPool::PThread::PThread( TFunc&& func, TArgs&&... args )
{
  using WrappedCall     = std::function<void()>;
  std::unique_ptr<WrappedCall> call = std::make_unique<WrappedCall>( std::bind( func, args... ) );

  using PThreadsStartFn = void* (*) ( void* );
  PThreadsStartFn threadFn = []( void* p ) -> void*
  {
    std::unique_ptr<WrappedCall> call( static_cast<WrappedCall*>( p ) );

    ( *call )();

    return nullptr;
  };

  pthread_attr_t attr;
  int ret = pthread_attr_init( &attr );
  CHECK( ret != 0, "pthread_attr_init() failed" );

  try
  {
    size_t currStackSize = 0;
    ret = pthread_attr_getstacksize( &attr, &currStackSize );
    CHECK( ret != 0, "pthread_attr_getstacksize() failed" );

    if( currStackSize < THREAD_MIN_STACK_SIZE )
    {
      ret = pthread_attr_setstacksize( &attr, THREAD_MIN_STACK_SIZE );
      CHECK( ret != 0, "pthread_attr_setstacksize() failed" );

#  if defined( _DEBUG ) && !defined( __MINGW32__ ) && !defined( __MINGW64__ )
      ret = pthread_attr_setguardsize( &attr, 1024 * 1024 );   // set stack guard size to 1MB to more reliably deteck stack overflows
      CHECK( ret != 0, "pthread_attr_setguardsize() failed" );
#  endif
    }
    m_joinable = 0 == pthread_create( &m_id, &attr, threadFn, call.get() );
    CHECK( !m_joinable, "pthread_create() faild" );

    call.release();   // will now be freed by the thread

    pthread_attr_destroy( &attr );
  }
  catch( ... )
  {
    pthread_attr_destroy( &attr );
    throw;
  }
}

NoMallocThreadPool::PThread& NoMallocThreadPool::PThread::operator=( PThread&& other )
{
  m_id             = other.m_id;
  m_joinable       = other.m_joinable;
  other.m_id       = 0;
  other.m_joinable = false;
  return *this;
}

void NoMallocThreadPool::PThread::join()
{
  if( m_joinable )
  {
    m_joinable = false;
    pthread_join( m_id, nullptr );
  }
}

#endif   // HAVE_PTHREADS

} // namespace vvenc

//! \}


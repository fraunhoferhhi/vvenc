/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */


/** \file     NoMallocThreadPool.cpp
    \brief    thread pool
*/

#include "NoMallocThreadPool.h"


#if __linux
#include <pthread.h>
#endif

#include <emmintrin.h>

//! \ingroup Utilities
//! \{

namespace vvenc {


NoMallocThreadPool::NoMallocThreadPool( int numThreads, const char * threadPoolName )
  : m_poolName( threadPoolName )
  , m_threads ( numThreads < 0 ? std::thread::hardware_concurrency() : numThreads )
{
  int tid = 0;
  for( auto& t: m_threads )
  {
    t = std::thread( &NoMallocThreadPool::threadProc, this, tid++ );
  }
}

NoMallocThreadPool::~NoMallocThreadPool()
{
  m_exitThreads = true;

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

void NoMallocThreadPool::threadProc( int threadId )
{
#if __linux
  if( !m_poolName.empty() )
  {
    std::string threadName( m_poolName + std::to_string( threadId ) );
    pthread_setname_np( pthread_self(), threadName.c_str() );
  }
#endif

  auto nextTaskIt = m_tasks.begin();
  while( !m_exitThreads )
  {
    auto taskIt = findNextTask( threadId, nextTaskIt );
    if( !taskIt.isValid() )
    {
      std::unique_lock<std::mutex> l( m_idleMutex, std::defer_lock );

      ITT_TASKSTART( itt_domain_thrd, itt_handle_TPspinWait );
      m_waitingThreads.fetch_add( 1, std::memory_order_relaxed );
      const auto startWait = std::chrono::steady_clock::now();
      while( !m_exitThreads )
      {
        taskIt = findNextTask( threadId, nextTaskIt );
        if( taskIt.isValid() || m_exitThreads )
        {
          break;
        }

        if( !l.owns_lock()
            && m_waitingThreads.load( std::memory_order_relaxed ) > 1
            && ( BUSY_WAIT_TIME.count() == 0 || std::chrono::steady_clock::now() - startWait > BUSY_WAIT_TIME )
            && !m_exitThreads )
        {
          ITT_TASKSTART(itt_domain_thrd, itt_handle_TPblocked);
          l.lock();
          ITT_TASKEND(itt_domain_thrd, itt_handle_TPblocked);
        }
        else
        {
          std::this_thread::yield();
        }
      }
      m_waitingThreads.fetch_sub( 1, std::memory_order_relaxed );
      ITT_TASKEND( itt_domain_thrd, itt_handle_TPspinWait );
    }
    if( m_exitThreads )
    {
      return;
    }

    processTask( threadId, *taskIt );

    nextTaskIt = taskIt;
    nextTaskIt.incWrap();
  }
}

NoMallocThreadPool::TaskIterator NoMallocThreadPool::findNextTask( int threadId, TaskIterator startSearch )
{
  if( !startSearch.isValid() )
  {
    startSearch = m_tasks.begin();
  }
  bool first = true;
  for( auto it = startSearch; it != startSearch || first; it.incWrap() )
  {
#if ENABLE_VALGRIND_CODE
    MutexLock lock( m_extraMutex );
#endif

    first = false;

    Slot& t = *it;
    auto expected = WAITING;
    if( t.state.load( std::memory_order_relaxed ) == WAITING && t.state.compare_exchange_strong( expected, RUNNING ) )
    {
      if( !t.barriers.empty() )
      {
        if( std::any_of( t.barriers.cbegin(), t.barriers.cend(), []( const Barrier* b ) { return b && b->isBlocked(); } ) )
        {
          // reschedule
          t.state.store( WAITING, std::memory_order_relaxed );
          continue;
        }
        t.barriers.clear();   // clear barriers, so we don't need to check them on the next try (we assume they won't get locked again)
      }
      if( t.readyCheck && t.readyCheck( threadId, t.param ) == false )
      {
        // reschedule
        t.state.store( WAITING, std::memory_order_relaxed );
        continue;
      }

      return it;
    }
  }
  return {};
}

bool NoMallocThreadPool::processTask( int threadId, NoMallocThreadPool::Slot& task )
{
  const bool success = task.func( threadId, task.param );
  if( !success )
  {
    task.state = WAITING;
    return false;
  }

#if ENABLE_VALGRIND_CODE
  MutexLock lock( m_extraMutex );
#endif

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

} // namespace vvenc

//! \}


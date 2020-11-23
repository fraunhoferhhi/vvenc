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
/** \file     NoMallocThreadPool.h
    \brief    thread pool
*/

#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <iostream>
#include <array>

#include "CommonLib/CommonDef.h"

//! \ingroup Utilities
//! \{

namespace vvenc {

#ifdef TRACE_ENABLE_ITT
static __itt_domain* itt_domain_thrd = __itt_domain_create( "Threading" );

static __itt_string_handle* itt_handle_TPspinWait = __itt_string_handle_create( "Spin_Wait" );
static __itt_string_handle* itt_handle_TPblocked  = __itt_string_handle_create( "Blocked" );
static __itt_string_handle* itt_handle_TPaddTask  = __itt_string_handle_create( "Add_Task" );

//static long itt_TP_blocked = 1;

#endif //TRACE_ENABLE_ITT

#if ENABLE_VALGRIND_CODE
typedef std::unique_lock< std::mutex > MutexLock;
#endif

// block threads after busy-waiting this long
const static auto BUSY_WAIT_TIME = [] {
  const char *env = getenv( "BUSY_WAIT_TIME" );
  if( env )
    return std::chrono::milliseconds( atoi( env ) );
  return std::chrono::milliseconds( 1 );
}();


// enable this if tasks need to be added from mutliple threads
#define ADD_TASK_THREAD_SAFE 0


// ---------------------------------------------------------------------------
// Synchronization tools
// ---------------------------------------------------------------------------

struct Barrier
{
  void unlock()
  {
    m_lockState.store( false );
  }

  void lock()
  {
    m_lockState.store( true );
  }

  bool isBlocked() const
  {
    return m_lockState;
  }

  Barrier()  = default;
  ~Barrier() = default;
  explicit Barrier( bool locked ) : m_lockState( locked ) {}

  Barrier( const Barrier & ) = delete;
  Barrier( Barrier && )      = delete;

  Barrier& operator=( const Barrier & ) = delete;
  Barrier& operator=( Barrier && )      = delete;

private:
  std::atomic_bool m_lockState{ true };
};

struct BlockingBarrier
{
  void unlock()
  {
    std::unique_lock<std::mutex> l( m_lock );
    m_intBarrier.unlock();
    if( !m_intBarrier.isBlocked() )
    {
      m_cond.notify_all();
    }
  }

  void lock()
  {
    std::unique_lock<std::mutex> l( m_lock );
    m_intBarrier.lock();
  }

  bool isBlocked() const
  {
    return m_intBarrier.isBlocked();
  }

  void wait() const
  {
    BlockingBarrier* nonconst = const_cast<BlockingBarrier*>(this);

    std::unique_lock<std::mutex> l( nonconst->m_lock );
    nonconst->m_cond.wait( l, [=] { return !m_intBarrier.isBlocked(); } );
  }

  BlockingBarrier()  = default;
  ~BlockingBarrier() { std::unique_lock<std::mutex> l( m_lock ); } // ensure all threads have unlocked the mutex, when we start destruction

  BlockingBarrier( const BlockingBarrier& ) = delete;
  BlockingBarrier( BlockingBarrier&& )      = delete;

  BlockingBarrier& operator=( const BlockingBarrier& ) = delete;
  BlockingBarrier& operator=( BlockingBarrier&& ) = delete;

  // cast to const ref Barrier, so we can use it for thread pool tasks:
  operator const Barrier&() const { return m_intBarrier; }

private:
  Barrier                 m_intBarrier;
  std::condition_variable m_cond;
  std::mutex              m_lock;
};

struct WaitCounter
{
  int operator++()
  {
    std::unique_lock<std::mutex> l( m_lock );
    done.lock();
    return ++m_count;
  }

  int operator--()
  {
    std::unique_lock<std::mutex> l( m_lock );
    const unsigned int new_count = --m_count;
    if( new_count == 0 )
    {
      m_cond.notify_all();
      done.unlock();
    }
    l.unlock(); // unlock mutex after done-barrier to prevent race between barrier and counter
    return new_count;
  }

  bool isBlocked() const
  {
    return 0 != m_count;
  }

  void wait() const
  {
    WaitCounter* nonconst = const_cast<WaitCounter*>(this);

    std::unique_lock<std::mutex> l( nonconst->m_lock );
    nonconst->m_cond.wait( l, [=] { return m_count == 0; } );
  }

  WaitCounter() = default;
  ~WaitCounter() { std::unique_lock<std::mutex> l( m_lock ); }   // ensure all threads have unlocked the mutex, when we start destruction

  WaitCounter( const WaitCounter & ) = delete;
  WaitCounter( WaitCounter && )      = delete;

  WaitCounter &operator=( const WaitCounter & ) = delete;
  WaitCounter &operator=( WaitCounter && )      = delete;

  Barrier done{ false };

private:
  std::condition_variable m_cond;
  std::mutex              m_lock;
  unsigned int            m_count = 0;
};



// ---------------------------------------------------------------------------
// Thread Pool
// ---------------------------------------------------------------------------

using CBarrierVec = std::vector<const Barrier*>;

class NoMallocThreadPool
{
  typedef enum
  {
    FREE = 0,
    PREPARING,
    WAITING,
    RUNNING
  } TaskState;

  using TaskFunc = bool ( * )( int, void * );

  struct Slot
  {
    TaskFunc               func      { nullptr };
    TaskFunc               readyCheck{ nullptr };
    void*                  param     { nullptr };
    WaitCounter*           counter   { nullptr };
    Barrier*               done      { nullptr };
    CBarrierVec            barriers;
    std::atomic<TaskState> state     { FREE };
  };


  class ChunkedTaskQueue
  {
    constexpr static int ChunkSize = 128;

    class Chunk
    {
      std::array<Slot, ChunkSize> m_slots;
      std::atomic<Chunk*>         m_next{ nullptr };
      Chunk&                      m_firstChunk;

      Chunk( Chunk* firstPtr ) : m_firstChunk{ *firstPtr } {}

      friend class ChunkedTaskQueue;
    };

  public:
    class Iterator : public std::iterator<std::forward_iterator_tag, Slot>
    {
      Slot*  m_slot  = nullptr;
      Chunk* m_chunk = nullptr;

    public:
      Iterator() = default;
      Iterator( Slot* slot, Chunk* chunk ) : m_slot( slot ), m_chunk( chunk ) {}

      Iterator& operator++()
      {
        CHECKD( m_slot == nullptr, "incrementing invalid iterator" );
        CHECKD( m_chunk == nullptr, "incrementing invalid iterator" );

        if( m_slot != &m_chunk->m_slots.back() )
        {
          ++m_slot;
        }
        else
        {
          m_chunk = m_chunk->m_next;
          if( m_chunk )
          {
            m_slot  = &m_chunk->m_slots.front();
          }
          else
          {
            m_slot  = nullptr;
          }
        }
        return *this;
      }

      // increment iterator and wrap around, if end is reached
      Iterator& incWrap()
      {
        CHECKD( m_slot == nullptr, "incrementing invalid iterator" );
        CHECKD( m_chunk == nullptr, "incrementing invalid iterator" );

        if( m_slot != &m_chunk->m_slots.back() )
        {
          ++m_slot;
        }
        else
        {
          if( (Chunk*)m_chunk->m_next )
          {
            m_chunk = m_chunk->m_next;
          }
          else
          {
            m_chunk = &m_chunk->m_firstChunk;
          }
          m_slot = &m_chunk->m_slots.front();
        }
        return *this;
      }

      bool operator==( const Iterator& rhs ) const { return m_slot == rhs.m_slot; } // don't need to compare m_chunk, because m_slot is a pointer
      bool operator!=( const Iterator& rhs ) const { return m_slot != rhs.m_slot; } // don't need to compare m_chunk, because m_slot is a pointer

      Slot& operator*() { return *m_slot; }

      bool isValid() const { return m_slot != nullptr && m_chunk != nullptr; }
    };

    ChunkedTaskQueue() = default;
    ~ChunkedTaskQueue()
    {
      Chunk* next = m_firstChunk.m_next;
      while( next )
      {
        Chunk* curr = next;
        next = curr->m_next;
        delete curr;
      }
    }

    ChunkedTaskQueue( const ChunkedTaskQueue& ) = delete;
    ChunkedTaskQueue( ChunkedTaskQueue&& )      = delete;

    // grow the queue by adding a chunk and return an iterator to the first new task-slot
    Iterator grow()
    {
      std::unique_lock<std::mutex> l( m_resizeMutex );  // prevent concurrent growth of the queue. Read access while growing is no problem
//      std::cerr << __PRETTY_FUNCTION__ << std::endl;

      m_lastChunk->m_next = new Chunk( &m_firstChunk );
      m_lastChunk         = m_lastChunk->m_next;

      return Iterator{ &m_lastChunk->m_slots.front(), m_lastChunk };
    }

    Iterator begin() { return Iterator{ &m_firstChunk.m_slots.front(), &m_firstChunk }; }
    Iterator end()   { return Iterator{ nullptr, nullptr }; }

  private:
    Chunk  m_firstChunk{ &m_firstChunk };
    Chunk* m_lastChunk = &m_firstChunk;

    std::mutex m_resizeMutex;
  };


public:
  NoMallocThreadPool( int numThreads = 1, const char *threadPoolName = nullptr );
  ~NoMallocThreadPool();

  template<class TParam>
  bool addBarrierTask( bool             ( *func )( int, TParam* ),
                       TParam*             param,
                       WaitCounter*        counter                      = nullptr,
                       Barrier*            done                         = nullptr,
                       const CBarrierVec&& barriers                     = {},
                       bool             ( *readyCheck )( int, TParam* ) = nullptr )
  {
    if( m_threads.empty() )
    {
      // if singlethreaded, execute all pending tasks
      if( m_nextFillSlot != m_tasks.begin() )
      {
        processTasksOnMainThread();
      }

      // when no barriers block this task, execute it directly
      if( std::none_of( barriers.begin(), barriers.end(), []( const Barrier* b ) { return b && b->isBlocked(); } )
          && ( !readyCheck || readyCheck( 0, param ) ) )
      {
        if( func( 0, param ) )
        {
          if( done != nullptr )
          {
            done->unlock();
          }
          return true;
        }
      }
    }

    while( true )
    {
#if ADD_TASK_THREAD_SAFE
      std::unique_lock<std::mutex> l(m_nextFillSlotMutex);
#endif
      CHECKD( !m_nextFillSlot.isValid(), "Next fill slot iterator should always be valid" );
      const auto startIt = m_nextFillSlot;

#if ADD_TASK_THREAD_SAFE
      l.unlock();
#endif

      bool first = true;
      for( auto it = startIt; it != startIt || first; it.incWrap() )
      {
#if ENABLE_VALGRIND_CODE
        MutexLock lock( m_extraMutex );
#endif

        first = false;

        auto& t = *it;
        auto expected = FREE;
        if( t.state.load( std::memory_order_relaxed ) == FREE && t.state.compare_exchange_strong( expected, PREPARING ) )
        {
          if( counter )
          {
            counter->operator++();
          }

          t.func       = (TaskFunc)func;
          t.readyCheck = (TaskFunc)readyCheck;
          t.param      = param;
          t.done       = done;
          t.counter    = counter;
          t.barriers   = std::move( barriers );
          t.state      = WAITING;

#if ADD_TASK_THREAD_SAFE
          l.lock();
#endif
          m_nextFillSlot.incWrap();
          return true;
        }
      }

#if ADD_TASK_THREAD_SAFE
      l.lock();
#endif
      m_nextFillSlot = m_tasks.grow();
    }
    return false;
  }

  bool processTasksOnMainThread();

  void shutdown( bool block );
  void waitForThreads();

  int numThreads() const { return (int)m_threads.size(); }

private:

  using TaskIterator = ChunkedTaskQueue::Iterator;

  // members
  std::string              m_poolName;
  std::atomic_bool         m_exitThreads{ false };
  std::vector<std::thread> m_threads;
  ChunkedTaskQueue         m_tasks;
  TaskIterator             m_nextFillSlot = m_tasks.begin();
#if ADD_TASK_THREAD_SAFE
  std::mutex               m_nextFillSlotMutex;
#endif
  std::mutex               m_idleMutex;
  std::atomic_uint         m_waitingThreads{ 0 };
#if ENABLE_VALGRIND_CODE
  std::mutex               m_extraMutex;
#endif

  // internal functions
  void         threadProc  ( int threadId );
  TaskIterator findNextTask( int threadId, TaskIterator startSearch );
  bool         processTask ( int threadId, Slot& task );
};

} // namespace vvenc

//! \}


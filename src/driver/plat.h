// Cross-platform shims for the QuestLHSync SteamVR driver. Windows uses its own API directly; this
// header fills in what the POSIX/Linux port needs, with the same spellings the Windows code uses.
#pragma once
#if defined(_WIN32)
#include <windows.h>
#else
#include <atomic>
#include <cstdint>
#include <ctime>
#include <sched.h>
#include <cstring>
#include <unistd.h>

inline void Sleep(int ms) {
  timespec t{ms / 1000, (long)(ms % 1000) * 1000000L};
  nanosleep(&t, nullptr);
}

inline void YieldProcessor() { sched_yield(); }

inline void MemoryBarrier() { std::atomic_thread_fence(std::memory_order_seq_cst); }

inline uint64_t GetTickCount64() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000 + (uint64_t)t.tv_nsec / 1000000;
}

// the two Microsoft spellings the code uses, on the shared status memory's seqlock (std::atomic<int32_t> is
// layout-compatible with int32_t, so the seqlock's plain fields can be incremented atomically)
inline void QlhsSeqInc(volatile int32_t *p) {
  std::atomic<int32_t> *a = reinterpret_cast<std::atomic<int32_t> *>(const_cast<int32_t *>(p));
  a->fetch_add(1, std::memory_order_acq_rel);
}

// localtime_s(&tm, &t) (Microsoft's argument order) -> localtime_r
inline struct tm *localtime_s(struct tm *dest, const time_t *src) { return localtime_r(src, dest); }

// the two Winsock spellings the code keeps, on Linux they're plain descriptors
using SOCKET = int;
constexpr SOCKET INVALID_SOCKET = -1;
inline int closesocket(int s) { return ::close(s); }
#endif

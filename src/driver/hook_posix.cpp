#ifndef _WIN32
#include "hook_posix.h"

#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>

namespace hookp {
namespace {

// the protection of the mapping holding a (from /proc/self/maps), -1 if none does
int ProtAt(uintptr_t a) {
  FILE *f = fopen("/proc/self/maps", "r");
  if (!f) return -1;
  char line[512];
  int prot = -1;
  while (prot < 0 && fgets(line, sizeof line, f)) {
    unsigned long lo, hi;
    char perm[8] = "";
    if (sscanf(line, "%lx-%lx %7s", &lo, &hi, perm) != 3 || a < lo || a >= hi) continue;
    prot = (perm[0] == 'r' ? PROT_READ : 0) | (perm[1] == 'w' ? PROT_WRITE : 0) | (perm[2] == 'x' ? PROT_EXEC : 0);
  }
  fclose(f);
  return prot;
}

}  // namespace

bool SlotPut(void **slot, void *value) {
  uintptr_t ps = (uintptr_t)sysconf(_SC_PAGESIZE), page = (uintptr_t)slot & ~(ps - 1);
  int prot = ProtAt((uintptr_t)slot);  // a vtable: read-only after relocation
  if (prot < 0) return false;
  if (!(prot & PROT_WRITE) && mprotect((void *)page, ps, prot | PROT_READ | PROT_WRITE) != 0) return false;
  __atomic_store_n(slot, value, __ATOMIC_RELEASE);  // pointer-aligned: callers see the old entry or the new one
  if (!(prot & PROT_WRITE)) mprotect((void *)page, ps, prot);
  return true;
}

}  // namespace hookp
#endif

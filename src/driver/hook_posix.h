// Linux: the pose hook goes on IVRServerDriverHost's vtable entry for TrackedDevicePoseUpdated (MinHook on Windows).
// Every caller of that class goes through it, every driver included; swapping a pointer is atomic.
#pragma once
#ifndef _WIN32

namespace hookp {

// *slot = value, the page made writable for it and given back its protection after. False when it can't be
bool SlotPut(void **slot, void *value);

}  // namespace hookp
#endif

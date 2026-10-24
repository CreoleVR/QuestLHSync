// The desktop window's few SDL2 calls, loaded at run time: without libSDL2 (or a display) the app still serves the
// SteamVR dashboard page, it just has no window. SDL2's ABI is stable, so the declarations are written out here and no
// SDL headers are needed to build.
#pragma once
#include <dlfcn.h>

#include <cstdint>

namespace sdl {

enum : uint32_t {
  INIT_VIDEO = 0x20,
  WINDOW_SHOWN = 0x4, WINDOW_HIDDEN = 0x8, WINDOW_RESIZABLE = 0x20, WINDOW_MINIMIZED = 0x40, WINDOW_ALLOW_HIGHDPI = 0x2000,
  WINDOWPOS_UNDEFINED = 0x1FFF0000u,
  RENDERER_ACCELERATED = 0x2,
  PIXELFORMAT_ABGR8888 = 0x16762004u,  // RGBA bytes in memory on little-endian
  TEXTUREACCESS_STREAMING = 1,
  QUIT = 0x100, WINDOWEVENT = 0x200, MOUSEMOTION = 0x400, MOUSEBUTTONDOWN = 0x401, MOUSEBUTTONUP = 0x402,
  WE_EXPOSED = 3, WE_SIZE_CHANGED = 6, WE_LEAVE = 11, WE_CLOSE = 14,
  BUTTON_LEFT = 1,
  CURSOR_ARROW = 0, CURSOR_HAND = 11,
};

struct WindowEvent { uint32_t type, timestamp, windowID; uint8_t event, p1, p2, p3; int32_t data1, data2; };
struct MotionEvent { uint32_t type, timestamp, windowID, which, state; int32_t x, y, xrel, yrel; };
struct ButtonEvent { uint32_t type, timestamp, windowID, which; uint8_t button, state, clicks, p1; int32_t x, y; };
union Event {
  uint32_t type;
  WindowEvent window;
  MotionEvent motion;
  ButtonEvent button;
  uint8_t padding[56];
};

struct Api {
  void *lib = nullptr;
  int (*Init)(uint32_t);
  void (*Quit)();
  int (*SetHint)(const char *, const char *);
  void *(*CreateWindow)(const char *, int, int, int, int, uint32_t);
  void (*DestroyWindow)(void *);
  void (*HideWindow)(void *);
  uint32_t (*GetWindowFlags)(void *);
  void (*SetWindowIcon)(void *, void *);
  void *(*CreateRGBSurfaceFrom)(void *, int, int, int, int, uint32_t, uint32_t, uint32_t, uint32_t);
  void (*FreeSurface)(void *);
  void *(*CreateRenderer)(void *, int, uint32_t);
  int (*RenderSetLogicalSize)(void *, int, int);
  void *(*CreateTexture)(void *, uint32_t, int, int, int);
  int (*UpdateTexture)(void *, const void *, const void *, int);
  int (*SetRenderDrawColor)(void *, uint8_t, uint8_t, uint8_t, uint8_t);
  int (*RenderClear)(void *);
  int (*RenderCopy)(void *, void *, const void *, const void *);
  void (*RenderPresent)(void *);
  int (*PollEvent)(Event *);
  void *(*CreateSystemCursor)(int);
  void (*SetCursor)(void *);

  bool Load() {
    for (const char *n : {"libSDL2-2.0.so.0", "libSDL2-2.0.so", "libSDL2.so"})
      if ((lib = dlopen(n, RTLD_NOW | RTLD_LOCAL))) break;
    if (!lib) return false;
    bool ok = true;
    auto get = [&](auto &fn, const char *name) {
      *(void **)&fn = dlsym(lib, name);
      ok = ok && fn;
    };
    get(Init, "SDL_Init");
    get(Quit, "SDL_Quit");
    get(SetHint, "SDL_SetHint");
    get(CreateWindow, "SDL_CreateWindow");
    get(DestroyWindow, "SDL_DestroyWindow");
    get(HideWindow, "SDL_HideWindow");
    get(GetWindowFlags, "SDL_GetWindowFlags");
    get(SetWindowIcon, "SDL_SetWindowIcon");
    get(CreateRGBSurfaceFrom, "SDL_CreateRGBSurfaceFrom");
    get(FreeSurface, "SDL_FreeSurface");
    get(CreateRenderer, "SDL_CreateRenderer");
    get(RenderSetLogicalSize, "SDL_RenderSetLogicalSize");
    get(CreateTexture, "SDL_CreateTexture");
    get(UpdateTexture, "SDL_UpdateTexture");
    get(SetRenderDrawColor, "SDL_SetRenderDrawColor");
    get(RenderClear, "SDL_RenderClear");
    get(RenderCopy, "SDL_RenderCopy");
    get(RenderPresent, "SDL_RenderPresent");
    get(PollEvent, "SDL_PollEvent");
    get(CreateSystemCursor, "SDL_CreateSystemCursor");
    get(SetCursor, "SDL_SetCursor");
    if (!ok) { dlclose(lib); lib = nullptr; }
    return ok;
  }
};

}  // namespace sdl

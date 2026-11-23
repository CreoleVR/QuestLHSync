# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

QuestLHSync aligns SteamVR's lighthouse space (trackers, controllers, base stations) to a streamed standalone headset's
own tracking (Quest Pro / 3 / 3S, Steam Frame, Galaxy XR via an external module). The headset's tracking cameras see the
base stations' laser flashes; the PC driver solves the lighthouse → headset transform from them. README.md covers
user-facing behaviour, settings and troubleshooting.

## Build

Needs Visual Studio 2022 C++ (found via vswhere), the Android NDK, Zig (Frame cross-compile) and Python 3. There is no
project file: every build is a direct `cl` / clang / zig call in the scripts, so a new source file must be added to
the script's command line by hand.

```
build.bat                        driver DLL + dashboard app (QuestLHSync.exe) + out\QuestLHSync-Installer.exe
build.bat overlay                dashboard app only (close QuestLHSync.exe first)
build.bat installer              installer only
build_replay.bat                 out\qlhs_replay.exe and out\qlhs_nettest.exe (the offline tools)
python magisk\build_module.py    Quest Magisk module -> out\QuestLHSync-quest-module-<ver>.zip
python frame\build.py            Steam Frame package -> out\QuestLHSync-frame-<ver>.tar.gz
python3 frame/installer/build.py Frame installer Flatpak (arm64 Linux only)
python release.py                packs out\release-<ver>\; refuses if any build is older than its sources
python install.py [headset|remove]  register the source-tree driver with SteamVR / push the module over adb
```

SteamVR must be closed to rebuild the driver (it holds the DLL loaded). The driver builds into
`driver\questlhsync\bin\win64\` (gitignored), which is the folder `install.py` registers in place.

The single version number is `QLHS_RELEASE` in `src/common/qlhs_status.h`; every Python build script reads it from
there (via `magisk/build_module.py`'s `VERSION`). A release commit ("QuestLHSync 1.xx") changes only that line.

## Testing

There is no unit-test suite. The solver is checked offline against session recordings (the dashboard's **Record
session** writes them to `%LOCALAPPDATA%\QuestLHSync\recordings\`):

```
out\qlhs_replay.exe <recording> --calib <camera calibration json> --dir <scratch state dir> [--out f] [--learn] [--pings] [--rays f]
python src\tools\fake_lhsyncd.py <recording> <calibration json> [start_s] [dur_s]   # fake headset on the LAN protocol
out\qlhs_nettest.exe <recording> --dir <scratch state dir> [--family "Quest 3"]      # driver's network link + solver against it
```

The header comments of `src/tools/replay.cpp` and `nettest.cpp` list all flags. `docs/TESTING.md` has the full procedures
(headset choice, light checks, installer, dashboard), the reference recordings and the results to expect from them;
run its section 1 after any change to `sync.cpp`. Use a scratch `--dir`: the tools read
and write `state.json` / `stations.json` there like the live driver does in `%LOCALAPPDATA%\QuestLHSync`.

## Architecture

Three pieces talk over the LAN and shared memory:

**Headset daemon `lhsyncd`** (C). `src/headset/lhsyncd.c` is the shared core (UDP 47281 discovery, TCP 47280 stream,
clock round trips); each headset implements `src/headset/headset.h`:
- Quest: `magisk/src/quest.c`. The camera reader `lhsight` is a Frida CModule (`magisk/src/lhsight.c` inlined into
  `lhsight.js`) injected into the sensors HAL with frida-inject, which is pinned to 17.10.0 by hash in
  `build_module.py` (newer versions crashed the HAL). Started by `magisk/module/service.sh`.
- Steam Frame: `frame/src/frame.c` + `frame/src/lhsight.c` (a plain child process, no root, systemd user service).
  The camera dmabufs are handed over by `frame/driver/xrfds.cpp`, a device-less SteamVR driver running inside the
  Frame's own vrserver, because only an ancestor of XRService may duplicate its fds.
- lhsight is read-only on the camera buffers and only runs while a PC is connected. It sends text lines
  (`F` frames with blobs, `T` heartbeats, `I/W/E` messages); the wire format is documented at the top of
  `src/headset/lhsyncd.c`, `src/driver/net.h` and each lhsight source. Changing it means changing both ends.

**PC SteamVR driver** (`src/driver/`, C++17, one DLL):
- `driver_main.cpp`: OpenVR driver entry, settings, logging, recordings, the shared-memory status. MinHooks
  `IVRServerDriverHost::TrackedDevicePoseUpdated` so every driver's pose calls pass through it: lighthouse devices get
  the transform prepended to their WorldFromDriver, and the HMD's poses (from whatever streamer) feed the solver.
  Switches itself off for wired / lighthouse HMDs.
- `net.cpp/.h` (`HeadsetLink`): discovery, choosing among several headsets, the TCP session, clock pings. Several
  headsets of a kind: the one that streams with this PC wins (this PC's TCP table, or the `peers=` the headset reports
  in its discovery answer); the reasoning is in `net.h`'s top comment. Broadcasts often don't reach headsets on Wi-Fi,
  so discovery also asks remembered/host/streamer-peer addresses by unicast.
- `sync.cpp/.h`: the core, shared with the offline tools. `sync.h`'s top comment maps its parts (Optics, FrameGrid,
  Clock, PoseHist, Frame, Solver, Timing, Sync). The fit is 4-DOF (yaw + translation; both spaces are gravity-aligned).
  The tuning constants at the top of `sync.h` each carry a comment with the measurements that justify them.
- `gravity.cpp/.h`: levels the reference frame with two stations by reading lighthouse devices' IMUs from their HID
  receivers (read-only).
- `relations.cpp/.h`: hooks SteamVR's lighthouse driver's base station "relationship" step and feeds it a running
  average, so base stations stop jumping.
- State lives in `%LOCALAPPDATA%\QuestLHSync`: per-headset `headsets\<model>-<id>\state.json`, shared `stations.json`
  (reference frame), `gravity.json`, `relations.json`, `questlhsync.log`.

**Windows apps**:
- `src/overlay/overlay_main.cpp` → `QuestLHSync.exe`, the dashboard page, launched by the driver. Talks to it only
  through the seqlocked shared memory struct in `src/common/qlhs_status.h` (status out, commands in). Changing that
  struct means bumping `QLHS_VERSION`. `QuestLHSync.exe --preview out.png <state>` renders a page without VR.
- `src/installer/installer_main.cpp` → the installer. With a `QuestLHSync-steamvr-<ver>.zip` next to it (how a local
  build gets installed: run it from `out\release-<ver>\`) it installs that and stays offline unless its
  Downloads toggle is on; without one it downloads the latest GitHub release. Installs to
  `%LOCALAPPDATA%\QuestLHSync\questlhsync` and registers it with vrpathreg.

`third_party/` holds vendored MinHook and OpenVR (headers, lib, `openvr_api.dll`).

## Conventions

- Comments are full explanatory prose: why a value or step exists, with the measurements behind it. Keep new code in
  that style and update header comments (protocol lines, file maps) when behaviour changes.
- Commit subjects are one plain sentence describing the behaviour change, often "X, rather than Y"; releases are
  "QuestLHSync <version>".

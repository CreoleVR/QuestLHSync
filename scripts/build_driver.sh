#!/bin/sh
# Builds the SteamVR driver for Linux, driver/questlhsync/bin/linux64/driver_questlhsync.so, and the dashboard app
# QuestLHSync beside it. Run it through build.sh (build.sh --driver). The WiVRn/Monado app is scripts/build_wivrn.sh.
# SteamVR runs inside Valve's Steam Linux Runtime "sniper" container, so the driver is built in its SDK image (docker or
# podman; the image is pulled on first use) to load there. QLHS_HOST_BUILD=1 builds with the host's compiler instead,
# for a host SteamVR (outside the container), at the host's glibc.
set -e
cd "$(dirname "$0")/.."
IMAGE=registry.gitlab.steamos.cloud/steamrt/sniper/sdk:latest
OUT=driver/questlhsync/bin/linux64

compile() {
  mkdir -p build/driver "$OUT"
  gcc -c -O2 -fPIC -w -DLPBYTE='uint8_t *' -include string.h \
    -Ithird_party/minhook/src third_party/minhook/src/hde/hde64.c -o build/driver/hde64.o
  g++ -shared -fPIC -O2 -std=c++17 -Wall -Wno-unused-function -pthread \
    -fvisibility=hidden -fvisibility-inlines-hidden -static-libstdc++ -static-libgcc -Wl,-z,nodelete -Wl,--exclude-libs,ALL -Wl,--version-script=src/driver/exports.map \
    -Ithird_party/openvr/headers -Ithird_party/minhook/src \
    src/driver/driver_main.cpp src/driver/sync.cpp src/driver/net.cpp src/driver/gravity.cpp \
    src/driver/relations.cpp src/driver/hook_linux.cpp build/driver/hde64.o \
    -o "$OUT/driver_questlhsync.so"
  echo "built $OUT/driver_questlhsync.so"
  # the dashboard app: runs beside the driver, linked to the openvr_api library that sits next to it
  cp third_party/openvr/bin/linux64/libopenvr_api.so "$OUT/"
  g++ -O2 -std=c++17 -Wall -Wno-unused-function -static-libstdc++ -static-libgcc \
    -Ithird_party/openvr/headers -Ithird_party/stb $(pkg-config --cflags sdl2) src/overlay/overlay_main.cpp \
    -L"$OUT" -l:libopenvr_api.so $(pkg-config --libs sdl2) -Wl,-rpath,'$ORIGIN' -o "$OUT/QuestLHSync"
  echo "built $OUT/QuestLHSync"
}

if [ "$1" = "--compile" ] || [ -n "$QLHS_HOST_BUILD" ]; then
  compile
  exit 0
fi
ENGINE=$(command -v docker || command -v podman) || { echo "docker or podman needed (or QLHS_HOST_BUILD=1)"; exit 1; }
# rootless podman maps container uids onto host subuids, so -u's uid would have no write access to the mounted /src;
# keep-id maps the builder onto its own host uid. Docker and rootful podman don't remap uids and need no flag.
USERNS=""
if [ "$(basename "$ENGINE")" = podman ] && [ "$(id -u)" -ne 0 ]; then USERNS="--userns=keep-id"; fi
"$ENGINE" image inspect "$IMAGE" >/dev/null 2>&1 || "$ENGINE" pull "$IMAGE"
exec "$ENGINE" run --rm $USERNS -v "$PWD":/src -w /src -u "$(id -u):$(id -g)" "$IMAGE" sh scripts/build_driver.sh --compile

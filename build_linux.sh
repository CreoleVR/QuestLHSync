#!/bin/sh
# builds the SteamVR driver for Linux (driver/questlhsync/bin/linux64/driver_questlhsync.so). Needs g++ (or $CXX)
# with C++17. Close SteamVR before replacing a loaded driver. "build_linux.sh install" also registers
# driver/questlhsync with SteamVR (vrpathreg adddriver).
set -e
cd "$(dirname "$0")"
CXX=${CXX:-g++}
# libstdc++ linked in when its static library is there (no dependence on the libstdc++ SteamVR's runtime picks)
STATIC=
if echo 'int main(){}' | "$CXX" -x c++ - -static-libstdc++ -static-libgcc -o /dev/null 2>/dev/null; then
  STATIC="-static-libstdc++ -static-libgcc"
fi
OUT=driver/questlhsync/bin/linux64
mkdir -p "$OUT"
"$CXX" -shared -fPIC -O2 -std=c++17 -Wall -Wno-unused-function -fvisibility=hidden -pthread \
  -Ithird_party/openvr/headers \
  src/driver/driver_main.cpp src/driver/sync.cpp src/driver/net.cpp src/driver/gravity.cpp src/driver/relations.cpp \
  src/driver/platform.cpp \
  -o "$OUT/driver_questlhsync.so" $STATIC -Wl,--no-undefined -Wl,-z,defs -lrt -ldl
echo "built $OUT/driver_questlhsync.so"
if [ "$1" = install ]; then
  for d in "$HOME/.steam/steam" "$HOME/.local/share/Steam" \
           "$HOME/.var/app/com.valvesoftware.Steam/.local/share/Steam"; do
    REG="$d/steamapps/common/SteamVR/bin/vrpathreg.sh"
    [ -x "$REG" ] && break
  done
  [ -x "$REG" ] || { echo "SteamVR's vrpathreg.sh not found"; exit 1; }
  "$REG" adddriver "$PWD/driver/questlhsync"
  echo "registered $PWD/driver/questlhsync with SteamVR"
fi

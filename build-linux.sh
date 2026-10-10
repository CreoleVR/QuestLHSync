#!/bin/sh
# Builds the QuestLHSync PC driver for Linux: driver/questlhsync/bin/linux64/driver_questlhsync.so
# (build.bat's Windows build compiles the same sources with MSVC for win64)
set -e
cd "$(dirname "$0")"

CXX=${CXX:-g++}
FLAGS="-std=c++20 -O2 -fPIC -fvisibility=hidden -DNDEBUG -Wall -Ithird_party/openvr/headers -Isrc/driver"
SOURCES="src/driver/driver_main.cpp src/driver/sync.cpp src/driver/net.cpp src/driver/gravity.cpp src/driver/relations.cpp"
OUT=driver/questlhsync/bin/linux64

mkdir -p "$OUT"
# The driver's only entry is HmdDriverFactory (visibility default). No openvr_api link: the driver side is
# header-only through VRDriverContext. -pthread for the worker, reader and connection threads; -ldl for dlopen
# (openvr's IVRDriverContext); rt not needed (clock_gettime is in libc).
$CXX $FLAGS -shared -pthread -o "$OUT/driver_questlhsync.so" $SOURCES
echo "built $OUT/driver_questlhsync.so"

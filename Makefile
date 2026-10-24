# Linux build (x86-64). Windows builds with build.bat as before.
#
#   make            driver + dashboard app into driver/questlhsync/bin/linux64
#   make tools      out/qlhs_replay and out/qlhs_nettest
#   make package    out/QuestLHSync-steamvr-linux-<version>.tar.gz (the questlhsync folder, install.sh, notices)
#   make clean
#
# Needs g++ (or clang++) with C++17. libstdc++ and libgcc are linked in statically, as /MT does on Windows, so the
# driver doesn't depend on which libstdc++ SteamVR's runtime puts first. The binaries need the glibc they were built
# against or newer, so releases are built in Valve's Steam Runtime 3 (sniper) SDK (glibc 2.31), as
# .github/workflows/build.yml does:
#   docker run --rm -v "$PWD":/src -w /src registry.gitlab.steamos.cloud/steamrt/sniper/sdk make package

CXX      ?= g++
CC       ?= gcc
OPT      ?= -O2
CXXFLAGS ?=
LDFLAGS  ?=

OUT  := driver/questlhsync/bin/linux64
OVR  := third_party/openvr
STB  := third_party/stb
B    := build/linux

COMMON   := -fPIC -fvisibility=hidden -Wall -Wno-unused-function -Wno-sign-compare
CXXF     := -std=c++17 $(OPT) $(COMMON) $(CXXFLAGS)
STATICRT := -static-libstdc++ -static-libgcc

VERSION  := $(shell sed -n 's/^\#define QLHS_RELEASE "\(.*\)".*/\1/p' src/common/qlhs_status.h)

DRIVER_SRC := src/driver/driver_main.cpp src/driver/sync.cpp src/driver/net.cpp src/driver/gravity.cpp \
              src/driver/relations.cpp src/driver/hook_posix.cpp
DRIVER_OBJ := $(patsubst src/%.cpp,$(B)/%.o,$(DRIVER_SRC))

OVRAPI_SRC := $(OVR)/src/openvr_api_public.cpp $(OVR)/src/jsoncpp.cpp $(wildcard $(OVR)/src/vrcore/*.cpp)
OVRAPI_OBJ := $(patsubst $(OVR)/src/%.cpp,$(B)/openvr/%.o,$(OVRAPI_SRC))

OVERLAY_OBJ := $(B)/overlay/overlay_main.o $(B)/overlay/openvr_compat.o

all: $(OUT)/driver_questlhsync.so $(OUT)/QuestLHSync

$(B)/driver/%.o: src/driver/%.cpp $(wildcard src/driver/*.h) src/common/qlhs_status.h
	@mkdir -p $(dir $@)
	$(CXX) $(CXXF) -I$(OVR)/headers -c $< -o $@

$(OUT)/driver_questlhsync.so: $(DRIVER_OBJ)
	@mkdir -p $(OUT)
	$(CXX) -shared -o $@ $^ $(STATICRT) -Wl,--no-undefined -Wl,-z,defs -Wl,--version-script=src/driver/exports.map \
	  -Wl,-z,relro,-z,now -lpthread -ldl -lrt $(LDFLAGS)

$(B)/openvr/%.o: $(OVR)/src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXF) -w -DVRCORE_NO_PLATFORM -DLINUX -DLINUX64 -DPOSIX -I$(OVR)/headers -I$(OVR)/src -I$(OVR)/src/vrcore -c $< -o $@

$(B)/overlay/%.o: src/overlay/%.cpp src/overlay/*.h src/overlay/*.inc src/common/qlhs_status.h
	@mkdir -p $(dir $@)
	$(CXX) $(CXXF) -I$(OVR)/headers -I$(STB) -c $< -o $@

$(OUT)/QuestLHSync: $(OVERLAY_OBJ) $(OVRAPI_OBJ)
	@mkdir -p $(OUT)
	$(CXX) -o $@ $^ $(STATICRT) -Wl,-z,relro,-z,now -lpthread -ldl -lrt $(LDFLAGS)

tools: out/qlhs_replay out/qlhs_nettest

out/qlhs_replay: src/tools/replay.cpp src/driver/sync.cpp $(wildcard src/driver/*.h)
	@mkdir -p out
	$(CXX) $(CXXF) -o $@ src/tools/replay.cpp src/driver/sync.cpp $(STATICRT) -lpthread

out/qlhs_nettest: src/tools/nettest.cpp src/driver/net.cpp src/driver/sync.cpp $(wildcard src/driver/*.h)
	@mkdir -p out
	$(CXX) $(CXXF) -o $@ src/tools/nettest.cpp src/driver/net.cpp src/driver/sync.cpp $(STATICRT) -lpthread

package: all
	@mkdir -p out/pkg
	rm -rf out/pkg/*
	cp -r driver/questlhsync out/pkg/questlhsync
	rm -rf out/pkg/questlhsync/bin/win64
	cp linux/install.sh linux/uninstall.sh linux/README.md LICENSE THIRD_PARTY_NOTICES.md out/pkg/
	tar czf out/QuestLHSync-steamvr-linux-$(VERSION).tar.gz -C out/pkg .
	@echo built out/QuestLHSync-steamvr-linux-$(VERSION).tar.gz

clean:
	rm -rf $(B) out $(OUT)

.PHONY: all tools package clean

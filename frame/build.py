"""Builds QuestLHSync for the Steam Frame: out/QuestLHSync-frame-<version>.tar.gz

  lhsyncd                     ../src/headset/lhsyncd.c (shared with the Quest Pro) + src/frame.c + src/lhsight.c
  questlhsync_frame/          driver/xrfds.cpp: the SteamVR driver that lends lhsyncd the camera buffers
  install.sh, uninstall.sh,   package/
  questlhsync.service

Run on the Frame itself, or any arm64 Linux with glibc <= the Frame's: cc and c++ from PATH.
"""
import io
import os
import platform
import subprocess
import sys
import tarfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "magisk"))
from build_module import VERSION  # noqa: E402

HERE = os.path.join(ROOT, "frame")
BUILD = os.path.join(HERE, "build")
OUT = os.path.join(ROOT, "out")
HEADSET = os.path.join(ROOT, "src", "headset")
PACKAGE = os.path.join(OUT, f"QuestLHSync-frame-{VERSION}.tar.gz")


def main():
    if platform.machine() not in ("aarch64", "arm64"):
        sys.exit("build this on the Steam Frame (or another arm64 Linux): it's an arm64 Linux build")
    os.makedirs(BUILD, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    daemon = os.path.join(BUILD, "lhsyncd")
    driver = os.path.join(BUILD, "driver_questlhsync_frame.so")
    subprocess.run(["cc", "-O2", "-Wall", "-Wno-format-truncation", f'-DMODULE_VERSION="{VERSION}"', f"-I{HEADSET}",
                    os.path.join(HEADSET, "lhsyncd.c"), os.path.join(HERE, "src", "frame.c"),
                    os.path.join(HERE, "src", "lhsight.c"), "-s", "-o", daemon], check=True)
    subprocess.run(["c++", "-std=c++17", "-O2", "-Wall", "-fPIC", "-shared", "-fvisibility=hidden",
                    f"-I{os.path.join(ROOT, 'third_party', 'openvr', 'headers')}", os.path.join(HERE, "driver", "xrfds.cpp"),
                    "-pthread", "-s", "-o", driver], check=True)
    with tarfile.open(PACKAGE, "w:gz") as t:
        def add(name, path, mode):
            data = open(path, "rb").read()
            if not name.endswith((".so", "lhsyncd")):
                data = data.replace(b"\r\n", b"\n")
            info = tarfile.TarInfo(f"QuestLHSync-frame/{name}")
            info.size, info.mode, info.mtime = len(data), mode, 1790985600  # 2026-10-03
            t.addfile(info, io.BytesIO(data))
        add("lhsyncd", daemon, 0o755)
        add("questlhsync_frame/driver.vrdrivermanifest", os.path.join(HERE, "driver", "driver.vrdrivermanifest"), 0o644)
        add("questlhsync_frame/bin/linuxarm64/driver_questlhsync_frame.so", driver, 0o755)
        for f, mode in (("install.sh", 0o755), ("uninstall.sh", 0o755), ("questlhsync.service", 0o644)):
            add(f, os.path.join(HERE, "package", f), mode)
    print(f"built {PACKAGE} ({os.path.getsize(PACKAGE) / 1e3:.0f} KB)")
    return PACKAGE


if __name__ == "__main__":
    main()

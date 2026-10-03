"""Builds the QuestLHSync Magisk module: out/QuestLHSync-magisk-<version>.zip

  lhsyncd       src/lhsyncd.c, NDK clang for arm64 (Android 10+)
  lhsight.js    src/lhsight.js with src/lhsight.c inlined (Frida CModule), no time limit
  frida-inject  Frida 17.10.0 android-arm64, downloaded once from Frida's GitHub release into frida/ (checked by hash)

The NDK comes from ANDROID_NDK_HOME / ANDROID_NDK_ROOT, else the newest one in the Android SDK (ANDROID_HOME or
%LOCALAPPDATA%/Android/Sdk).
"""
import glob
import hashlib
import lzma
import os
import re
import subprocess
import sys
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "out")
with open(os.path.join(os.path.dirname(HERE), "src", "common", "qlhs_status.h"), encoding="utf-8") as _h:
    VERSION = "v" + re.search(r'#define QLHS_RELEASE "([^"]+)"', _h.read()).group(1)  # the driver's
CODE = 5
# 17.10.0 android-arm64, the build lhsight was developed on. 17.19.0 crashed the sensors HAL on injection (SIGSEGV in
# its bootstrap thread), so test any upgrade before shipping it.
FRIDA_VERSION = "17.10.0"
FRIDA_URL = f"https://github.com/frida/frida/releases/download/{FRIDA_VERSION}/frida-inject-{FRIDA_VERSION}-android-arm64.xz"
FRIDA_SHA256 = "b71f0d1a75e444605ae57cdf80f21256ed056082f559dcecdb72ab8838549fb8"  # the unpacked binary
FRIDA_INJECT = os.path.join(HERE, "frida", f"frida-inject-{FRIDA_VERSION}")


def lf(text):
    return text.replace("\r\n", "\n").encode("utf-8")


def clang():
    ndk = os.environ.get("ANDROID_NDK_HOME") or os.environ.get("ANDROID_NDK_ROOT")
    if not ndk:
        sdk = os.environ.get("ANDROID_HOME") or os.path.join(os.environ.get("LOCALAPPDATA", ""), "Android", "Sdk")
        found = sorted(glob.glob(os.path.join(sdk, "ndk", "*")))
        ndk = found[-1] if found else ""
    exe = os.path.join(ndk, "toolchains", "llvm", "prebuilt", "windows-x86_64", "bin", "clang.exe")
    if not os.path.exists(exe):
        sys.exit("Android NDK not found: set ANDROID_NDK_HOME")
    return exe


def frida_inject():
    if not os.path.exists(FRIDA_INJECT):
        print(f"downloading {FRIDA_URL}")
        data = lzma.decompress(urllib.request.urlopen(FRIDA_URL).read())
        os.makedirs(os.path.dirname(FRIDA_INJECT), exist_ok=True)
        with open(FRIDA_INJECT, "wb") as f:
            f.write(data)
    data = open(FRIDA_INJECT, "rb").read()
    if hashlib.sha256(data).hexdigest() != FRIDA_SHA256:
        sys.exit(f"{FRIDA_INJECT} isn't Frida's {FRIDA_VERSION} release (hash mismatch): delete it and build again")
    return data


def main():
    os.makedirs(os.path.join(HERE, "build"), exist_ok=True)
    os.makedirs(OUT, exist_ok=True)
    daemon = os.path.join(HERE, "build", "lhsyncd")
    subprocess.run([clang(), "--target=aarch64-linux-android29", "-O2", "-Wall", "-Wno-unused-result", "-fPIE", "-pie",
                    f'-DMODULE_VERSION="{VERSION}"', os.path.join(HERE, "src", "lhsyncd.c"), "-llog", "-s", "-o", daemon],
                   check=True)
    js = open(os.path.join(HERE, "src", "lhsight.js"), encoding="utf-8").read()
    c = open(os.path.join(HERE, "src", "lhsight.c"), encoding="utf-8").read()
    js = js.replace("@CSRC@", c).replace("@DUR@", "0")
    prop = open(os.path.join(HERE, "module", "module.prop.in"), encoding="utf-8").read()
    prop = prop.replace("@VERSION@", VERSION).replace("@CODE@", str(CODE))
    zpath = os.path.join(OUT, f"QuestLHSync-magisk-{VERSION}.zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        def add(name, data, mode=0o644):
            info = zipfile.ZipInfo(name, date_time=(2026, 10, 3, 0, 0, 0))
            info.external_attr = (0o100000 | mode) << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, data)
        m = os.path.join(HERE, "module")
        for rel in ("META-INF/com/google/android/update-binary", "META-INF/com/google/android/updater-script",
                    "service.sh", "customize.sh", "uninstall.sh"):
            add(rel, lf(open(os.path.join(m, rel), encoding="utf-8").read()), 0o755 if rel.endswith((".sh", "binary")) else 0o644)
        add("module.prop", lf(prop))
        add("lhsight.js", lf(js))
        add("lhsyncd", open(daemon, "rb").read(), 0o755)
        add("frida-inject", frida_inject(), 0o755)
    print(f"built {zpath} ({os.path.getsize(zpath) / 1e6:.1f} MB)")
    return zpath


if __name__ == "__main__":
    sys.exit(0 if main() else 1)

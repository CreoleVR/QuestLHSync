"""Packs out/QuestLHSync-<version>.zip from what build.bat, magisk/build_module.py and frame/build.py built:

  questlhsync/                         the SteamVR driver folder (driver, dashboard app, openvr_api.dll)
  QuestLHSync-magisk-<version>.zip     the Quest Pro's Magisk module
  QuestLHSync-frame-<version>.tar.gz   the Steam Frame's package (built on the Frame: copy it into out\\)
  README.md, LICENSE, THIRD_PARTY_NOTICES.md

Refuses when a build is older than its sources, so a release never ships stale binaries.
"""
import os
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "magisk"))
from build_module import VERSION  # noqa: E402

DRIVER = os.path.join(HERE, "driver", "questlhsync")
BIN = os.path.join(DRIVER, "bin", "win64")
MODULE = os.path.join(HERE, "out", f"QuestLHSync-magisk-{VERSION}.zip")
FRAME = os.path.join(HERE, "out", f"QuestLHSync-frame-{VERSION}.tar.gz")


def newest(*dirs):
    t = 0
    for d in dirs:
        for root, _, files in os.walk(os.path.join(HERE, d)):
            t = max([t] + [os.path.getmtime(os.path.join(root, f)) for f in files])
    return t


def main():
    checks = [
        (os.path.join(BIN, "driver_questlhsync.dll"), newest("src/driver", "src/common", "third_party"), "build.bat"),
        (os.path.join(BIN, "QuestLHSync.exe"), newest("src/overlay", "src/common", "third_party"), "build.bat"),
        (MODULE, newest("magisk/src", "magisk/module", "src/headset"), "python magisk\\build_module.py"),
        (FRAME, newest("frame/src", "frame/driver", "frame/package", "src/headset"),
         "python3 frame/build.py on the Steam Frame, and copy it into out\\"),
    ]
    for path, src, how in checks:
        if not os.path.exists(path) or os.path.getmtime(path) < src:
            sys.exit(f"{os.path.relpath(path, HERE)} is missing or older than its sources: run {how}")
    out = os.path.join(HERE, "out", f"QuestLHSync-{VERSION.lstrip('v')}.zip")
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for rel in ("driver.vrdrivermanifest", "resources/settings/default.vrsettings",
                    "bin/win64/driver_questlhsync.dll", "bin/win64/QuestLHSync.exe", "bin/win64/openvr_api.dll"):
            z.write(os.path.join(DRIVER, rel), "questlhsync/" + rel)
        z.write(MODULE, os.path.basename(MODULE))
        z.write(FRAME, os.path.basename(FRAME))
        for f in ("README.md", "LICENSE", "THIRD_PARTY_NOTICES.md"):
            z.write(os.path.join(HERE, f), f)
    print(f"built {out} ({os.path.getsize(out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()

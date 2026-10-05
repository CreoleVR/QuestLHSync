"""Packs the release files into out/release-<version>/ from what build.bat, magisk/build_module.py and frame/build.py
built, one per device:

  QuestLHSync-steamvr-installer.exe        installs and updates the SteamVR driver (downloads the zip below)
  QuestLHSync-steamvr-<version>.zip        the SteamVR driver folder questlhsync/ (driver, dashboard app, openvr_api.dll)
  QuestLHSync-quest-module-<version>.zip   the Quest's Magisk module
  QuestLHSync-frame-module-<version>.tar.gz  the Steam Frame's package
  QuestLHSync-frame-installer.flatpak      the Steam Frame's installer app, when out/ has one (built on arm64 Linux)

Each package carries LICENSE and THIRD_PARTY_NOTICES.md. Refuses when a build is older than its sources, so a release
never ships stale binaries.
"""
import glob
import io
import os
import shutil
import sys
import tarfile
import time
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "magisk"))
from build_module import VERSION  # noqa: E402

DRIVER = os.path.join(HERE, "driver", "questlhsync")
BIN = os.path.join(DRIVER, "bin", "win64")
INSTALLER = os.path.join(HERE, "out", "QuestLHSync-Installer.exe")
MODULE = os.path.join(HERE, "out", f"QuestLHSync-quest-module-{VERSION.lstrip('v')}.zip")
FRAME = os.path.join(HERE, "out", f"QuestLHSync-frame-{VERSION}.tar.gz")
NOTICES = ("LICENSE", "THIRD_PARTY_NOTICES.md")


def newest(*dirs):
    t = 0
    for d in dirs:
        for root, _, files in os.walk(os.path.join(HERE, d)):
            t = max([t] + [os.path.getmtime(os.path.join(root, f)) for f in files])
    return t


def steamvr_zip(out):
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for rel in ("driver.vrdrivermanifest", "resources/settings/default.vrsettings",
                    "bin/win64/driver_questlhsync.dll", "bin/win64/QuestLHSync.exe", "bin/win64/openvr_api.dll"):
            z.write(os.path.join(DRIVER, rel), "questlhsync/" + rel)
        for f in NOTICES:
            z.write(os.path.join(HERE, f), f)


def module_zip(out):
    with zipfile.ZipFile(MODULE) as src, zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for info in src.infolist():  # as built: Magisk reads its META-INF and module.prop from it
            z.writestr(info, src.read(info))
        for f in NOTICES:
            z.write(os.path.join(HERE, f), f)


def frame_tar(out):
    with tarfile.open(FRAME, "r:gz") as src, tarfile.open(out, "w:gz") as t:
        for m in src.getmembers():
            t.addfile(m, src.extractfile(m) if m.isfile() else None)
        for f in NOTICES:
            with open(os.path.join(HERE, f), "rb") as fh:
                data = fh.read()
            info = tarfile.TarInfo(f"QuestLHSync-frame/{f}")
            info.size, info.mode, info.mtime = len(data), 0o644, int(time.time())
            t.addfile(info, io.BytesIO(data))


def main():
    checks = [
        (os.path.join(BIN, "driver_questlhsync.dll"), newest("src/driver", "src/common", "third_party"), "build.bat"),
        (os.path.join(BIN, "QuestLHSync.exe"), newest("src/overlay", "src/common", "third_party"), "build.bat"),
        (INSTALLER, newest("src/installer"), "build.bat installer"),
        (MODULE, newest("magisk/src", "magisk/module", "src/headset"), "python magisk\\build_module.py"),
        (FRAME, newest("frame/src", "frame/driver", "frame/package", "src/headset"),
         "python frame\\build.py"),
    ]
    for path, src, how in checks:
        if not os.path.exists(path) or os.path.getmtime(path) < src:
            sys.exit(f"{os.path.relpath(path, HERE)} is missing or older than its sources: run {how}")
    ver = VERSION.lstrip("v")
    dest = os.path.join(HERE, "out", f"release-{ver}")
    shutil.rmtree(dest, ignore_errors=True)
    os.makedirs(dest)
    shutil.copy2(INSTALLER, os.path.join(dest, "QuestLHSync-steamvr-installer.exe"))
    steamvr_zip(os.path.join(dest, f"QuestLHSync-steamvr-{ver}.zip"))
    module_zip(os.path.join(dest, f"QuestLHSync-quest-module-{ver}.zip"))
    frame_tar(os.path.join(dest, f"QuestLHSync-frame-module-{ver}.tar.gz"))
    flatpaks = sorted(glob.glob(os.path.join(HERE, "out", "QuestLHSync-frame-installer*.flatpak")), key=os.path.getmtime)
    if flatpaks:
        shutil.copy2(flatpaks[-1], os.path.join(dest, "QuestLHSync-frame-installer.flatpak"))
    for f in sorted(os.listdir(dest)):
        print(f"  {f}  ({os.path.getsize(os.path.join(dest, f)) / 1e6:.1f} MB)")
    if not flatpaks:
        print("  (no out/QuestLHSync-frame-installer*.flatpak: attach the Frame's installer app on its own)")
    print(f"built {dest}")


if __name__ == "__main__":
    main()

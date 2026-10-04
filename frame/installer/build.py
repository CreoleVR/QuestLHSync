"""Builds the Steam Frame's installer app: out/QuestLHSync-frame-installer-<version>.flatpak

  questlhsync_installer.py    the GTK window: fetches a release, runs its install.sh / uninstall.sh on the host
  io.github.CreoleVR.QuestLHSync.Installer.*   Flatpak manifest, desktop entry, metainfo, icon

Needs Linux with flatpak, flatpak-builder (on PATH, or the org.flatpak.Builder Flatpak) and the GNOME 50 SDK:
  flatpak install --user flathub org.flatpak.Builder org.gnome.Sdk//50
Builds for this machine's architecture: run it on the Frame (or any arm64 Linux) for the Frame.
The bundle pulls its GNOME runtime from Flathub when installed.
"""
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "magisk"))
from build_module import VERSION  # noqa: E402

HERE = os.path.join(ROOT, "frame", "installer")
APP_ID = "io.github.CreoleVR.QuestLHSync.Installer"
BUILD = os.path.join(ROOT, "frame", "build", "installer")
OUT = os.path.join(ROOT, "out")
BUNDLE = os.path.join(OUT, f"QuestLHSync-frame-installer-{VERSION}.flatpak")


def builder():
    if shutil.which("flatpak-builder"):
        return ["flatpak-builder"]
    if subprocess.run(["flatpak", "info", "org.flatpak.Builder"], capture_output=True).returncode == 0:
        return ["flatpak", "run", "org.flatpak.Builder"]
    sys.exit("flatpak-builder not found: flatpak install --user flathub org.flatpak.Builder org.gnome.Sdk//50")


def main():
    if not shutil.which("flatpak"):
        sys.exit("flatpak not found: build the installer on Linux")
    os.makedirs(OUT, exist_ok=True)
    repo = os.path.join(BUILD, "repo")
    subprocess.run([*builder(), "--force-clean", f"--state-dir={os.path.join(BUILD, 'state')}", f"--repo={repo}",
                    os.path.join(BUILD, "app"), os.path.join(HERE, f"{APP_ID}.yml")], check=True)
    subprocess.run(["flatpak", "build-bundle", "--runtime-repo=https://dl.flathub.org/repo/flathub.flatpakrepo",
                    repo, BUNDLE, APP_ID], check=True)
    print(f"built {BUNDLE} ({os.path.getsize(BUNDLE) / 1e3:.0f} KB)")
    return BUNDLE


if __name__ == "__main__":
    main()

# QuestLHSync

Lighthouse trackers, controllers and base stations aligned to a Quest's or a
Steam Frame's own tracking, with nothing mounted on the headset. The headset's
tracking cameras see the laser flashes of SteamVR 2.0 base stations, and
QuestLHSync uses them to keep the lighthouse space lined up with the
headset's while you play. No SpaceCalibrator, no tracker strapped to your head.

It has two parts: a **headset part**, which serves what the cameras see on your
local network (a Magisk module on a Quest, a user service and a small SteamVR
driver on a Steam Frame), and a **SteamVR driver** for the PC, which solves the
alignment, applies it to every lighthouse device and adds a page to the SteamVR
dashboard.

Steam Frame support is by [@NotZoeyDev](https://github.com/NotZoeyDev).

![The QuestLHSync dashboard page](docs/dashboard.png)

## Requirements

- A **Quest Pro, Quest 3 or Quest 3S**, rooted with
  [Magisk](https://github.com/topjohnwu/Magisk). Tested on the Quest Pro. The
  Quest 3 has the same tracking cameras but hasn't been tried yet, nor has
  the 3S.
- Or a **Steam Frame**. No root, no developer password: everything installs as
  your user.
- **SteamVR 2.0 base stations** (1.0 isn't supported).
- **Windows with SteamVR**, with the headset streamed by anything: Steam Link,
  Link, Air Link, Virtual Desktop, ALVR, CreoleCast...
- The PC and the headset on the **same local network**.
- At least **one lighthouse device switched on** (a tracker or an Index
  controller). SteamVR only shows base stations while one is on.

## Install

Each device has its own file in
[Releases](https://github.com/CreoleVR/QuestLHSync/releases).

1. **Headset:**
   - **Quest:** install `QuestLHSync-quest-module-<version>.zip` in the Magisk
     app (**Modules > Install from storage**) and reboot. Or over adb:

     ```
     adb push QuestLHSync-quest-module-<version>.zip /sdcard/Download/
     adb shell su -c "magisk --install-module /sdcard/Download/QuestLHSync-quest-module-<version>.zip"
     adb reboot
     ```

   - **Steam Frame:** in Desktop Mode, open `QuestLHSync-frame-installer.flatpak`
     in Discover, then start **QuestLHSync Installer** and click **Install**. Or
     copy `QuestLHSync-frame-module-<version>.tar.gz` to the Frame and, in a
     terminal on it or over ssh:

     ```
     tar xzf QuestLHSync-frame-module-<version>.tar.gz
     QuestLHSync-frame/install.sh
     ```

     Either way, SteamVR on the headset restarts once to load its driver.

2. **PC:** run `QuestLHSync-steamvr-installer.exe`. It downloads the latest
   release, installs the driver to `%LOCALAPPDATA%\QuestLHSync\questlhsync`,
   registers it with SteamVR (in place of a copy you registered by hand) and
   offers an update whenever a newer release is out. It closes SteamVR if it's
   running.

   Or by hand: extract the `questlhsync` folder from
   `QuestLHSync-steamvr-<version>.zip` somewhere permanent and, with SteamVR
   closed, register it:

   ```
   "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe" adddriver "C:\path\to\questlhsync"
   ```

3. Turn off SpaceCalibrator, OpenVR-SpaceSync or anything else that moves
   lighthouse devices. Two tools correcting the same devices fight each other.

## Use

Start SteamVR as usual. QuestLHSync finds the headset by itself. Its page is in
the SteamVR dashboard, with a copy on the desktop.

The first time, look around so the cameras catch both base stations (the
Frame's upper cameras see most of them). It locks within about half a minute,
and after that starts from the saved alignment. Wear a tracker or hold a
controller meanwhile: when two base stations could be either way round, the
devices you wear or hold decide.

With three base stations, glance at the third one too. QuestLHSync's reference
frame keeps whatever tilt SteamVR's lighthouse space had when it was first
seen, and with two base stations in view a tilt of 1.5° puts trackers on the
floor 15 cm to the side. Once the cameras have seen three base stations well
enough to tell, QuestLHSync levels the reference frame with the headset's
gravity and keeps the level in `stations.json`. With two, the lighthouse
controllers' and trackers' accelerometers level it as they move about.

- **Pause corrections** holds lighthouse devices where they are.
- **Record session** saves what the driver receives to a file, for bug reports.

## Settings

Optional, in `steamvr.vrsettings` under `"driver_questlhsync"`:

| Key | Default | |
|---|---|---|
| `enable` | `true` | `false` turns QuestLHSync off |
| `host` | `""` | The headset's IP address(es), comma-separated, for networks that drop broadcasts |
| `headset` | `""` | A headset serial to prefer when several answer |
| `anyHmd` | `false` | Use SteamVR's headset even when it isn't named a Quest Pro, 3, 3S or Steam Frame |
| `record` | `false` | Record every session (same as the button) |
| `gravity` | `true` | `false` turns levelling by the lighthouse devices' accelerometers off |

## Troubleshooting

- **"Looking for the headset":** the headset must be awake and on the same
  network. If your router drops broadcasts, set `host`.
- **"No camera frames":** the cameras only run while the headset tracks. Put it
  on. If it stays there, check `%LOCALAPPDATA%\QuestLHSync\questlhsync.log`:
  a `no camera buffers found` line means the headset part doesn't know this
  headset or OS build yet. Please open an issue with that line. On the Frame, a
  "questlhsync_frame SteamVR driver isn't running" line means SteamVR on the
  headset hasn't restarted since the install:
  `systemctl --user restart steamvr.service`.
- **"Waiting for base stations":** switch on a tracker or controller.
- **"Finding the base stations" for a long time:** face each base station for a
  few seconds.

## Privacy and security

The PC side writes only to `%LOCALAPPDATA%\QuestLHSync` and only talks to the
headset. The headset part answers anyone on the local network without
authentication. It sends bright-spot positions (never images), the camera
calibration, and the headset's serial number, model and firmware. Use it on a
network you trust. It only reads the cameras while a PC is connected, and
patches nothing on disk. On the Frame, `lhsyncd` logs to the user journal
(`journalctl --user -u questlhsync`).

## Uninstall

- **Quest:** remove the module in the Magisk app and reboot.
- **Steam Frame:** click **Uninstall** in QuestLHSync Installer, or run
  `QuestLHSync-frame/uninstall.sh` on the headset.
- **PC:** with SteamVR closed, click **Uninstall** in the installer, or run
  `vrpathreg removedriver "C:\path\to\questlhsync"` and delete
  `%LOCALAPPDATA%\QuestLHSync`.

## Building

Visual Studio 2022 with C++, the Android NDK, [Zig](https://ziglang.org) and
Python 3:

```
build.bat                        SteamVR driver + dashboard app (SteamVR closed) + out\QuestLHSync-Installer.exe
build.bat installer              only the installer
python magisk\build_module.py    Quest module (Magisk), into out\
python frame\build.py            Steam Frame package, into out\
python release.py                the release files, into out\release-<version>\
python install.py                register driver\questlhsync with SteamVR
python install.py headset        install the module over adb, no reboot
```

Zig cross-compiles the Steam Frame package (`zig` on PATH, or `ZIG` set to it).
On the Frame itself or any arm64 Linux, `frame/build.py` uses `cc` and `c++`
instead.

The Steam Frame's installer app is a Flatpak, built on Linux for the machine's
own architecture (so on the Frame, or any arm64 Linux) with
`python3 frame/installer/build.py`, into
`out/QuestLHSync-frame-installer-<version>.flatpak`. It needs flatpak-builder
and the GNOME SDK:
`flatpak install --user flathub org.flatpak.Builder org.gnome.Sdk//50`.
`release.py` adds it to the release files as `QuestLHSync-frame-installer.flatpak`
when it's in `out/`.

`lhsyncd`'s core (`src/headset/lhsyncd.c`) is shared by both headsets; each adds
its own side behind `src/headset/headset.h` (`magisk/src/quest.c`,
`frame/src/frame.c`).

## License

MIT, see [LICENSE](LICENSE). Third-party code is listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

QuestLHSync is not affiliated with Meta or Valve. Meta Quest is a trademark of
Meta Platforms, Inc.; Steam Frame and SteamVR are trademarks of Valve
Corporation. Rooting a headset and running code inside its system services is
at your own risk.

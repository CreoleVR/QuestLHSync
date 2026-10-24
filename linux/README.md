# QuestLHSync on Linux

The PC side of [QuestLHSync](https://github.com/CreoleVR/QuestLHSync) (the
SteamVR driver and its dashboard page) for SteamVR on Linux. The headset side is
the same on either PC: the Quest module or the Steam Frame package from the
[releases](https://github.com/CreoleVR/QuestLHSync/releases), installed as the
[main README](https://github.com/CreoleVR/QuestLHSync#install) says.

## What's different from Windows

- **`steadyStations` is Windows-only.** On Linux, SteamVR's lighthouse driver is
  left alone: base stations sit where SteamVR places them, and the setting is
  ignored.
- **Data folder:** `~/.local/share/QuestLHSync` (`$XDG_DATA_HOME/QuestLHSync`)
  in place of `%LOCALAPPDATA%\QuestLHSync`: the log, `stations.json`,
  `headset.txt`, recordings and each headset's `state.json`.
- **Installer:** `install.sh` in place of the installer app.
- **Dashboard page:** sent with `SetOverlayRaw`, at most 5 times a second and
  only while it's shown. The desktop copy needs `libSDL2` and a display;
  without them the page is in the SteamVR dashboard only.

Everything else (the alignment, the headset link, levelling by gravity, the
settings, the saved state) works as on Windows.

## Requirements

- SteamVR from a native Steam install (Flatpak Steam: see below), with the
  headset streamed to it (Steam Link, ALVR, ...).
- glibc 2.31 or newer.
- For levelling by gravity, read access to the lighthouse receivers'
  `/dev/hidraw*` nodes. Steam's udev rules (the `steam-devices` package, or
  `60-steam-vr.rules`) give it; without them levelling is off.
- Optional: `libSDL2` for the desktop window, `fontconfig` to pick the page's
  fonts (DejaVu or Noto otherwise).

## Install

With SteamVR closed:

```
mkdir questlhsync && tar xzf QuestLHSync-steamvr-linux-<version>.tar.gz -C questlhsync
cd questlhsync && ./install.sh
```

`install.sh` copies the driver to `~/.local/share/QuestLHSync/questlhsync` and
registers it with SteamVR's `vrpathreg.sh`. It looks in every Steam library; if
SteamVR is somewhere else, run `STEAMVR=/path/to/SteamVR ./install.sh`.
`./uninstall.sh` takes it out again (the log and saved alignment stay).

With Flatpak Steam, copy the `questlhsync` folder somewhere under
`~/.var/app/com.valvesoftware.Steam` and register it from inside the Flatpak:

```
flatpak run --command=bash com.valvesoftware.Steam -c '<SteamVR>/bin/vrpathreg.sh adddriver <that folder>'
```

Settings go in SteamVR's `steamvr.vrsettings` (usually
`~/.steam/steam/config/steamvr.vrsettings`), as on Windows.

## Building

```
make            driver + dashboard app into driver/questlhsync/bin/linux64
make tools      out/qlhs_replay, out/qlhs_nettest
make package    out/QuestLHSync-steamvr-linux-<version>.tar.gz
```

Needs `make` and `g++` or `clang++` with C++17, nothing else: OpenVR's client
library is built from `third_party/openvr/src`, and SDL2 is loaded at run time
when it's there. The binaries need the glibc they were built against or newer,
so release packages are built in Valve's Steam Runtime 3 SDK (glibc 2.31), as
`.github/workflows/build.yml` does:

```
docker run --rm -v "$PWD":/src -w /src registry.gitlab.steamos.cloud/steamrt/sniper/sdk make package
```

## Tested with

A Steam Frame streamed by Steam Link, four base stations, SteamVR 2.18.4, on
Garuda Linux (KDE Plasma 6.8 on Wayland): locked, levelled by gravity, the
dashboard page in SteamVR and on the desktop. Not tried yet with a Quest on
Linux (ALVR, Steam Link): reports welcome, with
`~/.local/share/QuestLHSync/questlhsync.log`.

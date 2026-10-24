#!/bin/sh
# QuestLHSync for SteamVR on Linux: installs the driver (and its dashboard app) to ~/.local/share/QuestLHSync/questlhsync
# and registers it with SteamVR. Run it from the unpacked release folder (or from the source tree after `make`), with
# SteamVR closed.
#   STEAMVR=/path/to/SteamVR ./install.sh     when SteamVR isn't in one of Steam's usual places
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$HERE/questlhsync
[ -f "$SRC/bin/linux64/driver_questlhsync.so" ] || SRC=$HERE/../driver/questlhsync  # the source tree, after make
[ -f "$SRC/bin/linux64/driver_questlhsync.so" ] || { echo "no driver_questlhsync.so next to this script: run make first" >&2; exit 1; }

DATA=${XDG_DATA_HOME:-$HOME/.local/share}
DEST=$DATA/QuestLHSync/questlhsync

# SteamVR: $STEAMVR, else every Steam library of the usual Steam roots
find_steamvr() {
  if [ -n "$STEAMVR" ]; then echo "$STEAMVR"; return; fi
  for root in "$HOME/.steam/steam" "$HOME/.steam/root" "$DATA/Steam" "$HOME/.local/share/Steam"; do
    [ -d "$root" ] || continue
    libs=$root
    if [ -f "$root/steamapps/libraryfolders.vdf" ]; then
      libs="$libs $(sed -n 's/^[[:space:]]*"path"[[:space:]]*"\(.*\)".*/\1/p' "$root/steamapps/libraryfolders.vdf")"
    fi
    for lib in $libs; do
      if [ -x "$lib/steamapps/common/SteamVR/bin/vrpathreg.sh" ]; then echo "$lib/steamapps/common/SteamVR"; return; fi
    done
  done
}
VR=$(find_steamvr)
if [ -z "$VR" ] || [ ! -x "$VR/bin/vrpathreg.sh" ]; then
  if [ -d "$HOME/.var/app/com.valvesoftware.Steam" ]; then
    echo "Only the Flatpak Steam was found. This script handles a native Steam install; for the Flatpak, copy the" >&2
    echo "questlhsync folder somewhere under ~/.var/app/com.valvesoftware.Steam and run, inside the Flatpak:" >&2
    echo "  flatpak run --command=bash com.valvesoftware.Steam -c '<SteamVR>/bin/vrpathreg.sh adddriver <that folder>'" >&2
  else
    echo "SteamVR not found: install it from Steam, or set STEAMVR=/path/to/SteamVR" >&2
  fi
  exit 1
fi
VRPATHREG=$VR/bin/vrpathreg.sh

if pgrep -x vrserver >/dev/null 2>&1; then
  echo "SteamVR is running: close it first (it keeps the old driver loaded)" >&2
  exit 1
fi

mkdir -p "$DEST/bin/linux64" "$DEST/resources/settings"
# new files next to the old ones, then renamed over them
put() { cp "$SRC/$1" "$DEST/$1.new" && chmod "$2" "$DEST/$1.new" && mv -f "$DEST/$1.new" "$DEST/$1"; }
put driver.vrdrivermanifest 644
put resources/settings/default.vrsettings 644
put bin/linux64/driver_questlhsync.so 755
put bin/linux64/QuestLHSync 755

"$VRPATHREG" adddriver "$DEST" >/dev/null 2>&1 || { echo "vrpathreg adddriver failed ($VRPATHREG)" >&2; exit 1; }
if "$VRPATHREG" show 2>/dev/null | grep -q "$DEST"; then
  echo "QuestLHSync installed in $DEST and registered with SteamVR ($VR)"
else
  echo "QuestLHSync installed in $DEST; SteamVR's driver list doesn't show it yet, check: $VRPATHREG show" >&2
fi
echo "Turn off SpaceCalibrator or anything else that moves lighthouse devices, then start SteamVR."
echo "Logs and saved alignment: $DATA/QuestLHSync"

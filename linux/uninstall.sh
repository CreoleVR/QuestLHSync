#!/bin/sh
# Unregisters QuestLHSync's driver from SteamVR and removes it. Logs and the saved alignment in
# ~/.local/share/QuestLHSync stay; delete that folder too to start over.
set -e
DATA=${XDG_DATA_HOME:-$HOME/.local/share}
DEST=$DATA/QuestLHSync/questlhsync
if pgrep -x vrserver >/dev/null 2>&1; then
  echo "SteamVR is running: close it first" >&2
  exit 1
fi
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
if [ -n "$VR" ] && [ -x "$VR/bin/vrpathreg.sh" ]; then
  "$VR/bin/vrpathreg.sh" removedriver "$DEST" >/dev/null 2>&1 || true
else
  echo "SteamVR not found: removing the folder only (unregister it with SteamVR's bin/vrpathreg.sh removedriver $DEST)" >&2
fi
rm -rf "$DEST"
echo "QuestLHSync removed (logs and saved alignment kept in $DATA/QuestLHSync)"

#!/bin/sh
# Removes QuestLHSync from the Steam Frame. The driver stays loaded until SteamVR on the headset restarts.
DEST=$HOME/.local/share/questlhsync
systemctl --user disable --now questlhsync.service 2>/dev/null
rm -f "$HOME/.config/systemd/user/questlhsync.service"
systemctl --user daemon-reload
/opt/steamvr/bin/linuxarm64/vrpathreg removedriver "$DEST/questlhsync_frame" >/dev/null 2>&1
rm -rf "$DEST"
echo "QuestLHSync removed (restart SteamVR on the headset to unload its driver)"

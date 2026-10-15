#!/bin/sh
# Builds QuestLHSync for WiVRn/Monado: out/questlhsync-xr, the desktop app with the dashboard page, and
# out/QuestLHSync-wivrn.AppImage, which packs it with SDL2 and a font. Run it through build.sh (build.sh --wivrn). It
# compiles in Valve's sniper SDK image (docker or podman), like the driver; SDL2 and the fonts come from that image too.
# The OpenXR loader and libmonado are the system's: install openxr and WiVRn or Monado (25.0+, not the WiVRn Flatpak).
set -e
cd "$(dirname "$0")/.."
IMAGE=registry.gitlab.steamos.cloud/steamrt/sniper/sdk:latest
APP=build/xrappdir

stage() {  # in the container
  mkdir -p out
  g++ -O2 -std=c++17 -Wall -Wno-unused-function -pthread -static-libstdc++ -static-libgcc \
    -Ithird_party -Ithird_party/openxr -Ithird_party/stb $(pkg-config --cflags sdl2) \
    src/xr/xr_main.cpp src/driver/sync.cpp src/driver/net.cpp $(pkg-config --libs sdl2) -ldl -Wl,-rpath,'$ORIGIN/../lib' -o out/questlhsync-xr
  echo "built out/questlhsync-xr"
  mkdir -p "$APP/usr/lib" "$APP/usr/share/fonts" "$APP/usr/share/doc/fonts-dejavu"
  cp -L "$(ldconfig -p | awk '/libSDL2-2.0.so.0 .*x86-64/ {print $NF; exit}')" "$APP/usr/lib/libSDL2-2.0.so.0"
  cp /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf /usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf \
     /usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf "$APP/usr/share/fonts/"
  cp /usr/share/doc/fonts-dejavu-core/copyright "$APP/usr/share/doc/fonts-dejavu/copyright"
}

if [ "$1" = "--stage" ]; then
  stage
  exit 0
fi

ENGINE=$(command -v docker || command -v podman) || { echo "docker or podman needed"; exit 1; }
# rootless podman maps container uids onto host subuids, so -u's uid would have no write access to the mounted /src;
# keep-id maps the builder onto its own host uid. Docker and rootful podman don't remap uids and need no flag.
USERNS=""
if [ "$(basename "$ENGINE")" = podman ] && [ "$(id -u)" -ne 0 ]; then USERNS="--userns=keep-id"; fi
rm -rf "$APP"
mkdir -p "$APP/usr/bin"
"$ENGINE" run --rm $USERNS -v "$PWD":/src -w /src -u "$(id -u):$(id -g)" "$IMAGE" sh scripts/build_wivrn.sh --stage
cp out/questlhsync-xr "$APP/usr/bin/"
cp LICENSE THIRD_PARTY_NOTICES.md "$APP/usr/share/doc/"

cat > "$APP/AppRun" <<'EOT'
#!/bin/sh
HERE="$(dirname "$(readlink -f "$0")")"
export APPDIR="${APPDIR:-$HERE}"
export QLHS_FONT_DIR="$APPDIR/usr/share/fonts"
# only SDL2 is ours: keep the bundled folder out of the way of the runtime's own libraries (WiVRn, the OpenXR loader)
unset LD_LIBRARY_PATH
exec "$APPDIR/usr/bin/questlhsync-xr" "$@"
EOT
chmod +x "$APP/AppRun"
cat > "$APP/questlhsync-wivrn.desktop" <<'EOT'
[Desktop Entry]
Type=Application
Name=QuestLHSync for WiVRn
Comment=Aligns lighthouse trackers to a Steam Frame or Quest under WiVRn/Monado
Exec=questlhsync-xr
Icon=questlhsync-wivrn
Categories=Utility;
Terminal=false
EOT
out/questlhsync-xr --icon "$APP/questlhsync-wivrn.png"
cp "$APP/questlhsync-wivrn.png" "$APP/.DirIcon"

mkdir -p build/tools out
TOOL=build/tools/appimagetool
[ -x "$TOOL" ] || { curl -fsSL -o "$TOOL" https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage && chmod +x "$TOOL"; }
ARCH=x86_64 "$TOOL" --appimage-extract-and-run "$APP" out/QuestLHSync-wivrn.AppImage
echo "built out/QuestLHSync-wivrn.AppImage"

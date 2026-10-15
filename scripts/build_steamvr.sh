#!/bin/sh
# Builds the Linux SteamVR installer as an AppImage: out/QuestLHSync-steamvr-installer.AppImage. It carries the
# driver and the dashboard app (scripts/build_driver.sh builds them), SDL2 and a font, and needs only glibc 2.31+ and a display from the
# system, so it runs on any distro with FUSE (or with --appimage-extract-and-run). Everything is built in Valve's sniper
# SDK image (docker or podman), like build_driver.sh. Run it through build.sh (build.sh --installer; the flag builds the driver first). appimagetool is downloaded to build/tools on first use.
set -e
cd "$(dirname "$0")/.."
IMAGE=registry.gitlab.steamos.cloud/steamrt/sniper/sdk:latest
APP=build/appdir
LIB=driver/questlhsync/bin/linux64

stage() {  # in the container: the installer and what it needs from the image
  mkdir -p build/installer "$APP/usr/bin" "$APP/usr/lib" "$APP/usr/share/fonts" "$APP/usr/share/doc/fonts-dejavu"
  g++ -O2 -std=c++17 -Wall -Wno-unused-function -static-libstdc++ -static-libgcc -pthread \
    -Ithird_party/stb $(pkg-config --cflags sdl2) src/installer/installer_linux.cpp \
    $(pkg-config --libs sdl2) -l:libz.a -ldl -Wl,-rpath,'$ORIGIN/../lib' -o "$APP/usr/bin/questlhsync-installer"
  cp -L "$(ldconfig -p | awk '/libSDL2-2.0.so.0 .*x86-64/ {print $NF; exit}')" "$APP/usr/lib/libSDL2-2.0.so.0"
  cp /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf /usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf \
     /usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf "$APP/usr/share/fonts/"
  cp /usr/share/doc/fonts-dejavu-core/copyright "$APP/usr/share/doc/fonts-dejavu/copyright"
  echo "built $APP/usr/bin/questlhsync-installer"
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
"$ENGINE" image inspect "$IMAGE" >/dev/null 2>&1 || "$ENGINE" pull "$IMAGE"
[ -n "$QLHS_DRIVER_BUILT" ] || scripts/build_driver.sh
rm -rf "$APP"
"$ENGINE" run --rm $USERNS -v "$PWD":/src -w /src -u "$(id -u):$(id -g)" "$IMAGE" sh scripts/build_steamvr.sh --stage

# the payload: the driver folder as the installer copies it
PAY="$APP/usr/share/questlhsync-payload"
mkdir -p "$PAY/questlhsync/bin/linux64" "$PAY/questlhsync/resources/settings"
cp driver/questlhsync/driver.vrdrivermanifest "$PAY/questlhsync/"
cp driver/questlhsync/resources/settings/default.vrsettings "$PAY/questlhsync/resources/settings/"
cp "$LIB/driver_questlhsync.so" "$LIB/QuestLHSync" "$LIB/libopenvr_api.so" "$PAY/questlhsync/bin/linux64/"
sed -n 's/^#define QLHS_RELEASE "\(.*\)".*/\1/p' src/common/qlhs_status.h > "$PAY/VERSION"
cp LICENSE THIRD_PARTY_NOTICES.md "$APP/usr/share/doc/"

cat > "$APP/AppRun" <<'EOF'
#!/bin/sh
HERE="$(dirname "$(readlink -f "$0")")"
export APPDIR="${APPDIR:-$HERE}"
export QLHS_FONT_DIR="$APPDIR/usr/share/fonts"
exec "$APPDIR/usr/bin/questlhsync-installer" "$@"
EOF
chmod +x "$APP/AppRun"
cat > "$APP/questlhsync-installer.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=QuestLHSync Installer
Comment=Installs the QuestLHSync SteamVR driver
Exec=questlhsync-installer
Icon=questlhsync-installer
Categories=Utility;
Terminal=false
EOF
"$LIB/QuestLHSync" --preview "$APP/questlhsync-installer.png" icon
cp "$APP/questlhsync-installer.png" "$APP/.DirIcon"

mkdir -p build/tools out
TOOL=build/tools/appimagetool
[ -x "$TOOL" ] || { curl -fsSL -o "$TOOL" https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage && chmod +x "$TOOL"; }
ARCH=x86_64 "$TOOL" --appimage-extract-and-run "$APP" out/QuestLHSync-steamvr-installer.AppImage
echo "built out/QuestLHSync-steamvr-installer.AppImage"

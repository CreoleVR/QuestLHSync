#!/usr/bin/env python3
"""QuestLHSync installer for the Steam Frame: a window around the release's own install.sh and uninstall.sh.

Install takes the latest release from GitHub, or a QuestLHSync-<version>.zip or QuestLHSync-frame-<version>.tar.gz you
pick, unpacks QuestLHSync-frame/ and runs its install.sh on the headset: lhsyncd as a systemd user service, and the
questlhsync_frame driver registered with the headset's SteamVR. Uninstall runs frame/package/uninstall.sh. Inside
Flatpak, both run on the host through flatpak-spawn --host.
"""
import json
import os
import re
import shutil
import subprocess
import tarfile
import threading
import urllib.request
import zipfile

import gi

gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
from gi.repository import Adw, Gio, GLib, Gtk  # noqa: E402

APP_ID = "io.github.CreoleVR.QuestLHSync.Installer"
LATEST = "https://api.github.com/repos/CreoleVR/QuestLHSync/releases/latest"
FRAME_TAR = re.compile(r"(^|/)QuestLHSync-frame-[^/]*\.tar\.gz$")
RELEASE_ZIP = re.compile(r"QuestLHSync-[^/]*\.zip$")
# the Flatpak installs this file and frame/package/uninstall.sh in the same layout as the source tree
UNINSTALL = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "package", "uninstall.sh")
STEAMVR = "/opt/steamvr"
VRPATHREG = f"{STEAMVR}/bin/linuxarm64/vrpathreg"
SERVICE = "questlhsync.service"
# install.sh runs on the host, so it is unpacked where the host sees the same path: in Flatpak, ~/.var/app/<id>/cache
WORK = os.path.join(GLib.get_user_cache_dir(), "questlhsync-installer")
# The Frame's Desktop Mode is a Plasma session nested in Game Mode with XDG_RUNTIME_DIR=/run/user/<uid>/nested_plasma
# and its own session bus, whose flatpak-spawn runs host commands with that too. systemctl --user finds the user's
# systemd at $XDG_RUNTIME_DIR/systemd/private, so every host command gets the real one.
HOST = [*(["flatpak-spawn", "--host"] if os.path.exists("/.flatpak-info") else []),
        "env", f"XDG_RUNTIME_DIR=/run/user/{os.getuid()}"]


class Failure(Exception):
    pass


def host(*argv):
    return subprocess.run([*HOST, *argv], capture_output=True, text=True)


def status():
    """(SteamVR found, lhsyncd service state, driver registered), from the host."""
    steamvr = host("test", "-x", VRPATHREG).returncode == 0
    enabled = host("systemctl", "--user", "is-enabled", SERVICE).stdout.strip()
    active = host("systemctl", "--user", "is-active", SERVICE).stdout.strip()
    drivers = host(VRPATHREG, "show").stdout if steamvr else ""
    return steamvr, (enabled, active), re.search(r"^\s*questlhsync_frame\s*:", drivers, re.M) is not None


def github(url):
    req = urllib.request.Request(url, headers={"Accept": "application/vnd.github+json", "User-Agent": APP_ID})
    return urllib.request.urlopen(req, timeout=30)


def latest_asset():
    """(tag, asset): the release's QuestLHSync-frame tar.gz when attached on its own, else the release zip."""
    with github(LATEST) as r:
        release = json.load(r)
    assets = release.get("assets", [])
    for pattern in (FRAME_TAR, RELEASE_ZIP):
        for a in assets:
            if pattern.search(a["name"]):
                return release["tag_name"], a
    raise Failure(f"release {release.get('tag_name')} has no QuestLHSync zip")


def unpack(path):
    """Unpacks QuestLHSync-frame/ from a release zip or a QuestLHSync-frame tar.gz, returns its install.sh."""
    dest = os.path.join(WORK, "package")
    shutil.rmtree(dest, ignore_errors=True)
    os.makedirs(dest)
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as z:
            name = next((n for n in z.namelist() if FRAME_TAR.search(n)), None)
            if not name:
                raise Failure(f"{os.path.basename(path)} has no QuestLHSync-frame-<version>.tar.gz")
            with z.open(name) as f, tarfile.open(fileobj=f, mode="r|gz") as t:
                t.extractall(dest, filter="data")
    else:
        with tarfile.open(path, "r:gz") as t:
            t.extractall(dest, filter="data")
    script = os.path.join(dest, "QuestLHSync-frame", "install.sh")
    if not os.path.isfile(script):
        raise Failure(f"{os.path.basename(path)} has no QuestLHSync-frame/install.sh")
    return script


def row(title, subtitle=""):
    # plain text: subtitles carry file names and error messages
    r = Adw.ActionRow(title=title, use_markup=False)
    r.set_subtitle(subtitle)  # after use_markup: construct properties come in no set order
    return r


class Window(Adw.ApplicationWindow):
    def __init__(self, app):
        super().__init__(application=app, title="QuestLHSync", default_width=640, default_height=720)
        self.spinner = Gtk.Spinner()
        header = Adw.HeaderBar()
        header.pack_end(self.spinner)
        refresh = Gtk.Button(icon_name="view-refresh-symbolic", tooltip_text="Refresh status")
        refresh.connect("clicked", lambda _: self.refresh())
        header.pack_start(refresh)

        state = Adw.PreferencesGroup(title="On this headset")
        self.steamvr_row = row("SteamVR")
        self.service_row = row("lhsyncd service")
        self.driver_row = row("questlhsync_frame SteamVR driver")
        for r in (self.steamvr_row, self.service_row, self.driver_row):
            state.add(r)

        actions = Adw.PreferencesGroup(
            title="Install",
            description="Starts lhsyncd as a systemd user service and registers the questlhsync_frame driver with "
                        "the headset's SteamVR, then restarts SteamVR on the headset once to load it. Nothing needs root.")
        self.latest_row = row("Latest release", "Checking GitHub…")
        self.file_row = row("From a file", "QuestLHSync-<version>.zip or QuestLHSync-frame-<version>.tar.gz")
        uninstall_row = row("Uninstall", "Stops and removes lhsyncd, unregisters the driver")
        self.install_button = self.button(self.latest_row, "Install", "suggested-action", self.on_install_latest)
        self.file_button = self.button(self.file_row, "Choose…", None, self.on_install_file)
        self.uninstall_button = self.button(uninstall_row, "Uninstall", "destructive-action", self.on_uninstall)
        for r in (self.latest_row, self.file_row, uninstall_row):
            actions.add(r)

        self.log_view = Gtk.TextView(editable=False, cursor_visible=False, monospace=True,
                                     wrap_mode=Gtk.WrapMode.WORD_CHAR, top_margin=8, bottom_margin=8,
                                     left_margin=8, right_margin=8)
        log_scroll = Gtk.ScrolledWindow(child=self.log_view, min_content_height=200, vexpand=True)
        log_scroll.add_css_class("card")

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=24, margin_top=24, margin_bottom=24,
                      margin_start=12, margin_end=12)
        for w in (state, actions, log_scroll):
            box.append(w)
        body = Adw.ToolbarView(content=Gtk.ScrolledWindow(child=Adw.Clamp(child=box, maximum_size=720)))
        body.add_top_bar(header)
        self.set_content(body)

        self.asset = None
        self.busy = False
        self.refresh()
        threading.Thread(target=self.check_latest, daemon=True).start()

    @staticmethod
    def button(row, label, style, handler):
        b = Gtk.Button(label=label, valign=Gtk.Align.CENTER)
        if style:
            b.add_css_class(style)
        b.connect("clicked", lambda _: handler())
        row.add_suffix(b)
        return b

    # every UI change from a worker thread goes through here
    def ui(self, fn, *args):
        def once():
            fn(*args)
            return GLib.SOURCE_REMOVE
        GLib.idle_add(once)

    def log(self, text):
        def append():
            buf = self.log_view.get_buffer()
            buf.insert(buf.get_end_iter(), text if text.endswith("\n") else text + "\n")
            self.log_view.scroll_to_mark(buf.get_insert(), 0, False, 0, 0)
        self.ui(append)

    def set_busy(self, busy):
        self.busy = busy
        self.spinner.set_spinning(busy)
        self.file_button.set_sensitive(not busy)
        self.uninstall_button.set_sensitive(not busy)
        self.install_button.set_sensitive(not busy)

    def refresh(self):
        def work():
            steamvr, (enabled, active), registered = status()
            self.ui(self.show_status, steamvr, enabled, active, registered)
        threading.Thread(target=work, daemon=True).start()

    def show_status(self, steamvr, enabled, active, registered):
        self.steamvr_row.set_subtitle(f"Found in {STEAMVR}" if steamvr else f"Not found in {STEAMVR}: is this a Steam Frame?")
        if enabled in ("", "not-found"):
            self.service_row.set_subtitle("Not installed")
        else:
            self.service_row.set_subtitle(
                f"{active.capitalize()}, {'starts' if enabled == 'enabled' else 'does not start'} at login")
        self.driver_row.set_subtitle("Registered" if registered else "Not registered")

    def check_latest(self):
        try:
            tag, asset = latest_asset()
            self.asset = asset
            self.ui(self.latest_row.set_subtitle, f"{tag}: {asset['name']} ({asset['size'] / 1e6:.1f} MB)")
        except Exception as e:
            self.ui(self.latest_row.set_subtitle, f"Couldn't reach GitHub: {e}")

    def run(self, title, work):
        """work() on a thread, its output in the log; then the status again."""
        if self.busy:
            return
        self.set_busy(True)
        self.log(f"── {title}")

        def go():
            try:
                work()
            except Exception as e:
                self.log(f"failed: {e}")
            finally:
                self.ui(self.set_busy, False)
                self.refresh()
        threading.Thread(target=go, daemon=True).start()

    def stream(self, *argv):
        p = subprocess.Popen([*HOST, *argv], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in p.stdout:
            self.log(line)
        if p.wait():
            raise Failure(f"exit status {p.returncode}")

    def install(self, path):
        script = unpack(path)
        self.log(f"unpacked {os.path.dirname(script)}")
        try:
            self.stream("sh", script)
        finally:
            # install.sh copied what it needs to ~/.local/share/questlhsync
            shutil.rmtree(os.path.join(WORK, "package"), ignore_errors=True)

    def on_install_latest(self):
        def work():
            if not self.asset:
                self.check_latest()
            if not self.asset:
                raise Failure("no release to download")
            os.makedirs(WORK, exist_ok=True)
            path = os.path.join(WORK, self.asset["name"])
            self.log(f"downloading {self.asset['browser_download_url']}")
            with github(self.asset["browser_download_url"]) as r, open(path + ".part", "wb") as f:
                shutil.copyfileobj(r, f, 1 << 20)
            os.replace(path + ".part", path)
            try:
                self.install(path)
            finally:
                os.remove(path)
        self.run("Install the latest release", work)

    def on_install_file(self):
        archives = Gtk.FileFilter(name="QuestLHSync release")
        for pattern in ("*.zip", "*.tar.gz"):
            archives.add_pattern(pattern)
        filters = Gio.ListStore.new(Gtk.FileFilter)
        filters.append(archives)
        dialog = Gtk.FileDialog(title="QuestLHSync release", filters=filters, default_filter=archives)

        def chosen(d, result):
            try:
                path = d.open_finish(result).get_path()
            except GLib.Error:
                return  # cancelled
            self.run(f"Install {os.path.basename(path)}", lambda: self.install(path))
        dialog.open(self, None, chosen)

    def on_uninstall(self):
        dialog = Adw.AlertDialog(heading="Uninstall QuestLHSync?",
                                 body="lhsyncd stops and is removed. The driver stays loaded until SteamVR on the "
                                      "headset restarts.")
        dialog.add_response("cancel", "Cancel")
        dialog.add_response("uninstall", "Uninstall")
        dialog.set_response_appearance("uninstall", Adw.ResponseAppearance.DESTRUCTIVE)
        dialog.set_close_response("cancel")

        def answered(d, result):
            if d.choose_finish(result) == "uninstall":
                script = open(UNINSTALL).read()
                self.run("Uninstall", lambda: self.stream("sh", "-c", script))
        dialog.choose(self, None, answered)


def main():
    app = Adw.Application(application_id=APP_ID)
    # SteamOS is dark (Vapor), but its portal answers color-scheme with the wrong type, so libadwaita reads no
    # preference and would go light. Dark unless the desktop asks for light.
    app.connect("startup", lambda a: a.get_style_manager().set_color_scheme(Adw.ColorScheme.PREFER_DARK))
    app.connect("activate", lambda a: (a.get_active_window() or Window(a)).present())
    app.run()


if __name__ == "__main__":
    main()

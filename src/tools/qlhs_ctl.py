#!/usr/bin/env python3
"""QuestLHSync from a terminal, for Linux where the dashboard isn't ported yet: the driver's status, and its commands.

  qlhs_ctl.py                 status once
  qlhs_ctl.py watch           status every second
  qlhs_ctl.py pause|resume|record-on|record-off|recalibrate

Reads and writes the driver's shared memory (/dev/shm/QuestLHSync, src/common/qlhs_status.h).
"""
import ctypes as C
import mmap
import os
import sys
import time

SHM = "/dev/shm/QuestLHSync"
MAGIC, VERSION = 0x53484C51, 2
STATES = ["starting", "no Quest/Frame HMD in SteamVR", "searching for the headset", "connecting",
          "connected, no camera frames", "fewer than 2 base stations", "acquiring", "LOCKED", "disabled",
          "wired headset: off"]
CMDS = {"pause": 2, "resume": 3, "record-on": 4, "record-off": 5, "recalibrate": 6}


class Station(C.Structure):
    _fields_ = [("serial", C.c_char * 32), ("support", C.c_int32), ("anchor", C.c_int32), ("measured", C.c_int32),
                ("pad", C.c_int32), ("last_seen", C.c_double), ("dist", C.c_double)]


class Status(C.Structure):
    _fields_ = [("magic", C.c_uint32), ("version", C.c_uint32), ("seq", C.c_int32), ("state", C.c_int32),
                ("updated", C.c_double), ("hmd", C.c_char * 64), ("hmd_system", C.c_char * 32),
                ("headset", C.c_char * 48), ("headset_addr", C.c_char * 48), ("headset_fw", C.c_char * 48),
                ("cam_fps", C.c_double), ("sight_rate", C.c_double), ("spot_rate", C.c_double),
                ("head_still", C.c_int32), ("pad2", C.c_int32), ("rtt_ms", C.c_double), ("expo_ms", C.c_double),
                ("expo_learned", C.c_int32), ("pad0", C.c_int32), ("nst", C.c_int32), ("pad1", C.c_int32),
                ("st", Station * 8), ("locked", C.c_int32), ("cond", C.c_int32), ("yaw_deg", C.c_double),
                ("t", C.c_double * 3), ("med_deg", C.c_double), ("nfit", C.c_int32), ("paused", C.c_int32),
                ("locked_for", C.c_double), ("lag_cm", C.c_double), ("recording", C.c_int32), ("nlog", C.c_uint32),
                ("log", (C.c_char * 120) * 8), ("cmd_seq", C.c_int32), ("cmd", C.c_int32)]


def open_shm():
    try:
        fd = os.open(SHM, os.O_RDWR)
    except FileNotFoundError:
        sys.exit("no QuestLHSync driver running (no %s): is SteamVR up?" % SHM)
    m = mmap.mmap(fd, C.sizeof(Status))
    os.close(fd)
    st = Status.from_buffer(m)
    if st.magic != MAGIC or st.version != VERSION:
        sys.exit("unexpected shared memory (magic %08x version %d)" % (st.magic, st.version))
    return m, st


def snapshot(st):
    for _ in range(100):  # the driver's seqlock: odd while it writes
        s = st.seq
        if s & 1:
            time.sleep(0.001)
            continue
        copy = Status.from_buffer_copy(st)
        if st.seq == s:
            return copy
    return Status.from_buffer_copy(st)


def show(s):
    d = lambda b: b.decode(errors="replace")
    print("state     %s%s%s" % (STATES[s.state] if 0 <= s.state < len(STATES) else s.state,
                               "  (paused)" if s.paused else "", "  (recording)" if s.recording else ""))
    print("SteamVR   %s via %s" % (d(s.hmd) or "-", d(s.hmd_system) or "-"))
    print("headset   %s %s  fw %s  rtt %.1f ms" % (d(s.headset) or "-", d(s.headset_addr) or "-", d(s.headset_fw) or "-",
                                                  s.rtt_ms))
    print("cameras   %.0f fps, spots %.0f/s, used %.0f/s%s" % (s.cam_fps, s.spot_rate, s.sight_rate,
                                                              "  head still" if s.head_still else ""))
    print("fit       yaw %+.2f deg  t [%.3f %.3f %.3f]  median %.2f deg  n %d  %s" % (
        s.yaw_deg, s.t[0], s.t[1], s.t[2], s.med_deg, s.nfit,
        ("locked %.0f s" % s.locked_for) if s.locked else "not locked"))
    for i in range(min(s.nst, 8)):
        e = s.st[i]
        seen = "never" if e.last_seen < 0 else "%.1f s ago" % e.last_seen
        print("  %-14s support %3d  %4.2f m  seen %s%s%s" % (d(e.serial), e.support, e.dist, seen,
                                                          "  anchor" if e.anchor else "",
                                                          "  measured" if e.measured else ""))


def main():
    m, st = open_shm()
    arg = sys.argv[1] if len(sys.argv) > 1 else "status"
    if arg in CMDS:
        st.cmd = CMDS[arg]
        st.cmd_seq = st.cmd_seq + 1  # after cmd: the driver reads cmd once it sees a new cmd_seq
        print("sent", arg)
    elif arg == "watch":
        try:
            while True:
                print("\033[2J\033[H", end="")
                show(snapshot(st))
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    elif arg == "status":
        show(snapshot(st))
    else:
        sys.exit(__doc__)
    del st
    m.close()


if __name__ == "__main__":
    main()

"""A stand-in for the headset's lhsyncd: replays a recorded log's camera lines in real time over the same protocol
(UDP discovery, TCP hello + calibration + lines, clock round trips). Its "headset clock" follows the log's own
headset<->PC mapping (arrival envelope, like qlhs_replay's), so a client's round-trip clock lands where the
old pipeline's did and the learned pose timing should come out near EXPO (0.020 s).

  python fake_lhsyncd.py <log> <calibration json> [start_s] [duration_s]
"""
import os
import socket
import sys
import threading
import time

import numpy as np

log, calib = sys.argv[1], sys.argv[2]
start = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
dur = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9

F = []  # (pc_log, line)
env = {}
for ln in open(log, encoding="utf-8", errors="replace"):
    s = ln.split()
    if len(s) >= 7 and s[1] == "F":
        pc, hs = int(s[0]) / 1e9, int(s[4]) / 1e6
        F.append((pc, " ".join(s[1:])))
        k = int(hs // 10)
        if k not in env or pc - hs < env[k][1]:
            env[k] = (hs, pc - hs)
x = np.array([v[0] for v in env.values()])
y = np.array([v[1] for v in env.values()])
a, b = np.polyfit(x - x.mean(), y, 1)
x0 = x.mean()
pc0 = F[0][0] + start
F = [f for f in F if pc0 <= f[0] < pc0 + dur]
print(f"{len(F)} frame lines from {start:.0f} s, clock drift {a * 1e6:.1f} ppm")


def hs_of_pc(pc):  # inverse of pc = hs + a (hs - x0) + b
    return (pc - b + a * x0) / (1 + a)


t_start = None
cal = open(calib, "rb").read()


def udp():
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.bind(("0.0.0.0", 47281))
    while True:
        d, addr = u.recvfrom(64)
        if d.startswith(b"QLHS?"):
            u.sendto(b"QLHS 1 FAKE0001 47280 Quest_Pro idle", addr)


if not os.environ.get("QLHS_NO_UDP"):  # QLHS_NO_UDP=1: a network that drops broadcasts (tests the TCP fallback)
    threading.Thread(target=udp, daemon=True).start()
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", 47280))
srv.listen(1)
c, addr = srv.accept()
c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
print("client", addr)
t_start = time.perf_counter()
lock = threading.Lock()


def now_hs():
    return hs_of_pc(pc0 + time.perf_counter() - t_start)


def send(b):
    with lock:
        c.sendall(b)


send(b"H QuestLHSync 1 serial=FAKE0001 model=Quest_Pro fw=replay module=fake\n")
send(b"C %d online/fake\n" % len(cal) + cal)
send(b"I fake qpc0 %.9f\n" % t_start)   # for nettest: where the log's time 0 sits on this PC's clock


def pings():
    buf = b""
    while True:
        d = c.recv(4096)
        if not d:
            return
        buf += d
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            p = ln.split()
            if len(p) == 3 and p[0] == b"P":
                send(b"Q %s %s %d\n" % (p[1], p[2], int(now_hs() * 1e9)))


threading.Thread(target=pings, daemon=True).start()
for pc, line in F:
    wait = (pc - pc0) - (time.perf_counter() - t_start)
    if wait > 0:
        time.sleep(wait)
    try:
        send(line.encode() + b"\n")
    except OSError:
        break
print("done")
c.close()

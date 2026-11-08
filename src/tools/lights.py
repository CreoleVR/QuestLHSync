"""Where the bright dots in a recording are, and which base station's rotor each one follows: the stations, their
reflections, and the lights that follow none (what Solver::SurveyLights finds, checked by hand).

  qlhs_replay <recording> --calib <calibration json> --dir <scratch dir> --pings --rays rays.txt
  python -I lights.py rays.txt <recording> [last_seconds]

Positions are in the headset's space (y up, metres). Each dot place is where its sightings cross (needs head movement:
a still head gives no depth); "rotor" is the station whose rotor period its seeing follows, by how much over fake
periods next to it (stations score 0.01-0.2 here, lights 0.00-0.004; a station mostly hidden can sit at 0.005).
The channels come from the recording's S lines.
"""
import collections
import sys

import numpy as np

ROTOR_TICKS = [959000, 957000, 953000, 949000, 947000, 943000, 941000, 939000,
               937000, 929000, 919000, 911000, 907000, 901000, 893000, 887000]  # sync.cpp kRotorTicks, 48 MHz
EPS = [0.0015, 0.0025, 0.0035, 0.0045, 0.0055, 0.0065]


def channels(recording):
    ch = {}
    for line in open(recording, errors="replace"):
        f = line.split()
        if len(f) >= 11 and f[1] == "S":
            ch[f[2]] = int(f[10])
    return ch


def rotor_score(times, period, window=10.0):
    def r(p):
        t = np.sort(times)
        tot, n, i = 0.0, 0, 0
        while i < len(t):
            j = np.searchsorted(t, t[i] + window)
            w = t[i:j]
            if len(w) >= 8:
                tot += abs(np.exp(2j * np.pi * (w - w[0]) / p).sum())
                n += len(w)
            i = j
        return tot / max(n, 1)
    return r(period) - np.mean([r(period * (1 + e)) for e in EPS] + [r(period * (1 - e)) for e in EPS])


def main():
    rays, recording = sys.argv[1], sys.argv[2]
    tail = float(sys.argv[3]) if len(sys.argv) > 3 else 0
    rows = []
    for line in open(rays):
        if line[0] != "R":
            continue
        f = line.split()
        if int(f[13]) >= 250 and float(f[12]) <= 60:  # bright, head turning under 60 deg/s
            rows.append([float(x) for x in f[3:12]] + [float(f[1])])
    R = np.array(rows)
    if tail:
        R = R[R[:, 9] >= R[:, 9].max() - tail]
    O, D, H, T = R[:, 0:3], R[:, 3:6], R[:, 6:9], R[:, 9]
    print(f"{len(R)} bright sightings over {T.max() - T.min():.0f} s")
    cell, votes, heads = 0.1, collections.Counter(), collections.defaultdict(set)
    steps = np.arange(0.6, 7.0, 0.05)
    for i in range(len(R)):
        hc = tuple(np.floor(H[i] / 0.25).astype(int))
        for k in set(map(tuple, np.floor((O[i] + np.outer(steps, D[i])) / cell).astype(int))):
            votes[k] += 1
            heads[k].add(hc)
    cand = sorted(((v * min(len(heads[k]), 10) / 10, k) for k, v in votes.items() if len(heads[k]) >= 4), reverse=True)

    def refine(c):
        m = None
        for _ in range(6):
            v = c - O
            pr = (v * D).sum(1)
            m = (pr > 0.5) & (np.degrees(np.arctan2(np.linalg.norm(v - pr[:, None] * D, axis=1), pr)) < 0.7)
            if m.sum() < 15:
                break
            A, b = np.zeros((3, 3)), np.zeros(3)
            for o, d in zip(O[m], D[m]):
                M = np.eye(3) - np.outer(d, d)
                A += M
                b += M @ o
            c = np.linalg.solve(A, b)
        return c, m

    ch = channels(recording)
    periods = {f"ch{c} {s}": ROTOR_TICKS[c - 1] / 48e6 for s, c in sorted(ch.items(), key=lambda kv: kv[1])
               if 1 <= c <= 16}
    places = []
    for _, k in cand:
        c, m = refine((np.array(k) + 0.5) * cell)
        if m is not None and m.sum() >= 15 and all(np.linalg.norm(c - p) > 0.3 for p, _ in places):
            places.append((c, m))
        if len(places) >= 10:
            break
    for c, m in places:
        t = np.unique(np.round(T[m], 4))
        hc = len(set(map(tuple, np.floor(H[m] / 0.25).astype(int))))
        sc = {k: rotor_score(t, p) for k, p in periods.items()}
        best = max(sc, key=sc.get) if sc else None
        verdict = best if best and sc[best] > 0.02 else "no rotor (a light?)" if sc and max(sc.values()) < 0.004 else "unclear"
        print(f"[{c[0]:6.2f} {c[1]:5.2f} {c[2]:6.2f}] {m.sum():5d} sightings from {hc:3d} head cells | "
              + "  ".join(f"{k.split()[0]} {v:+.3f}" for k, v in sc.items()) + f"  -> {verdict}")


if __name__ == "__main__":
    main()

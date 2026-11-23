"""How much of a replay the alignment spent on a given fit: qlhs_replay's --out file (a line a second: time, yaw deg,
tx, ty, tz) against the yaw of the right fit, and optionally of known wrong ones.

  python -I fitshare.py replay.out <right yaw deg> [wrong yaw deg ...]
  e.g. python -I fitshare.py r.out -73 -59 +107     (the 3-station room: right, on the lamp, its 180 deg twin)
"""
import sys

import numpy as np


def main():
    a = np.array([list(map(float, line.split())) for line in open(sys.argv[1]) if len(line.split()) == 5])
    t, yaw = (a[:, 0] - a[0, 0]) / 1e9, a[:, 1]
    right = float(sys.argv[2])
    near = lambda y0, tol: np.abs((yaw - y0 + 180) % 360 - 180) < tol
    ok = near(right, 3)
    print(f"{t[-1]:.0f} s: right fit ({right:+.1f} deg) {ok.mean() * 100:.1f}%, yaw spread on it {yaw[ok].std():.3f} deg")
    for w in sys.argv[3:]:
        print(f"  {float(w):+.1f} deg: {near(float(w), 3).mean() * 100:.1f}%")


if __name__ == "__main__":
    main()

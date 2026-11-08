"""Saves a headset's camera calibration, as lhsyncd sends it after its hello: what qlhs_replay's --calib needs.

  python -I getcalib.py <headset ip> <out.json>

It connects like the driver does, so the headset's camera reader runs for the moment the connection is open.
"""
import socket
import sys


def main():
    ip, out = sys.argv[1], sys.argv[2]
    s = socket.create_connection((ip, 47280), timeout=10)
    buf = b""
    while True:
        k = buf.find(b"\nC ")
        if k >= 0 and buf.find(b"\n", k + 1) >= 0:
            j = buf.index(b"\n", k + 1)
            n = int(buf[k + 1:j].split()[1])
            if len(buf) >= j + 1 + n:
                break
        if buf.startswith(b"H ") and b"\nE " in buf:
            sys.exit("the headset has no calibration to give: " + buf[buf.index(b"\nE ") + 3:].split(b"\n")[0].decode())
        d = s.recv(65536)
        if not d:
            sys.exit("the headset closed the connection before its calibration")
        buf += d
    s.close()
    with open(out, "wb") as f:
        f.write(buf[j + 1:j + 1 + n])
    hello = buf.split(b"\n")[0].decode()
    print(f"{hello}\nsaved {n} bytes ({buf[k + 1:j].decode()}) to {out}")


if __name__ == "__main__":
    main()

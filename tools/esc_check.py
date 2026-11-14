#!/usr/bin/env python3
"""Reset the Mega, then pulse each motor via ESC_TEST and print the commanded PWM
seen in telemetry. Usage: python3 tools/esc_check.py [us]   (props OFF!)"""
import glob, os, select, sys, termios, time

us = int(sys.argv[1]) if len(sys.argv) > 1 else 1150
port = sorted(glob.glob("/dev/ttyUSB*"))[0]
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[0] = a[1] = a[3] = 0
a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.B115200
a[4] = a[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, a)


def rd(sec):
    end, buf = time.time() + sec, b""
    while time.time() < end:
        if select.select([fd], [], [], 0.05)[0]:
            buf += os.read(fd, 4096)
    return buf.decode(errors="replace")


rd(2.5)
os.write(fd, b"TEL:1\n")
rd(0.3)
for m in range(4):
    os.write(fd, f"ESC_TEST:{m}:{us}\n".encode())
    out = rd(1.2)
    t = [l for l in out.splitlines() if l.startswith("T,") and len(l.split(",")) == 17]
    mx = [max(int(l.split(",")[13 + k]) for l in t) for k in range(4)] if t else None
    last = mx  # peak commanded us per motor during the 1.2 s window
    print(f"M{m}: ack={'ESC TEST' in out} motors_in_telemetry={last}")
    rd(1.5)
os.write(fd, b"TEL:0\n")
os.close(fd)

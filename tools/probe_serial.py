#!/usr/bin/env python3
"""
Probes the Arduino Mega 2560 + ESP8266 on /dev/ttyUSB0 using standard library termios.
Sends STATUS, SCAN, and ESP:AT to check both USB and onboard ESP8266 Serial3.
"""
import os
import time
import select
import termios

import glob
ports = sorted(glob.glob("/dev/ttyUSB*"))
PORT = ports[0] if ports else "/dev/ttyUSB0"
print(f"Using port: {PORT}")
fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attrs = termios.tcgetattr(fd)
attrs[0] = 0
attrs[1] = 0
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.B115200
attrs[3] = 0
attrs[4] = termios.B115200
attrs[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, attrs)


def read_for(seconds):
    end = time.time() + seconds
    buf = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            chunk = os.read(fd, 4096)
            if chunk:
                buf += chunk
    return buf.decode("utf-8", errors="replace")


print("--- Boot output (2.5s) ---")
print(read_for(2.5), end="")

for cmd in ["SCAN", "STATUS", "IMU_TEST:1"]:
    os.write(fd, (cmd + "\n").encode())
    print(f"--- Sent {cmd} ---")
    print(read_for(1.2), end="")
os.write(fd, b"IMU_TEST:0\n")
read_for(0.2)

os.close(fd)

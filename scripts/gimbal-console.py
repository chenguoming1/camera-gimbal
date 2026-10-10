#!/usr/bin/env python3
"""USB CDC line console using Python's standard library (macOS/Linux).
Use the controller's USB port, not ST-LINK's virtual serial port.
"""
import argparse
import errno
import os
import select
import sys
import termios

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('port', help='Controller CDC device, e.g. /dev/cu.usbmodemXXXX')
args = parser.parse_args()
fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
old = termios.tcgetattr(fd)
try:
    config = termios.tcgetattr(fd)
    config[0] = config[1] = config[3] = 0
    config[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    config[4] = config[5] = termios.B115200
    config[6][termios.VMIN] = 0
    config[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, config)
    pending = bytearray(b'status\n')
    print('Connected. Commands end with Enter. Ctrl-C sends emergency stop; Ctrl-D exits.')
    print('Keep 3S disconnected for orientation checks. Startup does not arm motors.')
    while True:
        try:
            readable, writable, _ = select.select([fd, sys.stdin], [fd] if pending else [], [], 1)
        except KeyboardInterrupt:
            # Drop queued commands before sending Ctrl-C to the controller.
            pending[:] = b'\x03\n'
            continue
        if fd in writable:
            try:
                sent = os.write(fd, pending)
                del pending[:sent]
            except BlockingIOError:
                pass
        if fd in readable:
            data = os.read(fd, 4096)
            if not data:
                raise OSError(errno.ENODEV, 'Controller disconnected')
            sys.stdout.write(data.decode('ascii', errors='replace'))
            sys.stdout.flush()
        if sys.stdin in readable:
            command = sys.stdin.readline()
            if not command:
                # A normal exit also requests stop; check controller telemetry.
                os.write(fd, b'\x03\n')
                break
            pending.extend(command.encode('ascii', errors='strict'))
finally:
    try:
        termios.tcsetattr(fd, termios.TCSANOW, old)
    finally:
        os.close(fd)

#!/usr/bin/env -S python3 -u
"""Send a sequence of lines to the Arduino over serial and capture responses.

Useful for verifying interactive chapters (shell, interpreter, TinyPython).

Example:
    uv run python scripts/probe_serial.py --send "print(1+2)" --send "help"
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports


def find_arduino_port() -> str | None:
    try:
        for port in serial.tools.list_ports.comports():
            if port.device.startswith("/dev/ttyACM"):
                return port.device
    except (TypeError, OSError):
        pass
    candidates = sorted(Path("/dev").glob("ttyACM*"))
    return str(candidates[0]) if candidates else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--send",
        action="append",
        default=[],
        help="Line to send (appends CR+LF). Repeat for multiple lines.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.4,
        help="Seconds between sent lines",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=4.0,
        help="Read window after last send (seconds)",
    )
    parser.add_argument(
        "--reset-dtr",
        action="store_true",
        help="Toggle DTR after opening to reset the board (most Arduinos do this)",
    )
    args = parser.parse_args()

    port = args.port or find_arduino_port()
    if port is None:
        print("No serial port found", file=sys.stderr)
        return 1

    print(f"[probe] opening {port} @ {args.baud}", flush=True)
    with serial.Serial(port, args.baud, timeout=0.2) as ser:
        if args.reset_dtr:
            ser.setDTR(False)
            time.sleep(0.2)
            ser.setDTR(True)
            time.sleep(2.0)
        # Drain any initial output briefly.
        time.sleep(0.5)
        ser.reset_input_buffer()

        for line in args.send:
            print(f"[probe] >>> {line!r}", flush=True)
            ser.write((line + "\r\n").encode("utf-8"))
            ser.flush()
            time.sleep(args.delay)

        deadline = time.time() + args.timeout
        while time.time() < deadline:
            chunk = ser.read(512)
            if chunk:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env -S python3 -u
"""Flash a chapter to Arduino UNO R4 WiFi and capture serial output.

This script:
1. Power-cycles the USB hub port via uhubctl (optional, robust against bricked sketches).
2. Uploads the chosen chapter with PlatformIO via uv.
3. Re-detects the serial device.
4. Sends a trigger byte (default 'S') and captures output for a fixed window.

Usage:
    uv run python scripts/flash_and_capture.py 01_boot
    uv run python scripts/flash_and_capture.py 02_baremetal --no-trigger --timeout 5
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports


USB_HUB_LOCATION = "1-13.3"
USB_HUB_PORT = "1"
ARDUINO_VID_PID = "2341:1002"
DEFAULT_BAUD = 115200


def log(msg: str) -> None:
    print(f"[flash] {msg}", flush=True)


def power_cycle_hub() -> None:
    """Power-cycle the configured USB hub port and wait for re-enumeration."""
    log(f"Power-cycling USB hub {USB_HUB_LOCATION} port {USB_HUB_PORT}...")
    subprocess.run(
        [
            "uhubctl",
            "-l",
            USB_HUB_LOCATION,
            "-p",
            USB_HUB_PORT,
            "-a",
            "cycle",
            "-d",
            "2",
        ],
        check=False,
    )
    for _ in range(40):
        if find_arduino_port() is not None:
            log("Arduino re-enumerated.")
            return
        time.sleep(0.5)
    log("WARNING: Arduino did not re-enumerate after power cycle.")


def find_arduino_port() -> str | None:
    """Find /dev/ttyACM* for an Arduino UNO R4 WiFi."""
    try:
        for port in serial.tools.list_ports.comports():
            if port.vid is not None and port.pid is not None:
                vid_pid = f"{port.vid:04x}:{port.pid:04x}"
                if vid_pid == ARDUINO_VID_PID:
                    return port.device
            if port.device.startswith("/dev/ttyACM"):
                return port.device
    except (TypeError, OSError):
        pass
    candidates = sorted(Path("/dev").glob("ttyACM*"))
    if candidates:
        return str(candidates[0])
    return None


def upload(chapter_dir: Path) -> None:
    """Upload a PlatformIO project using uv."""
    log(f"Uploading {chapter_dir.name}...")
    result = subprocess.run(
        ["uv", "run", "pio", "run", "-d", str(chapter_dir), "-t", "upload"],
        cwd=Path(__file__).resolve().parent.parent,
    )
    if result.returncode != 0:
        raise SystemExit(f"Upload failed for {chapter_dir.name}")


def capture(
    port: str,
    baud: int,
    trigger: bytes | None,
    timeout: float,
    settle: float,
) -> str:
    """Open the serial port, optionally send a trigger byte, and capture output."""
    log(f"Opening {port} @ {baud} baud (timeout={timeout}s)...")
    time.sleep(settle)
    chunks: list[bytes] = []
    start = time.time()
    deadline = start + timeout
    with serial.Serial(port, baud, timeout=0.2) as ser:
        if trigger is not None:
            time.sleep(0.5)
            ser.write(trigger)
            ser.flush()
            log(f"Sent trigger: {trigger!r}")
        while time.time() < deadline:
            chunk = ser.read(512)
            if chunk:
                chunks.append(chunk)
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
    return b"".join(chunks).decode("utf-8", errors="replace")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("chapter", help="Chapter directory name, e.g. 01_boot")
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD, help="Serial baudrate"
    )
    parser.add_argument(
        "--trigger",
        default="S",
        help="Single byte to send after open ('' to skip)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Total capture window in seconds",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=2.0,
        help="Seconds to wait after upload before opening serial",
    )
    parser.add_argument(
        "--no-power-cycle",
        action="store_true",
        help="Skip uhubctl power cycle before upload",
    )
    args = parser.parse_args()

    base = Path(__file__).resolve().parent.parent
    chapter_dir = base / args.chapter
    if not chapter_dir.is_dir():
        log(f"ERROR: Chapter dir not found: {chapter_dir}")
        return 1

    if not args.no_power_cycle:
        power_cycle_hub()

    upload(chapter_dir)

    time.sleep(args.settle)
    port = find_arduino_port()
    if port is None:
        log("No Arduino port found after upload.")
        return 2

    trigger_bytes = args.trigger.encode() if args.trigger else None
    print("===== SERIAL OUTPUT START =====", flush=True)
    capture(port, args.baud, trigger_bytes, args.timeout, args.settle)
    print("\n===== SERIAL OUTPUT END =====", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env -S python3 -u
"""Build, flash, and capture serial output for every chapter.

Per-chapter results land in scripts/results/<chapter>.log. A summary
table is printed at the end. The script keeps going on build failures
so we get a complete report rather than stopping at the first error.
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass
class ChapterConfig:
    name: str
    has_pio: bool = True
    trigger: str = ""  # bytes to send after serial opens
    timeout: float = 8.0
    settle: float = 2.5
    note: str = ""


CHAPTERS: list[ChapterConfig] = [
    ChapterConfig("00_intro", has_pio=False, note="Arduino IDE sketch (ut.ino), no PIO project"),
    ChapterConfig("01_boot", trigger="S", timeout=6.0),
    ChapterConfig("02_baremetal", timeout=8.0),
    ChapterConfig("03_context_switch", timeout=6.0),
    ChapterConfig("04_scheduler", timeout=8.0),
    ChapterConfig("05_shell", timeout=6.0, trigger="\n"),
    ChapterConfig("06_memory_protection", timeout=6.0),
    ChapterConfig("07_led_matrix", timeout=6.0),
    ChapterConfig("08_interpreter", timeout=6.0),
    ChapterConfig("09_integration", timeout=6.0),
    ChapterConfig("10_freertos", timeout=8.0),
    ChapterConfig("11_tiny_python", timeout=6.0),
    ChapterConfig("12_hardware_deep_dive", has_pio=False, note="no sample code shipped"),
]

CODE_ROOT = Path(__file__).resolve().parent.parent
RESULTS_DIR = Path(__file__).resolve().parent / "results"


def run_flash(chapter: ChapterConfig) -> tuple[int, str]:
    """Run flash_and_capture.py for the chapter; return (rc, log_path)."""
    log_path = RESULTS_DIR / f"{chapter.name}.log"
    args = [
        "uv",
        "run",
        "python",
        "-u",
        "scripts/flash_and_capture.py",
        chapter.name,
        "--trigger",
        chapter.trigger,
        "--timeout",
        str(chapter.timeout),
        "--settle",
        str(chapter.settle),
    ]
    print(f"\n{'=' * 70}", flush=True)
    print(f"[runner] Flashing {chapter.name}: {shlex.join(args)}", flush=True)
    print(f"{'=' * 70}", flush=True)
    with log_path.open("w", encoding="utf-8") as fp:
        proc = subprocess.run(
            args,
            cwd=CODE_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        fp.write(proc.stdout or "")
    # Show last 30 lines so a human can spot regressions while running.
    tail = "\n".join((proc.stdout or "").splitlines()[-30:])
    print(tail, flush=True)
    return proc.returncode, str(log_path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--only",
        nargs="+",
        help="Only run these chapters (by directory name)",
    )
    parser.add_argument(
        "--skip",
        nargs="+",
        default=[],
        help="Skip these chapters (by directory name)",
    )
    args = parser.parse_args()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    summary: list[tuple[str, str]] = []

    for ch in CHAPTERS:
        if args.only and ch.name not in args.only:
            continue
        if ch.name in args.skip:
            continue
        if not ch.has_pio:
            print(f"\n[runner] SKIP {ch.name} - {ch.note}", flush=True)
            summary.append((ch.name, f"SKIP ({ch.note})"))
            continue
        start = time.time()
        rc, log = run_flash(ch)
        elapsed = time.time() - start
        status = "OK" if rc == 0 else f"FAIL rc={rc}"
        summary.append((ch.name, f"{status}  {elapsed:.1f}s  ({log})"))

    print("\n" + "=" * 70, flush=True)
    print("SUMMARY", flush=True)
    print("=" * 70, flush=True)
    for name, status in summary:
        print(f"  {name:30s}  {status}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

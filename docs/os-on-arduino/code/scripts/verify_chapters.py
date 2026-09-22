#!/usr/bin/env python3
"""各章のサンプルコードをビルド・書き込みし、シリアル出力が本文どおりかを検証する.

Linux / macOS / Windows のいずれでも同じコマンドで動く。ボードが無い環境では
``--build-only`` を付けるとビルドだけを検証するので、CI にもそのまま載る。

使い方
------
    uv run python scripts/verify_chapters.py --list
    uv run python scripts/verify_chapters.py --build-only
    uv run python scripts/verify_chapters.py --only 01_boot 02_baremetal
    uv run python scripts/verify_chapters.py                 # 全章を実機で検証

終了コードは、1 章でも失敗したら 1 になる。
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover
    serial = None

# Windows のコンソールが CP932 のとき、表組みの記号で落ちないようにする。
try:
    sys.stdout.reconfigure(errors="replace")
    sys.stderr.reconfigure(errors="replace")
except Exception:  # pragma: no cover
    pass

CODE_ROOT = Path(__file__).resolve().parent.parent
RESULTS_DIR = Path(__file__).resolve().parent / "results"
BAUD = 115200

# UNO R4 WiFi。スケッチ動作中とブートローダとで PID が変わる。
ARDUINO_VIDS = {0x2341, 0x2A03}


@dataclass
class Check:
    """1 つの期待値。``pattern`` は捕捉したログ全体に対する正規表現。"""

    pattern: str
    why: str = ""
    absent: bool = False  # True なら「出てはいけない」
    # この期待値を確かめられないエミュレータ（"renode" / "qemu"）。実機では常に確かめる
    unsupported_on: tuple[str, ...] = ()
    sim_note: str = ""    # なぜ確かめないか（エミュレータの制約）


@dataclass
class Chapter:
    name: str
    checks: list[Check] = field(default_factory=list)
    handshake: str = ""   # 出力が始まるまで送り続ける 1 文字（setup() でしか出さない章）
    send: list[str] = field(default_factory=list)  # 接続後に送るコマンド行
    capture: float = 8.0  # 捕捉する秒数
    settle: float = 2.5   # 書き込み後、ポートが戻るまで待つ秒数
    env: str = ""         # platformio.ini に env が複数ある章はここで 1 つ選ぶ
    custom: str = ""      # 追加検査の名前（下の CUSTOM_CHECKS を参照）
    note: str = ""
    title: str = ""       # 本文の章タイトル（対話モードの見出し用）
    watch: list[str] = field(default_factory=list)   # 目で見て確かめるポイント
    try_cmds: list[str] = field(default_factory=list)  # 対話モードで試すとよいコマンド


# 期待値は本文の「期待される出力」「出力例」と付属コードの Serial.print から起こしている。
# 環境で変わる値（アドレス・時刻・ADC 値）は正規表現で幅を持たせる。
CHAPTERS: list[Chapter] = [
    Chapter(
        "00_intro",
        title="準備編 第1章 Arduino UNO R4 WiFi で作る自作OS",
        watch=[
            "LED マトリクスに「OS」の 2 文字が出る",
        ],
        checks=[
            Check(r"Loop count: \d+", "loop() が回っている"),
        ],
        capture=8.0,
    ),
    Chapter(
        "01_boot",
        title="本編 第1章 ブートシーケンス",
        watch=[
            "LED は光らない。シリアル出力だけの章",
        ],
        handshake="S",
        checks=[
            Check(r"=== Boot Sequence Check ==="),
            Check(r"bss_var \(should be 0\): 0", "BSS がゼロクリアされている"),
            Check(r"data_var \(should be 12345\): 12345", "DATA が Flash からコピーされている"),
            Check(r"VTOR:\s+0x2000[0-9A-Fa-f]{4}", "ベクタテーブルが RAM 上にある"),
            Check(r"If you see this, boot sequence completed!"),
        ],
        capture=6.0,
    ),
    Chapter(
        "01_boot_vector_dump",
        title="本編 第1章 演習1-2 ベクタテーブルの読み取り",
        watch=[
            "LED は光らない",
        ],
        handshake="S",
        checks=[
            Check(r"=== Vector Table Dump ==="),
            Check(r"\[ 0\] 0x20007F00\s+Initial SP", "本文 1.9 の 0x20007F00"),
            Check(r"\[ 8\] 0x00000000", "空きベクタが 8 桁ゼロ詰めで出る（本文の 0x0000XXXX 形式）"),
            Check(r"\[15\] 0x0000[0-9A-Fa-f]{4}\s+SysTick"),
        ],
        capture=6.0,
    ),
    Chapter(
        "02_baremetal",
        title="本編 第2章 ベアメタルからの出発",
        watch=[
            "基板の L LED が 500ms ごとに点滅する（LED タスク）",
            "協調的スケジューラなので、Heavy を有効にすると点滅が乱れる",
        ],
        checks=[
            Check(r"Running: LED"),
            Check(r"LED toggled"),
            Check(r"Running: Servo"),
            Check(r"Servo angle: \d+"),
            Check(r"Running: Sensor"),
            Check(r"Sensor value: \d+"),
        ],
        capture=8.0,
    ),
    Chapter(
        "03_context_switch",
        title="本編 第3章 コンテキストスイッチの実装",
        watch=[
            "L LED が 100ms ごとに点滅（LED1 タスク）",
            "LED マトリクスの左半分が点滅（LED2 タスク）",
            "2 つが同時に動いていれば os_yield() でタスクが切り替わっている",
        ],
        checks=[
            Check(r"Task1 yield #\d+"),
            Check(r"Task2 yield #\d+"),
            Check(r"Task3 count: \d+"),
        ],
        capture=8.0,
    ),
    Chapter(
        "04_scheduler",
        title="本編 第4章 プリエンプティブスケジューラ",
        watch=[
            "L LED とマトリクスが点滅し続ける",
            "Heavy タスクが重い計算を回している最中も点滅が止まらない ← これが本章の主張",
        ],
        checks=[
            Check(r"Count: \d+"),
            Check(r"Current task: \w+", "プリエンプションで実行中タスクが変わる"),
        ],
        custom="preemption",
        capture=10.0,
    ),
    Chapter(
        "05_shell",
        title="本編 第5章 対話型シェル",
        watch=[
            "L LED（LED1）とマトリクス左半分（LED2）が点滅",
            "kill 1 を打つと L LED が止まる。exec 1 で復活する",
        ],
        try_cmds=["ps", "kill 1", "ps", "exec 1", "info"],
        send=["help", "ps"],
        checks=[
            Check(r"Available commands:"),
            Check(r"ID\s+NAME\s+STATE\s+CPU%"),
            Check(r"^0\s+idle\s+\w+", "idle タスクが ID 0 で並ぶ"),
        ],
        custom="cpu_sum",
        capture=10.0,
    ),
    Chapter(
        "06_memory_protection",
        title="本編 第6章 メモリ保護",
        watch=[
            "L LED（Heartbeat）が点滅し続ける",
            "d や m でタスクを故意にクラッシュさせても、Heartbeat の点滅は止まらない",
        ],
        try_cmds=["p", "d", "sleep:5", "p", "m", "sleep:7", "p"],
        send=["p", "d", "sleep:5", "p"],
        checks=[
            Check(r"ID\s+NAME\s+STATE"),
            Check(r"FAULT DETECTED", "フォールトが検出される",
                  unsupported_on=("renode",),
                  sim_note="Renode は未マップ領域への書き込みでも BusFault を上げない"),
            Check(r"Reason: Divide by zero", "原因が特定できている",
                  unsupported_on=("renode",),
                  sim_note="Renode の Armv7-M は CCR.DIV_0_TRP を実装していない"),
            Check(r"Crash_Div\s+TERMINATED", "クラッシュしたタスクだけが終了する"),
            Check(r"\[Heartbeat\] \d+", "他のタスクは動き続ける ← 第6章の主張"),
        ],
        capture=16.0,
    ),
    Chapter(
        "07_led_matrix",
        title="本編 第7章 LEDマトリクス可視化",
        watch=[
            "マトリクス上半分に CPU 負荷グラフ（左が古い、右が新しい）",
            "マトリクス下半分にタスクごとの実行状況",
            "kill 3 で Heavy を止めると、負荷グラフが目に見えて下がる",
        ],
        try_cmds=["ps", "kill 3", "ps", "exec 3"],
        send=["ps"],
        checks=[
            Check(r"ID\s+NAME\s+STATE"),
        ],
        custom="cpu_sum",
        capture=10.0,
    ),
    Chapter(
        "07_led_matrix_v2",
        title="第7章 LEDマトリクス可視化 v2（書籍未掲載の改良版）",
        watch=[
            "行 0-1: CPU 全体の負荷が左から伸びる横棒（12 個で 100%）",
            "行 3-7: LED / Light / Heavy / Shell / Display の CPU 使用率",
            "kill 3 で Heavy の棒が消え、そのぶん CPU 全体の棒が縮む",
        ],
        try_cmds=["top", "kill 3", "kill 2", "exec 2", "exec 3", "top"],
        # top の表も ps と同じ形の行を出すので、cpu_sum（ps の合計）は使わない
        send=["ps", "top", "sleep:1.5", "top"],
        checks=[
            Check(r"ID\s+NAME\s+STATE"),
            Check(r"CPU \[[# ]{12}\] \d+%", "top がマトリクスと同じ横棒を出す"),
        ],
        capture=10.0,
    ),
    Chapter(
        "08_interpreter",
        title="本編 第8章 簡易インタプリタ",
        watch=[
            "L LED が 500ms ごとに点滅",
            "LED タスクが L を 500ms ごとに反転し続けるので、run LED 13 1 だけでは次の反転で消える。",
            "kill 1 で LED タスクを止めてから run LED 13 1 を送ると点灯したままになる",
        ],
        try_cmds=["run FORWARD 10", "run PRINT hello", "kill 1", "run LED 13 1", "run LED 13 0"],
        send=["run FORWARD 10", "run PRINT hello"],
        checks=[
            Check(r"\[Motor\] Forward 10", "インタプリタが FORWARD を実行する"),
            Check(r"\[Print\] hello", "インタプリタが PRINT を実行する"),
        ],
        capture=10.0,
    ),
    Chapter(
        "09_integration",
        title="本編 第9章 統合とロボット制御",
        watch=[
            "L LED（LED_Ind）が点滅",
            "マトリクスにタスクの状態が出る",
        ],
        try_cmds=["ps", "info"],
        send=["ps"],
        checks=[
            Check(r"ID\s+NAME\s+STATE"),
        ],
        custom="cpu_sum",
        capture=10.0,
    ),
    Chapter(
        "10_freertos",
        title="本編 第10章 FreeRTOSで同じことをやってみる",
        watch=[
            "L LED が点滅（自作OS の API を FreeRTOS 風に見せた互換デモ）",
        ],
        checks=[
            Check(r"\[FreeRTOS"),
        ],
        capture=10.0,
    ),
    Chapter(
        "10_freertos_real",
        title="本編 第10章 本物の FreeRTOS（integrated 例）",
        watch=[
            "L LED が点滅。Tick と Free heap が 1 秒ごとに出る",
        ],
        env="integrated",   # env が 6 つあるので 1 つ選ぶ（本文 10.3 の注記どおり）
        checks=[
            Check(r"=== FreeRTOS Status ==="),
            Check(r"Tick: \d+"),
            Check(r"Free heap: \d+"),
        ],
        capture=12.0,
    ),
    Chapter(
        "11_tiny_python",
        title="本編 第11章 TinyPython",
        watch=[
            "led.on() で L LED が点灯、led.off() で消灯",
            "matrix.fill(1) でマトリクスが全点灯する",
        ],
        try_cmds=["print(1 + 2)", "led.on()", "led.off()", "led.blink(5)", "for i in range(3): print(i)"],
        send=["print(1 + 2)",
              "def fact(n):", "    if n < 2:", "        return 1",
              "    return n * fact(n - 1)", "",
              "print(fact(7))",      # 課題11-2 の値。TP_MAX_SCOPES=8 の内側
              "print(fact(30))",     # 上限超え。黙って固まらずエラーになること
              "print(42)"],          # そのあとも REPL が生きていること
        checks=[
            Check(r"^3\s*$", "print(1 + 2) が 3 を返す（本文 11.11 の REPL）"),
            Check(r">>>", "REPL のプロンプトが出る"),
            Check(r"^5040\s*$", "fact(7) は深さ上限の内側なので通る（課題11-2）"),
            Check(r"RecursionError", "深すぎる再帰は TP_MAX_SCOPES で止まる"),
            Check(r"^42\s*$", "RecursionError のあとも REPL が生きている"),
        ],
        capture=22.0,
    ),
    Chapter(
        "12_hardware",
        title="本編 第12章 ハードウェア制御の深層",
        watch=[
            "**A0 の可変抵抗と D9 のサーボが要る章**（無ければ pot の値が揺れるだけ）",
            "可変抵抗を回すとサーボが追従する。kill 4 で Heavy を止めても滑らかさは変わらない",
            "＝ ハードウェアPWM なので CPU 負荷の影響を受けない",
        ],
        try_cmds=["ps", "kill 4", "exec 4"],
        checks=[
            Check(r"pot\(14bit\)=\d+"),
            Check(r"servo=", "12 章 12.4 のステータス行"),
            Check(r"servo=-?\d+(\.\d+)?\s*deg", "サーボは角度[deg]で出る（us ではない）"),
        ],
        capture=10.0,
    ),
    Chapter(
        "13_quadruped",
        title="本編 第13章 四脚ロボットを歩かせる",
        watch=[
            "**シリアルバスサーボ 8 個が要る章**。ボードだけでも iktest は動く（脚は動かない）",
            "サーボ未接続だと read_fail が増え、1 周期が 24ms（= 8 × 3ms のタイムアウト）になる",
        ],
        try_cmds=["iktest", "stand", "trot", "rl 0.2 0", "stop"],
        send=["iktest"],
        checks=[
            Check(r"target=\(0\.000, 0\.1600\)\s+hip=40\.20\s+knee=-67\.47\s+err=0\.000000"),
            Check(r"target=\(0\.000, 0\.1205\)\s+hip=60\.43\s+knee=-101\.02\s+err=0\.000000",
                  "立ち姿勢 home（本文 13.2.1 の +59.1/-101.0 の近く）"),
            Check(r"target=\(0\.000, 0\.2500\)\s+hip=0\.00\s+knee=4\.36\s+err=0\.053000",
                  "届かない位置で切り詰めが効く（本文 13.3.8）"),
        ],
        capture=12.0,
        note="サーボ未接続でも iktest は動く（脚は動かない）",
    ),
    Chapter(
        "adv1_sync",
        title="応用編 第1章 ロックと同期",
        watch=[
            "LED は使わない。meals: の 5 つの数字が全部増え続けていれば誰も飢えていない",
        ],
        checks=[
            Check(r"meals: P0=\d+ P1=\d+ P2=\d+ P3=\d+ P4=\d+",
                  "5 人の哲学者が全員食事できている＝デッドロックしていない"),
        ],
        custom="philosophers",
        capture=12.0,
    ),
    Chapter(
        "adv2_syscall",
        title="応用編 第2章 ユーザー／カーネルモードと SVC",
        watch=[
            "LED は使わない。出力はすべて SVC 経由で出ている",
        ],
        checks=[
            Check(r"\[A\] pid=\d+", "ユーザータスクが svc 経由で出力できている"),
            Check(r"\[B\] count=\d+"),
        ],
        capture=8.0,
    ),
    Chapter(
        "adv3_heap",
        title="応用編 第3章 ヒープ自作",
        watch=[
            "LED は使わない。heap dump が段階ごとに出る",
        ],
        handshake="S",
        checks=[
            Check(r"=== Advanced 3: my_malloc / my_free ==="),
            Check(r"double free detected", "二重 free を検出する"),
            Check(r"done\."),
        ],
        capture=10.0,
    ),
    Chapter(
        "adv4_fs",
        title="応用編 第4章 最小ファイルシステム",
        watch=[
            "LED は使わない。リセットするたび boot count が増える（Flash に残っている証拠）",
        ],
        handshake="S",
        checks=[
            Check(r"=== Advanced 4: LittleFS on Data Flash ==="),
            Check(r"boot count = \d+"),
            Check(r"done\."),
        ],
        capture=12.0,
    ),
]


# --------------------------------------------------------------------------
# 追加検査
# --------------------------------------------------------------------------

def check_cpu_sum(log: str) -> tuple[bool, str]:
    """ps の CPU% 合計が 100% を超えないこと（本文 4.7 / 7.5）。"""
    rows = re.findall(r"^\s*\d+\s+\S+\s+\w+\s+(\d+)%", log, re.M)
    if not rows:
        return True, "ps の CPU% 行が取れなかったので判定を省略"
    total = sum(int(r) for r in rows)
    ok = total <= 105  # 計測窓のずれで数 % は許容
    return ok, f"CPU% の合計 = {total}%（{len(rows)} タスク）"


def check_preemption(log: str) -> tuple[bool, str]:
    """重い計算が回っていても 1 秒周期のタスクが遅れないこと（4 章の主張そのもの）。

    協調的スケジューラ（2 章）ならここで間隔が崩れる。プリエンプティブなら崩れない。
    """
    ts = [int(m) for m in re.findall(r"\[(\d+)ms\] Count:", log)]
    if len(ts) < 4:
        return False, f"Count 行が {len(ts)} 本しか取れなかった"
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    heavy = log.count("Heavy task completed iteration")
    ok = all(900 <= g <= 1150 for g in gaps) and heavy > 0
    return ok, f"Count の間隔 = {gaps} ms / Heavy の完了 {heavy} 回"


def check_philosophers(log: str) -> tuple[bool, str]:
    """食事する哲学者が全員進んでいること（誰も飢えていない = デッドロックしていない）。"""
    rows = re.findall(r"meals: P0=(\d+) P1=(\d+) P2=(\d+) P3=(\d+) P4=(\d+)", log)
    if len(rows) < 2:
        return False, f"meals 行が {len(rows)} 本しか取れなかった"
    first = [int(x) for x in rows[0]]
    last = [int(x) for x in rows[-1]]
    grew = [b - a for a, b in zip(first, last)]
    return all(g > 0 for g in grew), f"各哲学者の食事回数の増分 = {grew}"


CUSTOM_CHECKS = {
    "philosophers": check_philosophers,
    "cpu_sum": check_cpu_sum,
    "preemption": check_preemption,
}


# --------------------------------------------------------------------------
# ポート検出（3 OS 共通）
# --------------------------------------------------------------------------

def find_board_port(exclude: str | None = None) -> str | None:
    """UNO R4 WiFi のシリアルポートを探す。

    Linux は /dev/ttyACM*、macOS は /dev/cu.usbmodem*、Windows は COMn になる。
    VID で絞り、macOS では tty.* ではなく cu.* を選ぶ。
    """
    if serial is None:
        return None
    cands: list[str] = []
    for p in serial.tools.list_ports.comports():
        if p.vid in ARDUINO_VIDS:
            cands.append(p.device)
    if not cands:
        # VID が取れない環境向けのフォールバック
        for p in serial.tools.list_ports.comports():
            d = p.device
            if "usbmodem" in d or d.startswith("/dev/ttyACM") or re.fullmatch(r"COM\d+", d):
                cands.append(d)
    if platform.system() == "Darwin":
        cu = [c for c in cands if "/cu." in c]
        if cu:
            cands = cu
    cands = [c for c in cands if c != exclude] or cands
    return sorted(cands)[0] if cands else None


def wait_for_port(timeout: float = 20.0) -> str | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        port = find_board_port()
        if port:
            return port
        time.sleep(0.3)
    return None


# --------------------------------------------------------------------------
# ビルド / 書き込み
# --------------------------------------------------------------------------

def pio_cmd(*args: str) -> list[str]:
    """PATH に pio が無くても動くよう python -m platformio で呼ぶ。"""
    return [sys.executable, "-m", "platformio", *args]


def run_pio(chapter: str, target: str | None, port: str | None,
            env: str = "") -> tuple[bool, str]:
    args = ["run", "-d", chapter]
    if env:
        args += ["-e", env]
    if target:
        args += ["-t", target]
    if port:
        args += ["--upload-port", port]
    proc = subprocess.run(
        pio_cmd(*args),
        cwd=CODE_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return proc.returncode == 0, proc.stdout or ""


# --------------------------------------------------------------------------
# 捕捉
# --------------------------------------------------------------------------

def drive(ch: Chapter, rd, wr) -> str:
    """章の手順どおりに入力を送り、出力を捕捉する.

    実機（シリアルポート）でもシミュレータ（ソケット）でも同じ手順を踏めるよう、
    読み書きの関数だけを受け取る。``rd()`` は bytes を返し、``wr(bytes)`` は送る。
    """
    buf: list[str] = []
    deadline = time.time() + ch.capture

    # setup() でしか出さない章は、出力が始まるまでハンドシェイク文字を送り続ける
    if ch.handshake:
        hs_deadline = time.time() + 6.0
        while time.time() < hs_deadline and not buf:
            wr(ch.handshake.encode())
            time.sleep(0.25)
            chunk = rd()
            if chunk:
                buf.append(chunk.decode("utf-8", errors="replace"))

    for line in ch.send:
        # "sleep:5" と書くと、そこで 5 秒待つ（クラッシュが起きるまで待つ等）
        m = re.fullmatch(r"sleep:([\d.]+)", line)
        if m:
            deadline = max(deadline, time.time() + float(m.group(1)))
            end = time.time() + float(m.group(1))
            while time.time() < end:
                chunk = rd()
                if chunk:
                    buf.append(chunk.decode("utf-8", errors="replace"))
            continue
        time.sleep(0.8)
        wr((line + "\n").encode())

    while time.time() < deadline:
        chunk = rd()
        if chunk:
            buf.append(chunk.decode("utf-8", errors="replace"))
    return "".join(buf)


def capture(ch: Chapter, port: str) -> str:
    """出力を捕捉する。

    macOS / Windows では、書き込み直後にボードが USB を張り直すことがある。
    そのとき read() は「Device not configured」で例外を投げるので、捕まえて
    ポートを開き直す。ここで落ちると、以降の章がまとめて検証できなくなる。
    """
    buf: list[str] = []
    deadline = time.time() + ch.capture

    def reopen():
        for _ in range(20):
            p = find_board_port() or port
            try:
                return serial.Serial(p, BAUD, timeout=0.2)
            except Exception:  # noqa: BLE001  まだ列挙が終わっていない
                time.sleep(0.5)
        return None

    ser = reopen()
    if ser is None:
        return f"[capture] ポートを開けません: {port}\n"

    def rd(n=4096):
        """読む。切れたら開き直して続ける。"""
        nonlocal ser
        try:
            return ser.read(n)
        except Exception:  # noqa: BLE001
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
            ser = reopen()
            buf.append("\n[capture] ポートが切れたので開き直しました\n")
            return b""

    def wr(data: bytes):
        nonlocal ser
        try:
            ser.write(data)
            ser.flush()
        except Exception:  # noqa: BLE001
            ser = reopen()

    try:
        time.sleep(0.3)
        try:
            ser.reset_input_buffer()
        except Exception:  # noqa: BLE001
            pass
        buf.append(drive(ch, rd, wr))
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
    return "".join(buf)



# --------------------------------------------------------------------------
# シミュレータ（RA4M1 のエミュレータ）で走らせる
# --------------------------------------------------------------------------

SIM_DIR = CODE_ROOT / "sim"
SIM_UART = "sci9"          # -D NO_USB の Serial（_UART1_ = P109/P110）は SCI9


def sim_process():
    """sim/emulator_process.py を読み込む（エミュレータを孤児にしない起動・停止）。"""
    if str(SIM_DIR) not in sys.path:
        sys.path.insert(0, str(SIM_DIR))
    import emulator_process
    return emulator_process


def has_sim_env(chapter: str) -> bool:
    """その章に [env:sim] があるか（実機専用の章には無い）。"""
    ini = CODE_ROOT / chapter / "platformio.ini"
    return ini.exists() and "[env:sim]" in ini.read_text(encoding="utf-8")


def find_renode(explicit: str | None = None) -> str | None:
    """renode の実行ファイルを探す。--renode > $RENODE > PATH > 既定の場所。"""
    for cand in (explicit, os.environ.get("RENODE")):
        if cand and Path(cand).exists():
            return cand
    found = shutil.which("renode")
    if found:
        return found
    default = Path("/Applications/Renode.app/Contents/MacOS/renode")
    return str(default) if default.exists() else None


def find_qemu(explicit: str | None = None) -> tuple[str | None, str]:
    """arduino-uno-r4 マシン入りの qemu-system-arm を探す（sim/emulator_process.py と共通）。"""
    return sim_process().find_qemu(explicit)


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def capture_emulator(ch: Chapter, cmd: list[str], uart_port: int, name: str,
                     capture_scale: float = 1.0) -> str:
    """エミュレータを起動し、UART（TCP）につないで章の手順どおりに捕捉する。

    ``capture_scale`` は捕捉時間の倍率。CI の遅いランナーではエミュレータの
    進みが実時間より遅くなるので、2.0 などに伸ばして取りこぼしを防ぐ。
    """
    if capture_scale != 1.0:
        ch = replace(ch, capture=ch.capture * capture_scale)
    rp = sim_process()
    proc = rp.start(cmd)
    sock = None
    try:
        deadline = time.time() + 60
        while time.time() < deadline and sock is None:
            try:
                sock = socket.create_connection(("127.0.0.1", uart_port), 2.0)
            except OSError:
                if proc.poll() is not None:
                    return f"[sim] {name} が起動直後に終了しました\n"
                time.sleep(0.3)
        if sock is None:
            return f"[sim] {name} の UART につながりません\n"
        sock.settimeout(0.2)

        def rd(n: int = 4096) -> bytes:
            try:
                return sock.recv(n)
            except (socket.timeout, OSError):
                return b""

        def wr(data: bytes) -> None:
            try:
                sock.sendall(data)
            except OSError:
                pass

        return drive(ch, rd, wr)
    finally:
        if sock is not None:
            sock.close()
        rp.stop(proc)           # 子プロセスまでグループごと止める


def capture_renode(ch: Chapter, elf: Path, renode: str,
                   capture_scale: float = 1.0) -> str:
    uart_port = free_port()
    with tempfile.TemporaryDirectory(prefix="verify-sim-") as tmp:
        resc = Path(tmp) / "run.resc"
        resc.write_text(
            "using sysbus\n"
            'mach create "unor4"\n'
            f"machine LoadPlatformDescription @{(SIM_DIR / 'unor4_board.repl').as_posix()}\n"
            f"sysbus LoadELF @{elf.as_posix()}\n"
            f'emulation CreateServerSocketTerminal {uart_port} "uart" false\n'
            f"connector Connect {SIM_UART} uart\n"
            "start\n",
            encoding="utf-8",
        )
        cmd = [renode, "--disable-xwt", "--plain", "--hide-log",
               "--port", str(free_port()), str(resc)]
        return capture_emulator(ch, cmd, uart_port, "Renode", capture_scale)


def capture_qemu(ch: Chapter, elf: Path, qemu: str, icount: str,
                 capture_scale: float = 1.0) -> str:
    uart_port = free_port()
    cmd = [qemu, "-M", sim_process().QEMU_MACHINE, "-kernel", str(elf),
           "-display", "none", "-monitor", "none",
           # wait=on: つなぐまで起動を待つので、最初の出力を取りこぼさない
           "-serial", f"tcp:127.0.0.1:{uart_port},server=on,wait=on"]
    if icount:
        # 1 命令あたりの時間を固定して、48 MHz の実機に近い速さで走らせる
        cmd += ["-icount", icount]
    return capture_emulator(ch, cmd, uart_port, "QEMU", capture_scale)


# --------------------------------------------------------------------------
# 本体
# --------------------------------------------------------------------------

def verify(ch: Chapter, args) -> dict:
    result = {"chapter": ch.name, "build": None, "upload": None,
              "checks": [], "status": "", "log": ""}

    if getattr(args, "sim", False):
        if not has_sim_env(ch.name):
            result["status"] = "SIM-SKIP"     # 実機の部品が要る章
            return result
        ok, out = run_pio(ch.name, None, None, "sim")
        result["build"] = "OK" if ok else "FAIL"
        if not ok:
            result["status"] = "BUILD-FAIL"
            result["log"] = out[-4000:]
            return result
        if args.build_only:
            result["status"] = "BUILD-OK"
            return result
        elf = CODE_ROOT / ch.name / ".pio/build/sim/firmware.elf"
        if args.emulator == "qemu":
            log = capture_qemu(ch, elf, args.qemu_path, args.icount,
                               args.capture_scale)
        else:
            log = capture_renode(ch, elf, args.renode_path, args.capture_scale)
        result["log"] = log
        result["status"] = judge(ch, log, result, emulator=args.emulator)
        return result

    ok, out = run_pio(ch.name, None, None, ch.env)
    result["build"] = "OK" if ok else "FAIL"
    if not ok:
        result["status"] = "BUILD-FAIL"
        result["log"] = out[-4000:]
        return result

    if args.build_only:
        result["status"] = "BUILD-OK"
        return result

    port = args.port or wait_for_port(10)
    if port is None:
        result["status"] = "NO-BOARD"
        return result

    ok, out = run_pio(ch.name, "upload", port, ch.env)
    result["upload"] = "OK" if ok else "FAIL"
    if not ok:
        result["status"] = "UPLOAD-FAIL"
        result["log"] = out[-4000:]
        return result

    time.sleep(ch.settle)
    port = args.port or wait_for_port(20) or port
    log = capture(ch, port)
    result["log"] = log
    result["status"] = judge(ch, log, result)
    return result


def judge(ch: Chapter, log: str, result: dict, emulator: str | None = None) -> str:
    """捕捉したログを期待値と突き合わせ、PASS / FAIL を返す。

    ``emulator`` はエミュレータで採ったログのときにその名前（実機なら None）。
    """
    failed = []
    for c in ch.checks:
        if emulator and emulator in c.unsupported_on:
            # エミュレータでは再現しない項目。黙って飛ばすと嘘になるので、
            # 結果には「飛ばした」と理由を残す。
            result["checks"].append(
                {"pattern": c.pattern, "why": c.why, "absent": c.absent,
                 "ok": True, "skipped": True, "sim_note": c.sim_note}
            )
            continue
        hit = re.search(c.pattern, log, re.M) is not None
        good = (not hit) if c.absent else hit
        result["checks"].append(
            {"pattern": c.pattern, "why": c.why, "absent": c.absent, "ok": good}
        )
        if not good:
            failed.append(c)
    if ch.custom:
        cok, msg = CUSTOM_CHECKS[ch.custom](log)
        result["checks"].append(
            {"pattern": f"<{ch.custom}>", "why": msg, "absent": False, "ok": cok}
        )
        if not cok:
            failed.append(Check(f"<{ch.custom}>", msg))
    return "PASS" if not failed else "FAIL"



# --------------------------------------------------------------------------
# 対話モード — 1 章ずつ焼いて、実機を見ながら Enter で進む
# --------------------------------------------------------------------------

def _reader(ser, stop_flag: list) -> None:
    """シリアルを読んで、そのまま画面に流し続けるスレッド。"""
    while not stop_flag:
        try:
            chunk = ser.read(256)
        except Exception:  # noqa: BLE001  ポートが閉じられた
            return
        if chunk:
            sys.stdout.write(chunk.decode("utf-8", errors="replace"))
            sys.stdout.flush()


def interactive(targets: list[Chapter], args) -> int:
    import threading

    print()
    print("対話モード。1 章ずつ書き込んで、実機を見ながら進みます。")
    print("  Enter だけ  … 次の章へ")
    print("  文字を打つ  … そのままボードに送る（シェルのコマンドなど）")
    print("  !           … その章の「試せること」を上から順に送る")
    print("  r / b / q   … 焼き直し / 前の章へ / 終了")
    print()

    i = 0
    while 0 <= i < len(targets):
        ch = targets[i]
        print("\n" + "=" * 72)
        print(f"[{i + 1}/{len(targets)}] {ch.name}"
              + (f"   {ch.title}" if ch.title else ""))
        print("=" * 72)
        if ch.watch:
            print("目で見るポイント:")
            for w in ch.watch:
                print(f"  ・{w}")
        if ch.try_cmds:
            print("試せること: " + " / ".join(ch.try_cmds))
        if ch.note:
            print(f"補足: {ch.note}")
        print("-" * 72)

        print("書き込み中 ...", end="", flush=True)
        ok, out = run_pio(ch.name, None, None, ch.env)
        if not ok:
            print(" ビルド失敗")
            print(out[-1500:])
            i += 1
            continue
        port = args.port or wait_for_port(10)
        if port is None:
            print(" ボードが見つかりません")
            return 2
        ok, out = run_pio(ch.name, "upload", port, ch.env)
        if not ok:
            print(" 書き込み失敗")
            print(out[-1500:])
            i += 1
            continue
        print(" OK")

        time.sleep(ch.settle)
        port = args.port or wait_for_port(20) or port
        try:
            ser = serial.Serial(port, BAUD, timeout=0.2)
        except Exception as exc:  # noqa: BLE001
            print(f"ポートを開けません: {exc}")
            return 2

        stop: list = []
        th = threading.Thread(target=_reader, args=(ser, stop), daemon=True)
        th.start()
        if ch.handshake:
            for _ in range(12):     # setup() でしか出さない章を起こす
                ser.write(ch.handshake.encode())
                ser.flush()
                time.sleep(0.2)

        print("-" * 72)
        action = "next"
        while True:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                action = "quit"
                break
            cmd = line.strip()
            if cmd == "":
                action = "next"
                break
            if cmd in ("q", "quit"):
                action = "quit"
                break
            if cmd == "r":
                action = "redo"
                break
            if cmd == "b":
                action = "back"
                break
            lines = ch.try_cmds if cmd == "!" else [line]
            if cmd == "!" and not ch.try_cmds:
                print("(この章に「試せること」はありません)")
                continue
            for one in lines:
                m = re.fullmatch(r"sleep:([\d.]+)", one)
                if m:
                    time.sleep(float(m.group(1)))
                    continue
                if cmd == "!":
                    print(f"\n>>> {one}")
                try:
                    ser.write((one + "\n").encode())
                    ser.flush()
                except Exception as exc:  # noqa: BLE001
                    print(f"送信できません: {exc}")
                    break
                # 返事が画面に出てから次のプロンプトに戻る
                time.sleep(1.2 if cmd == "!" else 0.4)

        stop.append(True)
        time.sleep(0.3)
        ser.close()

        if action == "quit":
            print("終了します。")
            return 0
        if action == "redo":
            continue
        i += -1 if action == "back" else 1
        if i < 0:
            i = 0
    print("\n最後の章まで来ました。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="+", help="この章だけ実行する")
    ap.add_argument("--skip", nargs="+", default=[], help="この章を飛ばす")
    ap.add_argument("--port", help="シリアルポートを明示する（COM5, /dev/ttyACM0, /dev/cu.usbmodem...）")
    ap.add_argument("--build-only", action="store_true", help="ボード無しでビルドだけ検証する")
    ap.add_argument("--sim", action="store_true",
                    help="実機の代わりにエミュレータで検証する（付録のシミュレータ）")
    ap.add_argument("--emulator", choices=["qemu", "renode"],
                    help="--sim で使うエミュレータ（既定: qemu）。指定すれば --sim も付く")
    ap.add_argument("--renode", dest="renode_path",
                    help="renode の実行ファイル（既定: $RENODE か PATH から探す）")
    ap.add_argument("--qemu", dest="qemu_path",
                    help="arduino-uno-r4 マシン入りの qemu-system-arm"
                         "（既定: $QEMU → ~/qemu-unor4 → PATH の順に探す）")
    ap.add_argument("--icount", default=sim_process().QEMU_ICOUNT,
                    help="QEMU の -icount（既定: 48 MHz 相当で実時間に合わせる。空でホストの速さ）")
    ap.add_argument("--capture-scale", type=float, default=1.0,
                    help="--sim のとき捕捉時間を何倍にするか（遅い CI 向け）")
    ap.add_argument("--list", action="store_true", help="章と期待値を一覧表示する")
    ap.add_argument("--json", help="結果を JSON で書き出す")
    ap.add_argument("-i", "--interactive", action="store_true",
                    help="1 章ずつ焼いて、実機を見ながら Enter で進む")
    args = ap.parse_args()
    if args.emulator:
        args.sim = True               # エミュレータを選んだならシミュレータで検証する
    args.emulator = args.emulator or "qemu"

    if args.list:
        for ch in CHAPTERS:
            print(f"{ch.name}")
            for c in ch.checks:
                print(f"    - {c.pattern}" + (f"   … {c.why}" if c.why else ""))
            if ch.custom:
                print(f"    - <{ch.custom}>")
        return 0

    if args.sim:
        sim_process().install_signal_handlers()
        if args.emulator == "qemu":
            args.qemu_path, why = find_qemu(args.qemu_path)
            if args.qemu_path is None and not args.build_only:
                print(why)
                return 2
        else:
            args.renode_path = find_renode(args.renode_path)
            if args.renode_path is None and not args.build_only:
                print("renode が見つかりません。--renode か環境変数 RENODE で場所を渡してください。")
                return 2
    elif serial is None and not args.build_only:
        print("pyserial がありません。`uv sync` を実行するか --build-only を付けてください。")
        return 2

    # 実機のログとシミュレータのログを混ぜない（どちらで採ったか分からなくなる）
    log_dir = RESULTS_DIR
    if args.sim:
        log_dir = RESULTS_DIR / ("sim" if args.emulator == "renode" else f"sim-{args.emulator}")
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"OS: {platform.system()} {platform.release()}  Python: {sys.version.split()[0]}")
    if args.sim and args.emulator == "qemu":
        print(f"シミュレータ: QEMU {args.qemu_path}  (-icount {args.icount or 'なし'})")
    elif args.sim:
        print(f"シミュレータ: Renode {args.renode_path}")
    elif not args.build_only:
        port = args.port or find_board_port()
        print(f"ボード: {port or '見つかりません'}")

    targets = [c for c in CHAPTERS
               if (not args.only or c.name in args.only) and c.name not in args.skip]

    if args.interactive:
        return interactive(targets, args)

    results = []
    for ch in targets:
        print(f"\n=== {ch.name} " + "=" * (60 - len(ch.name)), flush=True)
        t0 = time.time()
        try:
            r = verify(ch, args)
        except Exception as exc:  # noqa: BLE001  ここで止めると残りの章が測れない
            r = {"chapter": ch.name, "build": None, "upload": None, "checks": [],
                 "status": f"ERROR: {type(exc).__name__}: {exc}", "log": ""}
        r["seconds"] = round(time.time() - t0, 1)
        results.append(r)
        if r["log"]:
            (log_dir / f"{ch.name}.log").write_text(r["log"], encoding="utf-8")
        print(f"  -> {r['status']}  ({r['seconds']}s)", flush=True)
        for c in r["checks"]:
            mark = "SKIP" if c.get("skipped") else ("OK  " if c["ok"] else "NG  ")
            # 読者が見たいのは「何を確かめたか」なので、説明があればそれを出す。
            # 正規表現そのものは、落ちたときだけ添える（直す手がかりになる）。
            label = c["why"] or c["pattern"]
            if c.get("skipped"):
                detail = f"   ← {c['sim_note']}" if c.get("sim_note") else ""
            else:
                detail = "" if (c["ok"] or not c["why"]) else f"   [{c['pattern']}]"
            print(f"     {mark} {label}{detail}")

    print("\n" + "=" * 72)
    print(f"{'章':24s} {'結果':12s} 秒")
    print("=" * 72)
    bad = 0
    for r in results:
        print(f"{r['chapter']:24s} {r['status']:12s} {r['seconds']}")
        if r["status"] not in ("PASS", "BUILD-OK", "SIM-SKIP"):
            bad += 1
    print("=" * 72)
    skipped = sum(1 for r in results if r["status"] == "SIM-SKIP")
    print(f"{len(results) - bad - skipped} / {len(results) - skipped} 章が成功"
          + (f"（実機が要る {skipped} 章は対象外）" if skipped else ""))

    if args.json:
        Path(args.json).write_text(json.dumps(results, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""シェル / REPL に変な入力を投げて、OS が生き残るかを実機で確かめる.

読者は必ず打ち間違える。長すぎる行、存在しないタスク ID、制御文字、閉じていない
括弧 —— それでボードが固まるなら、本文がどれだけ正しくても読者は詰まる。

各章について次をやる。

1. 書き込む
2. 「行儀の悪い入力」を 1 つずつ送る
3. **そのたびに生存確認**（probe コマンドを送って応答が返るか）
4. どの入力で応答が止まったかを報告する

    uv run python scripts/fuzz_shell.py                 # 全対象
    uv run python scripts/fuzz_shell.py --only 05_shell
    uv run python scripts/fuzz_shell.py --port COM3
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:  # pragma: no cover
    pass

sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_chapters import (  # noqa: E402
    BAUD, run_pio, wait_for_port, find_board_port, serial,
)


# よくある打ち間違い・意地悪な入力。どれも「返ってこなくなったら負け」。
COMMON = [
    "",                       # 空行
    "   ",                    # 空白だけ
    "help",                   # 正常系（対照）
    "HELP",                   # 大文字（未知コマンド扱いのはず）
    "?",
    "nosuchcommand",
    "kill",                   # 引数なし
    "kill ",                  # 引数が空
    "kill 0",                 # idle は殺せないはず
    "kill -1",
    "kill 999",
    "kill 2147483647",
    "kill -2147483648",
    "kill abc",               # 数字でない
    "kill 1 2 3",
    "exec 999",
    "exec -1",
    "exec abc",
    "ps ps ps",
    "  ps  ",                 # 前後に空白
    "A" * 200,                # バッファ長 (64) を大きく超える行
    "kill " + "9" * 100,      # 数字の桁あふれ
    "\x01\x02\x03\x1b[A",     # 制御文字・矢印キーのエスケープ
    "ps\x7f\x7f",             # DEL を含む
    "%s %d %n %x",            # 書式指定子（printf 系の事故狙い）
    "../../etc/passwd",
    "ps;ps&&ps|ps",
]

PY_FUZZ = [
    "print(1 + 2)",
    "print(",                 # 閉じていない
    "print('abc",             # 閉じていない文字列
    "1/0",                    # ゼロ除算
    "undefined_name",
    "print(undefined_name)",
    "((((((((((1))))))))))",
    "[[[[[[[[[[",
    "print(999999999999999999999)",
    "def f(): return f()",    # 定義だけ（呼ばない）
    "for i in range(3): print(i)",
    "for i in range(: print(i)",
    "x = " + "1+" * 100 + "1",
    "print('" + "A" * 200 + "')",
    "\x01\x02\x03",
    "import nosuchmodule",
]

INTERP_FUZZ = [
    "run",                    # 引数なし → 複数行モードに入る
    "",                       # 空行で抜ける
    "run FORWARD",            # 引数不足
    "run FORWARD abc",
    "run FORWARD 999999999",
    "run SERVO 99 999",       # 範囲外チャネル
    "run LED 999 1",
    "run NOSUCHCMD 1",
    "run LOOP 999999",
    "run END",                # 対応する LOOP が無い
    "run " + "A" * 200,
]


@dataclass
class Target:
    name: str
    probe: str                       # 生存確認に送るコマンド
    alive: str                       # 応答に含まれるはずの正規表現
    inputs: list = field(default_factory=list)
    env: str = ""
    handshake: str = ""
    boot: float = 3.0
    probe_tries: int = 2   # 1 文字ずつ読む章は長い行を捌くのに時間がかかる
    # 「応答が返らないのが仕様どおり」の入力。落ちたのではなく寝ているだけのもの。
    expected_block: dict = field(default_factory=dict)
    note: str = ""


TARGETS = [
    Target("05_shell", "ps", r"ID\s+NAME\s+STATE", COMMON),
    Target("06_memory_protection", "p", r"ID\s+NAME\s+STATE",
           ["", "   ", "x", "P", "pp", "d d d", "\x01\x02", "A" * 200],
           probe_tries=8,   # 1 文字ずつ読むので 200 文字を捌くのに 15 秒ほどかかる
           note="'d' が 1 文字ずつ効くので、連打するとクラッシュタスクが増える"),
    Target("07_led_matrix", "ps", r"ID\s+NAME\s+STATE", COMMON),
    Target("08_interpreter", "ps", r"ID\s+NAME\s+STATE", COMMON + INTERP_FUZZ,
           expected_block={
               # motor_forward() は os_sleep(distance * 10) で「仮の待機」をする。
               # 巨大な値だと int が溢れて数日ぶん寝るので、シェルは返ってこない。
               # クラッシュではなく設計どおりの待機（リセットで復帰）。
               "run FORWARD 999999999": "os_sleep(distance * 10) で長時間眠るのが仕様",
           }),
    Target("09_integration", "ps", r"ID\s+NAME\s+STATE", COMMON),
    Target("11_tiny_python", "print(42)", r"^42\s*$", PY_FUZZ),
    Target("13_quadruped", "iktest", r"target=\(0\.000, 0\.1600\)",
           ["", "   ", "nosuch", "rl", "rl abc def", "rl 999 999",
            "rl -999 -999", "trot trot", "A" * 200, "\x01\x02\x03",
            "stop", "free", "stand"]),
]


def drain(ser, seconds: float) -> str:
    buf = []
    end = time.time() + seconds
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            buf.append(chunk.decode("utf-8", errors="replace"))
    return "".join(buf)


def probe_alive(ser, t: Target, tries: int = 0) -> tuple[bool, str]:
    """probe を送って応答が返るか。返らなければ固まっている。

    先に空行を 1 つ送る。`run`（引数なし）や `def f():` のように複数行モードへ
    入るコマンドがあり、空行で抜けないと probe がスクリプトの一部として食われて
    「固まった」と誤判定してしまうため。空行はどの章でも無害。
    """
    tries = tries or t.probe_tries
    ser.write(b"\n")
    ser.flush()
    drain(ser, 0.4)
    for _ in range(tries):
        ser.reset_input_buffer()
        ser.write((t.probe + "\n").encode())
        ser.flush()
        out = drain(ser, 2.5)
        if re.search(t.alive, out, re.M):
            return True, out
    return False, out


def fuzz(t: Target, args) -> dict:
    res = {"chapter": t.name, "sent": 0, "died_on": None, "status": ""}

    ok, out = run_pio(t.name, None, None, t.env)
    if not ok:
        res["status"] = "BUILD-FAIL"
        return res
    port = args.port or wait_for_port(10)
    if port is None:
        res["status"] = "NO-BOARD"
        return res
    ok, out = run_pio(t.name, "upload", port, t.env)
    if not ok:
        res["status"] = "UPLOAD-FAIL"
        return res

    time.sleep(t.boot)
    port = args.port or wait_for_port(20) or port
    ser = serial.Serial(port, BAUD, timeout=0.2)
    with ser:
        if t.handshake:
            for _ in range(10):
                ser.write(t.handshake.encode())
                ser.flush()
                time.sleep(0.2)
        drain(ser, 2.0)

        alive, _ = probe_alive(ser, t)
        if not alive:
            res["status"] = "起動直後に応答なし"
            return res

        for i, s in enumerate(t.inputs):
            ser.write((s + "\n").encode("utf-8", errors="replace"))
            ser.flush()
            drain(ser, 0.5)
            res["sent"] += 1
            alive, _ = probe_alive(ser, t)
            if not alive:
                why = t.expected_block.get(s)
                if why:
                    res.setdefault("known", []).append(f"{s!r}: {why}")
                    break          # 以降は測れないので、この章はここで打ち切る
                res["died_on"] = s
                res["status"] = f"入力 {i + 1} 件目で応答が止まった"
                return res
    res["status"] = "OK"
    return res


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="+")
    ap.add_argument("--port")
    args = ap.parse_args()

    if serial is None:
        print("pyserial がありません。uv sync してください。")
        return 2
    print(f"ボード: {args.port or find_board_port()}")

    results = []
    for t in TARGETS:
        if args.only and t.name not in args.only:
            continue
        print(f"\n=== {t.name}  ({len(t.inputs)} 種の入力) " + "=" * 20, flush=True)
        r = fuzz(t, args)
        results.append(r)
        print(f"  -> {r['status']}  ({r['sent']} 件送信)")
        for k in r.get("known", []):
            print(f"     既知（仕様どおり）: {k}")
        if r["died_on"] is not None:
            print(f"     止まった入力: {r['died_on']!r}")

    print("\n" + "=" * 60)
    bad = 0
    for r in results:
        mark = "OK  " if r["status"] == "OK" else "NG  "
        print(f"{mark}{r['chapter']:24s} {r['sent']:3d} 件  {r['status']}")
        if r["status"] != "OK":
            bad += 1
    print("=" * 60)
    print(f"{len(results) - bad} / {len(results)} 章が生き残りました")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())

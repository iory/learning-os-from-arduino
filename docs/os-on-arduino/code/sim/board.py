#!/usr/bin/env python3
"""Arduino UNO R4 WiFi の仮想ボード。

実機と同じ PlatformIO ビルドのファームウェアを RA4M1 のエミュレータで動かし、
12x8 LED マトリクス・内蔵 LED・シリアルコンソールをブラウザに表示する。

使い方
------
    cd ../04_scheduler && pio run -e sim
    python3 ../sim/board.py --chapter ../04_scheduler
    # → http://127.0.0.1:8080 を開く

エミュレータは 2 つから選べる。

- QEMU（既定）: arduino-uno-r4 マシンを足した QEMU が要る
  （https://github.com/iory/qemu-arduino-uno-r4/releases）。~/qemu-unor4 に展開するか、
  --qemu か環境変数 QEMU で場所を渡す。
- Renode: ``--emulator renode``。https://github.com/renode/renode/releases から入れる。
  PATH に無い場合は --renode か環境変数 RENODE で場所を渡す。第6章のフォールトは起きない。
"""

from __future__ import annotations

import argparse
import http.server
import json
import os
import queue
import re
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import emulator_process                                  # noqa: E402

HERE = Path(__file__).resolve().parent

# 実機と同じ RA4M1 のレジスタ／シンボル
PORT1_PCNTR1 = 0x40040020      # D13 (P102) を含むポート
LED_D13_BIT = 1 << 18          # PODR bit2 = PCNTR1 bit18
FRAMEBUFFER_SYMBOL = "_ZL11framebuffer"   # Arduino_LED_Matrix の static framebuffer
FRAMEBUFFER_BYTES = 12         # 96 LED = 12 バイト


def find_symbol(elf: Path, symbol: str) -> int | None:
    """ELF の symtab から静的シンボルのアドレスを引く。見つからなければ None.

    arm-none-eabi-nm を呼ばずに自前で読む。ツールチェーンの場所や
    拡張子 (.exe) を気にせず、どの OS でも同じように動く。
    """
    data = elf.read_bytes()
    if data[:4] != b"\x7fELF" or data[4] != 1:        # ELF32 のみ
        return None
    (e_shoff,) = struct.unpack_from("<I", data, 0x20)
    e_shentsize, e_shnum = struct.unpack_from("<HH", data, 0x2E)

    def section(i: int) -> tuple[int, int, int, int]:
        off = e_shoff + i * e_shentsize
        sh_name, sh_type = struct.unpack_from("<II", data, off)
        sh_offset, sh_size = struct.unpack_from("<II", data, off + 0x10)
        sh_link, = struct.unpack_from("<I", data, off + 0x18)
        return sh_type, sh_offset, sh_size, sh_link

    SHT_SYMTAB = 2
    for i in range(e_shnum):
        sh_type, sh_offset, sh_size, sh_link = section(i)
        if sh_type != SHT_SYMTAB:
            continue
        _, str_off, str_size, _ = section(sh_link)      # 対になる .strtab
        for off in range(sh_offset, sh_offset + sh_size, 16):
            st_name, st_value = struct.unpack_from("<II", data, off)
            if st_name == 0:
                continue
            end = data.index(b"\0", str_off + st_name)
            if data[str_off + st_name:end].decode("ascii", "replace") == symbol:
                return st_value
    return None


def free_port() -> int:
    """空いている TCP ポートを 1 つ borrow する。"""
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class Monitor:
    """Renode の Monitor に TCP でつなぎ、コマンドを一往復させる."""

    PROMPT = re.compile(rb"\([A-Za-z0-9_.\-]+\) $")

    def __init__(self, port: int, timeout: float = 30.0):
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), 2.0)
                break
            except OSError as exc:                      # まだ listen していない
                last = exc
                time.sleep(0.2)
        else:
            raise RuntimeError(f"Renode の Monitor につながらない: {last}")
        self.sock.settimeout(10.0)
        self.lock = threading.Lock()
        self._read_until_prompt()

    def _read_until_prompt(self) -> str:
        buf = b""
        while not self.PROMPT.search(buf):
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RuntimeError("Monitor の接続が切れた")
            buf += chunk
        return buf.decode("utf-8", "replace")

    def command(self, cmd: str) -> str:
        """コマンドを送って、その応答だけを返す.

        Renode は受け取ったコマンドをそのまま echo するので、その行を捨てる。
        捨てないと、コマンドに含まれるアドレス（例: 0x40040020）を
        読み出した値だと勘違いしてしまう。
        """
        with self.lock:
            self.sock.sendall((cmd + "\n").encode())
            out = self._read_until_prompt()
        _, echoed, rest = out.partition(cmd)
        return rest if echoed else out


def _connect(port: int, what: str, timeout: float = 30.0) -> socket.socket:
    """エミュレータが listen を始めるまで待ってつなぐ。"""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            return socket.create_connection(("127.0.0.1", port), 2.0)
        except OSError as exc:
            last = exc
            time.sleep(0.2)
    raise RuntimeError(f"{what} につながらない: {last}")


class RenodeBackend:
    """Renode の RA4M1（Arduino Uno R4 Minima 定義）で動かす."""

    name = "Renesas RA4M1 on Renode"

    def __init__(self, elf: Path, renode: Path, repl: str, uart: str,
                 monitor_port: int, uart_port: int):
        self.elf, self.renode, self.repl, self.uart = elf, renode, repl, uart
        self.monitor_port, self.uart_port = monitor_port, uart_port
        self.proc: subprocess.Popen | None = None

    def start(self) -> None:
        # Renode に渡す起動スクリプト。何を組み立てているか読めるよう、
        # 消さずに一時ディレクトリへ残して場所を表示する。
        script = Path(tempfile.mkdtemp(prefix="unor4-sim-")) / "board.resc"
        # パスは Windows でも "/" 区切りで書く（"\\" はエスケープが要るうえ、
        # 忘れると黙って読み飛ばされる）。
        script.write_text(
            "using sysbus\n"
            'mach create "unor4"\n'
            f"machine LoadPlatformDescription @{Path(self.repl).resolve().as_posix()}\n"
            f"sysbus LoadELF @{self.elf.resolve().as_posix()}\n"
            f'emulation CreateServerSocketTerminal {self.uart_port} "uartsock" false\n'
            f"connector Connect {self.uart} uartsock\n"
            "start\n",
            encoding="utf-8",
        )
        print(f"  renode script: {script}")
        self.proc = emulator_process.start(
            [str(self.renode), "--disable-xwt", "--plain", "--hide-log",
             "--port", str(self.monitor_port), str(script)])

    def connect(self) -> socket.socket:
        # Renode は Monitor が先に立ち上がり、UART のソケットはスクリプト実行後
        self.monitor = Monitor(self.monitor_port)
        return _connect(self.uart_port, "Renode の UART")

    def read_bytes(self, addr: int, n: int) -> list[int]:
        out = self.monitor.command(f"sysbus ReadBytes {hex(addr)} {n}")
        return [int(v, 16) for v in re.findall(r"0x([0-9A-Fa-f]{2})\b", out)]

    def read_u32(self, addr: int) -> int | None:
        m = re.search(r"0x([0-9A-Fa-f]{8})",
                      self.monitor.command(f"sysbus ReadDoubleWord {hex(addr)}"))
        return int(m.group(1), 16) if m else None

    def stop(self) -> None:
        emulator_process.stop(self.proc)    # dotnet 本体までグループごと止める
        self.proc = None


class Qmp:
    """QEMU の QMP（JSON の Monitor）で HMP コマンドを一往復させる."""

    def __init__(self, port: int):
        self.sock = _connect(port, "QEMU の QMP")
        self.sock.settimeout(10.0)
        self.file = self.sock.makefile("rw", encoding="utf-8")
        self.lock = threading.Lock()
        self._recv()                         # greeting
        self._call({"execute": "qmp_capabilities"})

    def _recv(self) -> dict:
        line = self.file.readline()
        if not line:
            raise RuntimeError("QMP の接続が切れた")
        return json.loads(line)

    def _call(self, msg: dict) -> dict:
        self.file.write(json.dumps(msg) + "\n")
        self.file.flush()
        while True:
            reply = self._recv()
            if "return" in reply or "error" in reply:  # event は読み捨てる
                return reply

    def hmp(self, line: str) -> str:
        with self.lock:
            reply = self._call({"execute": "human-monitor-command",
                                "arguments": {"command-line": line}})
        if "error" in reply:
            raise RuntimeError(reply["error"].get("desc", "QMP error"))
        return reply["return"]


def _xp_values(text: str) -> list[int]:
    """xp の出力（"0000000020000240: 0x00 0x3f ..."）から値だけを取り出す。"""
    vals = []
    for line in text.splitlines():
        if ":" in line:
            vals += [int(v, 16) for v in re.findall(r"0x([0-9a-fA-F]+)",
                                                     line.split(":", 1)[1])]
    return vals


class QemuBackend:
    """QEMU の arduino-uno-r4 マシンで動かす."""

    name = "Renesas RA4M1 on QEMU"

    def __init__(self, elf: Path, qemu: Path, icount: str,
                 monitor_port: int, uart_port: int):
        self.elf, self.qemu, self.icount = elf, qemu, icount
        self.monitor_port, self.uart_port = monitor_port, uart_port
        self.proc: subprocess.Popen | None = None

    def start(self) -> None:
        cmd = [str(self.qemu), "-M", emulator_process.QEMU_MACHINE, "-kernel", str(self.elf),
               "-display", "none", "-monitor", "none",
               # wait=on: クライアントがつなぐまで起動を待つので、最初の出力を落とさない
               "-serial", f"tcp:127.0.0.1:{self.uart_port},server=on,wait=on",
               "-qmp", f"tcp:127.0.0.1:{self.monitor_port},server=on,wait=off"]
        if self.icount:
            # 1 命令あたりの時間を固定して、48 MHz の実機に近い速さで走らせる
            cmd += ["-icount", self.icount]
        self.proc = emulator_process.start(cmd)

    def connect(self) -> socket.socket:
        # QEMU は UART (wait=on) につながるまで起動を止めているので、UART が先
        uart = _connect(self.uart_port, "QEMU の UART")
        self.qmp = Qmp(self.monitor_port)
        return uart

    def read_bytes(self, addr: int, n: int) -> list[int]:
        return _xp_values(self.qmp.hmp(f"xp /{n}bx {hex(addr)}"))

    def read_u32(self, addr: int) -> int | None:
        vals = _xp_values(self.qmp.hmp(f"xp /1wx {hex(addr)}"))
        return vals[0] if vals else None

    def stop(self) -> None:
        emulator_process.stop(self.proc)
        self.proc = None


class Board:
    def __init__(self, elf: Path, backend):
        self.elf = elf
        self.backend = backend
        self.fb_addr = find_symbol(elf, FRAMEBUFFER_SYMBOL)
        self.state = {"matrix": [0] * 96, "led": 0, "ticks": 0}
        self.serial_log: list[str] = []
        self.subscribers: list[queue.Queue] = []
        self.sub_lock = threading.Lock()
        self.uart_sock: socket.socket | None = None
        self.stopping = False

    @property
    def proc(self) -> subprocess.Popen | None:
        return self.backend.proc

    def start(self) -> None:
        self.backend.start()
        # UART とモニタを、エミュレータが求める順につなぐ
        self.uart_sock = self.backend.connect()
        threading.Thread(target=self._uart_loop, daemon=True).start()
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def stop(self) -> None:
        self.stopping = True          # 以降のポーリング失敗は想定内
        self.backend.stop()

    # ---- シリアル ----
    def _uart_loop(self) -> None:
        if self.uart_sock is None:
            return
        # 1 バイトずつ SSE に流さないよう 40ms ぶんまとめる。
        # recv にタイムアウトを付けて、データが途切れても取りこぼさない。
        self.uart_sock.settimeout(0.04)
        pending: list[str] = []
        while True:
            try:
                data = self.uart_sock.recv(4096)
                if not data:
                    return
                pending.append(data.decode("utf-8", "replace"))
                continue
            except socket.timeout:
                pass
            except OSError:
                return
            if not pending:
                continue
            text = "".join(pending)
            pending.clear()
            self.serial_log.append(text)
            del self.serial_log[:-400]
            self._publish({"serial": text})

    def send_serial(self, text: str) -> None:
        if self.uart_sock:
            try:
                self.uart_sock.sendall(text.encode())
            except OSError:
                pass

    # ---- LED / マトリクスのポーリング ----
    def _poll_loop(self) -> None:
        while True:
            try:
                if self.fb_addr is not None:
                    vals = self.backend.read_bytes(self.fb_addr, FRAMEBUFFER_BYTES)
                    if len(vals) >= FRAMEBUFFER_BYTES:
                        self.state["matrix"] = [(vals[p // 8] >> (p % 8)) & 1
                                                for p in range(96)]
                word = self.backend.read_u32(PORT1_PCNTR1)
                if word is not None:
                    self.state["led"] = 1 if word & LED_D13_BIT else 0
            except Exception as exc:
                if self.stopping:
                    return          # 終了処理でエミュレータを止めただけ
                print(f"[poll] {type(exc).__name__}: {exc}", file=sys.stderr)
                return
            self._publish({"matrix": self.state["matrix"], "led": self.state["led"]})
            time.sleep(0.05)

    # ---- SSE ----
    def _publish(self, payload: dict) -> None:
        with self.sub_lock:
            subs = list(self.subscribers)
        for q in subs:
            try:
                q.put_nowait(payload)
            except queue.Full:
                pass

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=200)
        with self.sub_lock:
            self.subscribers.append(q)
        q.put({"matrix": self.state["matrix"], "led": self.state["led"],
               "serial": "".join(self.serial_log)})
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self.sub_lock:
            if q in self.subscribers:
                self.subscribers.remove(q)


def add_emulator_args(parser: argparse.ArgumentParser) -> None:
    """board.py と record.py で共通のエミュレータ指定。"""
    parser.add_argument("--emulator", choices=["qemu", "renode"], default="qemu",
                        help="使うエミュレータ（既定: qemu）")
    parser.add_argument("--renode", type=Path,
                        default=Path(os.environ.get("RENODE", "renode")),
                        help="renode 実行ファイル")
    parser.add_argument("--repl", default=str(HERE / "unor4_board.repl"))
    parser.add_argument("--uart", default="sci9",
                        help="(renode) Serial が出てくる SCI チャネル。"
                             "Renode の RA4M1 は R4 Minima 定義なので実機と番号が違う")
    parser.add_argument("--qemu", type=Path,
                        help="(qemu) arduino-uno-r4 マシン入りの qemu-system-arm"
                             "（既定: $QEMU → ~/qemu-unor4 → PATH の順に探す）")
    parser.add_argument("--icount", default="shift=4,sleep=on",
                        help="(qemu) -icount の値。空にするとホストの速さで走る")


def make_backend(args, elf: Path, monitor_port: int, uart_port: int):
    if args.emulator == "qemu":
        qemu, why = emulator_process.find_qemu(args.qemu)
        if qemu is None:
            raise SystemExit(why)
        return QemuBackend(elf, Path(qemu), args.icount, monitor_port, uart_port)
    return RenodeBackend(elf, args.renode, args.repl, args.uart,
                         monitor_port, uart_port)


def make_handler(board: Board, title: str):
    page = (HERE / "board.html").read_text(encoding="utf-8")
    page = page.replace("{{TITLE}}", title).replace(
        "{{EMULATOR}}", board.backend.name)

    class Handler(http.server.BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):        # アクセスログは出さない
            pass

        def _send(self, body: bytes, ctype: str):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/board.jpg":
                self._send((HERE / "board_off.jpg").read_bytes(), "image/jpeg")
            elif self.path == "/cells.json":
                self._send((HERE / "matrix_cells.json").read_bytes(),
                           "application/json")
            elif self.path == "/":
                body = page.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                q = board.subscribe()
                try:
                    while True:
                        payload = q.get()
                        chunk = f"data: {json.dumps(payload)}\n\n".encode()
                        self.wfile.write(chunk)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    board.unsubscribe(q)
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != "/input":
                self.send_error(404)
                return
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length) or b"{}")
            board.send_serial(data.get("text", ""))
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chapter", type=Path, required=True,
                        help="章のディレクトリ (例: ../04_scheduler)")
    parser.add_argument("--elf", type=Path,
                        help="ファームウェアを直接指定する (既定: 章の sim ビルド)")
    add_emulator_args(parser)
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--monitor-port", type=int, default=13456)
    parser.add_argument("--uart-port", type=int, default=13457)
    args = parser.parse_args()
    emulator_process.install_signal_handlers()

    elf = args.elf or args.chapter / ".pio/build/sim/firmware.elf"
    if not elf.exists():
        hint = f"  cd {args.chapter} && pio run -e sim"
        if (args.chapter / ".pio/build/uno_r4_wifi/firmware.elf").exists():
            hint += ("\n（実機向けのビルドはありますが、エミュレータには USB が"
                     "無いので sim 環境のビルドが要ります）")
        print(f"ファームウェアが見つかりません: {elf}\n{hint}", file=sys.stderr)
        return 1

    board = Board(elf, make_backend(args, elf, args.monitor_port, args.uart_port))
    if board.fb_addr is None:
        print("注意: LED マトリクスのフレームバッファが見つからないので、"
              "マトリクス表示は無効になります。", file=sys.stderr)
    board.start()

    class Server(http.server.ThreadingHTTPServer):
        daemon_threads = True

        def handle_error(self, request, client_address):
            exc = sys.exc_info()[1]
            if isinstance(exc, (ConnectionResetError, BrokenPipeError)):
                return              # ブラウザがタブを閉じただけ
            super().handle_error(request, client_address)

    server = Server(
        ("127.0.0.1", args.http_port),
        make_handler(board, args.chapter.resolve().name))
    print(f"仮想 UNO R4 WiFi: http://127.0.0.1:{args.http_port}")
    print(f"  firmware: {elf}")
    print(f"  emulator: {board.backend.name}")
    print("  終了は Ctrl-C")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        board.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""エミュレータ（Renode / QEMU）を子プロセスとして起動・停止する（孤児を残さない）.

Homebrew 版の ``renode`` はシェルスクリプトで、中で ``dotnet`` を**子プロセス
として**起動する（``exec`` しない）。``Popen.terminate()`` はそのシェルしか
止めないので、本体の ``dotnet`` が親を失い、エミュレーションを全速で回し
続ける（1 つで CPU 150% ほど）。

そこで Renode を新しいプロセスグループで起動し、止めるときはグループごと
止める。Windows は ``taskkill /T`` でプロセスツリーごと止める。

呼ぶ側の Python が SIGTERM / SIGHUP（端末を閉じた）で終わるときも止まるよう、
``install_signal_handlers()`` で終了処理を走らせる。``kill -9`` で殺された
場合だけは後始末のしようがないので、エミュレータが残る。

``find_qemu()`` は arduino-uno-r4 マシン入りの QEMU を探す（board.py・record.py・
検証スクリプトで共通）。
"""

from __future__ import annotations

import atexit
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

QEMU_MACHINE = "arduino-uno-r4"
# サポートサイトの付録「実機がなくても試せる」の、QEMU のインストール手順
QEMU_INSTALL_GUIDE = ("https://iory.github.io/build-your-own-arduino-rtos/"
                      "getting-started/simulator.html#install-qemu")
# QEMU の -icount。1 命令 16ns（shift=4）≈ 48 MHz の実機と同じ速さで命令を数える。
# align=on が無いと、ゲストの時計は正しくても実行は壁時計より先へ進むので、空ループで
# 待つ点滅（第3章）が実機の 10 倍以上速く見える。sleep=on はアイドル中に実時間で待つ。
QEMU_ICOUNT = "shift=4,sleep=on,align=on"
# 付録で案内している展開先。ここに置けば --qemu を書かなくて済む
QEMU_HOME = Path.home() / "qemu-unor4"

_live: set[subprocess.Popen] = set()


def _qemu_candidates(explicit: str | os.PathLike | None):
    """探す順: --qemu > $QEMU > ~/qemu-unor4 > PATH。明示されたらそれだけを見る。"""
    if explicit:
        yield Path(explicit)
        return
    if os.environ.get("QEMU"):
        yield Path(os.environ["QEMU"])
        return
    exe = "qemu-system-arm.exe" if os.name == "nt" else "qemu-system-arm"
    yield QEMU_HOME / "bin" / exe
    # Windows の zip は qemu-unor4/<配布物の名前>/bin/ に展開される
    yield from sorted(QEMU_HOME.glob(f"*/bin/{exe}"))
    found = shutil.which("qemu-system-arm")
    if found:
        yield Path(found)


def find_qemu(explicit: str | os.PathLike | None = None) -> tuple[str | None, str]:
    """arduino-uno-r4 マシン入りの qemu-system-arm を探す。

    本家の QEMU には RA4M1 が無いので、PATH にある qemu-system-arm が使えるとは
    限らない。マシン一覧を見て確かめ、見つからなければ (None, 理由) を返す。
    """
    tried: list[str] = []
    for cand in _qemu_candidates(explicit):
        if not cand.is_file():
            tried.append(f"  {cand}: ありません")
            continue
        try:
            machines = subprocess.run([str(cand), "-machine", "help"], capture_output=True,
                                      text=True, timeout=30).stdout
        except (OSError, subprocess.TimeoutExpired) as exc:
            tried.append(f"  {cand}: 実行できません（{exc}）")
            continue
        if any(line.split()[:1] == [QEMU_MACHINE] for line in machines.splitlines()):
            return str(cand), ""
        tried.append(f"  {cand}: {QEMU_MACHINE} マシンがありません（本家の QEMU）")
    why = "\n".join([
        f"{QEMU_MACHINE} マシン入りの qemu-system-arm が見つかりません。",
        *tried,
        "",
        "インストール手順（サポートサイト）:",
        f"  {QEMU_INSTALL_GUIDE}",
        f"{QEMU_HOME} に展開すれば自動で見つかります。別の場所に置いたときは",
        "--qemu か環境変数 QEMU で指定してください。",
        "Renode を使う場合は --emulator renode を付けてください。",
    ])
    return None, why


def start(cmd: list[str]) -> subprocess.Popen:
    """エミュレータを自分専用のプロセスグループで起動する。"""
    kwargs: dict = {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL}
    if os.name == "nt":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        kwargs["start_new_session"] = True     # pgid = このプロセスの pid
    proc = subprocess.Popen(cmd, **kwargs)
    _live.add(proc)
    return proc


def _signal_group(pgid: int, sig: int) -> bool:
    """プロセスグループにシグナルを送る。送れたら True。

    グループが無ければ ESRCH。macOS は、回収前の終了済みプロセス（ゾンビ）しか
    残っていないグループにも EPERM を返すので、それも「もう無い」として扱う。
    """
    try:
        os.killpg(pgid, sig)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def stop(proc: subprocess.Popen | None, timeout: float = 5.0) -> None:
    """エミュレータをプロセスグループごと止める。

    起動したシェルが先に終わっていても、グループに dotnet が残っていれば
    止める（これが孤児の正体なので、``proc.poll()`` だけで判断しない）。
    """
    if proc is None:
        return
    _live.discard(proc)

    if os.name == "nt":
        subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            proc.wait(timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
        return

    pgid = proc.pid                             # start_new_session なので pgid == pid
    _signal_group(pgid, signal.SIGTERM)
    deadline = time.time() + timeout
    while time.time() < deadline:
        proc.poll()                             # 起動したシェルを回収する（ゾンビを残さない）
        if not _signal_group(pgid, 0):          # グループが空になった
            break
        time.sleep(0.1)
    else:
        _signal_group(pgid, signal.SIGKILL)     # SIGTERM で止まらなかった
    try:
        proc.wait(1.0)
    except subprocess.TimeoutExpired:
        pass


def stop_all() -> None:
    for proc in list(_live):
        stop(proc)


atexit.register(stop_all)


def install_signal_handlers() -> None:
    """SIGTERM / SIGHUP でも atexit と finally が走るようにする。

    既定のままだと、これらのシグナルで Python は後始末をせずに終わる。
    Renode は別セッションにいるので端末の SIGHUP も届かず、残ってしまう。
    """
    # ここでエミュレータを止めずに SystemExit を投げるだけにする。呼び出し側の
    # finally（「止めている最中」の印を立ててから止める）が先に走り、
    # 取りこぼしは atexit の stop_all() が拾う。
    def _exit(signum, _frame):
        sys.exit(128 + signum)

    for name in ("SIGTERM", "SIGHUP"):
        sig = getattr(signal, name, None)
        if sig is not None and signal.getsignal(sig) in (signal.SIG_DFL, None):
            signal.signal(sig, _exit)

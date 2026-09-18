#!/usr/bin/env python3
"""Drive the quadruped from a laptop over USB. Calibration and the 50 Hz loop.

This is the recommended way to bring the robot up. A laptop on the servo bus
needs no MCU flashing, prints every intermediate value, and turns a fix-and-retry
cycle into a keystroke. arduino_quad.ino is the eventual embedded target, not the
thing to debug against first.

    macOS:   pip install feetech-servo-sdk pyserial numpy
             ls /dev/tty.usb*       # the adapter shows up as tty.usbserial-* or tty.usbmodem*
    Windows: uv run pio device list # the adapter shows up as COM3, COM4, ...

    python quad_host.py --port /dev/tty.usbserial-XXXX scan
    python quad_host.py --port ... calibrate      # writes calib.json
    python quad_host.py --port ... stand          # hold the home stance
    python quad_host.py --port ... run --vx 0.05  # walk

WHAT IS VERIFIED AND WHAT IS NOT
  quad_policy.py -- the observation assembly and the network -- is checked
  against the simulator on every export: it reproduces mjlab's own observation
  vector and joint targets to 1e-5 over a full rollout
  (scripts/check_quad_control.sh). THIS file, the serial glue, has never talked
  to a real servo. The SDK call names are the part most likely to need fixing;
  they are isolated in Bus below so a mismatch is one place to patch.

SAFETY
  Power the servos from the battery, never from the laptop's USB. Hold or
  suspend the robot for `calibrate` and the first `run`. Ctrl-C releases torque.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

if os.name == "nt":
  import msvcrt
else:
  import select
  import termios
  import tty

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quad_policy import QuadPolicy  # noqa: E402
from quad_link import MODE_HOLD_HOME, MODE_HOLD_ZERO, MODE_WALK, RobotLink  # noqa: E402
from quad_recorder import Recording, list_cameras  # noqa: E402
from servo_bus import apply_runtime_limits, open_bus  # noqa: E402

BUNDLE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")

COUNTS_PER_REV = 4096
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV
BAUD_TRY = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]
REG_MIDDLE_CALIB = 40          # write 128 -> current position becomes 2048
JOINT_SPAN_RAD = 1.2           # home +- this must stay inside 0..4095 counts

# teleop の刻みと上下限。
#
# 2026-08-22 の方策 (run turnA_rb_s203) は、書き出し環境が指令範囲を明記している
# (HANDOFF.md の「書き出し環境」の行):
#   ARDUINO_QUAD_CMD_VX_MAX=0.25  ARDUINO_QUAD_CMD_VX_BACK_FRAC=1.0
#   ARDUINO_QUAD_CMD_WZ_MAX=1.0
# つまり vx は ±0.25 m/s、wz は ±1.0 rad/s で対称。
#
# 観測正規化器の command 統計から逆算しても同じところに来る (一様分布を仮定):
#   vx  mean +0.0033 std 0.1339 -> (-0.229, +0.235)
#   wz  mean -0.0008 std 0.5093 -> (-0.883, +0.881)
# この逆算は毎回 1 割ほど内側に出るので、採るのは文書値のほう。
#
# 前の方策 (gait_g_rb) との差がそのまま実機の症状に対応している。あちらは
# BACK_FRAC=0.45 で vx の統計が (-0.125, +0.231)、wz が ±0.366 と前進側に
# 偏っていた。実機で「後ろに下がらない」「左旋回の指令でも右に回る」が出たのは
# これが理由で、今回はどちらも対称な範囲で学習し直されている。
VX_STEP, VX_MIN, VX_MAX = 0.02, -0.25, 0.25
WZ_STEP, WZ_MAX = 0.05, 1.0
# quad_policy.phase() は ||cmd|| がこれ未満だと歩容クロックを 0 に固定する。
# つまりこれを下回る指令では歩かず、その場で立つ。
GAIT_DEADBAND = 0.1
# space を押してから home を保持しきるまでの最短ステップ数 (0.02 s 刻み)。
HOLD_RAMP_STEPS = 20
# 保持姿勢へ移るときの関節速度の上限 [rad/s]。home と 0 度姿勢は股 1.03 rad /
# 膝 1.76 rad 離れているので、固定ステップ数だと 0 度へ送る瞬間に飛ぶ。
HOLD_MAX_RATE = 1.0



class Bus:
  """Thin wrapper over the Feetech SDK.

  Everything version-specific lives here. If your scservo_sdk names differ,
  this is the only class to edit.
  """

  def __init__(self, port: str, baud: int):
    # The PyPI package `feetech-servo-sdk` ships only the low level
    # PortHandler/PacketHandler; the `sms_sts` convenience class this file
    # was written against lives in Feetech's own GitHub distribution and is
    # not importable here. feetech_cli covers the same ground and its
    # register map, sign-magnitude handling and torque behaviour were
    # measured against these exact STS3215 servos, so it backs the bus.
    try:
      from feetech_cli.controller import FeetechServoController
    except ImportError as exc:
      raise SystemExit(
        "feetech_cli が見つかりません。  リポジトリ直下で uv sync してください"
      ) from exc
    # port=None makes feetech_cli sweep the USB serial adapters and keep the
    # one servos actually answer on. That is what survives a replug: the
    # device path changes, the servos do not.
    self.c = FeetechServoController(port=port, baudrate=baud, timeout=0.02)
    try:
      self.c.open(require_servo=port is None)
    except OSError as exc:
      raise SystemExit(f"ポートを開けません: {port} ({exc})") from exc
    self.port = self.c.serial.port
    self.baud = baud

  def set_baud(self, baud: int) -> None:
    self.c.reopen(baud)
    self.baud = baud
    time.sleep(0.05)

  def ping(self, sid: int) -> bool:
    return self.c.ping(sid)

  def read_pos(self, sid: int):
    from feetech_cli.protocol import FeetechError
    try:
      return self.c.read_register(sid, "present_position")
    except (FeetechError, OSError):
      return None

  def write_pos(self, sid: int, pos: int, speed: int = 0, acc: int = 0) -> None:
    self.c.sync_write_positions([sid], [int(pos)], velocity=speed,
                                acceleration=acc)

  def sync_write_pos(self, ids, poss, speed: int = 0, acc: int = 0) -> None:
    # One broadcast packet for all eight joints. Addressing them one at a
    # time costs eight round trips, which does not fit the 20 ms budget.
    self.c.sync_write_positions(ids, [int(p) for p in poss], velocity=speed,
                                acceleration=acc)

  def torque(self, sid: int, on: bool) -> None:
    self.c.set_torque(sid, bool(on))

  def write_middle(self, sid: int) -> None:
    # Vendor calibration command: 128 into Torque_Enable makes the servo
    # store its current position as the 2048 middle. feetech_cli's set_zero
    # computes homing_offset directly and lands in the same place.
    from feetech_cli.registers import CONTROL_TABLE
    self.c._write_raw(sid, CONTROL_TABLE["torque_enable"], 128,
                      expect_status=False)

  def close(self) -> None:
    self.c.close()


def find_bus(port: str, baud: int, want: int = 8):
  """Open the port; if fewer than `want` servos answer, sweep the baud rates.

  A servo left at a different factory baud is silent, not wrong, which reads
  exactly like a wiring fault.
  """
  bus = Bus(port, baud)
  ids = [i for i in range(1, 21) if bus.ping(i)]
  if len(ids) >= want:
    return bus, ids
  print(f"  {baud} bps で {len(ids)} 個。他のボーレートを試します")
  best = (len(ids), baud, ids)
  for b in BAUD_TRY:
    if b == baud:
      continue
    bus.set_baud(b)
    got = [i for i in range(1, 21) if bus.ping(i)]
    print(f"    {b:>8} bps -> {len(got)} 個 {got}")
    if len(got) > best[0]:
      best = (len(got), b, got)
  bus.set_baud(best[1])
  return bus, best[2]


# ------------------------------------------------------------------ modes --
def start_recording(args, what):
  """Open a video recording if --video was asked for, else None.

  Waits for the camera before returning: ffmpeg needs about a second to open
  a Continuity Camera, and starting the robot first loses the stand-up.
  """
  if not args.video:
    return None
  rec = Recording(args.video, camera=args.camera, fps=args.video_fps)
  print(f"  録画: {rec.path}  (カメラ {rec.camera})")
  if not rec.wait_ready():
    rec.close()
    raise SystemExit("カメラから映像が来ません。iPhone のロックを解除して"
                     "近くに置き、--camera で番号を指定してみてください")
  print(f"  カメラ準備完了。{what}を録ります。イベントは {rec.events_path}")
  return rec


def _scan_hints() -> None:
  print("\n  !! 8 個そろっていません。順に確認:")
  print("     ・サーボの電源はバッテリから(8 個 x ストール 2.7 A)。USB からは取らない")
  print("     ・半二重の方向切替をするアダプタを通しているか")
  print("     ・ID が重複していないか(重複すると衝突して両方黙る)")


def cmd_scan(args) -> int:
  # --bus は他の全サブコマンドで効くので、ここだけ黙って直結を開くと嘘の結果が
  # 出る。実際に出た: ブリッジを指定したのに直結で開き、USB アダプタが刺さって
  # いないので Arduino の CDC ポートを掴んで、そこへサーボのプロトコルを流して
  # 全ボーレートで 0 個と報告した。配線を疑う十分な理由に見えてしまう。
  if args.bus == "bridge":
    bus = open_bus("bridge", args.port, args.baud)
    print(f"  バス: {bus.description}")
    ids = bus.scan(1, 20)
    pos = bus.read_positions(ids) if ids else []
    print(f"\n{len(ids)} 個: {ids}")
    for sid, p in zip(ids, pos):
      print(f"  ID {sid:>3}  pos={p}")
    if len(ids) != 8:
      _scan_hints()
    bus.close()
    return 0

  bus, ids = find_bus(args.port, args.baud)
  print(f"\n{bus.baud} bps で {len(ids)} 個: {ids}")
  for sid in ids:
    print(f"  ID {sid:>3}  pos={bus.read_pos(sid)}")
  if len(ids) != 8:
    _scan_hints()
  if bus.baud != 1000000:
    print(f"\n  !! 1 Mbps ではありません({bus.baud})。50 Hz 制御には 500 kbps 以上が要ります")
  bus.close()
  return 0


def cmd_calibrate(args) -> int:
  if args.bus == "bridge":
    # 下の手順は Bus (直結) のメソッドを直接使う。ブリッジで走らせると
    # --bus を無視して直結を開き、無関係なポートを掴む。
    print("  calibrate は直結のみ対応です。USB アダプタを挿して --bus direct で")
    return 1
  pol = QuadPolicy(BUNDLE)
  names = pol.joint_names
  bus, ids = find_bus(args.port, args.baud)
  if len(ids) < 8:
    print("8 個そろってから実行してください")
    bus.close()
    return 1

  print("\n=== 1. ID と関節の対応 ===")
  print("  ロボットを浮かせてください。1 個ずつ小さく動かします。")
  input("  準備できたら Enter > ")
  mapping = {}
  for sid in ids:
    p0 = bus.read_pos(sid)
    if p0 is None:
      continue
    print(f"\n  --- ID {sid} を動かします ---")
    bus.torque(sid, True)
    for _ in range(3):
      bus.write_pos(sid, min(p0 + 120, 4095), 300, 50); time.sleep(0.45)
      bus.write_pos(sid, max(p0 - 120, 0), 300, 50); time.sleep(0.45)
    bus.write_pos(sid, p0, 300, 50); time.sleep(0.4)
    bus.torque(sid, False)
    for j, n in enumerate(names):
      print(f"    {j} = {n}")
    a = input("  動いた関節の番号 (0-7, 分からなければ空 Enter) > ").strip()
    if a.isdigit() and 0 <= int(a) < 8:
      mapping[int(a)] = sid
      print(f"  -> {names[int(a)]} = ID {sid}")
  missing = [names[j] for j in range(8) if j not in mapping]
  if missing:
    print(f"  !! 未割当: {missing}")
    bus.close()
    return 1

  print("\n=== 2. ゼロ点 (q = 0) ===")
  print("  トルクを切ります。4 本すべてを次の姿勢にしてください:")
  print("    ・大腿(股関節軸 -> 膝軸)が鉛直")
  print("    ・そのとき足先は膝の真下から 8 mm ほど前")
  print("  数度ずれても構いません(学習でゼロ点 ±1.7°、関節角 ±8.6° をランダム化済み)")
  for sid in ids:
    bus.torque(sid, False)
  input("  姿勢を作ったら Enter > ")
  zero = {}
  for j in range(8):
    zero[j] = bus.read_pos(mapping[j])
    print(f"  {names[j]:<10} ID {mapping[j]:>2}  zero={zero[j]}")

  print("\n=== 3. 符号 ===")
  print("  約束: どの関節も「正 = 足先が後ろ(尾側)へ振れる」")
  sign = {}
  for j in range(8):
    input(f"  {names[j]}: 足先を後ろへ振って保持し Enter > ")
    p = bus.read_pos(mapping[j])
    d = p - zero[j]
    if abs(d) < 60:
      print(f"    !! 変化が {d} counts と小さすぎます。+1 と仮定します")
      sign[j] = 1
    else:
      sign[j] = 1 if d > 0 else -1
    print(f"    delta={d} -> sign={sign[j]:+d}")

  print("\n=== 4. 可動範囲 ===")
  ok = True
  for j in range(8):
    home = float(pol.default_q[j])
    c = [round(zero[j] + sign[j] * (home + s) / RAD_PER_COUNT)
         for s in (-JOINT_SPAN_RAD, JOINT_SPAN_RAD)]
    lo, hi = min(c), max(c)
    bad = lo < 0 or hi > 4095
    ok &= not bad
    print(f"  {names[j]:<10} counts {lo} .. {hi} {'!! 範囲外' if bad else 'ok'}")
  if not ok:
    print("  ホーンを 90° ずらして組み直すか、EEPROM 中点校正(--write-middle)を使ってください")

  calib = {"port": args.port, "baud": bus.baud,
           "joint_names": names,
           "id": [mapping[j] for j in range(8)],
           "sign": [sign[j] for j in range(8)],
           "zero": [int(zero[j]) for j in range(8)]}
  with open(CALIB, "w") as f:
    json.dump(calib, f, indent=2)
  print(f"\n  書き出し: {CALIB}")

  if args.write_middle:
    print("\n=== 5. ゼロ点を EEPROM に書く ===")
    print("  いまの姿勢を中点(2048)として記録します。q = 0 の姿勢のままにしてください。")
    if input("  実行する? (yes/no) > ").strip().lower() == "yes":
      for sid in ids:
        bus.torque(sid, False)
      time.sleep(0.1)
      for j in range(8):
        bus.write_middle(mapping[j])
        time.sleep(0.06)
        after = bus.read_pos(mapping[j])
        print(f"  {names[j]:<10} -> {after} {'ok' if abs(after - 2048) < 40 else '!! 2048 でない'}")
      calib["zero"] = [2048] * 8
      with open(CALIB, "w") as f:
        json.dump(calib, f, indent=2)
      print("  calib.json の zero を 2048 に更新しました")
  bus.close()
  return 0


def resolve_port(requested, recorded):
  """Pick the serial device to talk to.

  An explicit --port wins. Otherwise the path recorded at calibration time is
  used, but only if it still exists: macOS hands out a new /dev/cu.usbmodem*
  after a replug, and a stale path fails with a confusing "no such file".
  When it is gone, None is returned, which makes the bus hunt for the adapter
  that servos actually answer on.
  """
  if requested:
    return requested
  if recorded:
    import serial.tools.list_ports
    if any(p.device == recorded for p in serial.tools.list_ports.comports()):
      return recorded
    print(f"  {recorded} は今は見当たりません。USB を探索します")
  return None


class Robot:
  """Calibrated servo bus in joint space."""

  def __init__(self, port, baud, bus_kind="direct"):
    self.bus_kind = bus_kind
    if not os.path.exists(CALIB):
      raise SystemExit(f"{CALIB} がありません。先に calibrate を実行してください")
    with open(CALIB) as f:
      c = json.load(f)
    self.ids = c["id"]
    self.sign = np.array(c["sign"], np.float32)
    self.zero = np.array(c["zero"], np.float32)
    kind = getattr(self, "bus_kind", "direct")
    resolved = None if kind == "bridge" else resolve_port(port, c.get("port"))
    self.bus = open_bus(kind, port if kind == "bridge" else resolved,
                        baud or c["baud"])
    print(f"  バス: {self.bus.description}")
    # These live in SRAM and a power cycle puts them back to the slow factory
    # values, so they are set on every connection rather than once by hand.
    runtime = c.get("servo_runtime")
    if runtime:
      ok, _ = apply_runtime_limits(self.bus, self.ids, runtime)
      print("  サーボ速度設定: max_acceleration=%s accel_multiplier=%s %s" % (
        runtime.get("max_acceleration"), runtime.get("accel_multiplier"),
        "適用" if ok else "!! 一部失敗"))
    self.names = c["joint_names"]

  def read_q(self):
    """Measured joint angles [rad], or None if any servo went quiet.

    One batched call: the bridge turns this into a single USB exchange.
    """
    pos = self.bus.read_positions(self.ids)
    if any(p is None for p in pos):
      return None
    return (np.array(pos, np.float32) - self.zero) * RAD_PER_COUNT * self.sign

  def read_joint(self, j):
    """One joint's angle [rad], in a single bus round trip.

    read_q costs eight round trips, 6.4 ms on this bus. That is fine for the
    control loop and wrong for identification: the servo's own dead time
    between a position command and the shaft moving is 23 ms (measured), so
    seven unrelated reads are a third of the quantity being measured, added to
    the timestamp of the one that matters.
    """
    pos = self.bus.read_positions([self.ids[j]])[0]
    if pos is None:
      return None
    return float((pos - self.zero[j]) * RAD_PER_COUNT * self.sign[j])

  def write_q(self, q):
    counts = np.clip(np.asarray(q, np.float32) / RAD_PER_COUNT * self.sign
                     + self.zero, 0, 4095).astype(int)
    self.bus.sync_write_positions(self.ids, counts, 0, 0)

  def step_q(self, q_target):
    """Command joint angles and read the joints back, in one bus operation.

    The control loop's inner call. Ordering it as write-then-read rather than
    read-then-write costs nothing in behaviour -- the reading is simply taken
    just after the command instead of just before -- and lets a transport that
    can batch the two pay its latency once.

    Returns None if any servo went quiet, same as read_q.
    """
    counts = np.clip(np.asarray(q_target, np.float32) / RAD_PER_COUNT
                     * self.sign + self.zero, 0, 4095).astype(int)
    pos = self.bus.step(self.ids, counts, 0, 0)
    if any(p is None for p in pos):
      return None
    return (np.array(pos, np.float32) - self.zero) * RAD_PER_COUNT * self.sign

  def torque(self, on):
    self.bus.torque(self.ids, on)

  def ramp_to(self, q_target, seconds=2.0, dt=0.02):
    q0 = self.read_q()
    if q0 is None:
      raise SystemExit("サーボが応答しません")
    self.torque(True)
    n = max(int(seconds / dt), 1)
    for k in range(n + 1):
      self.write_q(q0 + (k / n) * (np.asarray(q_target, np.float32) - q0))
      time.sleep(dt)


def cmd_stand(args) -> int:
  pol = QuadPolicy(BUNDLE)
  r = Robot(args.port, args.baud, args.bus)
  print("  home 姿勢へ 2 秒かけて移動します。4 本とも膝が後ろに引けていれば正解です。")
  r.ramp_to(pol.default_q, 2.0)
  print("  保持中。Ctrl-C で解放。")
  try:
    while True:
      time.sleep(0.2)
  except KeyboardInterrupt:
    pass
  r.torque(False)
  r.bus.close()
  return 0


def cmd_run(args) -> int:
  pol = QuadPolicy(BUNDLE)
  r = Robot(args.port, args.baud, args.bus)
  dt = pol.dt

  rec = start_recording(args, "歩行")

  print("  home 姿勢へ移動します")
  r.ramp_to(pol.default_q, 2.0)
  if args.yes:
    print(f"  指令 vx={args.vx} wz={args.wz} で歩き始めます (--yes)")
  else:
    input(f"  指令 vx={args.vx} wz={args.wz} で歩き始めます。Enter で開始 (Ctrl-C で停止) > ")

  released = {"v": False}

  def release(*_):
    if not released["v"]:
      released["v"] = True
      try:
        r.write_q(pol.default_q)
        time.sleep(0.2)
        r.torque(False)
      finally:
        r.bus.close()
        if rec is not None:
          # After the servos are released, not before: the last thing the
          # video should show is the robot actually being let go.
          print(f"\n  録画を閉じています: {rec.close()}")
      print("\n  解放しました")
    sys.exit(0)

  signal.signal(signal.SIGINT, release)

  # Log every step when asked. On hardware the only thing you can see is that
  # the robot "does not turn right"; the joints are where the reason lives, and
  # the same command replayed in sim gives an exact reference to diff against
  # (scripts/quad_compare_hw_log.py).
  log_f = None
  if args.log:
    log_f = open(args.log, "w", encoding="utf-8")
    log_f.write("t,cmd_vx,cmd_wz," + ",".join(
      f"q_{n}" for n in pol.joint_names) + "," + ",".join(
      f"qd_{n}" for n in pol.joint_names) + "," + ",".join(
      f"tgt_{n}" for n in pol.joint_names) + "\n")

  pol.reset()
  q_prev = r.read_q()
  qd = np.zeros(8, np.float32)
  cmd = np.array([args.vx, 0.0, args.wz], np.float32)
  t_next = time.perf_counter()
  late_max = 0.0
  for i in range(int(args.seconds / dt)):
    t_next += dt
    t0 = time.perf_counter()
    q = r.read_q()
    if q is None:
      print("  !! サーボが応答しませんでした。保持します")
      continue
    # Finite difference, one-pole filtered. The servo also reports a speed, but
    # its units and filtering are undocumented enough that differencing the
    # position is the more predictable of the two.
    qd = 0.6 * qd + 0.4 * ((q - q_prev) / dt)
    q_prev = q
    tgt, _ = pol.step(q, qd, cmd, step_i=i)
    r.write_q(tgt)
    if rec is not None:
      rec.log(cmd_vx=cmd[0], cmd_wz=cmd[2],
              **{f"q_{n}": "%.5f" % v for n, v in zip(pol.joint_names, q)},
              **{f"tgt_{n}": "%.5f" % v for n, v in zip(pol.joint_names, tgt)})
    if log_f is not None:
      log_f.write("%.4f,%.4f,%.4f," % (i * dt, cmd[0], cmd[2])
                  + ",".join("%.5f" % v for v in q) + ","
                  + ",".join("%.5f" % v for v in qd) + ","
                  + ",".join("%.5f" % v for v in tgt) + "\n")
    t1 = time.perf_counter()
    late = t1 - t_next
    late_max = max(late_max, late)
    if i % int(1 / dt) == 0:
      print(f"  t={i * dt:5.1f}s  loop={1e3 * (t1 - t0):5.1f} ms  "
            f"late={1e3 * late:+5.1f} ms (max {1e3 * late_max:+5.1f})  "
            f"|q-home| max={np.abs(q - pol.default_q).max():.3f} rad")
    sleep = t_next - time.perf_counter()
    if sleep > 0:
      time.sleep(sleep)
  if log_f is not None:
    log_f.close()
  if rec is not None:
    print(f"\n  録画: {rec.close()}")
    print(f"  ログ: {args.log}")
  release()
  return 0



def cmd_stepid(args) -> int:
  """Record one joint's step response, for identifying the servo model.

  Everything downstream of the simulator is verified; the simulator's own servo
  model is not. stiffness/damping/armature/frictionloss in robot_cfg.py are all
  marked ESTIMATE, and they decide whether a policy that walks in sim walks on
  the floor. This is the measurement that replaces the guesses.

  The robot must hang with the legs free -- a step against the ground measures
  the floor, not the servo. Only the joint under test is driven; the rest hold
  the home pose so the leg does not flop into the frame.

  Three things about how the log is taken are not cosmetic, and an earlier
  version of this function got all three wrong:

  * Only the joint under test is read back. read_q costs 6.4 ms, and the
    quantity being identified -- the servo's 23 ms dead time between the
    position command and the shaft moving -- is only four of those samples
    wide. A single-joint read is 0.7 ms.
  * The row is timestamped when its reading came back, not after eight
    unrelated reads finished. Timestamping late inflated the measured dead
    time from 23 ms to 35 ms.
  * The amplitudes straddle the actuator's force limit. MJCF forcerange is
    +-2.942 N.m, so at stiffness 20 the actuator saturates past 0.147 rad of
    error and the stiffness stops showing up in the response at all. Fitting a
    single amplitude is not merely imprecise: on the +-0.062 rad block alone
    the search prefers stiffness 2.6 with frictionloss 0.085, which beats the
    right answer by 2x on that block and loses by 4-10x on every other one.
  """
  pol = QuadPolicy(BUNDLE)
  r = Robot(args.port, args.baud, args.bus)
  names = pol.joint_names
  if args.joint in names:
    j = names.index(args.joint)
  else:
    try:
      j = int(args.joint)
    except ValueError:
      print(f"  関節名が不明: {args.joint}\n  候補: {', '.join(names)}")
      return 1
  home = pol.default_q.copy()
  print(f"  対象: {names[j]}  基準 振幅 ±{args.amp:.2f} rad / 周期 "
        f"{args.period:.1f} s × {args.cycles} 回。これを振幅と速さを変えた "
        f"7 条件で回します")
  print("  脚が自由に動く状態で吊るしてください。ぶつけると値が意味を失います。")
  input("  準備できたら Enter: ")

  r.ramp_to(home, 2.0)
  time.sleep(0.5)

  # Several amplitudes and several rates, not one square wave. Two separate
  # reasons, and the first one is fatal on its own:
  #
  #   * the position actuator's force limit. Past forcerange/stiffness of
  #     error the torque is pinned and the stiffness has no effect on the
  #     trajectory, so a log made only of large steps constrains the force
  #     limit and not the gain. Only steps small enough to stay off the limit
  #     -- amp/6 here -- carry the stiffness.
  #   * a fast rise is where reflected inertia (armature) matters, while dry
  #     friction shows up in slow motion and at every direction reversal.
  #
  # Fitting one amplitude recovers a friction-dominated servo that replays
  # badly at every other amplitude. Measured, on this robot.
  blocks = [(args.amp, args.period),              # force-limited: sets the cap
            (args.amp, args.period / 3.0),        # fast: excites inertia
            (args.amp / 2.0, args.period),
            (args.amp / 3.0, args.period),
            (args.amp / 6.0, args.period),        # never saturates: stiffness
            (args.amp / 3.0, args.period / 3.0),
            (args.amp, args.period * 2.5)]        # slow: creep and stiction
  rows = []
  t0 = time.perf_counter()
  tgt = home.copy()
  for amp, period in blocks:
    print(f"    振幅 ±{amp:.3f} rad / 周期 {period:.2f} s")
    tb = time.perf_counter()
    while time.perf_counter() - tb < period * args.cycles:
      t = time.perf_counter() - tb
      hi = (t % period) < (period / 2.0)
      tgt[j] = home[j] + (amp if hi else -amp)
      r.write_q(tgt)
      q = r.read_joint(j)
      if q is None:
        continue
      # The clock is read here, right after this joint's reading came back,
      # and the command it is a response to went out 0.2 ms earlier.
      rows.append((time.perf_counter() - t0, tgt[j], q))

  # Free swing, torque off. Driven logs cannot separate reflected inertia from
  # dry friction -- the servo's own position loop dominates the response, and
  # varying amplitude and rate did not help (measured, on synthetic logs with
  # known answers). With the torque disabled that loop is gone and the leg is a
  # pendulum whose decay is governed by exactly those two.
  if args.swing:
    print("    自由振動 (トルクオフ)")
    tgt[j] = home[j] + args.amp * 1.5
    r.write_q(tgt)
    time.sleep(1.0)
    t_sw = time.perf_counter() - t0
    r.torque(False)
    while time.perf_counter() - t0 - t_sw < args.swing_s:
      q = r.read_joint(j)
      if q is None:
        continue
      # target = NaN marks "no command": the fit must not treat these samples
      # as a position loop tracking something.
      rows.append((time.perf_counter() - t0, float("nan"), q))
    r.torque(True)

  r.write_q(home)
  time.sleep(0.5)
  r.torque(False)
  with open(args.log, "w", encoding="utf-8") as f:
    f.write("t,target,measured\n")
    for t, tg, q in rows:
      f.write("%.5f,%.6f,%.6f\n" % (t, tg, q))
  dt = [rows[i + 1][0] - rows[i][0] for i in range(len(rows) - 1)]
  dt.sort()
  print(f"  {len(rows)} 点を {args.log} に記録 "
        f"(サンプル間隔 中央値 {1e3 * dt[len(dt) // 2]:.1f} ms, "
        f"最大 {1e3 * dt[-1]:.1f} ms)")
  print(f"  同定: ./scripts/fit_servo_model.sh {args.log} {names[j]}")
  return 0


class _PosixKeyReader:
  """Single key presses from the terminal, without waiting for Enter.

  Uses cbreak rather than raw mode on purpose: cbreak leaves ISIG alone, so
  Ctrl-C still raises KeyboardInterrupt and the torque release handler runs.
  In raw mode a panicking operator would have no way to stop the robot.
  """

  def __enter__(self):
    if not sys.stdin.isatty():
      raise SystemExit("teleop は端末から実行してください (キー入力を読みます)")
    self.fd = sys.stdin.fileno()
    self.saved = termios.tcgetattr(self.fd)
    tty.setcbreak(self.fd)
    return self

  def __exit__(self, *exc) -> bool:
    termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
    return False

  def keys(self):
    """Return every key pressed since the last call, oldest first."""
    out = []
    while select.select([sys.stdin], [], [], 0)[0]:
      ch = os.read(self.fd, 1).decode("utf-8", "ignore")
      if not ch:
        break
      out.append(ch)
    return out


class _WindowsKeyReader:
  """The same, for the Windows console, where there is no termios.

  Nothing has to be put back on exit: msvcrt reads the console buffer directly
  instead of changing a line discipline. Ctrl-C keeps working for the same
  reason cbreak is used on POSIX -- the console leaves ENABLE_PROCESSED_INPUT
  on, so Ctrl-C never reaches the input buffer and arrives as a
  KeyboardInterrupt instead, which is what releases torque.
  https://docs.python.org/3/library/msvcrt.html
  """

  def __enter__(self):
    if not sys.stdin.isatty():
      raise SystemExit("teleop は端末から実行してください (キー入力を読みます)")
    return self

  def __exit__(self, *exc) -> bool:
    return False

  def keys(self):
    """Return every key pressed since the last call, oldest first."""
    out = []
    while msvcrt.kbhit():
      ch = msvcrt.getwch()
      if ch in ("\x00", "\xe0"):
        # Arrow and function keys arrive as a two-character sequence. Drop the
        # second half rather than let it read as a movement command.
        msvcrt.getwch()
        continue
      out.append(ch)
    return out


KeyReader = _WindowsKeyReader if os.name == "nt" else _PosixKeyReader


def apply_key(key: str, vx: float, wz: float, limits=None):
  """Fold one key press into the command.

  `limits` is (vx_min, vx_max, wz_max); it defaults to the trained range.
  Returns (vx, wz, quit_requested, hold). `hold` is "home" or "zero" when the
  operator asked to stop in that pose, False when a movement key should resume
  walking, and None when the key changes neither. Unknown keys change nothing.
  """
  vx_min, vx_max, wz_max = limits or (VX_MIN, VX_MAX, WZ_MAX)
  if key in ("w", "W"):
    return min(vx + VX_STEP, vx_max), wz, False, False
  if key in ("s", "S"):
    return max(vx - VX_STEP, vx_min), wz, False, False
  if key in ("a", "A"):
    return vx, min(wz + WZ_STEP, wz_max), False, False
  if key in ("d", "D"):
    return vx, max(wz - WZ_STEP, -wz_max), False, False
  if key == " ":
    return 0.0, 0.0, False, "home"
  if key == "0":
    # Every joint to q = 0, which is servo count 2048 -- the pose the zero
    # calibration was written against. Useful for checking the zero and the
    # mechanical build; it is not the walking pose.
    return 0.0, 0.0, False, "zero"
  if key in ("x", "X", "q", "Q", "\x03"):
    return 0.0, 0.0, True, None
  return vx, wz, False, None


def hold_target(goal, hold_from, k: int, steps: int = HOLD_RAMP_STEPS):
  """Interpolate from the pose we stopped in toward `goal`.

  A stop must not depend on the policy settling by itself. Commanding zero
  velocity leaves the network running, and on the robot that was observed to
  keep marching in place. This bypasses the network instead: it walks the
  joint targets to `goal` and holds them there.
  """
  a = 1.0 if steps <= 0 else min(1.0, k / float(steps))
  return hold_from + a * (np.asarray(goal, np.float32) - hold_from)


def hold_steps_for(goal, hold_from, dt: float) -> int:
  """How many control steps a hold ramp needs to stay under HOLD_MAX_RATE.

  Fixed step counts are fine between nearby poses, but home and the zero pose
  are over 1.7 rad apart at the knees; ramping that in 0.4 s would throw the
  legs. Scaling with the distance keeps every transition at the same speed.
  """
  span = float(np.abs(np.asarray(goal, np.float32) - hold_from).max())
  return max(HOLD_RAMP_STEPS, int(math.ceil(span / (HOLD_MAX_RATE * dt))))


def cmd_teleop(args) -> int:
  """Walk the robot from the keyboard, ROS teleop_twist_keyboard style."""
  pol = QuadPolicy(BUNDLE)
  r = Robot(args.port, args.baud, args.bus)
  dt = pol.dt

  limits = (args.vx_min, args.vx_max, args.wz_max)

  beyond = (args.vx_min < VX_MIN or args.vx_max > VX_MAX
            or args.wz_max > WZ_MAX)

  print("  home 姿勢へ移動します")
  r.ramp_to(pol.default_q, 2.0)

  print()
  print(f"  w / s   前進速度 vx  +-{VX_STEP:.2f} m/s "
        f"({args.vx_min:+.2f} .. {args.vx_max:+.2f})")
  print(f"  a / d   旋回速度 wz  +-{WZ_STEP:.2f} rad/s "
        f"(+-{args.wz_max:.2f})")
  print("  space   停止。方策を切り離して home 姿勢を保持する")
  print("  0       全関節を 0 度へ (サーボ 2048 カウント、ゼロ校正の基準姿勢)。")
  print("          home から股 59 度 / 膝 101 度 動きます。吊るして使うこと")
  print("  r       録画の開始 / 停止 (もう一度押すと止まります)")
  print("  x       停止して脱力し終了 (Ctrl-C でも同じ)")
  print()
  print("  vy は常に 0 です。外転関節が無いので横移動はできません。")
  print(f"  ||cmd|| < {GAIT_DEADBAND:.1f} では歩容クロックの入力が 0 になります。"
        "歩くのは止まりませんが、")
  print("  クロックに同期せず自走周波数 (4.0-4.5 Hz) になります。")
  print(f"  既定の指令範囲は vx {VX_MIN:+.2f} .. {VX_MAX:+.2f} / "
        f"wz +-{WZ_MAX:.2f}")
  print("  (文書に記載が無いため観測正規化器の統計から逆算した推定値です)")
  if beyond:
    print("  !! 指定された範囲は学習範囲の外に出ます。分布外なので挙動の保証は")
    print("     ありません。オフラインで回すと関節目標の振幅が学習上限の 2 倍を")
    print("     超えます。吊るした状態で少しずつ試してください。")
  input("  Enter で開始 > ")

  released = {"v": False}
  session = RecordSession(args.record_dir, args.camera, args.video_fps,
                          with_video=not args.no_camera,
                          first_path=args.video)
  if args.video:
    print(f"  {session.toggle()}")

  def release(*_):
    if released["v"]:
      return
    released["v"] = True
    try:
      r.write_q(pol.default_q)
      time.sleep(0.2)
      r.torque(False)
    finally:
      r.bus.close()
      out = session.stop()
      if out:
        print(f"\n  記録終了 {out}")
    print("\n  解放しました")

  signal.signal(signal.SIGINT, lambda *_: (release(), sys.exit(0)))

  pol.reset()
  q_prev = r.read_q()
  if q_prev is None:
    release()
    raise SystemExit("サーボが応答しません")
  qd = np.zeros(8, np.float32)
  q = q_prev.copy()
  last_target = pol.default_q.copy()
  vx, wz = 0.0, 0.0
  # Start held: the policy only runs once a movement key is pressed.
  holding, hold_from, hold_k = True, q_prev.copy(), HOLD_RAMP_STEPS
  hold_goal, hold_steps, hold_name = pol.default_q.copy(), HOLD_RAMP_STEPS, "home"
  zero_q = np.zeros(8, np.float32)
  t_next = time.perf_counter()
  late_max = 0.0
  loop_ms = loop_max = 0.0
  i = 0
  try:
    with KeyReader() as keyboard:
      while True:
        t_next += dt
        t0 = time.perf_counter()
        for key in keyboard.keys():
          if key in ("r", "R"):
            print(f"\n  {session.toggle()}")
            continue
          vx, wz, quit_requested, hold = apply_key(key, vx, wz, limits)
          if quit_requested:
            raise KeyboardInterrupt
          if hold in ("home", "zero"):
            hold_goal = zero_q if hold == "zero" else pol.default_q.copy()
            hold_name = hold
            hold_from = last_target.copy()
            hold_steps = hold_steps_for(hold_goal, hold_from, dt)
            hold_k, holding = 0, True
          elif hold is False and holding:
            # Resume from wherever the joints actually are, with a clean
            # history: the observation carries three past steps of walking
            # and reusing them would restart mid stride.
            holding = False
            pol.reset()

        if holding:
          cmd = np.zeros(3, np.float32)
          tgt = hold_target(hold_goal, hold_from, hold_k, hold_steps)
          hold_k += 1
        else:
          cmd = np.array([vx, 0.0, wz], np.float32)
          tgt, _ = pol.step(q, qd, cmd, step_i=i)
        last_target = np.asarray(tgt, np.float32)

        q_new = r.step_q(tgt)
        if q_new is None:
          print("\n  !! サーボが応答しませんでした。保持します")
          continue
        qd = 0.6 * qd + 0.4 * ((q_new - q) / dt)
        q_prev = q
        q = q_new
        session.log(cmd_vx=cmd[0], cmd_wz=cmd[2],
                    hold=1 if holding else 0,
                    **{f"q_{n}": "%.5f" % v for n, v in zip(pol.joint_names, q)},
                    **{f"tgt_{n}": "%.5f" % v
                       for n, v in zip(pol.joint_names, last_target)})
        loop_ms = 1e3 * (time.perf_counter() - t0)
        loop_max = max(loop_max, loop_ms)

        late = time.perf_counter() - t_next
        late_max = max(late_max, late)
        if i % 10 == 0:
          if holding:
            where = "0度" if hold_name == "zero" else "home"
            state = (f"停止({where}保持)" if hold_k > hold_steps
                     else f"{where}へ移行中 ")
          elif float(np.linalg.norm(cmd)) >= GAIT_DEADBAND:
            state = "歩行/同期    "
          else:
            state = "歩行/自走    "
          status_line(
            "  vx={:+.2f} m/s  wz={:+.2f} rad/s  {}  "
            "loop={:5.1f} ms (max {:5.1f})  "
            "late={:+5.1f} ms (max {:+5.1f}){}".format(
              vx, wz, state, loop_ms, loop_max,
              1e3 * late, 1e3 * late_max,
              "  [REC]" if session.active else ""))
        i += 1
        sleep = t_next - time.perf_counter()
        if sleep > 0:
          time.sleep(sleep)
  except KeyboardInterrupt:
    pass
  finally:
    release()
  return 0


class RecordSession:
  """Start and stop recording from a key press, naming the files itself.

  Recording is a thing an operator decides to do in the middle of driving --
  "that was interesting, do it again with the camera on" -- so it belongs on a
  key, not on a command line flag chosen a minute earlier.

  Every session always writes the event log. The video is optional: with
  ``camera=False`` the filming happens on the phone itself, and the log's
  ``t_unix`` column is what lines the two up afterwards.
  """

  def __init__(self, directory, camera=None, fps=30, with_video=True,
               first_path=None):
    self.directory = directory
    self.camera = camera
    self.fps = fps
    self.with_video = with_video
    #: Where the first recording goes when the operator named one on the
    #: command line. Later ones are named by the clock.
    self.first_path = first_path
    self.rec = None

  @property
  def active(self):
    return self.rec is not None

  def toggle(self):
    """Start if stopped, stop if started. Returns a line to show the operator."""
    if self.rec is None:
      if self.first_path:
        path, self.first_path = self.first_path, None
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
      else:
        os.makedirs(self.directory, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        path = os.path.join(self.directory, f"run-{stamp}.mp4")
      try:
        self.rec = Recording(path, camera=(self.camera if self.with_video
                                           else False), fps=self.fps)
      except SystemExit as exc:
        # A missing camera must not take the robot down mid-run.
        return f"!! 録画を開始できません: {exc}"
      if self.with_video and not self.rec.wait_ready(timeout=8.0):
        self.rec.close()
        self.rec = None
        return "!! カメラから映像が来ません。--no-camera なら記録だけ取れます"
      return ("記録開始 " + (self.rec.path if self.with_video
                             else self.rec.events_path))
    out = self.rec.close()
    self.rec = None
    return f"記録終了 {out}"

  def log(self, **fields):
    if self.rec is not None:
      self.rec.log(**fields)

  def stop(self):
    if self.rec is not None:
      out = self.rec.close()
      self.rec = None
      return out
    return None


def status_line(text):
  """Rewrite the single status line in place, clearing whatever was longer.

  A '\r' only overwrites as many columns as the new line occupies, so a
  shorter line leaves the tail of the previous one on screen -- which is how
  "skipped=0  [REC]    絶  [REC]" happened. Padding with spaces cannot fix it
  reliably because these lines mix half and full width characters. The erase
  sequence is exact.
  """
  sys.stdout.write("\r" + text + "\x1b[K")
  sys.stdout.flush()


def _state_fields(state):
  """The robot's own loop numbers, carried forward between replies.

  The robot answers a few times a second, not every send, so writing only the
  fresh ones leaves most rows blank and the plotted trace becomes dots. The
  last known value is what was true until the next reply says otherwise.
  """
  state = state or {}
  return {"loop_us": state.get("loop_us", ""),
          "loop_max_us": state.get("loop_max_us", ""),
          "skipped": state.get("skipped", "")}


def _wifi_scripted(link, args) -> int:
  """One fixed command for `--seconds`, then home. No terminal needed.

  teleop is the right interface for a person, but it needs a tty, and a
  scripted run is what a check should be: the same command every time, for a
  stated number of seconds, with the robot's own loop numbers printed so the
  result is evidence rather than an impression.
  """
  print(f"  指令 vx={args.vx:+.2f} wz={args.wz:+.2f} を {args.seconds:.0f} 秒 (--yes)")
  rec = start_recording(args, "歩行")
  worst = {"loop": 0, "skipped": 0}
  last_state = None
  try:
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
      state = link.send_if_changed(args.vx, args.wz, MODE_WALK)
      if state:
        last_state = state
      if rec is not None:
        rec.log(cmd_vx=args.vx, cmd_wz=args.wz, mode=MODE_WALK,
                **_state_fields(last_state))
      if state:
        worst["loop"] = max(worst["loop"], state["loop_max_us"])
        worst["skipped"] = state["skipped"]
        print("\r  残り {:4.1f}s  loop={:5d} us (max {:5d})  skipped={}   "
              .format(t_end - time.monotonic(), state["loop_us"],
                      state["loop_max_us"], state["skipped"]), end="")
        sys.stdout.flush()
      time.sleep(0.05)
  except KeyboardInterrupt:
    pass
  finally:
    # Stop first, then let the caller close: a dropped link would take 300 ms
    # to be noticed by the watchdog, and this is the same stop, immediately.
    for _ in range(5):
      link.send(0.0, 0.0, MODE_HOLD_HOME)
      if rec is not None:
        rec.log(cmd_vx=0.0, cmd_wz=0.0, mode=MODE_HOLD_HOME,
                **_state_fields(last_state))
      time.sleep(0.05)
    link.close()
    if rec is not None:
      print(f"\n  録画: {rec.close()}")
  print(f"\n  home 保持へ戻しました。loop 最大 {worst['loop']} us, "
        f"skipped {worst['skipped']}")
  return 0


def cmd_wifi(args) -> int:
  """Drive the robot over WiFi, with the policy running on the MCU.

  The key handling is the same as teleop's -- same steps, same limits, same
  stop -- because only where the split falls has changed. Here the host sends
  a velocity and the robot closes the loop; over USB the host closed it.
  """
  link = RobotLink(args.host, args.udp_port)
  print(f"  ロボット: {link.host}:{link.port}")
  print()
  print(f"  w / s   前進速度 vx  +-{VX_STEP:.2f} m/s "
        f"({args.vx_min:+.2f} .. {args.vx_max:+.2f})")
  print(f"  a / d   旋回速度 wz  +-{WZ_STEP:.2f} rad/s (+-{args.wz_max:.2f})")
  print("  space   停止して home 姿勢を保持")
  print("  0       全関節を 0 度へ (ゼロ校正の基準姿勢。吊るして使うこと)")
  print("  r       録画の開始 / 停止 (もう一度押すと止まります)")
  print("  x       停止して終了 (Ctrl-C でも同じ)")
  print()
  print("  指令が 300 ms 途切れるとロボット側が自動で home 保持に入ります。")
  if args.yes:
    return _wifi_scripted(link, args)
  input("  Enter で開始 > ")

  limits = (args.vx_min, args.vx_max, args.wz_max)
  session = RecordSession(args.record_dir, args.camera, args.video_fps,
                          with_video=not args.no_camera,
                          first_path=args.video)
  if args.video:
    # --video means "record the whole session"; the r key is for deciding
    # part way through. Both end up in the same place.
    print(f"  {session.toggle()}")
  last_state = None
  vx, wz, mode = 0.0, 0.0, MODE_HOLD_HOME
  last_print = 0.0
  try:
    with KeyReader() as keyboard:
      while True:
        for key in keyboard.keys():
          if key in ("r", "R"):
            print(f"\n  {session.toggle()}")
            continue
          vx, wz, quit_requested, hold = apply_key(key, vx, wz, limits)
          if quit_requested:
            raise KeyboardInterrupt
          if hold == "home":
            mode = MODE_HOLD_HOME
          elif hold == "zero":
            mode = MODE_HOLD_ZERO
          elif hold is False:
            mode = MODE_WALK
        # Only on change, plus a keepalive: each packet costs the robot a
        # long radio transaction inside its control loop.
        state = link.send_if_changed(vx, wz, mode)
        if state:
          last_state = state
        session.log(cmd_vx=vx, cmd_wz=wz, mode=mode,
                    **_state_fields(last_state))
        now = time.monotonic()
        if now - last_print >= 0.2:
          last_print = now
          if state is None:
            status_line("  応答待ち...")
          else:
            where = {0: "歩行", 1: "停止(home)", 2: "停止(0度)"}
            tail = "  [REC]" if session.active else ""
            if link.link_age > 0.5:
              tail += "  !! 応答途絶"
            status_line(
              "  vx={:+.2f} wz={:+.2f}  {}  loop={:5d} us (max {:5d})"
              "  skipped={}{}".format(
                vx, wz, where.get(state["hold_mode"], "?"),
                state["loop_us"], state["loop_max_us"], state["skipped"],
                tail))
        # The robot services its radio every 100 ms, so sending faster than
        # that only queues packets it will discard. Keys are still read every
        # loop; only the transmit is paced.
        time.sleep(0.05)
  except KeyboardInterrupt:
    pass
  finally:
    link.close()
    out = session.stop()
    if out:
      print(f"\n  記録終了 {out}")
    print("\n  停止を送信しました")
  return 0


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("mode",
                  choices=["scan", "calibrate", "stand", "run", "stepid",
                           "teleop", "wifi"])
  ap.add_argument("--port", default=None,
                  help="/dev/tty.usbserial-XXXX (Windows は COM3 のような名前)")
  ap.add_argument("--baud", type=int, default=1000000)
  ap.add_argument("--bus", choices=["direct", "bridge"], default="direct",
                  help="サーボへの経路。direct=このPCのUSBアダプタ, "
                       "bridge=Arduino経由 (arduino/quad_bridge を書き込むこと)")
  ap.add_argument("--vx", type=float, default=0.05, help="前進速度 [m/s]")
  ap.add_argument("--wz", type=float, default=0.0, help="ヨー角速度 [rad/s]")
  ap.add_argument("--seconds", type=float, default=20.0)
  ap.add_argument("--vx-min", type=float, default=VX_MIN,
                  help="teleop の後退側の下限 [m/s] (既定 %(default)+.2f)")
  ap.add_argument("--vx-max", type=float, default=VX_MAX,
                  help="teleop の前進側の上限 [m/s] (既定 %(default)+.2f)")
  ap.add_argument("--wz-max", type=float, default=WZ_MAX,
                  help="teleop の旋回の上限 [rad/s] (既定 %(default).2f)")
  ap.add_argument("--host", default=None,
                  help="wifi: ロボットの IP。省略時はブロードキャストで探索")
  ap.add_argument("--udp-port", type=int, default=9000,
                  help="wifi: UDP ポート (.env の QUAD_UDP_PORT と合わせる)")
  ap.add_argument("--log", default=None,
                  help="run/stepid のログ CSV")
  ap.add_argument("--video", default=None,
                  help="run/wifi: 走行を録画する mp4。指令と関節角を同じ時間軸で "
                       "<video>.events.csv に書き、あとで overlay できる")
  ap.add_argument("--camera", default=None,
                  help="--video のカメラ。AVFoundation の番号か名前。"
                       "省略時は内蔵以外(=Continuity Camera の iPhone)を選ぶ")
  ap.add_argument("--record-dir", default="recordings",
                  help="r キーで撮ったものの置き場 (既定 %(default)s)")
  ap.add_argument("--no-camera", action="store_true",
                  help="録画は自分で撮るので、指令の記録だけ取る。"
                       "イベントの t_unix 列で後から映像に合わせられる")
  ap.add_argument("--video-fps", type=int, default=30,
                  help="--video の撮影レート (既定 %(default)d)")
  ap.add_argument("--list-cameras", action="store_true",
                  help="使えるカメラを並べて終了する")
  ap.add_argument("--joint", default="FL_hip_joint",
                  help="stepid: 対象の関節名または番号")
  ap.add_argument("--amp", type=float, default=0.25,
                  help="stepid: ステップ振幅 [rad]")
  ap.add_argument("--period", type=float, default=1.2,
                  help="stepid: 方形波の周期 [s]")
  ap.add_argument("--cycles", type=int, default=6, help="stepid: 周期数")
  ap.add_argument("--swing", action="store_true",
                  help="stepid: 最後にトルクを切って自由振動させる"
                       "(armature と frictionloss はこれが無いと決まらない)")
  ap.add_argument("--swing-s", type=float, default=4.0,
                  help="stepid: 自由振動の記録時間 [s]")
  ap.add_argument("--yes", action="store_true",
                  help="run / wifi: 対話プロンプトを飛ばし、--vx --wz --seconds で一定指令を流す")
  ap.add_argument("--write-middle", action="store_true",
                  help="calibrate 時にゼロ点を EEPROM に書く")
  args = ap.parse_args()
  if args.list_cameras:
    for index, name in list_cameras():
      print(f"  [{index}] {name}")
    return 0
  if args.video and args.mode not in ("run", "wifi", "teleop"):
    ap.error("--video は run / wifi / teleop でのみ使えます")
  if args.mode == "stepid" and not args.log:
    ap.error("stepid には --log <out.csv> が必要です")
  if args.mode == "calibrate" and not args.port:
    ap.error("calibrate は --port を明示してください "
             "(書き出す calib.json に記録されます)")
  return {"scan": cmd_scan, "calibrate": cmd_calibrate,
          "stand": cmd_stand, "run": cmd_run,
          "stepid": cmd_stepid,
          "teleop": cmd_teleop,
          "wifi": cmd_wifi}[args.mode](args)


if __name__ == "__main__":
  sys.exit(main())

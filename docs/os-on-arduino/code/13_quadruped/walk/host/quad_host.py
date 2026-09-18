#!/usr/bin/env python3
"""Drive the quadruped from a laptop over USB. Calibration and the 50 Hz loop.

This is the recommended way to bring the robot up. A laptop on the servo bus
needs no MCU flashing, prints every intermediate value, and turns a fix-and-retry
cycle into a keystroke. arduino_quad.ino is the eventual embedded target, not the
thing to debug against first.

    macOS:  pip install feetech-servo-sdk pyserial numpy
            ls /dev/tty.usb*        # the adapter shows up as tty.usbserial-* or tty.usbmodem*

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

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quad_policy import QuadPolicy  # noqa: E402

BUNDLE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")

COUNTS_PER_REV = 4096
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV
BAUD_TRY = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]
REG_MIDDLE_CALIB = 40          # write 128 -> current position becomes 2048
JOINT_SPAN_RAD = 1.2           # home +- this must stay inside 0..4095 counts


class Bus:
  """Thin wrapper over the Feetech SDK.

  Everything version-specific lives here. If your scservo_sdk names differ,
  this is the only class to edit.
  """

  def __init__(self, port: str, baud: int):
    try:
      from scservo_sdk import PortHandler, sms_sts
    except ImportError as exc:
      raise SystemExit(
        "scservo_sdk が見つかりません。  pip install feetech-servo-sdk") from exc
    self.ph = PortHandler(port)
    if not self.ph.openPort():
      raise SystemExit(f"ポートを開けません: {port}")
    if not self.ph.setBaudRate(baud):
      raise SystemExit(f"ボーレートを設定できません: {baud}")
    self.baud = baud
    self.h = sms_sts(self.ph)

  def set_baud(self, baud: int) -> None:
    self.ph.setBaudRate(baud)
    self.baud = baud
    time.sleep(0.05)

  def ping(self, sid: int) -> bool:
    r = self.h.ping(sid)
    res = r[1] if isinstance(r, (tuple, list)) else r
    return res == 0

  def read_pos(self, sid: int):
    r = self.h.ReadPos(sid)
    if isinstance(r, (tuple, list)):
      pos, res = r[0], r[1]
      return pos if res == 0 else None
    return r if r >= 0 else None

  def write_pos(self, sid: int, pos: int, speed: int = 0, acc: int = 0) -> None:
    self.h.WritePosEx(sid, int(pos), int(speed), int(acc))

  def sync_write_pos(self, ids, poss, speed: int = 0, acc: int = 0) -> None:
    fn = getattr(self.h, "SyncWritePosEx", None)
    if fn is None:
      for sid, p in zip(ids, poss):
        self.write_pos(sid, p, speed, acc)
      return
    for sid, p in zip(ids, poss):
      fn(int(sid), int(p), int(speed), int(acc))
    act = getattr(self.h, "RegWriteAction", None)
    if act:
      act()

  def torque(self, sid: int, on: bool) -> None:
    fn = getattr(self.h, "EnableTorque", None)
    if fn:
      fn(sid, 1 if on else 0)
    else:
      self.h.write1ByteTxRx(sid, 40, 1 if on else 0)

  def write_middle(self, sid: int) -> None:
    self.h.write1ByteTxRx(sid, REG_MIDDLE_CALIB, 128)

  def close(self) -> None:
    self.ph.closePort()


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
def cmd_scan(args) -> int:
  bus, ids = find_bus(args.port, args.baud)
  print(f"\n{bus.baud} bps で {len(ids)} 個: {ids}")
  for sid in ids:
    print(f"  ID {sid:>3}  pos={bus.read_pos(sid)}")
  if len(ids) != 8:
    print("\n  !! 8 個そろっていません。順に確認:")
    print("     ・サーボの電源はバッテリから(8 個 x ストール 2.7 A)。USB からは取らない")
    print("     ・半二重の方向切替をするアダプタを通しているか")
    print("     ・ID が重複していないか(重複すると衝突して両方黙る)")
  if bus.baud != 1000000:
    print(f"\n  !! 1 Mbps ではありません({bus.baud})。50 Hz 制御には 500 kbps 以上が要ります")
  bus.close()
  return 0


def cmd_calibrate(args) -> int:
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


class Robot:
  """Calibrated servo bus in joint space."""

  def __init__(self, port, baud):
    if not os.path.exists(CALIB):
      raise SystemExit(f"{CALIB} がありません。先に calibrate を実行してください")
    with open(CALIB) as f:
      c = json.load(f)
    self.ids = c["id"]
    self.sign = np.array(c["sign"], np.float32)
    self.zero = np.array(c["zero"], np.float32)
    self.bus = Bus(port or c["port"], baud or c["baud"])
    self.names = c["joint_names"]

  def read_q(self):
    counts = np.empty(8, np.float32)
    for i, sid in enumerate(self.ids):
      p = self.bus.read_pos(sid)
      if p is None:
        return None
      counts[i] = p
    return (counts - self.zero) * RAD_PER_COUNT * self.sign

  def write_q(self, q):
    counts = np.clip(np.asarray(q, np.float32) / RAD_PER_COUNT * self.sign
                     + self.zero, 0, 4095).astype(int)
    self.bus.sync_write_pos(self.ids, counts, 0, 0)

  def torque(self, on):
    for sid in self.ids:
      self.bus.torque(sid, on)

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
  r = Robot(args.port, args.baud)
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
  r = Robot(args.port, args.baud)
  dt = pol.dt

  print("  home 姿勢へ移動します")
  r.ramp_to(pol.default_q, 2.0)
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

  Samples as fast as the bus allows rather than at the control rate: the rise
  takes ~100 ms and 50 Hz would put five points on it. Real timestamps go in the
  log, so the fit does not have to assume the rate came out even.
  """
  pol = QuadPolicy(BUNDLE)
  r = Robot(args.port, args.baud)
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
        f"{args.period:.1f} s × {args.cycles} 回。これを速い・小さい・遅いの"
        f"4 条件で回します")
  print("  脚が自由に動く状態で吊るしてください。ぶつけると値が意味を失います。")
  input("  準備できたら Enter: ")

  r.torque(True)
  r.write_q(home)
  time.sleep(1.0)

  # Several amplitudes and several rates, not one square wave. The four
  # parameters do not all show up in the same motion: a fast rise is where
  # reflected inertia (armature) matters, while dry friction shows up in slow
  # motion and at every direction reversal. Fitting one square wave recovers
  # stiffness and damping and leaves the other two undetermined -- measured, on
  # a synthetic log whose answers were known.
  blocks = [(args.amp, args.period),
            (args.amp, args.period / 3.0),      # fast: excites inertia
            (args.amp / 4.0, args.period),      # small: friction dominates
            (args.amp, args.period * 2.5)]      # slow: creep and stiction
  rows = []
  t0 = time.perf_counter()
  tgt = home.copy()
  for amp, period in blocks:
    print(f"    振幅 {amp:.3f} rad / 周期 {period:.2f} s")
    tb = time.perf_counter()
    while time.perf_counter() - tb < period * args.cycles:
      t = time.perf_counter() - tb
      hi = (t % period) < (period / 2.0)
      tgt[j] = home[j] + (amp if hi else -amp)
      r.write_q(tgt)
      q = r.read_q()
      if q is None:
        continue
      rows.append((time.perf_counter() - t0, tgt[j], q[j]))

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
      q = r.read_q()
      if q is None:
        continue
      # target = NaN marks "no command": the fit must not treat these samples
      # as a position loop tracking something.
      rows.append((time.perf_counter() - t0, float("nan"), q[j]))
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


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("mode",
                  choices=["scan", "calibrate", "stand", "run", "stepid"])
  ap.add_argument("--port", default=None, help="/dev/tty.usbserial-XXXX")
  ap.add_argument("--baud", type=int, default=1000000)
  ap.add_argument("--vx", type=float, default=0.05, help="前進速度 [m/s]")
  ap.add_argument("--wz", type=float, default=0.0, help="ヨー角速度 [rad/s]")
  ap.add_argument("--seconds", type=float, default=20.0)
  ap.add_argument("--log", default=None,
                  help="run/stepid のログ CSV")
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
  ap.add_argument("--write-middle", action="store_true",
                  help="calibrate 時にゼロ点を EEPROM に書く")
  args = ap.parse_args()
  if args.mode == "stepid" and not args.log:
    ap.error("stepid には --log <out.csv> が必要です")
  if args.mode in ("scan", "calibrate", "stepid") and not args.port:
    ap.error("--port を指定してください (ls /dev/tty.usb*)")
  return {"scan": cmd_scan, "calibrate": cmd_calibrate,
          "stand": cmd_stand, "run": cmd_run,
          "stepid": cmd_stepid}[args.mode](args)


if __name__ == "__main__":
  sys.exit(main())

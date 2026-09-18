"""The servo bus, behind one interface with two transports.

The control code above this file -- observation assembly, the policy, the
50 Hz loop -- must not care how bytes reach the servos. Two ways exist:

  DirectBus   the adapter is on this machine's USB, feetech_cli drives it
  BridgeBus   the adapter is on an Arduino, which we drive over USB

They differ in more than plumbing. Direct pays a USB round trip per servo,
eight per control step. The bridge pays one per step and lets the MCU do the
eight fast reads on its own UART, which measured 203 us each against 765 us
from the host. Same interface, so switching is a command line flag.
"""
from __future__ import annotations

import struct
import time

# --- host <-> Arduino framing (mirrors arduino/quad_bridge/quad_bridge.ino) --
REQ_HEADER = 0xA5
RSP_HEADER = 0x5A
CMD_SCAN = 0x01
CMD_READPOS = 0x02
CMD_WRITE = 0x03
CMD_TORQUE = 0x04
CMD_HELLO = 0x05
CMD_STEP = 0x06
CMD_READREG = 0x07
CMD_WRITEREG = 0x08
CMD_ERROR = 0xFF

#: Per joint marker the firmware sends when a servo did not answer.
POS_FAILED = 0x7FFF

#: Control table addresses used outside the hot loop.
ADDR_LOCK = 55
#: Speed shaping registers. Both live in SRAM, so a power cycle silently puts
#: them back to the factory 50 / 1, which caps the joint at 21% of its no-load
#: speed (measured: 1.00 rad/s against 5.34 with them opened up). Nothing
#: reports this -- the robot just moves slowly -- so they are re-applied on
#: every connection rather than set once by hand.
ADDR_MAX_ACCELERATION = 85
ADDR_ACCEL_MULTIPLIER = 86
#: Registers below this address live in EEPROM and honour the lock.
EEPROM_END = 40


class ServoBus:
  """What the control code is allowed to assume about the bus.

  Positions are raw encoder counts; turning them into joint angles is the
  caller's job. Every method may raise OSError if the transport dies.
  """

  #: Human readable description of where this bus goes.
  description = "?"

  def ping(self, sid: int) -> bool:
    """Is this servo answering?"""
    raise NotImplementedError

  def scan(self, lo: int = 1, hi: int = 16):
    """Ids answering in [lo, hi], ascending."""
    raise NotImplementedError

  def read_positions(self, ids):
    """Present positions for `ids`, None for any servo that did not answer.

    Batched on purpose: this is the call in the 50 Hz loop, and a transport
    that can read all eight joints in one exchange should be able to say so.
    """
    raise NotImplementedError

  def sync_write_positions(self, ids, counts, speed: int = 0, acc: int = 0):
    """Command every joint in one broadcast. Nothing is read back."""
    raise NotImplementedError

  def torque(self, ids, on: bool):
    """Energise or release the listed servos."""
    raise NotImplementedError

  def step(self, ids, counts, speed: int = 0, acc: int = 0):
    """Command the joints and read them back, as one operation.

    This is what the control loop calls. Splitting it into a write and a
    read is correct but pays the transport latency twice, which on a USB
    bridge is the whole cost. Transports that can do both in one exchange
    override this; the default just does the two calls.
    """
    self.sync_write_positions(ids, counts, speed, acc)
    return self.read_positions(ids)

  def read_register(self, sid: int, addr: int, size: int):
    """Read `size` bytes at `addr` from one servo, or None if it went quiet.

    Diagnostics only -- the control loop must not use this, it is one
    transaction per call.
    """
    raise NotImplementedError

  def write_register(self, sid: int, addr: int, data) -> bool:
    """Write bytes at `addr` on one servo.

    EEPROM registers (`addr` below :data:`EEPROM_END`) are unlocked first and
    locked again afterwards, which is what the servos require. Returns False
    if the servo did not take it.
    """
    raise NotImplementedError

  def close(self):
    """Release the transport."""
    raise NotImplementedError


class DirectBus(ServoBus):
  """The adapter is on this machine's USB.

  Parameters
  ----------
  port : str or None
      Serial device, or None to find the adapter servos answer on.
  baud : int
      Bus speed.
  """

  def __init__(self, port, baud):
    try:
      from feetech_cli.controller import FeetechServoController
    except ImportError as exc:
      raise SystemExit(
        "feetech_cli が見つかりません。  リポジトリ直下で uv sync してください"
      ) from exc
    self.c = FeetechServoController(port=port, baudrate=baud, timeout=0.02)
    self.c.open(require_servo=port is None)
    self.port = self.c.serial.port
    self.baud = baud
    self.description = f"直結 {self.port} @ {baud}"

  def ping(self, sid):
    return self.c.ping(sid)

  def scan(self, lo=1, hi=16):
    return self.c.scan(min_id=lo, max_id=hi)

  def read_positions(self, ids):
    from feetech_cli.protocol import FeetechError
    out = []
    for sid in ids:
      try:
        out.append(self.c.read_register(sid, "present_position"))
      except FeetechError:
        out.append(None)
    return out

  def sync_write_positions(self, ids, counts, speed=0, acc=0):
    self.c.sync_write_positions(ids, [int(c) for c in counts],
                                velocity=speed, acceleration=acc)

  def read_register(self, sid, addr, size):
    from feetech_cli.protocol import FeetechError
    try:
      return bytes(self.c.packet_handler.read(sid, addr, size))
    except FeetechError:
      return None

  def write_register(self, sid, addr, data):
    from feetech_cli.protocol import FeetechError
    data = bytes(data)
    eeprom = addr < EEPROM_END
    try:
      if eeprom:
        self.c.unlock_eeprom(sid)
      self.c.packet_handler.write(sid, addr, data)
      return True
    except FeetechError:
      return False
    finally:
      if eeprom:
        try:
          self.c.lock_eeprom(sid)
        except FeetechError:
          pass

  def torque(self, ids, on):
    for sid in ids:
      self.c.set_torque(sid, bool(on))

  def close(self):
    self.c.close()


class BridgeBus(ServoBus):
  """The adapter is on an Arduino running arduino/quad_bridge.

  Parameters
  ----------
  port : str or None
      The Arduino's USB serial device, or None to find it by handshake.
  timeout : float
      Seconds to wait for one reply.
  """

  #: USB vendor ids worth trying when hunting for the board.
  VENDORS = (0x2341, 0x2A03, 0x1A86, 0x10C4)

  def __init__(self, port=None, timeout=0.5):
    import serial
    self.timeout = timeout
    candidates = [port] if port else self._candidates()
    if not candidates:
      raise SystemExit("Arduino が見つかりません。--port で指定してください")
    last = None
    for dev in candidates:
      try:
        self.ser = serial.Serial(dev, 115200, timeout=timeout)
      except OSError as exc:
        last = exc
        continue
      # The R4 resets when the port opens; wait for the sketch, then check
      # that whatever is there speaks this protocol before trusting it.
      time.sleep(1.6)
      self.ser.reset_input_buffer()
      try:
        info = self._exchange(CMD_HELLO, b"")
      except OSError as exc:
        last = exc
        self.ser.close()
        continue
      self.port = dev
      self.firmware = (info[0], info[1]) if len(info) >= 2 else (0, 0)
      self.baud = info[2] * 100000 if len(info) >= 3 else 0
      self.description = (
        f"Arduino 経由 {dev} (fw {self.firmware[0]}.{self.firmware[1]}, "
        f"バス {self.baud})")
      return
    raise SystemExit(f"quad_bridge が応答しません: {candidates} ({last})")

  @staticmethod
  def _candidates():
    """Serial devices that could be the board, likeliest first."""
    import serial.tools.list_ports
    ports = [p for p in serial.tools.list_ports.comports()
             if p.vid in BridgeBus.VENDORS]
    ports.sort(key=lambda p: 0 if p.vid == 0x2341 else 1)
    return [p.device for p in ports]

  # -- framing -------------------------------------------------------------
  @staticmethod
  def _checksum(cmd, payload):
    c = cmd ^ len(payload)
    for b in payload:
      c ^= b
    return c

  def _exchange(self, cmd, payload):
    """Send one request and return the reply payload."""
    frame = bytes([REQ_HEADER, cmd, len(payload)]) + bytes(payload) + \
        bytes([self._checksum(cmd, payload)])
    self.ser.reset_input_buffer()
    self.ser.write(frame)
    self.ser.flush()

    head = self._read_exact(1)
    deadline = time.monotonic() + self.timeout
    while head != bytes([RSP_HEADER]):
      if time.monotonic() > deadline:
        raise OSError("quad_bridge から応答ヘッダが来ません")
      head = self._read_exact(1)
    rcmd, rlen = self._read_exact(2)
    body = self._read_exact(rlen) if rlen else b""
    chk = self._read_exact(1)[0]
    if chk != self._checksum(rcmd, body):
      raise OSError("quad_bridge の応答チェックサム不一致")
    if rcmd == CMD_ERROR:
      raise OSError(f"quad_bridge がエラーを返しました: {list(body)}")
    if rcmd != cmd:
      raise OSError(f"応答コマンド不一致: {rcmd:#x} != {cmd:#x}")
    return body

  def _read_exact(self, n):
    data = self.ser.read(n)
    if len(data) < n:
      raise OSError(f"quad_bridge からの受信が足りません ({len(data)}/{n})")
    return data

  # -- interface -----------------------------------------------------------
  def ping(self, sid):
    return sid in self.scan(sid, sid)

  def scan(self, lo=1, hi=16):
    body = self._exchange(CMD_SCAN, bytes([lo, hi]))
    return list(body[1:1 + body[0]])

  def read_positions(self, ids):
    ids = list(ids)
    body = self._exchange(CMD_READPOS, bytes([len(ids)]) + bytes(ids))
    n = body[0]
    vals = struct.unpack_from("<%dh" % n, body, 1)
    return [None if v == POS_FAILED or v < 0 else int(v) for v in vals]

  def sync_write_positions(self, ids, counts, speed=0, acc=0):
    ids = list(ids)
    payload = bytearray([len(ids)])
    for sid, c in zip(ids, counts):
      payload += struct.pack("<Bh", sid, int(c))
    payload += struct.pack("<HB", int(speed), max(0, min(254, int(acc))))
    self._exchange(CMD_WRITE, bytes(payload))

  def torque(self, ids, on):
    ids = list(ids)
    self._exchange(CMD_TORQUE, bytes([len(ids)]) + bytes(ids) + bytes([1 if on else 0]))

  def step(self, ids, counts, speed=0, acc=0):
    ids = list(ids)
    payload = bytearray([len(ids)])
    for sid, c in zip(ids, counts):
      payload += struct.pack("<Bh", sid, int(c))
    payload += struct.pack("<HB", int(speed), max(0, min(254, int(acc))))
    body = self._exchange(CMD_STEP, bytes(payload))
    n = body[0]
    vals = struct.unpack_from("<%dh" % n, body, 1)
    return [None if v == POS_FAILED or v < 0 else int(v) for v in vals]

  def read_register(self, sid, addr, size):
    body = self._exchange(CMD_READREG, bytes([sid, addr, size]))
    n = body[0]
    return bytes(body[1:1 + n]) if n == size else None

  def write_register(self, sid, addr, data):
    # The firmware writes registers verbatim, so the EEPROM lock is worked
    # here. Lock lives in SRAM, so it is writable either way.
    data = bytes(data)
    eeprom = addr < EEPROM_END
    try:
      if eeprom:
        self._exchange(CMD_WRITEREG, bytes([sid, ADDR_LOCK, 0]))
        time.sleep(0.02)
      self._exchange(CMD_WRITEREG, bytes([sid, addr]) + data)
      return True
    except OSError:
      return False
    finally:
      if eeprom:
        try:
          self._exchange(CMD_WRITEREG, bytes([sid, ADDR_LOCK, 1]))
        except OSError:
          pass

  def close(self):
    try:
      self.ser.close()
    except OSError:
      pass


def open_bus(kind, port, baud):
  """Build the bus the caller asked for.

  Parameters
  ----------
  kind : str
      "direct" or "bridge".
  port : str or None
      Serial device, or None to search.
  baud : int
      Servo bus speed. Ignored by the bridge, which sets it in firmware.

  Returns
  -------
  ServoBus
      An open bus.
  """
  if kind == "bridge":
    return BridgeBus(port)
  if kind == "direct":
    return DirectBus(port, baud)
  raise SystemExit(f"未知のバス種別: {kind}")


def apply_runtime_limits(bus, ids, settings, tries=6):
  """Re-apply the SRAM speed registers, which a power cycle resets.

  Parameters
  ----------
  bus : ServoBus
      An open bus.
  ids : sequence of int
      Servos to set.
  settings : dict
      ``max_acceleration`` and ``accel_multiplier``.
  tries : int, optional
      Attempts per register.

  Returns
  -------
  tuple of (bool, dict)
      Whether every servo took every value, and what they read back.
  """
  wanted = [
    (ADDR_MAX_ACCELERATION, int(settings.get("max_acceleration", 254))),
    (ADDR_ACCEL_MULTIPLIER, int(settings.get("accel_multiplier", 0))),
  ]
  ok, seen = True, {}
  for sid in ids:
    for addr, value in wanted:
      done = False
      for _ in range(tries):
        if bus.write_register(sid, addr, bytes([value & 0xFF])):
          got = bus.read_register(sid, addr, 1)
          if got and got[0] == (value & 0xFF):
            done = True
            break
        time.sleep(0.05)
      ok = ok and done
      seen.setdefault(sid, {})[addr] = value if done else None
  return ok, seen

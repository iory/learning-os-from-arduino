"""The wireless link to a robot running arduino/quad_wifi.

Over USB the host owns the control loop and the bus is the interface. Over
WiFi that split moves: the robot owns the loop, and the interface becomes the
command. It has to be that way -- a WiFi round trip inside a 20 ms control
step is not something jitter allows -- so this module carries velocities, not
servo bytes.

Wire format, little endian both ways:
    command : 'Q','C', float vx, float wz, uint8 mode
              mode 0 = walk, 1 = hold the home pose, 2 = hold every joint at 0
    state   : 'Q','S', int16 joint[8] in 0.1 deg, uint16 loop_us,
              uint16 loop_max_us, uint16 skipped, uint8 hold_mode
"""
from __future__ import annotations

import re
import socket
import struct
import subprocess
import time

#: UDP port the firmware listens on. Matches QUAD_UDP_PORT in .env.
DEFAULT_PORT = 9000

MODE_WALK = 0
MODE_HOLD_HOME = 1
MODE_HOLD_ZERO = 2

_COMMAND = struct.Struct("<ffB")
_STATE = struct.Struct("<8h3HB")


class RobotLink:
  """A UDP connection to the robot.

  Parameters
  ----------
  host : str or None
      Robot address, or None to find it by broadcast.
  port : int
      UDP port.
  timeout : float
      Seconds to wait for a state reply.
  """

  #: The firmware replies several times a second; half a second of silence
  #: already means packets are being lost.
  stale_after = 0.5

  def __init__(self, host=None, port=DEFAULT_PORT, timeout=0.3):
    self.port = port
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    self.sock.settimeout(timeout)
    self.host = host or self.discover(port, self.sock)
    if self.host is None:
      raise SystemExit(
        "ロボットが見つかりません。R4 の電源と WiFi 接続を確認してください。"
        " --host で IP を直接指定もできます")
    # Discovery is done; from here the caller sends on a fixed cadence and
    # must never block waiting for a reply. Blocking would put the network
    # round trip inside the send loop, which is exactly what moving the
    # control loop onto the robot was meant to avoid.
    self.sock.setblocking(False)
    #: Set when a reply has been seen, so the caller can show link loss.
    self.last_state = None
    self.last_seen = 0.0
    self._last_command = None
    self._last_sent = 0.0

  @staticmethod
  def broadcast_addresses():
    """IPv4 broadcast addresses of this machine's interfaces.

    Returns
    -------
    list of str
        Addresses to try, the global broadcast first.
    """
    found = ["255.255.255.255"]
    try:
      out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
    except OSError:
      return found
    for line in out.splitlines():
      line = line.strip()
      if line.startswith("inet ") and "broadcast" in line:
        parts = line.split()
        found.append(parts[parts.index("broadcast") + 1])
    return found

  @classmethod
  def discover(cls, port, sock, attempts=6):
    """Find the robot by broadcasting a harmless hold command.

    The probe carries mode 1, so a robot that hears it stands rather than
    doing anything sudden.

    Parameters
    ----------
    port : int
        UDP port.
    sock : socket.socket
        Socket to probe with.
    attempts : int, optional
        Broadcast rounds before giving up.

    Returns
    -------
    str or None
        The robot's address.
    """
    probe = b"QC" + _COMMAND.pack(0.0, 0.0, MODE_HOLD_HOME)
    targets = cls.broadcast_addresses()
    for _ in range(attempts):
      for target in targets:
        try:
          sock.sendto(probe, (target, port))
        except OSError:
          continue
      deadline = time.monotonic() + 0.4
      while time.monotonic() < deadline:
        try:
          data, addr = sock.recvfrom(128)
        except socket.timeout:
          break
        if data[:2] == b"QS":
          return addr[0]
    return None

  def send_if_changed(self, vx, wz, mode, keepalive_hz=5.0, repeats=3):
    """Send when the command changes, and otherwise at the keepalive rate.

    Every packet the robot receives costs it a long radio transaction, so
    sending on a fixed high cadence is what breaks its control loop. A key
    press is sent at once (repeated a few times, since UDP may drop one) and
    the rest of the time only the watchdog needs feeding.

    Parameters
    ----------
    vx, wz : float
        Commanded velocities.
    mode : int
        One of the MODE_ constants.
    keepalive_hz : float, optional
        Rate for unchanged commands. Must stay above the robot's watchdog.
    repeats : int, optional
        Copies sent on a change.

    Returns
    -------
    dict or None
        The newest state, if the firmware is configured to reply.
    """
    now = time.monotonic()
    command = (round(vx, 4), round(wz, 4), int(mode))
    changed = command != self._last_command
    due = now - self._last_sent >= 1.0 / keepalive_hz
    if not (changed or due):
        return self.last_state
    self._last_command = command
    self._last_sent = now
    state = None
    for _ in range(repeats if changed else 1):
        state = self.send(vx, wz, mode)
    return state

  def send(self, vx, wz, mode):
    """Send one command and collect whatever state has already arrived.

    Never blocks: replies are picked up on a later call. The robot is not
    waiting on us either -- it runs its loop regardless.

    Parameters
    ----------
    vx : float
        Forward velocity [m/s].
    wz : float
        Yaw rate [rad/s].
    mode : int
        One of the MODE_ constants.

    Returns
    -------
    dict or None
        The newest state, or None if nothing has come back yet.
    """
    packet = b"QC" + _COMMAND.pack(float(vx), float(wz), int(mode))
    try:
      self.sock.sendto(packet, (self.host, self.port))
    except OSError:
      return self.last_state
    while True:
      try:
        data, _ = self.sock.recvfrom(128)
      except (BlockingIOError, socket.timeout, OSError):
        break
      if data[:2] == b"QS" and len(data) >= 2 + _STATE.size:
        fields = _STATE.unpack_from(data, 2)
        self.last_state = {
          "joints_deg": [v / 10.0 for v in fields[:8]],
          "loop_us": fields[8],
          "loop_max_us": fields[9],
          "skipped": fields[10],
          "hold_mode": fields[11],
        }
        self.last_seen = time.monotonic()
    return self.last_state

  @property
  def link_age(self):
    """float: Seconds since the last reply, or inf before the first one."""
    if not self.last_seen:
      return float("inf")
    return time.monotonic() - self.last_seen

  def round_trip(self, tries=20):
    """Measure the command-to-reply latency, for reporting only.

    Blocks on purpose, unlike :meth:`send`.

    Returns
    -------
    list of float
        Milliseconds per successful exchange.
    """
    out = []
    self.sock.setblocking(True)
    self.sock.settimeout(0.5)
    try:
      for _ in range(tries):
        began = time.perf_counter()
        self.sock.sendto(b"QC" + _COMMAND.pack(0.0, 0.0, MODE_HOLD_HOME),
                         (self.host, self.port))
        try:
          data, _ = self.sock.recvfrom(128)
        except socket.timeout:
          continue
        if data[:2] == b"QS":
          out.append((time.perf_counter() - began) * 1e3)
    finally:
      self.sock.setblocking(False)
    return out

  def close(self):
    """Stop the robot and release the socket."""
    for _ in range(4):
      try:
        self.sock.sendto(b"QC" + _COMMAND.pack(0.0, 0.0, MODE_HOLD_HOME),
                         (self.host, self.port))
      except OSError:
        break
      time.sleep(0.02)
    self.sock.close()

  @property
  def description(self):
    """str: Where the commands go, for the header line."""
    return f"WiFi {self.host}:{self.port}"


#: The chapter firmware's once-a-second report line.
_LOOP_LINE = re.compile(
  r"loop avg (\d+) us\s+max (\d+) us\s+late \d+/\d+\s+skipped (\d+)\s+read_fail (\d+)")


class SerialLink:
  """The USB serial link to a robot running the chapter firmware (src/main.cpp).

  Same interface as :class:`RobotLink`, so the keyboard loop does not care
  which one it drives. The firmware already runs the policy; this only sends
  `drive <vx> <wz>` often enough to keep its 300 ms watchdog fed, and reads
  back the report line it prints every second.

  Parameters
  ----------
  port : str or None
      The Arduino's serial device, or None to pick the one Arduino UNO R4 on
      this machine.
  baud : int
      Must match Serial.begin() in the firmware.
  """

  #: USB vendor id of Arduino boards.
  ARDUINO_VID = 0x2341
  #: The firmware reports once a second, so silence only means trouble after
  #: a missed report or so.
  stale_after = 1.5

  def __init__(self, port=None, baud=115200):
    import serial
    import serial.tools.list_ports
    if port is None:
      found = [p.device for p in serial.tools.list_ports.comports()
               if p.vid == self.ARDUINO_VID]
      if not found:
        raise SystemExit("Arduino が見つかりません。USB を挿して --port で指定してください")
      if len(found) > 1:
        raise SystemExit(f"Arduino が複数あります {found}。--port で指定してください")
      port = found[0]
    # The UNO R4 does not reset when its port is opened, so the robot keeps
    # whatever it was doing (normally holding home after boot).
    self.ser = serial.Serial(port, baud, timeout=0)
    self.description = f"USB シリアル {port}"
    self.last_state = None
    self.last_seen = 0.0
    self._hold_mode = MODE_HOLD_HOME
    self._last_mode = None
    self._buf = b""

  def send_if_changed(self, vx, wz, mode, keepalive_hz=None, repeats=None):
    """Send the command; unlike UDP, every call is cheap enough to send.

    Parameters
    ----------
    vx, wz : float
        Commanded velocities.
    mode : int
        One of the MODE_ constants.
    keepalive_hz, repeats : optional
        Accepted for interface compatibility with RobotLink and unused.

    Returns
    -------
    dict or None
        The newest state parsed from the firmware's output.
    """
    return self.send(vx, wz, mode)

  def send(self, vx, wz, mode):
    """Send one command and parse whatever the firmware has printed.

    Walking is sent every call to feed the watchdog. The holds are sent only
    when the mode changes: each `stop` restarts the firmware's 2 s ramp to
    home, so repeating it would keep the ramp from ever finishing.

    Returns
    -------
    dict or None
        The newest state, or None before the first report line.
    """
    if mode == MODE_WALK:
      line = f"drive {vx:.3f} {wz:.3f}\n"
    elif mode != self._last_mode:
      line = "stop\n" if mode == MODE_HOLD_HOME else "zero\n"
    else:
      line = None
    self._last_mode = mode
    if line is not None:
      try:
        self.ser.write(line.encode("ascii"))
      except OSError:
        return self.last_state
    self._read_lines()
    return self.last_state

  def _read_lines(self):
    try:
      self._buf += self.ser.read(4096)
    except OSError:
      return
    *lines, self._buf = self._buf.split(b"\n")
    for raw in lines:
      text = raw.decode("utf-8", "replace").strip()
      if text.startswith("mode=rl"):
        self._hold_mode = MODE_WALK
      elif text.startswith("mode=zero"):
        self._hold_mode = MODE_HOLD_ZERO
      elif text.startswith("mode=hold"):
        self._hold_mode = MODE_HOLD_HOME
      match = _LOOP_LINE.search(text)
      if match:
        self.last_state = {
          "loop_us": int(match.group(1)),
          "loop_max_us": int(match.group(2)),
          "skipped": int(match.group(3)),
          "read_fail": int(match.group(4)),
          "hold_mode": self._hold_mode,
        }
        self.last_seen = time.monotonic()
      elif self.last_state is not None:
        self.last_state["hold_mode"] = self._hold_mode

  @property
  def link_age(self):
    """float: Seconds since the last report line, or inf before the first."""
    if not self.last_seen:
      return float("inf")
    return time.monotonic() - self.last_seen

  def close(self):
    """Hold home and release the port."""
    try:
      self.ser.write(b"stop\n")
      self.ser.flush()
    except OSError:
      pass
    self.ser.close()

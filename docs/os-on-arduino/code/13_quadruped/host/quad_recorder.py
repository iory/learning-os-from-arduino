"""Record video of a run alongside the commands that produced it.

A video of the robot walking says what happened; it does not say what it was
told to do. The two together are what makes a recording worth keeping, and
they only go together if the clocks agree.

macOS makes the camera part easy: an iPhone shows up as an ordinary
AVFoundation device over Continuity Camera, so ffmpeg can read it with no app
on the phone. The part that needs care is the offset between the video's first
frame and the host's clock, because ffmpeg takes a second or so to open a
camera and nothing reports when the first photon landed.

This module solves that by reading ffmpeg's own progress stream. Each block
carries ``out_time_us``, the timestamp of the last frame written, so

    video_t0 = (time it was read) - out_time_us / 1e6

estimates when video time zero was on the host clock. Scheduling only ever
makes the read late, never early, so the smallest estimate across all blocks is
the best one, and it keeps improving while the recording runs.

    from quad_recorder import Recording

    with Recording("walk.mp4") as rec:
        while running:
            rec.log(cmd_vx=vx, cmd_wz=wz, q0=q[0])

The sidecar CSV is written next to the video as ``<video>.events.csv`` with a
``t_video`` column in the video's own timebase, which is what an overlay tool
needs and nothing else provides. It also carries ``t_unix``, so a video filmed
on the phone itself -- better picture, no Continuity Camera -- can be lined up
afterwards from the two wall clocks.

Passing ``camera=False`` skips ffmpeg entirely and keeps only the event log,
which is the right mode when the filming happens somewhere else.
"""
from __future__ import annotations

import csv
import os
import re
import shutil
import subprocess
import threading
import time

#: Devices that are the laptop rather than a phone.
_BUILTIN = ("macbook", "desk view", "capture screen", "facetime")


def list_cameras():
  """AVFoundation video devices, as ``(index, name)`` pairs.

  Returns
  -------
  list of (int, str)
      Empty when ffmpeg is missing or the device list cannot be read.
  """
  if shutil.which("ffmpeg") is None:
    return []
  out = subprocess.run(
    ["ffmpeg", "-hide_banner", "-f", "avfoundation",
     "-list_devices", "true", "-i", ""],
    capture_output=True, text=True).stderr
  cameras, in_video = [], False
  for line in out.splitlines():
    if "AVFoundation video devices" in line:
      in_video = True
      continue
    if "AVFoundation audio devices" in line:
      in_video = False
      continue
    m = re.search(r"\[(\d+)\]\s+(.*)$", line)
    if in_video and m:
      cameras.append((int(m.group(1)), m.group(2).strip()))
  return cameras


def pick_camera():
  """The index of the most likely external camera, or None.

  Prefers anything that is not the built-in camera or a screen: with
  Continuity Camera running, that is the iPhone.

  Returns
  -------
  int or None
  """
  cameras = list_cameras()
  for index, name in cameras:
    if not any(k in name.lower() for k in _BUILTIN):
      return index
  return cameras[0][0] if cameras else None


class Recording:
  """A running ffmpeg capture plus a time-aligned event log.

  Parameters
  ----------
  path : str
      Output video file. The event log goes to ``<path>.events.csv``.
  camera : int or str, optional
      AVFoundation device index or name. Defaults to :func:`pick_camera`.
  fps : int, optional
      Capture rate requested from the camera.
  bitrate : str, optional
      Video bitrate passed to the hardware encoder.
  """

  def __init__(self, path, camera=None, fps=30, bitrate="8M"):
    self.path = path
    self.events_path = os.path.splitext(path)[0] + ".events.csv"
    self._lock = threading.Lock()
    self._rows = []
    self._columns = None
    self._frames = 0
    self.proc = None
    self._reader = None

    if camera is False:
      # Events only: somebody else is holding the camera. Time zero is when
      # this object was made, which is when the operator asked to record.
      self.camera = None
      self._t0 = time.perf_counter()
      return

    if shutil.which("ffmpeg") is None:
      raise SystemExit("ffmpeg が見つかりません (brew install ffmpeg)")
    if camera is None:
      camera = pick_camera()
      if camera is None:
        raise SystemExit("カメラが見つかりません。iPhone を近くに置いて "
                         "Continuity Camera を有効にしてください")
    self.camera = str(camera)
    self._t0 = None            # host clock at video time zero, best estimate

    self.proc = subprocess.Popen(
      ["ffmpeg", "-hide_banner", "-loglevel", "error",
       "-f", "avfoundation", "-framerate", str(fps), "-i", self.camera,
       "-c:v", "h264_videotoolbox", "-b:v", bitrate, "-pix_fmt", "yuv420p",
       "-stats_period", "0.1", "-progress", "pipe:1", "-y", path],
      stdin=subprocess.PIPE, stdout=subprocess.PIPE,
      stderr=subprocess.PIPE, text=True, bufsize=1)
    self._reader = threading.Thread(target=self._read_progress, daemon=True)
    self._reader.start()

  # -- clock ---------------------------------------------------------------
  def _read_progress(self):
    """Track video time zero on the host clock from ffmpeg's progress blocks.

    Only the minimum matters. A block can be read late -- the pipe buffers,
    the thread is descheduled -- but never early, so every sample is an upper
    bound on ``t0`` and the smallest one is the closest.
    """
    for line in self.proc.stdout:
      now = time.perf_counter()
      key, _, value = line.strip().partition("=")
      if key == "frame":
        try:
          self._frames = int(value)
        except ValueError:
          pass
      elif key == "out_time_us":
        try:
          elapsed = int(value) / 1e6
        except ValueError:
          continue
        if elapsed <= 0:
          continue
        with self._lock:
          candidate = now - elapsed
          if self._t0 is None or candidate < self._t0:
            self._t0 = candidate

  @property
  def has_video(self):
    """bool: whether this recording is also capturing a video."""
    return self.proc is not None

  def wait_ready(self, timeout=15.0):
    """Block until the camera is delivering frames.

    Starting the robot before the camera is live loses the beginning of the
    run, which is the part where it stands up.

    Returns
    -------
    bool
        True if frames arrived within the timeout.
    """
    if not self.has_video:
      return True
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
      if self.proc.poll() is not None:
        raise SystemExit("ffmpeg が終了しました: "
                         + (self.proc.stderr.read() or "").strip())
      with self._lock:
        if self._t0 is not None:
          return True
      time.sleep(0.05)
    return False

  @property
  def video_t0(self):
    """float or None: host clock reading at video time zero."""
    with self._lock:
      return self._t0

  # -- events --------------------------------------------------------------
  def log(self, **fields):
    """Record one row, stamped in the video's timebase.

    Rows taken before the first frame get a negative ``t_video``; they are
    kept rather than dropped so a missing early row is visible as a gap
    instead of silently shifting everything.
    """
    now = time.perf_counter()
    wall = time.time()
    with self._lock:
      t0 = self._t0
      if self._columns is None:
        self._columns = list(fields)
      self._rows.append((now, wall, dict(fields)))
    if t0 is None:
      return
    return now - t0

  # -- lifetime ------------------------------------------------------------
  def close(self):
    """Stop the capture cleanly and write the event log.

    ffmpeg is asked to quit with 'q' rather than killed: a killed process
    leaves the mp4 without its index and the file will not seek.
    """
    if self.proc is not None and self.proc.poll() is None:
      try:
        self.proc.stdin.write("q")
        self.proc.stdin.flush()
      except (BrokenPipeError, ValueError):
        pass
      try:
        self.proc.wait(timeout=10)
      except subprocess.TimeoutExpired:
        self.proc.terminate()
        self.proc.wait(timeout=5)
    if self._reader is not None:
      self._reader.join(timeout=2)

    t0 = self.video_t0
    if t0 is None:
      print("  !! 映像の開始時刻を取れませんでした。録画開始時刻を 0 にします")
      t0 = self._rows[0][0] if self._rows else time.perf_counter()
    with open(self.events_path, "w", newline="", encoding="utf-8") as f:
      writer = csv.writer(f)
      writer.writerow(["t_video", "t_unix"] + (self._columns or []))
      for when, wall, fields in self._rows:
        writer.writerow(["%.4f" % (when - t0), "%.4f" % wall]
                        + [fields.get(c, "") for c in (self._columns or [])])
    return self.events_path if not self.has_video else self.path

  def __enter__(self):
    return self

  def __exit__(self, *_):
    self.close()
    return False

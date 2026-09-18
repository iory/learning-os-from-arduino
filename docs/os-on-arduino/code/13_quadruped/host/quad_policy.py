"""Host-side twin of quad_control.h: observation assembly + policy, in numpy.

Same contract as the C version, and verified the same way -- scripts/
check_quad_control.py replays a trained policy in mjlab and asserts this file
reproduces the environment's own observation vector and joint targets step for
step. If it drifts from quad_control.h, that check fails.

Why a Python twin exists: bringing this robot up from a laptop over USB is far
easier than flashing an MCU. Every intermediate value is printable, the servo
bus is one pyserial handle away, and there is no build step between a fix and a
retry. The Arduino path is the eventual embedded target, not the first thing to
get working.

Units are the simulation's: radians, rad/s, seconds, m/s. Servo counts are the
caller's problem (see quad_host.py).
"""
from __future__ import annotations

import json
import os

import numpy as np


class QuadPolicy:
  """Loads an exported policy and turns measurements into joint targets."""

  def __init__(self, bundle_dir: str) -> None:
    meta_path = os.path.join(bundle_dir, "arduino_quad_policy.json")
    npz_path = os.path.join(bundle_dir, "arduino_quad_policy.npz")
    with open(meta_path) as f:
      self.meta = json.load(f)
    z = np.load(npz_path)
    self.obs_mean = z["obs_mean"].astype(np.float32)
    self.obs_std = z["obs_std"].astype(np.float32)   # already includes the eps
    self.default_q = z["default_joint_pos"].astype(np.float32)
    self.action_scale = z["action_scale"].astype(np.float32)
    n = int(z["n_layers"])
    self.layers = [(z["w%d" % i].astype(np.float32), z["b%d" % i].astype(np.float32))
                   for i in range(n)]

    self.nj = int(self.meta["act_dim"])
    self.obs_dim = int(self.meta["obs_dim"])
    self.dt = float(self.meta["control_dt"])
    self.gait_period = float(self.meta["gait_period_s"])
    self.joint_names = list(self.meta["action_joint_order"])
    layout = {e["term"]: e for e in self.meta["obs_layout"]}
    self.hist = int(layout["joint_pos"]["history"])
    self._off = {k: (v["offset"], v["size_per_step"]) for k, v in layout.items()}
    # Which IMU terms this checkpoint wants. Read from the layout rather than
    # assumed: training with an IMU inserts terms and shifts every offset.
    self.needs_gyro = "base_ang_vel" in layout
    self.needs_gravity = "projected_gravity" in layout
    self.needs_imu = self.needs_gyro or self.needs_gravity
    self._terms = [k for k in ("base_ang_vel", "projected_gravity", "command",
                               "phase", "joint_pos", "joint_vel", "actions")
                   if k in layout]
    unknown = set(layout) - set(self._terms)
    if unknown:
      raise ValueError(f"observation terms not implemented here: {sorted(unknown)}")
    self.clip = self.meta.get("clip_actions")
    self.reset()

  # -- state ---------------------------------------------------------------
  def reset(self) -> None:
    h, nj = self.hist, self.nj
    self._buf = {"command": np.zeros((h, 3), np.float32),
                 "phase": np.zeros((h, 2), np.float32),
                 "joint_pos": np.zeros((h, nj), np.float32),
                 "joint_vel": np.zeros((h, nj), np.float32),
                 "actions": np.zeros((h, nj), np.float32)}
    if self.needs_gyro:
      self._buf["base_ang_vel"] = np.zeros((h, 3), np.float32)
    if self.needs_gravity:
      self._buf["projected_gravity"] = np.zeros((h, 3), np.float32)
    self._head = h - 1
    self._filled = 0
    self._last_action = np.zeros(nj, np.float32)
    self.step_i = 0

  def _push(self, **vals) -> None:
    self._head = (self._head + 1) % self.hist
    for k, v in vals.items():
      self._buf[k][self._head] = v
    if self._filled == 0:
      # mjlab's CircularBuffer primes every slot with the first sample.
      for k, v in vals.items():
        self._buf[k][:] = v
    self._filled = min(self._filled + 1, self.hist)

  def _flat(self, key) -> np.ndarray:
    idx = [(self._head + 1 + k) % self.hist for k in range(self.hist)]  # oldest first
    return self._buf[key][idx].reshape(-1)

  # -- the gait clock ------------------------------------------------------
  def phase(self, t: float, cmd) -> np.ndarray:
    if float(np.linalg.norm(cmd)) < 0.1:
      return np.zeros(2, np.float32)
    ph = (t % self.gait_period) / self.gait_period
    return np.array([np.sin(ph * 2 * np.pi), np.cos(ph * 2 * np.pi)], np.float32)

  # -- one control step ----------------------------------------------------
  def step(self, q, qd, cmd, step_i=None, ang_vel=None, gravity=None):
    """q, qd: measured joint state in policy order [rad], [rad/s].
    cmd: (vx, vy, wz); vy must be 0 -- no lateral DoF on this robot.
    ang_vel: body-frame rates [rad/s] from a gyro, when the policy wants them.
    gravity: gravity direction in the body frame, unit length; {0,0,-1} when
      level. Needs an attitude estimate -- it is not the raw accelerometer.
    Returns (joint_targets [rad], observation)."""
    q = np.asarray(q, np.float32)
    qd = np.asarray(qd, np.float32)
    cmd = np.asarray(cmd, np.float32)
    if step_i is None:
      step_i = self.step_i
    t = step_i * self.dt
    extra = {}
    if self.needs_gyro:
      if ang_vel is None:
        raise ValueError("this policy was trained with a gyro; pass ang_vel")
      extra["base_ang_vel"] = np.asarray(ang_vel, np.float32)
    if self.needs_gravity:
      if gravity is None:
        raise ValueError("this policy needs projected_gravity; pass gravity")
      extra["projected_gravity"] = np.asarray(gravity, np.float32)
    self._push(command=cmd, phase=self.phase(t, cmd),
               joint_pos=q - self.default_q, joint_vel=qd,
               actions=self._last_action, **extra)

    obs = np.empty(self.obs_dim, np.float32)
    for key in self._terms:
      off, per = self._off[key]
      obs[off:off + per * self.hist] = self._flat(key)

    x = (obs - self.obs_mean) / self.obs_std
    for w, b in self.layers[:-1]:
      x = x @ w.T + b
      x = np.where(x > 0, x, np.expm1(np.minimum(x, 0.0)))       # ELU
    act = x @ self.layers[-1][0].T + self.layers[-1][1]
    if self.clip:
      act = np.clip(act, -float(self.clip), float(self.clip))

    self._last_action = act.astype(np.float32)
    self.step_i = step_i + 1
    return self.default_q + self.action_scale * act, obs

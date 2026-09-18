#!/usr/bin/env python
"""Precompute the reference trot joint trajectory used to guide RL.

`openloop_trot.py` established that a scripted Cartesian trot walks this robot
at 0.05-0.09 m/s inside the servo envelope. This bakes that same trajectory into
a lookup table so a reward term can pull the policy toward it without running IK
in the training loop:

    q_ref[phase_bin, stride_bin, joint]     shape (P, S, 8), radians

* `phase_bin` samples one gait cycle. Each leg reads the table at its own
  phase offset, so the table itself holds a single leg's cycle applied to all
  four with the diagonal offsets of the task (RR/FL at 0, FR/RL at 0.5).
* `stride_bin` samples stride length, so the reference scales with the
  commanded speed: stride ~= v_cmd * gait_period.
* Joint order is the entity's own: RR_hip, RR_knee, FR_hip, FR_knee, FL_hip,
  FL_knee, RL_hip, RL_knee -- the same order the policy's actions use.

Guidance is a means, not an end: the task anneals this term's weight down so the
velocity-tracking reward takes over and the policy is free to leave the scripted
gait behind wherever it can do better.

Run:  uv run --with mujoco --with numpy \\
          python arduino_os_quad_robot/scripts/make_trot_reference.py
"""
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT = os.path.dirname(HERE)
MJCF = os.path.join(ROBOT, "mjcf", "arduino_os_quad_robot.xml")
OUT = os.path.join(ROBOT, "mjcf", "trot_reference.npz")

LEGS = ("FL", "RL", "RR", "FR")          # entity/model order
PHASE_OFFSET = {"FL": 0.0, "RL": 0.5, "RR": 0.0, "FR": 0.5}   # diagonal trot
STANCE_HEIGHT = 0.14
LIFT = 0.020                              # measured affordable (0.03 saturates)
DUTY = 0.5
N_PHASE = 64
STRIDES = np.array([0.0, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08])


def main() -> int:
  m = mujoco.MjModel.from_xml_path(MJCF)
  dk = mujoco.MjData(m)
  qadr = {}
  for leg in LEGS:
    for j in ("hip", "knee"):
      name = "{}_{}_joint".format(leg, j)
      qadr[name] = m.jnt_qposadr[
        mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]
  sid = {l: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, l) for l in LEGS}
  bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")

  def toe(leg, hip, knee):
    mujoco.mj_resetData(m, dk)
    dk.qpos[:3] = 0.0
    dk.qpos[3] = 1.0
    dk.qpos[4:7] = 0.0
    for l in LEGS:
      dk.qpos[qadr["{}_hip_joint".format(l)]] = hip if l == leg else 0.0
      dk.qpos[qadr["{}_knee_joint".format(l)]] = knee if l == leg else 0.0
    mujoco.mj_kinematics(m, dk)
    return dk.site_xpos[sid[leg]][[0, 2]] - dk.xpos[bid][[0, 2]]

  def ik(leg, target, seed):
    q = np.array(seed, dtype=float)
    for _ in range(80):
      f = toe(leg, q[0], q[1]) - target
      if np.linalg.norm(f) < 1e-6:
        break
      J = np.zeros((2, 2))
      eps = 1e-5
      base = toe(leg, q[0], q[1])
      for i in range(2):
        qp = q.copy()
        qp[i] += eps
        J[:, i] = (toe(leg, qp[0], qp[1]) - base) / eps
      q = np.clip(q - np.linalg.solve(J + 1e-9 * np.eye(2), f), -2.0, 2.0)
    return q

  # nominal toe x under the hip, per leg, at the home pose
  nom_x = {l: toe(l, 0.7748, -1.3474)[0] for l in LEGS}

  joint_names = []
  for leg in LEGS:
    joint_names += ["{}_hip_joint".format(leg), "{}_knee_joint".format(leg)]

  table = np.zeros((N_PHASE, len(STRIDES), 8), dtype=np.float32)
  for si, stride in enumerate(STRIDES):
    seed = {l: (0.7748, -1.3474) for l in LEGS}
    for pi in range(N_PHASE):
      t = pi / N_PHASE
      for li, leg in enumerate(LEGS):
        ph = (t + PHASE_OFFSET[leg]) % 1.0
        if ph < DUTY:                      # stance: foot travels backwards
          s = ph / DUTY
          px = nom_x[leg] + stride * (0.5 - s)
          pz = -STANCE_HEIGHT
        else:                              # swing: forwards, lifted
          s = (ph - DUTY) / (1.0 - DUTY)
          px = nom_x[leg] + stride * (-0.5 + s)
          pz = -STANCE_HEIGHT + LIFT * np.sin(np.pi * s)
        q = ik(leg, np.array([px, pz]), seed[leg])
        seed[leg] = tuple(q)
        table[pi, si, 2 * li] = q[0]
        table[pi, si, 2 * li + 1] = q[1]

  np.savez(OUT, q_ref=table, strides=STRIDES.astype(np.float32),
           joint_names=np.array(joint_names),
           stance_height=np.float32(STANCE_HEIGHT), lift=np.float32(LIFT),
           duty=np.float32(DUTY))
  print("wrote {}  q_ref{}  strides={}".format(OUT, table.shape, STRIDES))
  print("joint order:", joint_names)
  print("hip range over the cycle at stride 0.06: %.3f .. %.3f rad"
        % (table[:, 5, 0].min(), table[:, 5, 0].max()))
  print("knee range: %.3f .. %.3f rad" % (table[:, 5, 1].min(), table[:, 5, 1].max()))
  return 0


if __name__ == "__main__":
  sys.exit(main())

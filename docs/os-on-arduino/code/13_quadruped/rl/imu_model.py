"""The IMU the actor sees, with the errors a real one brings.

The upstream ``base_ang_vel`` / ``projected_gravity`` terms are exact: the gyro
sits at the base origin, perfectly aligned with the base, with no bias. On the
robot the sensor board is screwed into a printed seat (a few degrees of tilt
from print tolerance and screw torque), the gyro's zero drifts after the
start-up calibration, and the gravity direction comes out of an attitude filter.
A policy that leans on the IMU learns the exact values and is then handed
rotated, biased ones.

Two episode-constant errors, sampled per env on every reset:

* mount tilt -- a rotation of up to ``tilt_deg`` about a random axis, applied to
  both the angular velocity and the gravity direction (the chip's frame is not
  the body frame it was assumed to be);
* gyro bias -- a constant offset of up to ``gyro_bias`` rad/s per axis.

The observation delay (``delay_min_lag`` / ``delay_max_lag``) is mjlab's own and
is set in env_cfgs.py. The critic keeps the exact values.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

_ROBOT = SceneEntityCfg("robot")


def _state(env: ManagerBasedRlEnv) -> tuple[torch.Tensor, torch.Tensor]:
  """Per-env (rotation (N, 3, 3), gyro bias (N, 3)), identity / zero until the
  first reset event writes them."""
  rot = getattr(env, "_imu_mount_rot", None)
  if rot is None:
    rot = torch.eye(3, device=env.device).repeat(env.num_envs, 1, 1)
    bias = torch.zeros(env.num_envs, 3, device=env.device)
    env._imu_mount_rot = rot
    env._imu_gyro_bias = bias
  return env._imu_mount_rot, env._imu_gyro_bias


def randomize_imu(env: ManagerBasedRlEnv, env_ids: torch.Tensor | None,
                  tilt_deg: float, gyro_bias: float) -> None:
  """Reset event: draw a new mount tilt and gyro bias for ``env_ids``."""
  rot, bias = _state(env)
  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device)
  n = len(env_ids)
  axis = torch.randn(n, 3, device=env.device)
  axis = axis / torch.linalg.norm(axis, dim=1, keepdim=True)
  angle = torch.rand(n, device=env.device) * math.radians(tilt_deg)
  # Rodrigues: R = I + sin(a) K + (1 - cos(a)) K^2
  k = torch.zeros(n, 3, 3, device=env.device)
  k[:, 0, 1], k[:, 0, 2] = -axis[:, 2], axis[:, 1]
  k[:, 1, 0], k[:, 1, 2] = axis[:, 2], -axis[:, 0]
  k[:, 2, 0], k[:, 2, 1] = -axis[:, 1], axis[:, 0]
  s = torch.sin(angle).view(n, 1, 1)
  c = torch.cos(angle).view(n, 1, 1)
  eye = torch.eye(3, device=env.device).expand(n, 3, 3)
  rot[env_ids] = eye + s * k + (1.0 - c) * (k @ k)
  bias[env_ids] = (torch.rand(n, 3, device=env.device) * 2.0 - 1.0) * gyro_bias


def imu_ang_vel(env: ManagerBasedRlEnv,
                asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
  """Body angular velocity as the tilted, biased gyro reports it [rad/s]."""
  asset: Entity = env.scene[asset_cfg.name]
  rot, bias = _state(env)
  w = asset.data.root_link_ang_vel_b
  return torch.einsum("nji,nj->ni", rot, w) + bias


def imu_projected_gravity(env: ManagerBasedRlEnv,
                          asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
  """Gravity direction in the tilted sensor frame (unit vector)."""
  asset: Entity = env.scene[asset_cfg.name]
  rot, _ = _state(env)
  g = asset.data.projected_gravity_b
  return torch.einsum("nji,nj->ni", rot, g)

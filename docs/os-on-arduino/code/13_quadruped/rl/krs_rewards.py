"""Reward and termination terms the KrsQuad-* tasks add.

The terms in rewards.py are reused as they are; these are the ones the "clean
trot" requirements of the KRS robot needed on top: no hopping (a flight phase),
no sliding feet, no belly or knee on the floor.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactSensor

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def _moving(env: ManagerBasedRlEnv, command_name: str, threshold: float) -> torch.Tensor:
  command = env.command_manager.get_command(command_name)
  total = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
  return (total > threshold).float()


def feet_slip_speed(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Sum of the horizontal speed [m/s] of the feet that are on the ground.

  Upstream ``feet_slip`` prices the SQUARED speed, which on a robot walking at
  0.1 m/s is a few 1e-3 and teaches nothing: a stance foot scraping along at
  3 cm/s costs 1e-3. Linear in speed, the same scrape costs 0.03, comparable to
  the gait terms. Always active (standing still must not slide either). Use with
  a negative weight.
  """
  asset: Entity = env.scene[asset_cfg.name]
  sensor: ContactSensor = env.scene[sensor_name]
  in_contact = (sensor.data.found > 0).float()
  speed = torch.norm(asset.data.site_lin_vel_w[:, asset_cfg.site_ids, :2], dim=-1)
  return torch.sum(speed * in_contact, dim=1)


def flight_phase(
  env: ManagerBasedRlEnv,
  sensor_name: str,
) -> torch.Tensor:
  """1 while no foot touches the ground -- a hop. A trot always has two feet
  down. Use with a negative weight."""
  sensor: ContactSensor = env.scene[sensor_name]
  return (torch.sum(sensor.data.found > 0, dim=1) == 0).float()


def trot_pairs(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  pairs: tuple[tuple[int, int], tuple[int, int]],
  command_name: str,
  command_threshold: float,
) -> torch.Tensor:
  """Penalize the two feet of a diagonal pair disagreeing about contact.

  ``foot_gait`` asks each foot to follow the clock; this asks the diagonal
  partners to move TOGETHER, which is what makes the gait a trot rather than a
  four-beat walk or a pace, whatever the clock's phase. ``pairs`` holds contact
  sensor slot indices. Use with a negative weight; 0 (agree) .. 2 (both pairs
  disagree).
  """
  sensor: ContactSensor = env.scene[sensor_name]
  c = (sensor.data.found > 0).float()
  (a, b), (d, e) = pairs
  cost = torch.abs(c[:, a] - c[:, b]) + torch.abs(c[:, d] - c[:, e])
  return cost * _moving(env, command_name, command_threshold)


def contact_count(
  env: ManagerBasedRlEnv,
  sensor_name: str,
) -> torch.Tensor:
  """Number of the sensor's geoms touching the ground (belly, knees). Use with
  a negative weight."""
  sensor: ContactSensor = env.scene[sensor_name]
  return torch.sum((sensor.data.found > 0).float(), dim=1)


def any_contact(
  env: ManagerBasedRlEnv,
  sensor_name: str,
) -> torch.Tensor:
  """Termination: any of the sensor's geoms touches the ground."""
  sensor: ContactSensor = env.scene[sensor_name]
  return torch.any(sensor.data.found > 0, dim=1)


def base_lin_vel_z_l2(
  env: ManagerBasedRlEnv,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Squared vertical base speed -- bobbing and hopping. Negative weight."""
  asset: Entity = env.scene[asset_cfg.name]
  return torch.square(asset.data.root_link_lin_vel_b[:, 2])

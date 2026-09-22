"""Terms of the upstream velocity recipe that hard-code a 1-2 m/s robot.

The upstream recipe decides "is the robot being asked to move?" with a fixed
0.1 m/s: the gait-phase observation is zeroed below it, and the gait rewards
(foot_gait, foot_clearance, foot_slip, soft_landing), the stand_still penalty
and the standing/walking switch of the pose reward all turn on or off at it.
That is sensible for a Go2 commanded up to 2 m/s. This robot's whole forward
range is 0.09 m/s, so a straight 0.09 m/s command reaches the policy with the
phase clock switched off, no gait reward, the tight standing posture band and
the stand_still penalty -- everything but the velocity tracking says "stand".

env_cfgs.py passes $ARDUINO_QUAD_CMD_THRESHOLD (default 0.1, the upstream
behaviour) to all of them. The phase threshold is part of the deployment
contract -- the firmware builds the same observation -- so the exporter writes
it into the header.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.sensor import ContactSensor

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


def phase(env: ManagerBasedRlEnv, period: float, command_name: str,
          command_threshold: float = 0.1) -> torch.Tensor:
  """Upstream ``phase`` observation with its 0.1 as ``command_threshold``."""
  global_phase = (env.episode_length_buf * env.step_dt) % period / period
  out = torch.zeros(env.num_envs, 2, device=env.device)
  out[:, 0] = torch.sin(global_phase * torch.pi * 2.0)
  out[:, 1] = torch.cos(global_phase * torch.pi * 2.0)
  command = env.command_manager.get_command(command_name)
  stand = torch.linalg.norm(command, dim=1) < command_threshold
  return torch.where(stand.unsqueeze(1), torch.zeros_like(out), out)


def feet_down_when_standing(env: ManagerBasedRlEnv, sensor_name: str, command_name: str,
                            command_threshold: float = 0.1) -> torch.Tensor:
  """Number of feet off the ground while the command says stand still.

  ``stand_still`` only prices joint deviation from home, and the policies trained
  so far stand with one foot held a few millimetres up at the home angles
  (measured: one foot with a contact duty of 0.00 at a zero command). Use with a
  negative weight.
  """
  sensor: ContactSensor = env.scene[sensor_name]
  command = env.command_manager.get_command(command_name)
  total = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
  air = (sensor.data.found == 0).float().sum(dim=1)
  return air * (total <= command_threshold).float()

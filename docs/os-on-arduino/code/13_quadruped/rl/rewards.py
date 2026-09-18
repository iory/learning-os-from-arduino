"""Reward terms this task adds on top of upstream unitree_rl_mjlab.

Both fight the same failure mode: a swing-foot-lift reward makes the policy lift
its foot by SINKING the stance leg, i.e. a permanent bent-knee "Groucho" crouch
instead of a walk. Kept here (not patched into the upstream mdp package) so the
overlay is a pure add-on -- setup.sh only has to copy this directory.
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING

import numpy as np
import torch

from mjlab.entity import Entity
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactSensor

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def base_height_target(
  env: ManagerBasedRlEnv,
  target_height: float,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Penalize CROUCHING: squared deviation of the base height below a target.

  Only the below-target side is penalized (max(target - z, 0)^2), so this pulls
  the base UP toward `target_height` -- extending the stance knee -- without
  penalizing a momentarily taller base. Use with a negative weight.
  """
  asset: Entity = env.scene[asset_cfg.name]
  z = asset.data.root_link_pos_w[:, 2]
  return torch.square(torch.clamp(target_height - z, min=0.0))


def stance_knee_extension(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Penalize a BENT knee on the STANCE (in-contact) leg -> drives the stance
  knee toward STRAIGHT (extension = 0 rad), while leaving the swing knee free to
  flex and lift the foot.

  asset_cfg.joint_ids must be the two knee joints in the SAME left/right order
  as the foot contact sensor's slots. Use with a negative weight.
  """
  asset: Entity = env.scene[asset_cfg.name]
  sensor: ContactSensor = env.scene[sensor_name]
  knee = asset.data.joint_pos[:, asset_cfg.joint_ids]  # (B, 2), 0 = straight
  in_contact = (sensor.data.current_contact_time > 0.0).float()  # (B, 2)
  return torch.sum(in_contact * torch.square(knee), dim=1)


##
# Velocity tracking that cannot be satisfied by rocking in place.
#
# The upstream term scores the INSTANTANEOUS base velocity against the command.
# On this robot that turned out to be exploitable: measured on a 2500-iteration
# policy, the net forward displacement over 6 s was -0.014 m (i.e. nothing) while
# the body-frame vx had a standard deviation of 0.107 m/s (5th/95th percentile
# -0.18 / +0.13). The robot was rocking fore-aft in place, and since the tracking
# reward is a narrow Gaussian around the commanded speed, every time the rocking
# velocity swept through the command it collected full marks. The logged
# error_vel_xy of 0.12 looked like "partially tracking" and was really "violently
# oscillating around zero".
#
# Filtering the velocity before comparing removes the exploit outright: a
# zero-mean oscillation filters to zero and earns nothing, while actually
# travelling at the commanded speed earns the same as before.
##


class track_linear_velocity_filtered:
  """Track a low-pass filtered base linear velocity.

  ``filter_time`` is the time constant. It wants to be long enough to average
  out a gait cycle (this robot trots at ~0.6 s) but short enough that the reward
  still responds within an episode.
  """

  def __init__(self, cfg, env: ManagerBasedRlEnv) -> None:
    self._vel = torch.zeros(env.num_envs, 2, device=env.device)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    if env_ids is None:
      self._vel.zero_()
    else:
      self._vel[env_ids] = 0.0

  def __call__(
    self,
    env: ManagerBasedRlEnv,
    std: float,
    command_name: str,
    filter_time: float,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
  ) -> torch.Tensor:
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    assert command is not None
    vel = asset.data.root_link_lin_vel_b[:, :2]
    alpha = env.step_dt / (filter_time + env.step_dt)
    self._vel = (1.0 - alpha) * self._vel + alpha * vel
    error = torch.sum(torch.square(command[:, :2] - self._vel), dim=1)
    log = env.extras.get("log")
    if log is not None:
      log["Metrics/filtered_vx_mean"] = self._vel[:, 0].mean()
    return torch.exp(-error / std**2)


class base_vel_oscillation_l2:
  """Penalize the part of the base velocity that averages out to nothing.

  Squared difference between the instantaneous and the low-pass filtered base
  velocity, i.e. exactly the rocking that the filtered tracking term stops
  rewarding. Use with a negative weight; this makes the shake actively cost
  something rather than merely being unpaid.
  """

  def __init__(self, cfg, env: ManagerBasedRlEnv) -> None:
    self._vel = torch.zeros(env.num_envs, 2, device=env.device)

  def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
    if env_ids is None:
      self._vel.zero_()
    else:
      self._vel[env_ids] = 0.0

  def __call__(
    self,
    env: ManagerBasedRlEnv,
    filter_time: float,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
  ) -> torch.Tensor:
    asset: Entity = env.scene[asset_cfg.name]
    vel = asset.data.root_link_lin_vel_b[:, :2]
    alpha = env.step_dt / (filter_time + env.step_dt)
    self._vel = (1.0 - alpha) * self._vel + alpha * vel
    return torch.sum(torch.square(vel - self._vel), dim=1)


##
# Staying inside what an STS3215 can sustain.
#
# A flat L2 on torque or joint speed taxes every motion equally, so to bite at
# saturation it has to be big enough to also suppress the normal gait -- and
# tuned small enough not to, it does nothing. Measured on a policy that did
# track velocity: peak joint speed 4.99 rad/s (106 % of the servo's no-load
# speed) and peak torque 2.93 Nm (100 % of stall), while the L2 penalties
# contributed -0.02 against a tracking reward of 3.0.
#
# These two penalize only the EXCESS over a sustainable operating point, so a
# gait that stays inside the servo's envelope pays nothing at all and one that
# runs it flat out pays a lot.
##


def _excess_sq(value: torch.Tensor, limit: float) -> torch.Tensor:
  return torch.sum(torch.square(torch.clamp(value.abs() - limit, min=0.0)), dim=1)


def joint_torque_excess(
  env: ManagerBasedRlEnv,
  limit: float,
  back_emf_damping: float = 0.0,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Squared NET joint torque above ``limit`` [Nm], as a stand-in for heating.

  ``actuator_force`` is NOT the right quantity here, and using it was a real
  bug. The torque-speed curve is modelled as a passive viscous damper on the
  joint (see robot_cfg.py), so at speed the actuator has to produce that damping
  back on top of whatever the leg actually needs: at 2.6 rad/s, 0.624 * 2.6 =
  1.6 Nm of the reading is the actuator fighting our own damper. Measured on the
  scripted trot, ``actuator_force`` sat at the 2.94 Nm stall value while the net
  torque reaching the link was 0.24-0.38 Nm rms -- a third of the continuous
  rating. This robot weighs 1 kg; its servos are nowhere near overloaded.

  Worse, the sign is wrong. In a real motor i = (V - Ke*w)/R, so current -- and
  therefore heating -- FALLS with speed, while ``actuator_force`` in this model
  RISES with it. Thresholding ``actuator_force`` against the continuous rating
  is a speed penalty wearing a thermal costume, and it is what froze an earlier
  policy solid.

  net = actuator_force - b * qd corresponds to the real Kt*i, so that is the
  quantity to compare against the continuous rating. ``back_emf_damping`` is the
  nominal b; per-environment damping randomization makes this approximate, which
  is fine for a soft limit.
  """
  asset: Entity = env.scene[asset_cfg.name]
  net = asset.data.actuator_force - back_emf_damping * asset.data.joint_vel
  return _excess_sq(net, limit)


def joint_speed_excess(
  env: ManagerBasedRlEnv,
  limit: float,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """Squared joint speed above ``limit`` [rad/s]. A servo near its no-load speed
  has no torque left to reject a disturbance, so a gait that lives there in
  simulation falls over on hardware."""
  asset: Entity = env.scene[asset_cfg.name]
  return _excess_sq(asset.data.joint_vel, limit)


##
# Reference-gait guidance.
#
# Velocity tracking alone keeps finding degenerate optima on this robot -- rock
# in place, tremble to match the gait clock, walk on three legs -- because a
# real trot is a narrow, torque-limited target and everything around it is
# cheap. arduino_os_quad_robot/scripts/openloop_trot.py already produced a gait
# that demonstrably walks it at 0.05-0.09 m/s inside the servo envelope, so
# there is no reason to make RL rediscover the shape of that motion.
#
# This term pulls the joints toward that scripted trajectory. Its weight is
# annealed to zero by the curriculum: guidance gets the policy into the right
# basin, then gets out of the way so velocity tracking, the servo-envelope
# penalties and the disturbance rejection decide the final gait.
##


class reference_trot:
  """Match the precomputed scripted trot at the current gait phase and speed.

  The table is built by arduino_os_quad_robot/scripts/make_trot_reference.py and
  lives next to the MJCF. Its joint columns are permuted to the entity's own
  joint order at construction, so a change in either order cannot silently
  scramble the reference.
  """

  def __init__(self, cfg, env: ManagerBasedRlEnv) -> None:
    from .robot_cfg import ARDUINO_QUAD_XML

    path = os.environ.get(
      "ARDUINO_QUAD_TROT_REF", str(ARDUINO_QUAD_XML.parent / "trot_reference.npz"))
    if not os.path.exists(path):
      raise FileNotFoundError(
        "reference trot table missing: {}. Generate it with "
        "arduino_os_quad_robot/scripts/make_trot_reference.py".format(path))
    data = np.load(path)
    table_names = [str(n) for n in data["joint_names"]]

    asset: Entity = env.scene[cfg.params["asset_cfg"].name]
    entity_names = list(asset.joint_names)
    missing = set(entity_names) - set(table_names)
    if missing:
      raise ValueError(
        "reference trot table does not cover {}; regenerate it".format(sorted(missing)))
    perm = [table_names.index(n) for n in entity_names]

    q_ref = np.asarray(data["q_ref"])[:, :, perm]        # (P, S, J)
    self._q_ref = torch.as_tensor(q_ref, dtype=torch.float, device=env.device)
    self._strides = torch.as_tensor(
      np.asarray(data["strides"]), dtype=torch.float, device=env.device)
    self._n_phase = self._q_ref.shape[0]

  def __call__(
    self,
    env: ManagerBasedRlEnv,
    std: float,
    command_name: str,
    period: float,
    command_threshold: float,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
  ) -> torch.Tensor:
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    assert command is not None

    # Same clock as the `phase` observation and the foot_gait reward.
    phase = (env.episode_length_buf * env.step_dt / period) % 1.0
    pidx = (phase * self._n_phase).long().clamp(0, self._n_phase - 1)

    # Stride the scripted gait would use for the commanded speed.
    stride = command[:, 0].abs() * period
    sidx = torch.bucketize(stride, self._strides).clamp(0, self._strides.numel() - 1)

    q_ref = self._q_ref[pidx, sidx]                        # (B, J)
    error = torch.sum(torch.square(asset.data.joint_pos - q_ref), dim=1)
    # Forward only: the table encodes a forward stride, and mirroring it for
    # reverse would be a guess. Reverse and turning are left to the tracking
    # rewards.
    active = (command[:, 0] > command_threshold).float()
    return torch.exp(-error / std**2) * active


def foot_air_time_excess(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  limit: float,
) -> torch.Tensor:
  """Penalize a foot that stays off the ground far longer than a gait cycle.

  Every walking policy trained for this robot so far converged to a THREE-legged
  gait: one rear foot with a measured contact duty of exactly 0.00, carried
  through the whole episode. It tracks forward velocity fine and yaws away at
  0.35 rad/s because of the asymmetry, and on hardware it would be a robot
  dragging a leg.

  Nothing else in the reward set objects to it. ``feet_air_time`` takes a min
  over feet, so a permanently airborne foot is invisible to it; ``foot_gait``
  scores an average over feet, so three good feet outvote one bad one. This term
  is linear (not squared) in the excess so that a foot held up for a whole
  episode does not produce a reward spike large enough to destabilise training.
  Use with a negative weight; ``limit`` should be a couple of gait periods.
  """
  sensor: ContactSensor = env.scene[sensor_name]
  air = sensor.data.current_air_time
  return torch.sum(torch.clamp(air - limit, min=0.0), dim=1)

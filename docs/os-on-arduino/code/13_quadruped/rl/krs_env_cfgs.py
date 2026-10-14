"""krs_quad_robot velocity environments (KrsQuad-*).

The KRS robot's own task. It is built from the same upstream velocity recipe as
the Feetech tasks (env_cfgs.py) and reuses that file's reward FUNCTIONS, but
every number is set here for this machine, and nothing is read from the
ArduinoQuad-* configs or their checkpoints.

What is different from ArduinoQuad-Walk, and why:

1. **IMU always on.** The actor sees the gyro (``base_ang_vel``) and the
   gravity direction (``projected_gravity``), each with a 3-step history -- the
   105-dim layout of the Feetech IMU policy (include/arduino_quad_policy_imu.h),
   so the firmware's observation code fits as it is. The sensor errors of
   imu_model.py (mount tilt, gyro bias) and up to one control step of delay are
   on by default, since the board's seat on the KRS body is not measured.
2. **A quarter of the torque.** KRS-3302: 0.54-0.66 Nm depending on the supply
   (6 V adapter .. 7.4 V), against the STS3215's 2.94. The effort limit is
   randomized over that band; the torque/speed penalties are scaled to it.
3. **Clean trot.** The user asked for a regular diagonal trot on all four legs:
   no hopping, no dragged or sliding foot, no three-legged gait, no belly or
   knee on the floor, little body sway, no trembling. Each of those has its own
   term below (see "rewards"), and the belly and knees are real collision
   geometry (robot cfg) so their contacts can be seen at all.
4. **Mass uncertainty.** 0.61 kg of which 78 g of battery and 25 g of Arduino
   sit to the right (-y), with the battery position and the Arduino's mass both
   assumptions; base mass and CoM are randomized wide.
"""

import dataclasses
import math
import os

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
import src.tasks.velocity.mdp as vmdp
from src.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from . import imu_model
from . import krs_rewards
from . import rewards as local_mdp
from . import slow_robot
from .krs_robot_cfg import (
  BASE_BODY,
  FRONT_LEG_CAPS,
  HOME_BASE_HEIGHT,
  KRS_QUAD_ACTION_SCALE,
  NO_LOAD_SPEED,
  RATED_TORQUE,
  STALL_TORQUE,
  STALL_TORQUE_6V,
  get_krs_quad_robot_cfg,
)

# Contact-sensor slot order = the order the toes appear in the MJCF, NOT the
# order of the pattern tuple (env_cfgs.py explains the trap). build_mjcf.py
# writes the legs as FL, RL, RR, FR; krs_smoke.py asserts it.
FOOT_ORDER = ("FL", "RL", "RR", "FR")
TOE_GEOMS = tuple(f"{leg}_toe" for leg in FOOT_ORDER)
TOE_SITES = FOOT_ORDER
# Diagonal trot: FL+RR on phase 0, RL+FR on phase 0.5.
GAIT_OFFSET = [0.0, 0.5, 0.0, 0.5]
DIAGONAL_PAIRS = ((0, 2), (1, 3))  # slot indices: (FL, RR), (RL, FR)
KNEE_GEOMS = tuple(f"{leg}_knee_ball" for leg in FOOT_ORDER)

# Gait period [s]. The legs are the Feetech robot's length (0.21 m), where
# 0.30-0.32 s was measured to be the natural trot.
GAIT_PERIOD = float(os.environ.get("KRS_QUAD_GAIT_PERIOD", "0.32"))
# Peak toe height in swing [m] (the toe site sits on the floor when standing).
SWING_HEIGHT = float(os.environ.get("KRS_QUAD_SWING_HEIGHT", "0.02"))

# Command envelope. Forward up to 0.12 m/s (0.6 body lengths/s), backward to
# half that, yaw +-0.5 rad/s, no lateral (no abduction joint).
_CMD_VX_MAX = float(os.environ.get("KRS_QUAD_CMD_VX_MAX", "0.12"))
CMD_LIN_X = (-0.5 * _CMD_VX_MAX, _CMD_VX_MAX)
_CMD_WZ_MAX = float(os.environ.get("KRS_QUAD_CMD_WZ_MAX", "0.5"))
CMD_ANG_Z = (-_CMD_WZ_MAX, _CMD_WZ_MAX)
# Below this |command| the robot is told to stand: the phase clock reads zero
# and the gait terms switch off. 0.1 (upstream) would be most of the forward
# range of this robot (slow_robot.py), so it is lowered.
CMD_THRESHOLD = 0.03

_STD_STANDING = {r"(FL|FR|RL|RR)_hip_joint": 0.05, r"(FL|FR|RL|RR)_knee_joint": 0.08}
_STD_WALKING = {r"(FL|FR|RL|RR)_hip_joint": 0.3, r"(FL|FR|RL|RR)_knee_joint": 0.45}

# IMU errors (imu_model.py). The KRS body's sensor seat is not measured.
IMU_TILT_DEG = float(os.environ.get("KRS_QUAD_IMU_TILT_DEG", "3.0"))
IMU_GYRO_BIAS = float(os.environ.get("KRS_QUAD_IMU_GYRO_BIAS", "0.03"))
IMU_DELAY_MAX = int(os.environ.get("KRS_QUAD_IMU_DELAY_MAX", "1"))


def krs_quad_walk_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  cfg = make_velocity_env_cfg()
  cfg.scene.entities = {"robot": get_krs_quad_robot_cfg()}

  # --- flat ground, no height scan ------------------------------------------
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None
  cfg.scene.sensors = tuple(
    s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan")
  del cfg.observations["actor"].terms["height_scan"]
  del cfg.observations["critic"].terms["height_scan"]
  cfg.curriculum.pop("terrain_levels", None)

  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  # --- contact sensors ------------------------------------------------------
  def ground_sensor(name, geoms, **kw):
    return ContactSensorCfg(
      name=name,
      primary=ContactMatch(mode="geom", pattern=geoms, entity="robot"),
      secondary=ContactMatch(mode="body", pattern="terrain"),
      reduce="netforce", num_slots=1, **kw)

  cfg.scene.sensors = (cfg.scene.sensors or ()) + (
    ground_sensor("feet_ground_contact", TOE_GEOMS,
                  fields=("found", "force"), track_air_time=True),
    ground_sensor("belly_ground_contact", ("base_box",), fields=("found",)),
    ground_sensor("knee_ground_contact", KNEE_GEOMS, fields=("found",)),
    # Front leg capsules can only touch rear ones (krs_robot_cfg.py), so any
    # contact they report is a leg striking another leg.
    ContactSensorCfg(
      name="leg_leg_contact",
      primary=ContactMatch(mode="geom", pattern=FRONT_LEG_CAPS, entity="robot"),
      reduce="netforce", num_slots=1, fields=("found",)),
  )

  # --- observations: the IMU version's 105-dim layout -----------------------
  actor = cfg.observations["actor"]
  actor.history_length = 3
  for t in ("base_ang_vel", "projected_gravity"):
    # Copies, so the critic keeps the exact sensor.
    actor.terms[t] = dataclasses.replace(actor.terms[t], delay_max_lag=IMU_DELAY_MAX)
  if IMU_TILT_DEG > 0 or IMU_GYRO_BIAS > 0:
    cfg.events["imu_error"] = EventTermCfg(
      mode="reset", func=imu_model.randomize_imu,
      params={"tilt_deg": IMU_TILT_DEG, "gyro_bias": IMU_GYRO_BIAS})
    actor.terms["base_ang_vel"].func = imu_model.imu_ang_vel
    actor.terms["base_ang_vel"].params = {}
    actor.terms["projected_gravity"].func = imu_model.imu_projected_gravity
    actor.terms["projected_gravity"].params = {}
  # Noise scaled to this servo: ICS reports 4000 counts over 270 deg
  # (0.0012 rad) plus backlash; speed is differentiated position.
  actor.terms["joint_pos"].noise.n_min = -0.02
  actor.terms["joint_pos"].noise.n_max = 0.02
  actor.terms["joint_vel"].noise.n_min = -0.4
  actor.terms["joint_vel"].noise.n_max = 0.4
  actor.terms["base_ang_vel"].noise.n_min = -0.1
  actor.terms["base_ang_vel"].noise.n_max = 0.1
  for group in ("actor", "critic"):
    term = cfg.observations[group].terms["phase"]
    term.func = slow_robot.phase
    term.params["period"] = GAIT_PERIOD
    term.params["command_threshold"] = CMD_THRESHOLD
  cfg.observations["critic"].terms["foot_height"].params["asset_cfg"].site_names = TOE_SITES

  # --- actions ----------------------------------------------------------------
  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = KRS_QUAD_ACTION_SCALE

  # --- commands ---------------------------------------------------------------
  twist = cfg.commands["twist"]
  assert isinstance(twist, UniformVelocityCommandCfg)
  twist.ranges.lin_vel_x = CMD_LIN_X
  twist.ranges.lin_vel_y = (0.0, 0.0)
  twist.ranges.ang_vel_z = CMD_ANG_Z
  twist.ranges.heading = None
  twist.heading_command = False
  twist.rel_standing_envs = 0.1
  twist.viz.z_offset = 0.2
  cfg.curriculum["command_vel"].params["velocity_stages"] = [
    {"step": 0, "lin_vel_x": (-0.03, 0.06), "lin_vel_y": (0.0, 0.0),
     "ang_vel_z": (-0.2, 0.2)},
    {"step": 500 * 24, "lin_vel_x": CMD_LIN_X, "lin_vel_y": (0.0, 0.0),
     "ang_vel_z": CMD_ANG_Z},
  ]

  cfg.viewer.body_name = BASE_BODY
  cfg.viewer.distance = 0.8
  cfg.viewer.elevation = -12.0

  # --- domain randomization ---------------------------------------------------
  cfg.events["foot_friction"].params["asset_cfg"].geom_names = TOE_GEOMS
  cfg.events["foot_friction"].params["ranges"] = (0.35, 1.0)  # bare PLA .. rubber
  # CoM: the battery and Arduino sit right of centre and the battery position is
  # only an estimate, so +-2.5 cm fore-aft and sideways, +-1.5 cm vertically.
  cfg.events["base_com"].params["asset_cfg"].body_names = (BASE_BODY,)
  cfg.events["base_com"].params["ranges"] = {
    0: (-0.025, 0.025), 1: (-0.025, 0.025), 2: (-0.015, 0.015)}
  # Base mass (0.383 kg incl. battery and boards) x0.8..x1.3: cables, a heavier
  # board, a different battery.
  cfg.events["base_mass"] = EventTermCfg(
    mode="startup", func=dr.body_mass,
    params={"asset_cfg": SceneEntityCfg("robot", body_names=(BASE_BODY,)),
            "ranges": (0.8, 1.3), "operation": "scale"})
  # Servo zero: horn spline (ICS horns have 24 teeth) and the hand calibration.
  cfg.events["encoder_bias"].params["bias_range"] = (-0.03, 0.03)
  # Pushes for a 0.6 kg robot. $KRS_QUAD_PUSH_SCALE multiplies them.
  push = float(os.environ.get("KRS_QUAD_PUSH_SCALE", "1"))
  cfg.events["push_robot"].params["velocity_range"] = {
    k: (push * lo, push * hi) for k, (lo, hi) in {
      "x": (-0.2, 0.2), "y": (-0.2, 0.2), "z": (-0.1, 0.1),
      "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.4, 0.4),
    }.items()}
  # ICS position loop: gains are ESTIMATES (krs_robot_cfg.py), so x0.5..x2.
  cfg.events["servo_gains"] = EventTermCfg(
    mode="startup", func=dr.pd_gains,
    params={"asset_cfg": SceneEntityCfg("robot"), "kp_range": (0.5, 2.0),
            "kd_range": (0.5, 2.0), "operation": "scale"})
  # Supply voltage: 5.9 V adapter .. 7.2 V Ni-MH -> 0.54 .. 0.66 Nm.
  cfg.events["servo_effort"] = EventTermCfg(
    mode="startup", func=dr.effort_limits,
    params={"asset_cfg": SceneEntityCfg("robot"),
            "effort_limit_range": (STALL_TORQUE_6V / STALL_TORQUE, 1.0),
            "operation": "scale"})
  cfg.events["servo_armature"] = EventTermCfg(
    mode="startup", func=dr.joint_armature,
    params={"asset_cfg": SceneEntityCfg("robot"), "ranges": (0.5, 2.0),
            "operation": "scale"})
  cfg.events["servo_damping"] = EventTermCfg(
    mode="startup", func=dr.joint_damping,
    params={"asset_cfg": SceneEntityCfg("robot"), "ranges": (0.8, 1.3),
            "operation": "scale"})
  cfg.events["servo_friction"] = EventTermCfg(
    mode="startup", func=dr.joint_friction,
    params={"asset_cfg": SceneEntityCfg("robot"), "ranges": (0.0, 0.02),
            "operation": "abs"})

  # --- rewards ----------------------------------------------------------------
  r = cfg.rewards
  # Tracking. The instantaneous term gives the gradient; the filtered one pays
  # only for velocity that survives a gait cycle, i.e. real travel (rewards.py).
  r["track_linear_velocity"].weight = 1.0
  r["track_linear_velocity"].params["std"] = 0.3 * CMD_LIN_X[1]
  r["track_angular_velocity"].weight = 1.5
  r["track_angular_velocity"].params["std"] = 0.5 * CMD_ANG_Z[1]
  r["track_lin_vel_filtered"] = RewardTermCfg(
    func=local_mdp.track_linear_velocity_filtered, weight=3.0,
    params={"command_name": "twist", "std": 0.25 * CMD_LIN_X[1],
            "filter_time": GAIT_PERIOD})
  # Body: level, steady, at standing height, not bobbing.
  r["body_orientation_l2"].params["asset_cfg"].body_names = (BASE_BODY,)
  r["body_orientation_l2"].weight = -10.0
  r["body_ang_vel"].params["asset_cfg"].body_names = (BASE_BODY,)
  r["body_ang_vel"].weight = -0.1
  r["angular_momentum"].weight = -0.01
  r["base_lin_vel_z"] = RewardTermCfg(func=krs_rewards.base_lin_vel_z_l2, weight=-4.0)
  r["base_vel_oscillation"] = RewardTermCfg(
    func=local_mdp.base_vel_oscillation_l2, weight=-4.0,
    params={"filter_time": GAIT_PERIOD})
  r["base_height"] = RewardTermCfg(
    func=local_mdp.base_height_target, weight=-100.0,
    params={"target_height": HOME_BASE_HEIGHT})
  # Gait: diagonal pairs on the clock, both partners together, real swing.
  r["foot_gait"].weight = 1.0
  r["foot_gait"].params["period"] = GAIT_PERIOD
  r["foot_gait"].params["offset"] = GAIT_OFFSET
  r["trot_pairs"] = RewardTermCfg(
    func=krs_rewards.trot_pairs, weight=-0.5,
    params={"sensor_name": "feet_ground_contact", "pairs": DIAGONAL_PAIRS,
            "command_name": "twist", "command_threshold": CMD_THRESHOLD})
  r["feet_air_time"] = RewardTermCfg(
    func=vmdp.feet_air_time, weight=2.0,
    params={"sensor_name": "feet_ground_contact", "threshold": GAIT_PERIOD / 2.0,
            "command_name": "twist", "command_threshold": CMD_THRESHOLD})
  r["feet_swing_height"] = RewardTermCfg(
    func=vmdp.feet_swing_height, weight=-2.0,
    params={"sensor_name": "feet_ground_contact", "target_height": SWING_HEIGHT,
            "command_name": "twist", "command_threshold": CMD_THRESHOLD,
            "asset_cfg": SceneEntityCfg("robot", site_names=TOE_SITES)})
  r["foot_clearance"].weight = -1.0
  r["foot_clearance"].params["target_height"] = SWING_HEIGHT
  r["foot_clearance"].params["asset_cfg"].site_names = TOE_SITES
  # A foot held up for more than two cycles: three-legged gait / dragged leg.
  r["foot_air_time_excess"] = RewardTermCfg(
    func=local_mdp.foot_air_time_excess, weight=-2.0,
    params={"sensor_name": "feet_ground_contact", "limit": 2.0 * GAIT_PERIOD})
  # Feet: no sliding, no hopping, land softly.
  r.pop("foot_slip")
  r["feet_slip_speed"] = RewardTermCfg(
    func=krs_rewards.feet_slip_speed, weight=-1.5,
    params={"sensor_name": "feet_ground_contact",
            "asset_cfg": SceneEntityCfg("robot", site_names=TOE_SITES)})
  r["flight_phase"] = RewardTermCfg(
    func=krs_rewards.flight_phase, weight=-2.0,
    params={"sensor_name": "feet_ground_contact"})
  r["soft_landing"].weight = -0.05
  # Belly and knees stay off the floor.
  r["knee_contact"] = RewardTermCfg(
    func=krs_rewards.contact_count, weight=-2.0,
    params={"sensor_name": "knee_ground_contact"})
  r["leg_leg_contact"] = RewardTermCfg(
    func=krs_rewards.contact_count, weight=-5.0,
    params={"sensor_name": "leg_leg_contact"})
  # Smoothness: no 50 Hz chatter, no trembling.
  r["action_rate_l2"].weight = -0.2
  r["action_acc_l2"] = RewardTermCfg(func=envs_mdp.action_acc_l2, weight=-0.1)
  r["joint_vel_l2"] = RewardTermCfg(func=envs_mdp.joint_vel_l2, weight=-0.002)
  r["joint_acc_l2"].weight = -1e-6
  # Stay inside the servo: net torque under the RMS budget, speed under 80 % of
  # no-load, angles well inside +-135 deg.
  r["torque_excess"] = RewardTermCfg(
    func=local_mdp.joint_torque_excess, weight=-20.0,
    params={"limit": RATED_TORQUE, "back_emf_damping": STALL_TORQUE / NO_LOAD_SPEED})
  r["speed_excess"] = RewardTermCfg(
    func=local_mdp.joint_speed_excess, weight=-0.3,
    params={"limit": 0.8 * NO_LOAD_SPEED})
  r["joint_pos_limits"].weight = -10.0
  # Posture is a regularizer, not an objective.
  r["pose"].weight = 0.3
  r["pose"].params["std_standing"] = _STD_STANDING
  r["pose"].params["std_walking"] = _STD_WALKING
  r["pose"].params["std_running"] = _STD_WALKING
  r["pose"].params["walking_threshold"] = CMD_THRESHOLD
  for term in ("foot_gait", "foot_clearance", "soft_landing", "stand_still"):
    r[term].params["command_threshold"] = CMD_THRESHOLD
  r["stand_still"].weight = -0.5
  r["stand_feet_down"] = RewardTermCfg(
    func=slow_robot.feet_down_when_standing, weight=-0.5,
    params={"sensor_name": "feet_ground_contact", "command_name": "twist",
            "command_threshold": CMD_THRESHOLD})
  r["alive"] = RewardTermCfg(func=envs_mdp.is_alive, weight=0.2)
  r["is_terminated"].weight = -50.0

  # --- terminations -----------------------------------------------------------
  cfg.terminations["fell_over"].params["limit_angle"] = math.radians(35.0)
  cfg.terminations["low_base"] = TerminationTermCfg(
    func=envs_mdp.root_height_below_minimum, params={"minimum_height": 0.07})
  cfg.terminations["belly_contact"] = TerminationTermCfg(
    func=krs_rewards.any_contact, params={"sensor_name": "belly_ground_contact"})

  if play:
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.curriculum = {}
    twist.ranges.lin_vel_x = CMD_LIN_X
    twist.ranges.ang_vel_z = CMD_ANG_Z
  return cfg


def krs_quad_robust_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Walk, started from off-nominal states. Fine-tune a Walk checkpoint here
  (same spaces), with $KRS_QUAD_PUSH_SCALE=3 as the Feetech IMU policy was."""
  cfg = krs_quad_walk_env_cfg(play=play)
  cfg.events["reset_base"].params["pose_range"] = {
    "x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (0.0, 0.01),
    "roll": (-0.1, 0.1), "pitch": (-0.1, 0.1), "yaw": (-3.14, 3.14)}
  cfg.events["reset_base"].params["velocity_range"] = {
    "x": (-0.1, 0.1), "y": (-0.1, 0.1), "z": (-0.05, 0.05),
    "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.3, 0.3)}
  cfg.events["reset_robot_joints"].params["position_range"] = (-0.1, 0.1)
  cfg.events["reset_robot_joints"].params["velocity_range"] = (-0.3, 0.3)
  return cfg

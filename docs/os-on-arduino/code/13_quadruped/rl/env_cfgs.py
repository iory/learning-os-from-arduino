"""arduino_os_quad_robot velocity environment configurations.

Built on the upstream velocity recipe, taking the quadruped-specific parts from
the Unitree Go2 config (four-foot contact sensor, trot phase offsets) and then
changing the three things that make this robot different:

1. **No IMU.** The hardware is an Arduino Uno R4 WiFi driving eight Feetech
   STS3215 servos over a serial bus; there is no inertial sensor. So the ACTOR
   observation drops ``base_ang_vel`` (gyro) and ``projected_gravity`` -- a
   policy that used them in sim could not be run on the robot at all. What is
   left is pure proprioception: joint angles, joint speeds, the last action, the
   velocity command and the gait-phase clock, with a 3-step history so the
   policy can read contact events out of the joint traces. The CRITIC keeps
   everything, including base linear/angular velocity: it is only used during
   training (asymmetric actor-critic), so privileged information there is free.

2. **Two sagittal joints per leg.** There is no abduction, so lateral velocity
   is not a thing this machine can do: ``lin_vel_y`` is pinned to zero rather
   than left in the command distribution as an unachievable target. Yaw is
   still commanded -- it comes out as differential stride length.

3. **Small, light, slow.** 1.04 kg, 0.14 m stance height, and servos that top
   out at 4.717 rad/s. Command range, swing height, gait period and the
   termination heights are all scaled to that: a 0.6 s trot with a ~0.25 rad hip
   swing gives roughly 0.16 m/s, so the command tops out at 0.25 m/s
   (~1.3 body lengths/s) instead of the 2 m/s of the upstream recipe.

The domain randomization is aimed squarely at sim2real for a hobby servo: the
servo's internal P/D gains, its reflected rotor inertia, its dry friction and
its back-EMF damping are all estimates (see robot_cfg.py), so all four are
randomized wide, together with a 5-20 ms command delay, foot friction from
"bare plastic" to "rubber", and up to +0.25 kg of battery/electronics that the
CAD export does not contain.
"""

import math
import os

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
import src.tasks.velocity.mdp as vmdp
from src.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from . import rewards as local_mdp
from .robot_cfg import (
  ARDUINO_QUAD_ACTION_SCALE,
  BASE_BODY,
  HOME_BASE_HEIGHT,
  NO_LOAD_SPEED,
  RATED_TORQUE,
  STALL_TORQUE,
  get_arduino_quad_robot_cfg,
)

# Contact-sensor slot order. THIS IS NOT THE ORDER YOU WRITE IN THE CONFIG:
# ContactSensor resolves its `pattern` through Entity.find_geoms(), which
# returns matches in MODEL order, so the slots come out in the order the feet
# appear in the MJCF -- RR, FR, FL, RL, inherited from the CAD export's joint
# order -- no matter how the tuple below is written.
#
# That matters because feet_gait's `offset` is positional. A previous version
# listed the feet as (FR, FL, RR, RL) with offsets [0, 0.5, 0.5, 0], intending a
# diagonal trot; applied to the real slot order it asked for RR+RL together and
# FR+FL together, i.e. a BOUND. Nothing errors, the reward just quietly rewards
# a gait a 1 kg servo quadruped cannot do, and the policy settles for standing
# still instead. scripts/quad_smoke.sh now asserts this order at runtime.
#
# Diagonal trot in the real slot order: FL+RR on phase 0, RL+FR on phase 0.5.
# (The order changed when the yaw flip renamed the legs -- quad_smoke.sh caught
#  it, which is exactly why that assertion exists.)
FOOT_ORDER = ("FL", "RL", "RR", "FR")
TOE_GEOMS = tuple("{}_toe".format(leg) for leg in FOOT_ORDER)
TOE_SITES = FOOT_ORDER
GAIT_OFFSET = [0.0, 0.5, 0.0, 0.5]

# Gait clock period [s]. MEASURED, not assumed: a policy trained with a 0.6 s
# clock actually trotted at 3.43 Hz (period 0.292 s), i.e. at twice the clock
# rate, so foot_gait only agreed with it on alternate cycles and never locked
# the phase -- one foot's contact phase had a standard deviation of 0.44
# (i.e. random). Stride frequency scales roughly as 1/sqrt(leg length): Go2 has
# 0.4 m legs and trots at 2-3 Hz, so 0.19 m legs want 3-4 Hz. 0.6 s was simply
# too slow. $ARDUINO_QUAD_GAIT_PERIOD overrides it.
GAIT_PERIOD = float(os.environ.get("ARDUINO_QUAD_GAIT_PERIOD", "0.30"))
# m, ABSOLUTE toe height (the toe site sits 3.4 mm off the ground when
# standing), i.e. ~19 mm of lift at the default. The scripted trot shows 30 mm
# of lift at a 0.5 s period saturates the servos, 20 mm does not -- but that was
# measured on a hand-written gait, and a learned one may spend the servo budget
# differently. This is what makes the walk look like a shuffle rather than a
# stride, so it is a knob: $ARDUINO_QUAD_SWING_HEIGHT.
SWING_HEIGHT = float(os.environ.get("ARDUINO_QUAD_SWING_HEIGHT", "0.022"))

# Command envelope, measured -- NOT guessed. scripts/quad_openloop_trot.py runs a
# hand-written diagonal trot straight on the MJCF and reports what this machine
# can actually do:
#
#   stride 0.08, lift 0.03, T 0.50 -> 0.138 m/s  tau p99 2.94 (= STALL)  qd p99 4.98
#   stride 0.06, lift 0.03, T 0.50 -> 0.114 m/s  tau p99 2.94 (= STALL)  qd p99 4.69
#   stride 0.06, lift 0.02, T 0.70 -> 0.086 m/s  tau p99 2.01            qd p99 2.84
#   stride 0.04, lift 0.02, T 0.70 -> 0.048 m/s  tau p99 1.89            qd p99 2.54
#
# So the reachable band is roughly 0.05-0.14 m/s and the top of it is against the
# torque limit. An earlier version of this file asked for up to 0.25 m/s, which
# is simply not a thing this robot can do: for most of the command distribution
# no action earned meaningful tracking reward, and standing still was the safe
# answer. 45 rpm servos on a 0.19 m leg are slow; the command range has to say so.
# 0.12-0.14 m/s IS reachable, but only by pinning the servos against their stall
# torque (see the table above). The band that is reachable *inside the servo
# envelope* is about 0.05-0.09 m/s, and that is what the robot should be asked
# for -- commanding a speed it can only hit by saturating is how you get a
# policy that works in sim and cooks the gearboxes on hardware.
# $ARDUINO_QUAD_CMD_VX_MAX overrides the forward ceiling. Note the RL policies
# reach 0.09 m/s using only ~53 % of the servo's no-load speed, i.e. with a lot
# of headroom left, so the ceiling is worth sweeping rather than assuming.
_CMD_VX_MAX = float(os.environ.get("ARDUINO_QUAD_CMD_VX_MAX", "0.09"))
# Backward was capped at 45 % of the forward ceiling because a first pass only
# needed to walk forward. That cap is a trained-range boundary, not a soft
# preference: commanding -0.15 when the policy has only ever seen -0.0675 is
# extrapolation, and on hardware it looks exactly like "reverse is broken".
_CMD_VX_BACK_FRAC = float(os.environ.get("ARDUINO_QUAD_CMD_VX_BACK_FRAC", "0.45"))
CMD_LIN_X = (-_CMD_VX_BACK_FRAC * _CMD_VX_MAX, _CMD_VX_MAX)
# Turn command ceiling. +-0.4 was inherited without ever being checked against
# what the machine can do, and it was the binding constraint: commanded past it,
# the same policies reach 0.56-0.66 rad/s with the servos at ~60 % of no-load.
# The forward ceiling had exactly this bug (0.148 -> 0.455 m/s once raised).
# Turn authority is what has to beat the robot's yaw drift on hardware, so the
# ceiling is a knob: $ARDUINO_QUAD_CMD_WZ_MAX.
_CMD_WZ_MAX = float(os.environ.get("ARDUINO_QUAD_CMD_WZ_MAX", "0.4"))
CMD_ANG_Z = (-_CMD_WZ_MAX, _CMD_WZ_MAX)

# Per-joint posture spread for the `pose` reward. Knees carry the crouch, so
# they get the loosest band.
_STD_STANDING = {r"(FL|FR|RL|RR)_hip_joint": 0.05, r"(FL|FR|RL|RR)_knee_joint": 0.08}
# How far a joint may wander from the home pose before `pose` starts charging
# for it. Together with that reward's weight this is the main brake on how much
# the legs actually move: tight bands plus a strong weight buy a tidy posture
# and a shuffling gait. $ARDUINO_QUAD_POSE_STD scales both bands.
_POSE_STD = float(os.environ.get("ARDUINO_QUAD_POSE_STD", "1.0"))
_STD_WALKING = {r"(FL|FR|RL|RR)_hip_joint": 0.25 * _POSE_STD,
                r"(FL|FR|RL|RR)_knee_joint": 0.40 * _POSE_STD}
_STD_RUNNING = _STD_WALKING


def arduino_quad_flat_env_cfg(play: bool = False,
                              full_collision: bool = False) -> ManagerBasedRlEnvCfg:
  """Flat-ground velocity tracking, plain recipe. Smoke/bring-up task."""
  cfg = make_velocity_env_cfg()

  cfg.scene.entities = {
    "robot": get_arduino_quad_robot_cfg(full_collision=full_collision)}

  # --- ground: flat by default, no height scan ----------------------------
  # $ARDUINO_QUAD_TERRAIN=rough swaps in uneven ground SCALED TO THIS ROBOT.
  # The upstream ROUGH_TERRAINS_CFG has 0-10 cm stairs and 20 cm waves, which
  # is a kerb to a machine that stands 11 cm tall -- using it unchanged would
  # measure "can it climb a wall", not "can it walk on a floor that is not
  # flat". Roughness here is 5-20 mm, about a fifth of the stance height, which
  # is what a carpet, a doorway strip or a warped floor actually looks like.
  assert cfg.scene.terrain is not None
  if os.environ.get("ARDUINO_QUAD_TERRAIN", "plane") == "rough":
    import mjlab.terrains as _tg
    from mjlab.terrains.terrain_generator import TerrainGeneratorCfg as _TGCfg
    _amp = float(os.environ.get("ARDUINO_QUAD_TERRAIN_AMP", "1.0"))
    cfg.scene.terrain.terrain_type = "generator"
    cfg.scene.terrain.terrain_generator = _TGCfg(
      size=(8.0, 8.0), border_width=20.0, num_rows=6, num_cols=6,
      # noise_step must stay above the heightfield's vertical resolution: it is
      # divided by vertical_scale and truncated to an integer, so anything
      # finer quantises to a step of 0 and np.arange divides by zero.
      sub_terrains={
        "flat": _tg.BoxFlatTerrainCfg(proportion=0.3),
        "rough": _tg.HfRandomUniformTerrainCfg(
          proportion=0.4, noise_range=(0.005 * _amp, 0.020 * _amp),
          noise_step=max(0.005 * _amp, 0.005), border_width=0.25),
        "wave": _tg.HfWaveTerrainCfg(
          proportion=0.3, amplitude_range=(0.005 * _amp, 0.025 * _amp),
          num_waves=4, border_width=0.25),
      },
      add_lights=True,
    )
  else:
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

  # --- contact sensing ----------------------------------------------------
  # Only the four toe spheres collide (robot_cfg.FEET_ONLY_COLLISION), so there
  # is no "illegal contact" geom left to watch: falling over is caught by
  # orientation and base height instead.
  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(mode="geom", pattern=TOE_GEOMS, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  cfg.scene.sensors = (cfg.scene.sensors or ()) + (feet_ground_cfg,)

  # --- observations: NO IMU on this robot ---------------------------------
  # These two terms are the IMU. Removing them from the actor is the whole
  # reason this config exists; the critic dict is a separate dict that still
  # holds them (plus base_lin_vel), which is exactly the privileged-critic setup.
  # $ARDUINO_QUAD_IMU selects how much of an IMU the actor is allowed to see.
  # The two terms are not equally easy to get on hardware, so they are separate:
  #   none  (default) -- as built today, no IMU at all
  #   gyro            -- base_ang_vel only. A raw 3-axis rate gyro, no fusion,
  #                      no attitude estimate. This is the piece that closes the
  #                      loop on the yaw command, which is currently open-loop.
  #   full            -- also projected_gravity, which needs an attitude filter
  #                      (accel+gyro) whose conventions have to match the sim.
  _IMU = os.environ.get("ARDUINO_QUAD_IMU", "none")
  assert _IMU in ("none", "gyro", "full"), _IMU
  if _IMU == "none":
    del cfg.observations["actor"].terms["base_ang_vel"]
  if _IMU in ("none", "gyro"):
    del cfg.observations["actor"].terms["projected_gravity"]
  # A short history lets the policy infer contact and body motion from the
  # joint traces alone -- with no IMU it has nothing else to work with.
  cfg.observations["actor"].history_length = 3

  # Sensor noise rescaled to this robot. Upstream's +-1.5 rad/s joint-velocity
  # noise is 7 % of a Go2 joint's range but 32 % of a 4.717 rad/s servo's.
  cfg.observations["actor"].terms["joint_vel"].noise.n_min = -0.3
  cfg.observations["actor"].terms["joint_vel"].noise.n_max = 0.3
  # STS3215: 4096 counts / 360 deg = 0.0015 rad, plus perhaps 1 deg of backlash.
  cfg.observations["actor"].terms["joint_pos"].noise.n_min = -0.02
  cfg.observations["actor"].terms["joint_pos"].noise.n_max = 0.02

  cfg.observations["actor"].terms["phase"].params["period"] = GAIT_PERIOD
  cfg.observations["critic"].terms["phase"].params["period"] = GAIT_PERIOD
  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"].site_names = TOE_SITES

  # --- actions ------------------------------------------------------------
  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = ARDUINO_QUAD_ACTION_SCALE

  # --- commands: no lateral DoF on this machine ---------------------------
  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.ranges.lin_vel_x = CMD_LIN_X
  twist_cmd.ranges.lin_vel_y = (0.0, 0.0)
  twist_cmd.ranges.ang_vel_z = CMD_ANG_Z
  twist_cmd.ranges.heading = None
  twist_cmd.heading_command = False   # yaw rate is commanded directly
  twist_cmd.rel_standing_envs = 0.1
  twist_cmd.viz.z_offset = 0.2

  cfg.curriculum["command_vel"].params["velocity_stages"] = [
    {"step": 0, "lin_vel_x": (-0.02, 0.05),
     "lin_vel_y": (0.0, 0.0), "ang_vel_z": (-0.2, 0.2)},
    {"step": 600 * 24, "lin_vel_x": CMD_LIN_X,
     "lin_vel_y": (0.0, 0.0), "ang_vel_z": CMD_ANG_Z},
  ]

  cfg.viewer.body_name = BASE_BODY
  cfg.viewer.distance = 0.9
  cfg.viewer.elevation = -12.0

  # --- events: sim2real domain randomization ------------------------------
  cfg.events["foot_friction"].params["asset_cfg"].geom_names = TOE_GEOMS
  # 0.4 = bare printed/aluminium bracket end on a hard floor,
  # 1.1 = a rubber tip. A policy that needs the sticky end will not survive.
  # $ARDUINO_QUAD_FRICTION_LO / _HI. Pinning both to one value at play time is
  # how "will it slip in place" gets answered with a number instead of a guess.
  cfg.events["foot_friction"].params["ranges"] = (
    float(os.environ.get("ARDUINO_QUAD_FRICTION_LO", "0.4")),
    float(os.environ.get("ARDUINO_QUAD_FRICTION_HI", "1.1")))
  cfg.events["base_com"].params["asset_cfg"].body_names = (BASE_BODY,)
  # A battery is the last thing to go on, and where it goes moves the centre of
  # mass more than its weight alone suggests. Both are knobs so the tolerance
  # can be measured instead of assumed.
  _COM = float(os.environ.get("ARDUINO_QUAD_COM_SCALE", "1.0"))
  cfg.events["base_com"].params["ranges"] = {
    0: (-0.02 * _COM, 0.02 * _COM), 1: (-0.01 * _COM, 0.01 * _COM),
    2: (-0.01 * _COM, 0.02 * _COM)}
  # Servo zero-offset calibration error. mjlab models this properly rather than
  # as observation noise: the bias is added to the observed joint angle AND
  # subtracted from the actuator target, which is exactly what a wrong `zero`
  # in JOINTS[] does -- the robot sits somewhere other than where the policy
  # thinks it does, while the numbers stay self-consistent.
  #
  # +-1.7 deg was the original figure and it is not a real number: a servo zeroed
  # by hand, through a horn with 24 teeth (15 deg per tooth), lands several
  # degrees out per joint. $ARDUINO_QUAD_ENCODER_BIAS sets the half-range.
  _ENC_BIAS = float(os.environ.get("ARDUINO_QUAD_ENCODER_BIAS", "0.03"))
  cfg.events["encoder_bias"].params["bias_range"] = (-_ENC_BIAS, _ENC_BIAS)
  # Smaller pushes than the upstream humanoid/Go2 numbers: this robot weighs 1 kg.
  cfg.events["push_robot"].params["velocity_range"] = {
    "x": (-0.25, 0.25), "y": (-0.25, 0.25), "z": (-0.1, 0.1),
    "roll": (-0.4, 0.4), "pitch": (-0.4, 0.4), "yaw": (-0.5, 0.5),
  }
  # The servo's internal position loop: gains unknown, so scale them wide.
  cfg.events["servo_gains"] = EventTermCfg(
    mode="startup", func=dr.pd_gains,
    params={"asset_cfg": SceneEntityCfg("robot"),
            # $ARDUINO_QUAD_KP_LO/HI, $ARDUINO_QUAD_KD_LO/HI. Pinning these
            # OUTSIDE the trained band at play time is how model mismatch gets
            # measured: a sim-to-sim push test keeps the dynamics the policy
            # trained on, so feedback has nothing to correct and every policy
            # scores the same. What sim2real actually asks is what happens when
            # the servo is not the servo in the MJCF.
            "kp_range": (float(os.environ.get("ARDUINO_QUAD_KP_LO", "0.6")),
                         float(os.environ.get("ARDUINO_QUAD_KP_HI", "1.6"))),
            "kd_range": (float(os.environ.get("ARDUINO_QUAD_KD_LO", "0.4")),
                         float(os.environ.get("ARDUINO_QUAD_KD_HI", "2.0"))),
            "operation": "scale"},
  )
  # Torque falls with voltage/temperature; never let it be MORE than the spec.
  cfg.events["servo_effort"] = EventTermCfg(
    mode="startup", func=dr.effort_limits,
    params={"asset_cfg": SceneEntityCfg("robot"),
            "effort_limit_range": (0.7, 1.0), "operation": "scale"},
  )
  # Reflected rotor inertia -- an estimate from a guessed rotor, so +-2x.
  cfg.events["servo_armature"] = EventTermCfg(
    mode="startup", func=dr.joint_armature,
    params={"asset_cfg": SceneEntityCfg("robot"),
            "ranges": (0.5, 2.0), "operation": "scale"},
  )
  # Back-EMF damping is a datasheet ratio, so a tighter band than the estimates.
  cfg.events["servo_damping"] = EventTermCfg(
    mode="startup", func=dr.joint_damping,
    params={"asset_cfg": SceneEntityCfg("robot"),
            "ranges": (0.7, 1.4), "operation": "scale"},
  )
  # Gearbox stiction: absolute, because the nominal 0.01 Nm is a pure guess.
  # This is also the only stand-in for gear backlash. True free play needs a
  # second degree of freedom per joint, which MuJoCo will not give for free;
  # Coulomb friction reproduces the part that matters to a position loop -- a
  # band around the target where the joint does not move -- but NOT the part
  # where the load swings freely through the play. $ARDUINO_QUAD_STICTION sets
  # the upper end.
  _STICTION = float(os.environ.get("ARDUINO_QUAD_STICTION", "0.04"))
  cfg.events["servo_friction"] = EventTermCfg(
    mode="startup", func=dr.joint_friction,
    params={"asset_cfg": SceneEntityCfg("robot"),
            "ranges": (0.0, _STICTION), "operation": "abs"},
  )
  # The CAD export has no battery, no Arduino, no wiring. The robot itself is
  # 1.04 kg, so 250 g is a quarter of it again. $ARDUINO_QUAD_PAYLOAD_MAX.
  _PAYLOAD = float(os.environ.get("ARDUINO_QUAD_PAYLOAD_MAX", "0.25"))
  cfg.events["payload"] = EventTermCfg(
    mode="startup", func=dr.body_mass,
    params={"asset_cfg": SceneEntityCfg("robot", body_names=(BASE_BODY,)),
            "ranges": (0.0, _PAYLOAD), "operation": "add"},
  )

  # --- rewards ------------------------------------------------------------
  # The tracking std has to scale with the command range, not be picked by feel.
  # Upstream uses std = 0.5 against a 2.0 m/s command ceiling, i.e. std is a
  # quarter of the largest command. A first attempt here used 0.12 against a
  # 0.25 m/s ceiling (ratio 0.48) and that alone stopped the robot walking:
  # exp(-0.05^2/0.12^2) = 0.84, so standing perfectly still collected ~75 % of
  # the tracking reward at the curriculum's small commands, while the servo-
  # protection penalties made actually moving cost more than that. Same ratio as
  # upstream, applied to this robot's ceilings:
  # Weights: translating has to be worth more than looking busy. Measured at
  # iter 1200 of an earlier run, the income was foot_gait 1.63 > pose 0.87 >
  # track_ang 0.63 > track_lin 0.59, and the policy had duly converged to
  # trembling in place -- joint amplitude +-2 deg, 5 mm of swing, 0.00 m/s
  # forward -- because a foot that flickers on and off the ground at the right
  # moments satisfies the gait clock without going anywhere. Tracking is now the
  # dominant term by a clear margin.
  # Instantaneous tracking is kept at a low weight -- it is the term that gives a
  # gradient toward "move the right way at all" -- while the filtered term below
  # is what actually pays, because it cannot be collected by rocking in place.
  cfg.rewards["track_linear_velocity"].weight = 0.5
  cfg.rewards["track_linear_velocity"].params["std"] = 0.25 * CMD_LIN_X[1]   # 0.062
  # 2.0, not 3.0. Yaw is commanded around zero, so a robot that simply does not
  # turn collects this term in full -- raising it raises the income of standing
  # still, which is the failure mode this whole task keeps falling into.
  cfg.rewards["track_angular_velocity"].weight = 2.0
  # std as a fraction of the command ceiling. 0.7 is loose: at a 0.4 ceiling it
  # pays ~60 % of the reward for only reaching 0.2. The linear-velocity term had
  # the same problem and was tightened to 0.25 of its ceiling, which is what
  # stopped the policy from standing still and collecting. This one was left at
  # 0.7. $ARDUINO_QUAD_ANG_STD_FRAC.
  cfg.rewards["track_angular_velocity"].params["std"] = (
    float(os.environ.get("ARDUINO_QUAD_ANG_STD_FRAC", "0.7")) * CMD_ANG_Z[1])
  cfg.rewards["body_orientation_l2"].params["asset_cfg"].body_names = (BASE_BODY,)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = (BASE_BODY,)
  cfg.rewards["angular_momentum"].weight = -0.005   # 1 kg robot: tiny numbers
  cfg.rewards["is_terminated"].weight = -50.0
  cfg.rewards["action_rate_l2"].weight = -0.1       # servo-friendly smoothness
  # Posture is a regularizer, not an objective: holding the home pose exactly is
  # what NOT walking looks like, so it must not compete with tracking.
  cfg.rewards["pose"].weight = 0.5
  cfg.rewards["pose"].params["std_standing"] = _STD_STANDING
  cfg.rewards["pose"].params["std_walking"] = _STD_WALKING
  cfg.rewards["pose"].params["std_running"] = _STD_RUNNING
  cfg.rewards["foot_gait"].params["period"] = GAIT_PERIOD
  cfg.rewards["foot_gait"].params["offset"] = GAIT_OFFSET
  cfg.rewards["foot_clearance"].params["target_height"] = SWING_HEIGHT
  cfg.rewards["foot_clearance"].params["asset_cfg"].site_names = TOE_SITES
  cfg.rewards["foot_slip"].params["asset_cfg"].site_names = TOE_SITES

  # --- terminations -------------------------------------------------------
  # 35 deg, not the 60 the upstream recipe uses. Six of eight training seeds in
  # the yaw-flipped frame converged to sitting back on the rear legs with both
  # front feet held ~0.1 m in the air at 22 deg of pitch -- a posture a 1 kg
  # quadruped on flat ground has no business reaching (the seeds that walked sit
  # at 4-8 deg). Terminating there kills the rearing basin early instead of
  # letting a policy spend a whole episode in it.
  cfg.terminations["fell_over"].params["limit_angle"] = math.radians(35.0)
  cfg.terminations["low_base"] = TerminationTermCfg(
    func=envs_mdp.root_height_below_minimum,
    params={"minimum_height": 0.06},
  )

  if play:
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.curriculum = {}
    twist_cmd.ranges.lin_vel_x = CMD_LIN_X
    twist_cmd.ranges.ang_vel_z = CMD_ANG_Z

  return cfg


def arduino_quad_walk_env_cfg(play: bool = False,
                              full_collision: bool = False) -> ManagerBasedRlEnvCfg:
  """Flat recipe + the gait shaping that turns a shuffle into a trot.

  The plain velocity recipe has a well-known local optimum: keep all four feet
  planted and slide to track the command (feet_air_time ~ 0). Making sliding
  expensive and lifting cheap is what breaks it -- same lesson as
  a 5 DoF leg on a much larger machine, retuned for a 1 kg one.
  """
  cfg = arduino_quad_flat_env_cfg(play=play, full_collision=full_collision)

  cfg.rewards["alive"] = RewardTermCfg(func=envs_mdp.is_alive, weight=0.5)
  cfg.rewards["foot_slip"].weight = -1.0        # was -0.25: point feet slide easily
  # 0.75, not the 3.0 a previous attempt used. foot_gait scores "is this foot in
  # contact when the clock says it should be", which a robot standing still and
  # shivering can satisfy; at 3.0 it out-earned velocity tracking and that is
  # exactly what the policy went and did. Upstream's default is 0.5.
  cfg.rewards["foot_gait"].weight = 0.75
  # $ARDUINO_QUAD_CLEARANCE_W. This was raised from the upstream -1.0 to -6.0 to
  # "pick the feet up", which is backwards: feet_clearance costs
  # |height - target| * horizontal foot speed at every step, and a swinging foot
  # has to pass through low heights at take-off and touch-down. The cheapest way
  # to pay less is to move the foot slowly -- i.e. take shorter steps. Raising
  # the target makes it worse still, which is why lifting the target from 22 mm
  # to 55 mm bought almost nothing. feet_swing_height (scored on peak height at
  # landing) is the term that actually asks for lift; this one should stay small.
  cfg.rewards["foot_clearance"].weight = float(
    os.environ.get("ARDUINO_QUAD_CLEARANCE_W", "-6.0"))
  cfg.rewards["feet_air_time"] = RewardTermCfg(
    func=vmdp.feet_air_time,
    # threshold is both the target stance/swing duration AND the reward ceiling:
    # a GAIT_PERIOD trot with duty 0.5 spends GAIT_PERIOD/2 in each mode.
    weight=2.5,
    params={"sensor_name": "feet_ground_contact", "threshold": GAIT_PERIOD / 2.0,
            "command_name": "twist", "command_threshold": 0.05},
  )
  # Peak swing height, scored when the foot lands. foot_clearance alone bought a
  # 2 mm scuffing trot: it costs |z - target| * foot_speed, which a foot that
  # skims the floor slowly pays almost nothing for. This one is measured per
  # step and does not care how fast the foot got there.
  cfg.rewards["feet_swing_height"] = RewardTermCfg(
    func=vmdp.feet_swing_height,
    weight=float(os.environ.get("ARDUINO_QUAD_SWING_W", "-3.0")),
    params={"sensor_name": "feet_ground_contact", "target_height": SWING_HEIGHT,
            "command_name": "twist", "command_threshold": 0.05,
            "asset_cfg": SceneEntityCfg("robot", site_names=TOE_SITES)},
  )

  # --- keep the gait inside what an STS3215 can actually do ----------------
  # Measured on the first run of this recipe: the policy converged to a 4.4
  # step/s flutter that ran the joints at 99 % of the servo's no-load speed and
  # 96 % of its stall torque. Nothing in the reward made that expensive, and a
  # gait that only exists because the sim let the motors run flat out is exactly
  # what does not survive contact with hardware. These three terms price it in.
  cfg.rewards["action_rate_l2"].weight = -0.3   # was -0.1: 50 Hz chatter
  cfg.rewards["joint_vel_l2"] = RewardTermCfg(
    func=envs_mdp.joint_vel_l2, weight=-0.005)
  cfg.rewards["joint_torques_l2"] = RewardTermCfg(
    func=envs_mdp.joint_torques_l2, weight=-0.002)
  # The two that hold the line, on the RIGHT quantities.
  #
  # Torque: compared against the CONTINUOUS rating, but on the NET torque
  # (actuator output minus the back-EMF damper), because that is what tracks
  # motor current. Measured on the scripted trot, net rms is 0.24-0.38 Nm
  # against a 0.98 Nm rating -- this robot is nowhere near overloaded, and an
  # earlier version that thresholded raw actuator_force at 0.98 was really
  # penalising joint speed and stopped the robot walking at all.
  #
  # Speed is the actual limiter here: 45 rpm servos mean the fastest gait
  # measured (0.138 m/s) runs the joints at 106 % of no-load, where there is no
  # torque left to reject a disturbance. 70 % of no-load keeps a margin.
  cfg.rewards["torque_excess"] = RewardTermCfg(
    func=local_mdp.joint_torque_excess, weight=-2.0,
    params={"limit": RATED_TORQUE,
            "back_emf_damping": STALL_TORQUE / NO_LOAD_SPEED})
  cfg.rewards["speed_excess"] = RewardTermCfg(
    func=local_mdp.joint_speed_excess, weight=-0.3,
    params={"limit": 0.7 * NO_LOAD_SPEED})
  # Squared metres on a 0.14 m robot are small numbers, hence the large weight:
  # a 2 cm sag costs 0.02 reward, comparable to the gait terms.
  cfg.rewards["base_height"] = RewardTermCfg(
    func=local_mdp.base_height_target,
    weight=-50.0,
    params={"target_height": HOME_BASE_HEIGHT},
  )

  # Pays only for velocity that survives a 0.6 s average, i.e. for actually
  # travelling. See rewards.py for the measurement that motivated this.
  cfg.rewards["track_lin_vel_filtered"] = RewardTermCfg(
    func=local_mdp.track_linear_velocity_filtered,
    weight=4.0,
    params={"command_name": "twist", "std": 0.25 * CMD_LIN_X[1],
            "filter_time": GAIT_PERIOD},
  )
  cfg.rewards["base_vel_oscillation"] = RewardTermCfg(
    func=local_mdp.base_vel_oscillation_l2,
    weight=-2.0,
    params={"filter_time": GAIT_PERIOD},
  )
  # Guidance toward the scripted trot that is already known to walk this robot,
  # annealed to zero so it shapes the search and not the final gait.
  #
  # OFF by default. Measured: with guidance the policy reached the same forward
  # speed as without it, but once the weight annealed out it drifted to a MORE
  # aggressive gait -- p99 torque 2.32 vs 1.71 Nm, peak joint speed 107 % of
  # no-load vs 90 % -- and kept the same dragged rear leg. Guiding toward a
  # trajectory the policy is then free to leave does not, on this robot, leave
  # anything behind. $ARDUINO_QUAD_TROT_GUIDE=1 turns it back on.
  if os.environ.get("ARDUINO_QUAD_TROT_GUIDE", "0") == "1":
    _add_trot_guidance(cfg)

  # Make dragging a leg cost something. limit = 2 gait cycles.
  cfg.rewards["foot_air_time_excess"] = RewardTermCfg(
    func=local_mdp.foot_air_time_excess,
    weight=-1.0,
    params={"sensor_name": "feet_ground_contact", "limit": 2.0 * GAIT_PERIOD},
  )

  # Standing still must not be comfortable: with the rocking exploit closed,
  # the fallback failure mode is to freeze and collect posture/alive income.
  cfg.rewards["alive"].weight = 0.1
  # $ARDUINO_QUAD_POSE_W: lowering this lets the legs swing further from home.
  cfg.rewards["pose"].weight = float(os.environ.get("ARDUINO_QUAD_POSE_W", "0.3"))

  _apply_trial_overrides(cfg)
  return cfg




def _add_trot_guidance(cfg) -> None:
  """Reference-gait guidance, annealed to zero. See the call site for why it is
  off by default."""
  cfg.rewards["trot_reference"] = RewardTermCfg(
    func=local_mdp.reference_trot,
    weight=2.0,
    # std is against the SUM of squared joint errors over all 8 joints, so the
    # effective per-joint tolerance is std/sqrt(8) = 0.21 rad (12 deg). Tighter
    # than that and an untrained policy earns ~0 and gets no gradient to follow.
    params={"command_name": "twist", "std": 0.6, "period": GAIT_PERIOD,
            "command_threshold": 0.02,
            "asset_cfg": SceneEntityCfg("robot")},
  )
  cfg.curriculum["trot_guidance"] = CurriculumTermCfg(
    func=vmdp.reward_weight,
    params={"reward_name": "trot_reference",
            "weight_stages": [{"step": 0, "weight": 2.0},
                              {"step": 700 * 24, "weight": 0.7},
                              {"step": 1400 * 24, "weight": 0.0}]},
  )


# Reward-weight knobs a search loop may override (see _apply_trial_overrides).
_TRIAL_WEIGHT_KNOBS = {
  "alive_weight": "alive",
  "track_lin_weight": "track_linear_velocity",
  "track_ang_weight": "track_angular_velocity",
  "action_rate_weight": "action_rate_l2",
  "is_terminated_weight": "is_terminated",
  "foot_gait_weight": "foot_gait",
  "foot_clearance_weight": "foot_clearance",
  "foot_slip_weight": "foot_slip",
  "body_orientation_weight": "body_orientation_l2",
  "pose_weight": "pose",
  "feet_air_time_weight": "feet_air_time",
  "feet_swing_height_weight": "feet_swing_height",
  "joint_vel_weight": "joint_vel_l2",
  "joint_torques_weight": "joint_torques_l2",
  "base_height_weight": "base_height",
}


def _apply_trial_overrides(cfg) -> None:
  """Optional: override reward weights from a JSON file named by
  $ARDUINO_QUAD_TRIAL_CONFIG (used by reward-search loops). Values are clamped;
  unknown keys are ignored. No file -> nothing happens."""
  import json

  path = os.environ.get("ARDUINO_QUAD_TRIAL_CONFIG")
  if not path or not os.path.exists(path):
    return
  with open(path) as f:
    ov = json.load(f)

  def _num(v, lo, hi):
    try:
      x = float(v)
    except (TypeError, ValueError):
      return None
    if x != x or x in (float("inf"), float("-inf")):
      return None
    return max(lo, min(hi, x))

  for knob, term in _TRIAL_WEIGHT_KNOBS.items():
    if knob in ov and term in cfg.rewards:
      w = _num(ov[knob], -500.0, 500.0)
      if w is not None:
        cfg.rewards[term].weight = w


def arduino_quad_robust_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Walk, but started from off-nominal states. Fine-tuning stage before hardware.

  ``ArduinoQuad-Walk`` always resets to an exactly upright, exactly-home,
  zero-velocity robot. A real one never starts there: the servos have
  calibration error, the legs are still settling from the ramp-to-home, and the
  first command usually arrives while somebody is still putting it down. A
  policy trained only from the perfect state has no data anywhere near those
  conditions.

  Resume a walking checkpoint into this task -- same observation and action
  spaces, same experiment name, so ``--agent.resume`` works across the two --
  and turn on the ERFI torque perturbation at the same time::

      ARDUINO_QUAD_ERFI=1 ARDUINO_QUAD_ERFI_OFF=0.10 ARDUINO_QUAD_ERFI_STEP=0.05 \
        ./scripts/train.sh ArduinoQuad-Robust 4500 4096 \
        --agent.resume True --agent.load-run <walk_run_dir> \
        --agent.load-checkpoint model_2999.pt
  """
  cfg = arduino_quad_walk_env_cfg(play=play)

  cfg.events["reset_base"].params["pose_range"] = {
    "x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (0.0, 0.02),
    "roll": (-0.15, 0.15), "pitch": (-0.15, 0.15), "yaw": (-3.14, 3.14),
  }
  cfg.events["reset_base"].params["velocity_range"] = {
    "x": (-0.1, 0.1), "y": (-0.1, 0.1), "z": (-0.05, 0.05),
    "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.3, 0.3),
  }
  # +-8.6 deg of joint error at reset, and legs that are already moving.
  cfg.events["reset_robot_joints"].params["position_range"] = (-0.15, 0.15)
  cfg.events["reset_robot_joints"].params["velocity_range"] = (-0.5, 0.5)
  return cfg


def arduino_quad_recovery_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Get back on your feet from a fallen pose.

  Three things have to change from the walking task:

  1. **Whole-body collision.** With the walking task's four contact spheres a
     fallen robot's body is inside the floor and there is nothing to push
     against, so this task uses ``FULL_COLLISION``.
  2. **No fall terminations.** ``fell_over`` and ``low_base`` exist to stop the
     walking policy wasting rollout on a lost cause; here the fallen state *is*
     the task, so both are removed and the episode is short instead.
  3. **No gait rewards.** Air time, swing height, cadence and slip are all
     meaningless when the robot is on its side. What is left is: get upright,
     get the body up to standing height, get the joints back to the home pose,
     and do not cook the servos doing it.

  The observation and action layout is deliberately IDENTICAL to the walking
  task (command pinned to zero, so the gait-phase clock reads zero as well), so
  the same exported header and the same deploy C code run this policy too.

  **Honest limitation:** with no IMU the robot cannot tell that it has fallen,
  and the policy cannot see its own orientation -- only joint angles, joint
  speeds and their history, from which the contact configuration is at best
  weakly observable. So this is a triggered behaviour ("send 'u' over serial and
  it tries to stand up"), not an automatic reflex, and it is much more likely to
  work from a belly-down pose than from an upside-down one -- with eight
  sagittal joints and no abduction there is very little roll authority to right
  itself with. scripts/quad_recovery_report.sh measures the success rate per
  starting attitude rather than assuming.
  """
  cfg = arduino_quad_walk_env_cfg(play=play, full_collision=True)

  # A fallen robot generates a lot more contacts than four toes do.
  cfg.sim.njmax = 1000
  cfg.sim.contact_sensor_maxmatch = 256

  # Drop it in from an arbitrary attitude, close to the floor, joints anywhere.
  cfg.events["reset_base"].params["pose_range"] = {
    "x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (-0.08, 0.0),
    "roll": (-3.14, 3.14), "pitch": (-1.2, 1.2), "yaw": (-3.14, 3.14),
  }
  cfg.events["reset_base"].params["velocity_range"] = {}
  cfg.events["reset_robot_joints"].params["position_range"] = (-1.0, 1.0)
  cfg.events["reset_robot_joints"].params["velocity_range"] = (-0.5, 0.5)
  cfg.events.pop("push_robot", None)

  # Stand still: zero command, so `phase` is zero too and the observation
  # vector keeps exactly the walking layout.
  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.ranges.lin_vel_x = (0.0, 0.0)
  twist_cmd.ranges.lin_vel_y = (0.0, 0.0)
  twist_cmd.ranges.ang_vel_z = (0.0, 0.0)
  twist_cmd.ranges.heading = None
  twist_cmd.heading_command = False
  twist_cmd.rel_standing_envs = 1.0
  cfg.curriculum.pop("command_vel", None)

  # Being on the floor is the starting condition, not a failure.
  cfg.terminations.pop("fell_over", None)
  cfg.terminations.pop("low_base", None)
  cfg.episode_length_s = 6.0

  for term in ("foot_gait", "foot_clearance", "foot_slip", "feet_air_time",
               "feet_swing_height", "soft_landing", "stand_still",
               "track_linear_velocity", "track_angular_velocity",
               "angular_momentum", "body_ang_vel", "is_terminated"):
    cfg.rewards.pop(term, None)

  # Getting upright IS the reward. body_orientation_l2 is |projected gravity xy|^2,
  # so it goes from ~1.0 lying on a side to ~0 standing.
  #
  # The weights below are the second attempt. The first one used pose=2.0 and
  # orientation=-8, and the policy found the obvious hack: servo the joints to
  # the home angles *while still lying on the floor*. Measured at iter 255 the
  # pose reward was 1.69 out of 2.0 while the orientation term had got WORSE
  # (0.02 -> 0.16) and the base was parked at 0.061 m. Matching home joint
  # angles is easy and getting up is hard, so any meaningful weight on posture
  # buys lying down. Posture is now only a tie-breaker for the final stance.
  cfg.rewards["body_orientation_l2"].weight = -20.0
  cfg.rewards["base_height"].weight = -150.0
  cfg.rewards["base_height"].params["target_height"] = HOME_BASE_HEIGHT
  cfg.rewards["pose"].weight = 0.3
  cfg.rewards["pose"].params["std_standing"] = _STD_STANDING
  # No `alive` term: nothing terminates in this task, so it would be a constant
  # offset with no gradient -- it only makes the reward curve harder to read.
  cfg.rewards.pop("alive", None)
  # Servo protection stays: thrashing on the floor is exactly how you strip a
  # plastic gear train.
  cfg.rewards["action_rate_l2"].weight = -0.2
  cfg.rewards["joint_vel_l2"].weight = -0.01
  cfg.rewards["joint_torques_l2"].weight = -0.01

  return cfg

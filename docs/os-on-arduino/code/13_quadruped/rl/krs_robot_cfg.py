"""krs_quad_robot constants for mjlab (the KRS version of the quadruped).

Same leg layout as the Feetech robot -- two sagittal joints per leg, hip pitch
+ knee pitch, no abduction -- but a different machine in every number that
matters, so it has its own model, its own tasks (``KrsQuad-*``) and its own
policy. Nothing here is shared with robot_cfg.py except the joint naming.

  mass 0.611 kg (measured parts; krs_quad_robot/README.md)
  hip spacing 0.132 m front-rear, 0.061 m left-right; feet 0.19 m apart
  thigh 0.088 m, shank 0.120 m (hip axis -> knee axis -> toe), from the URDF
  q = 0: legs straight down, servo centre (pulse 7500)

Actuator: Kondo KRS-3302 ICS (https://kondo-robot.com/product/03146)

  stall torque  6.7 kgf.cm = 0.657 Nm at 7.4 V, 5.5 kgf.cm = 0.539 Nm at 6.0 V
  speed         0.16 s/60 deg = 6.545 rad/s at 7.4 V (0.20 s/60 deg at 6.0 V)
  range         270 deg (+-135 deg from centre)
  gear ratio    262.92:1

That is under a quarter of the STS3215's 2.94 Nm, on a robot 0.6 times the
mass: the KRS robot has about 40 % of the Feetech robot's torque per kilogram.
The robot is powered from a 7.2 V Ni-MH pack or a 5.9 V AC adapter, so the
torque on the floor is anywhere in 0.54-0.66 Nm; env_cfgs randomizes over that.

As on the Feetech robot, the DC motor's torque-speed slope is modelled as
viscous damping, b = 0.657 / 6.545 = 0.100 Nm.s/rad (datasheet ratio; the 6 V
pair gives 0.103). The servo's internal position loop (ICS "stretch") and the
reflected rotor inertia are ESTIMATES and are randomized wide.

The MJCF is built by krs_quad_robot/scripts/build_mjcf.py; this file finds it
through $KRS_QUAD_XML (rl/scripts/env.sh exports it) because rl/ is placed
inside an unitree_rl_mjlab checkout by a symlink and cannot use relative paths.
The home stance is read from the MJCF's ``home`` keyframe, where build_mjcf.py
wrote it after solving the leg IK -- it is not restated here.
"""

import os
from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

LEGS = ("FL", "FR", "RL", "RR")
BASE_BODY = "base_link"
JOINT_EXPR = r"(FL|FR|RL|RR)_(hip|knee)_joint"

# Datasheet, KRS-3302 ICS at 7.4 V (module docstring).
STALL_TORQUE = 0.657      # Nm (6.7 kgf.cm)
STALL_TORQUE_6V = 0.539   # Nm (5.5 kgf.cm), the low end of what the robot sees
NO_LOAD_SPEED = 6.545     # rad/s (0.16 s / 60 deg)
JOINT_LIMIT = 2.356194    # rad, +-135 deg
# Kondo gives no continuous rating. Half of stall is used as the RMS budget
# the gait should stay under, the same 1/3-1/2 ratio hobby servos are usually
# rated at (the STS3215 is 0.98 / 2.94). This is a design target, not a spec.
RATED_TORQUE = 0.5 * STALL_TORQUE


def _resolve_xml() -> Path:
  xml = os.environ.get("KRS_QUAD_XML")
  if not xml:
    root = os.environ.get("ARDUINO_QUAD_ROOT")
    if not root:
      raise RuntimeError(
        "KRS_QUAD_XML / ARDUINO_QUAD_ROOT is not set: cannot find the "
        "krs_quad_robot MJCF. Run through rl/scripts/train.sh or "
        "`source rl/scripts/env.sh` first.")
    xml = os.path.join(root, "krs_quad_robot", "mjcf", "krs_quad_robot.xml")
  path = Path(xml)
  if not path.exists():
    raise FileNotFoundError(
      f"krs_quad_robot MJCF not found: {path} "
      "(build it with krs_quad_robot/scripts/build_mjcf.py)")
  return path


KRS_QUAD_XML: Path = _resolve_xml()


def _read_home() -> tuple[float, dict[str, float]]:
  """Base height and joint angles of the MJCF's ``home`` keyframe."""
  model = mujoco.MjModel.from_xml_path(str(KRS_QUAD_XML))
  key = model.key("home")
  joints = {}
  for j in range(model.njnt):
    if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
      joints[model.joint(j).name] = float(key.qpos[model.jnt_qposadr[j]])
  return float(key.qpos[2]), joints


HOME_BASE_HEIGHT, HOME_JOINT_POS = _read_home()


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(KRS_QUAD_XML))
  for act in list(spec.actuators):   # mjlab adds actuators from the cfg below
    spec.delete(act)
  for key in list(spec.keys):        # mjlab resets from init_state
    spec.delete(key)
  try:
    spec.delete(spec.geom("ground"))  # mjlab adds terrain
  except Exception:  # noqa: BLE001 - absent if rebuilt without a ground
    pass
  return spec


##
# Actuator. All eight joints are the same servo.
##
# ESTIMATE: the ICS position loop. Sized like the measured STS3215 loop, which
# reaches stall torque at 0.12 rad of error (25 Nm/rad x 0.118 = 2.94 Nm):
# 0.657 Nm / 0.12 rad = 5.5 Nm/rad. Randomized x0.5..x2 in env_cfgs.
KRS_KP = 5.5
# ESTIMATE: same damping ratio to kp as the measured STS3215 (0.5 / 25).
KRS_KD = 0.11
# ESTIMATE: reflected rotor inertia. A ~1e-8 kg.m^2 rotor through 262.92:1.
KRS_ARMATURE = 7e-4
# ESTIMATE: command dead time, in 5 ms physics steps. The Feetech robot measured
# 27 ms end to end (bus + inference + the servo's own processing). ICS goes
# through the RCB-4mini, one more hop, so 20-40 ms is assumed.
KRS_DELAY_MIN_LAG = 4
KRS_DELAY_MAX_LAG = 8

KRS3302_ACTUATOR = BuiltinPositionActuatorCfg(
  target_names_expr=(JOINT_EXPR,),
  stiffness=KRS_KP,
  damping=KRS_KD,
  effort_limit=STALL_TORQUE,
  armature=KRS_ARMATURE,
  frictionloss=0.005,   # ESTIMATE: gearbox dry friction
  viscous_damping=STALL_TORQUE / NO_LOAD_SPEED,  # back-EMF, datasheet ratio
  delay_min_lag=KRS_DELAY_MIN_LAG,
  delay_max_lag=KRS_DELAY_MAX_LAG,
)

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, HOME_BASE_HEIGHT),
  joint_pos=dict(HOME_JOINT_POS),
  joint_vel={".*": 0.0},
)

##
# Collision. The toes are the only geoms meant to touch the floor. The body and
# the knees also collide with the floor (never with each other), so that
# sitting on the belly or kneeling is a real contact the task can see and punish
# instead of the body sinking through the ground unnoticed.
##
#
# The legs also collide with each other, front against rear: the hips are only
# 132 mm apart and a front knee trails 71 mm behind its hip, so a rear leg
# swinging forward can strike it. Without these capsules the simulation lets
# the legs pass through each other, and a policy can learn moves the real robot
# cannot make (seen on hardware: rear feet hitting the front legs when the body
# was tilted). Contact bits: floor geoms 1/0 (they hit the terrain, 1/1, and
# nothing else); front capsules 4/0 and rear capsules 0/4, so a front capsule
# touches only a rear one -- (4 & 4) -- and never the floor or its own pair.
##
TOE_GEOMS = tuple(f"{leg}_toe" for leg in LEGS)
_TOE_REGEX = r"^(FL|FR|RL|RR)_toe$"
_BODY_REGEX = r"^(base_box|(FL|FR|RL|RR)_knee_ball)$"
_FRONT_CAP_REGEX = r"^(FL|FR)_(thigh|shank)_cap$"
_REAR_CAP_REGEX = r"^(RL|RR)_(thigh|shank)_cap$"
FRONT_LEG_CAPS = tuple(f"{leg}_{part}_cap" for leg in ("FL", "FR")
                       for part in ("thigh", "shank"))

KRS_COLLISION = CollisionCfg(
  geom_names_expr=(_TOE_REGEX, _BODY_REGEX, _FRONT_CAP_REGEX, _REAR_CAP_REGEX),
  contype={_TOE_REGEX: 1, _BODY_REGEX: 1, _FRONT_CAP_REGEX: 4, _REAR_CAP_REGEX: 0},
  conaffinity={_TOE_REGEX: 0, _BODY_REGEX: 0, _FRONT_CAP_REGEX: 0,
               _REAR_CAP_REGEX: 4},
  condim={_TOE_REGEX: 3, _BODY_REGEX: 3, _FRONT_CAP_REGEX: 3, _REAR_CAP_REGEX: 3},
  priority={_TOE_REGEX: 1},
  # A bare printed PLA bracket on a hard floor is ~0.3-0.5; randomized in env_cfgs.
  friction={_TOE_REGEX: (0.6,), _BODY_REGEX: (0.4,), _FRONT_CAP_REGEX: (0.3,),
            _REAR_CAP_REGEX: (0.3,)},
  solimp={_TOE_REGEX: (0.9, 0.95, 0.023)},
  disable_other_geoms=True,
)

KRS_QUAD_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(KRS3302_ACTUATOR,),
  soft_joint_pos_limit_factor=0.9,
)


def get_krs_quad_robot_cfg() -> EntityCfg:
  return EntityCfg(
    init_state=HOME_KEYFRAME,
    collisions=(KRS_COLLISION,),
    spec_fn=get_spec,
    articulation=KRS_QUAD_ARTICULATION,
  )


KRS_QUAD_ACTION_SCALE: dict[str, float] = {JOINT_EXPR: 0.25}


if __name__ == "__main__":
  from mjlab.entity.entity import Entity

  robot = Entity(get_krs_quad_robot_cfg())
  model = robot.spec.compile()
  print("krs_quad_robot entity compiled: nq=%d nu=%d nsensor=%d mass=%.3f kg"
        % (model.nq, model.nu, model.nsensor, sum(model.body_mass)))
  print("home: base %.3f m, %s" % (HOME_BASE_HEIGHT, HOME_JOINT_POS))

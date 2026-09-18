"""arduino_os_quad_robot constants for mjlab.

A ~1 kg 3D-printed quadruped driven by eight Feetech STS3215 serial bus servos:
**two sagittal joints per leg (hip pitch + knee pitch), no abduction**. The
whole machine lives in the X-Z plane, so it can track forward/backward velocity
and yaw (by differential stride) but *not* lateral velocity -- the task config
pins ``lin_vel_y`` to zero for that reason.

Measured from the asset (see arduino_os_quad_robot/scripts/build_mjcf.py):

  mass 1.0378 kg | wheelbase 0.15 m | track 0.21 m | leg 0.102 + 0.119 m
  straight-leg height 0.190 m | home stance height 0.140 m
  holding torque at home: 0.16 Nm worst joint = 5.5 % of stall

The MJCF is built and owned by this repository
(arduino_os_quad_robot/mjcf/arduino_os_quad_robot.xml). It ships a ground plane,
per-joint <position> actuators and a home keyframe for standalone MuJoCo use;
get_spec() strips those three (mjlab supplies terrain, its own actuators and
init_state) and leaves the named toe spheres for CollisionCfg to select.

This file is placed into an unitree_rl_mjlab checkout by rl/scripts/setup.sh, so it
cannot find the MJCF by relative path -- $ARDUINO_QUAD_XML (or
$ARDUINO_QUAD_ROOT) points at it; scripts/env.sh exports both.

Actuator model (Feetech STS3215/ST3215, **12 V** variant, datasheet):

  stall torque 2.942 Nm (30 kgf.cm), no-load speed 4.717 rad/s (45 rpm),
  rated (continuous) torque 0.98 Nm (10 kgf.cm), 1:345 gearbox, 4096 counts/turn

The URDF's <limit effort="2.942" velocity="4.717"> matches that variant exactly.
Note there is also a 7.4 V STS3215 (C001) at 19.5 kgf.cm = 1.91 Nm stall -- if
the robot is actually built with that one, every torque number here is 54 % too
high and the asset has to be rebuilt.

A DC motor's torque falls linearly from stall to zero at no-load speed. That
slope is modelled as passive viscous damping, b = 2.942/4.717 = 0.624 Nm.s/rad,
baked into the MJCF joints; a saturating step command then coasts out at exactly
4.72 rad/s (measured). Without it the sim swings the legs several times faster
than a 45 rpm servo can, which is the classic servo-robot sim2real failure.
``stiffness`` (the servo's internal position loop) and ``armature`` (reflected
rotor inertia of the 1:345 gearbox) are ESTIMATES -- they head the
system-identification list, and the events in env_cfgs.py randomize both widely.
"""

import os
from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

LEGS = ("FL", "FR", "RL", "RR")
BASE_BODY = "base_link"
FOOT_BODIES = tuple("{}_foot".format(leg) for leg in LEGS)
TOE_GEOMS = tuple("{}_toe".format(leg) for leg in LEGS)
TOE_SITES = LEGS

# Datasheet, STS3215 12 V variant (see module docstring for the source).
STALL_TORQUE = 2.942     # Nm  (30 kgf.cm), at zero speed
NO_LOAD_SPEED = 4.717    # rad/s (45 rpm), at zero torque
RATED_TORQUE = 0.98      # Nm  (10 kgf.cm) continuous. Not a simulation limit --
                         # nothing here models heating -- but the number a gait
                         # should stay under in RMS if the servos are to survive
                         # more than a few minutes of walking.

# Home stance: all four legs identical, knee apex backwards, each toe under its
# own hip axis. Solved by IK (see arduino_os_quad_robot/scripts/build_mjcf.py);
# the table is precomputed so the task does not need an IK solver at import.
#
# How extended the leg is at each height, and the sagittal toe Jacobian's
# condition number (lower = the leg can push in any direction equally well):
#
#   base 0.14 -> 76 % of full reach, knee 99 deg, cond 3.5   <- default
#   base 0.13 -> 71 %,               knee 90 deg, cond 3.1
#   base 0.12 -> 66 %,               knee 82 deg, cond 2.8
#   base 0.11 -> 61 %,               knee 75 deg, cond 2.6
#   base 0.10 -> 56 %,               knee 68 deg, cond 2.4
#   (q = 0 would be 99 % extended, knee 176 deg, cond 76 -- near singular. The
#    robot is never reset there.)
#
# $ARDUINO_QUAD_HOME_HEIGHT picks one, for A/B-ing stance height without
# touching the asset. Changing it changes default_joint_pos, so a policy trained
# at one height must be deployed with the matching value (it is written into the
# exported header).
# The 2-link leg has two IK solutions for a toe under the hip. This is the one
# with the knee TRAILING (0.06-0.08 m behind the hip axis), which is both the
# assembly the physical robot has and the one that walks: a scripted trot gets
# 0.156 m/s at 8.3 deg of body pitch this way versus 0.089 m/s at up to 21.5 deg
# with the knee leading (arduino_os_quad_robot/scripts/openloop_trot.py).
#
# These angles are in the yaw-flipped frame that normalize_urdf.py produces --
# the CAD export's +X end is the tail. Re-solve them if that ever changes.
_HOME_TABLE = {  # base height [m] -> (hip, knee) [rad]
  0.10: (1.1118, -1.8874),
  0.11: (1.0307, -1.7629),
  0.12: (0.9484, -1.6327),
  0.13: (0.8637, -1.4950),
  0.14: (0.7748, -1.3474),
}
HOME_BASE_HEIGHT = round(float(os.environ.get("ARDUINO_QUAD_HOME_HEIGHT", "0.14")), 2)
if HOME_BASE_HEIGHT not in _HOME_TABLE:
  raise ValueError(
    "ARDUINO_QUAD_HOME_HEIGHT must be one of {} (got {})".format(
      sorted(_HOME_TABLE), HOME_BASE_HEIGHT))
HOME_HIP, HOME_KNEE = _HOME_TABLE[HOME_BASE_HEIGHT]


def _resolve_xml() -> Path:
  """Locate the MJCF from the environment (no relative path is possible here)."""
  xml = os.environ.get("ARDUINO_QUAD_XML")
  if not xml:
    root = os.environ.get("ARDUINO_QUAD_ROOT")
    if not root:
      raise RuntimeError(
        "ARDUINO_QUAD_XML / ARDUINO_QUAD_ROOT is not set: this task cannot find "
        "the arduino_os_quad_robot MJCF. Run through rl/scripts/train.sh or "
        "rl/scripts/play.sh, or `source rl/scripts/env.sh` "
        "first."
      )
    xml = os.path.join(root, "arduino_os_quad_robot", "mjcf",
                       "arduino_os_quad_robot.xml")
  path = Path(xml)
  if not path.exists():
    raise FileNotFoundError(f"arduino_os_quad_robot MJCF not found: {path}")
  return path


ARDUINO_QUAD_XML: Path = _resolve_xml()


def get_spec() -> mujoco.MjSpec:
  # meshdir is relative ("assets"); MjSpec resolves it next to the xml on disk.
  spec = mujoco.MjSpec.from_file(str(ARDUINO_QUAD_XML))
  for act in list(spec.actuators):   # mjlab adds actuators from the cfg below
    spec.delete(act)
  for key in list(spec.keys):        # mjlab resets from init_state (HOME_KEYFRAME)
    spec.delete(key)
  try:
    spec.delete(spec.geom("ground"))  # mjlab adds terrain
  except Exception:  # noqa: BLE001 - ground may be absent if rebuilt without it
    pass
  return spec


##
# Actuators. One group: all eight joints are the same servo.
#
# effort_limit is the datasheet STALL torque. That is a hard cap, not a working
# point: the home stance needs 0.16 Nm, and the gait should stay near the
# 0.98 Nm continuous rating in RMS (scripts/quad_gait_report.sh measures it).
# The torque-speed roll-off is in the MJCF as viscous damping (see module
# docstring), so this does not hand the policy 2.9 Nm at 4 rad/s.
##
# $ARDUINO_QUAD_EFFORT_SCALE multiplies the effort limit (default 1.0 = spec).
_EFFORT_SCALE = float(os.environ.get("ARDUINO_QUAD_EFFORT_SCALE", "1.0"))

# Actuator command lag, in PHYSICS timesteps (dt = 0.005 s).
#
# This was 1..4 steps (5..20 ms) on the reasoning that the lag is "bus round
# trip + inference". Measured on hardware, that pair is only about 4 ms
# (bus 0.2 ms + inference 3.6 ms). The servo's own dead time -- command
# processing and motion profiling inside the STS3215 -- adds ~23 ms, and that
# term did not exist in the estimate at all. It was not a bad guess; it was a
# missing line item.
#
# Total 27 ms = 5.4 physics steps = 1.35 CONTROL periods: the servo is acting on
# a command older than one control step. Identified independently on two joints
# (FL_hip 26.7 ms, FL_knee 28.8 ms), so 4..7 steps = 20..35 ms brackets it.
# $ARDUINO_QUAD_DELAY_MIN / _MAX override for play-time sweeps.
_DELAY_MIN_LAG = int(os.environ.get("ARDUINO_QUAD_DELAY_MIN", "4"))
_DELAY_MAX_LAG = int(os.environ.get("ARDUINO_QUAD_DELAY_MAX", "7"))

STS3215_ACTUATOR = BuiltinPositionActuatorCfg(
  target_names_expr=(r"(FL|FR|RL|RR)_(hip|knee)_joint",),
  # MEASURED (2026-08-21, hardware step response, two joints):
  # FL_hip 25.7, FL_knee 30.7, residual 0.0142 rad (0.81 deg). The 110 % band
  # overlaps at 22..38, and everything at or below 18 is rejected by both
  # joints -- the old 20.0 sat at the bottom edge. An earlier fit said 13, but
  # that run had no delay in the model, and an unmodelled dead time can only be
  # explained by lowering the gain. With the delay in, the gain goes up.
  stiffness=25.0,
  # MEASURED: 0.574 / 0.584 on the two joints, i.e. the old estimate was
  # already right. Left as it was rather than chasing the third digit.
  damping=0.5,
  effort_limit=STALL_TORQUE * _EFFORT_SCALE,
  armature=0.002,       # ESTIMATE: reflected rotor inertia, 1:345 gearbox
  frictionloss=0.01,    # ESTIMATE: gearbox dry friction
  viscous_damping=STALL_TORQUE / NO_LOAD_SPEED,  # 0.624, back-EMF (datasheet)
  delay_min_lag=_DELAY_MIN_LAG,
  delay_max_lag=_DELAY_MAX_LAG,
)

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, HOME_BASE_HEIGHT),
  joint_pos={
    **{f"{leg}_hip_joint": HOME_HIP for leg in LEGS},
    **{f"{leg}_knee_joint": HOME_KNEE for leg in LEGS},
  },
  joint_vel={".*": 0.0},
)

##
# Collision: only the four toe spheres collide (r = 8 mm, tangent to the real
# 13 mm bracket end face). Everything else is disabled -- the leg-to-body
# clearance was scanned over the whole joint range and never drops below 23 mm,
# so there is nothing for self-collision to catch.
#
# friction 0.7 assumes a rubber/silicone tip on the bracket. A BARE printed or
# aluminium bracket end is more like 0.3-0.4 on a hard floor; the foot_friction
# event randomizes down to 0.4 so a policy that only works when it is sticky
# does not survive training.
##
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=TOE_GEOMS,
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.7,),
  solimp=(0.9, 0.95, 0.023),
  disable_other_geoms=True,
)

# Recovery needs the WHOLE robot to collide: with only the toes enabled a
# fallen robot's body sinks through the floor and there is nothing to push off.
# Same contype/conaffinity trick as the Go2 config -- contype=1/conaffinity=0
# collides with the terrain (contype 1 & conaffinity 1) but never with itself,
# which is what we want since the leg-to-body clearance never drops below 23 mm
# anyway. The toes keep their own friction/condim/solimp.
_TOE_REGEX = r"^(FL|FR|RL|RR)_toe$"

FULL_COLLISION = CollisionCfg(
  geom_names_expr=(r".*_collision\d*$", _TOE_REGEX),
  condim={_TOE_REGEX: 3, r".*_collision\d*$": 3},
  priority={_TOE_REGEX: 1},
  friction={_TOE_REGEX: (0.7,), r".*_collision\d*$": (0.4,)},
  solimp={_TOE_REGEX: (0.9, 0.95, 0.023)},
  contype=1,
  conaffinity=0,
)

ARDUINO_QUAD_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(STS3215_ACTUATOR,),
  soft_joint_pos_limit_factor=0.9,
)


def get_arduino_quad_robot_cfg(full_collision: bool = False) -> EntityCfg:
  """Robot config. ``full_collision`` turns on the whole-body collision set,
  which only the fall-recovery task needs (walking is cheaper and cleaner with
  four contact spheres)."""
  return EntityCfg(
    init_state=HOME_KEYFRAME,
    collisions=(FULL_COLLISION if full_collision else FEET_ONLY_COLLISION,),
    spec_fn=get_spec,
    articulation=ARDUINO_QUAD_ARTICULATION,
  )


# One number for every joint: the servos are identical.
ARDUINO_QUAD_ACTION_SCALE: dict[str, float] = {
  r"(FL|FR|RL|RR)_(hip|knee)_joint": 0.25,
}


if __name__ == "__main__":
  from mjlab.entity.entity import Entity

  robot = Entity(get_arduino_quad_robot_cfg())
  model = robot.spec.compile()
  print("arduino_os_quad_robot entity compiled: nq=%d nu=%d nsensor=%d"
        % (model.nq, model.nu, model.nsensor))

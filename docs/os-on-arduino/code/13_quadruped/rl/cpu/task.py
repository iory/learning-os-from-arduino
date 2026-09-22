"""Every number of ArduinoQuad-Walk / -Robust, without mjlab.

The mjlab overlay spreads the task over robot_cfg.py, env_cfgs.py, rewards.py,
rl_cfg.py and the upstream velocity recipe they build on. The CPU trainer needs
the same numbers in one place, so this file restates them -- with the SAME
environment-variable knobs, so a run can be reproduced on either trainer with
the same command line. The reasons behind each value live next to it in those
files; they are not repeated here.

``parity_check.py`` compares this file against the mjlab configuration when
mjlab is installed, so the two cannot drift apart silently.
"""

import math
import os
from pathlib import Path


def _f(name: str, default: str) -> float:
    return float(os.environ.get(name, default))


def _i(name: str, default: str) -> int:
    return int(os.environ.get(name, default))


# --- robot (robot_cfg.py) ---------------------------------------------------
ROOT = Path(__file__).resolve().parents[2]  # 13_quadruped/
MJCF = Path(os.environ.get(
    "ARDUINO_QUAD_XML",
    str(ROOT / "arduino_os_quad_robot" / "mjcf" / "arduino_os_quad_robot.xml")))

# MJCF order. It is also the order of the policy's 8 outputs and of the
# joint_pos / joint_vel observations, because mjlab keeps model order.
JOINT_NAMES = (
    "FL_hip_joint", "FL_knee_joint", "RL_hip_joint", "RL_knee_joint",
    "RR_hip_joint", "RR_knee_joint", "FR_hip_joint", "FR_knee_joint",
)
BASE_BODY = "base_link"
# Contact slots come out in MODEL order too (see FOOT_ORDER in env_cfgs.py).
FOOT_ORDER = ("FL", "RL", "RR", "FR")
TOE_GEOMS = tuple("{}_toe".format(leg) for leg in FOOT_ORDER)
TOE_SITES = FOOT_ORDER

STALL_TORQUE = 2.942       # Nm, STS3215 12 V datasheet
NO_LOAD_SPEED = 4.717      # rad/s
RATED_TORQUE = 0.98        # Nm, continuous

HOME_TABLE = {  # base height [m] -> (hip, knee) [rad]
    0.10: (1.1118, -1.8874),
    0.11: (1.0307, -1.7629),
    0.12: (0.9484, -1.6327),
    0.13: (0.8637, -1.4950),
    0.14: (0.7748, -1.3474),
}
HOME_BASE_HEIGHT = round(_f("ARDUINO_QUAD_HOME_HEIGHT", "0.14"), 2)
if HOME_BASE_HEIGHT not in HOME_TABLE:
    raise ValueError("ARDUINO_QUAD_HOME_HEIGHT must be one of {} (got {})".format(
        sorted(HOME_TABLE), HOME_BASE_HEIGHT))
HOME_HIP, HOME_KNEE = HOME_TABLE[HOME_BASE_HEIGHT]
DEFAULT_JOINT_POS = tuple(HOME_HIP if "hip" in j else HOME_KNEE for j in JOINT_NAMES)

# Built-in position actuator, one per joint (BuiltinPositionActuatorCfg).
KP = 25.0
KD = 0.5
EFFORT_LIMIT = STALL_TORQUE * _f("ARDUINO_QUAD_EFFORT_SCALE", "1.0")
ARMATURE = 0.002
FRICTIONLOSS = 0.01
VISCOUS_DAMPING = STALL_TORQUE / NO_LOAD_SPEED   # 0.624, back-EMF
# Command lag in PHYSICS steps, resampled every physics step (mjlab DelayBuffer).
DELAY_MIN_LAG = _i("ARDUINO_QUAD_DELAY_MIN", "4")
DELAY_MAX_LAG = _i("ARDUINO_QUAD_DELAY_MAX", "7")
SOFT_JOINT_POS_LIMIT_FACTOR = 0.9
ACTION_SCALE = 0.25

# Feet-only collision (FEET_ONLY_COLLISION).
TOE_FRICTION = 0.7
TOE_SOLIMP = (0.9, 0.95, 0.023)
TOE_CONDIM = 3

# --- simulation (velocity_env_cfg.py + flat overrides) ----------------------
PHYSICS_DT = 0.005
DECIMATION = 4
STEP_DT = PHYSICS_DT * DECIMATION
EPISODE_LENGTH_S = 20.0
SOLVER_ITERATIONS = 10
SOLVER_LS_ITERATIONS = 20
CCD_ITERATIONS = 50

# --- gait / command (env_cfgs.py) -------------------------------------------
GAIT_PERIOD = _f("ARDUINO_QUAD_GAIT_PERIOD", "0.30")
GAIT_OFFSET = (0.0, 0.5, 0.0, 0.5)   # FOOT_ORDER: FL+RR, then RL+FR
SWING_HEIGHT = _f("ARDUINO_QUAD_SWING_HEIGHT", "0.022")
_CMD_VX_MAX = _f("ARDUINO_QUAD_CMD_VX_MAX", "0.09")
_CMD_VX_BACK_FRAC = _f("ARDUINO_QUAD_CMD_VX_BACK_FRAC", "0.45")
CMD_LIN_X = (-_CMD_VX_BACK_FRAC * _CMD_VX_MAX, _CMD_VX_MAX)
_CMD_WZ_MAX = _f("ARDUINO_QUAD_CMD_WZ_MAX", "0.4")
CMD_ANG_Z = (-_CMD_WZ_MAX, _CMD_WZ_MAX)
CMD_RESAMPLING_TIME = (3.0, 8.0)
CMD_REL_STANDING_ENVS = 0.1
# (common_step_counter threshold, lin_vel_x, ang_vel_z); stage applies once the
# counter is PAST the threshold.
CMD_CURRICULUM = (
    (0, (-0.02, 0.05), (-0.2, 0.2)),
    (600 * 24, CMD_LIN_X, CMD_ANG_Z),
)

# --- observations -----------------------------------------------------------
ACTOR_HISTORY = 3
JOINT_POS_NOISE = 0.02
JOINT_VEL_NOISE = 0.3
PHASE_ZERO_BELOW = 0.1   # |command| below this -> phase observation is zero
ACTOR_TERMS = ("command", "phase", "joint_pos", "joint_vel", "actions")
CRITIC_TERMS = (
    "base_ang_vel", "projected_gravity", "command", "phase", "joint_pos",
    "joint_vel", "actions", "base_lin_vel", "foot_height", "foot_air_time",
    "foot_contact", "foot_contact_forces",
)

# --- rewards (env_cfgs.py: Flat, then the Walk overrides) -------------------
_POSE_STD = _f("ARDUINO_QUAD_POSE_STD", "1.0")
STD_STANDING = {"hip": 0.05, "knee": 0.08}
STD_WALKING = {"hip": 0.25 * _POSE_STD, "knee": 0.40 * _POSE_STD}
STD_RUNNING = STD_WALKING
POSE_WALKING_THRESHOLD = 0.1
POSE_RUNNING_THRESHOLD = 1.5
TRACK_LIN_STD = 0.25 * CMD_LIN_X[1]
TRACK_ANG_STD = _f("ARDUINO_QUAD_ANG_STD_FRAC", "0.7") * CMD_ANG_Z[1]

REWARD_WEIGHTS = {
    "track_linear_velocity": 0.5,
    "track_angular_velocity": 2.0,
    "body_orientation_l2": -1.0,
    "pose": _f("ARDUINO_QUAD_POSE_W", "0.3"),
    "body_ang_vel": -0.05,
    "angular_momentum": -0.005,
    "is_terminated": -50.0,
    "joint_acc_l2": -2.5e-7,
    "joint_pos_limits": -10.0,
    "action_rate_l2": -0.3,
    "foot_gait": 0.75,
    "foot_clearance": _f("ARDUINO_QUAD_CLEARANCE_W", "-6.0"),
    "foot_slip": -1.0,
    "soft_landing": -1e-3,
    "stand_still": -1.0,
    "alive": 0.1,
    "feet_air_time": 2.5,
    "feet_swing_height": _f("ARDUINO_QUAD_SWING_W", "-3.0"),
    "joint_vel_l2": -0.005,
    "joint_torques_l2": -0.002,
    "torque_excess": -2.0,
    "speed_excess": -0.3,
    "base_height": -50.0,
    "track_lin_vel_filtered": 4.0,
    "base_vel_oscillation": -2.0,
    "foot_air_time_excess": -1.0,
}
FOOT_GAIT_THRESHOLD = 0.56
FOOT_GAIT_CMD_THRESHOLD = 0.1
FOOT_CLEARANCE_CMD_THRESHOLD = 0.1
FOOT_SLIP_CMD_THRESHOLD = 0.1
SOFT_LANDING_CMD_THRESHOLD = 0.1
STAND_STILL_CMD_THRESHOLD = 0.1
FEET_AIR_TIME_THRESHOLD = GAIT_PERIOD / 2.0
FEET_AIR_TIME_CMD_THRESHOLD = 0.05
FEET_SWING_CMD_THRESHOLD = 0.05
TORQUE_EXCESS_LIMIT = RATED_TORQUE
SPEED_EXCESS_LIMIT = 0.7 * NO_LOAD_SPEED
VEL_FILTER_TIME = GAIT_PERIOD
FOOT_AIR_TIME_EXCESS_LIMIT = 2.0 * GAIT_PERIOD

# --- terminations -----------------------------------------------------------
FELL_OVER_ANGLE = math.radians(35.0)
MIN_BASE_HEIGHT = 0.06

# --- events: resets and pushes ----------------------------------------------
_ZERO6 = {k: (0.0, 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")}
RESET_POSE_RANGE = {**_ZERO6, "x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)}
RESET_VELOCITY_RANGE = dict(_ZERO6)
RESET_JOINT_POS_RANGE = (0.0, 0.0)
RESET_JOINT_VEL_RANGE = (0.0, 0.0)
# ArduinoQuad-Robust: start from off-nominal states.
ROBUST_POSE_RANGE = {
    "x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (0.0, 0.02),
    "roll": (-0.15, 0.15), "pitch": (-0.15, 0.15), "yaw": (-3.14, 3.14),
}
ROBUST_VELOCITY_RANGE = {
    "x": (-0.1, 0.1), "y": (-0.1, 0.1), "z": (-0.05, 0.05),
    "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.3, 0.3),
}
ROBUST_JOINT_POS_RANGE = (-0.15, 0.15)
ROBUST_JOINT_VEL_RANGE = (-0.5, 0.5)

PUSH_INTERVAL_S = (5.0, 6.0)
PUSH_VELOCITY_RANGE = {
    "x": (-0.25, 0.25), "y": (-0.25, 0.25), "z": (-0.1, 0.1),
    "roll": (-0.4, 0.4), "pitch": (-0.4, 0.4), "yaw": (-0.5, 0.5),
}

# --- events: startup domain randomization (one draw per environment) --------
FOOT_FRICTION_RANGE = (_f("ARDUINO_QUAD_FRICTION_LO", "0.4"),
                       _f("ARDUINO_QUAD_FRICTION_HI", "1.1"))   # abs, shared by 4 toes
_COM = _f("ARDUINO_QUAD_COM_SCALE", "1.0")
BASE_COM_RANGE = ((-0.02 * _COM, 0.02 * _COM),    # add to body_ipos x
                  (-0.01 * _COM, 0.01 * _COM),    # y
                  (-0.01 * _COM, 0.02 * _COM))    # z
_ENC_BIAS = _f("ARDUINO_QUAD_ENCODER_BIAS", "0.03")
ENCODER_BIAS_RANGE = (-_ENC_BIAS, _ENC_BIAS)
KP_SCALE_RANGE = (_f("ARDUINO_QUAD_KP_LO", "0.6"), _f("ARDUINO_QUAD_KP_HI", "1.6"))
KD_SCALE_RANGE = (_f("ARDUINO_QUAD_KD_LO", "0.4"), _f("ARDUINO_QUAD_KD_HI", "2.0"))
EFFORT_SCALE_RANGE = (0.7, 1.0)
ARMATURE_SCALE_RANGE = (0.5, 2.0)
DAMPING_SCALE_RANGE = (0.7, 1.4)
FRICTIONLOSS_RANGE = (0.0, _f("ARDUINO_QUAD_STICTION", "0.04"))   # abs
PAYLOAD_RANGE = (0.0, _f("ARDUINO_QUAD_PAYLOAD_MAX", "0.25"))     # add to base mass

# ERFI (runner.py): off unless ARDUINO_QUAD_ERFI=1.
ERFI = os.environ.get("ARDUINO_QUAD_ERFI", "0") == "1"
ERFI_OFFSET_SCALE = _f("ARDUINO_QUAD_ERFI_OFF", "0.15")
ERFI_STEP_SCALE = _f("ARDUINO_QUAD_ERFI_STEP", "0.05")

# Optional uneven ground ($ARDUINO_QUAD_TERRAIN=rough), 5-20 mm at amp 1.0.
TERRAIN = os.environ.get("ARDUINO_QUAD_TERRAIN", "plane")
TERRAIN_AMP = _f("ARDUINO_QUAD_TERRAIN_AMP", "1.0")


# --- PPO (rl_cfg.py; the dict mjlab hands to rsl_rl) ------------------------
def ppo_runner_cfg() -> dict:
    """``asdict(arduino_quad_ppo_runner_cfg())`` with tensorboard logging."""
    return {
        "class_name": "OnPolicyRunner",
        "seed": 42,
        "num_steps_per_env": 24,
        "max_iterations": 10001,
        "obs_groups": {"actor": ("actor",), "critic": ("critic",)},
        "save_interval": 100,
        "experiment_name": "arduino_quad_velocity",
        "run_name": "",
        "logger": "tensorboard",
        "clip_actions": None,
        "actor": {
            "class_name": "MLPModel",
            "hidden_dims": (96, 64),
            "activation": "elu",
            "obs_normalization": True,
            "distribution_cfg": {
                "class_name": "GaussianDistribution", "init_std": 1.0, "std_type": "scalar"},
        },
        "critic": {
            "class_name": "MLPModel",
            "hidden_dims": (256, 128),
            "activation": "elu",
            "obs_normalization": True,
            "distribution_cfg": None,
        },
        "algorithm": {
            "class_name": "PPO",
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "learning_rate": 1.0e-3,
            "schedule": "adaptive",
            "gamma": 0.99,
            "lam": 0.95,
            "entropy_coef": 0.01,
            "desired_kl": 0.01,
            "max_grad_norm": 1.0,
            "value_loss_coef": 1.0,
            "use_clipped_value_loss": True,
            "clip_param": 0.2,
            "normalize_advantage_per_mini_batch": False,
            "optimizer": "adam",
            "share_cnn_encoders": False,
        },
    }

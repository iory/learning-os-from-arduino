"""PPO config for the KrsQuad-* tasks.

Same network sizes as the Feetech policy (rl_cfg.py) and for the same reason:
the actor has to run at 50 Hz on the Arduino Uno R4 WiFi. With the IMU the
input is 105 (35 per step x 3), so the actor is 105 -> 96 -> 64 -> 8,
16,904 parameters, 66 KB as float32. Its own experiment name keeps its runs
and checkpoints apart from the Feetech ones.
"""

from mjlab.rl import RslRlOnPolicyRunnerCfg

from .rl_cfg import arduino_quad_ppo_runner_cfg


def krs_quad_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  cfg = arduino_quad_ppo_runner_cfg()
  cfg.experiment_name = "krs_quad_velocity"
  cfg.max_iterations = 4000
  return cfg

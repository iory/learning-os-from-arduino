"""PPO config for the arduino_os_quad_robot velocity task.

The actor is deliberately tiny. It has to run at 50 Hz on the robot's Arduino
Uno R4 WiFi (Renesas RA4M1: 48 MHz Cortex-M4, 256 KB flash, 32 KB RAM), so the
upstream (512, 256, 128) actor -- ~190 k parameters, 750 KB as float32 -- is not
deployable and there is no way to shrink it after training. Sizing:

  actor obs   = 8 joint pos + 8 joint vel + 8 actions + 3 command + 2 phase = 29
                x 3 history steps                                          = 87
  actor net   = 87->96->64->8
  parameters  = 87*96 + 96*64 + 64*8 + biases                          = 15,176
  as float32                                                           = 61 KB
  MACs / inference                                                     = 15,008
  at 50 Hz                                                             = 0.75 MMAC/s

which leaves most of the flash for the sketch and the Feetech bus driver, and is
a fraction of a millisecond of compute per control step on an M4.

The critic never leaves the training machine, so it stays large -- it also gets
the privileged observations (base linear/angular velocity, projected gravity,
foot heights and contact forces) that the actor cannot have on this hardware.
"""

from mjlab.rl import (
  RslRlModelCfg,
  RslRlOnPolicyRunnerCfg,
  RslRlPpoAlgorithmCfg,
)


def arduino_quad_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(96, 64),  # MCU-deployable; see module docstring
      activation="elu",
      obs_normalization=True,
      distribution_cfg={
        "class_name": "GaussianDistribution",
        "init_std": 1.0,
        "std_type": "scalar",
      },
    ),
    critic=RslRlModelCfg(
      hidden_dims=(256, 128),  # training only, never deployed
      activation="elu",
      obs_normalization=True,
    ),
    algorithm=RslRlPpoAlgorithmCfg(
      value_loss_coef=1.0,
      use_clipped_value_loss=True,
      clip_param=0.2,
      entropy_coef=0.01,
      num_learning_epochs=5,
      num_mini_batches=4,
      learning_rate=1.0e-3,
      schedule="adaptive",
      gamma=0.99,
      lam=0.95,
      desired_kl=0.01,
      max_grad_norm=1.0,
    ),
    experiment_name="arduino_quad_velocity",
    save_interval=100,
    num_steps_per_env=24,
    max_iterations=10001,
  )

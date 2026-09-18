"""Task registration for arduino_os_quad_robot (mjlab velocity family).

Registered ids:
  ArduinoQuad-Flat      平地・素の velocity レシピ(立ち上げ/スモーク用)
  ArduinoQuad-Walk      平地 + 歩容シェーピング(本命。トロット歩行はこれ)
  ArduinoQuad-Robust    Walk と同じだが初期状態を崩す(実機前の頑健化 fine-tune)
  ArduinoQuad-Recovery  転倒姿勢から立ち上がる(全身衝突・転倒終了なし)

Rough terrain is deliberately NOT registered: the upstream rough recipe feeds a
height scan into the actor, and this robot has neither a height sensor nor an
IMU, so a rough-terrain policy trained that way could not be deployed.

This package is auto-imported by upstream ``src/tasks/__init__.py``; nothing
else needs to reference it.
"""

from ._compat import apply as _apply_compat

_apply_compat()  # mjlab 1.3.0 + warp 1.14 の版ずれを吸収(詳細は _compat.py)

from mjlab.tasks.registry import register_mjlab_task  # noqa: E402

from .env_cfgs import (  # noqa: E402
  arduino_quad_flat_env_cfg,
  arduino_quad_recovery_env_cfg,
  arduino_quad_robust_env_cfg,
  arduino_quad_walk_env_cfg,
)
from .rl_cfg import arduino_quad_ppo_runner_cfg
from .runner import ArduinoQuadOnPolicyRunner

register_mjlab_task(
  task_id="ArduinoQuad-Flat",
  env_cfg=arduino_quad_flat_env_cfg(),
  play_env_cfg=arduino_quad_flat_env_cfg(play=True),
  rl_cfg=arduino_quad_ppo_runner_cfg(),
  runner_cls=ArduinoQuadOnPolicyRunner,
)
register_mjlab_task(
  task_id="ArduinoQuad-Robust",
  env_cfg=arduino_quad_robust_env_cfg(),
  play_env_cfg=arduino_quad_robust_env_cfg(play=True),
  rl_cfg=arduino_quad_ppo_runner_cfg(),
  runner_cls=ArduinoQuadOnPolicyRunner,
)
register_mjlab_task(
  task_id="ArduinoQuad-Recovery",
  env_cfg=arduino_quad_recovery_env_cfg(),
  play_env_cfg=arduino_quad_recovery_env_cfg(play=True),
  rl_cfg=arduino_quad_ppo_runner_cfg(),
  runner_cls=ArduinoQuadOnPolicyRunner,
)
register_mjlab_task(
  task_id="ArduinoQuad-Walk",
  env_cfg=arduino_quad_walk_env_cfg(),
  play_env_cfg=arduino_quad_walk_env_cfg(play=True),
  rl_cfg=arduino_quad_ppo_runner_cfg(),
  runner_cls=ArduinoQuadOnPolicyRunner,
)

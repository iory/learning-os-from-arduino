"""PPO runner for ArduinoQuad-* with an opt-in ERFI torque perturbation.

ERFI = extended random force injection (Campanaro et al.; arXiv:2504.06585
reports it beating pure parameter DR on hardware). Each control step a
randomized JOINT-TORQUE perturbation is added, so the policy learns to reject
actuator dynamics that parameter DR does not cover.

That matters more here than on a large machine: an STS3215's internal
control loop is a closed firmware black box, its gearbox has backlash and
stick-slip, and its torque sags with battery voltage and winding temperature.
None of that is in the MJCF. Two components:

  * a per-EPISODE constant torque bias, resampled at reset (ARDUINO_QUAD_ERFI_OFF)
  * a per-STEP noise                                       (ARDUINO_QUAD_ERFI_STEP)

both as a fraction of the joint's effort limit (the 2.942 Nm datasheet stall
torque). The action writes only POSITION targets, so this adds a real torque ON
TOP of the position PD -- a non-ideal-actuator regularizer, not an ideal-torque
exploit.

MEASURED ON THIS ROBOT: it does not help. Fine-tuning ArduinoQuad-Robust with
and without ERFI from the same checkpoint, then sweeping push strength (128 envs
x 8 s, scripts/quad_stress.sh):

    push x4    ERFI seed1 80.5 %   seed2 90.6 %   |   no ERFI 89.8 %
    push x6    ERFI seed1 45.3 %   seed2 46.1 %   |   no ERFI 55.5-60.9 %

Both ERFI seeds bracket the no-ERFI result at x4 and sit clearly below it at x6,
so the honest reading is "no measurable benefit", not "harmful" -- seed spread is
~10 points at x4. The reset randomization in the Robust task is what actually
buys robustness here (fine-tuning at all: 66 % -> 90 % at x4).

A plausible reason: this machine's net joint torque runs at a third of the
servo's continuous rating, so a torque-space perturbation is small relative to
what the controller already rejects. On a 60 kg machine (CubeMars actuators
worked near their limits) ERFI did help.

OFF by default; if you want to try it anyway:

    ARDUINO_QUAD_ERFI=1 ARDUINO_QUAD_ERFI_OFF=0.10 ARDUINO_QUAD_ERFI_STEP=0.05 \
      ./scripts/train_quad.sh ArduinoQuad-Walk ...

Inference does not need any of this -- a checkpoint trained with ERFI replays
fine on the plain runner.
"""

import os

import torch

from src.tasks.velocity.rl.runner import VelocityOnPolicyRunner


class ArduinoQuadOnPolicyRunner(VelocityOnPolicyRunner):
  """VelocityOnPolicyRunner + optional ERFI, hooked by wrapping ``env.step``
  (rather than copying the ``learn()`` loop, which would break on rsl_rl drift).
  """

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._erfi_setup()

  def _erfi_setup(self) -> None:
    if os.environ.get("ARDUINO_QUAD_ERFI", "0") != "1":
      return

    robot = self.env.unwrapped.scene["robot"]
    mj = self.env.unwrapped.sim.mj_model
    act_eff = {}
    for a in range(mj.nu):
      act_eff[mj.actuator(a).name.split("/")[-1]] = float(mj.actuator_forcerange[a, 1])
    self._erfi_eff = torch.tensor(
      [act_eff.get(n, 0.0) for n in list(robot.joint_names)], device=self.device)
    self._erfi_off_scale = float(os.environ.get("ARDUINO_QUAD_ERFI_OFF", "0.15"))
    self._erfi_step_scale = float(os.environ.get("ARDUINO_QUAD_ERFI_STEP", "0.05"))
    self._erfi_offset = torch.zeros(
      int(self.env.num_envs), self._erfi_eff.shape[0], device=self.device)
    self._erfi_resample(slice(None))

    base_step = self.env.step

    def _step(actions):
      self._erfi_apply()
      out = base_step(actions)
      dones = out[2]
      if dones.any():
        self._erfi_resample(dones.bool())
      return out

    self.env.step = _step
    print(f"[ArduinoQuad] ERFI on: off_scale={self._erfi_off_scale} "
          f"step_scale={self._erfi_step_scale}, "
          f"{int((self._erfi_eff > 0).sum())} joints", flush=True)

  def _erfi_resample(self, mask) -> None:
    """Resample the per-episode torque bias for the masked (just-reset) envs."""
    o = self._erfi_offset
    if isinstance(mask, torch.Tensor):
      mask = mask.to(o.device)
    o[mask] = (torch.rand_like(o[mask]) * 2 - 1) * self._erfi_off_scale * self._erfi_eff

  def _erfi_apply(self) -> None:
    """Set this step's ERFI torque (episodic bias + fresh per-step noise) as an
    additive joint-effort target; re-set every step (the target is transient)."""
    noise = (
      (torch.rand_like(self._erfi_offset) * 2 - 1)
      * self._erfi_step_scale
      * self._erfi_eff
    )
    self.env.unwrapped.scene["robot"].set_joint_effort_target(self._erfi_offset + noise)

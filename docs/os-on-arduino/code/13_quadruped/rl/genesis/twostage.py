"""Two-stage actor: stage A (the existing 87-dim MLP) + stage B (per-joint residual).

At control step t the env gives a 4-frame actor history. Per term, frames are
oldest first (the same layout as the 3-frame one):

    states  s_{t-3} s_{t-2} s_{t-1} s_t        (command, phase, joint_pos, joint_vel)
    actions a_{t-4} a_{t-3} a_{t-2} a_{t-1}    (the final outputs that were applied)

* Stage A sees the first 3 frames of every term: exactly what the old policy saw
  one control step earlier. ``a_hat`` = its output (8).
* Stage B, one small MLP per joint (8 separate ones, no weight sharing), sees only
  that joint: ``a_hat_j``, a_{t-3..t-1}_j, q_{t-2..t}_j, qd_{t-2..t}_j  (10 values),
  and outputs a residual: ``a_j = a_hat_j + delta_j``. The last layer starts at 0,
  so a = a_hat at the start of training.

Modes: ``baseline`` is stage A alone (no B), trained from scratch in the same env;
``frozen`` loads stage A from a checkpoint and freezes it (normalizer
included); ``joint`` trains A and B together from scratch.
"""

import sys
from pathlib import Path

import torch
import torch.nn as nn
from rsl_rl.models import MLPModel
from rsl_rl.modules import EmpiricalNormalization

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cpu"))
import task as T  # noqa: E402

HISTORY_B = 3        # frames stage B sees of each of a, q, qd
B_IN = 1 + 3 * HISTORY_B
TERM_DIMS = {"command": 3, "phase": 2, "joint_pos": 8, "joint_vel": 8, "actions": 8}


class StageB(nn.Module):
    """8 independent MLPs  B_IN -> H -> H -> 1  (ELU), batched as einsums."""

    def __init__(self, n_joints: int, hidden: int) -> None:
        super().__init__()
        def lin(i, o, zero=False):
            w = torch.zeros(n_joints, i, o) if zero else (torch.rand(n_joints, i, o) * 2 - 1) / i ** 0.5
            return nn.Parameter(w), nn.Parameter(torch.zeros(n_joints, o))
        self.w1, self.b1 = lin(B_IN, hidden)
        self.w2, self.b2 = lin(hidden, hidden)
        self.w3, self.b3 = lin(hidden, 1, zero=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:   # (N, J, B_IN) -> (N, J)
        h = torch.nn.functional.elu(torch.einsum("nji,jio->njo", x, self.w1) + self.b1)
        h = torch.nn.functional.elu(torch.einsum("nji,jio->njo", h, self.w2) + self.b2)
        return (torch.einsum("nji,jio->njo", h, self.w3) + self.b3).squeeze(-1)


class TwoStageActor(MLPModel):
    def __init__(self, obs, obs_groups, obs_set, output_dim, hidden_dims=(96, 64), activation="elu",
                 obs_normalization=True, distribution_cfg=None, mode="frozen", hidden_b=16,
                 history=4) -> None:
        assert mode in ("frozen", "joint", "baseline")
        assert history == HISTORY_B + 1
        self.mode, self.history, self.nj = mode, history, output_dim
        self._a_dim = sum(TERM_DIMS[t] for t in T.ACTOR_TERMS) * (history - 1)
        super().__init__(obs, obs_groups, obs_set, output_dim, hidden_dims, activation,
                         False, distribution_cfg)             # self.mlp is stage A
        self.stage_a_norm = EmpiricalNormalization(self._a_dim)
        if mode != "baseline":       # baseline = stage A alone, trained from scratch (the comparison)
            self.stage_b = StageB(output_dim, hidden_b)
            self.b_norm = EmpiricalNormalization((output_dim, B_IN))
        if mode == "frozen":
            for p in list(self.mlp.parameters()) + list(self.stage_a_norm.parameters()):
                p.requires_grad_(False)

    def _get_latent_dim(self) -> int:       # stage A's input, not the whole 4-frame obs
        return self._a_dim

    # --- splitting the 4-frame observation ------------------------------------

    def _terms(self, obs) -> dict:
        x = torch.cat([obs[g] for g in self.obs_groups], dim=-1)
        out, off = {}, 0
        for t in T.ACTOR_TERMS:
            d = TERM_DIMS[t]
            out[t] = x[:, off:off + d * self.history].reshape(-1, self.history, d)
            off += d * self.history
        return out

    def _a_input(self, terms) -> torch.Tensor:           # frames 0..2 of every term
        return torch.cat([terms[t][:, :self.history - 1].flatten(1) for t in T.ACTOR_TERMS], dim=1)

    def _a_hat(self, terms) -> torch.Tensor:
        return self.mlp(self.stage_a_norm(self._a_input(terms)))

    def _b_input(self, terms, a_hat) -> torch.Tensor:    # (N, J, 10), frames 1..3
        sl = slice(1, self.history)
        return torch.cat([a_hat[:, :, None],
                          terms["actions"][:, sl].transpose(1, 2),
                          terms["joint_pos"][:, sl].transpose(1, 2),
                          terms["joint_vel"][:, sl].transpose(1, 2)], dim=2)

    # --- MLPModel interface -----------------------------------------------------

    def forward(self, obs, masks=None, hidden_state=None, stochastic_output=False):
        terms = self._terms(obs)
        a_hat = self._a_hat(terms)
        mean = a_hat
        if self.mode != "baseline":
            mean = a_hat + self.stage_b(self.b_norm(self._b_input(terms, a_hat)))
        if self.distribution is not None:
            if stochastic_output:
                self.distribution.update(mean)
                return self.distribution.sample()
            return self.distribution.deterministic_output(mean)
        return mean

    def update_normalization(self, obs) -> None:
        terms = self._terms(obs)
        if self.mode != "frozen":
            self.stage_a_norm.update(self._a_input(terms))
        if self.mode == "baseline":
            return
        with torch.no_grad():
            self.b_norm.update(self._b_input(terms, self._a_hat(terms)))

    def train(self, mode: bool = True):
        super().train(mode)
        if self.mode == "frozen":      # the frozen normalizer must not drift
            self.stage_a_norm.eval()
        return self

    def load_stage_a(self, path: str) -> None:
        """Stage A and its normalizer from a single-stage checkpoint (87 -> 96 -> 64 -> 8)."""
        sd = torch.load(path, map_location="cpu", weights_only=False)["actor_state_dict"]
        self.mlp.load_state_dict({k[4:]: v for k, v in sd.items() if k.startswith("mlp.")})
        self.stage_a_norm.load_state_dict(
            {k[len("obs_normalizer."):]: v for k, v in sd.items() if k.startswith("obs_normalizer.")})

    def as_jit(self):
        raise NotImplementedError("the two-stage actor is not exported (sim only)")

    as_onnx = as_jit


def apply(cfg: dict, mode: str, init_std: float | None = None, hidden_b: int = 16) -> dict:
    """The runner cfg with the actor replaced by the two-stage one."""
    cfg["actor"].update(class_name="twostage:TwoStageActor", mode=mode, hidden_b=hidden_b,
                        history=HISTORY_B + 1)
    cfg["actor"]["distribution_cfg"]["init_std"] = init_std if init_std is not None else (
        0.2 if mode == "frozen" else 1.0)
    return cfg


def from_checkpoint(path: str):
    """(runner cfg, GenesisQuadEnv kwargs) for a two-stage checkpoint, or None for a single-stage one."""
    sd = torch.load(path, map_location="cpu", weights_only=False)["actor_state_dict"]
    if "stage_a_norm._mean" not in sd:
        return None
    has_b = "stage_b.w1" in sd
    cfg = apply(T.ppo_runner_cfg(), "joint" if has_b else "baseline",
                hidden_b=sd["stage_b.w1"].shape[-1] if has_b else 16)
    return cfg, {"actor_history": HISTORY_B + 1, "delay_min_lag": 0, "delay_max_lag": 0}

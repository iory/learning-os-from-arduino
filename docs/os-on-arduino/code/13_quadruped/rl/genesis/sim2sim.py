"""Walk one policy in Genesis and in MuJoCo at fixed commands, with the same numbers.

    uv run sim2sim.py --checkpoint logs/rsl_rl/arduino_quad_velocity/<run>/model_2299.pt
    uv run sim2sim.py --npz ../../arduino_quad_policy.npz --sim genesis   # the shipped (MuJoCo) policy in Genesis

``--sim mujoco`` is the CPU trainer's env (``../cpu/quad_env.py``), the same
physics as the browser simulator. ``--sim genesis`` is this directory's env.
Both run in play mode (no observation noise, no pushes) with the nominal robot
unless ``--dr``. The columns are those of ``../cpu/evaluate.py``: speeds over the
second half from the base displacement along its own heading, roll / pitch RMS,
falls, and the gait (duty range, diagonal agreement, toe lift, slip).
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cpu"))
import task as T  # noqa: E402


def npz_policy(path: str, device):
    """Torch forward pass of an exported policy (host/export_quad_policy.py)."""
    z = np.load(path)
    layers = [(torch.tensor(z["w%d" % i], device=device), torch.tensor(z["b%d" % i], device=device))
              for i in range(int(z["n_layers"]))]
    mean = torch.tensor(z["obs_mean"], device=device)
    std = torch.tensor(z["obs_std"], device=device)

    def act(obs):
        x = (obs["actor"].to(device) - mean) / std
        for w, b in layers[:-1]:
            x = torch.nn.functional.elu(x @ w.T + b)
        return x @ layers[-1][0].T + layers[-1][1]

    return act


class MujocoView:
    """Base pose and toe state of the CPU env, as numpy."""

    def __init__(self, n: int, dr: bool, seed: int) -> None:
        from quad_env import QuadEnv
        self.env = QuadEnv(n, play=True, domain_rand=dr, seed=seed)
        self.device = torch.device("cpu")

    def pose(self):
        q = self.env.qpos
        return q[:, :3].copy(), q[:, 3:7].copy()

    def feet(self):
        e, n = self.env, self.env.num_envs
        sd = e.sd
        return (sd[:, e.found_idx] > 0,
                sd[:, e.sitepos_idx].reshape(n, -1, 3)[..., 2],
                np.linalg.norm(sd[:, e.sitevel_idx].reshape(n, -1, 3)[..., :2], axis=-1))


class GenesisView:
    """The same readings from the Genesis env."""

    def __init__(self, n: int, dr: bool, seed: int, device: str) -> None:
        from gs_env import GenesisQuadEnv
        self.env = GenesisQuadEnv(n, play=True, domain_rand=dr, device=device, seed=seed)
        self.device = self.env.device

    def pose(self):
        r = self.env.root
        return r["pos"].cpu().numpy(), r["quat"].cpu().numpy()

    def feet(self):
        e = self.env
        return (e.in_contact.cpu().numpy(), e.foot_pos[..., 2].cpu().numpy(),
                torch.linalg.norm(e.foot_vel[..., :2], dim=-1).cpu().numpy())


def quat_rpy_yaw(q: np.ndarray):
    w, x, y, z = q.T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def run(view, policy, vx_cmd: float, wz_cmd: float, seconds: float) -> dict:
    env = view.env
    n = env.num_envs
    env.set_command(vx_cmd, wz_cmd)
    obs = env.get_observations()
    steps = int(round(seconds / T.STEP_DT))
    half = steps // 2
    falls = np.zeros(n, dtype=np.int64)
    rolls, pitches, contacts, foot_z, foot_v = [], [], [], [], []
    for t in range(steps):
        obs, _, done, extras = env.step(policy(obs).to(view.device))
        d = np.asarray(done.cpu()).astype(bool) & ~np.asarray(extras["time_outs"].cpu())
        falls += d
        pos, quat = view.pose()
        roll, pitch, yaw = quat_rpy_yaw(quat)
        if t == half:
            start_xy, start_yaw, last_yaw = pos[:, :2].copy(), yaw.copy(), yaw.copy()
            turned = np.zeros(n)
        if t > half:
            turned += (yaw - last_yaw + math.pi) % (2 * math.pi) - math.pi
            last_yaw = yaw.copy()
            rolls.append(roll)
            pitches.append(pitch)
            c, z, v = view.feet()
            contacts.append(c)
            foot_z.append(z)
            foot_v.append(v)
    span = (steps - half) * T.STEP_DT
    heading = start_yaw + 0.5 * turned
    disp = pos[:, :2] - start_xy
    vx = (disp[:, 0] * np.cos(heading) + disp[:, 1] * np.sin(heading)) / span
    wz = turned / span
    ok = falls == 0
    out = {"cmd_vx": vx_cmd, "cmd_wz": wz_cmd, "falls": int(falls.sum()), "n": n}
    if not ok.any():
        return out
    c = np.array(contacts)[:, ok]
    z = np.array(foot_z)[:, ok]
    v = np.array(foot_v)[:, ok]
    duty = c.mean(axis=(0, 1))
    out.update({
        "vx": float(vx[ok].mean()), "wz": float(wz[ok].mean()),
        "roll": math.degrees(float(np.sqrt(np.mean(np.square(np.array(rolls)[:, ok]))))),
        "pitch": math.degrees(float(np.sqrt(np.mean(np.square(np.array(pitches)[:, ok]))))),
        "duty": (float(duty.min()), float(duty.max())),
        "diag": float(np.mean((c[..., 0] == c[..., 2]) & (c[..., 1] == c[..., 3]))),
        "lift": 1e3 * float(np.mean(np.percentile(z, 90, axis=0))),
        "slip": 1e3 * float((v * c).sum() / max(c.sum(), 1)),
    })
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--checkpoint")
    src.add_argument("--npz")
    p.add_argument("--sim", choices=("genesis", "mujoco", "both"), default="both")
    p.add_argument("--commands", default="0.06,0;0.09,0;0.12,0;0,0.3;0.06,-0.3",
                   help="vx,wz pairs separated by ';'")
    p.add_argument("--envs", type=int, default=64)
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--dr", action="store_true", help="keep the startup domain randomization")
    p.add_argument("--device", default="cuda", help="Genesis device")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    sims = ("genesis", "mujoco") if args.sim == "both" else (args.sim,)
    print("{} ({} envs x {:.0f} s, {} robot)".format(
        args.npz or args.checkpoint, args.envs, args.seconds, "randomized" if args.dr else "nominal"))
    print("{:>7} {:>6} {:>6} | {:>7} {:>7} | {:>5} {:>5} | {:>5} | {:>9} {:>5} {:>5} {:>5}".format(
        "sim", "cmd_vx", "cmd_wz", "vx", "wz", "roll", "pitch", "falls", "duty", "diag", "lift", "slip"))
    for sim in sims:
        view = (GenesisView(args.envs, args.dr, args.seed, args.device) if sim == "genesis"
                else MujocoView(args.envs, args.dr, args.seed))
        if args.npz:
            policy = npz_policy(args.npz, view.device)
        else:
            from rsl_rl.runners import OnPolicyRunner
            runner = OnPolicyRunner(view.env, T.ppo_runner_cfg(), None, str(view.device))
            runner.load(args.checkpoint)
            policy = runner.get_inference_policy(device=str(view.device))
        for pair in args.commands.split(";"):
            vx_cmd, wz_cmd = (float(v) for v in pair.split(","))
            with torch.inference_mode():
                r = run(view, policy, vx_cmd, wz_cmd, args.seconds)
            if "vx" not in r:
                print("{:>7} {:6.3f} {:6.3f} | every robot fell ({} falls)".format(sim, vx_cmd, wz_cmd, r["falls"]))
                continue
            print("{:>7} {:6.3f} {:6.3f} | {:7.3f} {:7.3f} | {:5.1f} {:5.1f} | {:5d} | {:4.2f}-{:4.2f} {:5.2f} {:5.1f} {:5.1f}"
                  .format(sim, vx_cmd, wz_cmd, r["vx"], r["wz"], r["roll"], r["pitch"], r["falls"],
                          r["duty"][0], r["duty"][1], r["diag"], r["lift"], r["slip"]))
        view.env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Walk a policy in the plain-MuJoCo env at fixed commands and report numbers.

    uv run evaluate.py --checkpoint logs/rsl_rl/arduino_quad_velocity/<run>/model_1499.pt
    uv run evaluate.py --npz ../../walk/arduino_quad_policy.npz      # an exported policy
    uv run evaluate.py --checkpoint <ckpt> --video walk.mp4          # also film env 0

Each command runs ``--envs`` robots for ``--seconds`` in mjlab's play mode (no
observation noise, no pushes). Speeds are averaged over the second half, from
the base displacement along its own heading, the same way the site's walking
numbers were measured. ``--dr`` keeps the training domain randomization on;
by default every robot is the nominal one.
"""

import argparse
import math
import subprocess
import sys

import mujoco
import numpy as np
import torch

import task as T
from quad_env import QuadEnv, quat_to_mat


def npz_policy(path: str):
    """Numpy forward pass of an exported policy (host/export_quad_policy.py)."""
    z = np.load(path)
    layers = [(z["w%d" % i], z["b%d" % i]) for i in range(int(z["n_layers"]))]
    mean, std = z["obs_mean"], z["obs_std"]

    def act(obs: torch.Tensor) -> torch.Tensor:
        x = (obs["actor"].numpy() - mean) / std
        for w, b in layers[:-1]:
            x = x @ w.T + b
            x = np.where(x > 0.0, x, np.expm1(np.minimum(x, 0.0)))
        return torch.from_numpy((x @ layers[-1][0].T + layers[-1][1]).astype(np.float32))

    return act


def checkpoint_policy(path: str, env: QuadEnv):
    from rsl_rl.runners import OnPolicyRunner
    runner = OnPolicyRunner(env, T.ppo_runner_cfg(), None, "cpu")
    runner.load(path)
    return runner.get_inference_policy(device="cpu")


def render_model() -> tuple[mujoco.MjModel, mujoco.MjData]:
    """The full MJCF (meshes included) for filming; physics comes from the env."""
    m = mujoco.MjModel.from_xml_path(str(T.MJCF))
    return m, mujoco.MjData(m)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--checkpoint")
    src.add_argument("--npz")
    p.add_argument("--commands", default="0.06,0;0.09,0;0.12,0;0,0.3;0.06,-0.3",
                   help="vx,wz pairs separated by ';'")
    p.add_argument("--envs", type=int, default=64)
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--dr", action="store_true", help="keep the startup domain randomization")
    p.add_argument("--video", default=None, help="write an mp4 of env 0 for the first command")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    env = QuadEnv(args.envs, play=True, domain_rand=args.dr, seed=args.seed)
    policy = npz_policy(args.npz) if args.npz else checkpoint_policy(args.checkpoint, env)
    steps = int(round(args.seconds / T.STEP_DT))
    half = steps // 2
    print("{} ({} envs x {:.0f} s, {} robot)".format(
        args.npz or args.checkpoint, args.envs, args.seconds, "randomized" if args.dr else "nominal"))
    print("{:>6} {:>6} | {:>7} {:>7} | {:>7} {:>7} | {:>5} {:>5} | {:>5} | {:>9} {:>5} {:>5} {:>5}".format(
        "cmd_vx", "cmd_wz", "vx", "vx_sd", "wz", "wz_sd", "roll", "pitch", "falls",
        "duty", "diag", "lift", "slip"))
    for ci, pair in enumerate(args.commands.split(";")):
        vx_cmd, wz_cmd = (float(v) for v in pair.split(","))
        env.set_command(vx_cmd, wz_cmd)
        obs = env.get_observations()
        falls = np.zeros(args.envs, dtype=np.int64)
        frames = []
        video = args.video if ci == 0 else None
        if video:
            rm, rd = render_model()
            renderer = mujoco.Renderer(rm, 480, 640)
            cam = mujoco.MjvCamera()
            cam.type, cam.distance, cam.elevation, cam.azimuth = mujoco.mjtCamera.mjCAMERA_TRACKING, 0.9, -12.0, 120.0
            cam.trackbodyid = mujoco.mj_name2id(rm, mujoco.mjtObj.mjOBJ_BODY, T.BASE_BODY)
        rolls, pitches, contacts, foot_z, foot_v = [], [], [], [], []
        start_xy = start_yaw = None
        for t in range(steps):
            with torch.inference_mode():
                obs, _, done, extras = env.step(policy(obs))
            d = done.numpy().astype(bool) & ~extras["time_outs"].numpy()
            falls += d
            q = env.qpos
            R = quat_to_mat(q[:, 3:7])
            yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
            if t == half:
                start_xy, start_yaw = q[:, :2].copy(), yaw.copy()
                last_yaw = yaw.copy()
                turned = np.zeros(args.envs)
            if t > half:
                dyaw = (yaw - last_yaw + math.pi) % (2 * math.pi) - math.pi
                turned += dyaw
                last_yaw = yaw.copy()
                rolls.append(np.arctan2(R[:, 2, 1], R[:, 2, 2]))
                pitches.append(np.arcsin(np.clip(-R[:, 2, 0], -1.0, 1.0)))
                sd = env.sd
                contacts.append(sd[:, env.found_idx] > 0)
                foot_z.append(sd[:, env.sitepos_idx].reshape(args.envs, -1, 3)[..., 2])
                foot_v.append(np.linalg.norm(sd[:, env.sitevel_idx].reshape(args.envs, -1, 3)[..., :2], axis=-1))
            if video:
                rd.qpos[:] = q[0]
                mujoco.mj_forward(rm, rd)
                renderer.update_scene(rd, cam)
                frames.append(renderer.render().copy())
        span = (steps - half) * T.STEP_DT
        heading = start_yaw + 0.5 * turned
        disp = q[:, :2] - start_xy
        vx = (disp[:, 0] * np.cos(heading) + disp[:, 1] * np.sin(heading)) / span
        wz = turned / span
        ok = falls == 0
        rms = lambda a: math.degrees(float(np.sqrt(np.mean(np.square(np.array(a)[:, ok]))))) if ok.any() else float("nan")  # noqa: E731
        # Gait quality over the second half, robots that never fell:
        #   duty  fraction of time each foot is on the ground (min-max over feet;
        #         a spread means a favoured or dragged leg)
        #   diag  fraction of time the diagonal pairs (FL+RR, RL+FR) agree -- a
        #         clean trot is near 1
        #   lift  90th-percentile toe-site height [mm], averaged over feet
        #   slip  mean horizontal toe speed while in contact [mm/s]
        c = np.array(contacts)[:, ok]
        z = np.array(foot_z)[:, ok]
        v = np.array(foot_v)[:, ok]
        duty = c.mean(axis=(0, 1))
        diag = np.mean((c[..., 0] == c[..., 2]) & (c[..., 1] == c[..., 3]))
        lift = 1e3 * np.mean(np.percentile(z, 90, axis=0))
        slip = 1e3 * float((v * c).sum() / max(c.sum(), 1))
        print("{:6.3f} {:6.3f} | {:7.3f} {:7.3f} | {:7.3f} {:7.3f} | {:5.1f} {:5.1f} | {:5d} | {:4.2f}-{:4.2f} {:5.2f} {:5.1f} {:5.1f}".format(
            vx_cmd, wz_cmd, vx[ok].mean() if ok.any() else float("nan"), vx[ok].std() if ok.any() else float("nan"),
            wz[ok].mean() if ok.any() else float("nan"), wz[ok].std() if ok.any() else float("nan"),
            rms(rolls), rms(pitches), int(falls.sum()), duty.min(), duty.max(), diag, lift, slip))
        if video:
            fps = int(round(1.0 / T.STEP_DT))
            proc = subprocess.run(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                 "-s", "640x480", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p", video],
                input=np.stack(frames).tobytes())
            print("wrote {} ({} frames)".format(video, len(frames)) if proc.returncode == 0
                  else "ffmpeg failed ({}); no video".format(proc.returncode))
    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Walk one robot with a policy and write the walk to an mp4 (MuJoCo's renderer).

    uv run render.py --checkpoint logs/rsl_rl/arduino_quad_velocity/<run>/model_2299.pt
    uv run render.py --npz ../../arduino_quad_policy.npz --out walk.mp4

The walk is simulated by ``--sim mujoco`` (default; the CPU trainer's env, the
same physics as the browser simulator) or ``--sim genesis``. Either way the
frames are drawn by MuJoCo from the full MJCF, meshes included, with a camera
that follows the base. The commands change in place, without a reset, every
``--seconds``. On Linux with no display (Colab, a server over ssh)
``MUJOCO_GL`` is set to ``egl``; elsewhere, a Mac included, MuJoCo keeps its default.
A ``MUJOCO_GL`` already set is left alone.
"""

import argparse
import os
import shutil
import subprocess
import sys

# MuJoCo picks its OpenGL when it is imported. A Linux machine with no display
# has only EGL; a desktop or a Mac (which has no EGL) is right with the default.
if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from sim2sim import GenesisView, MujocoView, T, npz_policy  # noqa: E402


def qpos_of(view, sim: str) -> np.ndarray:
    """Free joint (pos, quat wxyz) then the 8 legs in MJCF order, for robot 0."""
    if sim == "mujoco":
        return view.env.qpos[0].copy()
    pos, quat = view.pose()
    return np.concatenate([pos[0], quat[0], view.env.jp[0].cpu().numpy()])


def scene_model() -> mujoco.MjModel:
    """The full MJCF with a checkered floor and a sky, so the walk is visible."""
    spec = mujoco.MjSpec.from_file(str(T.MJCF))
    tex = mujoco.mjtTexture
    spec.add_texture(name="grid", type=tex.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.82, 0.82, 0.82], rgb2=[0.62, 0.62, 0.62], width=256, height=256)
    spec.add_texture(name="sky", type=tex.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.55, 0.7, 0.9], rgb2=[0.95, 0.97, 1.0], width=256, height=256)
    mat = spec.add_material(name="grid", texrepeat=[10.0, 10.0], texuniform=True)   # 10 cm squares
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    ground = spec.geom("ground")
    ground.material, ground.rgba = "grid", [1.0, 1.0, 1.0, 1.0]
    return spec.compile()


def set_command(view, sim: str, vx: float, wz: float) -> None:
    """Change the command without resetting the robot."""
    if sim == "mujoco":
        view.env.fixed_command = np.array([vx, 0.0, wz])
    else:
        view.env.fixed_command = torch.tensor([vx, 0.0, wz], device=view.device)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--checkpoint")
    src.add_argument("--npz")
    p.add_argument("--sim", choices=("mujoco", "genesis"), default="mujoco")
    p.add_argument("--commands", default="0.06,0;0.12,0;0,0.3;0.06,-0.3",
                   help="vx,wz pairs separated by ';', played in turn")
    p.add_argument("--seconds", type=float, default=4.0, help="per command")
    p.add_argument("--out", default="walk.mp4")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=360)
    p.add_argument("--fps", type=int, default=25)
    p.add_argument("--dr", action="store_true", help="keep the startup domain randomization")
    p.add_argument("--device", default="gpu", help="Genesis device: gpu (CUDA, or Metal on a Mac) or cpu")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg is needed to write the mp4")
    pairs = [tuple(float(v) for v in s.split(",")) for s in args.commands.split(";")]
    if len(pairs) * args.seconds > T.EPISODE_LENGTH_S:
        sys.exit("{} commands x {} s is longer than one episode ({} s)".format(
            len(pairs), args.seconds, T.EPISODE_LENGTH_S))

    view = (GenesisView(1, args.dr, args.seed, args.device) if args.sim == "genesis"
            else MujocoView(1, args.dr, args.seed))
    if args.npz:
        policy = npz_policy(args.npz, view.device)
    else:
        from rsl_rl.runners import OnPolicyRunner
        runner = OnPolicyRunner(view.env, T.ppo_runner_cfg(), None, str(view.device))
        runner.load(args.checkpoint)
        policy = runner.get_inference_policy(device=str(view.device))

    model = scene_model()
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, args.height, args.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, T.BASE_BODY)
    cam.distance, cam.elevation = 0.7, -20.0

    ffmpeg = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", "{}x{}".format(args.width, args.height), "-r", str(args.fps), "-i", "-",
         "-pix_fmt", "yuv420p", "-vcodec", "libx264", args.out],
        stdin=subprocess.PIPE)
    view.env.set_command(*pairs[0])
    obs = view.env.get_observations()
    w, x, y, z = qpos_of(view, args.sim)[3:7]
    yaw0 = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    cam.azimuth = yaw0 + 135.0   # front-left of the robot's starting heading
    steps = int(round(args.seconds / T.STEP_DT))
    every = max(1, int(round(1.0 / (args.fps * T.STEP_DT))))
    falls, frames = 0, 0
    with torch.inference_mode():
        for vx, wz in pairs:
            set_command(view, args.sim, vx, wz)
            for t in range(steps):
                obs, _, done, extras = view.env.step(policy(obs).to(view.device))
                falls += int(bool(done[0]) and not bool(extras["time_outs"][0]))
                if t % every:
                    continue
                data.qpos[:] = qpos_of(view, args.sim)
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, cam)
                ffmpeg.stdin.write(renderer.render().tobytes())
                frames += 1
    ffmpeg.stdin.close()
    ffmpeg.wait()
    view.env.close()
    print("wrote {} ({} frames, {:.1f} s, {} sim, falls {})".format(
        args.out, frames, frames / args.fps, args.sim, falls))
    return ffmpeg.returncode


if __name__ == "__main__":
    sys.exit(main())

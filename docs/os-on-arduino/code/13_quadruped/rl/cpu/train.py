"""Train ArduinoQuad-Walk / -Robust with PPO on plain MuJoCo.

    uv run train.py                                  # Walk, 2048 envs, CPU
    uv run train.py --device cuda                    # same physics, network on the GPU
    uv run train.py --task robust --resume logs/rsl_rl/arduino_quad_velocity/<run>/model_1999.pt

Physics always runs on the CPU (``mujoco.rollout``); ``--device`` only moves the
PPO update. The runner config, the network and the checkpoint format are those
of the mjlab overlay (rsl-rl-lib 5.0.1), so a checkpoint from either trainer
resumes in the other and goes through the same exporter
(``host/export_quad_policy.py --backend cpu``).

Runs are written to logs/rsl_rl/arduino_quad_velocity/<date>_<run-name>/, the
same layout as mjlab, with params/env.yaml recording the task constants.
"""

import argparse
import ctypes
import datetime
import os
import sys
import time

import numpy as np
import torch
from rsl_rl.runners import OnPolicyRunner

import task as T
from quad_env import QuadEnv

HERE = os.path.dirname(os.path.abspath(__file__))


def physical_cores() -> int:
    """Physical cores this process may use. OpenMP threads spin, so giving
    torch the SMT siblings as well slows the update down and starves the
    rollout pool during collection."""
    try:
        n = len(os.sched_getaffinity(0))
    except AttributeError:   # macOS
        n = os.cpu_count() or 1
    try:
        with open("/sys/devices/system/cpu/smt/active") as f:
            if f.read().strip() == "1":
                n //= 2
    except OSError:
        pass
    return max(1, n)


def keep_freed_memory() -> bool:
    """Let glibc reuse freed PPO minibatch buffers instead of unmapping them.

    A minibatch activation is tens of MB, above glibc's mmap threshold, so by
    default every elementwise op maps fresh pages and the kernel zero-fills
    them. Serving everything from the heap and never trimming it removes those
    page faults. Linux/glibc only; returns False (and changes nothing) elsewhere.
    """
    if not sys.platform.startswith("linux"):
        return False
    try:
        mallopt = ctypes.CDLL("libc.so.6").mallopt
    except (OSError, AttributeError):
        return False
    mallopt.argtypes = (ctypes.c_int, ctypes.c_int)
    m_trim_threshold, m_top_pad, m_mmap_max = -1, -2, -4
    ok = mallopt(m_mmap_max, 0) == 1
    ok = mallopt(m_trim_threshold, 2**31 - 1) == 1 and ok
    return mallopt(m_top_pad, 64 << 20) == 1 and ok


def to_yaml(obj, indent: int = 0) -> str:
    """Minimal YAML for nested dicts / lists of scalars (no dependency)."""
    pad = " " * indent
    if isinstance(obj, dict):
        out = []
        for k, v in obj.items():
            if isinstance(v, (dict, list, tuple)):
                out.append("{}{}:\n{}".format(pad, k, to_yaml(v, indent + 2)))
            else:
                out.append("{}{}: {}\n".format(pad, k, v))
        return "".join(out)
    return "".join(
        "{}-\n{}".format(pad, to_yaml(v, indent + 2)) if isinstance(v, (dict, list, tuple))
        else "{}- {}\n".format(pad, v) for v in obj)


def _plain(v):
    """Tuples -> lists, recursively, so to_yaml writes them as sequences."""
    if isinstance(v, dict):
        return {k: _plain(x) for k, x in v.items()}
    if isinstance(v, (tuple, list)):
        return [_plain(x) for x in v]
    return v


def write_params(log_dir: str, args: argparse.Namespace, agent_cfg: dict) -> None:
    """params/env.yaml and params/agent.yaml, in the shape the exporter reads."""
    os.makedirs(os.path.join(log_dir, "params"), exist_ok=True)
    robust = args.task == "robust"
    env = {
        "trainer": "plain-mujoco",
        "task": "ArduinoQuad-Robust" if robust else "ArduinoQuad-Walk",
        "num_envs": args.num_envs,
        "joint_pos": dict(zip(T.JOINT_NAMES, T.DEFAULT_JOINT_POS)),
        "phase": {"period": T.GAIT_PERIOD},
        "reset_base": {
            "pose_range": {k: list(v) for k, v in
                           (T.ROBUST_POSE_RANGE if robust else T.RESET_POSE_RANGE).items()},
            "velocity_range": {k: list(v) for k, v in
                               (T.ROBUST_VELOCITY_RANGE if robust else T.RESET_VELOCITY_RANGE).items()},
        },
        "reset_robot_joints": {
            "position_range": list(T.ROBUST_JOINT_POS_RANGE if robust else T.RESET_JOINT_POS_RANGE),
            "velocity_range": list(T.ROBUST_JOINT_VEL_RANGE if robust else T.RESET_JOINT_VEL_RANGE),
        },
        "task_constants": {k: _plain(v) for k, v in vars(T).items()
                           if k.isupper() and isinstance(v, (bool, int, float, str, tuple, dict))},
        "env_overrides": {k: v for k, v in sorted(os.environ.items()) if k.startswith("ARDUINO_QUAD_")},
    }
    with open(os.path.join(log_dir, "params", "env.yaml"), "w") as f:
        f.write(to_yaml(env))
    agent = {"experiment_name": agent_cfg["experiment_name"], "run_name": agent_cfg["run_name"],
             "seed": agent_cfg["seed"], "device": args.device,
             "load_run": os.path.basename(os.path.dirname(args.resume)) if args.resume else None,
             "load_checkpoint": os.path.basename(args.resume) if args.resume else None}
    with open(os.path.join(log_dir, "params", "agent.yaml"), "w") as f:
        f.write(to_yaml(agent))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--task", choices=("walk", "robust"), default="walk")
    p.add_argument("--num-envs", type=int, default=2048)
    p.add_argument("--iterations", type=int, default=1500)
    p.add_argument("--device", default="cpu", help="where the PPO update runs: cpu, cuda, cuda:1, ...")
    p.add_argument("--threads", type=int, default=None, help="mujoco.rollout threads (default: all CPUs)")
    p.add_argument("--torch-threads", type=int, default=None, help="default: physical cores")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--run-name", default="")
    p.add_argument("--resume", default=None, help="checkpoint to continue from (mjlab or CPU run)")
    p.add_argument("--log-root", default=os.path.join(HERE, "logs", "rsl_rl"))
    args = p.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    torch.set_num_threads(args.torch_threads or physical_cores())
    print("[train] glibc keeps freed memory: {}".format(keep_freed_memory()))

    agent_cfg = T.ppo_runner_cfg()
    agent_cfg["seed"] = args.seed
    agent_cfg["run_name"] = args.run_name
    agent_cfg["max_iterations"] = args.iterations
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(args.log_root, agent_cfg["experiment_name"],
                           stamp + ("_" + args.run_name if args.run_name else ""))

    env = QuadEnv(args.num_envs, robust=args.task == "robust", threads=args.threads, seed=args.seed)
    print("[train] {} envs, actor obs {}, critic obs {}, rollout threads {}, torch threads {}, "
          "update on {}".format(args.num_envs, env.actor_dim, env.critic_dim, env.threads,
                                torch.get_num_threads(), args.device))
    runner = OnPolicyRunner(env, agent_cfg, log_dir, args.device)
    if args.resume:
        runner.load(args.resume)
        print("[train] resumed from {}".format(args.resume))
    write_params(log_dir, args, agent_cfg)
    t0 = time.time()
    runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=True)
    print("[train] {} iterations in {:.1f} min -> {}".format(
        args.iterations, (time.time() - t0) / 60.0, log_dir))
    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

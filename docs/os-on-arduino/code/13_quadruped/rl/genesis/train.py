"""Train ArduinoQuad-Walk with PPO on Genesis (GPU).

    uv run train.py                                    # Walk, 4096 envs, GPU (CUDA, or Metal on a Mac)
    uv run train.py --num-envs 1024 --iterations 300   # a short check
    uv run train.py --task robust --resume logs/rsl_rl/arduino_quad_velocity/<run>/model_2299.pt

The task, the network and the runner config are those of the CPU trainer
(``../cpu/task.py``), so the checkpoint has the same format and goes through
the same exporter (``host/export_quad_policy.py --backend genesis``). Runs are
written to logs/rsl_rl/arduino_quad_velocity/<date>_<run-name>/ with
params/env.yaml recording the task constants, as for the other two trainers.
"""

import argparse
import datetime
import importlib.util
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
from rsl_rl.runners import OnPolicyRunner

from gs_env import GenesisQuadEnv, T

HERE = Path(__file__).resolve().parent


def cpu_trainer():
    """``../cpu/train.py`` as a module, for its params/*.yaml writer."""
    spec = importlib.util.spec_from_file_location("cpu_train", HERE.parent / "cpu" / "train.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--task", choices=("walk", "robust"), default="walk")
    p.add_argument("--num-envs", type=int, default=4096)
    p.add_argument("--iterations", type=int, default=2300)
    p.add_argument("--device", default="gpu",
                   help="Genesis and the PPO update: gpu (CUDA, or Metal/MPS on a Mac) or cpu")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--run-name", default="genesis")
    p.add_argument("--resume", default=None, help="checkpoint to continue from (any of the three trainers)")
    p.add_argument("--log-root", default=str(HERE / "logs" / "rsl_rl"))
    args = p.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    agent_cfg = T.ppo_runner_cfg()
    agent_cfg["seed"] = args.seed
    agent_cfg["run_name"] = args.run_name
    agent_cfg["max_iterations"] = args.iterations
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(args.log_root, agent_cfg["experiment_name"],
                           stamp + ("_" + args.run_name if args.run_name else ""))

    env = GenesisQuadEnv(args.num_envs, robust=args.task == "robust", device=args.device, seed=args.seed)
    print("[train] Genesis, {} envs, actor obs {}, critic obs {}, device {}".format(
        args.num_envs, env.actor_dim, env.critic_dim, env.device))
    runner = OnPolicyRunner(env, agent_cfg, log_dir, str(env.device))
    if args.resume:
        runner.load(args.resume)
        print("[train] resumed from {}".format(args.resume))
    cpu_trainer().write_params(log_dir, args, agent_cfg)
    env_yaml = Path(log_dir) / "params" / "env.yaml"
    env_yaml.write_text(env_yaml.read_text().replace("trainer: plain-mujoco", "trainer: genesis", 1))
    t0 = time.time()
    runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=True)
    print("[train] {} iterations in {:.1f} min -> {}".format(
        args.iterations, (time.time() - t0) / 60.0, log_dir))
    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

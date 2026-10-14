"""Measure a KrsQuad-* checkpoint: tracking, gait, body motion and servo margins.

Answers the questions the KRS policy has to pass with numbers instead of a
look at a video:

* does it track the command (forward, backward, turning)?
* is it a diagonal trot on all four feet -- no hop, no three-legged gait, no
  dragged or sliding foot?
* how much does the body tilt and sway, and does the belly or a knee touch?
* how close do the joints come to +-135 deg, the stall torque and the no-load
  speed?

It also checks the two things that silently break a trot reward: the actor's
observation size (105 = the IMU layout) and the foot order of the contact
sensor (the gait offsets are positional).

Run inside the upstream checkout (it imports ``src.tasks``)::

    source rl/scripts/env.sh
    cd "$ARDUINO_QUAD_UPSTREAM"
    python "$ARDUINO_QUAD_ROOT/rl/scripts/krs_eval.py" --ckpt <model_*.pt> [--json out.json]

Without --ckpt it only builds the env and runs the two checks (smoke test).
"""
import argparse
import json
import os
from dataclasses import asdict


def _parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--ckpt", default=None)
  p.add_argument("--task", default="KrsQuad-Walk")
  p.add_argument("--num-envs", type=int, default=64)
  p.add_argument("--seconds", type=float, default=10.0)
  p.add_argument("--warmup", type=float, default=2.0, help="ignored at the start [s]")
  p.add_argument("--json", default=None, help="write the results here")
  p.add_argument("--device", default="cuda:0")
  p.add_argument("--push-scale", type=float, default=0.0,
                 help="push every 2 s with the training push ranges x this "
                      "(0 = no pushes). Falls and leg-leg contact then measure "
                      "how the policy recovers")
  return p.parse_args()


# (vx [m/s], wz [rad/s]) commands to measure.
COMMANDS = ((0.0, 0.0), (0.04, 0.0), (0.08, 0.0), (0.12, 0.0), (-0.06, 0.0),
            (0.08, 0.4), (0.0, -0.4))


def main() -> int:
  args = _parse_args()
  os.environ.setdefault("MUJOCO_GL", "egl")

  import numpy as np
  import torch

  import src.tasks  # noqa: F401
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
  from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls

  from src.tasks.velocity.config.arduino_quad import krs_env_cfgs as K
  from src.tasks.velocity.config.arduino_quad import krs_robot_cfg as R

  env_cfg = load_env_cfg(args.task, play=True)
  env_cfg.scene.num_envs = args.num_envs
  twist = env_cfg.commands["twist"]
  twist.rel_standing_envs = 0.0
  twist.resampling_time_range = (1e9, 1e9)
  agent_cfg = load_rl_cfg(args.task)
  base = ManagerBasedRlEnv(cfg=env_cfg, device=args.device)
  env = RslRlVecEnvWrapper(base, clip_actions=agent_cfg.clip_actions)

  # --- checks -----------------------------------------------------------------
  obs_dim = base.observation_manager.group_obs_dim["actor"]
  obs_dim = int(np.prod(obs_dim))
  assert obs_dim == 105, f"actor obs is {obs_dim}, expected 105 (IMU layout)"
  feet = base.scene["feet_ground_contact"]
  slots = [s.primary_name for s in feet._slots if s.field_name == "found"]
  assert slots == list(K.TOE_GEOMS), (slots, K.TOE_GEOMS)
  print(f"[check] actor obs {obs_dim}, foot slots {slots}: OK")
  if args.ckpt is None:
    return 0

  runner = (load_runner_cls(args.task) or MjlabOnPolicyRunner)(
    env, asdict(agent_cfg), device=args.device)
  runner.load(args.ckpt, load_cfg={"actor": True}, strict=True, map_location=args.device)
  policy = runner.get_inference_policy(device=args.device)

  robot = base.scene["robot"]
  belly = base.scene["belly_ground_contact"]
  knees = base.scene["knee_ground_contact"]
  legleg = base.scene["leg_leg_contact"]
  cmd_term = base.command_manager.get_term("twist")
  site_ids = robot.find_sites(K.TOE_SITES, preserve_order=True)[0]
  b_emf = R.STALL_TORQUE / R.NO_LOAD_SPEED
  dt = base.step_dt
  n_steps = int(args.seconds / dt)
  n_warm = int(args.warmup / dt)
  joint_names = list(robot.joint_names)

  results = {"checkpoint": os.path.abspath(args.ckpt), "task": args.task,
             "num_envs": args.num_envs, "seconds": args.seconds, "commands": []}
  q_min = np.full(len(joint_names), np.inf)
  q_max = np.full(len(joint_names), -np.inf)
  tau_peak = np.zeros(len(joint_names))
  act_peak = np.zeros(len(joint_names))
  qd_peak = np.zeros(len(joint_names))

  from mjlab.envs.mdp.events import push_by_setting_velocity
  push_ranges = {k: (args.push_scale * lo, args.push_scale * hi) for k, (lo, hi) in {
    "x": (-0.2, 0.2), "y": (-0.2, 0.2), "z": (-0.1, 0.1),
    "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.4, 0.4)}.items()}
  all_ids = torch.arange(args.num_envs, device=args.device)
  results["push_scale"] = args.push_scale

  for vx, wz in COMMANDS:
    env.reset()
    obs = env.get_observations()
    rec = {k: [] for k in ("vx", "wz", "contact", "roll", "pitch", "z", "vz",
                           "slip", "belly", "knee", "legleg", "acc", "tau", "qd")}
    falls = torch.zeros(args.num_envs, dtype=torch.bool, device=args.device)
    prev = [None, None]
    for t in range(n_steps):
      cmd_term.command[:, 0] = vx
      cmd_term.command[:, 1] = 0.0
      cmd_term.command[:, 2] = wz
      if args.push_scale > 0 and t % int(2.0 / dt) == int(1.0 / dt):
        push_by_setting_velocity(base, all_ids, push_ranges)
      with torch.inference_mode():
        a = policy(obs)
      obs, _, dones, _ = env.step(a)
      falls |= dones.bool() & ~base.termination_manager.time_outs
      if t < n_warm:
        prev = [a.clone(), prev[0]]
        continue
      g = robot.data.projected_gravity_b
      found = (feet.data.found > 0)
      speed = torch.norm(robot.data.site_lin_vel_w[:, site_ids, :2], dim=-1)
      rec["vx"].append(robot.data.root_link_lin_vel_b[:, 0])
      rec["wz"].append(robot.data.root_link_ang_vel_b[:, 2])
      rec["contact"].append(found)
      rec["roll"].append(torch.atan2(g[:, 1], -g[:, 2]))
      rec["pitch"].append(torch.atan2(-g[:, 0], -g[:, 2]))
      rec["z"].append(robot.data.root_link_pos_w[:, 2])
      rec["vz"].append(robot.data.root_link_lin_vel_b[:, 2])
      rec["slip"].append((speed * found).sum(1) / found.sum(1).clamp(min=1))
      rec["belly"].append((belly.data.found > 0).any(1))
      rec["knee"].append((knees.data.found > 0).any(1))
      rec["legleg"].append((legleg.data.found > 0).any(1))
      if prev[1] is not None:
        rec["acc"].append((a - 2 * prev[0] + prev[1]).abs().mean(1))
      prev = [a.clone(), prev[0]]
      qd = robot.data.joint_vel
      net = robot.data.actuator_force - b_emf * qd
      rec["tau"].append(net)
      rec["qd"].append(qd)
      q = robot.data.joint_pos.cpu().numpy()
      q_min = np.minimum(q_min, q.min(0))
      q_max = np.maximum(q_max, q.max(0))
      act_peak = np.maximum(act_peak, robot.data.actuator_force.abs().max(0)[0].cpu().numpy())

    ok = ~falls
    S = {k: torch.stack(v).float() for k, v in rec.items()}  # [T, B, ...]
    c = S["contact"][:, ok]                                   # [T, B, 4]
    n_down = c.sum(-1)
    # FL, RL, RR, FR -> diagonal pairs (FL, RR) and (RL, FR)
    diag = ((c[..., 0] == c[..., 2]) & (c[..., 1] == c[..., 3])).float()
    pure_trot = (((c[..., 0] == 1) & (c[..., 2] == 1) & (c[..., 1] == 0) & (c[..., 3] == 0))
                 | ((c[..., 0] == 0) & (c[..., 2] == 0) & (c[..., 1] == 1) & (c[..., 3] == 1))).float()
    touchdowns = ((c[1:] == 1) & (c[:-1] == 0)).float().sum(0)  # [B, 4]
    duration = c.shape[0] * dt
    tau = S["tau"][:, ok]
    qd = S["qd"][:, ok]
    tau_peak = np.maximum(tau_peak, tau.abs().amax((0, 1)).cpu().numpy())
    qd_peak = np.maximum(qd_peak, qd.abs().amax((0, 1)).cpu().numpy())
    deg = 180.0 / np.pi
    row = {
      "cmd_vx": vx, "cmd_wz": wz,
      "fell": int(falls.sum()),
      "vx_mean": float(S["vx"][:, ok].mean()),
      "wz_mean": float(S["wz"][:, ok].mean()),
      "duty_per_foot": [round(float(x), 3) for x in c.mean((0, 1))],
      "step_hz_per_foot": [round(float(x), 2) for x in (touchdowns.mean(0) / duration)],
      "diagonal_sync": float(diag.mean()),
      "pure_trot_2feet": float(pure_trot.mean()),
      "flight_0feet": float((n_down == 0).float().mean()),
      "three_plus_feet": float((n_down >= 3).float().mean()),
      "one_foot": float((n_down == 1).float().mean()),
      "stance_slip_mps": float(S["slip"][:, ok].mean()),
      "roll_deg_mean": float(S["roll"][:, ok].mean() * deg),
      "roll_deg_std": float(S["roll"][:, ok].std() * deg),
      "pitch_deg_mean": float(S["pitch"][:, ok].mean() * deg),
      "pitch_deg_std": float(S["pitch"][:, ok].std() * deg),
      "tilt_deg_p99": float(torch.quantile(
        torch.sqrt(S["roll"][:, ok] ** 2 + S["pitch"][:, ok] ** 2).flatten()[:200000], 0.99) * deg),
      "base_z_mean": float(S["z"][:, ok].mean()),
      "base_z_std_mm": float(S["z"][:, ok].std(0).mean() * 1e3),
      "vz_rms": float(S["vz"][:, ok].pow(2).mean().sqrt()),
      "belly_contact": float(S["belly"][:, ok].mean()),
      "knee_contact": float(S["knee"][:, ok].mean()),
      "leg_leg_contact": float(S["legleg"][:, ok].mean()),
      "action_jerk": float(S["acc"][:, ok].mean()),
      "net_torque_rms_max_joint": float(tau.pow(2).mean((0, 1)).sqrt().max()),
      "net_torque_p99": float(torch.quantile(tau.abs().flatten()[:500000], 0.99)),
      "joint_speed_p99": float(torch.quantile(qd.abs().flatten()[:500000], 0.99)),
    }
    results["commands"].append(row)
    print(json.dumps(row))

  results["joint_range_deg"] = {
    n: [round(float(np.degrees(lo)), 1), round(float(np.degrees(hi)), 1)]
    for n, lo, hi in zip(joint_names, q_min, q_max)}
  results["limit_deg"] = round(float(np.degrees(R.JOINT_LIMIT)), 1)
  results["home_deg"] = {n: round(float(np.degrees(v)), 1) for n, v in R.HOME_JOINT_POS.items()}
  results["net_torque_peak_Nm"] = dict(zip(joint_names, np.round(tau_peak, 3).tolist()))
  results["actuator_force_peak_Nm"] = dict(zip(joint_names, np.round(act_peak, 3).tolist()))
  results["joint_speed_peak_radps"] = dict(zip(joint_names, np.round(qd_peak, 2).tolist()))
  results["stall_torque_Nm"] = R.STALL_TORQUE
  results["no_load_speed_radps"] = R.NO_LOAD_SPEED
  print(json.dumps({k: v for k, v in results.items() if k != "commands"}, indent=1))
  if args.json:
    with open(args.json, "w") as f:
      json.dump(results, f, indent=2)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())

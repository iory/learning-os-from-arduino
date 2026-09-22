"""Check that the plain-MuJoCo task is the mjlab task.

Run it with the mjlab environment's Python (the one ``rl/scripts/setup.sh``
installs), from this directory. It does not need a GPU: mjlab runs on warp's
CPU backend here.

    source ../scripts/env.sh
    python parity_check.py [--npz ../../walk/arduino_quad_policy.npz]

1. Configuration: every number in task.py against the live mjlab config of
   ArduinoQuad-Walk / -Robust (reward weights and parameters, observation
   noise, commands and their curriculum, events, terminations, actuator,
   delay, timing, PPO).
2. Dynamics: both envs start from the same state, get the same command and the
   same actions (from ``--npz``, or a fixed smooth pattern), with the command
   delay pinned to 5 physics steps and no domain randomization so nothing is
   drawn at random. Joint and base state, the actor observation and every
   reward term are compared step by step.

Exit status is non-zero when something does not match.
"""

import argparse
import math
import os
import sys

# Pin everything the dynamics comparison would otherwise draw at random.
# Both task.py and robot_cfg.py read these at import time.
os.environ.setdefault("ARDUINO_QUAD_DELAY_MIN", "5")
os.environ.setdefault("ARDUINO_QUAD_DELAY_MAX", "5")

import numpy as np  # noqa: E402
import torch  # noqa: E402

import task as T  # noqa: E402

FAILS: list[str] = []


def check(name: str, ours, theirs, tol: float = 1e-9) -> None:
    a = np.asarray(ours, dtype=np.float64).ravel()
    b = np.asarray(theirs, dtype=np.float64).ravel()
    ok = a.shape == b.shape and bool(np.all(np.abs(a - b) <= tol))
    if not ok:
        FAILS.append("{}: task.py {} vs mjlab {}".format(name, ours, theirs))


def check_config() -> None:
    import src.tasks  # noqa: F401  (registers ArduinoQuad-*)
    from src.tasks.velocity.config.arduino_quad import env_cfgs, rl_cfg, robot_cfg

    walk = env_cfgs.arduino_quad_walk_env_cfg()
    robust = env_cfgs.arduino_quad_robust_env_cfg()
    check("physics_dt", T.PHYSICS_DT, walk.sim.mujoco.timestep)
    check("decimation", T.DECIMATION, walk.decimation)
    check("episode_length_s", T.EPISODE_LENGTH_S, walk.episode_length_s)
    check("solver iterations", (T.SOLVER_ITERATIONS, T.SOLVER_LS_ITERATIONS, T.CCD_ITERATIONS),
          (walk.sim.mujoco.iterations, walk.sim.mujoco.ls_iterations, walk.sim.mujoco.ccd_iterations))

    act = robot_cfg.STS3215_ACTUATOR
    check("actuator", (T.KP, T.KD, T.EFFORT_LIMIT, T.ARMATURE, T.FRICTIONLOSS, T.VISCOUS_DAMPING),
          (act.stiffness, act.damping, act.effort_limit, act.armature, act.frictionloss, act.viscous_damping))
    check("delay", (T.DELAY_MIN_LAG, T.DELAY_MAX_LAG), (act.delay_min_lag, act.delay_max_lag))
    check("soft joint limit factor", T.SOFT_JOINT_POS_LIMIT_FACTOR,
          robot_cfg.ARDUINO_QUAD_ARTICULATION.soft_joint_pos_limit_factor)
    check("home", (T.HOME_BASE_HEIGHT, T.HOME_HIP, T.HOME_KNEE),
          (robot_cfg.HOME_BASE_HEIGHT, robot_cfg.HOME_HIP, robot_cfg.HOME_KNEE))
    check("action scale", T.ACTION_SCALE, list(robot_cfg.ARDUINO_QUAD_ACTION_SCALE.values()))
    col = robot_cfg.FEET_ONLY_COLLISION
    check("toe contact", (T.TOE_FRICTION, *T.TOE_SOLIMP, T.TOE_CONDIM),
          (col.friction[0], *col.solimp, col.condim))
    if tuple(env_cfgs.FOOT_ORDER) != T.FOOT_ORDER:
        FAILS.append("FOOT_ORDER {} vs {}".format(T.FOOT_ORDER, env_cfgs.FOOT_ORDER))

    actor = walk.observations["actor"]
    if tuple(actor.terms) != T.ACTOR_TERMS:
        FAILS.append("actor terms {} vs {}".format(T.ACTOR_TERMS, tuple(actor.terms)))
    if tuple(walk.observations["critic"].terms) != T.CRITIC_TERMS:
        FAILS.append("critic terms {} vs {}".format(T.CRITIC_TERMS, tuple(walk.observations["critic"].terms)))
    check("actor history", T.ACTOR_HISTORY, actor.history_length)
    check("joint_pos noise", (-T.JOINT_POS_NOISE, T.JOINT_POS_NOISE),
          (actor.terms["joint_pos"].noise.n_min, actor.terms["joint_pos"].noise.n_max))
    check("joint_vel noise", (-T.JOINT_VEL_NOISE, T.JOINT_VEL_NOISE),
          (actor.terms["joint_vel"].noise.n_min, actor.terms["joint_vel"].noise.n_max))
    check("gait period", T.GAIT_PERIOD, actor.terms["phase"].params["period"])
    for group in ("actor", "critic"):
        check(group + " phase threshold", T.PHASE_ZERO_BELOW,
              walk.observations[group].terms["phase"].params.get("command_threshold", 0.1))
    if actor.terms["joint_pos"].params.get("biased", False):
        FAILS.append("mjlab joint_pos is biased=True; the CPU env observes the unbiased angle")

    tw = walk.commands["twist"]
    # mjlab's own UniformVelocityCommand -- NOT the unitree_rl_mjlab one of the
    # same name, which zeroes any sampled command with norm <= 0.1.
    if type(tw).__module__ != "mjlab.tasks.velocity.mdp.velocity_command":
        FAILS.append("command class {}.{}; the CPU env reproduces mjlab's".format(
            type(tw).__module__, type(tw).__name__))
    check("command extras", (tw.rel_world_envs, tw.rel_forward_envs, tw.init_velocity_prob), (0.0, 0.0, 0.0))
    check("command resampling", T.CMD_RESAMPLING_TIME, tw.resampling_time_range)
    check("rel_standing_envs", T.CMD_REL_STANDING_ENVS, tw.rel_standing_envs)
    check("lin_vel_y", (0.0, 0.0), tw.ranges.lin_vel_y)
    if tw.heading_command:
        FAILS.append("mjlab uses a heading command; the CPU env does not")
    stages = walk.curriculum["command_vel"].params["velocity_stages"]
    check("curriculum", [(s[0], *s[1], *s[2]) for s in T.CMD_CURRICULUM],
          [(s["step"], *s["lin_vel_x"], *s["ang_vel_z"]) for s in stages])

    theirs = {k: v.weight for k, v in walk.rewards.items()}
    if set(theirs) != set(T.REWARD_WEIGHTS):
        FAILS.append("reward terms: only in task.py {}, only in mjlab {}".format(
            sorted(set(T.REWARD_WEIGHTS) - set(theirs)), sorted(set(theirs) - set(T.REWARD_WEIGHTS))))
    for k in sorted(set(theirs) & set(T.REWARD_WEIGHTS)):
        check("weight " + k, T.REWARD_WEIGHTS[k], theirs[k])
    r = walk.rewards
    check("track_lin std", T.TRACK_LIN_STD, r["track_linear_velocity"].params["std"])
    check("track_ang std", T.TRACK_ANG_STD, r["track_angular_velocity"].params["std"])
    check("filtered std", T.TRACK_LIN_STD, r["track_lin_vel_filtered"].params["std"])
    check("filter time", (T.VEL_FILTER_TIME, T.VEL_FILTER_TIME),
          (r["track_lin_vel_filtered"].params["filter_time"], r["base_vel_oscillation"].params["filter_time"]))
    p = r["pose"].params
    check("pose thresholds", (T.POSE_WALKING_THRESHOLD, T.POSE_RUNNING_THRESHOLD),
          (p["walking_threshold"], p["running_threshold"]))
    for regime, ours in (("standing", T.STD_STANDING), ("walking", T.STD_WALKING), ("running", T.STD_RUNNING)):
        got = {("hip" if "hip" in k else "knee"): v for k, v in p["std_" + regime].items()}
        check("pose std " + regime, (ours["hip"], ours["knee"]), (got["hip"], got["knee"]))
    g = r["foot_gait"].params
    check("foot_gait", (T.GAIT_PERIOD, *T.GAIT_OFFSET, T.FOOT_GAIT_THRESHOLD, T.FOOT_GAIT_CMD_THRESHOLD),
          (g["period"], *g["offset"], g["threshold"], g["command_threshold"]))
    check("foot_clearance", (T.SWING_HEIGHT, T.FOOT_CLEARANCE_CMD_THRESHOLD),
          (r["foot_clearance"].params["target_height"], r["foot_clearance"].params["command_threshold"]))
    check("foot_slip thr", T.FOOT_SLIP_CMD_THRESHOLD, r["foot_slip"].params["command_threshold"])
    check("soft_landing thr", T.SOFT_LANDING_CMD_THRESHOLD, r["soft_landing"].params["command_threshold"])
    check("stand_still thr", T.STAND_STILL_CMD_THRESHOLD, r["stand_still"].params["command_threshold"])
    check("feet_air_time", (T.FEET_AIR_TIME_THRESHOLD, T.FEET_AIR_TIME_CMD_THRESHOLD),
          (r["feet_air_time"].params["threshold"], r["feet_air_time"].params["command_threshold"]))
    check("feet_swing_height", (T.SWING_HEIGHT, T.FEET_SWING_CMD_THRESHOLD),
          (r["feet_swing_height"].params["target_height"], r["feet_swing_height"].params["command_threshold"]))
    check("torque_excess", (T.TORQUE_EXCESS_LIMIT, T.VISCOUS_DAMPING),
          (r["torque_excess"].params["limit"], r["torque_excess"].params["back_emf_damping"]))
    check("speed_excess", T.SPEED_EXCESS_LIMIT, r["speed_excess"].params["limit"])
    check("base_height", T.HOME_BASE_HEIGHT, r["base_height"].params["target_height"])
    check("air_time_excess", T.FOOT_AIR_TIME_EXCESS_LIMIT, r["foot_air_time_excess"].params["limit"])
    if "stand_feet_down" in r:
        check("stand_feet_down thr", T.STAND_FEET_CMD_THRESHOLD, r["stand_feet_down"].params["command_threshold"])

    tm = walk.terminations
    if set(tm) != {"time_out", "fell_over", "low_base"}:
        FAILS.append("terminations {}".format(sorted(tm)))
    check("fell_over", T.FELL_OVER_ANGLE, tm["fell_over"].params["limit_angle"])
    check("low_base", T.MIN_BASE_HEIGHT, tm["low_base"].params["minimum_height"])

    ev = walk.events
    keys = ("x", "y", "z", "roll", "pitch", "yaw")
    rng6 = lambda d: [d.get(k, (0.0, 0.0)) for k in keys]  # noqa: E731
    for cfg, pose, vel, jp, jv, label in (
            (walk, T.RESET_POSE_RANGE, T.RESET_VELOCITY_RANGE, T.RESET_JOINT_POS_RANGE, T.RESET_JOINT_VEL_RANGE, "walk"),
            (robust, T.ROBUST_POSE_RANGE, T.ROBUST_VELOCITY_RANGE, T.ROBUST_JOINT_POS_RANGE,
             T.ROBUST_JOINT_VEL_RANGE, "robust")):
        e = cfg.events
        check(label + " reset pose", rng6(pose), rng6(e["reset_base"].params["pose_range"]))
        check(label + " reset vel", rng6(vel), rng6(e["reset_base"].params["velocity_range"]))
        check(label + " reset joints", (*jp, *jv),
              (*e["reset_robot_joints"].params["position_range"], *e["reset_robot_joints"].params["velocity_range"]))
    check("push interval", T.PUSH_INTERVAL_S, ev["push_robot"].interval_range_s)
    check("push velocity", rng6(T.PUSH_VELOCITY_RANGE), rng6(ev["push_robot"].params["velocity_range"]))
    check("foot friction", T.FOOT_FRICTION_RANGE, ev["foot_friction"].params["ranges"])
    if not ev["foot_friction"].params.get("shared_random") or ev["foot_friction"].params["operation"] != "abs":
        FAILS.append("foot friction is expected to be shared + abs")
    com = ev["base_com"].params["ranges"]
    check("base com", T.BASE_COM_RANGE, [com[i] for i in range(3)])
    check("encoder bias", T.ENCODER_BIAS_RANGE, ev["encoder_bias"].params["bias_range"])
    check("kp/kd", (*T.KP_SCALE_RANGE, *T.KD_SCALE_RANGE),
          (*ev["servo_gains"].params["kp_range"], *ev["servo_gains"].params["kd_range"]))
    check("effort", T.EFFORT_SCALE_RANGE, ev["servo_effort"].params["effort_limit_range"])
    check("armature", T.ARMATURE_SCALE_RANGE, ev["servo_armature"].params["ranges"])
    check("damping", T.DAMPING_SCALE_RANGE, ev["servo_damping"].params["ranges"])
    check("frictionloss", T.FRICTIONLOSS_RANGE, ev["servo_friction"].params["ranges"])
    check("payload", T.PAYLOAD_RANGE, ev["payload"].params["ranges"])
    ours = {"foot_friction", "base_com", "encoder_bias", "servo_gains", "servo_effort", "servo_armature",
            "servo_damping", "servo_friction", "payload", "reset_base", "reset_robot_joints", "push_robot"}
    if set(ev) != ours:
        FAILS.append("events: only in task.py {}, only in mjlab {}".format(
            sorted(ours - set(ev)), sorted(set(ev) - ours)))

    ppo = rl_cfg.arduino_quad_ppo_runner_cfg()
    mine = T.ppo_runner_cfg()
    check("actor dims", mine["actor"]["hidden_dims"], ppo.actor.hidden_dims)
    check("critic dims", mine["critic"]["hidden_dims"], ppo.critic.hidden_dims)
    for k, v in mine["algorithm"].items():
        if k != "class_name":
            if getattr(ppo.algorithm, k) != v:
                FAILS.append("ppo {}: {} vs {}".format(k, v, getattr(ppo.algorithm, k)))
    check("steps per env", mine["num_steps_per_env"], ppo.num_steps_per_env)
    if mine["actor"]["distribution_cfg"] != ppo.actor.distribution_cfg:
        FAILS.append("actor distribution {} vs {}".format(mine["actor"]["distribution_cfg"],
                                                          ppo.actor.distribution_cfg))


def check_dynamics(npz: str | None, steps: int) -> None:
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.tasks.registry import load_env_cfg
    import evaluate
    from quad_env import QuadEnv

    n, cmd = 4, (0.09, 0.0, 0.2)
    cfg = load_env_cfg("ArduinoQuad-Walk", play=True)
    cfg.scene.num_envs = n
    for k in ("foot_friction", "base_com", "encoder_bias", "servo_gains", "servo_effort", "servo_armature",
              "servo_damping", "servo_friction", "payload", "push_robot"):
        cfg.events.pop(k, None)
    ref = ManagerBasedRlEnv(cfg=cfg, device="cpu")
    ref.reset()
    twist = ref.command_manager.get_term("twist")

    def pin_command() -> None:
        twist.vel_command_b[:] = torch.tensor(cmd)
        twist.time_left[:] = 1e9
        twist.is_standing_env[:] = False

    pin_command()
    ref.observation_manager.reset()
    obs_ref = ref.observation_manager.compute(update_history=True)

    env = QuadEnv(n, play=True, domain_rand=False)
    env.fixed_command = np.array(cmd)
    env.command[:] = env.fixed_command
    nq = env.model.nq
    env.state[:, 1:1 + nq] = ref.sim.data.qpos.cpu().numpy()
    env.state[:, 1 + nq:1 + nq + env.model.nv] = ref.sim.data.qvel.cpu().numpy()
    env.state[:, 1:3] -= ref.scene.env_origins[:, :2].cpu().numpy()   # per-env grid offset
    env.episode_length[:] = ref.episode_length_buf.cpu().numpy()
    env._need_backfill[:] = True
    env._observe(np.arange(n))
    policy = evaluate.npz_policy(npz) if npz else None
    names = ref.reward_manager.active_terms
    worst: dict[str, float] = {}
    where: dict[str, str] = {}
    step = [0]

    # An env drops out of the comparison once either side resets it: the reset
    # pose is drawn from each env's own random stream.
    alive = np.ones(n, dtype=bool)

    def note(key: str, a: np.ndarray, b: np.ndarray) -> None:
        if not alive.any():
            return
        d = np.abs(np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64))[alive]
        if float(d.max()) > worst.get(key, -1.0):
            worst[key] = float(d.max())
            where[key] = "step {} env {}".format(
                step[0], int(np.flatnonzero(alive)[np.unravel_index(d.argmax(), d.shape)[0]]))

    note("actor obs (t=0)", env.obs["actor"].numpy(), obs_ref["actor"].cpu().numpy())
    for t in range(steps):
        if policy is not None:
            a = policy({"actor": obs_ref["actor"].cpu()})
        else:
            a = torch.from_numpy(0.6 * np.sin(2 * math.pi * (t * T.STEP_DT / T.GAIT_PERIOD
                                                             + np.arange(8)[None] / 8.0)).repeat(n, 0)).float()
        step[0] = t
        obs_ref, _, term_ref, out_ref, _ = ref.step(a)
        pin_command()
        _, _, done, _ = env.step(a)
        alive &= ~(done.numpy().astype(bool) | term_ref.cpu().numpy() | out_ref.cpu().numpy())
        if t < 25:   # chaos grows the gap afterwards; the first 0.5 s is the like-for-like part
            note("joint pos", env.qpos[:, env.qadr], ref.scene["robot"].data.joint_pos.cpu().numpy())
            note("base z", env.qpos[:, 2], ref.scene["robot"].data.root_link_pos_w[:, 2].cpu().numpy())
            note("actor obs", env.obs["actor"].numpy(), obs_ref["actor"].cpu().numpy())
            sr = ref.reward_manager._step_reward.cpu().numpy()
            for i, name in enumerate(names):
                note("reward " + name, env.step_reward[name], sr[:, i])
    print("\nlargest |plain MuJoCo - mjlab| over the first 25 steps ({} of {} envs never reset):"
          .format(int(alive.sum()), n))
    for k, v in worst.items():
        print("  {:34s} {:.3e}  ({})".format(k, v, where[k]))
    if worst["joint pos"] > 5e-3 or worst["actor obs (t=0)"] > 1e-4:
        FAILS.append("dynamics diverge from mjlab (joint pos {:.2e})".format(worst["joint pos"]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", default=None, help="exported policy to drive both envs (default: a fixed pattern)")
    ap.add_argument("--steps", type=int, default=50)
    ap.add_argument("--config-only", action="store_true")
    args = ap.parse_args()
    check_config()
    print("configuration: {}".format("OK" if not FAILS else "{} mismatches".format(len(FAILS))))
    if not args.config_only:
        check_dynamics(args.npz, args.steps)
    for f in FAILS:
        print("MISMATCH " + f)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
